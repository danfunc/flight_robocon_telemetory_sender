// ===========================================================================
//  core1_io — core1 ベアメタル センサ I/O ループ (設計書 §5, step2: NDOF のまま)
// ===========================================================================
//  I2C バスと BNO055/BME280 を core1 が専有し、生の整数サンプルをコア間リング
//  (include/core_ring.hpp) へ押す。core0 の Shizuku スケジューラは I2C の停滞に
//  一切影響されなくなる。
//
//  【この TU の絶対制約】
//  ・Shizuku API (svc / obj_api / yield) と printf を使わない。異常はカウンタで
//    数えて 1Hz の STATUS レコードに載せる。
//  ・float を一切使わない (整数のみ)。生 int16 LSB のままリングへ push し、
//    /16, /100 の float 換算は core0 側で行う。これにより core1 の CPACR/FPU
//    問題 (per-core 初期化の有無) を丸ごと回避する。ビルド後に objdump で
//    v*.f32 が 0 本であることを確認すること。
//  ・待ちは busy_wait/グリッド比較のみ (yield は存在しない)。
//
//  現行 BNO055_DRIVER.cpp から移植した整数ロジック:
//    - 0xFFFF 破損 ch の据え置き (held_raw) + 1 回読み直し
//    - euler の ±1 LSB デッドバンド (int16)
//    - fail_streak → i2c_bus::recover() → センサ再 init のエスカレーション
//  BME280 は整数補償 (compensate_T/P) までを core1 で行い Pa / 0.01℃ で push。
// ===========================================================================
#include <core_ring.hpp>
#include <cstdint>
#include <cstring>
#include <hardware/timer.h> // timer_hw (now_us の直接レジスタ読み)
#include <i2c_bus.hpp>
#include <kernel.hpp> // cpu_busy_us (svc を使わず直接書く使用率計測)
#include <pico/multicore.h>
#include <pico/platform.h> // __not_in_flash_func
#include <pico/time.h>

namespace shizu {
namespace {

using namespace core_ring;

// ---- XIP (フラッシュ) 競合対策 ----------------------------------------------
// core1 はホットループを全速で回すため、フラッシュ実行のままだと XIP キャッシュ/
// バスを core0 (BTstack 送信パス) と取り合い、BLE スループットを ~7% 削る実測。
// ホットパスは __not_in_flash_func で SRAM (.time_critical) に置き、周期タイマ
// 読みも now_us() (フラッシュ常駐) ではなくレジスタ直読みにする。
// 初期化系 (bno/bme_init_sensor 等) はコールドパスなのでフラッシュのまま。
static inline uint64_t now_us() {
  uint32_t hi = timer_hw->timerawh;
  uint32_t lo;
  while (true) {
    lo = timer_hw->timerawl;
    uint32_t hi2 = timer_hw->timerawh;
    if (hi2 == hi)
      break;
    hi = hi2; // lo 読みの間に桁上がり → 取り直し
  }
  return ((uint64_t)hi << 32) | lo;
}

// ===========================================================================
//  BNO055 (BNO055_DRIVER.cpp から移植した整数部)
// ===========================================================================
constexpr uint8_t BNO055_ADDR = 0x28;
constexpr uint8_t REG_CHIP_ID = 0x00;
constexpr uint8_t REG_GRV_DATA_X_LSB = 0x2E;
constexpr uint8_t REG_LIA_DATA_X_LSB = 0x28;
constexpr uint8_t REG_EUL_HEAD_LSB = 0x1A;
constexpr uint8_t REG_CALIB_STAT = 0x35;
constexpr uint8_t REG_OPR_MODE = 0x3D;
constexpr uint8_t REG_SYS_TRIGGER = 0x3F;
constexpr uint8_t REG_UNIT_SEL = 0x3B;
constexpr uint8_t OPMODE_CONFIG = 0x00;
constexpr uint8_t OPMODE_NDOF = 0x0C;
constexpr uint8_t USE_MODE = OPMODE_NDOF;
constexpr uint8_t REG_CALIB_OFFSET_START = 0x55;
constexpr int CALIB_PROFILE_LEN = 22;

constexpr uint8_t REG_BLOCK_START = REG_EUL_HEAD_LSB; // 0x1A
constexpr int MOTION_BLOCK_LEN =
    (REG_GRV_DATA_X_LSB + 6) - REG_EUL_HEAD_LSB; // 26

constexpr uint8_t BME280_ADDR = 0x76;
constexpr uint8_t BME_REG_ID = 0xD0;
constexpr uint8_t BME_REG_RESET = 0xE0;
constexpr uint8_t BME_REG_CTRL_HUM = 0xF2;
constexpr uint8_t BME_REG_CTRL_MEAS = 0xF4;
constexpr uint8_t BME_REG_CONFIG = 0xF5;
constexpr uint8_t BME_REG_PRESS_MSB = 0xF7;

// ---- 実行時状態 (コマンドリング経由で core0 から変更される) -----------------
bool g_split_read = true;
bool g_reject_ffff = true;
bool g_paused_bno = false;
bool g_paused_bme = false;
// 連続 I2C 失敗が何回目で 20Hz バックオフに入るか。1=毎回退避 (旧挙動)、大きいほど
// 一過性の単発失敗を 100Hz グリッドのまま突き抜ける。K コマンドで可変 (既定 5)。
// 一過性の読み衝突 (NDOF フュージョン更新との衝突。0xFFFF と同根) に 50ms 罰を
// 与えて実効レートが 50〜70Hz に落ちていたのを、この閾値で吸収する。
int g_fail_backoff_n = 5;

// ---- 健全性カウンタ (STATUS レコードで core0 へ) -----------------------------
bool g_bno_ok = false;
bool g_bme_ok = false;
uint16_t g_i2c_fail_count = 0; // 累積 (ラップ許容)
uint8_t g_recover_count = 0;
uint8_t g_reinit_count = 0;
uint8_t g_last_calib = 0;

// ---- 0xFFFF 破損率 A/B 測定 (ブロック vs split-read) の 1s 窓カウンタ ----------
// X0/X1 でモードを切替え、CH_DIAG で「現モード + 直近 1s の 読み総数/破損数」を
// core0 へ送る。core0 が破損率を printf する。AMG 化せず 0xFFFF を潰せるか判定用。
uint16_t g_motion_reads = 0; // 成功したバースト読み総数
uint16_t g_ffff_reads = 0;   // うち attempt0 に 1ch でも 0xFFFF があった数

// ---- BNO055 整数ロジック -----------------------------------------------------
int16_t held_raw[9] = {0};
int16_t held_eul[3] = {0};
bool eul_primed = false;

int16_t __not_in_flash_func(euler_deadband)(int16_t raw, int16_t &held) {
  int d = (int)raw - held;
  if (!eul_primed || d > 1 || d < -1)
    held = raw;
  return held;
}

void bno_set_mode(uint8_t mode) {
  i2c_bus::write_reg(BNO055_ADDR, REG_OPR_MODE, OPMODE_CONFIG);
  busy_wait_us(30000);
  if (mode != OPMODE_CONFIG) {
    i2c_bus::write_reg(BNO055_ADDR, REG_OPR_MODE, mode);
    busy_wait_us(30000);
  }
}

bool bno_init_sensor() {
  uint8_t id;
  if (i2c_bus::read_regs(BNO055_ADDR, REG_CHIP_ID, &id, 1) < 0)
    return false;
  if (id != 0xA0)
    return false;
  // POR リセット → 起動待ち (~650ms)。core1 専有なので busy_wait でよい。
  i2c_bus::write_reg(BNO055_ADDR, REG_SYS_TRIGGER, 0x20);
  busy_wait_us(700000);
  i2c_bus::write_reg(BNO055_ADDR, REG_UNIT_SEL, 0x00); // m/s^2, deg
  busy_wait_us(10000);
  bno_set_mode(USE_MODE);
  return true;
}

int __not_in_flash_func(read_u16)(uint8_t reg, int16_t *out) {
  uint8_t b[2];
  int r = i2c_bus::read_regs(BNO055_ADDR, reg, b, 2);
  if (r < 0)
    return r;
  *out = (int16_t)((b[1] << 8) | b[0]);
  return 0;
}

// euler + linaccel + gravity の生 int16 9 個。g_split_read で 26B ブロック /
// 2B×9 個別読みを切り替え (BNO055_DRIVER.cpp の read_raw9 と同一)。
bool __not_in_flash_func(read_raw9)(int16_t r[9]) {
  if (g_split_read) {
    static const uint8_t regs[9] = {
        REG_EUL_HEAD_LSB,   REG_EUL_HEAD_LSB + 2,   REG_EUL_HEAD_LSB + 4,
        REG_LIA_DATA_X_LSB, REG_LIA_DATA_X_LSB + 2, REG_LIA_DATA_X_LSB + 4,
        REG_GRV_DATA_X_LSB, REG_GRV_DATA_X_LSB + 2, REG_GRV_DATA_X_LSB + 4};
    for (int i = 0; i < 9; ++i)
      if (read_u16(regs[i], &r[i]) < 0)
        return false;
    return true;
  }
  uint8_t buf[MOTION_BLOCK_LEN];
  if (i2c_bus::read_regs(BNO055_ADDR, REG_BLOCK_START, buf, MOTION_BLOCK_LEN) <
      0)
    return false;
  const uint8_t *eul = &buf[REG_EUL_HEAD_LSB - REG_BLOCK_START];
  const uint8_t *lia = &buf[REG_LIA_DATA_X_LSB - REG_BLOCK_START];
  const uint8_t *grv = &buf[REG_GRV_DATA_X_LSB - REG_BLOCK_START];
  r[0] = (int16_t)((eul[1] << 8) | eul[0]);
  r[1] = (int16_t)((eul[3] << 8) | eul[2]);
  r[2] = (int16_t)((eul[5] << 8) | eul[4]);
  r[3] = (int16_t)((lia[1] << 8) | lia[0]);
  r[4] = (int16_t)((lia[3] << 8) | lia[2]);
  r[5] = (int16_t)((lia[5] << 8) | lia[4]);
  r[6] = (int16_t)((grv[1] << 8) | grv[0]);
  r[7] = (int16_t)((grv[3] << 8) | grv[2]);
  r[8] = (int16_t)((grv[5] << 8) | grv[4]);
  return true;
}

// 0xFFFF 破損の据え置き + euler デッドバンドを適用した生 int16 9 個を返す。
// (BNO055_DRIVER.cpp read_motion の整数部そのまま。calib 読みは 1Hz の
//  STATUS 側へ移した)
bool __not_in_flash_func(read_motion9)(int16_t r[9]) {
  for (int attempt = 0; attempt < 2; ++attempt) {
    if (!read_raw9(r))
      return false;
    // 破損計数 (A/B 測定用): attempt0 のバーストに 0xFFFF が 1ch でもあるか。
    // g_reject_ffff の設定に依らず常に測る (破損率の素の値が欲しい)。
    bool any_ffff = false;
    for (int k = 0; k < 9; ++k)
      if (r[k] == -1) {
        any_ffff = true;
        break;
      }
    if (attempt == 0) {
      ++g_motion_reads;
      if (any_ffff)
        ++g_ffff_reads;
    }
    if (!g_reject_ffff)
      break;
    if (!any_ffff)
      break;
  }
  if (g_reject_ffff)
    for (int k = 0; k < 9; ++k) {
      if (r[k] == -1)
        r[k] = held_raw[k];
      else
        held_raw[k] = r[k];
    }
  r[0] = euler_deadband(r[0], held_eul[0]);
  r[1] = euler_deadband(r[1], held_eul[1]);
  r[2] = euler_deadband(r[2], held_eul[2]);
  eul_primed = true;
  return true;
}

// ===========================================================================
//  BME280 (BME280_DRIVER.cpp から移植した整数補償部)
// ===========================================================================
uint16_t dig_T1;
int16_t dig_T2, dig_T3;
uint16_t dig_P1;
int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
int32_t t_fine;

bool bme_init_sensor() {
  uint8_t id;
  if (i2c_bus::read_regs(BME280_ADDR, BME_REG_ID, &id, 1) < 0)
    return false;
  if (id != 0x60)
    return false;
  i2c_bus::write_reg(BME280_ADDR, BME_REG_RESET, 0xB6);
  busy_wait_us(10000);
  uint8_t b[26];
  if (i2c_bus::read_regs(BME280_ADDR, 0x88, b, 26) < 0)
    return false;
  dig_T1 = (uint16_t)(b[0] | (b[1] << 8));
  dig_T2 = (int16_t)(b[2] | (b[3] << 8));
  dig_T3 = (int16_t)(b[4] | (b[5] << 8));
  dig_P1 = (uint16_t)(b[6] | (b[7] << 8));
  dig_P2 = (int16_t)(b[8] | (b[9] << 8));
  dig_P3 = (int16_t)(b[10] | (b[11] << 8));
  dig_P4 = (int16_t)(b[12] | (b[13] << 8));
  dig_P5 = (int16_t)(b[14] | (b[15] << 8));
  dig_P6 = (int16_t)(b[16] | (b[17] << 8));
  dig_P7 = (int16_t)(b[18] | (b[19] << 8));
  dig_P8 = (int16_t)(b[20] | (b[21] << 8));
  dig_P9 = (int16_t)(b[22] | (b[23] << 8));
  // 温度 oversampling x2, 気圧 x16, normal mode (現行と同一設定)。
  i2c_bus::write_reg(BME280_ADDR, BME_REG_CTRL_HUM, 0x01);
  i2c_bus::write_reg(BME280_ADDR, BME_REG_CTRL_MEAS, (2 << 5) | (5 << 2) | 3);
  i2c_bus::write_reg(BME280_ADDR, BME_REG_CONFIG, (0 << 5) | (4 << 2));
  busy_wait_us(100000);
  return true;
}

// データシートの固定小数点補償式 (整数のみ)。戻り値 0.01℃。
int32_t __not_in_flash_func(compensate_T)(int32_t adc_T) {
  int32_t var1 =
      ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
  int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) *
                    ((adc_T >> 4) - ((int32_t)dig_T1))) >>
                   12) *
                  ((int32_t)dig_T3)) >>
                 14;
  t_fine = var1 + var2;
  return (t_fine * 5 + 128) >> 8;
}

// 戻り値 Pa (整数)。
uint32_t __not_in_flash_func(compensate_P)(int32_t adc_P) {
  int64_t var1, var2, p;
  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)dig_P6;
  var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
  var2 = var2 + (((int64_t)dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) +
         ((var1 * (int64_t)dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;
  if (var1 == 0)
    return 0;
  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
  return (uint32_t)p / 256;
}

// 生読み → 整数補償。press_pa [Pa], temp_cc [0.01℃]。
bool __not_in_flash_func(bme_read)(uint32_t *press_pa, int32_t *temp_cc) {
  uint8_t buf[6];
  if (i2c_bus::read_regs(BME280_ADDR, BME_REG_PRESS_MSB, buf, 6) < 0)
    return false;
  int32_t adc_P = (int32_t)((buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4));
  int32_t adc_T = (int32_t)((buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4));
  *temp_cc = compensate_T(adc_T);
  *press_pa = compensate_P(adc_P);
  return true;
}

// ---- 地上気圧較正 (整数 Pa の逐次平均。起動時 40 / rezero 20) ---------------
uint32_t ground_want = 0;     // 残り目標サンプル数 (0=較正中でない)
uint32_t ground_got = 0;
uint32_t ground_attempts = 0; // 全滅検出用
uint64_t ground_sum_pa = 0;
int64_t ground_sum_cc = 0;

void ground_start(uint32_t samples) {
  ground_want = samples;
  ground_got = 0;
  ground_attempts = 0;
  ground_sum_pa = 0;
  ground_sum_cc = 0;
}

// ---- リング push ヘルパ ------------------------------------------------------
void __not_in_flash_func(push_rec)(uint8_t ch, uint32_t t_us, const void *pl, size_t n,
              uint8_t flags = 0) {
  record_t r;
  r.ch_id = ch;
  r.flags = flags;
  r.t_us = t_us;
  memset(r.payload, 0, sizeof(r.payload));
  memcpy(r.payload, pl, n);
  g_data_stream.hdl().push(
      r); // stream API のライブラリ push (SVC 無し、SRAM 内)
}

void __not_in_flash_func(push_baro_like)(uint8_t ch, uint32_t t_us, uint32_t press_pa,
                    int32_t temp_cc) {
  uint8_t pl[6];
  pl[0] = (uint8_t)(press_pa & 0xFF);
  pl[1] = (uint8_t)((press_pa >> 8) & 0xFF);
  pl[2] = (uint8_t)((press_pa >> 16) & 0xFF);
  pl[3] = (uint8_t)((press_pa >> 24) & 0xFF);
  int16_t t16 = (int16_t)temp_cc; // 0.01℃、±327℃ まで表現可なので直キャスト
  pl[4] = (uint8_t)(t16 & 0xFF);
  pl[5] = (uint8_t)((t16 >> 8) & 0xFF);
  push_rec(ch, t_us, pl, 6);
}

// ---- コマンドリング処理 ------------------------------------------------------
void __not_in_flash_func(handle_cmds)() {
  cmd_rec_t c;
  uint32_t lost = 0;
  while (g_cmd_ring.pop(&c, &lost)) {
    switch (c.op) {
    case CMD_SET_PAUSED_BNO:
      g_paused_bno = (c.arg != 0);
      break;
    case CMD_SET_PAUSED_BME:
      g_paused_bme = (c.arg != 0);
      break;
    case CMD_SET_READ_MODE:
      g_split_read = (c.arg != 0);
      break;
    case CMD_SET_FFFF_REJECT:
      g_reject_ffff = (c.arg != 0);
      break;
    case CMD_REZERO:
      if (g_bme_ok)
        ground_start(20);
      break;
    case CMD_SET_FAIL_BACKOFF:
      g_fail_backoff_n = (int)c.arg; // 0/1=毎回退避(旧), N=N回連続で退避
      break;
    default:
      break;
    }
  }
}

// ---- 較正プロファイル サイドバンド処理 ---------------------------------------
// モード切替 (busy_wait 30ms×2) を伴うので処理後は BNO グリッドを取り直すこと。
bool __not_in_flash_func(handle_calib_sideband)() {
  uint32_t req = g_calib_xfer.req_seq;
  if (req == g_calib_xfer.ack_seq)
    return false;
  __dmb(); // req_seq 観測 → op/data 読みの順序を保証
  uint8_t op = g_calib_xfer.op;
  bool ok = false;
  if (g_bno_ok) {
    bno_set_mode(OPMODE_CONFIG);
    if (op == 1) { // save: 現オフセットを吸い出す
      uint8_t buf[CALIB_PROFILE_LEN];
      ok = (i2c_bus::read_regs(BNO055_ADDR, REG_CALIB_OFFSET_START, buf,
                               CALIB_PROFILE_LEN) >= 0);
      if (ok)
        memcpy(const_cast<uint8_t *>(g_calib_xfer.data), buf,
               CALIB_PROFILE_LEN);
    } else if (op == 2) { // load: オフセットを書き戻す
      ok = true;
      for (int i = 0; i < CALIB_PROFILE_LEN; ++i)
        if (i2c_bus::write_reg(BNO055_ADDR, REG_CALIB_OFFSET_START + i,
                               g_calib_xfer.data[i]) < 0)
          ok = false;
    }
    bno_set_mode(USE_MODE);
  }
  g_calib_xfer.ok = ok ? 1 : 0;
  __dmb(); // data/ok の書き込みを ack 公開より先に完了させる
  g_calib_xfer.ack_seq = req;
  return true;
}

} // namespace

// ===========================================================================
//  センサ I/O ループ本体 — Shizuku の SENSOR_IO スレッド entry (core1
//  ピン留め、 POC=1) 兼ベアメタル core1 entry (POC=0)。yield は呼ばず core1
//  を専有する (協調スケジューラ上でも「決して yield しないスレッド」=
//  実質コア占有)。中身は 従来の core1_main と完全に同一 (I2C / 整数 / busy_wait
//  / stream push)。
// ===========================================================================
[[noreturn]] void __not_in_flash_func(sensor_io_main)() {
  i2c_bus::init();
  g_bno_ok = bno_init_sensor();
  g_bme_ok = bme_init_sensor();
  if (g_bme_ok)
    ground_start(40); // 起動時の地上気圧較正 (現行 init と同じ 40 サンプル平均)

  uint64_t now = now_us();
  uint64_t next_bno = now;
  uint64_t next_bme = now;
  uint64_t next_status = now + 1000000;
  int fail_streak = 0;     // BNO055 連続失敗数
  int bme_fail_streak = 0; // BME280 連続失敗数 (不在時の 2Hz 退避用)
  bool prev_paused_bno = false, prev_paused_bme = false;

  while (true) {
    // 使用率計測: このイテレーションで実 I/O 作業 (センサ読み/較正/status) をしたか。
    // したイテレーションの経過を core1 の busy として計上する (作業無し = grid 締切
    // 待ちの高速ポーリング = idle)。この TU は svc 禁止なので cpu_busy_us へ直接書く。
    const uint64_t t_iter = now_us();
    bool did_work = false;
    handle_cmds();
    if (handle_calib_sideband()) {
      next_bno = now_us(); // モード切替 (~100ms) 後はグリッド取り直し
      did_work = true;
    }

    // 再開時はグリッドを現在へ取り直す (溜まった締切の連射を防ぐ)。
    if (prev_paused_bno && !g_paused_bno)
      next_bno = now_us();
    if (prev_paused_bme && !g_paused_bme)
      next_bme = now_us();
    prev_paused_bno = g_paused_bno;
    prev_paused_bme = g_paused_bme;

    // ---- BNO055: 100Hz 絶対グリッド (NDOF フュージョン出力の上限) ----
    now = now_us();
    if (g_bno_ok && !g_paused_bno && (int64_t)(now - next_bno) >= 0) {
      did_work = true;
      int16_t r9[9];
      uint32_t t = (uint32_t)now;
      uint8_t flags = ((int64_t)(now - next_bno) > 2000) ? FLAG_OFF_GRID : 0;
      if (read_motion9(r9)) {
        fail_streak = 0;
        push_rec(CH_EUL, t, &r9[0], 6, flags);
        push_rec(CH_LIA, t, &r9[3], 6, flags);
        push_rec(CH_GRV, t, &r9[6], 6, flags);
        next_bno += 10000;
        if ((int64_t)(now_us() - next_bno) > 0) // 大幅遅れは再同期
          next_bno = now_us();
      } else {
        // 現行のエスカレーションを core1 内で完結 (printf の代わりにカウンタ)。
        ++g_i2c_fail_count;
        ++fail_streak;
        if (fail_streak >= 20) {
          // バス救出でも戻らない → センサごと再 init (POR ~700ms busy_wait、
          // core0 には影響しない)。それも失敗 = センサ不在が続いている:
          // タイムアウト連打が I2C/XIP バスを汚すだけなので 2Hz へ退避する。
          // 復帰チェックはこの 2Hz の reinit 試行そのもの (成功したら復帰)。
          if (bno_init_sensor()) {
            ++g_reinit_count; // 実際に再初期化できた回数を数える
            fail_streak = 0;
            next_bno = now_us();
          } else {
            fail_streak = 20;          // 次の失敗でも即この分岐 (reinit 試行) に入る
            next_bno = now_us() + 500000; // 不在中は 2Hz
          }
        } else {
          if (fail_streak % 5 == 0) {
            i2c_bus::recover();
            ++g_recover_count;
          }
          if (fail_streak >= g_fail_backoff_n) {
            next_bno = now_us() + 50000; // 連続失敗が閾値到達 → 20Hz 退避
          } else {
            // 一過性の単発失敗: 50ms 罰を与えず 100Hz グリッドを維持して突き抜ける
            // (単発の読み衝突は次グリッドで直ることが多い)。
            next_bno += 10000;
            if ((int64_t)(now_us() - next_bno) > 0)
              next_bno = now_us();
          }
        }
      }
    }

    // ---- BME280: ~21Hz (気圧 x16 oversampling の変換時間が律速) ----
    now = now_us();
    if (g_bme_ok && !g_paused_bme && (int64_t)(now - next_bme) >= 0) {
      did_work = true;
      uint32_t pa;
      int32_t cc;
      if (bme_read(&pa, &cc)) {
        bme_fail_streak = 0;
        push_baro_like(CH_BARO, (uint32_t)now, pa, cc);
        if (ground_want > 0) {
          ground_sum_pa += pa;
          ground_sum_cc += cc;
          if (++ground_got >= ground_want) {
            push_baro_like(CH_GROUND, (uint32_t)now,
                           (uint32_t)(ground_sum_pa / ground_got),
                           (int32_t)(ground_sum_cc / (int64_t)ground_got));
            ground_want = 0;
          }
        }
        next_bme += 47000;
        if ((int64_t)(now_us() - next_bme) > 0)
          next_bme = now_us();
      } else {
        ++g_i2c_fail_count;
        ++bme_fail_streak;
        // 較正中に読みが全滅し続けたら既定値で完了させる (現行の 1013.25 相当)。
        if (ground_want > 0 && ++ground_attempts >= ground_want * 3 &&
            ground_got == 0) {
          push_baro_like(CH_GROUND, (uint32_t)now, 101325, 1500);
          ground_want = 0;
        }
        // BNO と同じ退避方針: 20 連続失敗で再 init を試み、それも失敗 =
        // センサ不在 → 2Hz へ退避してタイムアウト連打でバスを汚さない。
        if (bme_fail_streak >= 20) {
          if (bme_init_sensor()) {
            bme_fail_streak = 0;
            next_bme = now_us(); // 復帰: 21Hz グリッドを取り直す
          } else {
            bme_fail_streak = 20; // 2Hz の reinit 試行を復帰チェックとして継続
            next_bme = now_us() + 500000;
          }
        } else {
          next_bme += 47000; // 一時的な失敗は 21Hz グリッド維持
          if ((int64_t)(now_us() - next_bme) > 0)
            next_bme = now_us();
        }
      }
    }

    // ---- STATUS: 1Hz (calib byte はここでだけ読む: 1B 独立トランザクション) ----
    now = now_us();
    if ((int64_t)(now - next_status) >= 0) {
      did_work = true;
      // 失敗が続いている間は calib 読みも省く (無駄なタイムアウトを足さない)。
      if (g_bno_ok && !g_paused_bno && fail_streak < 5) {
        uint8_t c;
        if (i2c_bus::read_regs(BNO055_ADDR, REG_CALIB_STAT, &c, 1) >= 0)
          g_last_calib = c;
        else
          ++g_i2c_fail_count;
      }
      uint8_t pl[6];
      pl[0] = g_last_calib;
      pl[1] = (uint8_t)((g_bno_ok ? HEALTH_BNO_OK : 0) |
                        (g_bme_ok ? HEALTH_BME_OK : 0) |
                        (g_paused_bno ? HEALTH_BNO_PAUSED : 0) |
                        (g_paused_bme ? HEALTH_BME_PAUSED : 0));
      pl[2] = (uint8_t)(g_i2c_fail_count & 0xFF);
      pl[3] = (uint8_t)(g_i2c_fail_count >> 8);
      pl[4] = g_recover_count;
      pl[5] = g_reinit_count;
      push_rec(CH_STATUS, (uint32_t)now, pl, 6);
      // 0xFFFF 破損率 A/B: 現モード + 直近 1s の 読み総数/破損数 を DIAG で送る。
      uint8_t dpl[6];
      dpl[0] = g_split_read ? 1 : 0;
      dpl[1] = g_reject_ffff ? 1 : 0;
      dpl[2] = (uint8_t)(g_motion_reads & 0xFF);
      dpl[3] = (uint8_t)(g_motion_reads >> 8);
      dpl[4] = (uint8_t)(g_ffff_reads & 0xFF);
      dpl[5] = (uint8_t)(g_ffff_reads >> 8);
      push_rec(CH_DIAG, (uint32_t)now, dpl, 6);
      g_motion_reads = 0;
      g_ffff_reads = 0;
      next_status += 1000000;
      if ((int64_t)(now - next_status) > 0)
        next_status = now + 1000000;
    }
    // 締切ポーリングのタイトループ (core1 は I/O 専有なので 100% 回してよい。この TU の
    // 絶対制約により sleep/yield=svc は使わない — 待ちは busy-poll のまま)。
    // 使用率: 作業したイテレーションの経過を busy に足す (grid 締切待ちの高速空回りは
    // 足さない)。scheduler が回らない core1 でも svc 無しで使用率を得る。
    if (did_work)
      cpu_busy_us[1] += now_us() - t_iter;
  }
}

void core1_io_launch() { multicore_launch_core1(sensor_io_main); }

} // namespace shizu
