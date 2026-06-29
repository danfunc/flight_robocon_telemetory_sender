// ===========================================================================
//  BNO055_DRIVER — 9 軸 IMU を Shizuku オブジェクト化したドライバ
// ===========================================================================
//  元実装 test_firmware/altitude_fusion_wifi.c の BNO055 部分を移植。NDOF モード
//  でセンサ内蔵フュージョンを使い、重力ベクトル・線形加速度・オイラー角・較正状態を
//  周期サンプリングしてキャッシュ、read_latest で他オブジェクトへ渡す。
// ===========================================================================
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <export_method.hpp>
#include <i2c_bus.hpp>
#include <obj_api.hpp>
#include <object_headers/BNO055_DRIVER.hpp>
#include <pico/time.h>

namespace shizu {

// ---- レジスタ定義 ----------------------------------------------------------
static constexpr uint8_t BNO055_ADDR = 0x28;
static constexpr uint8_t REG_CHIP_ID = 0x00;
static constexpr uint8_t REG_GRV_DATA_X_LSB = 0x2E; // gravity vector
static constexpr uint8_t REG_LIA_DATA_X_LSB = 0x28; // linear accel
static constexpr uint8_t REG_EUL_HEAD_LSB = 0x1A;   // euler heading/roll/pitch
static constexpr uint8_t REG_CALIB_STAT = 0x35;
static constexpr uint8_t REG_OPR_MODE = 0x3D;
static constexpr uint8_t REG_SYS_TRIGGER = 0x3F;
static constexpr uint8_t REG_UNIT_SEL = 0x3B;

static constexpr uint8_t OPMODE_CONFIG = 0x00;
static constexpr uint8_t OPMODE_NDOF = 0x0C;
static constexpr uint8_t USE_MODE = OPMODE_NDOF;

// 較正オフセットプロファイル領域 (ACC/MAG/GYR offset + radius)。CONFIG モードでのみ
// 読み書きできる。0x55..0x6A の 22 バイト。
static constexpr uint8_t REG_CALIB_OFFSET_START = 0x55;
static constexpr int CALIB_PROFILE_LEN = BNO055_CALIB_PROFILE_LEN; // 22

// ---- 最新サンプル ----------------------------------------------------------
static bno055_sample_t latest = {};

// ---- 実行時状態 (他オブジェクトのメソッド呼び出しで変更される) --------------
// 読みモード: false=26B ブロック読み(既定) / true=16bit 値ごとの 2B 個別読み。
// バースト破損(heading 以外が 0xFFFF 化)の切り分け/回避用。協調スケジューラ単一
// コアなので volatile な単純フラグで十分(トランザクション中は yield しない)。
static volatile bool g_split_read = false;
// 0xFFFF 破損バースト(roll&pitch 同時 0xFFFF)を検出してサンプル破棄するか。既定 on。
// off にすると生の破損値もそのまま公開する(破損率の実測・比較用)。
static volatile bool g_reject_ffff = true;
// 較正オフセットの保存/復元ハンドシェイク。モード切替(yield 伴う)はサンプリング
// と競合するため、コマンドはフラグで受けて BNO スレッドのループ内で実行する。
static volatile uint8_t g_calib_cmd = 0; // 0=none, 1=save(読出), 2=load(書込)
static volatile bool g_calib_done = false;
static volatile bool g_calib_ok = false;
static uint8_t g_calib_buf[CALIB_PROFILE_LEN];

// push 先 (sink)。set_sample_sink / set_calib_sink で (obj<<16)|method を登録。
static uint32_t sample_sink_obj = 0xFFFF, sample_sink_method = 0;
static uint32_t calib_sink_obj = 0xFFFF, calib_sink_method = 0;

// ===========================================================================
//  BNO055 低レベル
// ===========================================================================
static void set_mode(uint8_t mode) {
  i2c_bus::write_reg(BNO055_ADDR, REG_OPR_MODE, OPMODE_CONFIG);
  obj_api::yield_us(30000);
  if (mode != OPMODE_CONFIG) {
    i2c_bus::write_reg(BNO055_ADDR, REG_OPR_MODE, mode);
    obj_api::yield_us(30000);
  }
}

static bool bno_init_sensor() {
  uint8_t id;
  if (i2c_bus::read_regs(BNO055_ADDR, REG_CHIP_ID, &id, 1) < 0) {
    printf("[BNO055] I2C read failed at ID\n");
    return false;
  }
  if (id != 0xA0) {
    printf("[BNO055] wrong ID 0x%02X (expected 0xA0)\n", id);
    return false;
  }

  // POR リセット → 起動待ち (~650ms)。yield で他スレッドを止めない。
  i2c_bus::write_reg(BNO055_ADDR, REG_SYS_TRIGGER, 0x20);
  obj_api::yield_us(700000);
  i2c_bus::write_reg(BNO055_ADDR, REG_UNIT_SEL, 0x00); // m/s^2, deg
  obj_api::yield_us(10000);
  set_mode(USE_MODE);
  return true;
}

// euler(0x1A)〜gravity(0x33) は連続領域なので euler / linaccel / gravity を 3 つに
// 分けず 1 ブロック読みでまとめて取る(I2C 占有=協調スケジューラの凍結を削減)。
// 途中の quaternion(0x20) は捨てる。
// calib(0x35) はこのバーストに含めず別の 1B トランザクションで読む: 400kHz Fast-mode
// では BNO055 の長いバースト末尾バイトが化け(calib が 0xFF に張り付く症状を実測)、
// calib はちょうど末尾に当たるため。短い独立読みなら末尾露出を避けられる。
static constexpr uint8_t REG_BLOCK_START = REG_EUL_HEAD_LSB; // 0x1A
static constexpr int MOTION_BLOCK_LEN =
    (REG_GRV_DATA_X_LSB + 6) - REG_EUL_HEAD_LSB; // 0x1A..0x33 = 26

// 静止時でも NDOF フュージョンの euler 出力は ±1 LSB(1/16°=0.0625°)で量子化
// ディザし続け、これがそのまま 0.06° の揺れとしてテレメトリに乗る。前回採用した
// 生値(1/16°単位の int16)から ±1 LSB 以内の変化はノイズとみなして据え置く
// デッドバンド。|Δ|≥2 LSB の実運動だけ通すので遅延ゼロ。整数比較のみ=soft-float
// 安全。レートを下げても振幅(±1 LSB)は変わらないので、ここで潰すのが本筋。
static int16_t held_eul[3] = {0, 0, 0};
static bool eul_primed = false;
static int16_t euler_deadband(int16_t raw, int16_t &held) {
  int d = (int)raw - held;
  if (!eul_primed || d > 1 || d < -1)
    held = raw;
  return held;
}

// 16bit レジスタ 1 個を 2B 読む。失敗 <0。
static int read_u16(uint8_t reg, int16_t *out) {
  uint8_t b[2];
  int r = i2c_bus::read_regs(BNO055_ADDR, reg, b, 2);
  if (r < 0)
    return r;
  *out = (int16_t)((b[1] << 8) | b[0]);
  return 0;
}

// euler(h/r/p) + linaccel(x/y/z) + gravity(x/y/z) の生 int16 を 9 個取得する。
// g_split_read に応じて、1 回の 26B ブロック読み か、9 回の 2B 個別読みを使う。
// 個別読みは「どのバーストも先頭 2B(=heading)だけは生存する」破損パターンの回避策。
static bool read_raw9(int16_t r[9]) {
  if (g_split_read) {
    static const uint8_t regs[9] = {
        REG_EUL_HEAD_LSB,       REG_EUL_HEAD_LSB + 2,   REG_EUL_HEAD_LSB + 4,
        REG_LIA_DATA_X_LSB,     REG_LIA_DATA_X_LSB + 2, REG_LIA_DATA_X_LSB + 4,
        REG_GRV_DATA_X_LSB,     REG_GRV_DATA_X_LSB + 2, REG_GRV_DATA_X_LSB + 4};
    for (int i = 0; i < 9; ++i)
      if (read_u16(regs[i], &r[i]) < 0)
        return false;
    return true;
  }
  uint8_t buf[MOTION_BLOCK_LEN];
  if (i2c_bus::read_regs(BNO055_ADDR, REG_BLOCK_START, buf, MOTION_BLOCK_LEN) < 0)
    return false;
  const uint8_t *eul = &buf[REG_EUL_HEAD_LSB - REG_BLOCK_START];   // 0
  const uint8_t *lia = &buf[REG_LIA_DATA_X_LSB - REG_BLOCK_START]; // 14
  const uint8_t *grv = &buf[REG_GRV_DATA_X_LSB - REG_BLOCK_START]; // 20
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

// 重力/線形加速度/オイラー角/較正状態を 1 サンプル読む。失敗時 false。
static bool read_motion(bno055_sample_t *s) {
  // バースト破損検出: NDOF を ~100Hz でポーリングすると、フュージョン更新に読みが
  // 重なったとき heading(先頭2B)以外のバースト全体(roll/pitch/linaccel/gravity)が
  // 丸ごと 0xFFFF に化ける現象がある(実測 ~19%、roll&pitch がきっかり -0.06°=0xFFFF、
  // az もぴったり 0)。roll&pitch==0xFFFF を破損センチネルとして検出し、1 回読み直して
  // 駄目ならサンプルごと捨てる(false)。偽の「水平」を制御へ流す方が危険なため、
  // 据え置きではなく破棄して valid=false にする。
  int16_t r[9];
  bool clean = false;
  for (int attempt = 0; attempt < 2 && !clean; ++attempt) {
    if (!read_raw9(r))
      return false;
    // roll&pitch==0xFFFF = 破損バースト。g_reject_ffff が off なら検出せず素通し。
    clean = !g_reject_ffff || !(r[1] == -1 && r[2] == -1);
  }
  if (!clean)
    return false; // 破損サンプルは公開しない

  s->heading = euler_deadband(r[0], held_eul[0]) / 16.0f;
  s->roll = euler_deadband(r[1], held_eul[1]) / 16.0f;
  s->pitch = euler_deadband(r[2], held_eul[2]) / 16.0f;
  eul_primed = true; // 3 ch 揃って初期化済みに

  s->lax = r[3] / 100.0f;
  s->lay = r[4] / 100.0f;
  s->laz = r[5] / 100.0f;
  s->gx = r[6] / 100.0f;
  s->gy = r[7] / 100.0f;
  s->gz = r[8] / 100.0f;

  // calib は別 1B トランザクション(バースト末尾破損回避。冒頭コメント参照)。
  // 失敗時は前回値を維持して motion サンプル自体は捨てない。
  uint8_t calib;
  if (i2c_bus::read_regs(BNO055_ADDR, REG_CALIB_STAT, &calib, 1) < 0)
    s->calib = latest.calib;
  else
    s->calib = calib;
  return true;
}

// 較正オフセット保存/復元を BNO スレッド内で実行する(モード切替の yield が
// サンプリングと競合しないよう、ループから直列に呼ぶ)。CONFIG モードでオフセット
// 領域(0x55..0x6A)を読み書きし、NDOF に戻す。
static void handle_calib_cmd() {
  uint8_t cmd = g_calib_cmd;
  if (cmd == 0)
    return;
  set_mode(OPMODE_CONFIG);
  if (cmd == 1) { // save: 現オフセットを吸い出す
    g_calib_ok = (i2c_bus::read_regs(BNO055_ADDR, REG_CALIB_OFFSET_START,
                                     g_calib_buf, CALIB_PROFILE_LEN) >= 0);
  } else { // load: オフセットを書き戻す
    bool ok = true;
    for (int i = 0; i < CALIB_PROFILE_LEN; ++i)
      if (i2c_bus::write_reg(BNO055_ADDR, REG_CALIB_OFFSET_START + i,
                             g_calib_buf[i]) < 0)
        ok = false;
    g_calib_ok = ok;
  }
  set_mode(USE_MODE);
  g_calib_done = true;
  g_calib_cmd = 0;
  // 結果を sink へ push (event-driven。登録時のみ)。受け手が同期コピーする。
  if (calib_sink_obj != 0xFFFF) {
    bno055_calib_xfer_t x;
    x.done = 1;
    x.ok = g_calib_ok ? 1 : 0;
    memcpy(x.data, g_calib_buf, CALIB_PROFILE_LEN);
    obj_api::svc(obj_api::svc_num::CALL_METHOD, calib_sink_obj, calib_sink_method,
                 (uint32_t)(uintptr_t)&x);
  }
}

// ===========================================================================
//  公開メソッド
// ===========================================================================
static void method_read_latest(uint32_t _caller_obj_id,
                               uint32_t _caller_thread_id, uint32_t out_ptr,
                               uint32_t _arg1) {
  (void)_caller_obj_id;
  (void)_caller_thread_id;
  (void)_arg1;
  if (out_ptr == 0)
    return;
  memcpy((void *)(uintptr_t)out_ptr, (const void *)&latest, sizeof(latest));
}

// arg0: 0=ブロック読み / 非0=2B 個別読み。
static void method_set_read_mode(uint32_t, uint32_t, uint32_t mode, uint32_t) {
  g_split_read = (mode != 0);
}

// arg0: 0=0xFFFF 破損を素通し / 非0=検出して破棄(既定)。
static void method_set_ffff_reject(uint32_t, uint32_t, uint32_t on, uint32_t) {
  g_reject_ffff = (on != 0);
}

// arg0 = (obj<<16)|method。新サンプル push 先を登録。
static void method_set_sample_sink(uint32_t, uint32_t, uint32_t packed, uint32_t) {
  sample_sink_obj = (packed >> 16) & 0xFFFF;
  sample_sink_method = packed & 0xFFFF;
}

// arg0 = (obj<<16)|method。較正 save/load 完了の push 先を登録。
static void method_set_calib_sink(uint32_t, uint32_t, uint32_t packed, uint32_t) {
  calib_sink_obj = (packed >> 16) & 0xFFFF;
  calib_sink_method = packed & 0xFFFF;
}

// 較正オフセットの吸い出し要求(実処理は BNO ループ内 handle_calib_cmd)。
static void method_calib_save(uint32_t, uint32_t, uint32_t, uint32_t) {
  g_calib_done = false;
  g_calib_ok = false;
  g_calib_cmd = 1;
}

// arg0: uint8_t[22] への ptr。オフセットを取り込み書き戻し要求する。
static void method_calib_load(uint32_t, uint32_t, uint32_t in_ptr, uint32_t) {
  if (in_ptr == 0)
    return;
  memcpy(g_calib_buf, (const void *)(uintptr_t)in_ptr, CALIB_PROFILE_LEN);
  g_calib_done = false;
  g_calib_ok = false;
  g_calib_cmd = 2;
}

// arg0: bno055_calib_xfer_t* 。直近 save/load の done/ok とダンプを返す。
static void method_calib_get(uint32_t, uint32_t, uint32_t out_ptr, uint32_t) {
  if (out_ptr == 0)
    return;
  bno055_calib_xfer_t x;
  x.done = g_calib_done ? 1 : 0;
  x.ok = g_calib_ok ? 1 : 0;
  memcpy(x.data, g_calib_buf, CALIB_PROFILE_LEN);
  memcpy((void *)(uintptr_t)out_ptr, &x, sizeof(x));
}

// ===========================================================================
//  オブジェクトエントリ
// ===========================================================================
void BNO055_DRIVER::init() {
  printf("[BNO055] init\n");
  export_method<method_read_latest>(BNO055_DRIVER::METHOD_IDs::read_latest);
  export_method<method_set_read_mode>(BNO055_DRIVER::METHOD_IDs::set_read_mode);
  export_method<method_set_ffff_reject>(BNO055_DRIVER::METHOD_IDs::set_ffff_reject);
  export_method<method_set_sample_sink>(BNO055_DRIVER::METHOD_IDs::set_sample_sink);
  export_method<method_set_calib_sink>(BNO055_DRIVER::METHOD_IDs::set_calib_sink);
  export_method<method_calib_save>(BNO055_DRIVER::METHOD_IDs::calib_save);
  export_method<method_calib_load>(BNO055_DRIVER::METHOD_IDs::calib_load);
  export_method<method_calib_get>(BNO055_DRIVER::METHOD_IDs::calib_get);

  i2c_bus::init();
  if (!bno_init_sensor()) {
    printf("[BNO055] init failed — driver idles (valid=false)\n");
    while (true) {
      latest.valid = false;
      obj_api::yield_us(500000);
    }
  }
  printf("[BNO055] sensor OK (mode=0x%02X)\n", USE_MODE);

  // NDOF フュージョン出力はハード的に 100Hz 固定なので、その限界で読み続ける
  // (これより速く読んでも同じ値しか出ない)。
  uint64_t next = time_us_64();
  while (true) {
    // 較正の保存/復元要求があれば先に処理(モード切替で数十ms掛かるのでグリッド再同期)。
    if (g_calib_cmd != 0) {
      handle_calib_cmd();
      next = time_us_64();
    }
    next += 10000; // 100Hz の絶対グリッド
    bno055_sample_t s = {};
    if (read_motion(&s)) {
      s.seq = latest.seq + 1;
      s.valid = true;
      latest = s;
      // 新サンプルを sink へ push (event-driven。登録時のみ)。
      if (sample_sink_obj != 0xFFFF)
        obj_api::svc(obj_api::svc_num::CALL_METHOD, sample_sink_obj,
                     sample_sink_method, (uint32_t)(uintptr_t)&latest);
    } else
      latest.valid = false;

    uint64_t now = time_us_64();
    if ((int64_t)(next - now) < 0) // 万一読みが10ms超 → 取りこぼし再同期
      next = now;
    else
      obj_api::yield_until_us(next); // 締切に張り付いて true 100Hz(±数µs)
  }
}

} // namespace shizu
