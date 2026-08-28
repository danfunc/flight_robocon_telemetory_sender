// ===========================================================================
//  BNO055 (IMU) — Shizuku オブジェクト版 (v1)
// ===========================================================================
//  レジスタ/初期化シーケンスは flight_robocon_telemetory_sender/core1_io.cpp
//  の BNO055 セクションをそのまま踏襲(実機で検証済みの値)。I2C そのものは
//  Shizuku 側の汎用 i2c オブジェクト (peripherals.cpp) を経由する。
#include "bno055.hpp"
#include "shizuku/kernel.hpp"
#include "shizuku/object_api.hpp"
#include "shizuku/objects/peripherals.hpp"
#include "shizuku/stream.hpp"
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace bno055 {
namespace {

using ARCH = shizuku::KERNEL::ARCH;
using BOARD = shizuku::KERNEL::BOARD;

struct call_result {
  uintptr_t error;
  uintptr_t value;
};

call_result api(shizuku::object_api number, uintptr_t a1 = 0, uintptr_t a2 = 0,
                uintptr_t a3 = 0) {
  const auto result = ARCH::syscall((uintptr_t)number, a1, a2, a3);
  return {result.error, result.value};
}

uintptr_t export_method(method m, uintptr_t entry) {
  return api(shizuku::object_api::EXPORT_METHOD, (uintptr_t)m, entry).error;
}

// ---- 出口: 自分のストリーム 1 本 (D46)
// --------------------------------------- ★かつては ble_uart::SEND を同期
// CALL_METHOD で叩いていたが廃止した。
//   CALL_METHOD は呼び出し元スレッドで走るので、共有 TX リングの producer が
//   複数になり (bno055 / bme280 / ble_uart 自身)、`wr` の非アトミックな
//   read-modify-write が競合して**無音でレコードが消える**状態だった。
//   ストリームは object 対 object なので、送り手は自分の路を持ち、
//   合流は flight_controller (ハブ) がやる。
shizuku::stream::storage<sample_t, 8> g_out;
uintptr_t g_out_id = 0;
uint32_t g_dropped = 0;
uint32_t g_sequence = 0;

uintptr_t method_get_stream(uintptr_t, uintptr_t, uintptr_t, uintptr_t) {
  return g_out_id;
}

// ---- 配線・レジスタ (旧実装 core1_io.cpp の値をそのまま) --------------------
constexpr uint32_t I2C_INSTANCE = 1; // I2C1 (GP6/GP7)
constexpr uint32_t SDA_PIN = 6;
constexpr uint32_t SCL_PIN = 7;
constexpr uint32_t BAUDRATE = 100000; // 400k は「バースト末尾が化ける」実測あり
constexpr uint32_t TIMEOUT_US = 10000; // 26B バーストに 2ms は足りなかった実測

constexpr uint8_t BNO055_ADDR = 0x28;
constexpr uint8_t REG_CHIP_ID = 0x00;
constexpr uint8_t REG_EUL_HEAD_LSB = 0x1A;
constexpr uint8_t REG_LIA_DATA_X_LSB = 0x28;
constexpr uint8_t REG_GRV_DATA_X_LSB = 0x2E;
constexpr uint8_t REG_OPR_MODE = 0x3D;
constexpr uint8_t REG_SYS_TRIGGER = 0x3F;
constexpr uint8_t REG_UNIT_SEL = 0x3B;
constexpr uint8_t OPMODE_CONFIG = 0x00;
constexpr uint8_t OPMODE_NDOF = 0x0C;
constexpr uint8_t REG_BLOCK_START = REG_EUL_HEAD_LSB; // 0x1A
constexpr int MOTION_BLOCK_LEN =
    (REG_GRV_DATA_X_LSB + 6) - REG_EUL_HEAD_LSB; // 26

// ---- I2C ヘルパ (Shizuku の汎用 i2c オブジェクトを CALL_METHOD で叩く)
// -------
bool i2c_write_reg(uint8_t reg, uint8_t value) {
  uint8_t tx[2] = {reg, value};
  shizuku::objects::i2c_transfer t{};
  t.instance = I2C_INSTANCE;
  t.address = BNO055_ADDR;
  t.tx = tx;
  t.tx_len = sizeof(tx);
  const auto r =
      api(shizuku::object_api::CALL_METHOD, shizuku::objects::I2C_OBJECT,
          (uintptr_t)shizuku::objects::i2c_method::WRITE, (uintptr_t)&t);
  return r.value == sizeof(tx);
}

bool i2c_read_regs(uint8_t reg, uint8_t *buf, uint32_t len) {
  shizuku::objects::i2c_transfer t{};
  t.instance = I2C_INSTANCE;
  t.address = BNO055_ADDR;
  t.tx = &reg;
  t.tx_len = 1;
  t.rx = buf;
  t.rx_len = len;
  const auto r =
      api(shizuku::object_api::CALL_METHOD, shizuku::objects::I2C_OBJECT,
          (uintptr_t)shizuku::objects::i2c_method::WRITE_READ, (uintptr_t)&t);
  return r.value == len;
}

// ---- BNO055 初期化 (旧実装 core1_io.cpp: bno_init_sensor / bno_set_mode) ----
void bno_set_mode(uint8_t mode) {
  i2c_write_reg(REG_OPR_MODE, OPMODE_CONFIG);
  api(shizuku::object_api::SLEEP_US, 30000);
  if (mode != OPMODE_CONFIG) {
    i2c_write_reg(REG_OPR_MODE, mode);
    api(shizuku::object_api::SLEEP_US, 30000);
  }
}

bool bno_init_sensor() {
  uint8_t id = 0;
  if (!i2c_read_regs(REG_CHIP_ID, &id, 1) || id != 0xA0)
    return false;
  i2c_write_reg(REG_SYS_TRIGGER, 0x20); // POR リセット
  api(shizuku::object_api::SLEEP_US, 700000);
  i2c_write_reg(REG_UNIT_SEL, 0x00); // m/s^2, deg
  api(shizuku::object_api::SLEEP_US, 10000);
  bno_set_mode(OPMODE_NDOF);
  return true;
}

// ---- 0xFFFF (= int16 の -1) 混入の据え置き
// ------------------------------------ ★NDOF フュージョンの更新中に読むと、値が
// 0xFFFF のまま返ることがある (実測。
//   `EUL=5759,-1,-1 LIA=-1,-1,-1 GRV=-1,-1,-1` のような形で、**値ごとに**
//   バラバラに出る — 3 軸まとめてではない)。旧実装 (core1_io.cpp)
//   は「据え置いて 前回値を使う」で対策していた。
//   ★これを直さないと下流が実際に壊れる: telemetry は「各センサの最新値を
//     保持して 1 行にする」ので、一度 -1 を掴むと**次の正常読みまでその欄が
//     -0.01 に張り付く** (重力 9.8 が -0.01
//     として送られる)。センサの故障モードは
//     センサが吸収するのが筋なので、ここで止める。
//   ★物理的に -1 (= -0.01 m/s^2 / -1/16 deg) は「ほぼ 0」なので、真値だった
//     場合に据え置いても実害は無い (旧実装と同じ割り切り)。
int32_t g_last_good[9] = {};
bool g_have_last = false;
uint32_t g_glitches = 0;

void hold_glitches(int16_t r[9]) {
  for (uint32_t i = 0; i < 9; ++i) {
    if (r[i] == (int16_t)-1) {
      ++g_glitches;
      if (g_have_last)
        r[i] = (int16_t)g_last_good[i];
    } else {
      g_last_good[i] = r[i];
    }
  }
  g_have_last = true;
}

// euler(3) + linaccel(3) + gravity(3) の生 int16。★v1 は 1 回読みのみ
// (旧実装の split-read A/B 切替・デッドバンドは今回のスコープ外)。
bool read_motion9(int16_t r[9]) {
  uint8_t buf[MOTION_BLOCK_LEN];
  if (!i2c_read_regs(REG_BLOCK_START, buf, MOTION_BLOCK_LEN))
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

// ---- ポーリング (SPAWN で起こす)
// ---------------------------------------------
constexpr uint32_t PERIOD_US =
    100000; // 10Hz (core0 を BLE と共有するため控えめ)
constexpr uint32_t REPORT_EVERY = 10; // 1 秒ごとに診断 printf

uintptr_t poll_loop(uintptr_t, uintptr_t, uintptr_t, uintptr_t) {
  // ★席に座るのはこのスレッド (席は発行元オブジェクトから導出される)。
  api(shizuku::object_api::STREAM_BIND, g_out_id,
      (uintptr_t)shizuku::stream::role::PRODUCER);
  if (!bno_init_sensor()) {
    BOARD::diag_printf("[BNO055] init failed (no ACK / wrong chip id) — "
                       "配線 (I2C%lu SDA=GP%lu SCL=GP%lu) と電源を確認\n",
                       (unsigned long)I2C_INSTANCE, (unsigned long)SDA_PIN,
                       (unsigned long)SCL_PIN);
    return 1;
  }
  BOARD::diag_printf("[BNO055] init ok (NDOF, %luHz)\n",
                     (unsigned long)(1000000u / PERIOD_US));

  uint64_t next = BOARD::time_us() + PERIOD_US;
  uint32_t since_report = 0;
  uint32_t fail_count = 0;
  while (true) {
    // ★実験 (2026-08-26): 遅れている周回でも **必ず** SLEEP_US を通す。
    //   関門 (停止中は抜けない) が sleep_us の中にあるので、呼ばない周回は
    //   素通りしてしまう、という仮説の検証。
    const int64_t remaining = (int64_t)(next - BOARD::time_us());
    api(shizuku::object_api::SLEEP_US,
        (uintptr_t)(remaining > 0 ? remaining : 0));
    next += PERIOD_US;

    int16_t r[9];
    if (!read_motion9(r)) {
      ++fail_count;
      continue;
    }
    hold_glitches(r);

    // ★数値のまま流す。整形 (テレメトリ 1 行への合流) はハブの仕事。
    sample_t s{};
    s.kind = xno::sample_kind::IMU;
    s.sequence = ++g_sequence;
    for (uint32_t i = 0; i < 9; ++i)
      s.value[i] = r[i];
    if (!g_out.hdl().push(s))
      ++g_dropped;

    if (++since_report >= REPORT_EVERY) {
      since_report = 0;
    }
  }
  return 0;
}

uintptr_t bno055_main(uintptr_t, uintptr_t, uintptr_t, uintptr_t) {
  uintptr_t failures =
      api(shizuku::object_api::DECLARE_NAME, (uintptr_t)"bno055").error;
  failures += export_method(method::POLL, (uintptr_t)&poll_loop);
  failures += export_method(method::GET_STREAM, (uintptr_t)&method_get_stream);

  g_out.init();
  const auto created =
      api(shizuku::object_api::STREAM_CREATE, (uintptr_t)&g_out.desc);
  failures += created.error;
  g_out_id = created.value;

  // I2C バスの設定。★複数のセンサが同じバスを共有する場合、最初に呼んだ側の
  //   設定が有効になる (i2c_init
  //   は再設定可能だが、ここでは単純化して毎回叩く)。
  shizuku::objects::i2c_config config{};
  config.instance = I2C_INSTANCE;
  config.baudrate = BAUDRATE;
  config.sda_pin = SDA_PIN;
  config.scl_pin = SCL_PIN;
  config.timeout_us = TIMEOUT_US;
  api(shizuku::object_api::CALL_METHOD, shizuku::objects::I2C_OBJECT,
      (uintptr_t)shizuku::objects::i2c_method::CONFIGURE, (uintptr_t)&config);

  return failures;
}

} // namespace

uint32_t register_bno055() {
  // ★affinity は指定しない。このオブジェクトは cyw43/btstack に一切触れず
  //   (出口は自分のストリームだけ)、I2C は Shizuku の i2c オブジェクトが
  //   自前のロックで直列化する。ストリームは 2 コア間で成り立つ設計なので、
  //   どのコアで走ってもよい。
  //   ※ 以前は OBJECT_ON_CORE(0) を付けていた。ble_uart::SEND を同期呼び出しで
  //     叩いていた頃、共有 TX リングの直列化が「全 producer が core0 に居る」
  //     ことに依存していたため。ハブ化 (D46) でその依存ごと消えた。
  const auto created = api(shizuku::object_api::CREATE_OBJECT, OBJECT,
                           (uintptr_t)&bno055_main, shizuku::OBJECT_PRIVILEGED);
  const auto started = api(shizuku::object_api::CALL_METHOD, OBJECT, 0, 0);
  if (created.error != 0 || started.error != 0 || started.value != 0) {
    BOARD::diag_printf(
        "[BNO055] FAILED: create=%lu call=%lu exports_failed=%lu\n",
        (unsigned long)created.error, (unsigned long)started.error,
        (unsigned long)started.value);
    return 1;
  }
  BOARD::diag_printf("[BNO055] registered (object %lu, stream %lu)\n",
                     (unsigned long)OBJECT, (unsigned long)g_out_id);
  return 0;
}

uint32_t start_bno055() {
  const auto spawned =
      api(shizuku::object_api::SPAWN, OBJECT, (uintptr_t)method::POLL, 0);
  if (spawned.error != 0) {
    BOARD::diag_printf("[BNO055] could not spawn the poll loop (%lu)\n",
                       (unsigned long)spawned.error);
    return 1;
  }
  BOARD::diag_printf("[BNO055] poll thread %lu started\n",
                     (unsigned long)spawned.value);
  return 0;
}

} // namespace bno055
