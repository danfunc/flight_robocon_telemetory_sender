// ===========================================================================
//  BME280 (気圧/温度) — Shizuku オブジェクト版 (v1)
// ===========================================================================
//  レジスタ/補償式は flight_robocon_telemetory_sender/core1_io.cpp の
//  BME280 セクション(Bosch データシートの固定小数点補償式)をそのまま踏襲。
#include "bme280.hpp"
#include "shizuku/kernel.hpp"
#include "shizuku/object_api.hpp"
#include "shizuku/objects/peripherals.hpp"
#include "shizuku/stream.hpp"
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace bme280 {
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

// ---- 出口: 自分のストリーム 1 本 (D46。理由は bno055.cpp の同じ箇所を参照)
// ----
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
constexpr uint32_t BAUDRATE = 100000;
constexpr uint32_t TIMEOUT_US = 10000;

constexpr uint8_t BME280_ADDR = 0x76;
constexpr uint8_t BME_REG_ID = 0xD0;
constexpr uint8_t BME_REG_RESET = 0xE0;
constexpr uint8_t BME_REG_CTRL_HUM = 0xF2;
constexpr uint8_t BME_REG_CTRL_MEAS = 0xF4;
constexpr uint8_t BME_REG_CONFIG = 0xF5;
constexpr uint8_t BME_REG_PRESS_MSB = 0xF7;

uint16_t dig_T1;
int16_t dig_T2, dig_T3;
uint16_t dig_P1;
int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
int32_t t_fine;

// ---- I2C ヘルパ (BNO055 側と同じ骨格) ---------------------------------------
bool i2c_write_reg(uint8_t reg, uint8_t value) {
  uint8_t tx[2] = {reg, value};
  shizuku::objects::i2c_transfer t{};
  t.instance = I2C_INSTANCE;
  t.address = BME280_ADDR;
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
  t.address = BME280_ADDR;
  t.tx = &reg;
  t.tx_len = 1;
  t.rx = buf;
  t.rx_len = len;
  const auto r =
      api(shizuku::object_api::CALL_METHOD, shizuku::objects::I2C_OBJECT,
          (uintptr_t)shizuku::objects::i2c_method::WRITE_READ, (uintptr_t)&t);
  return r.value == len;
}

// ---- 初期化 (旧実装 core1_io.cpp: bme_init_sensor) --------------------------
bool bme_init_sensor() {
  uint8_t id = 0;
  if (!i2c_read_regs(BME_REG_ID, &id, 1) || id != 0x60)
    return false;
  i2c_write_reg(BME_REG_RESET, 0xB6);
  api(shizuku::object_api::SLEEP_US, 10000);
  uint8_t b[26];
  if (!i2c_read_regs(0x88, b, 26))
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
  // 温度 oversampling x2, 気圧 x16, normal mode (旧実装と同一設定)。
  i2c_write_reg(BME_REG_CTRL_HUM, 0x01);
  i2c_write_reg(BME_REG_CTRL_MEAS, (2 << 5) | (5 << 2) | 3);
  i2c_write_reg(BME_REG_CONFIG, (0 << 5) | (4 << 2));
  api(shizuku::object_api::SLEEP_US, 100000);
  return true;
}

// データシートの固定小数点補償式 (整数のみ)。戻り値 0.01℃。
int32_t compensate_T(int32_t adc_T) {
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
uint32_t compensate_P(int32_t adc_P) {
  int64_t var1, var2, p;
  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)dig_P6;
  var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
  var2 = var2 + (((int64_t)dig_P4) << 35);
  var1 =
      ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
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
bool bme_read(uint32_t *press_pa, int32_t *temp_cc) {
  uint8_t buf[6];
  if (!i2c_read_regs(BME_REG_PRESS_MSB, buf, 6))
    return false;
  int32_t adc_P = (int32_t)((buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4));
  int32_t adc_T = (int32_t)((buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4));
  *temp_cc = compensate_T(adc_T);
  *press_pa = compensate_P(adc_P);
  return true;
}

// ---- ポーリング (SPAWN で起こす)
// ---------------------------------------------
constexpr uint32_t PERIOD_US =
    200000; // 5Hz (気圧/温度は BNO055 ほど速く要らない)
constexpr uint32_t REPORT_EVERY = 5; // 1 秒ごとに診断 printf

uintptr_t poll_loop(uintptr_t, uintptr_t, uintptr_t, uintptr_t) {
  // ★席に座るのはこのスレッド (席は発行元オブジェクトから導出される)。
  api(shizuku::object_api::STREAM_BIND, g_out_id,
      (uintptr_t)shizuku::stream::role::PRODUCER);
  if (!bme_init_sensor()) {
    BOARD::diag_printf("[BME280] init failed (no ACK / wrong chip id) — "
                       "配線 (I2C%lu SDA=GP%lu SCL=GP%lu, addr 0x76) を確認\n",
                       (unsigned long)I2C_INSTANCE, (unsigned long)SDA_PIN,
                       (unsigned long)SCL_PIN);
    return 1;
  }
  BOARD::diag_printf("[BME280] init ok (%luHz)\n",
                     (unsigned long)(1000000u / PERIOD_US));

  uint64_t next = BOARD::time_us() + PERIOD_US;
  uint32_t since_report = 0;
  uint32_t fail_count = 0;
  while (true) {
    const int64_t remaining = (int64_t)(next - BOARD::time_us());
    if (remaining > 0)
      api(shizuku::object_api::SLEEP_US, (uintptr_t)remaining);
    next += PERIOD_US;

    uint32_t press_pa;
    int32_t temp_cc;
    if (!bme_read(&press_pa, &temp_cc)) {
      ++fail_count;
      continue;
    }

    sample_t s{};
    s.kind = xno::sample_kind::BARO;
    s.sequence = ++g_sequence;
    s.value[0] = (int32_t)press_pa;
    s.value[1] = temp_cc;
    if (!g_out.hdl().push(s))
      ++g_dropped;

    if (++since_report >= REPORT_EVERY) {
      since_report = 0;
      BOARD::diag_printf("[BME280] PRESS=%lu TEMP=%ld (fail=%lu drop=%lu)\n",
                         (unsigned long)press_pa, (long)temp_cc,
                         (unsigned long)fail_count, (unsigned long)g_dropped);
    }
  }
  return 0;
}

uintptr_t bme280_main(uintptr_t, uintptr_t, uintptr_t, uintptr_t) {
  uintptr_t failures =
      api(shizuku::object_api::DECLARE_NAME, (uintptr_t)"bme280").error;
  failures += export_method(method::POLL, (uintptr_t)&poll_loop);
  failures += export_method(method::GET_STREAM, (uintptr_t)&method_get_stream);

  g_out.init();
  const auto stream_created =
      api(shizuku::object_api::STREAM_CREATE, (uintptr_t)&g_out.desc);
  failures += stream_created.error;
  g_out_id = stream_created.value;

  // ★I2C バスは BNO055 と共有。既に CONFIGURE 済みでも再設定は無害
  //   (i2c_init は再初期化可能) なので、単独登録でも動くようにここでも呼ぶ。
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

uint32_t register_bme280() {
  // ★affinity は指定しない (理由は bno055.cpp の register_bno055() と同じ)。
  const auto created = api(shizuku::object_api::CREATE_OBJECT, OBJECT,
                           (uintptr_t)&bme280_main, shizuku::OBJECT_PRIVILEGED);
  const auto started = api(shizuku::object_api::CALL_METHOD, OBJECT, 0, 0);
  if (created.error != 0 || started.error != 0 || started.value != 0) {
    BOARD::diag_printf(
        "[BME280] FAILED: create=%lu call=%lu exports_failed=%lu\n",
        (unsigned long)created.error, (unsigned long)started.error,
        (unsigned long)started.value);
    return 1;
  }
  BOARD::diag_printf("[BME280] registered (object %lu, stream %lu)\n",
                     (unsigned long)OBJECT, (unsigned long)g_out_id);
  return 0;
}

uint32_t start_bme280() {
  const auto spawned =
      api(shizuku::object_api::SPAWN, OBJECT, (uintptr_t)method::POLL, 0);
  if (spawned.error != 0) {
    BOARD::diag_printf("[BME280] could not spawn the poll loop (%lu)\n",
                       (unsigned long)spawned.error);
    return 1;
  }
  BOARD::diag_printf("[BME280] poll thread %lu started\n",
                     (unsigned long)spawned.value);
  return 0;
}

} // namespace bme280
