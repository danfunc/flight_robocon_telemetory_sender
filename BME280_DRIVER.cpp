// ===========================================================================
//  BME280_DRIVER — 気圧/温度センサを Shizuku オブジェクト化したドライバ
// ===========================================================================
//  元実装 test_firmware/altitude_fusion_wifi.c の BME280 部分を Shizuku の
//  オブジェクトモデルへ移植したもの。専用スレッド (init) が:
//    1) I2C/センサを初期化し較正係数を読む
//    2) 地上気圧をキャリブレーション (高度ゼロ点)
//    3) 周期的に気圧/温度を読み、補償演算 + 高度換算して内部キャッシュへ
//  を行い、read_latest メソッドで他オブジェクトへスナップショットを渡す。
// ===========================================================================
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <export_method.hpp>
#include <i2c_bus.hpp>
#include <obj_api.hpp>
#include <soft_math.hpp> // FPU を踏まない powf (kernel が FP 例外フレーム非対応のため)
#include <object_headers/BME280_DRIVER.hpp>
#include <pico/time.h>

namespace shizu {

// ---- レジスタ定義 ----------------------------------------------------------
static constexpr uint8_t BME280_ADDR = 0x76;
static constexpr uint8_t REG_ID = 0xD0;
static constexpr uint8_t REG_RESET = 0xE0;
static constexpr uint8_t REG_CTRL_HUM = 0xF2;
static constexpr uint8_t REG_CTRL_MEAS = 0xF4;
static constexpr uint8_t REG_CONFIG = 0xF5;
static constexpr uint8_t REG_PRESS_MSB = 0xF7;

// ---- 較正係数 (init 時にセンサから読み出す) --------------------------------
static uint16_t dig_T1;
static int16_t dig_T2, dig_T3;
static uint16_t dig_P1;
static int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
static int32_t t_fine;

// ---- 高度ゼロ点 ------------------------------------------------------------
static float ground_hpa = 1013.25f;

// ---- 最新サンプル (read_latest が返す) -------------------------------------
// 協調スケジューリングなので、更新中に別スレッドが割り込むことはない
// (本スレッドが yield しない限り)。read_latest 側も yield せずコピーするだけ。
static bme280_sample_t latest = {0, 0.f, 0.f, 0.f, false};

// 新サンプルの push 先 (sink)。set_sample_sink で (obj<<16)|method を登録。0xFFFF=無効。
static uint32_t sample_sink_obj = 0xFFFF;
static uint32_t sample_sink_method = 0;

// サンプリング一時停止 (スループット試験中に I2C を止めて帯域に振る)。
static volatile bool g_paused = false;

// ===========================================================================
//  BME280 低レベル
// ===========================================================================
static bool bme_init_sensor() {
  uint8_t id;
  if (i2c_bus::read_regs(BME280_ADDR, REG_ID, &id, 1) < 0) {
    printf("[BME280] I2C read failed at ID\n");
    return false;
  }
  if (id != 0x60) {
    printf("[BME280] wrong ID 0x%02X (expected 0x60)\n", id);
    return false;
  }

  i2c_bus::write_reg(BME280_ADDR, REG_RESET, 0xB6);
  obj_api::yield_us(10000);

  uint8_t b[26];
  if (i2c_bus::read_regs(BME280_ADDR, 0x88, b, 26) < 0) {
    printf("[BME280] calib read failed\n");
    return false;
  }
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

  // 温度 oversampling x2, 気圧 oversampling x16, normal mode。
  i2c_bus::write_reg(BME280_ADDR, REG_CTRL_HUM, 0x01);
  i2c_bus::write_reg(BME280_ADDR, REG_CTRL_MEAS, (2 << 5) | (5 << 2) | 3);
  i2c_bus::write_reg(BME280_ADDR, REG_CONFIG, (0 << 5) | (4 << 2));
  obj_api::yield_us(100000);
  return true;
}

// データシートの固定小数点補償式 (整数演算のみ。t_fine を共有する)。
static int32_t compensate_T(int32_t adc_T) {
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

static uint32_t compensate_P(int32_t adc_P) {
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

// 気圧 → 高度 (国際標準大気の簡易式)。
static float pressure_to_altitude(float press_hpa, float temp_c, float p0_hpa) {
  return ((temp_c + 273.15f) / 0.0065f) *
         (soft_math::powf(p0_hpa / press_hpa, 0.190234f) - 1.0f);
}

// 生レジスタ読み → 温度/気圧へ補償。失敗時 false。
static bool read_raw(float *temp_c, float *press_hpa) {
  uint8_t buf[6];
  if (i2c_bus::read_regs(BME280_ADDR, REG_PRESS_MSB, buf, 6) < 0)
    return false;
  int32_t adc_P = (int32_t)((buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4));
  int32_t adc_T = (int32_t)((buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4));
  *temp_c = compensate_T(adc_T) / 100.0f;
  *press_hpa = compensate_P(adc_P) / 100.0f;
  return true;
}

// 地上気圧を複数サンプル平均で測る (高度ゼロ点)。長い待ちは yield で協調する。
static float calibrate_ground(int samples) {
  float sum_p = 0.0f;
  int ok = 0;
  for (int i = 0; i < samples; i++) {
    float t, p;
    if (read_raw(&t, &p)) {
      sum_p += p;
      ok++;
    }
    obj_api::yield_us(30000); // 30ms 毎。yield なので BLE 等も並行して回る
  }
  if (ok == 0) {
    printf("[BME280] *** all calibration reads failed ***\n");
    return 1013.25f;
  }
  return sum_p / ok;
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
  // 呼び出し元のバッファへ最新スナップショットをコピー (同一アドレス空間)。
  memcpy((void *)(uintptr_t)out_ptr, (const void *)&latest, sizeof(latest));
}

// arg0 = (obj<<16)|method。新サンプル push 先を登録。
static void method_set_sample_sink(uint32_t, uint32_t, uint32_t packed, uint32_t) {
  sample_sink_obj = (packed >> 16) & 0xFFFF;
  sample_sink_method = packed & 0xFFFF;
}

// arg0: 非0=サンプリング一時停止 / 0=再開。
static void method_set_paused(uint32_t, uint32_t, uint32_t on, uint32_t) {
  g_paused = (on != 0);
}

static void method_rezero(uint32_t _a, uint32_t _b, uint32_t _c, uint32_t _d) {
  (void)_a;
  (void)_b;
  (void)_c;
  (void)_d;
  // その場の数サンプルで地上気圧を取り直す (打ち上げ直前のゼロ点合わせ)。
  ground_hpa = calibrate_ground(20);
  printf("[BME280] rezero -> ground=%d Pa\n", (int)(ground_hpa * 100.0f));
}

// ===========================================================================
//  オブジェクトエントリ
// ===========================================================================
void BME280_DRIVER::init() {
  printf("[BME280] init\n");
  export_method<method_read_latest>(BME280_DRIVER::METHOD_IDs::read_latest);
  export_method<method_rezero>(BME280_DRIVER::METHOD_IDs::rezero);
  export_method<method_set_sample_sink>(BME280_DRIVER::METHOD_IDs::set_sample_sink);
  export_method<method_set_paused>(BME280_DRIVER::METHOD_IDs::set_paused);

  i2c_bus::init();
  if (!bme_init_sensor()) {
    printf("[BME280] init failed — driver idles (valid=false)\n");
    while (true) {
      latest.valid = false;
      obj_api::yield_us(500000);
    }
  }
  printf("[BME280] sensor OK, calibrating ground pressure...\n");
  ground_hpa = calibrate_ground(40);
  printf("[BME280] ground pressure = %d Pa\n", (int)(ground_hpa * 100.0f));

  // 変換時間に合わせてサンプリングしてキャッシュを更新し続ける。
  // 現設定 (温度×2, 気圧×16, 湿度×1, normal mode) の 1 変換は最悪 ~46ms かかる
  // ので、新しいデータが出る実レートの上限は ~21Hz。これより速くポーリングしても
  // 変換完了前の同じ値を二度読みするだけなので、その限界に合わせて回す。
  while (true) {
    // スループット試験中は I2C を止めて帯域を空ける(yield で BLE は回り続ける)。
    if (g_paused) {
      obj_api::yield_us(50000);
      continue;
    }
    float t, p;
    if (read_raw(&t, &p)) {
      bme280_sample_t s;
      s.seq = latest.seq + 1;
      s.temp_c = t;
      s.press_hpa = p;
      s.alt_m = pressure_to_altitude(p, t, ground_hpa);
      s.valid = true;
      latest = s; // 構造体ごと差し替え (yield しないので原子的)
      // 新サンプルを sink へ push (event-driven。登録時のみ)。
      if (sample_sink_obj != 0xFFFF)
        obj_api::svc(obj_api::svc_num::CALL_METHOD, sample_sink_obj,
                     sample_sink_method, (uint32_t)(uintptr_t)&latest);
    } else {
      latest.valid = false;
    }
    obj_api::yield_us(47000); // ~21Hz (気圧×16 oversampling の変換時間が律速)
  }
}

} // namespace shizu
