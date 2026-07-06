// ===========================================================================
//  BME280_DRIVER — 気圧/温度センサの Shizuku オブジェクト (core0 側: 換算係)
// ===========================================================================
//  step2 (docs/sensor_stream_protocol.md §7) で I2C アクセスと整数補償演算
//  (compensate_T/P) は core1 (core1_io.cpp) へ移った。本オブジェクトは
//  I2C を一切触らず、
//    - BNO055_DRIVER のリング排出スレッドから bme280_on_baro / bme280_on_ground
//      で整数値 (Pa / 0.01℃) を受け取り、float 換算 + 測高公式 (hypsometric)
//      で bme280_sample_t を作って公開する
//    - rezero はコマンドリングで core1 へ転送し、結果は CH_GROUND で返る
//  外部インタフェース (メソッド ID / セマンティクス) は従来と完全互換。
// ===========================================================================
#include <cmath> // 標準 libm の powf (測高公式は core0 に残る)
#include <core_ring.hpp>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <export_method.hpp>
#include <obj_api.hpp>
#include <object_headers/BME280_DRIVER.hpp>
#include <pico/time.h>

namespace shizu {

// ---- 高度ゼロ点 (CH_GROUND レコードで core1 から届く) ------------------------
static float ground_hpa = 1013.25f;
// 地上較正時の基準温度 [℃]。測高公式には「地上と現在の平均温度 (気層平均)」を
// 使う。瞬時のチップ温度だけを使うと、自己発熱の立ち上がりや気流による温度変動が
// スケール係数としてそのまま高度に乗ってしまう。
static float ground_temp_c = 15.0f;
// 起動時較正 (CH_GROUND 初回受信) まではサンプルを公開しない (従来の
// 「calibrate_ground が終わるまで latest.valid=false」と等価)。
static bool g_ground_ready = false;

// ---- 最新サンプル (read_latest が返す) -------------------------------------
static bme280_sample_t latest = {0, 0.f, 0.f, 0.f, false};

// 新サンプルの push 先 (sink)。set_sample_sink で (obj<<16)|method を登録。
static uint32_t sample_sink_obj = 0xFFFF;
static uint32_t sample_sink_method = 0;

// 鮮度監視 (BARO レコードが 500ms 途絶えたら valid=false。I2C 断相当)。
static uint64_t g_last_baro_us = 0;

// 気圧 → 高度 (測高公式)。温度は「地上較正時 T0 と現在 T の平均」= 気層平均。
static float pressure_to_altitude(float press_hpa, float temp_c, float p0_hpa) {
  float t_layer_c = 0.5f * (temp_c + ground_temp_c);
  return ((t_layer_c + 273.15f) / 0.0065f) *
         (powf(p0_hpa / press_hpa, 0.190234f) - 1.0f);
}

// ===========================================================================
//  リング排出スレッド (BNO055_DRIVER) からのディスパッチフック
// ===========================================================================
// 協調スケジューラなので BNO スレッドからの直接呼び出しで競合しない (yield なし)。
// sink push (svc CALL_METHOD) も同スレッド上で同期実行される。
void bme280_on_ground(uint32_t press_pa, int16_t temp_cc) {
  ground_hpa = press_pa / 100.0f;
  ground_temp_c = temp_cc / 100.0f;
  g_ground_ready = true;
  printf("[BME280] ground pressure = %lu Pa\n", (unsigned long)press_pa);
}

void bme280_on_baro(uint32_t press_pa, int16_t temp_cc, uint32_t t_us) {
  (void)t_us;
  g_last_baro_us = time_us_64();
  if (!g_ground_ready)
    return; // 起動時較正が終わるまで非公開 (従来挙動と同じ)
  float t = temp_cc / 100.0f;
  float p = press_pa / 100.0f; // Pa → hPa
  bme280_sample_t s;
  s.seq = latest.seq + 1;
  s.temp_c = t;
  s.press_hpa = p;
  s.alt_m = pressure_to_altitude(p, t, ground_hpa);
  s.valid = true;
  latest = s; // 構造体ごと差し替え (yield しないので原子的)
  if (sample_sink_obj != 0xFFFF)
    obj_api::svc(obj_api::svc_num::CALL_METHOD, sample_sink_obj,
                 sample_sink_method, (uint32_t)(uintptr_t)&latest);
}

// ===========================================================================
//  公開メソッド (ID/セマンティクスは従来どおり)
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

// arg0 = (obj<<16)|method。新サンプル push 先を登録。
static void method_set_sample_sink(uint32_t, uint32_t, uint32_t packed,
                                   uint32_t) {
  sample_sink_obj = (packed >> 16) & 0xFFFF;
  sample_sink_method = packed & 0xFFFF;
}

// arg0: 非0=サンプリング一時停止 / 0=再開 (core1 へ転送)。
static void method_set_paused(uint32_t, uint32_t, uint32_t on, uint32_t) {
  core_ring::cmd_rec_t c = {};
  c.op = core_ring::CMD_SET_PAUSED_BME;
  c.arg = (on != 0);
  core_ring::g_cmd_ring.push(c);
}

// 地上気圧の再較正 (打ち上げ直前のゼロ点合わせ)。core1 が次の 20 サンプルを
// 整数 Pa で平均し CH_GROUND で返す (~1 秒後に反映される非同期動作)。
static void method_rezero(uint32_t _a, uint32_t _b, uint32_t _c, uint32_t _d) {
  (void)_a;
  (void)_b;
  (void)_c;
  (void)_d;
  core_ring::cmd_rec_t c = {};
  c.op = core_ring::CMD_REZERO;
  core_ring::g_cmd_ring.push(c);
  printf("[BME280] rezero requested (async, ~1s)\n");
}

// ===========================================================================
//  オブジェクトエントリ
// ===========================================================================
void BME280_DRIVER::init() {
  printf("[BME280] init (core1 stream, conversion on core0)\n");
  export_method<method_read_latest>(BME280_DRIVER::METHOD_IDs::read_latest);
  export_method<method_rezero>(BME280_DRIVER::METHOD_IDs::rezero);
  export_method<method_set_sample_sink>(
      BME280_DRIVER::METHOD_IDs::set_sample_sink);
  export_method<method_set_paused>(BME280_DRIVER::METHOD_IDs::set_paused);

  // データはリング排出スレッド (BNO055_DRIVER) 経由で届くので、本スレッドは
  // 鮮度監視だけを行う (BARO が 500ms 途絶えたら valid=false)。
  while (true) {
    uint64_t now = time_us_64();
    if (latest.valid && now - g_last_baro_us > 500000)
      latest.valid = false;
    obj_api::yield_us(100000);
  }
}

} // namespace shizu
