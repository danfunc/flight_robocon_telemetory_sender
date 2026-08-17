// ===========================================================================
//  BME280_DRIVER — 気圧/温度センサの Shizuku オブジェクト (core0 側: 換算係)
// ===========================================================================
//  step2 (docs/sensor_stream_protocol.md §7) で I2C アクセスと整数補償演算
//  (compensate_T/P) は core1 (core1_io.cpp) へ移った。本オブジェクトは
//  I2C を一切触らず、
//    - ID_BARO_RAW の consumer として整数値 (Pa / 0.01℃) を受け取り、float 換算 +
//      測高公式 (hypsometric) で bme280_sample_t を作って ID_BME_SAMPLE へ流す
//    - rezero は g_cmd_stream で core1 へ転送し、結果は is_ground=1 のレコードで返る
//  旧 bme280_on_baro / bme280_on_ground (BNO055 のスレッドから直接呼ばれる関数) と
//  set_sample_sink (CALL_METHOD push) は廃止。前者は他オブジェクトのスレッドで自分の
//  状態を書き換える経路、後者は consumer を producer のスタックで走らせる経路で、
//  どちらも「協調スケジューラだから競合しない」に依存していた。
//  外部インタフェースのうち read_latest / rezero / set_paused は従来と互換。
// ===========================================================================
#include <cmath> // 標準 libm の powf (測高公式は core0 に残る)
#include <core_ring.hpp>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <driver_streams.hpp>
#include <export_method.hpp>
#include <obj_api.hpp>
#include <object_headers/BME280_DRIVER.hpp>
#include <pico/time.h>
#include <log.hpp>

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
// 値初期化 (valid=false もゼロ初期化で入る)。要素を並べた集約初期化にすると
// bme280_sample_t にフィールドが増えるたびに壊れるので {} で受ける。
static bme280_sample_t latest = {};

// 出力ストリーム (ID_BME_SAMPLE)。consumer は TELEMETRY_SENDER。実レート ~21Hz で
// consumer の送信周期の方が遅いこともあるが、高度は最新が要る値なので LOSSY。
static stream::storage<bme280_sample_t, 16> g_bme_tx;

// 鮮度監視 (BARO レコードが 500ms 途絶えたら valid=false。I2C 断相当)。
static uint64_t g_last_baro_us = 0;

// 気圧 → 高度 (測高公式)。温度は「地上較正時 T0 と現在 T の平均」= 気層平均。
static float pressure_to_altitude(float press_hpa, float temp_c, float p0_hpa) {
  float t_layer_c = 0.5f * (temp_c + ground_temp_c);
  return ((t_layer_c + 273.15f) / 0.0065f) *
         (powf(p0_hpa / press_hpa, 0.190234f) - 1.0f);
}

// ===========================================================================
//  ID_BARO_RAW の排出 (本オブジェクト自身のスレッドで走る)
// ===========================================================================
static void on_ground(uint32_t press_pa, int16_t temp_cc) {
  ground_hpa = press_pa / 100.0f;
  ground_temp_c = temp_cc / 100.0f;
  g_ground_ready = true;
  log::printf("[BME280] ground pressure = %lu Pa\n", (unsigned long)press_pa);
}

static void on_baro(uint32_t press_pa, int16_t temp_cc, uint32_t t_us) {
  g_last_baro_us = time_us_64();
  if (!g_ground_ready)
    return; // 起動時較正が終わるまで非公開 (従来挙動と同じ)
  float t = temp_cc / 100.0f;
  float p = press_pa / 100.0f; // Pa → hPa
  bme280_sample_t s;
  s.seq = latest.seq + 1;
  s.t_us = t_us; // core1 が付けた標本時刻 (consumer の v_baro dt はこれの差分)
  s.temp_c = t;
  s.press_hpa = p;
  s.alt_m = pressure_to_altitude(p, t, ground_hpa);
  s.valid = true;
  latest = s; // read_latest 用のスナップショット (書くのは本スレッドだけ)
  g_bme_tx.hdl().push(latest); // LOSSY: 満杯なら最古を上書き
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

// arg0: 非0=サンプリング一時停止 / 0=再開 (core1 へ転送)。
static void method_set_paused(uint32_t, uint32_t, uint32_t on, uint32_t) {
  core_ring::cmd_rec_t c = {};
  c.op = core_ring::CMD_SET_PAUSED_BME;
  c.arg = (on != 0);
  core_ring::g_cmd_stream.hdl().push_mp(c); // MP_PROD (BNO055 と共有)
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
  core_ring::g_cmd_stream.hdl().push_mp(c); // MP_PROD (BNO055 と共有)
  log::printf("[BME280] rezero requested (async, ~1s)\n");
}

// ===========================================================================
//  オブジェクトエントリ
// ===========================================================================
void BME280_DRIVER::init() {
  log::printf("[BME280] init (core1 stream, conversion on core0)\n");
  export_method<method_read_latest>(BME280_DRIVER::METHOD_IDs::read_latest);
  export_method<method_rezero>(BME280_DRIVER::METHOD_IDs::rezero);
  export_method<method_set_paused>(BME280_DRIVER::METHOD_IDs::set_paused);

  // 出力を先に登録してから入力を待つ。本オブジェクトは BNO055 より先に起動する
  // (IO_CONTROLLER の async_call 順) ので、ID_BARO_RAW はまだ存在しない — open_wait
  // が登録されるまで yield して待つので、この順序でも正しく繋がる。
  drv_stream::publish(drv_stream::ID_BME_SAMPLE, &g_bme_tx.desc, "BME280");
  auto rx = drv_stream::subscribe<drv_stream::baro_raw_t>(
      drv_stream::ID_BARO_RAW, "BME280");

  // baro の実レートは ~21Hz。50Hz の絶対グリッドで drain し、「BNO055 の demux →
  // 高度換算 → TELEMETRY」に足す遅延を 20ms 以内に抑える (旧実装は BNO スレッドから
  // の同期呼び出しで遅延 0 だった分の埋め合わせ)。
  uint64_t next = time_us_64();
  while (true) {
    next += 20000;

    if (rx.valid()) {
      drv_stream::baro_raw_t b;
      while (rx.pop(&b)) {
        if (b.is_ground)
          on_ground(b.press_pa, b.temp_cc);
        else
          on_baro(b.press_pa, b.temp_cc, b.t_us);
      }
    }

    // 鮮度監視 (BARO が 500ms 途絶えたら valid=false。I2C 断相当)。
    uint64_t now = time_us_64();
    if (latest.valid && now - g_last_baro_us > 500000)
      latest.valid = false;

    if ((int64_t)(next - now) < 0) {
      next = now;
      obj_api::yield(); // 周期超過でも必ず 1 回は譲る (無 yield 凍結の防止)
    } else
      obj_api::yield_until_us(next);
  }
}

} // namespace shizu
