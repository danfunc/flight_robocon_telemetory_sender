#include <call_method.hpp>
#include <cstdint>
#include <hardware/pwm.h>
#include <obj_api.hpp>
#include <object_headers/BLE_UART_DRIVER.hpp>
#include <object_headers/BME280_DRIVER.hpp>
#include <object_headers/BNO055_DRIVER.hpp>
#include <object_headers/CYW43_BL_DRIVER.hpp>
#include <object_headers/FLIGHT_CONTROLLER.hpp>
#include <object_headers/HELLO_WORLD.hpp>
#include <object_headers/IO_CONTROLLER.hpp>
#include <object_headers/LINE_SENSOR_DRIVER.hpp>
#include <object_headers/TELEMETRY_SENDER.hpp>
#include <object_headers/WS2812_DRIVER.hpp>
#include <object_id.hpp>
#include <pico/stdlib.h>
#include <stdio.h>

// オブジェクト起動 = create_object(id) + async_call(id, entry) の 2 段。thread 番号は
// async_call が空きスロットを自動確保する。初回実行順は「スレッド番号の昇順」= この
// async_call を呼んだ順なので、依存先 (BLE, BNO055, FLIGHT) を先に async_call すること
// が本質 (手番の割り当てではなく呼び出し順で担保される)。
// 戻り値 = 割り当てられた thread_id (async_call の r1 戻り)。
static uint32_t start_object(object_ids id, uintptr_t entry) {
  shizu::obj_api::create_object((uint32_t)id);
  return shizu::obj_api::async_call((uint32_t)id, entry).value;
}

void shizu::IO_CONTROLLER::main() {
  printf("IO_CONTROLLER::init\n");
  start_object(object_ids::WS2812_DRIVER, (uintptr_t)WS2812_DRIVER::init);

  // CYW43 の無線は排他利用のため、CYW43_BL_DRIVER (プロコン HID) と
  // BLE_UART_DRIVER は同時起動できない。ここでは BLE UART を起動する。
  // プロコン制御に戻す場合は下を入れ替える。
  // start_object(object_ids::CYW43_BL_DRIVER, (uintptr_t)CYW43_BL_DRIVER::init);
  // BLE I/O は budget 除外 (無制限バトン組)。理由: cyw43/btstack は init や暗号化で
  // 3ms を正当に超える上、malloc/ロックを内部で使うため期限スライスできない。
  // 「BLE 以外は 3ms 以上 CPU をロックしない」ポリシーの唯一の例外。
#if !SHIZU_NO_BLE
#if SHIZU_BLE_ON_CORE1
  // 実験: BLE を core1 で走らせる。cyw43/btstack/async_context のコア縛りは
  // 「init を実行したコアと同じコアからしか触れない」なので、init〜poll ループ全体が
  // core1 ピン留めスレッドに載れば整合する (CYW43 の GPIO IRQ / async_context の
  // owner も core1 に付く)。affinity=CORE1 と budget0 (バトン組) は生成時に確定
  // (READY 公開前) — 「作ってから set_*」の隙間でこのスレッドが core0 に claim され
  // cyw43_arch_init が core0 で始まる事故を構造的に塞ぐ。
  shizu::obj_api::create_object((uint32_t)object_ids::BLE_UART_DRIVER);
  shizu::obj_api::async_call((uint32_t)object_ids::BLE_UART_DRIVER,
                             (uintptr_t)BLE_UART_DRIVER::init, 0,
                             shizu::AFFINITY_CORE1, /*budget0=*/true);
#else
  const uint32_t ble_tid =
      start_object(object_ids::BLE_UART_DRIVER, (uintptr_t)BLE_UART_DRIVER::init);
  shizu::obj_api::set_thread_budget(ble_tid, 0);
#endif
#else
  // Pico 2 (無印) テストビルド: BLE 無し。TELEMETRY の BLE_UART への call_method は
  // 対象未生成 = call_error::BAD_OBJECT で無害に返り (METHOD_CALL の型付きエラー化)、
  // TX ストリームは LOSSLESS 満杯 → メッセージ丸ごと破棄カウントで詰まらない。
  printf("[IO] SHIZU_NO_BLE build: BLE_UART_DRIVER not started\n");
#endif

  // 飛行テレメトリ構成: BME280/BNO055 (I2C センサ) → TELEMETRY_SENDER が購読して融合し
  // BLE_UART 経由で送信。BLE_UART を先に、TELEMETRY を最後に async_call する。
  start_object(object_ids::BME280_DRIVER, (uintptr_t)BME280_DRIVER::init);
  start_object(object_ids::BNO055_DRIVER, (uintptr_t)BNO055_DRIVER::init);
  // 飛行制御は TELEMETRY より先に async_call し、on_state/read_control の export を
  // 済ませてから TELEMETRY の最初の call_method を受けられるようにする。
#if SHIZU_STEP1_UNPRIV_FLIGHT_CONTROLLER
  // MPU Step1 プロトタイプ (HANDOFF §6, 実機未検証): FLIGHT_CONTROLLER を
  // unprivileged で起動する。set_object_unprivileged は create_object の直後・
  // async_call (= create_thread が context->control を確定する) より前に呼ぶこと
  // (kernel.hpp の SET_OBJECT_UNPRIVILEGED コメント参照)。
  shizu::obj_api::create_object((uint32_t)object_ids::FLIGHT_CONTROLLER);
  shizu::obj_api::set_object_unprivileged((uint32_t)object_ids::FLIGHT_CONTROLLER);
  shizu::obj_api::async_call((uint32_t)object_ids::FLIGHT_CONTROLLER,
                             (uintptr_t)FLIGHT_CONTROLLER::main);
#else
  start_object(object_ids::FLIGHT_CONTROLLER, (uintptr_t)FLIGHT_CONTROLLER::main);
#endif
  start_object(object_ids::TELEMETRY_SENDER, (uintptr_t)TELEMETRY_SENDER::main);

  // BLE スループット/RSSI ベンチ用の HELLO_WORLD を使う場合は上の TELEMETRY を外して:
  // start_object(object_ids::HELLO_WORLD, (uintptr_t)HELLO_WORLD::main);

  // IO_CONTROLLER 自身は以降ハートビートを刻むだけ (BLE 送信は TELEMETRY が担う)。
  while (true) {
    obj_api::yield_us(1000000);
  }
}
