// XNO 側 (このリポジトリ) の bring-up ファームウェア。
#include "blink.hpp"
#include "bme280.hpp"
#include "bno055.hpp"
#include "flight_controller.hpp"
#include "logger.hpp"
#include "neopixel.hpp"
#include "object_ids.hpp"
#include "pico/stdlib.h"
#include "shizuku/app_entry.hpp"
#include "shizuku/kernel.hpp"
#include "shizuku/kernel_object.hpp"
#include "shizuku/object_api.hpp"
#include "shizuku/objects/ble_uart.hpp"
#include "shizuku/objects/flash_fs.hpp"
#include "shizuku/objects/gdb_stub.hpp"
#include "shizuku/objects/ota.hpp"
#include "shizuku/objects/peripherals.hpp"
#include "shizuku/objects/usb_cdc.hpp"
#include "shizuku_shell.hpp"
#include "telemetry.hpp"
#include "tx_frame.hpp"
#include <cstdio>

// センサ有効化フラグ
constexpr bool XNO_ENABLE_SENSORS = true;

void shizuku::app_entry() {
  shizuku::KERNEL::BOARD::diag_printf("[XNO BOOT] app_entry start\n");
  shizuku::kernel_instance.set_recovery_thread(0);

  // Core 1 を起こす
  if (shizuku::kernel_object_instance.start_secondary_core())
    shizuku::KERNEL::BOARD::diag_printf("[XNO BOOT] secondary core launched\n");

  shizuku::objects::register_peripherals();
  shizuku::KERNEL::BOARD::diag_printf("[XNO BOOT] peripherals registered\n");

  // ---- flash FS (プロパティの置き場) --------------------------------------
  //  ★**ここより前には置けない**。目録が空なら mount が初期化を書きに行き、
  //    その書き込みは flash_safe_execute (= 他コアを止める) を通る。止められる
  //    側の core1 が起きていないと rc=-4 で無音失敗するので、
  //    start_secondary_core() より後でなければならない。
  //  ★領域は flash_map.hpp が決めている (btstack の bonding バンクとは
  //    重ならないことを static_assert が確かめてある)。
  if (shizuku::objects::register_flash_fs() != 0)
    shizuku::KERNEL::BOARD::diag_printf(
        "[XNO BOOT] flash fs unavailable (プロパティは既定値で動く)\n");

  // ---- 管理シェルの初期化 ----
  const bool neopixel_registered = xno::neopixel::register_neopixel() == 0;
  if (!neopixel_registered)
    shizuku::KERNEL::BOARD::diag_printf("[XNO BOOT] neopixel registration failed\n");
  xno::shell::register_shell();

  // ---- 1) 登録 (生成 + export + 自分のストリーム作成) ----------------------
  shizuku::objects::ble_uart::register_ble_uart(xno_object_id::ble_uart);
  if (XNO_ENABLE_SENSORS) {
    bno055::register_bno055();
    bme280::register_bme280();
  }
  telemetry::register_telemetry();
  flight_controller::register_flight_controller();
  logger::register_logger();
  shizuku::objects::ota::register_ota(xno_object_id::ota, xno_object_id::blink,
                                      xno_object_id::ble_uart);
  blink::register_blink(xno_object_id::blink);
  shizuku::KERNEL::BOARD::diag_printf("[XNO BOOT] objects registered\n");

  // ---- 2) 配線 -------------------------------------------------------------
  {
    using shizuku::object_api;
    const auto call = [](uintptr_t object, uintptr_t m, uintptr_t a) {
      return shizuku::KERNEL::ARCH::syscall((uintptr_t)object_api::CALL_METHOD,
                                            object, m, a);
    };
    auto sid = [](const auto &res) -> uintptr_t {
      return res.value != 0 ? res.value : res.error;
    };

    // shell UART0 -> NeoPixel. Use error/value separately, never an error as ID.
    if (neopixel_registered) {
      const auto safety_stream = call(xno::shell::OBJECT,
          (uintptr_t)xno::shell::method::GET_SAFETY_STREAM, 0);
      if (safety_stream.error == 0 && safety_stream.value != xno::NO_STREAM) {
        const auto wired = call(xno::neopixel::OBJECT,
            (uintptr_t)xno::neopixel::method::SET_INPUT_STREAM,
            safety_stream.value);
        if (wired.error != 0 || wired.value != 0)
          shizuku::KERNEL::BOARD::diag_printf("[XNO BOOT] neopixel wiring failed\n");
      } else {
        shizuku::KERNEL::BOARD::diag_printf("[XNO BOOT] safety stream unavailable\n");
      }
    }

    // センサ → telemetry
    shizuku::KERNEL::ARCH::syscall_result bno_stream{}, bme_stream{};
    if (XNO_ENABLE_SENSORS) {
      bno_stream =
          call(bno055::OBJECT, (uintptr_t)bno055::method::GET_STREAM, 0);
      bme_stream =
          call(bme280::OBJECT, (uintptr_t)bme280::method::GET_STREAM, 0);
      call(telemetry::OBJECT, (uintptr_t)telemetry::method::ADD_INPUT,
           sid(bno_stream));
      call(telemetry::OBJECT, (uintptr_t)telemetry::method::ADD_INPUT,
           sid(bme_stream));
      // 自動操縦アルゴリズムへセンサストリームを接続
      call(flight_controller::OBJECT,
           (uintptr_t)flight_controller::method::ADD_SENSOR_STREAM,
           sid(bno_stream));
      call(flight_controller::OBJECT,
           (uintptr_t)flight_controller::method::ADD_SENSOR_STREAM,
           sid(bme_stream));
    }

    // telemetry / flight_controller / shizuku_shell → logger
    const auto tele_stream =
        call(telemetry::OBJECT, (uintptr_t)telemetry::method::GET_STREAM, 0);
    const auto fc_stream =
        call(flight_controller::OBJECT,
             (uintptr_t)flight_controller::method::GET_STREAM, 0);
    const auto shell_stream =
        call(xno::shell::OBJECT, (uintptr_t)xno::shell::method::GET_STREAM, 0);

    call(logger::OBJECT, (uintptr_t)logger::method::ADD_INPUT,
         logger::pack_input(sid(tele_stream), logger::PRIO_BULK));
    call(logger::OBJECT, (uintptr_t)logger::method::ADD_INPUT,
         logger::pack_input(sid(fc_stream), logger::PRIO_CONTROL));
    call(logger::OBJECT, (uintptr_t)logger::method::ADD_INPUT,
         logger::pack_input(sid(shell_stream), logger::PRIO_CONTROL));

    // logger → ble_uart (NUS TX notify)
    const auto log_stream =
        call(logger::OBJECT, (uintptr_t)logger::method::GET_STREAM, 0);
    call(xno_object_id::ble_uart,
         (uintptr_t)shizuku::objects::ble_uart::method::SET_TX_STREAM,
         sid(log_stream));

    // ble_uart RX (NUS write) → shizuku_shell
    const auto rx_stream =
        call(xno_object_id::ble_uart,
             (uintptr_t)shizuku::objects::ble_uart::method::GET_RX_STREAM, 0);
    call(xno::shell::OBJECT, (uintptr_t)xno::shell::method::SET_RX_STREAM,
         sid(rx_stream));

    // ota → logger
    const auto ota_stream =
        call(xno_object_id::ota,
             (uintptr_t)shizuku::objects::ota::method::GET_STREAM, 0);
    call(logger::OBJECT, (uintptr_t)logger::method::ADD_INPUT,
         logger::pack_input(sid(ota_stream), logger::PRIO_CONTROL));

    // ble_uart OTA write → ota
    const auto ota_rx_stream =
        call(xno_object_id::ble_uart,
             (uintptr_t)shizuku::objects::ble_uart::method::GET_OTA_STREAM, 0);
    call(xno_object_id::ota,
         (uintptr_t)shizuku::objects::ota::method::SET_INPUT_STREAM,
         sid(ota_rx_stream));
    // シェルへ「ブリッジ終了後に ota の入力を戻す先」を教える
    // (UART バイナリブリッジ機能。BLE の代替経路)
    call(xno::shell::OBJECT,
         (uintptr_t)xno::shell::method::SET_BLE_OTA_STREAM,
         sid(ota_rx_stream));

    // ---- 2.5) デバッグ対象 (blink) 起動 & GDB stub (★ble_uart 起動より前) ----
    blink::start_blink(xno_object_id::blink);
    const uint32_t blink_tid = blink::get_thread_id();
    const auto gdb_link =
        shizuku::objects::start_gdb_stub_over_stream(blink_tid);
    if (gdb_link.ok) {
      call(xno_object_id::ble_uart,
           (uintptr_t)shizuku::objects::ble_uart::method::SET_GDB_STREAMS,
           (gdb_link.to_stub << 16) | gdb_link.from_stub);
      shizuku::KERNEL::BOARD::diag_printf(
          "[XNO BOOT] gdb over BLE (streams %lu/%lu), watching blink "
          "(thread %lu)\n",
          (unsigned long)gdb_link.to_stub, (unsigned long)gdb_link.from_stub,
          (unsigned long)blink_tid);
    }
  }

  // ---- 3) 起動 (poll スレッド群 ＆ シェルの開始) ---------------------------
  // Consumer first: its initial dispatch precedes the shell producer.
  if (neopixel_registered && xno::neopixel::start_neopixel() != 0)
    shizuku::KERNEL::BOARD::diag_printf("[XNO BOOT] neopixel spawn failed\n");
  shizuku::objects::ble_uart::start_ble_uart(xno_object_id::ble_uart);
  logger::start_logger();
  shizuku::objects::ota::start_ota(xno_object_id::ota);
  telemetry::start_telemetry();
  flight_controller::start_flight_controller();

  if (XNO_ENABLE_SENSORS) {
    bno055::start_bno055();
    bme280::start_bme280();
  }

  // 管理シェルを Core 0 で起動 (BLE & CDC 両対応)
  xno::shell::start_shell();

  shizuku::KERNEL::BOARD::diag_printf("[XNO BOOT] all threads started\n");
  shizuku::KERNEL::BOARD::diag_printf("[XNO BOOT] flight_robocon xno bring-up "
                                      "v12.0 (CROSS-CORE LED SYNC VERIFIED)\n");

  while (true)
    shizuku::KERNEL::ARCH::syscall((uintptr_t)shizuku::object_api::YIELD);
}

int main() {
  shizuku::objects::usb_cdc_init();
  sleep_ms(1000);
  for (int i = 0; i < 5; ++i) {
    printf("[XNO] waiting for the host to open CDC (%d)\n", i);
    sleep_ms(200);
  }
  shizuku::kernel_instance.init();
  shizuku::kernel_object_instance.init();
  shizuku::kernel_instance.set_object_handler(
      shizuku::KERNEL_OBJECT::handler_entry(),
      (uint32_t)shizuku::KERNEL_OBJECT::KERNEL_OBJECT_ID);
  const auto boot = shizuku::kernel_object_instance.lend_boot_stack();
  shizuku::kernel_instance.bootstrap(shizuku::app_entry, boot.base, boot.bytes);
}
