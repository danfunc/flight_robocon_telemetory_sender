// XNO 側 (このリポジトリ) の bring-up ファームウェア。
#include "blink.hpp"
#include "bme280.hpp"
#include "bno055.hpp"
#include "flash_fs.hpp"
#include "flight_controller.hpp"
#include "logger.hpp"
#include "object_ids.hpp"
#include "ota.hpp"
#include "pico/stdlib.h"
#include "shizuku/app_entry.hpp"
#include "shizuku/kernel.hpp"
#include "shizuku/kernel_object.hpp"
#include "shizuku/object_api.hpp"
#include "shizuku/objects/ble_uart.hpp"
#include "shizuku/objects/gdb_stub.hpp"
#include "shizuku/objects/peripherals.hpp"
#include "shizuku/objects/usb_cdc.hpp"
#include "shizuku_loader.hpp"
#include "shizuku_shell.hpp"
#include "telemetry.hpp"
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

  // ---- Flash FS & 動的オブジェクトローダー & 管理シェルの初期化 ----
  xno::fs::init();
  xno::loader::init();
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
  ota::register_ota();
  blink::register_blink(xno_object_id::blink);
  shizuku::KERNEL::BOARD::diag_printf("[XNO BOOT] objects registered\n");

  // ---- 2) 配線 -------------------------------------------------------------
  {
    using shizuku::object_api;
    const auto call = [](uintptr_t object, uintptr_t m, uintptr_t a) {
      return shizuku::KERNEL::ARCH::syscall((uintptr_t)object_api::CALL_METHOD,
                                            object, m, a);
    };
    // センサ → telemetry
    shizuku::KERNEL::ARCH::syscall_result bno_stream{}, bme_stream{};
    if (XNO_ENABLE_SENSORS) {
      bno_stream =
          call(bno055::OBJECT, (uintptr_t)bno055::method::GET_STREAM, 0);
      bme_stream =
          call(bme280::OBJECT, (uintptr_t)bme280::method::GET_STREAM, 0);
      call(telemetry::OBJECT, (uintptr_t)telemetry::method::ADD_INPUT,
           bno_stream.value);
      call(telemetry::OBJECT, (uintptr_t)telemetry::method::ADD_INPUT,
           bme_stream.value);
      // 自動操縦アルゴリズムへセンサストリームを接続
      call(flight_controller::OBJECT,
           (uintptr_t)flight_controller::method::ADD_SENSOR_STREAM,
           bno_stream.value);
      call(flight_controller::OBJECT,
           (uintptr_t)flight_controller::method::ADD_SENSOR_STREAM,
           bme_stream.value);
    }

    // telemetry / flight_controller → logger
    const auto tele_stream =
        call(telemetry::OBJECT, (uintptr_t)telemetry::method::GET_STREAM, 0);
    const auto fc_stream =
        call(flight_controller::OBJECT,
             (uintptr_t)flight_controller::method::GET_STREAM, 0);
    call(logger::OBJECT, (uintptr_t)logger::method::ADD_INPUT,
         logger::pack_input(tele_stream.value, logger::PRIO_BULK));
    call(logger::OBJECT, (uintptr_t)logger::method::ADD_INPUT,
         logger::pack_input(fc_stream.value, logger::PRIO_CONTROL));

    // logger → ble_uart (NUS TX notify)
    const auto log_stream =
        call(logger::OBJECT, (uintptr_t)logger::method::GET_STREAM, 0);
    call(xno_object_id::ble_uart,
         (uintptr_t)shizuku::objects::ble_uart::method::SET_TX_STREAM,
         log_stream.value);

    // ble_uart RX (NUS write) → flight_controller
    const auto rx_stream =
        call(xno_object_id::ble_uart,
             (uintptr_t)shizuku::objects::ble_uart::method::GET_RX_STREAM, 0);
    call(flight_controller::OBJECT,
         (uintptr_t)flight_controller::method::SET_RX_STREAM, rx_stream.value);

    // ota → logger
    const auto ota_stream =
        call(ota::OBJECT, (uintptr_t)ota::method::GET_STREAM, 0);
    call(logger::OBJECT, (uintptr_t)logger::method::ADD_INPUT,
         logger::pack_input(ota_stream.value, logger::PRIO_CONTROL));

    // ble_uart OTA write → ota
    const auto ota_rx_stream =
        call(xno_object_id::ble_uart,
             (uintptr_t)shizuku::objects::ble_uart::method::GET_OTA_STREAM, 0);
    call(ota::OBJECT, (uintptr_t)ota::method::SET_INPUT_STREAM,
         ota_rx_stream.value);

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
  shizuku::objects::ble_uart::start_ble_uart(xno_object_id::ble_uart);
  logger::start_logger();
  ota::start_ota();
  telemetry::start_telemetry();
  flight_controller::start_flight_controller();
  if (XNO_ENABLE_SENSORS) {
    bno055::start_bno055();
    bme280::start_bme280();
  }

  // 管理シェルを Core 0 で起動 (対話型 CLI)
  xno::shell::start_shell();

  // Flash FS 上の自動起動オブジェクトを実行 (例: /bin/blink.bin)
  xno::loader::run_autorun();

  shizuku::KERNEL::BOARD::diag_printf("[XNO BOOT] all threads started\n");
  shizuku::KERNEL::BOARD::diag_printf(
      "[XNO BOOT] flight_robocon xno bring-up ok\n");

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
      shizuku::KERNEL_OBJECT::handler_entry());
  const auto boot = shizuku::kernel_object_instance.lend_boot_stack();
  shizuku::kernel_instance.bootstrap(shizuku::app_entry, boot.base, boot.bytes);
}
