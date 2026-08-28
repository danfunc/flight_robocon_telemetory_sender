// ★測定専用: 「BLE 受信経路だけ」を RAM に載せられるかを見るための最小合成。
//   OTA (無線リフラッシュ) は flash 消去中 XIP が止まるので、受信に関わるコード・
//   const データ・ISR が**全部** RAM に居る必要がある。pico-sdk の copy_to_ram
//   バイナリタイプはそれを一括で満たすが、その分 text が RAM を食う。
//   ここでは「センサ/テレメトリ/GDB を落とした BLE だけの像」の text+bss を測り、
//   RP2350 の SRAM 520KB に対して現実的かを判断する材料にする。
#include "shizuku/objects/ble_uart.hpp"
#include "pico/stdlib.h"
#include "shizuku/app_entry.hpp"
#include "shizuku/kernel.hpp"
#include "shizuku/kernel_object.hpp"
#include "shizuku/object_api.hpp"
#include "shizuku/objects/peripherals.hpp"
#include "shizuku/objects/usb_cdc.hpp"

void shizuku::app_entry() {
  shizuku::kernel_instance.set_recovery_thread(0);
  shizuku::objects::register_peripherals();
  shizuku::objects::ble_uart::register_ble_uart(xno_object_id::ble_uart);
  shizuku::objects::ble_uart::start_ble_uart(xno_object_id::ble_uart);
  while (true)
    shizuku::KERNEL::ARCH::syscall((uintptr_t)shizuku::object_api::YIELD);
}

int main() {
  shizuku::objects::usb_cdc_init();
  shizuku::kernel_instance.init();
  shizuku::kernel_object_instance.init();
  shizuku::kernel_instance.set_object_handler(
      shizuku::KERNEL_OBJECT::handler_entry());
  const auto boot = shizuku::kernel_object_instance.lend_boot_stack();
  shizuku::kernel_instance.bootstrap(shizuku::app_entry, boot.base, boot.bytes);
}
