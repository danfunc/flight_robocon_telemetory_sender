
#include "hardware/structs/scb.h"
#include <cstdio>
#include <cstdlib>
#include <hardware/exception.h>
#include <hardware/gpio.h>
#include <pico/stdio.h>
#include <pico/stdlib.h>
#include <pico/time.h>
#include <shizu.hpp>
#include <time.h>
#include <tusb.h> // HardFault 中に USB CDC を手動ポンプするため

void *get_psp() {
  void *psp;
  asm volatile("MRS %0, PSP" : "=r"(psp));
  return psp;
}
void *get_msp() {
  void *msp;
  asm volatile("MRS %0, MSP" : "=r"(msp));
  return msp;
}

void test_svc_handler() {
  printf("test_svc_handler\n");
  //  sleep_ms(100);
}

void hardfault_handler() {

  uint32_t volatile exc_lr;
  asm volatile("mov %[lr], lr" : [lr] "=r"(exc_lr));
  // EXC_RETURN bit2: 0=MSP, 1=PSP。例外発生時のスタックポインタを選び、
  // そこに積まれた例外フレーム {r0,r1,r2,r3,r12,lr,pc,xPSR} を読む。
  uint32_t *sf = (exc_lr & 0x4) ? (uint32_t *)get_psp() : (uint32_t *)get_msp();
  while (1) {
    printf("HardFault: exc_lr=%lx psp=%p msp=%p\n", exc_lr, get_psp(),
           get_msp());
    printf("  r0=%08lx r1=%08lx r2=%08lx r3=%08lx\n", sf[0], sf[1], sf[2],
           sf[3]);
    printf("  r12=%08lx lr=%08lx pc=%08lx xpsr=%08lx\n", sf[4], sf[5], sf[6],
           sf[7]);
    // HardFault は最高優先度なので stdio_usb の低優先度 IRQ (tud_task) が走れず、
    // printf がホストへ届かない=「無言フリーズ」に見える。ここで手動ポンプして
    // ダンプを届ける (busy_wait はタイマ IRQ 不要なので sleep_ms の代わりに使う)。
    for (int i = 0; i < 1000; ++i) {
      tud_task();
      busy_wait_us(1000);
    }
  }
}

// ---------------------------------------------------------------------------
//  生存ビーコン (フリーズ箇所の特定用デバッグ計装)
// ---------------------------------------------------------------------------
// 協調スケジューラでは 1 スレッドが yield せず回り続けると全スレッドが止まり、
// ログが完全に沈黙する。このビーコンはタイマ IRQ (スレッドとは独立に走る) から
// 5 秒ごとに「今 RUNNING のスレッド」と「割り込まれた地点の pc」を出す。
// フリーズ時に同じ pc が出続ければ、そのアドレスが無限ループの現場
// (arm-none-eabi-addr2line -e build/main.elf <pc> で行番号に変換できる)。
// 何も出なくなった場合は IRQ ごと止まる系 (HardFault 連鎖 / 割り込み禁止での
// ハング) と切り分けられる。
// poll/evt は BLE_UART_DRIVER の生存カウンタ: poll が進まない=BLE スレッド停止、
// poll は進むが evt が進まない=HCI イベント経路 (CYW43↔host) の停止、と読む。
namespace shizu {
extern volatile uint32_t ble_dbg_poll, ble_dbg_evt;
}
static bool beacon_cb(repeating_timer_t *) {
  uint32_t running_thread = 0xFFFFFFFF, running_obj = 0xFFFFFFFF;
  for (uint32_t i = 0; i < 128; ++i) {
    if (shizu::thread_table[i].state == shizu::thread_t::state_t::RUNNING) {
      running_thread = i;
      running_obj = shizu::thread_table[i].object_id;
      break;
    }
  }
  uint32_t psp;
  asm volatile("MRS %0, PSP" : "=r"(psp));
  const uint32_t *f = (const uint32_t *)psp; // 割り込まれたスレッドの例外フレーム
  printf("[BEACON] thr=%lu obj=%lu pc=%08lx lr=%08lx ble_poll=%lu evt=%lu\n",
         (unsigned long)running_thread, (unsigned long)running_obj, f[6], f[5],
         (unsigned long)shizu::ble_dbg_poll, (unsigned long)shizu::ble_dbg_evt);
  return true;
}

int main(int argc, char const *argv[]) {
  stdio_init_all();
  exception_set_exclusive_handler(HARDFAULT_EXCEPTION, hardfault_handler);
  sleep_ms(2000);
  // 生存ビーコン開始 (5s 周期、タイマ IRQ 駆動なのでスレッドが固まっても出続ける)。
  static repeating_timer_t beacon_timer;
  add_repeating_timer_ms(-5000, beacon_cb, nullptr, &beacon_timer);
  shizu::init();
  while (1) {
    sleep_ms(500);
    printf("no_return\n");
  }
  return 0;
}
