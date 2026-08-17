#include <flash_clock.hpp>

#include <hardware/clocks.h>
#include <hardware/regs/addressmap.h>
#include <hardware/structs/qmi.h>
#include <hardware/sync.h>
#include <pico/stdlib.h>
#if SHIZU_FLASH_DRIFT_PROBE
#include <cstdio>
#include <hardware/flash.h>
#include <pico/flash.h>
#endif

// 測定で分かっている RXDELAY の窓 (tools/ のスイープ = SHIZU_XIP_BENCH の [XIP] 行、
// 2026-08-17 実測。RXDELAY は clk_sys の**半サイクル**単位)。
//
//   clk_sys 150MHz  div=2 (75MHz) : rx 1,2,3 = OK / rx 4,5 = 読み違い
//   clk_sys 150MHz  div=3 (50MHz) : rx 0..4  = OK
//   clk_sys 300MHz  div=3 (100MHz): rx 0 = 読み違い / rx 1..4 = OK
//   clk_sys 300MHz  div=2 (150MHz): rx 1 = 読み違い / rx 2..5 = OK (定格外)
//
// 速いほど下限が上がり (往復遅延が相対的に伸びる)、上限は「半サイクル数 × 周期」が
// 10ns 程度で頭打ちになる。rx=2 は上の 4 ケースすべてで窓の内側に入る唯一の値なので
// 既定にしてある。★クロックを変えたらスイープを回し直すこと。
namespace shizu {
namespace {

uint32_t g_applied_timing = 0;

// QMI の M0_TIMING を書き換える。**必ず RAM から**、割り込みを止めて呼ぶこと。
// 書いた直後に nocache 窓から 1 語読んで新しいタイミングを確定させる。
void __no_inline_not_in_flash_func(write_timing)(uint32_t timing) {
  qmi_hw->m[0].timing = timing;
  (void)*(const volatile uint32_t *)XIP_NOCACHE_NOALLOC_BASE;
  __compiler_memory_barrier();
}

} // namespace

uint32_t flash_clock_apply_for(uint32_t sys_hz, uint32_t target_hz,
                              uint32_t rxdelay) {
  if (!sys_hz || !target_hz)
    return flash_clock_hz();
  // target を**超えない**最小の分周比 = 切り上げ。1..255 にクランプ。
  uint32_t div = (sys_hz + target_hz - 1u) / target_hz;
  if (div < 1u)
    div = 1u;
  if (div > 255u)
    div = 255u;
  if (rxdelay > 7u)
    rxdelay = 7u;

  const uint32_t timing =
      (qmi_hw->m[0].timing &
       ~(QMI_M0_TIMING_CLKDIV_BITS | QMI_M0_TIMING_RXDELAY_BITS)) |
      (div << QMI_M0_TIMING_CLKDIV_LSB) |
      (rxdelay << QMI_M0_TIMING_RXDELAY_LSB);

  const uint32_t save = save_and_disable_interrupts();
  write_timing(timing);
  g_applied_timing = qmi_hw->m[0].timing; // 自己申告でなく読み戻す
  restore_interrupts(save);
  return sys_hz / div;
}

uint32_t flash_clock_apply(uint32_t target_hz, uint32_t rxdelay) {
  return flash_clock_apply_for(clock_get_hz(clk_sys), target_hz, rxdelay);
}

uint32_t flash_rxdelay_for(uint32_t sys_hz, uint32_t delay_ps) {
  if (!sys_hz)
    return 0;
  // 半サイクル [ps] = 1e12 / (2 * sys_hz)。段数 = delay_ps / 半サイクル
  //                                        = delay_ps * 2 * sys_hz / 1e12。四捨五入。
  const uint64_t num = (uint64_t)delay_ps * 2ull * (uint64_t)sys_hz;
  uint32_t steps = (uint32_t)((num + 500000000000ull) / 1000000000000ull);
  if (steps > 7u)
    steps = 7u; // RXDELAY は 3bit
  return steps;
}

uint32_t flash_clock_hz() {
  const uint32_t div =
      (qmi_hw->m[0].timing & QMI_M0_TIMING_CLKDIV_BITS) >>
      QMI_M0_TIMING_CLKDIV_LSB;
  return div ? clock_get_hz(clk_sys) / div : 0u;
}

bool flash_clock_drifted() {
  return g_applied_timing != 0u && qmi_hw->m[0].timing != g_applied_timing;
}

#if SHIZU_FLASH_DRIFT_PROBE
// 「フラッシュ操作の後にタイミングが ROM 既定へ戻るのか」を実測する。
//
// ★書き込まずに確かめられる。flash.c の `flash_do_cmd` は `flash_range_erase` と
//   **まったく同じ終わり方**をする:
//       flash_flush_cache() → flash_enable_xip_via_boot2() → flash_restore_hardware_state()
//   そして `flash_restore_hardware_state` が戻すのは QSPI パッドと **m[1] (CS1)** だけで、
//   **m[0] (フラッシュ本体) の timing は保存も復元もされない**。
//   なので `flash_get_unique_id()` (= flash_do_cmd 経由) を 1 回呼べば、消去も書き込みも
//   せずに同じ経路を踏める。BTstack のペアリング鍵保存が通るのもこの経路。
void flash_clock_drift_probe() {
  printf("[FLASHP] before      timing=%08lx flash=%lu Hz\n",
         (unsigned long)qmi_hw->m[0].timing, (unsigned long)flash_clock_hz());

  // JEDEC ID (0x9F) も読んでおく。定格 (133MHz か否か) の根拠を推測でなく実物にする。
  uint8_t tx[4] = {0x9f, 0, 0, 0}, rx[4] = {0, 0, 0, 0};
  uint8_t uid[FLASH_UNIQUE_ID_SIZE_BYTES] = {0};
  {
    // XIP が切れている間はフラッシュ上のコード (USB IRQ ハンドラ等) を踏めない。
    const uint32_t save = save_and_disable_interrupts();
    flash_do_cmd(tx, rx, sizeof(tx));
    flash_get_unique_id(uid);
    restore_interrupts(save);
  }
  printf("[FLASHP] jedec mfr=%02x type=%02x cap=%02x (size=%luKB) uid=%02x%02x%02x%02x%02x%02x%02x%02x\n",
         rx[1], rx[2], rx[3], (unsigned long)((1ul << rx[3]) / 1024ul),
         uid[0], uid[1], uid[2], uid[3], uid[4], uid[5], uid[6], uid[7]);
  printf("[FLASHP] after do_cmd timing=%08lx flash=%lu Hz drifted=%d\n",
         (unsigned long)qmi_hw->m[0].timing, (unsigned long)flash_clock_hz(),
         (int)flash_clock_drifted());
}

// 上のドリフトが**実運用で起きるのか**を決める第二の問い。
// BTstack のペアリング鍵保存 (btstack_flash_bank.c) は `flash_safe_execute` 経由で、
// これは core1 が走っていると `multicore_lockout_victim_is_initialized(1)` を要求する。
// このプロジェクトは `flash_safe_execute_core_init()` をどこからも呼んでいないので、
// SDK のコードを読む限り `PICO_ERROR_NOT_PERMITTED` (=-4) で**書き込み自体が
// 実行されない**はず。だとすると鍵は永続化されておらず、ドリフトも起きない。
// 読みで決めつけず、no-op を渡して戻り値を実測する (何も書かないので安全)。
// ★core1 が動き出した後に呼ぶこと — core1 停止中は判定が変わる。
void flash_clock_safe_exec_probe() {
  const int rc = flash_safe_execute(
      [](void *) { /* 何もしない。知りたいのは到達可否だけ */ }, nullptr, 500);
  printf("[FLASHP] flash_safe_execute(noop) rc=%d (%s)\n", rc,
         rc == PICO_OK ? "OK = フラッシュ書き込みは実行される"
                       : "NG = BTstack のペアリング鍵保存は実行されていない");
}
#endif

} // namespace shizu
