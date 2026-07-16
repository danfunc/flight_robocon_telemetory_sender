#ifndef SHIZU_PANIC_RING_HPP
#define SHIZU_PANIC_RING_HPP
// ===========================================================================
//  panic リング — per-core の noinit RAM に panic メッセージを退避する
// ===========================================================================
//  背景: pico SDK の panic() は stdio (mutex + USB CDC) で印字するため、
//  例外コンテキストや core1 からはメッセージが黙って失われる (実障害: core1 の
//  panic が「r1 のレジスタ残骸から推理する」羽目になった)。
//  CMakeLists の PICO_PANIC_FUNCTION=shizu_panic により、panic() は stdio を
//  一切通らず shizu_panic() へ直行する。shizu_panic はロックを取らず、自コアの
//  スロットへ整形済みメッセージ + コンテキスト (コア/例外番号/スレッド/時刻) を
//  書いて戻る (戻った先で SDK が bkpt ループ → 既存の HardFault レジスタダンプは
//  従来どおり出る)。
//  読み出しは 2 経路:
//   ・main() 起動時: 前回実行の残骸 (リセットを跨いで残る) を印字してクリア。
//   ・ビーコン (core0 タイマ IRQ): 稼働中に新しい panic を見つけたら印字
//     (core1 が死んでも core0 の IRQ は生きているので必ず届く)。
// ===========================================================================
#include <cstdint>

namespace shizu {

struct panic_slot_t {
  uint32_t magic; // PANIC_MAGIC なら有効
  uint32_t seq;   // 書き込みごとに増える (ビーコンの「新規」判定用)
  uint32_t core;
  uint32_t exception; // IPSR (0=スレッドモード, 11=SVCall, 14=PendSV, 15=SysTick)
  uint32_t thread_id; // panic 時点の current_thread_id[core]
  uint64_t time_us;
  char msg[120];
};
constexpr uint32_t PANIC_MAGIC = 0x50414E49; // "PANI"

extern panic_slot_t panic_slots[2]; // per-core (noinit RAM, kernel.cpp で定義)

// 起動時: 前回実行の panic 残骸があれば印字してクリア (main() から呼ぶ)。
void panic_ring_boot_report();
// 稼働中: 未報告の新しい panic があれば印字 (ビーコンから呼ぶ)。
void panic_ring_poll_report();

} // namespace shizu

// panic() の差し替え先 (PICO_PANIC_FUNCTION)。ロックフリーで自コアスロットへ記録。
extern "C" void shizu_panic(const char *fmt, ...);

#endif // SHIZU_PANIC_RING_HPP
