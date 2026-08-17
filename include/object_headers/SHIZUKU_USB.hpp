#ifndef SHIZUKU_USB_HPP
#define SHIZUKU_USB_HPP
#include <cstdint>

namespace shizu {

// ===========================================================================
//  SHIZUKU_USB — USB CDC への printf を「待たない」経路にするドライバ
// ===========================================================================
//  実測 (2026-08-07): BLE 接続時の秒級フリーズの正体は cyw43 ではなく printf。
//  SDK の pico_stdio_usb は out_chars の中で
//    ・pico_stdio の print_mutex を PICO_STDIO_DEADLOCK_TIMEOUT_MS (1000ms) 待ち
//    ・CDC バッファ満杯なら PICO_STDIO_USB_STDOUT_TIMEOUT_US (500ms) まで
//      tud_task() をビジー回し (yield しない)
//  という「呼び出し元スレッドで待つ」設計になっている。ドライバの printf の大半は
//  btstack コールバック = cyw43_arch_poll() の中で出るため、これが「1 秒の LONG
//  poll」に化けていた (poll ever が常に ~1.005s = 1000ms 定数そのもの)。
//
//  ここでは待ちをスケジューラ管轄へ戻す (yield_us 絶対グリッドと同じ思想):
//    ・printf (out_chars) は **リングへ積んで即 return**。どのスレッド/コア/IRQ から
//      呼んでも待たない。溢れたら捨てて欠落バイト数を数える。
//    ・排出はこのオブジェクトの専用スレッドが行い、**CDC の空き分だけ**を SDK の
//      stdio_usb.out_chars へ渡す → SDK 側のブロッキング分岐に構造的に入らない。
//      空き無し/リング空なら yield してスケジューラへ返す。
//  tinyusb は据え置き (tud_task は SDK の低優先度 IRQ が回し続ける = 干渉しない)。
//  詳細は SHIZUKU_USB.cpp 冒頭。
class SHIZUKU_USB {
public:
  // 排出スレッドの entry (async_call でこのオブジェクトのスレッドとして起動する)。
  static void main();
};

// ---- 起動前に呼ぶフック (main.cpp) ----------------------------------------
// stdio_init_all() の直後に呼ぶこと。自前 stdio ドライバを登録し SDK の stdio_usb を
// stdio チェインから外す。この時点から printf は非ブロッキングになる (排出スレッドが
// 走り出すまではリングに溜まるだけ)。
void shizuku_usb_install();

// リングに溜まったものを **その場で** (ブロッキング許容で) 吐き出す。排出スレッドが
// 走れない文脈 — panic / boot report / 最終手段 — 専用。RT 保護より可視性を優先する。
void shizuku_usb_emergency_drain();

// 診断: 溢れて捨てた累積バイト数 (排出スレッドが復帰時に 1 行報告する)。
uint32_t shizuku_usb_dropped();

// 診断: リングに溜まったまま出せていないバイト数。**排出スレッドが生きているかの
// 唯一の外部指標**。系が固まると printf はリングに積まれるだけで表に出ないので
// (panic すら消える)、ビーコン IRQ がこれを見て詰まりを検出する。
uint32_t shizuku_usb_ring_used();

// ---- log.hpp のシンク実体 ---------------------------------------------------
// リングへ直接積む (待たない)。**pico_stdio を通らない** = print_mutex の
// 1000ms WFE スピンを踏まないので、生 printf より速く RT に安全。log::sink::USB の実体。
void shizuku_usb_push(const char *buf, uint32_t len);
// ブロッキングで直に吐く。log::sink::PRINTK の実体 — スケジューラを疑うときや
// ブート最初期の「何があっても出したい」用。RT を壊すので通常運転では使わない。
void shizuku_usb_printk(const char *buf, uint32_t len);

} // namespace shizu
#endif // SHIZUKU_USB_HPP
