#ifndef SHIZU_LOG_HPP
#define SHIZU_LOG_HPP
#include <cstdarg>
#include <cstdint>

namespace shizu {
namespace log {

// ===========================================================================
//  ログシンクの選択 — 方針はオブジェクトの外に置く
// ===========================================================================
//  ドライバは log::printf() を呼ぶだけで、自分の出力先を**知らないし選べない**。
//  どのオブジェクトの出力をどこへ流すかは合成側 (IO_CONTROLLER = オブジェクトを
//  起動している場所) が set_sink() で決める。affinity / grant budget を async_call の
//  呼び出し側が決めるのと同じ形で、「機構はオブジェクト、方針は外」を保つ。
//
//  3 つのシンクは**保証が違う**。用途で選ぶこと:
//
//  | sink   | 待つか       | スケジューラ起動前 | IRQ/例外文脈 | 溢れたら      |
//  |--------|--------------|--------------------|--------------|---------------|
//  | USB    | 待たない     | 溜めて後から排出   | 可 (積むだけ)| 新しい方を破棄|
//  | BLE    | 待たない     | 不可 (接続が要る)  | 可 (積むだけ)| メッセージ破棄|
//  | PRINTK | **待つ**     | 可                 | 可           | 待って出す    |
//
//  ・USB    … SHIZUKU_USB のリングへ直接積む。**pico_stdio を通らない**ので
//             print_mutex (最大 1000ms の WFE スピン) を踏まない = 生 printf より速く安全。
//             通常運転のドライバはこれ。
//  ・BLE    … BLE_UART の TX へ回す。母艦でしか見られないが、USB を挿せない飛行中に
//             効く。接続前/未認可の間は捨てられる (fail-open でなく単に届かない)。
//  ・PRINTK … ブロッキングで直に吐く「最後の手段」。スケジューラ自体を疑うとき、
//             ブート最初期、panic 直前など。**RT を壊すので通常運転では使わない**。
//  ・NONE   … 捨てる。うるさいドライバを黙らせる (再ビルド不要で外から切れる)。
enum class sink : uint8_t {
  NONE = 0,
  USB = 1,
  BLE = 2,
  PRINTK = 3,
};

// ---- 方針 API (合成側だけが呼ぶ) -------------------------------------------
// obj_id のログ出力先を決める。未設定のオブジェクトは既定シンクに従う。
void set_sink(uint32_t obj_id, sink s);
// 未設定オブジェクトの既定 (初期値 USB)。
void set_default_sink(sink s);
// 全オブジェクト (0..127) を s に揃える。個別の set_sink はこの後に呼ぶこと。
void set_all_sinks(sink s);
// 現在の割り当てを引く (診断用)。
sink sink_of(uint32_t obj_id);

// ---- 出力 API (ドライバが呼ぶ) ---------------------------------------------
// 呼び出し元オブジェクトの割り当てシンクへ流す。ドライバ側に選択肢は無い。
void write(const char *buf, uint32_t len);
void printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
void vprintf(const char *fmt, va_list ap);

// シンクを明示指定する版。原則使わない (方針が外にある意味が薄れる) が、
// 「ここだけは何があっても出したい」= PRINTK を名指しする用途のために残す。
void printf_to(sink s, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

// 診断: シンク別に捨てた回数 (BLE 競合 / NONE / 溢れ)。
uint32_t dropped(sink s);

} // namespace log
} // namespace shizu
#endif // SHIZU_LOG_HPP
