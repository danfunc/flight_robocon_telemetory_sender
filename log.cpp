// ===========================================================================
//  log — オブジェクト単位でログの出力先を切り替える (方針はオブジェクトの外)
// ===========================================================================
//  ドライバは log::printf() を呼ぶだけで自分の出力先を知らない。割り当ては合成側
//  (IO_CONTROLLER) が set_sink() で行う。「機構はオブジェクト、方針は外」の踏襲で、
//  affinity / grant budget を async_call 側が決めるのと同じ構図。
//
//  実装上の要点:
//   ・呼び出し元の識別は get_current_obj_id() (SVC)。**ドライバが名乗る余地が無い**
//     ので、外の割り当てを迂回できない。
//   ・整形はスタック上の固定バッファ。ヒープを踏まない (SVC 経路の malloc ゼロ方針)。
//   ・USB シンクは pico_stdio を通らず SHIZUKU_USB のリングへ直接積む → print_mutex の
//     WFE スピン (最大 1000ms) を構造的に回避する。生 printf より安全。
//   ・BLE シンクは BLE_UART の送信ストリームが SPSC なので、複数オブジェクトが同時に
//     流すと壊れる。非ブロッキングなトークンで直列化し、取れなければ**その行を捨てる**
//     (ログは落として良い。待つとログのために RT を壊すという本末転倒になる)。
// ===========================================================================
#include <call_method.hpp>
#include <cstdio>
#include <cstring>
#include <get_current_obj_id.hpp>
#include <log.hpp>
#include <object_headers/BLE_UART_DRIVER.hpp>
#include <object_headers/SHIZUKU_USB.hpp>
#include <object_id.hpp>

namespace shizu {
namespace log {
namespace {

// オブジェクト数は object_table と同じ 128 上限 (object_id.hpp のフラット空間)。
constexpr uint32_t MAX_OBJECTS = 128;
// 0 = 未設定 → 既定シンクに従う。sink::NONE を明示した場合と区別するため、
// テーブルは「未設定」を別値で持つ (UNSET)。
constexpr uint8_t UNSET = 0xFF;

uint8_t g_sink[MAX_OBJECTS] = {}; // 全要素 0 で初期化されるので下で UNSET へ直す
bool g_table_ready = false;
sink g_default = sink::USB;

uint32_t g_drop_ble = 0, g_drop_none = 0, g_drop_fmt = 0;

// BLE シンクの直列化トークン (SPSC ストリーム保護)。待たない: 取れなければ捨てる。
volatile uint32_t g_ble_busy = 0;

void ensure_table() {
  if (g_table_ready)
    return;
  for (uint32_t i = 0; i < MAX_OBJECTS; ++i)
    g_sink[i] = UNSET;
  g_table_ready = true;
}

// 1 行の整形バッファ。長い行は切り詰める (ログのために大きなスタックを積まない)。
constexpr uint32_t LINE_MAX = 256;

void emit(sink s, const char *buf, uint32_t len) {
  switch (s) {
  case sink::USB:
    shizuku_usb_push(buf, len);
    break;
  case sink::PRINTK:
    shizuku_usb_printk(buf, len);
    break;
  case sink::BLE: {
    // 非ブロッキングに占有を試す。競合したら捨てる (SPSC を壊さないことが優先)。
    uint32_t expected = 0;
    if (!__atomic_compare_exchange_n(&g_ble_busy, &expected, 1u, /*weak=*/false,
                                     __ATOMIC_ACQUIRE, __ATOMIC_RELAXED)) {
      ++g_drop_ble;
      return;
    }
    ble_tx_buf_t b{reinterpret_cast<const uint8_t *>(buf), len};
    // BLE 未起動/未接続なら BAD_OBJECT や内部破棄で黙って落ちる (ログの正しい挙動)。
    call_method(object_ids::BLE_UART_DRIVER,
                BLE_UART_DRIVER::METHOD_IDs::send_buf, (uint32_t)(uintptr_t)&b);
    __atomic_store_n(&g_ble_busy, 0u, __ATOMIC_RELEASE);
    break;
  }
  case sink::NONE:
  default:
    ++g_drop_none;
    break;
  }
}

} // namespace

void set_sink(uint32_t obj_id, sink s) {
  ensure_table();
  if (obj_id >= MAX_OBJECTS)
    return;
  g_sink[obj_id] = (uint8_t)s;
}

void set_default_sink(sink s) { g_default = s; }

void set_all_sinks(sink s) {
  ensure_table();
  for (uint32_t i = 0; i < MAX_OBJECTS; ++i)
    g_sink[i] = (uint8_t)s;
}

sink sink_of(uint32_t obj_id) {
  ensure_table();
  if (obj_id >= MAX_OBJECTS)
    return g_default;
  const uint8_t v = g_sink[obj_id];
  return (v == UNSET) ? g_default : (sink)v;
}

void write(const char *buf, uint32_t len) {
  if (len == 0)
    return;
  emit(sink_of(get_current_obj_id().value), buf, len);
}

void vprintf(const char *fmt, va_list ap) {
  const sink s = sink_of(get_current_obj_id().value);
  if (s == sink::NONE) { // 整形コストごと省く (黙らせる意味がある)
    ++g_drop_none;
    return;
  }
  char line[LINE_MAX];
  int n = vsnprintf(line, sizeof(line), fmt, ap);
  if (n < 0) {
    ++g_drop_fmt;
    return;
  }
  uint32_t len = (uint32_t)n;
  if (len >= sizeof(line))
    len = sizeof(line) - 1; // 切り詰め (vsnprintf は必要長を返す)
  emit(s, line, len);
}

void printf(const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  vprintf(fmt, ap);
  va_end(ap);
}

void printf_to(sink s, const char *fmt, ...) {
  if (s == sink::NONE)
    return;
  char line[LINE_MAX];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(line, sizeof(line), fmt, ap);
  va_end(ap);
  if (n < 0) {
    ++g_drop_fmt;
    return;
  }
  uint32_t len = (uint32_t)n;
  if (len >= sizeof(line))
    len = sizeof(line) - 1;
  emit(s, line, len);
}

uint32_t dropped(sink s) {
  switch (s) {
  case sink::BLE:
    return g_drop_ble;
  case sink::NONE:
    return g_drop_none;
  case sink::USB:
    return shizuku_usb_dropped();
  default:
    return g_drop_fmt;
  }
}

} // namespace log
} // namespace shizu
