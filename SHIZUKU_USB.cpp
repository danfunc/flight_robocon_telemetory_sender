// ===========================================================================
//  SHIZUKU_USB — printf を待たせない USB CDC 出力
// ===========================================================================
//  【なぜ要るか (実測)】
//  2026-08-07 の実機 A/B (BLE 接続/切断 各 n=3、シリアル非接続で計測):
//    Stage1 + stdio 既定       : 単発 poll 最大 1.006/1.006/1.005s, RT late 1.008s
//    Stage1 + stdio 非ブロック : 単発 poll 最大 0.038/0.038/0.038s, RT late 0.046s
//  「BLE を接続するとフリーズする」の原因は cyw43 ではなく printf だった。値が常に
//  ~1.005s と一定なのが手がかりで、これは PICO_STDIO_DEADLOCK_TIMEOUT_MS=1000 そのもの。
//  cyw43 SDK の do_ioctl timeout / STALL / could not bring bus up 警告は全ログで
//  一度も出ていない = 無線側は正常だった。
//
//  【設計】待ちをスケジューラ管轄へ戻す。
//    producer: out_chars → リングへ memcpy して即 return (待たない/落ちても止めない)
//    consumer: このオブジェクトの排出スレッドが
//                n = min(リング連続可読, tud_cdc_write_available())
//              だけを SDK の stdio_usb.out_chars へ渡す。**空き分しか渡さない**ので
//              SDK 内のブロッキング分岐 (空き 0 で 500ms ビジー回し) に構造的に入らない。
//              渡すものが無ければ yield_us でスケジューラへ返す。
//
//  【tinyusb を置き換えない理由】干渉していないから。tud_task() は SDK の低優先度
//  IRQ (USB IRQ + 周期タイマ) が回しており、スレッドを止めても USB は生き続ける。
//  tud_cdc_write() 自体は「入るだけ入れて受理数を返す」非ブロッキング FIFO push で、
//  欲しいプリミティブそのもの。ブロックしていたのは上のグルー層だけ。自前 USB スタックに
//  すると列挙/CDC クラス/descriptor に加えて picotool の reset-via-baud (フラッシュ手順)
//  まで再実装が要る割に、この問題には一切効かない。
//
//  【排他】out_chars は pico_stdio の stdio_put_string が print_mutex を持った状態で
//  呼ぶので producer は実質直列化されている (= リングは SPSC 扱いでよい)。保持時間が
//  memcpy 数µs になるため、旧実装で 1000ms まで伸びていたこの mutex の待ちも消える。
//  排出スレッドは stdio_usb.out_chars 経由なので SDK 内の stdio_usb_mutex も正しく通る。
// ===========================================================================
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <obj_api.hpp>
#include <object_headers/SHIZUKU_USB.hpp>
#include <pico/stdio.h>
#include <pico/stdio/driver.h>
#if !SHIZU_USB_MULTI_CDC
#include <pico/stdio_usb.h>
#endif

#if SHIZU_USB_MULTI_CDC
// マルチ CDC ビルドでは SHIZUKU_USB が USB デバイススタックを所有するため、
// SDK の stdio_usb ドライバは存在しない。排出先を tud_cdc_n_* へ差し替える実装は
// 次段 (未実装) — 現状はビルドを通すためのスタブで、リングは排出されない。
stdio_driver_t stdio_usb = {};
#endif
extern "C" {
// SDK が非 static で公開しているドライバ実体。実際の CDC 書き込みはこれに委譲する
// (mutex/tud_task の作法を SDK に任せられる。渡す量を空き以内に絞ることで
//  ブロッキング分岐には入らない)。
#if !SHIZU_USB_MULTI_CDC
extern stdio_driver_t stdio_usb;
#endif
#if !PICO_NO_HARDWARE
#include <tusb.h> // tud_cdc_write_available / tud_cdc_connected
#endif
}

namespace shizu {
namespace {

// ---- リング --------------------------------------------------------------
// 2 の冪。ブート時 (排出スレッド起動前) のログを丸ごと抱えられる程度に取る。
// .bss 常駐 = カーネルデータ分離の方針どおりヒープを使わない。
constexpr uint32_t RING_SIZE = 8192;
constexpr uint32_t RING_MASK = RING_SIZE - 1;
static_assert((RING_SIZE & RING_MASK) == 0, "RING_SIZE must be a power of two");

char g_ring[RING_SIZE];
volatile uint32_t g_head = 0; // producer が書いた総バイト数 (ラップしない単調増加)
volatile uint32_t g_tail = 0; // consumer が出した総バイト数
volatile uint32_t g_dropped = 0; // 溢れて捨てた累積バイト数

inline uint32_t ring_used() {
  return __atomic_load_n(&g_head, __ATOMIC_ACQUIRE) -
         __atomic_load_n(&g_tail, __ATOMIC_RELAXED);
}

// 1 回に SDK へ渡す上限。CDC の FIFO (既定 256B) を超えて渡しても意味が無く、
// 排出スレッドが 1 回の実行で握る時間も短く抑えたい。
constexpr uint32_t CHUNK_MAX = 128;
// 排出スレッドが「出すものが無い/CDC が満杯」のときにスケジューラへ返す間隔。
// 出せる限りは連続で出すので、これはアイドル時の巡回間隔でしかない。
constexpr uint32_t IDLE_NAP_US = 2000;

// ---- 自前 stdio ドライバ ---------------------------------------------------
// ★ここが「待たない」の本体。どのスレッド/コア/IRQ から呼ばれても memcpy だけで返る。
void ring_out_chars(const char *buf, int len) {
  if (len <= 0)
    return;
  const uint32_t head = __atomic_load_n(&g_head, __ATOMIC_RELAXED);
  const uint32_t used = head - __atomic_load_n(&g_tail, __ATOMIC_ACQUIRE);
  const uint32_t free_bytes = RING_SIZE - used;
  uint32_t n = (uint32_t)len;
  if (n > free_bytes) {
    // 溢れた分は捨てる (新しい方を捨てる = 既に積んだ行を壊さない)。ここで待つと
    // 「printf が呼び出し元を止める」旧実装に逆戻りするので絶対に待たない。
    g_dropped = g_dropped + (n - free_bytes);
    n = free_bytes;
    if (n == 0)
      return;
  }
  const uint32_t off = head & RING_MASK;
  const uint32_t first = (n < RING_SIZE - off) ? n : (RING_SIZE - off);
  memcpy(&g_ring[off], buf, first);
  if (n > first)
    memcpy(&g_ring[0], buf + first, n - first);
  // 本体の書き込みを publish してから head を進める (consumer は acquire で読む)。
  __atomic_store_n(&g_head, head + n, __ATOMIC_RELEASE);
}

// stdio_flush() 経路。呼び出し元を止めないのが本ドライバの契約なので、ここでは
// 「出せる分だけ出す」に留める (完全な同期排出が要る場面は emergency_drain)。
void ring_out_flush() {
  if (stdio_usb.out_flush)
    stdio_usb.out_flush();
}

// 入力は SDK 実装をそのまま使う。in_chars は「データがある時だけ」短く mutex を取る
// 実装で、満杯待ちのようなブロッキングは無い (BLE_UART の NC 確認 getchar_timeout_us
// もこの経路)。
int ring_in_chars(char *buf, int len) {
  return stdio_usb.in_chars ? stdio_usb.in_chars(buf, len) : PICO_ERROR_NO_DATA;
}

stdio_driver_t g_ring_driver = {
    .out_chars = ring_out_chars,
    .out_flush = ring_out_flush,
    .in_chars = ring_in_chars,
#if PICO_STDIO_USB_SUPPORT_CHARS_AVAILABLE_CALLBACK
    .set_chars_available_callback = nullptr,
#endif
    .next = nullptr,
#if PICO_STDIO_ENABLE_CRLF_SUPPORT
    // CRLF 変換は pico_stdio 側 (stdio_put_string) がドライバごとに適用するので、
    // リングには **変換済み** のバイトが入る。排出時は stdio_usb.out_chars を直接
    // 叩く = 変換ラッパを通らないため二重変換にならない。
    .last_ended_with_cr = false,
    .crlf_enabled = PICO_STDIO_USB_DEFAULT_CRLF,
#endif
};

// CDC の空き。USB 未接続なら SDK 側が捨てるので満杯にはならないが、ここでは
// 「空きが分かるなら空き分だけ」を守る (分からない構成では CHUNK_MAX で刻む)。
inline uint32_t cdc_space() {
#if !PICO_NO_HARDWARE
  if (!tud_cdc_connected())
    return CHUNK_MAX; // 非接続時は SDK が即捨てる = 詰まらない
  const uint32_t avail = tud_cdc_write_available();
  return avail;
#else
  return CHUNK_MAX;
#endif
}

// リングから 1 チャンクだけ出す。出したバイト数を返す (0 = 出せなかった)。
uint32_t drain_once() {
  const uint32_t used = ring_used();
  if (used == 0)
    return 0;
  uint32_t space = cdc_space();
  if (space == 0)
    return 0; // CDC 満杯。★ここで待たない (待つと SDK のビジー回しと同罪)
  if (space > CHUNK_MAX)
    space = CHUNK_MAX;
  const uint32_t tail = __atomic_load_n(&g_tail, __ATOMIC_RELAXED);
  const uint32_t off = tail & RING_MASK;
  uint32_t n = used;
  if (n > space)
    n = space;
  if (n > RING_SIZE - off)
    n = RING_SIZE - off; // ラップ跨ぎは 2 回に分ける
  stdio_usb.out_chars(&g_ring[off], (int)n);
  __atomic_store_n(&g_tail, tail + n, __ATOMIC_RELEASE);
  return n;
}

} // namespace

// ---- 公開フック ------------------------------------------------------------
void shizuku_usb_install() {
  // 自前ドライバを stdio チェインへ入れ、SDK の stdio_usb を外す。以後 printf は
  // ring_out_chars だけを通る (stdio_usb は排出スレッドが直接呼ぶ形で使い続ける)。
  stdio_set_driver_enabled(&g_ring_driver, true);
  stdio_set_driver_enabled(&stdio_usb, false);
}

uint32_t shizuku_usb_ring_used() { return ring_used(); }

void shizuku_usb_emergency_drain() {
  // panic / boot report 用。ここだけは可視性優先でブロッキングを許す (RT はもう
  // 守る意味が無い文脈)。無限には回さない: SDK 側も 500ms で諦める。
  for (uint32_t guard = 0; guard < RING_SIZE; ++guard) {
    const uint32_t used = ring_used();
    if (used == 0)
      break;
    const uint32_t tail = __atomic_load_n(&g_tail, __ATOMIC_RELAXED);
    const uint32_t off = tail & RING_MASK;
    uint32_t n = used;
    if (n > RING_SIZE - off)
      n = RING_SIZE - off;
    stdio_usb.out_chars(&g_ring[off], (int)n); // ここは詰まったら待つ (承知の上)
    __atomic_store_n(&g_tail, tail + n, __ATOMIC_RELEASE);
  }
  if (stdio_usb.out_flush)
    stdio_usb.out_flush();
}

uint32_t shizuku_usb_dropped() { return g_dropped; }

// ---- log.hpp のシンク実体 ---------------------------------------------------
// log::sink::USB。printf (pico_stdio) を経由せずリングへ直接積むので、print_mutex の
// WFE スピン (最大 PICO_STDIO_DEADLOCK_TIMEOUT_MS) を**構造的に踏まない**。
// CRLF 変換は通らない = 呼び出し元が "\n" を書いたらそのまま LF で出る。
void shizuku_usb_push(const char *buf, uint32_t len) {
  ring_out_chars(buf, (int)len);
}

// log::sink::PRINTK。リングを迂回して直に吐く (詰まったら待つ)。ここは待つのが仕様
// — スケジューラ自体が怪しい/まだ動いていない文脈で「確実に出す」ためのもの。
void shizuku_usb_printk(const char *buf, uint32_t len) {
  // 先にリングを吐き切って順序を保つ (これまでの非同期ログの後ろに繋げる)。
  shizuku_usb_emergency_drain();
  stdio_usb.out_chars(buf, (int)len);
  if (stdio_usb.out_flush)
    stdio_usb.out_flush();
}

// ---- オブジェクトエントリ (排出スレッド) ------------------------------------
void SHIZUKU_USB::main() {
  printf("[USB] ring drain thread up (%lu B ring, non-blocking printf)\n",
         (unsigned long)RING_SIZE);
  uint32_t reported_drops = 0;
  while (true) {
    // 出せる限り連続で出す (バースト直後を素早く掃く)。1 回の滞在時間は
    // CHUNK_MAX×数回ぶんに収まり、budget 3ms を超えない。
    uint32_t burst = 0;
    while (burst < 16 && drain_once() != 0)
      ++burst;

    // 欠落があったら復帰時に 1 回だけ報告する (リングが空いてから積む = 自分で
    // 溢れさせない)。printf 経由なので次の周回で排出される。
    const uint32_t drops = g_dropped;
    if (drops != reported_drops && ring_used() == 0) {
      const uint32_t delta = drops - reported_drops;
      reported_drops = drops;
      printf("[USB] dropped %lu B (total %lu B) — ring overflow\n",
             (unsigned long)delta, (unsigned long)drops);
    }

    if (burst == 0)
      obj_api::yield_us(IDLE_NAP_US); // 出すものが無い/CDC 満杯 → スケジューラへ返す
    else
      obj_api::yield(); // まだ残っているかも: 一度譲ってから続ける
  }
}

} // namespace shizu
