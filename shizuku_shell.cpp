// ===========================================================================
//  shizuku_shell.cpp — Shizuku OS 対話型・ワイヤレス管理シェル (CLI / BLE /
//  CDC)
// ===========================================================================
#include "shizuku_shell.hpp"
#include "blink.hpp"
#include "flight_controller.hpp"
#include "hardware/dma.h"
#include "hardware/regs/dma.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/watchdog.h"
#include "pico/stdlib.h"
#include "fw_version.hpp"
#include "props.hpp"
#include "shizuku/kernel.hpp"
#include "shizuku/object_api.hpp"
#include "shizuku/objects/ble_uart.hpp"
#include "shizuku/objects/flash_fs.hpp"
#include "shizuku/objects/ota.hpp"
#include "shizuku/stream.hpp"
#include "telemetry.hpp"
#include "tx_frame.hpp"
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

extern "C" {
void rom_reset_usb_boot(uint32_t usb_activity_gpio_pin_mask,
                        uint32_t disable_interface_mask);
extern const uint8_t __flash_binary_start[];
extern const uint8_t __flash_binary_end[];
}

namespace xno::shell {
namespace {

using ARCH = shizuku::KERNEL::ARCH;
using BOARD = shizuku::KERNEL::BOARD;
using frame_t = xno::tx_frame;

struct api_result {
  uintptr_t error;
  uintptr_t value;
};

__attribute__((always_inline)) static inline api_result
api(shizuku::object_api number, uintptr_t a1 = 0, uintptr_t a2 = 0,
    uintptr_t a3 = 0, uintptr_t a4 = 0) {
  register uintptr_t r0 asm("r0") = (uintptr_t)number;
  register uintptr_t r1 asm("r1") = a1;
  register uintptr_t r2 asm("r2") = a2;
  register uintptr_t r3 asm("r3") = a3;
  register uintptr_t r12 asm("r12") = a4;
  asm volatile("svc 0"
               : "+r"(r0), "+r"(r1)
               : "r"(r2), "r"(r3), "r"(r12)
               : "memory");
  return {r0, r1};
}

uintptr_t export_method(method m, uintptr_t entry) {
  return api(shizuku::object_api::EXPORT_METHOD, (uintptr_t)m, entry).error;
}

// ---- ハードウェア UART 定義 (Pico 2 W GP0:TX, GP1:RX) -----------------------
#define SHELL_UART uart0
constexpr uint32_t SHELL_UART_BAUD = 115200;
constexpr uint SHELL_UART_TX_PIN = 0;
constexpr uint SHELL_UART_RX_PIN = 1;

// ---- UART0 RX を DMA で「ストリーム」へ流し込む -----------------------------
//  ★なぜ要るか (2026-09-01 実測): 従来はポーリングで FIFO (32B) を読んでいた。
//    安全装置は 100ms ごとに 2 行 (約 100B) をまとめて送るので、115200 では
//    32B の FIFO は **2.8ms で溢れる**。シェルスレッドが BLE 等に取られて
//    その間走らないと、行の**途中が欠ける**。実際にそうなり、
//    `SAFE2,0,LINK_LOST` が `SA,LINK_LOST` になって未知コマンド扱いされ、
//    その返事が UART と BLE を埋めて **OTA も OTW も通らなくなった**。
//    DMA なら**スレッドが走っていなくても受け続ける**ので、取りこぼしが
//    「スケジューリングが間に合うか」に依存しなくなる。
//  ★★baud を上げるのはこれを入れてから。1Mbaud だと FIFO は 0.32ms で
//    溢れるので、ポーリングのまま速くすると悪化するだけ。
//
//  ★★★行き先を**私物の環ではなくストリームにする**。理由は 2 つ:
//    (1) `descriptor.wr` は「公開済みレコード数」の**単調カウンタ**で、
//        rec_size=1 のバイトストリームなら **DMA が書いた総バイト数がそのまま
//        wr**。つまり公開は 1 ストアで済み、DMA とストリームの意味論が
//        そのまま一致する。
//    (2) 追い越しの検出と「落ちた数」の計上は `pop()` が既に持っている
//        (stream.hpp)。自前の環だと同じ処理を書き直すことになり、
//        **数え方が 2 つある**状態になる。
//    こうしておけば、UART をペリフェラルオブジェクトが持つ形へ移すときも、
//    この初期化の置き場所が変わるだけで、読む側は何も変えなくてよい。
//    `connect()` でストリーム間 DMA へ繋ぐ道も開く。
// ★環の大きさも線速で決める。1024B は 1 Mbaud だと **10ms 分**しかなく、
//   シェルスレッドがそれ以上走らなければ DMA に追い越される (上の
//   g_uart_ota と同じ話が 1 段手前でも起きる)。4096B なら約 40ms 分。
constexpr uint32_t UART_RX_RING_BITS = 12; // 2^12 = 4096B
constexpr uint32_t UART_RX_RING = 1u << UART_RX_RING_BITS;
// ★DMA の環アドレッシングは**バッファがその大きさに整列していること**を要求する。
//   storage<> は先頭に descriptor を置くので整列を保証できない。だから
//   バッファと記述子を別々に持ち、記述子から指す。
alignas(UART_RX_RING) uint8_t g_uart_rx_buf[UART_RX_RING];
shizuku::stream::descriptor g_uart_rx_desc{};
uintptr_t g_uart_rx_stream_id = xno::NO_STREAM;
shizuku::stream::handle<uint8_t> g_uart_rx;
int g_uart_dma_ch = -1;
uint32_t g_uart_rx_lost = 0; // pop() が報告した「落ちた数」の累計 (診断用)
// ★★診断用。g_uart_rx_lost はこれまで積算されるだけで**どこにも出力されて
//   いなかった**。OTW の inflate failed の原因候補 (シェルスレッドが長く
//   スケジュールされず、非 LOSSLESS な DMA 環 (4096B ≈ 1Mbaud で 40ms 分) が
//   uart_bridge_push_byte で 244B へ切り出す前に溢れて上書きされる) を
//   実機で切り分けるための足跡。UBRIDGE 区間ごとの増分を UBRIDGE_DONE に載せる。
uint32_t g_uart_rx_lost_at_bridge_start = 0;

// ★★RP2350 の TRANS_COUNT は **[31:28] が MODE / [27:0] がカウント**
//   (RP2040 には無いフィールド)。ここに 0xFFFFFFFF を書くと MODE=0xF =
//   **ENDLESS** が選ばれ、その名のとおり**カウントが減らなくなる**。
//   すると下の引き算が常に 0 を返し、wr が一度も進まない = ストリームが
//   永久に空に見える。DMA は正常にバッファへ書いているのに、**受信が
//   まるごと止まったように見える**。2026-09-01 に実機で踏んだ。
//   MODE=NORMAL のまま最大まで使うので、要求数は 28bit に収める。
constexpr uint32_t UART_RX_DMA_COUNT = DMA_CH0_TRANS_COUNT_COUNT_BITS; // 0x0FFFFFFF

// DMA が今までに書いた総バイト数。transfer_count は残数なので引き算で出す。
// ★読み出しでも MODE のビットを落としてから使う。
uint32_t uart_rx_total() {
  const uint32_t remaining = dma_channel_hw_addr(g_uart_dma_ch)->transfer_count &
                             DMA_CH0_TRANS_COUNT_COUNT_BITS;
  return UART_RX_DMA_COUNT - remaining;
}

// DMA が進めた分をストリームへ公開する。★DMA は wr を書けないので、
//   ここだけが producer の仕事。1 ストアで済むのがバイトストリームの利点。
void uart_rx_publish() {
  if (g_uart_dma_ch < 0)
    return;
  __atomic_store_n(&g_uart_rx_desc.wr, uart_rx_total(), __ATOMIC_RELEASE);
}

void uart_rx_dma_start() {
  g_uart_rx_desc.base = g_uart_rx_buf;
  g_uart_rx_desc.rec_size = 1;
  g_uart_rx_desc.capacity = UART_RX_RING;
  g_uart_rx_desc.flags = 0; // 上書き許容 (producer は待てない = DMA だから)
  g_uart_rx_desc.wr = 0;
  g_uart_rx_desc.rd = 0;
  g_uart_rx_desc.producer = shizuku::stream::NO_OWNER;
  g_uart_rx_desc.consumer = shizuku::stream::NO_OWNER;
  g_uart_rx = shizuku::stream::handle<uint8_t>(&g_uart_rx_desc);

  g_uart_dma_ch = dma_claim_unused_channel(false);
  if (g_uart_dma_ch < 0) {
    BOARD::diag_printf("[SHELL] no DMA channel — UART は従来のポーリング\n");
    return; // 空きが無ければ従来のポーリングのまま (遅いが動く)
  }
  // ★FIFO は切る。DREQ は FIFO のしきい値で上がるので、有効なままだと
  //   しきい値に満たない末尾が次のバイトが来るまで届かず、行末が遅れる。
  uart_set_fifo_enabled(SHELL_UART, false);
  dma_channel_config c = dma_channel_get_default_config(g_uart_dma_ch);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, true);
  channel_config_set_ring(&c, true, UART_RX_RING_BITS); // 書き側を環に
  channel_config_set_dreq(&c, uart_get_dreq(SHELL_UART, false));
  dma_channel_configure(g_uart_dma_ch, &c, g_uart_rx_buf,
                        &uart_get_hw(SHELL_UART)->dr, UART_RX_DMA_COUNT, true);
  BOARD::diag_printf("[SHELL] UART RX -> stream %lu via DMA ch%d\n",
                     (unsigned long)g_uart_rx_stream_id, g_uart_dma_ch);
}

// 以降、UART の読み口はこの 2 つだけ。DMA が取れなかった場合は素の UART へ
// 落ちるので、呼ぶ側は経路を意識しなくてよい。
uint32_t uart_rx_available() {
  if (g_uart_dma_ch < 0)
    return uart_is_readable(SHELL_UART) ? 1u : 0u;
  uart_rx_publish();
  return g_uart_rx.available();
}

// 受信ストリームに溜まっている分を全部捨てる。
// ★★中継の出入りで必ず通すこと (2026-09-02 実測)。中断した転送の**続き**が
//   環に残ったまま行モードへ戻ると、シェルは像の続きをコマンドとして解釈し
//   始める。実際に `PING` が `FPING` になって届き (残骸の 1 バイトが頭に
//   食い込んだ)、以後の会話が壊れた。捨てるのは「読み飛ばし」ではなく
//   **rd を wr に合わせる** = ストリームの意味論そのままで済む。
void uart_rx_discard_all() {
  if (g_uart_dma_ch < 0) {
    while (uart_is_readable(SHELL_UART))
      (void)uart_getc(SHELL_UART);
    return;
  }
  uart_rx_publish();
  __atomic_store_n(&g_uart_rx_desc.rd,
                   __atomic_load_n(&g_uart_rx_desc.wr, __ATOMIC_ACQUIRE),
                   __ATOMIC_RELEASE);
}

int uart_rx_getc() {
  if (g_uart_dma_ch < 0)
    return uart_is_readable(SHELL_UART) ? (int)uart_getc(SHELL_UART) : -1;
  uart_rx_publish();
  uint8_t v = 0;
  uint32_t lost = 0;
  if (!g_uart_rx.pop(&v, &lost)) {
    g_uart_rx_lost += lost;
    return -1;
  }
  g_uart_rx_lost += lost;
  return (int)v;
}

// ---- 出力ストリーム (logger 経由 BLE TX notify)
// ------------------------------
shizuku::stream::storage<frame_t, 16> g_out;
uintptr_t g_out_id = 0;

// ---- 入力ストリーム (ble_uart の RX)
// -----------------------------------------
uintptr_t g_rx_id = xno::NO_STREAM;
shizuku::stream::handle<frame_t> g_rx;

// ---- UART バイナリブリッジ (XIAO 経由の OTA、BLE の代替経路) ---------------
//  ★UART0 の読み手は shizuku_shell だけ (二人読みにしない)。バイナリ転送中は
//    「行として読んでコマンド解釈する」代わりに「そのまま ota の入力ストリームへ
//    積む」へモードを切り替える。持ち主(UART0 のリーダー)は変えず、
//    バイトの行き先だけを変える設計。
// ★★枠数は **flash の停止時間 × 線速** で決める (2026-09-02 実測で 4 → 64)。
//   ota は 1 セクタ消去 + 書き込みで数十 ms 止まる。その間に線から来る分を
//   ここで飲めなければ取りこぼす。4 枠 (976B) では 24 kB/s ですら
//   66KB 地点で `input overrun` になった。
//   BLE 側 (ble_uart.cpp の g_ota_rx) は同じ理由で最初から 32 枠あり、
//   **UART 側だけ 4 枠のまま取り残されていた** —— 経路が増えたときに
//   片方だけ直し忘れる、の典型。
//   64 枠 = 15.6KB は、1 Mbaud (約 100 kB/s) で 150ms 分の余裕にあたる。
shizuku::stream::storage<frame_t, 64> g_uart_ota;
uintptr_t g_uart_ota_id = 0;
// ブリッジ終了後に ota の入力を戻す先。★0 で初期化しないこと —
//   **ストリーム番号 0 は正当な番号**なので、0 を「未配線」の印に使うと、
//   配線に失敗したまま「0 番へ戻す」= 無関係のストリームへ ota を繋いでしまう。
uintptr_t g_ble_ota_stream_id = xno::NO_STREAM;
bool g_uart_bridge_active = false;
uint32_t g_uart_bridge_remaining = 0;
// 中継中に 1 バイトも来ない時間がこれを超えたら抜ける (XIAO 側の 3 秒より長く
// 取る: 先に向こうが諦めて ABORT を送れるようにし、両方が同時に切れるのを避ける)。
constexpr uint64_t UART_BRIDGE_IDLE_US = 5000000;
uint64_t g_uart_bridge_last_us = 0;

// ★中継中でも聞こえる「やめろ」の合言葉 (XIAO 側と一致させること)。
//   ブリッジ中はバイトを全部イメージとして飲むので、"UBRIDGE_ABORT\n" のような
//   文字列は届かない (データと区別が付かない)。長い固定並びだけが聞き取れる。
//   ★代償は偽陽性 — イメージ中に偶然この 8 バイトが並ぶと中断する。8 バイトなら
//     2^-64 で無視でき、当たっても結果は「CRC が合わずに再送」。**取りこぼすより
//     誤検出するほうが安全側**なのでこの交換を選んでいる。
constexpr uint8_t BRIDGE_ABORT_MAGIC[8] = {0x55, 0xA5, 'U', 'B', 'R', 'K',
                                           0x5A, 0xAA};

// ★モード切替 (raw ⇄ 行コマンド) の同期用。ASCII の ACK/NAK をそのまま使う
//   (印字可能な範囲の外なので、ログ行やコマンド行と混ざる余地が無い —
//   BRIDGE_ABORT_MAGIC と同じ発想)。end_uart_bridge() が baud を戻す**前**、
//   まだ相手 (XIAO) が聞いている速度のうちに 1 バイトだけ送る。
//   XIAO 側はこれを見た時点で「Pico はもう何も出さない」と確信して即座に
//   baud を戻せるので、2 秒のアイドル待ちに頼らずに済む (待ちは相手が古い
//   ファームのときの後方互換フォールバックとして残す)。XIAO 側の定数と
//   値を一致させること (flight_robocon_safety/src/shell.rs)。
constexpr uint8_t UART_BRIDGE_ACK = 0x06; // 転送・commit とも成功
constexpr uint8_t UART_BRIDGE_NAK = 0x15; // 失敗 (CRC 不一致・inflate 失敗等)
uint8_t g_abort_window[sizeof(BRIDGE_ABORT_MAGIC)] = {};
uint32_t g_abort_filled = 0;

// 直近 8 バイトが合言葉と一致したか。★1 バイトずつずらして見る (窓)。
bool abort_magic_seen(uint8_t b) {
  for (uint32_t i = 1; i < sizeof(g_abort_window); ++i)
    g_abort_window[i - 1] = g_abort_window[i];
  g_abort_window[sizeof(g_abort_window) - 1] = b;
  if (g_abort_filled < sizeof(g_abort_window)) {
    ++g_abort_filled;
    return false;
  }
  for (uint32_t i = 0; i < sizeof(g_abort_window); ++i)
    if (g_abort_window[i] != BRIDGE_ABORT_MAGIC[i])
      return false;
  return true;
}
frame_t g_uart_bridge_frame{};

void uart_bridge_push_byte(uint8_t b) {
  g_uart_bridge_frame.data[g_uart_bridge_frame.len++] = b;
  if (g_uart_bridge_frame.len >= sizeof(g_uart_bridge_frame.data)) {
    g_uart_ota.hdl().push(g_uart_bridge_frame);
    g_uart_bridge_frame = frame_t{};
  }
}

void uart_bridge_flush() {
  if (g_uart_bridge_frame.len > 0) {
    g_uart_ota.hdl().push(g_uart_bridge_frame);
    g_uart_bridge_frame = frame_t{};
  }
}

// 中継中だけ使う速度。★常用の速度は上げない — 上げると、**片方だけ焼き替えた
//   瞬間に会話できなくなる**。PING も `nc forget!` も通らなくなり、
//   締め出しからの逃げ道ごと失う。速いのが要るのは転送中だけなので、
//   そこだけ切り替えて必ず戻す。
uint32_t g_uart_bridge_baud = SHELL_UART_BAUD;

void begin_uart_bridge(uint32_t total_bytes, uint32_t baud) {
  using shizuku::objects::ota::method;
  api(shizuku::object_api::CALL_METHOD, xno_object_id::ota,
      (uintptr_t)method::SET_INPUT_STREAM, g_uart_ota_id);
  g_uart_bridge_remaining = total_bytes;
  g_uart_bridge_frame = frame_t{};
  g_uart_bridge_active = true;
  g_uart_bridge_last_us = BOARD::time_us();
  g_abort_filled = 0; // 前の中継の名残りで即座に誤爆させない
  uart_rx_discard_all(); // 前の会話の残りを像の先頭と間違えない
  g_uart_rx_lost_at_bridge_start = g_uart_rx_lost; // この区間の増分を測る基点
  uart_puts(SHELL_UART, "UBRIDGE_READY\n");
  // ★★合図を**送り切ってから**速度を変える。TX FIFO に残ったまま切り替えると、
  //   相手は READY の途中から別の速度で受け取ることになり、合言葉が壊れる。
  if (baud != SHELL_UART_BAUD) {
    uart_tx_wait_blocking(SHELL_UART);
    uart_set_baudrate(SHELL_UART, baud);
    g_uart_bridge_baud = baud;
  }
}

void end_uart_bridge() {
  using shizuku::objects::ota::method;
  uart_bridge_flush();
  g_uart_bridge_active = false;
  // ★★戻す前に**流し切るのを待つ**。ota は自分のスレッドで非同期に汲むので、
  //   押し込んだ直後に入力を差し替えると、まだ汲まれていない末尾が宙に浮いて
  //   そのまま捨てられる (環は 4 枠 = 976 バイトしかない)。像の最後だけが
  //   欠ける形になり、CRC は弾いてくれるが原因は分かりにくい。
  for (uint32_t spin = 0; spin < 2000 && g_uart_ota.hdl().available() > 0;
       ++spin)
    api(shizuku::object_api::YIELD);
  // ★★★available()==0 は「積んだ分は pop された」しか言っていない。pop の
  //   中で feed()/finish_upload() (最終セクタの flash 書き込み・CRC 読み返し・
  //   "done:" 行の送出) が走っている最中でも、pop 自体は先に rd を進めて
  //   戻ってしまうので available() は 0 に見える。ここで baud を戻すと、
  //   ota がまだ古い (速い) baud のつもりで送っている行の途中で速度が変わり、
  //   XIAO 側は自分の 2 秒アイドルで別々に baud を戻す (協調していない) ため、
  //   両者の切り戻しタイミングがずれて完了行が文字化けする
  //   (2026-09-02 実測: 230400/1Mbaud で "done: ... crc=..." が化けて消えた。
  //   115200 = 速度を一切変えない設定では発生しない = この経路でのみ起きる)。
  //   ota が本当に手ぶらになるのを見てから baud を戻す。
  // ★★条件は GET_STATE==IDLE ではなく **GET_QUIESCENT** を使う。チャンク
  //   単位再送 (XNOR) のラウンドの合間、ota は IDLE ではなく CSEEK (次の
  //   チャンクを待っている) で待機しており、IDLE を待つと永久に来ない。
  //   かといって転送を捨てさせるわけにはいかない (受領ビットマップごと
  //   消えて全再送になる)。GET_QUIESCENT は「メッセージの途中でもなく
  //   feed() の最中でもない」を ota 自身に判定させたもので、従来の XNOZ
  //   経路では実質 IDLE と同じ意味になる (後方互換)。
  for (uint32_t spin = 0;
       spin < 2000 &&
       api(shizuku::object_api::CALL_METHOD, xno_object_id::ota,
           (uintptr_t)method::GET_QUIESCENT, 0)
               .value == 0;
       ++spin)
    api(shizuku::object_api::YIELD);
  // ★★ここで baud を戻す前に、XIAO へ「もう出すものは無い」を machine-readable
  //   な 1 バイトで伝える。commit が成功した場合はここへ戻ってこない
  //   (flash_safe_execute の中で直接再起動する、"no return") ので、その
  //   ケースは今まで通り XIAO 側の 2 秒アイドル待ちが拾う — ACK は
  //   「ステージングだけして commit しない」経路 (再送・実機試験) を主に
  //   縮めるためのもの。
  {
    const bool ok =
        api(shizuku::object_api::CALL_METHOD, xno_object_id::ota,
            (uintptr_t)method::GET_LAST_OK, 0)
            .value != 0;
    const uint8_t sync = ok ? UART_BRIDGE_ACK : UART_BRIDGE_NAK;
    uart_putc_raw(SHELL_UART, sync);
  }
  if (g_ble_ota_stream_id != xno::NO_STREAM) {
    api(shizuku::object_api::CALL_METHOD, xno_object_id::ota,
        (uintptr_t)method::SET_INPUT_STREAM, g_ble_ota_stream_id);
  } else {
    // 配線されていないなら**戻さない**。適当な番号へ繋ぐより、BLE OTA が
    // 効かないまま次の再起動を待つほうが安全 (回復手段を壊さない)。
    uart_puts(SHELL_UART, "UBRIDGE_WARN no ble ota stream to restore\n");
  }
  // ★★DMA 環 (g_uart_rx_desc) は非 LOSSLESS = 上書き許容。ここが溢れると
  //   uart_bridge_push_byte に渡る前にバイトが消え、ota 側の「input overrun」
  //   (g_uart_ota 側の lost) には一切現れない。inflate failed の原因候補の
  //   一つなので、区間ごとの増分を必ず報告する (0 ならこの経路は無罪)。
  {
    char line[48];
    snprintf(line, sizeof(line), "UBRIDGE_LOST %lu\n",
             (unsigned long)(g_uart_rx_lost - g_uart_rx_lost_at_bridge_start));
    uart_puts(SHELL_UART, line);
  }
  uart_puts(SHELL_UART, "UBRIDGE_DONE\n");
  // ★★速度を必ず常用へ戻す。**この経路を通らずに抜けると口が死ぬ**ので、
  //   中継から出る道は全部ここを通す (完了・中断・無通信タイムアウト)。
  //   戻す前に DONE を送り切る — 相手はまだ速い方で聞いているため。
  if (g_uart_bridge_baud != SHELL_UART_BAUD) {
    uart_tx_wait_blocking(SHELL_UART);
    uart_set_baudrate(SHELL_UART, SHELL_UART_BAUD);
    g_uart_bridge_baud = SHELL_UART_BAUD;
  }
  // ★速度を戻した**後**に捨てる。残っているのは古い速度で受けたバイトなので、
  //   行として解釈しても意味が無いどころか害になる。
  uart_rx_discard_all();
}

// ---- シェル応答送信用ヘルパー (BLE TX notify & UART0) -----------------------
void shell_send_frame(const frame_t &f) {
  // ★焼いている最中は BLE へ何も積まない。上の読み飛ばしで大半は塞げるが、
  //   出口でも止めておく (経路が増えたときに片方だけ直し忘れるため)。
  if (shizuku::objects::ota::flash_busy())
    return;
  g_out.hdl().push(f);
}

void shell_printf(const char *fmt, ...) {
  char buf[64];
  va_list args;
  va_start(args, fmt);
  int len = vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  if (len > 0) {
    frame_t f{};
    f.len = (uint16_t)(len < 64 ? len : 63);
    memcpy(f.data, buf, f.len);
    shell_send_frame(f);
    if (uart_is_enabled(SHELL_UART)) {
      uart_puts(SHELL_UART, buf);
    }
  }
  printf("%s", buf);
  fflush(stdout);
}

// ---- fs コマンド -----------------------------------------------------------
//  ★flash FS は「たまに書いて、ずっと読む」ための媒体。ここから **書く** 系を
//    出すのは、シェルが「止まってよい経路」だから — 消去は 1 セクタ約 33ms、
//    その間 XIP が止まる = 系全体が止まる。周期スレッドから触れる口は出さない。
void handle_fs_command(const char *argument) {
  using shizuku::objects::flash_fs_method;
  using shizuku::objects::FLASH_FS_OBJECT;
  const auto fs = [](flash_fs_method method, uintptr_t a) {
    return api(shizuku::object_api::CALL_METHOD, FLASH_FS_OBJECT,
               (uintptr_t)method, a);
  };
  if (!xno::props::available()) {
    shell_printf("fs: unavailable (flash fs が登録されていない)\n");
    return;
  }
  if (argument[0] == '\0' || strcmp(argument, "ls") == 0) {
    shizuku::objects::flash_entry entry{};
    uint32_t shown = 0;
    for (uint32_t index = 0; index < 64; ++index) {
      entry = shizuku::objects::flash_entry{};
      entry.index = index;
      if (fs(flash_fs_method::LIST, (uintptr_t)&entry).value == 0)
        break;
      shell_printf("  %-24s %6lu B  @%p\n", entry.name,
                   (unsigned long)entry.bytes, (void *)entry.address);
      ++shown;
    }
    if (shown == 0)
      shell_printf("  (empty)\n");
    return;
  }
  if (strcmp(argument, "stat") == 0) {
    shizuku::objects::flash_status status{};
    fs(flash_fs_method::STATUS, (uintptr_t)&status);
    shell_printf("fs: @%p %lu KiB, %lu files, used %lu B, free %lu B\n",
                 (void *)status.region_address,
                 (unsigned long)(status.region_bytes / 1024),
                 (unsigned long)status.entries,
                 (unsigned long)status.used_bytes,
                 (unsigned long)status.free_bytes);
    return;
  }
  if (strncmp(argument, "cat ", 4) == 0) {
    shizuku::objects::flash_lookup lookup{argument + 4, 0, 0};
    fs(flash_fs_method::LOOKUP, (uintptr_t)&lookup);
    if (lookup.address == 0) {
      shell_printf("fs: '%s' not found\n", argument + 4);
      return;
    }
    // ★XIP なので写さずにそのまま読める。印字だけは長さで縛る (媒体には
    //   1MB 置けるが、シェルの行に流してよい量ではない)。
    constexpr uint32_t LIMIT = 192;
    const uint32_t bytes = lookup.bytes < LIMIT ? lookup.bytes : LIMIT;
    const uint8_t *data = (const uint8_t *)lookup.address;
    shell_printf("fs: '%s' %lu B @%p\n", argument + 4,
                 (unsigned long)lookup.bytes, (void *)lookup.address);
    char hex[3 * 16 + 1];
    for (uint32_t offset = 0; offset < bytes; offset += 16) {
      uint32_t n = 0;
      for (uint32_t i = 0; i < 16 && offset + i < bytes; ++i)
        n += (uint32_t)snprintf(hex + n, sizeof(hex) - n, "%02x ",
                                data[offset + i]);
      shell_printf("  %04lx  %s\n", (unsigned long)offset, hex);
    }
    if (lookup.bytes > bytes)
      shell_printf("  ... (%lu B 省略)\n",
                   (unsigned long)(lookup.bytes - bytes));
    return;
  }
  if (strncmp(argument, "rm ", 3) == 0) {
    shell_printf(xno::props::remove(argument + 3) ? "fs: removed '%s'\n"
                                                  : "fs: '%s' not found\n",
                 argument + 3);
    return;
  }
  if (strcmp(argument, "format!") == 0) {
    // ★'!' を要求する。FORMAT は**配ったアドレスを全部腐らせる**ので、
    //   打ち間違いで走ってよい操作ではない。
    fs(flash_fs_method::FORMAT, 0);
    shell_printf("fs: formatted (置いてあったものは全部消えた)\n");
    return;
  }
  shell_printf("fs: unknown (ls | stat | cat <name> | rm <name> | format!)\n");
}

// ---- ペアリング (numeric comparison / 施錠) の口 ---------------------------
//  ★なぜシェルに置くか: 飛行前は USB を繋がない運用なので、ble_uart が持って
//    いた「CDC で y/n」だけでは NC の承認ができない。UART0 (安全装置の XIAO)
//    の読み書きを持っているのはこのシェルなので、承認の口もここに出す。
//  ★ここから btstack は一切触らない。ble_uart のメソッドを呼んで**旗を立てる**
//    だけで、実際の sm_* は ble_uart の poll スレッドが叩く
//    ([[no-btstack-from-caller-thread]]: 呼び出し元スレッドから btstack に
//    入ると CYW43 の SPI バスごと固まる)。
namespace ble = shizuku::objects::ble_uart;

api_result ble_call(ble::method m, uintptr_t a = 0) {
  return api(shizuku::object_api::CALL_METHOD, xno_object_id::ble_uart,
             (uintptr_t)m, a);
}

void print_pairing_state(const ble::pairing_state &s) {
  shell_printf("NCSTAT: locked=%u bonds=%u pending=%u link=%u auth=%u\n",
               s.locked, s.bonded, s.nc_pending, s.connected, s.authorized);
  shell_printf("        strikes=%lu allow=%lus block=%lus\n",
               (unsigned long)s.strikes, (unsigned long)s.allow_seconds_left,
               (unsigned long)s.block_seconds_left);
  if (s.nc_pending)
    shell_printf("        待機中の番号 %06lu ('nc y' / 'nc n')\n",
                 (unsigned long)s.nc_passkey);
}

void handle_nc_command(const char *argument) {
  ble::pairing_state s{};
  if (argument[0] == '\0' || strcmp(argument, "status") == 0) {
    ble_call(ble::method::GET_PAIRING_STATE, (uintptr_t)&s);
    print_pairing_state(s);
    return;
  }
  if (strcmp(argument, "y") == 0 || strcmp(argument, "yes") == 0) {
    const auto r = ble_call(ble::method::PAIRING_ANSWER, 1);
    shell_printf(r.value ? "NC: confirmed\n" : "NC: 訊かれていない\n");
    return;
  }
  if (strcmp(argument, "n") == 0 || strcmp(argument, "no") == 0) {
    const auto r = ble_call(ble::method::PAIRING_ANSWER, 0);
    shell_printf(r.value ? "NC: declined\n" : "NC: 訊かれていない\n");
    return;
  }
  if (strcmp(argument, "lock") == 0) {
    ble_call(ble::method::SET_PAIRING_LOCK, 1);
    shell_printf("NC: locked (新規ペアリングを拒否)\n");
    return;
  }
  if (strcmp(argument, "allow") == 0) {
    // ★「解錠」ではなく**一回券**。ble_uart 側で時間切れとペアリング成立の
    //   両方で失効する。戻し忘れを人の記憶に頼らないため。
    ble_call(ble::method::SET_PAIRING_LOCK, 0);
    ble_call(ble::method::GET_PAIRING_STATE, (uintptr_t)&s);
    shell_printf("NC: 次の 1 回だけペアリングを許可 (残り %lus)\n",
                 (unsigned long)s.allow_seconds_left);
    return;
  }
  if (strcmp(argument, "forget!") == 0) {
    // ★'!' を要求する。**打ち間違いで走ってよい操作ではない** —— 母艦との
    //   ボンドが消えるので、次の接続は必ず NC のやり直しになる。fs format! と
    //   同じ作法。
    ble_call(ble::method::FORGET_BONDS);
    shell_printf("NC: forgetting bonds (リンクを一度落とします)\n");
    return;
  }
  shell_printf("nc: unknown (status | y | n | lock | allow | forget!)\n");
}

// ---- コマンド解釈ディスパッチャ
// ----------------------------------------------
void handle_command_line(char *line) {
  while (*line && ((uint8_t)*line <= ' ' || (uint8_t)*line > 126))
    ++line;
  size_t len = strlen(line);
  while (len > 0 && ((uint8_t)line[len - 1] <= ' ' || (uint8_t)line[len - 1] > 126)) {
    line[--len] = '\0';
  }
  if (len == 0)
    return;

  // 1. システム制御 (REBOOT, BOOTSEL)
  if (strcmp(line, "reboot") == 0 || strcmp(line, "RB") == 0) {
    shell_printf("Rebooting system...\n");
    sleep_ms(100);
    watchdog_reboot(0, 0, 10);
  } else if (strcmp(line, "bootsel") == 0 || strcmp(line, "BS") == 0) {
    shell_printf("Rebooting into BOOTSEL mode...\n");
    sleep_ms(100);
    rom_reset_usb_boot(0, 0);
  }
  // 2. テレメトリレート・操縦 (R<ms>, STATUS, ARM, DISARM)
  else if (line[0] == 'R' || line[0] == 'r') {
    const uint32_t rate_ms = (uint32_t)strtoul(line + 1, nullptr, 10);
    if (rate_ms >= 5 && rate_ms <= 10000) {
      api(shizuku::object_api::CALL_METHOD, telemetry::OBJECT,
          (uintptr_t)telemetry::method::SET_RATE, rate_ms);
      shell_printf("RATE ok %lu ms\n", (unsigned long)rate_ms);
    } else {
      shell_printf("RATE err\n");
    }
  } else if (strncmp(line, "STATUS", 6) == 0 || strcmp(line, "status") == 0) {
    // ★グローバル直読みをやめ、持ち主に訊く (telemetry.cpp 冒頭の理由と同じ)。
    ::flight_controller::control_state cs{};
    api(shizuku::object_api::CALL_METHOD, ::flight_controller::OBJECT,
        (uintptr_t)::flight_controller::method::GET_CONTROL_STATE, (uintptr_t)&cs);
    shell_printf(
        "STATUS: armed=%u vstate=%u elev=%.1f rud=%.1f thr=%.1f h_est=%.1f\n",
        cs.armed, cs.vstate, cs.elevator, cs.rudder, cs.throttle * 100.0f,
        cs.h_est);
  } else if (strncmp(line, "DISARM", 6) == 0) {
    // ★DISARM を先に見る。"ARM" の前方一致で先に拾ってしまうと、DISARM が
    //   ARM として通る (2 文字目以降を見ていないため)。
    api(shizuku::object_api::CALL_METHOD, ::flight_controller::OBJECT,
        (uintptr_t)::flight_controller::method::ARM, 0);
    shell_printf("DISARM ok\n");
  } else if (strncmp(line, "ARM", 3) == 0) {
    // ★番号は enum から取る。3 と直書きしてあったが、3 は POLL で ARM は 7
    //   だった (つまり ARM も DISARM も POLL を呼んでいた)。
    api(shizuku::object_api::CALL_METHOD, ::flight_controller::OBJECT,
        (uintptr_t)::flight_controller::method::ARM, 1);
    shell_printf("ARM ok\n");
  } else if (strncmp(line, "PITCH ", 6) == 0 ||
             strncmp(line, "pitch ", 6) == 0) {
    float deg = strtof(line + 6, nullptr);
    int32_t cdeg = (int32_t)(deg * 100.0f);
    api(shizuku::object_api::CALL_METHOD, ::flight_controller::OBJECT,
        (uintptr_t)::flight_controller::method::SET_PITCH_REF, (uintptr_t)cdeg);
    shell_printf("PITCH ref %.1f deg\n", deg);
  } else if (strncmp(line, "HEAD ", 5) == 0 || strncmp(line, "head ", 5) == 0 ||
             strncmp(line, "HEADING ", 8) == 0 ||
             strncmp(line, "heading ", 8) == 0) {
    const char *p =
        (line[0] == 'H' && line[1] == 'E' && (line[4] == 'I' || line[4] == 'i'))
            ? (line + 8)
            : (line + 5);
    float deg = strtof(p, nullptr);
    int32_t cdeg = (int32_t)(deg * 100.0f);
    api(shizuku::object_api::CALL_METHOD, ::flight_controller::OBJECT,
        (uintptr_t)::flight_controller::method::SET_HEADING_REF,
        (uintptr_t)cdeg);
    shell_printf("HEADING ref %.1f deg\n", deg);
  } else if (strncmp(line, "ALT ", 4) == 0 || strncmp(line, "alt ", 4) == 0) {
    float alt_m = strtof(line + 4, nullptr);
    int32_t mm = (int32_t)(alt_m * 1000.0f);
    api(shizuku::object_api::CALL_METHOD, ::flight_controller::OBJECT,
        (uintptr_t)::flight_controller::method::SET_ALT_REF, (uintptr_t)mm);
    shell_printf("ALT ref %.1f m\n", alt_m);
  } else if (strcmp(line, "version") == 0 || strcmp(line, "VER") == 0 ||
             strcmp(line, "crc") == 0) {
    // ★計算は fw_version へ寄せた。telemetry 側も同じ値を出す必要があり、
    //   2 箇所に同じ CRC 実装を置くとズレたときに気づけない。
    const auto &id = xno::firmware_id();
    shell_printf("VER: size=%lu crc=%08lx\n", (unsigned long)id.bytes,
                 (unsigned long)id.crc32);
  } else if (strncmp(line, "fs", 2) == 0 &&
             (line[2] == '\0' || line[2] == ' ')) {
    handle_fs_command(line[2] == '\0' ? "" : line + 3);
  } else if (strncmp(line, "nc", 2) == 0 &&
             (line[2] == '\0' || line[2] == ' ')) {
    handle_nc_command(line[2] == '\0' ? "" : line + 3);
  } else if (strncmp(line, "kill! ", 6) == 0) {
    // ★'!' を要求する。fs format! / nc forget! と同じ作法 —— KILL_THREAD は
    //   BLE/UART/CDC のどの経路からも任意のスレッド ID を撃ててしまい
    //   (飛行制御スレッドも例外ではない)、打ち間違いで走ってよい操作ではない。
    //   対象の可否そのもの (自分自身/各コア最初の1本/デバッガ本体か) は
    //   カーネル側 (handler.cpp kill_thread) がその場で判定するので、
    //   シェル側で二重に絞り込みはしない (基準が変わったときのズレを避ける)。
    char *endptr = nullptr;
    unsigned long id = strtoul(line + 6, &endptr, 10);
    if (endptr == line + 6) {
      shell_printf("kill: usage: kill! <thread_id>\n");
    } else {
      const auto r = api(shizuku::object_api::KILL_THREAD, (uintptr_t)id);
      if (r.error == 0) {
        shell_printf("kill: thread %lu stopped\n", id);
      } else {
        // ★対応表は handler.cpp kill_thread が実際に設定する error だけを書く
        //   (推測で埋めない)。それ以外の番号は名前を出さず番号だけ見せる。
        const char *why = "";
        if (r.error == (uintptr_t)shizuku::object_error::NOT_PRIVILEGED)
          why = " (NOT_PRIVILEGED: 呼び手が特権でない/対象がデバッガ本体)";
        else if (r.error == (uintptr_t)shizuku::object_error::BAD_OBJECT)
          why = " (BAD_OBJECT: 範囲外 / 0 / 各コア最初の1本)";
        shell_printf("kill: error %lu%s\n", (unsigned long)r.error, why);
      }
    }
  } else if (strncmp(line, "kill ", 5) == 0) {
    // ★'!' 抜きは実行しない。打ち間違いで飛行制御スレッドを落とす事故を防ぐ。
    shell_printf("kill: '!' が要る (任意スレッドを強制停止できるため): "
                 "kill! <thread_id>\n");
  } else if (strcmp(line, "PING") == 0 || strcmp(line, "ping") == 0) {
    shell_printf("PONG\n");
  } else if (strncmp(line, "SAFE", 4) == 0) {
    // 外部安全装置 (Seeed XIAO) からの安全ハートビート / アラート。
    // 必要に応じて安全インターロックの反映が可能。
    // ★接頭辞を "SAFE" 4 文字だけで見る。以前は "SAFE," と "SAFE_" を
    //   個別に並べていたが、XIAO の Rust 化で 2 行目 `SAFE2,...` が増えた
    //   瞬間にどちらにも一致しなくなり、**10Hz で "unknown command" を
    //   返し続ける**状態になった。返事は UART0 と BLE の両方へ出るので、
    //   (1) 母艦の OTW スクリプトの drain() が永久に idle にならず固まる
    //   (2) 焼いている最中の BLE トラフィックが CYW43 を壊す
    //       (HANDOFF 2026-08-30 に「XIAO の定期送信で OTA が毎回死ぬ」として
    //        記録済みの形) という二重の壊れ方をする。
    // ★安全装置が名乗る行は**将来増える前提**で、種類ごとに列挙しない。
    //   ここは「相手の近況報告には黙って頷く」場所であって、
    //   語彙を検査する場所ではない。
  } else if (strncmp(line, "UBRIDGE ", 8) == 0) {
    // XIAO からの UART バイナリブリッジ開始要求 (BLE の代替 OTA 経路)。
    // a1 = これから生バイトで流れてくる総バイト数 (ota.hpp のヘッダ+本体)。
    // 書式: UBRIDGE <bytes> [baud]。baud 省略時は常用の速度のまま
    //   (古い母艦・古い XIAO とそのまま繋がる = 後方互換)。
    char *rest = nullptr;
    const uint32_t total = (uint32_t)strtoul(line + 8, &rest, 10);
    uint32_t baud = SHELL_UART_BAUD;
    if (rest != nullptr && *rest != '\0') {
      const uint32_t req = (uint32_t)strtoul(rest, nullptr, 10);
      // ★上下限を持つ。相手の打ち間違いで到達不能な速度にされると、
      //   こちらは戻す機会すら得られない (受け取れないので DONE も出せない)。
      if (req >= 115200u && req <= 3000000u)
        baud = req;
    }
    if (total > 0) {
      begin_uart_bridge(total, baud);
    }
  } else if (strcmp(line, "OTANEED") == 0) {
    // チャンク単位再送 (XNOR) のラウンド区切り。まだ受け取れていない seq の
    // 一覧を ota に吐かせる ("NEED n=.." / "NEEDSEQ .." / "NEEDEND")。
    // ★★これは**中継の外**、常用の 115200 で叩くための口。同じ問い合わせは
    //   帯域内 (seq=0xFFFF のチャンクヘッダ) でもできるが、OTW で攻めた baud を
    //   使うときは**返事そのものが化けたら再送機構ごと成立しない**。制御は
    //   必ず化けない速度で通す、というのがこの口を別に持つ理由。
    //   BLE 側は baud の問題が無いので帯域内で済ませてよい。
    using shizuku::objects::ota::method;
    api(shizuku::object_api::CALL_METHOD, xno_object_id::ota,
        (uintptr_t)method::GET_MISSING, 0);
  } else if (strcmp(line, "OTARESET") == 0) {
    // 走りかけの転送を捨てて待ち受けへ戻す。
    // ★★諦めた転送のあと ota は最大 2 分 CSEEK に居座る (チャンク再送の
    //   ラウンドの合間を守るための長いタイムアウト)。その間に次の転送を
    //   始めると、**XNOR のファイルヘッダをチャンクデータとして食う**ので
    //   先へ進まない。転送の頭でこれを撃てば必ず待ち受けから始まる。
    // ★中継の外 (常用 115200) から叩く口。同じことは帯域内の制御フレーム
    //   (seq=0xFFFE) でもできるが、UART の中継はバイト列が連続していて
    //   「フレームの最後に置く」を送り手が保証しにくい。有線ではこちらを使う。
    using shizuku::objects::ota::method;
    api(shizuku::object_api::CALL_METHOD, xno_object_id::ota,
        (uintptr_t)method::RESET, 0);
  } else if (strcmp(line, "help") == 0 || strcmp(line, "?") == 0) {
    shell_printf(
        "Commands: status, arm, disarm, pitch <deg>, head <deg>, alt <m>\n");
    shell_printf("          r<ms>, version, reboot, bootsel, ping\n");
    shell_printf("          fs [ls|stat|cat <name>|rm <name>|format!]\n");
    shell_printf("          nc [status|y|n|lock|allow|forget!]\n");
    shell_printf("          kill! <thread_id>\n");
    shell_printf("          UBRIDGE <bytes> [baud], OTANEED, OTARESET\n");
  } else {
    shell_printf("unknown command: %s (try 'help')\n", line);
  }
}

// Method 0: GET_STREAM
uintptr_t method_get_stream(uintptr_t, uintptr_t, uintptr_t, uintptr_t) {
  return g_out_id;
}

// Method 1: SET_RX_STREAM
uintptr_t method_set_rx_stream(uintptr_t stream_id, uintptr_t, uintptr_t,
                               uintptr_t) {
  g_rx_id = stream_id;
  return 0;
}

// Method 2: PROCESS_CMD
uintptr_t method_process_cmd(uintptr_t cmd_ptr, uintptr_t, uintptr_t,
                             uintptr_t) {
  if (cmd_ptr != 0) {
    handle_command_line((char *)cmd_ptr);
  }
  return 0;
}

// Method 5: SET_BLE_OTA_STREAM
uintptr_t method_set_ble_ota_stream(uintptr_t stream_id, uintptr_t, uintptr_t,
                                    uintptr_t) {
  g_ble_ota_stream_id = stream_id;
  return 0;
}

// メインポーリングループ (Core 0: BLE ストリーム ＆ CDC ＆ UART0 を監視)
uintptr_t poll_loop(uintptr_t, uintptr_t, uintptr_t, uintptr_t) {
  char cdc_line[128] = "";
  size_t cdc_pos = 0;
  char uart_line[128] = "";
  size_t uart_pos = 0;

  // ハードウェア UART0 初期化 (外部安全装置・XIAO との通信)
  uart_init(SHELL_UART, SHELL_UART_BAUD);
  gpio_set_function(SHELL_UART_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(SHELL_UART_RX_PIN, GPIO_FUNC_UART);
  gpio_pull_up(SHELL_UART_RX_PIN);
  gpio_pull_up(SHELL_UART_TX_PIN);
  uart_set_hw_flow(SHELL_UART, false, false);
  uart_set_format(SHELL_UART, 8, 1, UART_PARITY_NONE);
  uart_set_fifo_enabled(SHELL_UART, true);
  // ★DMA を張るのはここ (UART を叩けるようになった直後)。中で FIFO を切る。
  uart_rx_dma_start();
  {
    const auto created =
        api(shizuku::object_api::STREAM_CREATE, (uintptr_t)&g_uart_rx_desc);
    if (created.error == 0)
      g_uart_rx_stream_id = created.value;
  }

  // 出力ストリーム (logger への応答) を PRODUCER としてバインド
  if (g_out_id != 0) {
    api(shizuku::object_api::STREAM_BIND, g_out_id,
        (uintptr_t)shizuku::stream::role::PRODUCER);
  }
  // UART ブリッジ用ストリーム (ota への中継) も PRODUCER としてバインド
  if (g_uart_ota_id != 0) {
    api(shizuku::object_api::STREAM_BIND, g_uart_ota_id,
        (uintptr_t)shizuku::stream::role::PRODUCER);
  }

  // BLE RX ストリームのオープン ＆ バインド (ポーリングスレッド内で実行)
  if (g_rx_id != xno::NO_STREAM && !g_rx.valid()) {
    const auto opened = api(shizuku::object_api::STREAM_OPEN, g_rx_id);
    if (opened.error == 0 && opened.value != 0) {
      g_rx = shizuku::stream::handle<frame_t>(
          (shizuku::stream::descriptor *)opened.value);
      api(shizuku::object_api::STREAM_BIND, g_rx_id,
          (uintptr_t)shizuku::stream::role::CONSUMER);
      BOARD::diag_printf("[SHELL] bound to BLE RX stream %lu\n",
                         (unsigned long)g_rx_id);
    }
  }

  // ペアリングの問いかけを取りこぼさないための追跡。★通し番号を覚えるのは、
  //   「今 pending か」だけを見ていると、このループの周期 (最短 5ms、実際は
  //   他の仕事に取られる) の隙間に出て消えた要求を一度も表示できないため。
  uint32_t last_nc_generation = 0;
  uint32_t last_block_seconds = 0;
  uint64_t next_pairing_poll_us = 0;

  while (true) {
    bool did_work = false;

    // 0. ペアリング状態の見張り (200ms ごと)。
    //    ★NC の番号を **UART0 へ自分から流す**のがここの仕事。ble_uart から
    //      直接 UART へ書かせない理由は、UART0 に書き手が二人できると行が
    //      混ざるため (読み手を一人に保っているのと同じ理由)。
    //    ★焼いている最中はやらない。UART/BLE へ行を出すと、IRQ を止めている
    //      最中の BLE トラフィックが CYW43 を壊す (下の flash_busy と同じ話)。
    //    ★★中継中も**やらない**。ブリッジ中の UART0 は生のイメージが流れる
    //      専用線で、ここから行を書くと XIAO 経由で母艦の画面へ割り込む
    //      (向きが逆なので像は壊れないが、母艦の drain() を idle にさせない)。
    //      「焼いている間は黙る」のと同じ理由で、経路ごとに書き忘れないこと。
    if (BOARD::time_us() >= next_pairing_poll_us && !g_uart_bridge_active &&
        !shizuku::objects::ota::flash_busy()) {
      next_pairing_poll_us = BOARD::time_us() + 200000;
      ble::pairing_state ps{};
      if (ble_call(ble::method::GET_PAIRING_STATE, (uintptr_t)&ps).value != 0) {
        if (ps.nc_generation != last_nc_generation) {
          last_nc_generation = ps.nc_generation;
          if (ps.nc_pending) {
            did_work = true;
            shell_printf("NC,%06lu\n", (unsigned long)ps.nc_passkey);
            shell_printf("NC: 相手の表示と同じなら 'nc y'、違えば 'nc n'\n");
          }
        }
        // クールダウンに入った/明けたことは黙っていると原因不明の
        // 「見つからない」になるので、遷移だけ知らせる。
        if ((ps.block_seconds_left != 0) != (last_block_seconds != 0)) {
          shell_printf("NC: advertising %s\n",
                       ps.block_seconds_left ? "OFF (pairing cooldown)" : "ON");
        }
        last_block_seconds = ps.block_seconds_left;
      }
    }

    // 1. BLE RX ストリームからコマンドを受信
    if (!g_rx.valid() && g_rx_id != xno::NO_STREAM) {
      const auto opened = api(shizuku::object_api::STREAM_OPEN, g_rx_id);
      if (opened.error == 0 && opened.value != 0) {
        g_rx = shizuku::stream::handle<frame_t>(
            (shizuku::stream::descriptor *)opened.value);
        api(shizuku::object_api::STREAM_BIND, g_rx_id,
            (uintptr_t)shizuku::stream::role::CONSUMER);
      }
    }

    if (g_rx.valid()) {
      frame_t f{};
      uint32_t lost = 0;
      while (g_rx.pop(&f, &lost)) {
        did_work = true;
        char cmd_buf[sizeof(f.data) + 1];
        size_t copy_len = f.len < sizeof(f.data) ? f.len : sizeof(f.data) - 1;
        memcpy(cmd_buf, f.data, copy_len);
        cmd_buf[copy_len] = '\0';
        char *nl = strpbrk(cmd_buf, "\r\n");
        if (nl)
          *nl = '\0';
        handle_command_line(cmd_buf);
      }
    }

    // 2. CDC (USB stdio) からの入力
    int c = getchar_timeout_us(0);
    if (c != PICO_ERROR_TIMEOUT && c != PICO_ERROR_NO_DATA && c >= 0) {
      did_work = true;
      if (c == '\r' || c == '\n') {
        if (cdc_pos > 0) {
          cdc_line[cdc_pos] = '\0';
          printf("\r\n");
          handle_command_line(cdc_line);
          cdc_pos = 0;
        }
        printf("shizuku> ");
        fflush(stdout);
      } else if (c == 0x08 || c == 0x7F) { // Backspace
        if (cdc_pos > 0) {
          --cdc_pos;
          printf("\b \b");
          fflush(stdout);
        }
      } else if (cdc_pos + 1 < sizeof(cdc_line) && c >= 32 && c <= 126) {
        cdc_line[cdc_pos++] = (char)c;
        putchar(c);
        fflush(stdout);
      }
    }

    // 3. UART0 (Seeed XIAO 等の安全装置) からのコマンド受信
    //    ★ブリッジ中は「行として解釈する」のをやめ、生バイトをそのまま
    //      ota の入力ストリームへ積む (二人読みを避けるため、UART0 の
    //      リーダーはここ一つのまま、バイトの行き先だけを切り替える)。
    if (g_uart_bridge_active) {
      while (g_uart_bridge_remaining > 0 && uart_rx_available() != 0) {
        const int got = uart_rx_getc();
        if (got < 0)
          break;
        did_work = true;
        uint8_t b = (uint8_t)got;
        if (abort_magic_seen(b)) {
          // 相手が降りた。数え終わるのを待たずに畳む (待つと居座る)。
          uart_puts(SHELL_UART, "UBRIDGE_ABORTED\n");
          end_uart_bridge();
          break;
        }
        uart_bridge_push_byte(b);
        --g_uart_bridge_remaining;
        g_uart_bridge_last_us = BOARD::time_us();
      }
      if (!g_uart_bridge_active)
        goto bridge_done;
      // ★★無通信で抜ける道を必ず持つこと。母艦や XIAO が途中で諦めると、
      //   ここは「残りバイトを待ち続ける」ので永久に中継のまま居座る。
      //   その間 UART はコマンドを受け付けず、しかも ota の入力が UART 側を
      //   向いたままなので **BLE OTA も効かない** —— 回復手段を増やすための
      //   機能が、失敗すると回復手段を全部塞ぐ (実際に踏んだ)。
      //   抜けるときは end_uart_bridge() を必ず通し、入力を BLE へ戻す。
      if (g_uart_bridge_remaining == 0) {
        end_uart_bridge();
      } else if (BOARD::time_us() - g_uart_bridge_last_us >
                 (uint64_t)UART_BRIDGE_IDLE_US) {
        uart_puts(SHELL_UART, "UBRIDGE_TIMEOUT\n");
        end_uart_bridge();
      }
    bridge_done:;
    } else if (shizuku::objects::ota::flash_busy()) {
      // ★★焼いている間は UART の行を**解釈しない**。解釈すると知らない
      //   コマンドへの返事が BLE の TX ストリームへ出て、IRQ を止めている
      //   最中の BLE トラフィックが CYW43 を壊す (2026-08-30 に 3 回踏んだ:
      //   安全装置 XIAO の定期送信がきっかけで OTA が毎回死んだ)。
      //   捨てるのではなく**読み飛ばす**だけにして、FIFO の溢れも防ぐ。
      while (uart_rx_available() != 0)
        (void)uart_rx_getc();
      uart_pos = 0;
    } else {
      while (uart_rx_available() != 0) {
        const int got = uart_rx_getc();
        if (got < 0)
          break;
        did_work = true;
        char ch = (char)got;
        if (ch == '\r' || ch == '\n') {
          if (uart_pos > 0) {
            uart_line[uart_pos] = '\0';
            handle_command_line(uart_line);
            uart_pos = 0;
          }
        } else if (ch == 0x08 || ch == 0x7F) {
          if (uart_pos > 0) {
            --uart_pos;
          }
        } else if (uart_pos + 1 < sizeof(uart_line) && (uint8_t)ch >= 32 && (uint8_t)ch <= 126) {
          uart_line[uart_pos++] = ch;
        }
        if (g_uart_bridge_active)
          break; // UBRIDGE がこの行で発火した場合、残りは次周回でブリッジ経路へ
      }
    }

    if (!did_work) {
      api(shizuku::object_api::SLEEP_US, 5000); // 5ms
    }
  }
  return 0;
}

uintptr_t shell_main(uintptr_t, uintptr_t, uintptr_t, uintptr_t) {
  uintptr_t failures =
      api(shizuku::object_api::DECLARE_NAME, (uintptr_t)"shizuku_shell").error;
  failures += export_method(method::MAIN, (uintptr_t)&shell_main);
  failures +=
      export_method(method::SET_RX_STREAM, (uintptr_t)&method_set_rx_stream);
  failures += export_method(method::GET_STREAM, (uintptr_t)&method_get_stream);
  failures +=
      export_method(method::PROCESS_CMD, (uintptr_t)&method_process_cmd);
  failures += export_method(method::POLL, (uintptr_t)&poll_loop);
  failures += export_method(method::SET_BLE_OTA_STREAM,
                            (uintptr_t)&method_set_ble_ota_stream);

  g_out.init();
  const auto created =
      api(shizuku::object_api::STREAM_CREATE, (uintptr_t)&g_out.desc);
  failures += created.error;
  g_out_id = created.value;

  g_uart_ota.init();
  const auto uart_ota_created =
      api(shizuku::object_api::STREAM_CREATE, (uintptr_t)&g_uart_ota.desc);
  failures += uart_ota_created.error;
  g_uart_ota_id = uart_ota_created.value;
  return failures;
}

} // namespace

uint32_t register_shell(uintptr_t obj_id) {
  const auto created =
      api(shizuku::object_api::CREATE_OBJECT, obj_id, (uintptr_t)&shell_main,
          shizuku::OBJECT_PRIVILEGED | shizuku::OBJECT_ON_CORE(0));
  if (created.error != 0)
    return 0;
  const auto inited = api(shizuku::object_api::CALL_METHOD, obj_id, 0, 0);
  return inited.error == 0 ? 1 : 0;
}

uint32_t start_shell(uintptr_t obj_id) {
  const auto res =
      api(shizuku::object_api::SPAWN, obj_id, (uintptr_t)method::POLL, 0);
  return (uint32_t)res.value;
}

} // namespace xno::shell
