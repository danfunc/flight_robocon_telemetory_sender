#ifndef SHIZU_BLE_TX_STREAM_HPP
#define SHIZU_BLE_TX_STREAM_HPP
// ===========================================================================
//  BLE TX の優先度付きマルチストリーム — フレーム型と producer ヘルパ
// ===========================================================================
//  単一 FIFO バイトリングの head-of-line blocking (ping 応答がバルクテレメトリの
//  後ろに並び最悪 ~100ms 待つ) を解消するため、Shizuku ストリーム抽象で N 本の
//  優先度付き TX ストリームに置換する。BLE_UART が「マルチストリーム consumer」に
//  なり、CAN_SEND_NOW ごとに空でない最高優先度ストリームから先に排出する。
//
//  priority は consumer(BLE_UART) 側のポリシーなので stream_desc_t にもカーネルにも
//  持たせない (register_tx_stream で BLE_UART のローカル表に載せる)。
// ===========================================================================
#include <cstdint>
#include <cstring>
#include <stream.hpp>

namespace shizu {
namespace ble_tx {

// 1 notify = 1 LL PDU の上限 (MTU 247 - 3 = 244)。★CYW43 wedge 回避の不可侵規則★。
constexpr uint32_t FRAME_MAX = 244;
// frame_t::len の bit15: 後続フラグメントあり (同一メッセージが次レコードへ続く)。
// consumer はこのビットが立っている間、同一ストリームから連続排出し他ストリームへ
// 切り替えない (host のバイトパーサへメッセージ途中で別ストリームが割り込む破損を防ぐ)。
constexpr uint16_t MORE = 0x8000;
constexpr uint16_t LEN_MASK = 0x7FFF;

// TX ストリームの 1 レコード = 1 notify 分のフレーム。
struct frame_t {
  uint16_t len; // bit15=MORE, 下位=バイト数 (0..244)
  uint8_t data[FRAME_MAX];
};

// TX ストリーム ID (stream_selftest の id0 と衝突しない 1..)。
constexpr uint32_t STREAM_BULK = 1; // 通常テレメトリ (低優先)
constexpr uint32_t STREAM_CTRL = 2; // ping/stats 等の制御応答 (高優先)

// consumer(BLE_UART) がストリームを排出する優先度。大きいほど先に排出。
constexpr uint32_t PRIO_CTRL = 200; // 制御応答 (最優先)
constexpr uint32_t PRIO_SYS = 100;  // BLE_UART 内部行 (RSSI/status)
constexpr uint32_t PRIO_BULK = 10;  // 通常テレメトリ

using handle_t = stream::handle<frame_t>;

// ---- ctrl レイテンシ計装 (ping >100ms の device/host 切り分け用) ------------
// producer (TELEMETRY の ble_send_prio) が ctrl フレームを push した時刻を stamp し、
// consumer (BLE_UART の flush_tx) が ctrl フレームを notify に載せた時点で
// 「device 内滞在時間」を計算する。BLE_UART が 1s の tx 統計行に last/max を出す。
// device 滞在が数十 ms 未満なのに GUI の RTT が 100ms 超なら、犯人は host 側で確定。
extern volatile uint64_t ctrl_enq_us;      // 最後に ctrl へ push した時刻 (0=なし)
extern volatile uint32_t ctrl_lat_last_us; // 直近の滞在時間
extern volatile uint32_t ctrl_lat_max_us;  // 1s 窓の最大 (印字時にリセット)

// メッセージ (任意長バイト列) を ≤244B フレームに分割して push する。全フラグメントの
// 空きが無ければ 1 個も push せず false を返す (部分メッセージを絶対に出さない =
// lossless 丸ごと破棄)。producer は単一かつ協調原子 (書き込み中 yield しない) なので、
// writable_slots の事前チェックは push 完了まで有効 (consumer は空きを増やす方向にしか
// 動かない)。戻り false = 丸ごとドロップ (呼び出し側でカウントする)。
inline bool push_msg(handle_t h, const uint8_t *p, uint32_t total) {
  if (!h.valid())
    return false;
  if (total == 0)
    return true; // 空メッセージは no-op (空 notify を送らない)
  const uint32_t frames = (total + FRAME_MAX - 1) / FRAME_MAX;
  if (h.writable_slots() < frames)
    return false; // 空き不足: 丸ごと破棄
  uint32_t off = 0;
  for (uint32_t i = 0; i < frames; ++i) {
    frame_t f;
    uint32_t n = total - off;
    if (n > FRAME_MAX)
      n = FRAME_MAX;
    f.len = (uint16_t)n | ((i + 1 < frames) ? MORE : (uint16_t)0);
    memcpy(f.data, p + off, n);
    h.push(f); // 事前チェック済みにつき必ず成功 (LOSSLESS)
    off += n;
  }
  return true;
}

} // namespace ble_tx
} // namespace shizu
#endif // SHIZU_BLE_TX_STREAM_HPP
