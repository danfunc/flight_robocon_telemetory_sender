#ifndef FLIGHT_ROBOCON_LOGGER_HPP
#define FLIGHT_ROBOCON_LOGGER_HPP
#include <cstdint>
#include "object_ids.hpp"
#include "tx_frame.hpp"

// ===========================================================================
//  logger — 出ていく行の合流点 (syslogd と同じ役)
// ===========================================================================
//  ★仕事はひとつ: **複数の行ソースを 1 本に束ねる**。行の中身は解釈しない
//    (誰が作ったかも、どこへ出るかも知らない)。
//
//  ★なぜ独立した object なのか:
//    (1) ストリームの席は producer 1 / consumer 1 (D46)。テレメトリと
//        コマンド応答という**2 つの出し手**が居る以上、どこかで束ねる必要が
//        あり、束ねる方針 (どれを先に出すか・溢れたらどれを捨てるか) は
//        object の中に置くのが Shizuku の分担 (方針をカーネルにも stream
//        ライブラリにも置かない = D1)。
//    (2) トランスポート (ble_uart) に束ねさせると、転送が「送り手が何者か」を
//        知ることになる。ble_uart は入力ストリーム 1 本だけを見る素の
//        トランスポートのままにしておきたい。
//    (3) **出ていく行が全部ここを通る**ので、flash へのログ (計画 Phase E) は
//        ここに tee を足すだけで済む。出口を増やすたびに全 object を触る、が
//        起きない。
//
//  ★優先度: 数が**小さい方が先**に出る。コマンド応答 (対話の待ち時間が
//    一番目立つ) を、量の多いテレメトリより先に出すために使う — 旧実装
//    (BLE_UART_DRIVER.cpp) が ctrl をバルクより優先していたのと同じ判断。
namespace logger {

constexpr uintptr_t OBJECT = xno_object_id::logger;

constexpr uint32_t MAX_INPUTS = 4;

// 優先度の目安 (小さいほど先)。
constexpr uint32_t PRIO_CONTROL = 0; // コマンド応答・状態通知
constexpr uint32_t PRIO_BULK = 10;   // テレメトリ

enum struct method : uintptr_t {
  MAIN = 0,
  // a0 = (ストリーム番号 << 16) | 優先度。★旧実装 register_tx_stream と同じ
  //      詰め方 (BLE_UART_DRIVER.cpp)。
  ADD_INPUT = 1,
  // 戻り値 = 束ねた行のストリーム番号。
  GET_STREAM = 2,
  POLL = 3,
};

using frame_t = xno::tx_frame;

inline constexpr uintptr_t pack_input(uintptr_t stream_id, uint32_t priority) {
  return (stream_id << 16) | (priority & 0xFFFFu);
}

uint32_t register_logger();
uint32_t start_logger();

} // namespace logger
#endif // FLIGHT_ROBOCON_LOGGER_HPP
