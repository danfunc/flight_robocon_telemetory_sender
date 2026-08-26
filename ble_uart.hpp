#ifndef FLIGHT_ROBOCON_BLE_UART_HPP
#define FLIGHT_ROBOCON_BLE_UART_HPP
#include <cstdint>
#include "object_ids.hpp"
#include "tx_frame.hpp"

// ===========================================================================
//  BLE UART (Nordic UART Service) — Shizuku オブジェクト版
// ===========================================================================
//  旧実装 (flight_robocon_telemetory_sender/BLE_UART_DRIVER.cpp, CMake ビルド)
//  からのポート。範囲は「BLE トランスポートだけ」— advertise / pairing (NC) /
//  notify・write / CI 強制。コマンド解釈 (shell 的な機能) はここには置かず、
//  RX は Shizuku ストリームとして外へ出すだけ (別オブジェクトが購読する)。
//
//  ★affinity は指定しない (core0 起点)。当初 core1 化を予定していたが、
//    peripherals.cpp の led_main() が pico2_w で既に cyw43_arch_init() を
//    core0 から呼んでおり (LED が CYW43 チップの GPIO 配下)、CYW43 は
//    「初期化したコアからしか触れない」という実測済みの制約
//    (shizuku/object_api.hpp の OBJECT_ON_CORE コメント参照) と衝突するため、
//    今回は core1 化を見送った (ユーザー判断)。将来 core1 化を再検討する
//    ときは、まず LED オブジェクト側を cyw43 直叩きから抽象化すること。
namespace ble_uart {

constexpr uintptr_t OBJECT = xno_object_id::ble_uart;

enum struct method : uintptr_t {
  MAIN = 0,
  // 戻り値 = RX ストリームの番号 (STREAM_OPEN で購読する)。まだ接続前でも
  // 呼んでよい (ストリーム自体は init 完了後すぐに存在する)。
  GET_RX_STREAM = 1,
  // a0 = TX ストリームの番号。ここへ流れてきたフレームを notify で送る。
  // ★席は 1 つ (SPSC) なので、送り手は**ちょうど 1 オブジェクト**
  //   (flight_controller = ハブ)。合成側が起動前に一度だけ渡す。
  SET_TX_STREAM = 2,
  // cyw43_arch_poll ループ本体。SPAWN 専用 — CALL_METHOD で直接呼ばないこと
  // (無限ループなので戻ってこない)。
  POLL = 3,
  // a0 = (stub へ送る番号 << 16) | (stub から受ける番号)。
  // ★GDB (RSP) を**専用の characteristic** で運ぶための配線。NUS と分ける理由は
  //   ble_uart.gatt のコメント参照 (RSP に CSV が挟まると握手ごと壊れる)。
  SET_GDB_STREAMS = 4,
  // 戻り値 = OTA 受信ストリームの番号。OTA 専用 characteristic への write を
  // そのまま流す (中身は解釈しない)。★向きは RX と同じ —
  // **ble_uart が producer**、ota が consumer。
  GET_OTA_STREAM = 5,
  // リンクを切ってほしい、という**要求だけ**を置く。実際に gap_disconnect を
  // 呼ぶのは poll ループ。★ここで直接 btstack を突いてはいけない —
  // 呼び出し元スレッドから触ると poll ループと再入して CYW43 の SPI バスを
  // 壊す (下の SEND 廃止と同じ理由)。
  // ★用途: OTA の commit。両コアを 1.7 秒止めて flash を焼く間、BLE を
  //   繋いだままにしておくと CYW43 が道連れになる。
  REQUEST_DISCONNECT = 6,
};

// ★かつて SEND (同期 CALL_METHOD で 1 メッセージ積む) を export していたが
//   **廃止した** (2026-08-24)。CALL_METHOD は呼び出し元スレッドで走るので、
//   (a) その中から btstack を突くと poll ループと再入して CYW43 の SPI バスを
//   壊し (実機で `Bus error 0x81` / `cybt_bus_request` 停止)、(b) TX リングへの
//   push も producer が複数になって D46 のモデルに反する。送り手はストリームで
//   繋ぐこと — 送りたいオブジェクトは自前のストリームを持ち、flight_controller
//   (ハブ) がそれらを束ねて 1 本にしてここへ流す。

// 1 メッセージの実体は中立ヘッダ側に置いてある (tx_frame.hpp のコメント参照)。
using frame_t = xno::tx_frame;

// 合成側 (ブート後のスレッドモード) から呼ぶ。オブジェクトを生成し、init を
// 一度呼んでメソッドを export させ、RX ストリームを作り、BT スタックを起動する。
// ★poll スレッドはまだ起こさない — 起こすのは配線 (SET_TX_STREAM) が
//   済んでから start_ble_uart() で。
// 戻り = 失敗した手数 (0 なら成功)。戻り値は捨てないこと (D12)。
uint32_t register_ble_uart();

// 配線後に呼ぶ。poll スレッドを SPAWN し、budget 0 (バトン) を与える。
uint32_t start_ble_uart();

} // namespace ble_uart
#endif // FLIGHT_ROBOCON_BLE_UART_HPP
