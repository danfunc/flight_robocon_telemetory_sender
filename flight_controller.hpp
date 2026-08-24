#ifndef FLIGHT_ROBOCON_FLIGHT_CONTROLLER_HPP
#define FLIGHT_ROBOCON_FLIGHT_CONTROLLER_HPP
#include <cstdint>
#include "object_ids.hpp"
#include "tx_frame.hpp"

// ===========================================================================
//  flight_controller — 上りコマンドの解釈と機体の判断
// ===========================================================================
//  ★仕事はひとつ: **コマンドを読んで決める**。センサの符号化 (telemetry) も、
//    行の合流 (logger) も、送受信 (ble_uart) も持たない。
//    入口 = RX の行ストリーム、出口 = 応答の行ストリーム。
//
//  ★役の分け方 (UNIX 哲学。ストリームがパイプの役をする):
//      bno055 ──samples──┐
//                         ├→ [telemetry] ──lines──┐
//      bme280 ──samples──┘                         ├→ [logger] ─→ [ble_uart]
//                                                  │
//      ble_uart ──rx──→ [flight_controller] ──lines┘
//    どのリンクも producer 1 / consumer 1 のまま (D46)。
//
//  ★これ以前は各センサが ble_uart::SEND を同期 CALL_METHOD で叩いていた。
//    呼び出し元スレッドで走るため btstack と再入して CYW43 の SPI バスを壊し
//    (実機で `Bus error 0x81` / `cybt_bus_request` 停止)、TX リングも
//    producer 3 つで無音欠落していた。この分割はその両方を構造的に消す。
//
//  ★v1 のスコープはコマンド解釈まで。**姿勢制御・ミキサ・セーフティは無い** —
//    旧実装の FLIGHT_CONTROLLER 相当を載せるならこの object を育てる
//    (センサ値が要るようになったら telemetry の出力ではなく、センサの
//    サンプルストリームを直接もう 1 本購読する形にすること — 符号化済みの
//    テキストを読み返すのは筋が悪い)。
namespace flight_controller {

constexpr uintptr_t OBJECT = xno_object_id::flight_controller;

enum struct method : uintptr_t {
  MAIN = 0,
  // a0 = ble_uart の RX ストリーム番号。
  SET_RX_STREAM = 1,
  // 戻り値 = 応答行のストリーム番号 (logger が購読する)。
  GET_STREAM = 2,
  POLL = 3,
};

using frame_t = xno::tx_frame;

uint32_t register_flight_controller();
uint32_t start_flight_controller();

} // namespace flight_controller
#endif // FLIGHT_ROBOCON_FLIGHT_CONTROLLER_HPP
