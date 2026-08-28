#ifndef FLIGHT_ROBOCON_BLE_UART_HPP
#define FLIGHT_ROBOCON_BLE_UART_HPP
#include "shizuku/objects/ble_uart.hpp"
#include "object_ids.hpp"
#include "tx_frame.hpp"

// ===========================================================================
//  BLE UART (Nordic UART Service) — Shizuku オブジェクト版 (ラッパー)
// ===========================================================================
//  本実装は Shizuku リポジトリ (@shizuku//modules/pico_sdk_support:ble_uart) に
//  アップストリーム統合されました。
namespace ble_uart {

using namespace shizuku::objects::ble_uart;

constexpr uintptr_t OBJECT = xno_object_id::ble_uart;

inline uint32_t register_ble_uart() {
  return shizuku::objects::ble_uart::register_ble_uart(OBJECT);
}

inline uint32_t start_ble_uart() {
  return shizuku::objects::ble_uart::start_ble_uart(OBJECT);
}

} // namespace ble_uart
#endif // FLIGHT_ROBOCON_BLE_UART_HPP
