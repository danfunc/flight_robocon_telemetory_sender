#ifndef FLIGHT_ROBOCON_SHIZUKU_SHELL_HPP
#define FLIGHT_ROBOCON_SHIZUKU_SHELL_HPP

#include "object_ids.hpp"
#include <cstdint>

namespace xno::shell {

// シェルオブジェクト ID
constexpr uintptr_t OBJECT = xno_object_id::shell;

enum struct method : uintptr_t {
  MAIN = 0,
  SET_RX_STREAM = 1, // a1 = ble_uart の RX ストリーム ID
  GET_STREAM = 2,    // 戻り値 = 応答用 frame_t ストリーム ID
  PROCESS_CMD = 3,   // a1 = コマンド文字列ポインタ
  POLL = 4,
  SET_BLE_OTA_STREAM = 5, // a1 = ota の入力を BLE 側へ戻すためのストリームID
};

// シェルの登録 (オブジェクト生成 ＆ export)
uint32_t register_shell(uintptr_t obj_id = OBJECT);

// シェルスレッドの起動 (BLE/CDC でポーリング開始)
uint32_t start_shell(uintptr_t obj_id = OBJECT);

} // namespace xno::shell

#endif // FLIGHT_ROBOCON_SHIZUKU_SHELL_HPP
