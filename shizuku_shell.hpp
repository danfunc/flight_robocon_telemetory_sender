#ifndef FLIGHT_ROBOCON_SHIZUKU_SHELL_HPP
#define FLIGHT_ROBOCON_SHIZUKU_SHELL_HPP

#include "object_ids.hpp"
#include <cstdint>

namespace xno::shell {

// シェルオブジェクト ID
constexpr uintptr_t OBJECT = xno_object_id::shell;

// シェルの登録 (オブジェクト生成 ＆ export)
uint32_t register_shell(uintptr_t obj_id = OBJECT);

// シェルスレッドの起動 (CDC Channel 0 でポーリング開始)
uint32_t start_shell(uintptr_t obj_id = OBJECT);

} // namespace xno::shell

#endif // FLIGHT_ROBOCON_SHIZUKU_SHELL_HPP
