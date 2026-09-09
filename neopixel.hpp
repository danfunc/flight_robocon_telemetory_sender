#ifndef XNO_NEOPIXEL_HPP
#define XNO_NEOPIXEL_HPP
#include "object_ids.hpp"
#include <cstdint>

namespace xno::neopixel {
constexpr uintptr_t OBJECT = xno_object_id::neopixel;
// 5V給電、3.3V->5Vレベル変換(AHCT等)、共通GND接続を前提とする。詳細は docs/NEOPIXEL.md 参照。
constexpr unsigned GPIO_PIN = 16;
enum class method : uintptr_t { MAIN = 0, POLL = 1, SET_INPUT_STREAM = 2 };
uint32_t register_neopixel(uintptr_t obj_id = OBJECT);
uint32_t start_neopixel(uintptr_t obj_id = OBJECT);
} // namespace xno::neopixel
#endif
