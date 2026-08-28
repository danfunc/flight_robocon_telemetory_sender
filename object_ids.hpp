#ifndef FLIGHT_ROBOCON_OBJECT_IDS_HPP
#define FLIGHT_ROBOCON_OBJECT_IDS_HPP

#ifndef SHIZUKU_DYNAMIC_MODULE
#include "shizuku/object_ids.hpp"
namespace xno_object_id {
constexpr uintptr_t base = shizuku::object_id::count;
#else
namespace xno_object_id {
constexpr uintptr_t base = 16;
#endif

constexpr uintptr_t ble_uart = base + 0;
constexpr uintptr_t bno055 = base + 1;
constexpr uintptr_t bme280 = base + 2;
constexpr uintptr_t flight_controller = base + 3;
constexpr uintptr_t blink = base + 4;
constexpr uintptr_t telemetry = base + 5;
constexpr uintptr_t logger = base + 6;
constexpr uintptr_t ota = base + 7;
constexpr uintptr_t shell = base + 8;

constexpr uintptr_t dyn_obj_start = base + 9;
constexpr uintptr_t dyn_obj_end = 31;

} // namespace xno_object_id
#endif // FLIGHT_ROBOCON_OBJECT_IDS_HPP
