#ifndef FLIGHT_ROBOCON_OBJECT_IDS_HPP
#define FLIGHT_ROBOCON_OBJECT_IDS_HPP
#include <cstdint>

// ★番号を手で書かない (Shizuku D28)。Shizuku 側が使い終えた次から始める。
//   `count` はビルドシステムが生成する値なので、Shizuku にオブジェクトが
//   増えても勝手に後ろへずれる。**ここに数字を書いた瞬間に衝突する** —
//   実際 `base = 32` と書いた版は `shizuku::object_id::debug_target` (= 32)
//   と ble_uart がぶつかっていた。
#include "shizuku/object_ids.hpp"
#include "shizuku/config.hpp"

namespace xno_object_id {
constexpr uintptr_t base = shizuku::object_id::count;

constexpr uintptr_t ble_uart = base + 0;
constexpr uintptr_t bno055 = base + 1;
constexpr uintptr_t bme280 = base + 2;
constexpr uintptr_t flight_controller = base + 3;
constexpr uintptr_t blink = base + 4;
constexpr uintptr_t telemetry = base + 5;
constexpr uintptr_t logger = base + 6;
constexpr uintptr_t ota = base + 7;
constexpr uintptr_t shell = base + 8;

// 動的にロードするモジュールへ配る番号。★上限はカーネルの表の大きさで決まる
//   (SHIZUKU_OBJECT_COUNT)。ここを手で書くと表からはみ出した番号を配って
//   CREATE_OBJECT が BAD_OBJECT で黙って失敗する (dyn_obj_end = 63 と
//   書いてあった版が実際にそれ)。
constexpr uintptr_t dyn_obj_start = base + 9;
constexpr uintptr_t dyn_obj_end = shizuku::KERNEL_OBJECT::OBJECT_COUNT - 1;

static_assert(dyn_obj_start <= dyn_obj_end,
              "オブジェクト表が XNO のオブジェクトで埋まっている "
              "(configs/BUILD.bazel の SHIZUKU_OBJECT_COUNT を増やすこと)");

} // namespace xno_object_id
#endif // FLIGHT_ROBOCON_OBJECT_IDS_HPP
