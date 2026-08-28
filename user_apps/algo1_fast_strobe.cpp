// ===========================================================================
//  algo1_fast_strobe.cpp — 6連超高速ハイパーストロボ動的モジュール
// ===========================================================================
#include "shizuku/object_api.hpp"
#include <cstdint>

namespace algo1 {
uintptr_t app_main(uintptr_t, uintptr_t, uintptr_t, uintptr_t);
}

extern "C" __attribute__((section(".text.entry"))) uintptr_t
dynamic_module_main(uintptr_t a1, uintptr_t a2, uintptr_t a3, uintptr_t a4) {
  return algo1::app_main(a1, a2, a3, a4);
}

namespace algo1 {
namespace {

constexpr uintptr_t BLINK_OBJECT = 20; // xno_object_id::blink
constexpr uintptr_t LED_OBJECT = 5;
enum struct led_method : uintptr_t { WRITE = 1 };
struct led_request { uint32_t value; };

struct api_result {
  uintptr_t error;
  uintptr_t value;
};

__attribute__((always_inline)) static inline api_result
api(shizuku::object_api number, uintptr_t a1 = 0, uintptr_t a2 = 0,
    uintptr_t a3 = 0, uintptr_t a4 = 0) {
  register uintptr_t r0 asm("r0") = (uintptr_t)number;
  register uintptr_t r1 asm("r1") = a1;
  register uintptr_t r2 asm("r2") = a2;
  register uintptr_t r3 asm("r3") = a3;
  register uintptr_t r12 asm("r12") = a4;
  asm volatile("svc 0"
               : "+r"(r0), "+r"(r1)
               : "r"(r2), "r"(r3), "r"(r12)
               : "memory");
  return {r0, r1};
}

static inline void set_led(bool on) {
  led_request req{on ? 1u : 0u};
  api(shizuku::object_api::CALL_METHOD, LED_OBJECT,
      (uintptr_t)led_method::WRITE, (uintptr_t)&req);
}

// Method 5: 停止 (LED 消灯)
uintptr_t method_stop(uintptr_t, uintptr_t, uintptr_t, uintptr_t) {
  set_led(false);
  return 1;
}

} // namespace

// Method 1: メインループ (SPAWN または直接呼び出し用)
uintptr_t poll_loop(uintptr_t, uintptr_t, uintptr_t, uintptr_t) {
  while (true) {
    for (int i = 0; i < 6; ++i) {
      set_led(true);
      api(shizuku::object_api::SLEEP_US, 30000);
      set_led(false);
      api(shizuku::object_api::SLEEP_US, 30000);
    }
    api(shizuku::object_api::SLEEP_US, 400000);
  }
  return 0;
}

// Method 0: 初期化エントリポイント
uintptr_t app_main(uintptr_t, uintptr_t, uintptr_t, uintptr_t) {
  // 1. 組み込み blink を RAM 上で停止
  api(shizuku::object_api::CALL_METHOD, BLINK_OBJECT, 3 /* SET_ENABLED */, 0);
  api(shizuku::object_api::CALL_METHOD, BLINK_OBJECT, 5 /* STOP */, 0);

  // 2. メソッドエクスポート
  api(shizuku::object_api::DECLARE_NAME, (uintptr_t)"algo1_fast");
  api(shizuku::object_api::EXPORT_METHOD, 1 /* POLL */, (uintptr_t)&poll_loop);
  api(shizuku::object_api::EXPORT_METHOD, 5 /* STOP */, (uintptr_t)&method_stop);
  return 0;
}

} // namespace algo1
