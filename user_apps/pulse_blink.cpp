#include <cstdint>
#include "shizuku/object_api.hpp"

#ifdef SHIZUKU_DYNAMIC_MODULE
namespace shizuku::objects {
constexpr uintptr_t LED_OBJECT = 5;
enum struct led_method : uintptr_t { WRITE = 1 };
struct led_request { uint32_t value; };
}
#endif

namespace pulse_blink {
namespace {

struct api_result {
  uintptr_t error;
  uintptr_t value;
};

static inline api_result api(shizuku::object_api number, uintptr_t a1 = 0,
                             uintptr_t a2 = 0, uintptr_t a3 = 0,
                             uintptr_t a4 = 0) {
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

static inline void set_led_hw(bool on) {
  shizuku::objects::led_request req{on ? 1u : 0u};
  api(shizuku::object_api::CALL_METHOD, shizuku::objects::LED_OBJECT,
      (uintptr_t)shizuku::objects::led_method::WRITE, (uintptr_t)&req);
}

volatile bool g_running = false;
volatile uint32_t g_count = 0;

// 5連パルス・コズミックビーコン (20ms ON / 40ms OFF × 5回 → 600ms OFF)
uintptr_t poll_loop(uintptr_t, uintptr_t, uintptr_t, uintptr_t) {
  g_running = true;

  while (g_running) {
    // 5 連フラッシュ
    for (int i = 0; i < 5 && g_running; ++i) {
      set_led_hw(true);
      api(shizuku::object_api::SLEEP_US, 20 * 1000);
      set_led_hw(false);
      api(shizuku::object_api::SLEEP_US, 40 * 1000);
      ++g_count;
    }
    // 長インターバル
    if (g_running) {
      api(shizuku::object_api::SLEEP_US, 600 * 1000);
    }
  }

  set_led_hw(false);
  api(shizuku::object_api::EXIT_THREAD);
  return 0;
}

uintptr_t method_stop(uintptr_t, uintptr_t, uintptr_t, uintptr_t) {
  g_running = false;
  set_led_hw(false);
  return 1;
}

uintptr_t method_get_count(uintptr_t, uintptr_t, uintptr_t, uintptr_t) {
  return g_count;
}

uintptr_t pulse_main(uintptr_t, uintptr_t, uintptr_t, uintptr_t) {
  api(shizuku::object_api::DECLARE_NAME, (uintptr_t)"pulse_blink");
  api(shizuku::object_api::EXPORT_METHOD, 1 /* POLL */, (uintptr_t)&poll_loop);
  api(shizuku::object_api::EXPORT_METHOD, 5 /* STOP */, (uintptr_t)&method_stop);
  api(shizuku::object_api::EXPORT_METHOD, 6 /* GET_COUNT */, (uintptr_t)&method_get_count);
  return 0;
}

} // namespace
} // namespace pulse_blink

// 動的オブジェクト用エントリポイント
extern "C" __attribute__((section(".text.entry"))) uintptr_t
dynamic_module_main(uintptr_t a1, uintptr_t a2, uintptr_t a3, uintptr_t a4) {
  return pulse_blink::pulse_main(a1, a2, a3, a4);
}
