// ===========================================================================
//  user_app.cpp — Flash FS から動的ロードされるユーザーランド Shizuku オブジェクト
// ===========================================================================
#include "shizuku/object_api.hpp"
#include <cstdint>

namespace user_app {
uintptr_t app_main(uintptr_t, uintptr_t, uintptr_t, uintptr_t);
}

// 動的オブジェクト用エントリポイント (ファイルの先頭 = オフセット 0 に配置)
extern "C" __attribute__((section(".text.entry"))) uintptr_t dynamic_module_main(uintptr_t a1, uintptr_t a2, uintptr_t a3, uintptr_t a4) {
  return user_app::app_main(a1, a2, a3, a4);
}

namespace user_app {
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

// Method 1: 計算メソッド (純粋関数: a * b + 300)
uintptr_t method_compute(uintptr_t a, uintptr_t b, uintptr_t, uintptr_t) {
  return a * b + 300;
}

} // namespace

// Method 0: オブジェクトメインスレッド (エントリポイント)
uintptr_t app_main(uintptr_t, uintptr_t, uintptr_t, uintptr_t) {
  api(shizuku::object_api::DECLARE_NAME, (uintptr_t)"user_app_v3");
  api(shizuku::object_api::EXPORT_METHOD, 1 /* COMPUTE */, ((uintptr_t)&method_compute) | 1u);

  // メインループ: 1秒ごとにスリープ
  for (uint32_t count = 0; count < 1000; ++count) {
    api(shizuku::object_api::SLEEP_US, 1000000);
  }
  api(shizuku::object_api::EXIT_THREAD);
  return 0;
}

} // namespace user_app
