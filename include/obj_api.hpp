#ifndef SHIZU_OBJ_API_HPP
#define SHIZU_OBJ_API_HPP
#include <cstdint>
#include <cstdlib>
#include <pico/stdlib.h>

namespace shizu {
namespace obj_api {
enum struct svc_num : uint32_t {
  YIELD = 0,
  CREATE_OBJECT = 2,
  CREATE_THREAD = 3,
  EXPORT_METHOD = 4,
  EXIT_METHOD = 5,
  CALL_METHOD = 6,
  SET_OBJECT_MD = 7,
  CALL_METHOD_VIA_MD = 8,
  GET_CURRENT_OBJ_ID = 9,
  SET_OBJ_MEMORY = 10,
  GET_OBJ_MEMORY = 11,
};

template <typename T>
  requires(sizeof(T) <= 4)
struct result_t {
  enum struct state_t : uint32_t { SUCCESS = 0, FAILURE } result;
  uint32_t value;
  T get_value() { return static_cast<T>(value); }
};

static result_t<uint32_t> svc(svc_num arg0, uintptr_t arg1, uintptr_t arg2,
                              uintptr_t arg3, uintptr_t r12 = 0) {
  uint32_t return_code, value;
  asm volatile("mov r0,%[arg0];"
               "mov r1,%[arg1];"
               "mov r2,%[arg2];"
               "mov r3,%[arg3];"
               "mov r12,%[r12];"
               "svc 0x00;"
               "mov %[return_code],r0;"
               "mov %[value],r1"
               : [return_code] "=r"(return_code), [value] "=r"(value)
               : [arg0] "r"(arg0), [arg1] "r"(arg1), [arg2] "r"(arg2),
                 [arg3] "r"(arg3), [r12] "r"(r12)
               : "r0", "r1", "r2", "r3", "ip", "lr", "memory", "cc");
  return {.result = (result_t<uint32_t>::state_t)return_code, .value = value};
};

template <uintptr_t sys_call_num>
  requires(sys_call_num <= 255) // SVC instruction immediate must be under 256.
static __always_inline result_t<uint32_t> svc(uintptr_t arg0, uintptr_t arg1,
                                              uintptr_t arg2, uintptr_t arg3,
                                              uintptr_t r12 = 0) {
  uint32_t return_code, value;
  asm volatile(
      "mov r0,%[arg0];"
      "mov r1,%[arg1];"
      "mov r2,%[arg2];"
      "mov r3,%[arg3];"
      "mov r12,%[r12];"
      "svc %[sys_call_num];"
      "mov %[return_code],r0;"
      "mov %[value],r1"
      : [return_code] "=r"(return_code), [value] "=r"(value)
      : [arg0] "r"(arg0), [arg1] "r"(arg1), [arg2] "r"(arg2), [arg3] "r"(arg3),
        [r12] "r"(r12), [sys_call_num] "i"(sys_call_num)
      : "r0", "r1", "r2", "r3", "ip", "lr", "memory");
  return {.result = (result_t<uint32_t>::state_t)return_code, .value = value};
};

static void yield() { svc(svc_num::YIELD, 0, 0, 0, 0); }

__always_inline static void yield_until(bool (*condition)()) {
  while (1) {
    if (condition() == false) {
      yield();
    } else {
      break;
    }
  }
};
result_t<uint32_t> create_object(uint32_t obj_num, uint32_t thread_num,
                                 uintptr_t entry);

result_t<uint32_t> export_method(uint32_t method_num, uintptr_t entry);

result_t<uint32_t> set_memory(uint32_t memory_num, uint32_t value);
result_t<uint32_t> get_memory(uint32_t memory_num);
result_t<uint32_t> set_object_md(uint32_t md_id, uint32_t target_obj_id,
                                 uint32_t target_method_id);

__always_inline static void yield_us(uint64_t us) {
  uint64_t start = time_us_64();
  while (time_us_64() - start < us) {
    shizu::obj_api::yield();
  }
}

// 絶対締切 deadline_us まで待つ。余裕があるうちは yield で他スレッドに譲り、最後の
// spin_us[µs] だけは yield せずビジースピンで締切に張り付く。yield 1 回で他スレッドが
// 走る時間ぶんオーバーシュートする yield_us と違い、締切±数µs に収まる。
// 注意: 協調スケジューラなので spin 中は他オブジェクトが完全に止まる。precision と
// starvation のトレードオフなので spin_us は「最長 1 スレッドの実行時間」程度に留める。
// 既に締切を過ぎていれば即 return。
__always_inline static void yield_until_us(uint64_t deadline_us,
                                           uint64_t spin_us = 300) {
  while ((int64_t)(deadline_us - time_us_64()) > (int64_t)spin_us) {
    shizu::obj_api::yield();
  }
  while ((int64_t)(deadline_us - time_us_64()) > 0) {
    tight_loop_contents();
  }
}

} // namespace obj_api
} // namespace shizu
#endif // SHIZU_OBJ_API_HPP