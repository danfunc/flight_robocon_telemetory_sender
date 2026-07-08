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
  // ---- ストリーム制御プレーン (include/stream.hpp)。データプレーン(push/pop)は
  //      SVC を通らないライブラリ。ここはディスクリプタ登録・discovery・待ち/起こし
  //      だけをカーネルに委ねる (カーネルハンドラは別途実装予定) ----
  CREATE_STREAM = 12,  // r1=id, r2=stream_desc_t*, r3=flags。owner=呼び出し元 obj
  OPEN_STREAM = 13,    // r1=id → r1(戻)=stream_desc_t*。consumer の discovery 用
  BIND_STREAM = 14,    // r1=id, r2=role。単一 producer/consumer をカーネルが強制
  STREAM_WAIT = 15,    // r1=id。空なら consumer を SUSPEND (SVC 内で空再検査→寝る)
  STREAM_NOTIFY = 16,  // r1=id。waiter がいれば起こす (producer が稀パスで呼ぶ)
  SLEEP_US = 17,       // r1=µs。締切まで自スレッドを scheduler にスキップさせる
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

// 締切 (now + us) まで自スレッドをスケジューラの round-robin から外す (sleep)。
// busy-yield と違い、寝ている間は他スレッドが走り、CPU を無駄に舐めない。
static void sleep_us(uint32_t us) { svc(svc_num::SLEEP_US, us, 0, 0, 0); }

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
  // sleep 化: スケジューラが締切まで自スレッドを round-robin から外す (旧: busy-yield
  // ループで毎周期起こされていた)。呼び出し側は正のデルタを渡す前提 (絶対グリッドは
  // next<now を signed で弾いてから呼ぶ)。
  sleep_us((uint32_t)us);
}

// 絶対締切 deadline_us まで待つ。余裕があるうちは yield で他スレッドに譲り、最後の
// spin_us[µs] だけは yield せずビジースピンで締切に張り付く。yield 1 回で他スレッドが
// 走る時間ぶんオーバーシュートする yield_us と違い、締切±数µs に収まる。
// 注意: 協調スケジューラなので spin 中は他オブジェクトが完全に止まる。precision と
// starvation のトレードオフなので spin_us は「最長 1 スレッドの実行時間」程度に留める。
// 既に締切を過ぎていれば即 return。
__always_inline static void yield_until_us(uint64_t deadline_us,
                                           uint64_t spin_us = 300) {
  uint64_t now = time_us_64();
  // 締切の spin_us 手前まではスケジューラにスキップさせ (sleep)、最後の spin_us だけ
  // busy-spin で締切に張り付く (sleep の round-robin ジッタを避け ±数µs に収める)。
  if ((int64_t)(deadline_us - now) > (int64_t)spin_us)
    sleep_us((uint32_t)(deadline_us - now - spin_us));
  while ((int64_t)(deadline_us - time_us_64()) > 0) {
    tight_loop_contents();
  }
}

} // namespace obj_api
} // namespace shizu
#endif // SHIZU_OBJ_API_HPP