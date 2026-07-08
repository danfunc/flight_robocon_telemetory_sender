#ifndef SHIZU_KERNEL_OBJECT_HPP
#define SHIZU_KERNEL_OBJECT_HPP
#include <kernel.hpp>
#include <svc.hpp>
namespace shizu {
void kernel_object_main();
static inline uint32_t get_current_thread_id() {
  result_t<uint32_t> result =
      svc<(uint32_t)kernel_object_svc_num::GET_CURRENT_THREAD_ID>(0, 0, 0, 0);
  return result.value;
}
// claim (READY→RUNNING CAS + affinity 検査 + switch) を型付きエラーで呼ぶ。
// .error.is_ok() = 切替成功 / NOT_READY = 競り負け or 非 READY / BAD_AFFINITY。
static inline svc_result_t<switch_error> try_switch_thread(uint32_t tid) {
  return ::svc_typed<(uint32_t)kernel_object_svc_num::TRY_SWITCH_THREAD,
                     switch_error>(tid, 0, 0, 0);
}
} // namespace shizu
#endif // SHIZU_KERNEL_OBJECT_HPP