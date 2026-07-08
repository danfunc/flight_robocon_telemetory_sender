#ifndef SHIZU_HPP
#define SHIZU_HPP
#include <cstdlib>
#include <kernel.hpp>
#include <kernel_object.hpp>
#include <svc.hpp>
namespace shizu {
[[noreturn]] __always_inline void set_current_context_as_kernel_init() {

  void *entry_PSP = (void *)(((uint32_t)malloc(4096) + 4096) & ~0xF);
  object_table[0].thread_table.insert(0);
  object_table[0].state = object_t::state_t::KERNEL_OBJECT;
  thread_table[0].context = new context_t();
  thread_table[0].context->sp = (shizu::exception_frame_t *)entry_PSP;
  // スレッド 0 も基本フレーム/PSP へ復帰する前提で EXC_RETURN を種付けする。
  thread_table[0].context->exc_return = 0xFFFFFFFD;
  // カーネルオブジェクトの呼び出しスタックも create 時に 1 回だけ確保 (SVC パス脱 malloc)。
  thread_table[0].call_stack.frames = (method_call_stack_t *)malloc(
      sizeof(method_call_stack_t) * call_stack_t::MAX_DEPTH);
  thread_table[0].call_stack.depth = 0;
  thread_table[0].state = thread_t::state_t::RUNNING;
  uintptr_t CONTROL_MASK = 1 << 1;
  asm volatile("MSR PSP,%[entry_psp];"
               "MSR CONTROL, %[CONTROL_MASK];"
               "isb;"
               :
               : [entry_psp] "r"(entry_PSP), [CONTROL_MASK] "r"(CONTROL_MASK)
               : "memory");
  kernel_object_main();
  while (1)
  {
  }
  
}
[[noreturn]] __always_inline void init() {
  cpu_manager::init();
  object_table_init();
  thread_table_init();
  set_current_context_as_kernel_init();
}

} // namespace shizu
#endif // SHIZU_HPP