
#ifndef SHIZU_KERNEL_HPP
#define SHIZU_KERNEL_HPP
#include <cstdint>
#include <set>
#include <stack>

int main(int argc, char const *argv[]);

namespace shizu {
using method_t = uint32_t (*)(uint32_t, uint32_t, uint32_t, uint32_t);
void exit(uint32_t return_code);
[[noreturn]] __always_inline void init();
[[noreturn]] __always_inline void set_current_context_as_kernel_init();
class cpu_manager;
struct object_t;
struct thread_t;
struct context_t;
struct exception_frame_t;
struct method_call_stack_t;

extern object_t object_table[128];
extern thread_t thread_table[128];
namespace FOR_KERNEL_OBJECT {
void export_method(uint32_t obj_num, uint32_t method_num, method_t entry);
void exit_method(uint32_t return_code);
void create_object(uint32_t obj_num, uint32_t thread_num, method_t entry);
void create_thread(uint32_t obj_num, uint32_t thread_num, method_t entry);
} // namespace FOR_KERNEL_OBJECT

} // namespace shizu

extern "C" {
void svc_asm_handler();
shizu::context_t *get_current_context();
void svc_cpp_handler(shizu::context_t *context);
}

namespace shizu {

class cpu_manager {
  static uint32_t current_thread_id;
  static void init();
  struct svc_handler_descriptor {
    method_t entry_point;
    uint32_t handling_object_id;
  } static svc_handler_info;
  friend void shizu::init();
  friend shizu::context_t * ::get_current_context();
  friend int ::main(int argc, char const *argv[]);
  friend void ::svc_cpp_handler(shizu::context_t *context);
};

struct object_t {
  uint32_t id;
  uint32_t svc_handler_obj_id;
  uint32_t svc_handler_method_num;
  std::set<uint32_t> thread_table;
  method_t method_table[128];
  enum struct state_t {
    UNINITIALIZED = 0,
    KERNEL_OBJECT = 1,
    KERNEL_SPACE_OBJECT = 2,

  } state;
};

enum struct kernel_object_svc_num : uint32_t {
  GET_CURRENT_THREAD_ID = 0,
  METHOD_CALL = 200,
  METHOD_EXIT = 201,
  SVC_EXIT = 201,
  SET_SVC_HANDLER = 202,
  SWITCH_THREAD = 203,
  GET_OBJECT_ID_FROM_THREAD_ID = 204,
  GET_THREAD_STATE = 205,

};

struct context_t {
  uint32_t r4, r5, r6, r7, r8, r9, r10, r11; // offset 0..28
  exception_frame_t *sp;                      // offset 32 (must stay 32)
  // --- 遅延 FPU コンテキストスイッチ用 (lazy FPU context switch) ---
  // exc_return: スレッドごとにキャッシュした EXC_RETURN。bit4=0 なら拡張(FP)フレーム。
  // fp[16]    : callee-saved な FP レジスタ S16-S31 の退避域。
  // svc_asm_handler.S の .equ CTX_EXC_RETURN(36)/CTX_FP(40) と一致させること。
  uint32_t exc_return; // offset 36
  uint32_t fp[16];     // offset 40 (S16-S31)
};

struct exception_frame_t {
  uint32_t r0, r1, r2, r3, r12, lr, pc, xPSR;
};

struct method_call_stack_t {
  exception_frame_t stack_frame;
  context_t context;
  uint32_t caller_object_id;
  uint32_t caller_syscall_num;
};

struct thread_t {
  enum struct state_t { UNINITIALIZED = 0, READY, RUNNING, SUSPENDED } state;
  context_t *context;
  uint32_t object_id;
  uint32_t thread_id;
  void ready_to_run();
  std::stack<method_call_stack_t> call_stack;
};

void object_table_init();
void thread_table_init();

} // namespace shizu
#endif // SHIZU_KERNEL_HPP