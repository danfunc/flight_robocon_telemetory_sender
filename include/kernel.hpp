
#ifndef SHIZU_KERNEL_HPP
#define SHIZU_KERNEL_HPP
#include <cstdint>
#include <set>

// デュアルコアトグル: 1 = core1 を Shizuku カーネルとして起動し、センサ I/O を
// SENSOR_IO スレッド (core1 ピン留め) として走らせる。0 = 従来どおり core1 を
// ベアメタル sensor I/O (core1_io_launch) に使う (フォールバック)。
#ifndef SHIZU_CORE1_KERNEL_POC
#define SHIZU_CORE1_KERNEL_POC 1
#endif

// ストリーム自己テスト トグル: 1 = STREAM_TEST オブジェクトの producer/consumer
// スレッド対を作り、共有メモリストリームで create/open/bind/push/pop を実動確認する。
#ifndef SHIZU_STREAM_SELFTEST
#define SHIZU_STREAM_SELFTEST 0
#endif

int main(int argc, char const *argv[]);

namespace shizu {
using method_t = uint32_t (*)(uint32_t, uint32_t, uint32_t, uint32_t);
void exit(uint32_t return_code);
[[noreturn]] __always_inline void init();
[[noreturn]] __always_inline void set_current_context_as_kernel_init();
[[noreturn]] void init_core1();  // core1 のカーネル入口 (core1_boot.cpp)
void core1_kernel_launch();      // core0 から core1 を起動する (core1_boot.cpp)
void stream_selftest_launch();   // ストリーム自己テストの起動 (stream_selftest.cpp)
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
  static uint32_t current_thread_id[2]; // per-core: SIO cpuid (get_core_num) で索引
  static void init();
  struct svc_handler_descriptor {
    method_t entry_point;
    uint32_t handling_object_id;
  } static svc_handler_info;
  friend void shizu::init();
  friend void shizu::init_core1();
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
  // SWITCH_THREAD を「READY→RUNNING の CAS claim + affinity 検査 + 成否返却」に
  // 再定義した別名。旧 SWITCH_THREAD 呼び出し元は r0 を見ないので互換。
  TRY_SWITCH_THREAD = 203,
  GET_OBJECT_ID_FROM_THREAD_ID = 204,
  GET_THREAD_STATE = 205,

};

// TRY_SWITCH_THREAD (claim) の型付きエラー。error_t<switch_error> で受ける。
// (SMP 基盤: 各コアのスケジューラが READY スレッドを原子的に claim して、2 コアが
//  同一 context_t を走らせないようにする。AMP 相当は affinity 固定という policy。)
enum struct switch_error : uint32_t {
  OK = 0,       // 切替成功 (自スレッド指定の no-op も OK 扱い)
  NOT_READY,    // 対象が READY でない / claim 競り負け (他コアが先に取った)
  BAD_AFFINITY, // 対象がこのコアで走ることを許されていない
};
// affinity ビットマスク。bit0=core0, bit1=core1。thread_table_init の既定は CORE0
// (core1 が既存スレッドを勝手に claim しないよう安全側に倒す = AMP。core1 で走らせる
//  スレッドだけ明示的に CORE1 を立てる)。
constexpr uint32_t AFFINITY_CORE0 = 0b01;
constexpr uint32_t AFFINITY_CORE1 = 0b10;
constexpr uint32_t AFFINITY_ALL = 0b11;

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

// 固定深さのメソッド呼び出しスタック。frames は create 時に 1 回だけ malloc する
// (SVC 例外ハンドラ内で malloc しないための措置 = SMP 前提。std::stack の push は
//  例外文脈で malloc し得たのでやめた)。深さ超過は panic。claim したコアだけが触る
//  single-owner なのでロック不要。thread_table[128] の静的ゼロ初期化で frames=nullptr。
struct call_stack_t {
  static constexpr uint32_t MAX_DEPTH = 16; // メソッド呼び出しネスト上限 (ABI 定数)
  method_call_stack_t *frames;              // create 時に MAX_DEPTH 分確保
  uint32_t depth;
  bool empty() const { return depth == 0; }
  bool full() const { return depth >= MAX_DEPTH; }
  method_call_stack_t &top() { return frames[depth - 1]; }
  void push(const method_call_stack_t &f) { frames[depth++] = f; }
  void pop() { --depth; }
};

struct thread_t {
  // state は claim の CAS 対象なので 32bit 幅を固定する (kernel.cpp の
  // __atomic_compare_exchange_n が (uint32_t*)&state で叩く)。
  enum struct state_t : uint32_t {
    UNINITIALIZED = 0,
    READY,
    RUNNING,
    SUSPENDED
  } state;
  // affinity: 走行を許すコアのビットマスク (bit0=core0, bit1=core1)。既定は全コア。
  // TRY_SWITCH_THREAD の claim がこれを検査する (AMP 相当は「core1 固定」等の policy)。
  uint32_t affinity;
  // wake_at: sleep 中の起床時刻 (µs, time_us_64 基準)。スケジューラは wake_at > now の
  // スレッドをスキップする (sleep 中は round-robin から外す)。0 や過去値なら即 runnable。
  uint64_t wake_at;
  context_t *context;
  uint32_t object_id;
  uint32_t thread_id;
  void ready_to_run();
  call_stack_t call_stack;
};

void object_table_init();
void thread_table_init();

} // namespace shizu
#endif // SHIZU_KERNEL_HPP