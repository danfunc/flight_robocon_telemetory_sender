
#ifndef SHIZU_KERNEL_HPP
#define SHIZU_KERNEL_HPP
#include <cstdint>
#include <pico.h> // get_core_num (grant_active_this_core)
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

// 時限実行権移譲 (run_for/GRANT_CPU) の自己テスト トグル: 1 = GRANT_TEST オブジェクトの
// スレッド群で EXPIRED/YIELDED/ネスト/長期限 (SysTick チャンク継ぎ) を実動確認する。
#ifndef SHIZU_GRANT_SELFTEST
#define SHIZU_GRANT_SELFTEST 0
#endif

// SMP ストレス トグル: 1 = SMP_STRESS オブジェクトの worker 群 (core0×2 / core1×2 /
// 両コア移動×1) が SLEEP/SWITCH の SVC を両コアから同時多発させ、カーネル SVC 経路の
// 2 コア安全性を数分単位で炙る。reporter がカウンタを 2s ごとに印字 (全て前進 = 健全)。
#ifndef SHIZU_SMP_STRESS
#define SHIZU_SMP_STRESS 0
#endif

// スケジューラが budget 付きスレッドへ与える時限の既定値 [µs]。
// 意味論は quantum でなく「凍結ウォッチドッグ」: 全ドライバは正常時これより十分
// 短い周期で yield する (最長の正当ホールドは I2C 100kHz 26B 読み ≈2.4ms) ので、
// 期限切れは「従来なら全系凍結だった異常時」にしか発火しない。
#ifndef SHIZU_DEFAULT_GRANT_BUDGET_US
#define SHIZU_DEFAULT_GRANT_BUDGET_US 3000
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
void grant_selftest_launch();    // 時限移譲自己テストの起動 (grant_selftest.cpp)
void smp_stress_launch();        // 2 コア SVC ストレスの起動 (smp_stress.cpp)
// per-core の例外優先度設定 (SVC 最優先 > SysTick > PendSV 最低)。SHPR は banked な
// ので core0 は cpu_manager::init、core1 は init_core1 がそれぞれ呼ぶ。
void init_exception_priorities();
enum struct grant_end : uint32_t; // 前方宣言 (定義は下の grant セクション)
// grant を 1 段 pop して grantor へ復帰させる (kernel.cpp 内部: PendSV=EXPIRED /
// SWITCH_THREAD インターセプト=YIELDED の共通経路)。
void grant_pop_to_grantor(uint32_t core, grant_end reason);
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
void create_object(uint32_t obj_num); // オブジェクトのみ生成 (スレッド無し)
void create_thread(uint32_t obj_num, uint32_t thread_num, method_t entry,
                   uint32_t arg = 0); // arg は entry の r0 に載る
// 空きスレッドスロット (1..127) を自動確保して entry を非同期起動する。戻り = 割り当てた
// thread_id。オブジェクト起動 = create_object(id) + async_call(id, main) の 2 段で行う。
uint32_t async_call(uint32_t obj_num, method_t entry, uint32_t arg = 0);
} // namespace FOR_KERNEL_OBJECT

} // namespace shizu

extern "C" {
void svc_asm_handler();
void pendsv_asm_handler(); // grant 期限の強制切替入口 (svc_asm_handler.S)
shizu::context_t *get_current_context();
void svc_cpp_handler(shizu::context_t *context);
void pendsv_cpp_handler(shizu::context_t *context);
void systick_grant_handler(); // grant 期限の監視 (tickless ワンショット)
void shizu_panic(const char *fmt, ...); // panic() の差し替え先 (panic_ring.hpp)
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
  friend void ::pendsv_cpp_handler(shizu::context_t *context);
  friend void shizu::grant_pop_to_grantor(uint32_t core, grant_end reason);
  friend void ::shizu_panic(const char *fmt, ...); // panic 記録に thread id を載せる
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
  // 時限実行権移譲: arg0=対象 tid, arg1=最大実行時間[µs]。対象を claim して切替え、
  // 期限到来 (SysTick→PendSV 強制切替) か対象の yield/switch (早期復帰) で戻る。
  // 戻り: r0=grant_error, r1=grant_end (EXPIRED/YIELDED)。
  GRANT_CPU = 206,

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

// ---- 時限実行権移譲 (GRANT_CPU / run_for) ---------------------------------
// GRANT_CPU の型付きエラー。先頭 3 値は switch_error と値を揃える (claim 共通ヘルパの
// 結果をそのままキャストで写せるように)。
enum struct grant_error : uint32_t {
  OK = 0,       // 移譲成功 (自スレッド指定の no-op も OK)
  NOT_READY,    // 対象が READY でない / claim 競り負け
  BAD_AFFINITY, // 対象がこのコアで走ることを許されていない
  DEPTH,        // grant スタック満杯 (ネスト深さ上限)
};
// grant の終了理由 (r1 で返る)。
enum struct grant_end : uint32_t {
  EXPIRED = 0, // 期限到来による強制復帰 (PendSV)
  YIELDED = 1, // 移譲先の yield/switch/sleep による早期復帰
};
// per-core grant スタック。ネスト許容 (内側 deadline は外側へクランプされるので
// over-time は構造的に不可能)。SVC/SysTick/PendSV は全て自コアのスタックだけを触り、
// 優先度 (SVC > SysTick > PendSV) で直列化されるためロック不要。SVC パス脱 malloc
// 方針に従い固定配列。
struct grant_frame_t {
  uint32_t grantor;  // 復帰先スレッド (WAIT_GRANT で待つ)
  uint64_t deadline; // 期限 (µs, time_us_64 基準)。外側 deadline でクランプ済み
};
struct grant_stack_t {
  static constexpr uint32_t MAX_DEPTH = 8;
  grant_frame_t frames[MAX_DEPTH];
  uint32_t depth;
};
extern grant_stack_t grant_stacks[2]; // per-core (SIO cpuid で索引)

// 自コアで grant がアクティブか (= 自分は誰かの grantee として走っているか)。
// kernel_obj_svc_handler の YIELD/SLEEP_US が「grantor への早期復帰」へ分岐するのに使う。
inline bool grant_active_this_core() {
  return grant_stacks[get_core_num()].depth != 0;
}

// ---- per-core CPU 使用率計測 (スケジューラが実務へ渡した時間の累積) ---------
// 各コアの共通 idle ループ (scheduler_idle_loop) が sched_pick_next 1 回ごとに、
// 「実際にスレッドを走らせて戻ってきた経過時間」を cpu_busy_us へ足す。誰も runnable
// でなくスピンした分は足さない。使用率 = Δcpu_busy_us/Δwall。スケジューラが唯一の
// 会計点なので core0/core1 一様、core1 専用計装が不要 (SENSOR_IO が idle 時に sleep で
// スケジューラへ戻る前提)。単一コアが自分の要素だけ触るのでロック不要。
extern volatile uint64_t cpu_busy_us[2]; // スケジューラが実務へ渡した累積時間 [µs]

// 各コアの共通スケジューラ idle ループ (kernel_object.cpp)。core0=thread0、
// core1=idle スレッドがそれぞれ自分の tid で呼び、以後この 1 実装だけが両コアで回る。
[[noreturn]] void scheduler_idle_loop(uint32_t self_tid);

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
    SUSPENDED,
    // grant 発行中の grantor。READY でないので他コアの claim 対象にならず、復帰は
    // 発行コア上の PendSV (期限) か SWITCH_THREAD インターセプト (早期復帰) のみ。
    WAIT_GRANT,
  } state;
  // affinity: 走行を許すコアのビットマスク (bit0=core0, bit1=core1)。既定は全コア。
  // TRY_SWITCH_THREAD の claim がこれを検査する (AMP 相当は「core1 固定」等の policy)。
  uint32_t affinity;
  // grant_budget_us: スケジューラ (sched_pick_next) がこのスレッドへ CPU を渡すときの
  // 時限 [µs]。0 = 無制限 (従来の SWITCH_THREAD バトンパス)。>0 = GRANT_CPU で host
  // される (期限で必ず回収される guest)。不変条件: budget>0 のスレッドは guest として
  // しか走らないので、yield を怠っても系を凍らせられない。バトン (無限移譲) を受け
  // 取れるのは budget 0 のスレッドだけ (thread0 idle / BLE_UART / core1 組)。
  uint32_t grant_budget_us;
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