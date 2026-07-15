// ===========================================================================
//  Shizuku マイクロカーネル (RP2350 / Cortex-M33) — コア実装
// ===========================================================================
//
//  【全体像】
//  Shizuku は「オブジェクト」と「スレッド」を基本単位とする協調型 (cooperative)
//  マイクロカーネル。MMU による空間分離は行わず
//  (単一物理アドレス空間)、代わりに ARM の SVC 例外を IPC
//  のトラップ口として使い、オブジェクト境界をまたぐ呼び出し を「カーネルが pc /
//  現在オブジェクト ID を張り替える」ことで実現する。
//
//  ・object_t  : 0..127 のスロット。method_table[128]
//  (公開メソッドの関数ポインタ)、
//                所属スレッド集合、状態を持つ。id=0
//                は特権を持つ「カーネルオブジェクト」。
//  ・thread_t  : 0..127 のスロット。context_t* (退避レジスタ + PSP) と
//  call_stack
//                (メソッド呼び出しのネストを積むスタック) を持つ。
//
//  【ディスパッチは "svc 番号" ではなく "発行元オブジェクトの種別" で決まる】
//   カーネルは svc 命令の番号で経路を分けてはいない。svc を発行したスレッドが
//   属するオブジェクトが「カーネルオブジェクト」か「一般オブジェクト」かだけで
//   経路が決まる (svc_cpp_handler 冒頭の object_table[...].state 判定)。svc 番号は
//   経路を決めた後、そのオブジェクトが呼びたいプリミティブ/API の選択に使うだけ。
//
//   API は用途別に 2 系統ある:
//   (1) kernel_object_svc_num … カーネルオブジェクト (id=0) が使う低位プリミティブ。
//        METHOD_CALL(200) / METHOD_EXIT(201) / SET_SVC_HANDLER(202) /
//        SWITCH_THREAD(203) / GET_CURRENT_THREAD_ID(0) など。発行元がカーネル
//        オブジェクトのときだけ svc_cpp_handler が switch で直接処理する。
//   (2) obj_api::svc_num … 一般オブジェクトが使う高位 API (CREATE_OBJECT,
//        EXPORT_METHOD, CALL_METHOD, YIELD, SET/GET_OBJ_MEMORY ...)。発行元が
//        一般オブジェクトなら、カーネルは内容を解釈せず「登録済み SVC ハンドラ
//        (= カーネルオブジェクトの kernel_obj_svc_handler)」へトランポリンする。
//
//  【ディスパッチの肝 (svc_cpp_handler)】
//   ・発行元がカーネルオブジェクト  → プリミティブを switch で即実行。
//   ・発行元が一般オブジェクト      → 現フレームを call_stack に積み、pc を
//      登録済みハンドラへ張り替え、svc 番号/呼び出し元 obj/thread を r4/r5/r6
//      に載せて 同じスレッド上でカーネルオブジェクトとして走らせる (権限委譲)。
//
//  【メソッド呼び出し (METHOD_CALL)】
//   呼び出し元と同一スレッド上で、対象オブジェクトの method_table[m] に pc
//   を移し、 current object id を対象 obj に張り替える
//   (保護されたサブルーチン呼び出し)。 戻り (METHOD_EXIT) で call_stack を pop
//   し、元の pc / オブジェクト ID を復元する。 → 戻り値は r1 に 1
//   つだけ載る。大きなデータはアドレスを arg に渡して
//     呼び出し先がコピーする (単一アドレス空間なのでポインタが共有できる)。
//
//  【スケジューリング — 方針はカーネルではなくカーネルオブジェクトが持つ】
//   カーネルが提供する機構は 2 つだけ: SWITCH_THREAD (無限時間のバトンパス) と
//   GRANT_CPU (時限移譲 = SysTick/PendSV による期限回収付き)。方針 (誰にどちらで
//   渡すか) はカーネルオブジェクトの sched_pick_next が持つ:
//     ・grant_budget_us == 0 のスレッド → バトンパス (従来の協調、無限移譲)。
//        該当: thread0 idle / BLE_UART / core1 idle / SENSOR_IO。
//     ・grant_budget_us > 0 のスレッド → GRANT_CPU で host (既定 3ms、凍結
//        ウォッチドッグ)。guest は yield で host へ早期復帰し、yield を怠っても
//        期限で必ず回収される = 1 ドライバの暴走が全系を凍らせられない。
//   期限は quantum ではなく異常時の安全網: 全ドライバは正常時 3ms より十分短く
//   yield するので発火せず、その限り従来の協調原子性 (yield しない限り I2C/共有
//   状態の処理は原子) は保たれる。発火した瞬間だけ torn read があり得る (従来の
//   全系凍結の代替として許容)。budget>0 のスレッドは guest としてしか走らない
//   ため、バトン (無限移譲) を受け取れるのは budget 0 組だけ。
//
//  svc_asm_handler.S が例外入口で {r4-r11, PSP} を context_t に退避し、ここで
//  svc_cpp_handler が論理を処理して context を更新、復帰時に書き戻す。
// ===========================================================================
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <hardware/clocks.h>            // clock_get_hz (SysTick 装填のサイクル換算)
#include <hardware/exception.h>
#include <hardware/irq.h>
#include <hardware/regs/m33.h>          // M33_ICSR_PENDSVSET_BITS
#include <hardware/structs/scb.h>       // scb_hw->icsr (PendSV pend)
#include <hardware/structs/systick.h>   // systick_hw (banked per-core)
#include <kernel.hpp>
#include <pico/stdlib.h>
#include <shizu.hpp>

static_assert(offsetof(shizu::context_t, sp) == 32,
              "context_t layout mismatch: psp offset");
// svc_asm_handler.S の .equ CTX_EXC_RETURN / CTX_FP と一致していること。
static_assert(offsetof(shizu::context_t, exc_return) == 36,
              "context_t layout mismatch: exc_return offset (asm CTX_EXC_RETURN)");
static_assert(offsetof(shizu::context_t, fp) == 40,
              "context_t layout mismatch: fp offset (asm CTX_FP)");
// claim は (uint32_t*)&thread.state を __atomic で叩くので 4B 幅を保証する。
static_assert(sizeof(shizu::thread_t::state_t) == 4,
              "thread state must be a 32-bit word for __atomic claim CAS");
extern "C" {
void svc_asm_handler();
shizu::context_t *get_current_context();
}

void *get_psp();
namespace shizu {
object_t object_table[128];
thread_t thread_table[128];

void object_table_init() {
  for (uint32_t i = 0; i < 128; i++) {
    object_table[i].id = i;
    object_table[i].thread_table.clear();
    object_table[i].state = object_t::state_t::UNINITIALIZED;
  }
}

void create_object(uint32_t obj_num) {
  if (object_table[obj_num].state != object_t::state_t::UNINITIALIZED) {
    panic("object already initialized");
  }
  object_table[obj_num].state = object_t::state_t::KERNEL_SPACE_OBJECT;
}

void thread_table_init() {
  for (size_t i = 0; i < 128; i++) {
    thread_table[i].state = thread_t::state_t::UNINITIALIZED;
    thread_table[i].affinity = AFFINITY_CORE0; // 既定は core0 ピン (core1 は明示のみ)
    thread_table[i].wake_at = 0;               // 0 = sleep していない (即 runnable)
    // 既定は budget 付き (凍結ウォッチドッグ)。無制限にしたいスレッドだけ 0 を明示
    // する (thread0 / BLE_UART / core1 組)。
    thread_table[i].grant_budget_us = SHIZU_DEFAULT_GRANT_BUDGET_US;
    thread_table[i].context = nullptr;
    thread_table[i].object_id = 0;
    thread_table[i].thread_id = i;
  }
}

uint32_t cpu_manager::current_thread_id[2] = {0, 0};

// ---------------------------------------------------------------------------
//  時限実行権移譲 (GRANT_CPU) の期限強制 — tickless SysTick + PendSV
// ---------------------------------------------------------------------------
//  grant 未使用時のコストは厳密ゼロ (SysTick は grant がある間しか動かない)。
//  プリエンプトされ得るのはアクティブ grant の grantee だけで、それ以外の
//  スレッドは従来の協調保証 (yield しない限り原子) を完全に維持する。
//  優先度 SVC > SysTick > PendSV により、SVC プリミティブ実行中に期限処理が
//  割り込むことはなく、grant スタックは自コア内で直列化される (ロック不要)。
grant_stack_t grant_stacks[2] = {};

// per-core CPU 使用率計測 (スケジューラが実務へ渡した時間の累積)。宣言は kernel.hpp。
volatile uint64_t cpu_busy_us[2] = {0, 0};

// clk_sys のサイクル/µs。cpu_manager::init で実クロックから 1 回キャッシュする
// (両コア同一クロック)。150 MHz 既定値は init 前の保険。
static uint32_t g_cycles_per_us = 150;

static constexpr uint32_t SYSTICK_MAX_LOAD = 0x00FFFFFF; // 24bit (~111ms @150MHz)
// PendSV がカーネルオブジェクト走行中に切替を延期したときの再試行間隔 [µs]。
static constexpr uint32_t GRANT_RETRY_US = 50;

// 自コアの SysTick を deadline (µs 絶対時刻) に向けてワンショット装填する。
// 残りが 24bit を超えるときは最大チャンクで装填し、systick_grant_handler が継ぐ。
static void systick_arm_for(uint64_t deadline) {
  const uint64_t now = time_us_64();
  uint64_t cycles = (deadline > now) ? (deadline - now) * g_cycles_per_us : 1;
  if (cycles > SYSTICK_MAX_LOAD)
    cycles = SYSTICK_MAX_LOAD;
  if (cycles < 100)
    cycles = 100; // 装填直後発火の下限 (rvr=0 は SysTick 停止扱いになるのも防ぐ)
  systick_hw->rvr = (uint32_t)cycles;
  systick_hw->cvr = 0; // 書き込みでカウンタクリア → rvr から再カウント
  systick_hw->csr = 0x7; // ENABLE | TICKINT | CLKSOURCE(プロセッサクロック)
}

static void systick_disarm() { systick_hw->csr = 0; }

// grant を 1 段 pop して grantor へ復帰させる。PendSV (期限, EXPIRED) と
// SWITCH_THREAD インターセプト (早期復帰, YIELDED) の共通経路。呼び出し文脈は
// 自コアの SVC または PendSV で、優先度により直列化済み。grantee (現行スレッド)
// の context 退避は asm 入口が完了させていること。
void grant_pop_to_grantor(uint32_t core, grant_end reason) {
  grant_stack_t &gs = grant_stacks[core];
  const uint32_t grantee = cpu_manager::current_thread_id[core];
  const uint32_t grantor = gs.frames[gs.depth - 1].grantor;
  gs.depth--;
  // grantor の復帰値: r0=OK は GRANT_CPU 成功時に設定済み。r1 に終了理由。
  thread_table[grantor].context->sp->r1 = (uint32_t)reason;
  // WAIT_GRANT→RUNNING は発行コアだけが遷移させる単独所有 (plain store で足りる)。
  thread_table[grantor].state = thread_t::state_t::RUNNING;
  cpu_manager::current_thread_id[core] = grantor;
  // grantee を release。RELEASE store 以降、他コアが claim してよい。
  __atomic_store_n((uint32_t *)&thread_table[grantee].state,
                   (uint32_t)thread_t::state_t::READY, __ATOMIC_RELEASE);
  // 次の期限: 空→SysTick 停止 / 既期限→即 PendSV 連鎖 (クランプ同時期限の多段
  // ネストはここで連鎖的に巻き戻る) / それ以外→新 top へ再装填。
  if (gs.depth == 0) {
    systick_disarm();
  } else if (gs.frames[gs.depth - 1].deadline <= time_us_64()) {
    scb_hw->icsr = M33_ICSR_PENDSVSET_BITS;
  } else {
    systick_arm_for(gs.frames[gs.depth - 1].deadline);
  }
}

// per-core の例外優先度。SVC 最優先 = プリミティブの原子性 (現行の暗黙前提の明示化)。
// SysTick は SVC 未満・SDK IRQ(0x80) 以上 (期限判定 + pend だけの極小ハンドラ)。
// PendSV 最低 = 全 IRQ が捌けてからコンテキストスイッチ。SHPR は banked なので
// core0 は cpu_manager::init、core1 は init_core1 がそれぞれ呼ぶ。
void init_exception_priorities() {
  exception_set_priority(SVCALL_EXCEPTION, 0x00);
  exception_set_priority(SYSTICK_EXCEPTION, 0x40);
  exception_set_priority(PENDSV_EXCEPTION, PICO_LOWEST_IRQ_PRIORITY);
}

void cpu_manager::init() {
  exception_set_exclusive_handler(SVCALL_EXCEPTION, svc_asm_handler);
  // grant 期限強制の土台。RAM ベクタテーブルは両コア共有なので登録はここ (core0)
  // の 1 回だけ (core1 で再登録すると exclusive 二重登録 panic)。
  exception_set_exclusive_handler(PENDSV_EXCEPTION, pendsv_asm_handler);
  exception_set_exclusive_handler(SYSTICK_EXCEPTION, systick_grant_handler);
  g_cycles_per_us = clock_get_hz(clk_sys) / 1000000u;
  init_exception_priorities();
}

void thread_t::ready_to_run() {
  this->context = new context_t();
  this->state = state_t::READY;
}

cpu_manager::svc_handler_descriptor cpu_manager::svc_handler_info{nullptr, 0};

namespace FOR_KERNEL_OBJECT {

void create_object(uint32_t obj_num, uint32_t thread_num, method_t entry) {
  shizu::create_object(obj_num);
  shizu::FOR_KERNEL_OBJECT::create_thread(obj_num, thread_num, entry);
}

// オブジェクトのみ生成 (スレッド無し)。起動は別途 async_call(id, main) で行う。
void create_object(uint32_t obj_num) { shizu::create_object(obj_num); }

// 空きスレッドスロット (1..127, 0 はカーネルオブジェクト予約) を自動確保して entry を
// 非同期起動する。戻り = 割り当てた thread_id。
uint32_t async_call(uint32_t obj_num, method_t entry, uint32_t arg) {
  for (uint32_t t = 1; t < 128; ++t) {
    if (thread_table[t].state == thread_t::state_t::UNINITIALIZED) {
      shizu::FOR_KERNEL_OBJECT::create_thread(obj_num, t, entry, arg);
      return t;
    }
  }
  panic("async_call: no free thread slot (all 127 in use)");
  return 0;
}

void create_thread(uint32_t obj_num, uint32_t thread_num, method_t entry,
                   uint32_t arg) {
  if (thread_table[thread_num].state == thread_t::state_t::UNINITIALIZED) {
    thread_table[thread_num].object_id = obj_num;
    thread_table[thread_num].ready_to_run();
    // BLE/CYW43 (btstack) の送信・暗号化パスは深く、2KB ではオーバーフローして
    // HardFault する。1 スレッドあたり 8KB へ拡張 (RP2350 は RAM に余裕あり)。
    constexpr uint32_t THREAD_STACK_SIZE = 8192;
    thread_table[thread_num].context->sp =
        (exception_frame_t *)(((uint32_t)malloc(THREAD_STACK_SIZE) +
                               THREAD_STACK_SIZE - sizeof(exception_frame_t)) &
                              ~((uint32_t)0xfL));
    thread_table[thread_num].context->sp->pc = (uint32_t)entry;
    thread_table[thread_num].context->sp->lr =
        (uint32_t)shizu::FOR_KERNEL_OBJECT::exit_method;
    thread_table[thread_num].context->sp->r0 = arg; // entry の第1引数
    thread_table[thread_num].context->sp->r1 = 0;
    thread_table[thread_num].context->sp->r2 = 0;
    thread_table[thread_num].context->sp->r3 = 0;
    thread_table[thread_num].context->sp->r12 = 0;
    thread_table[thread_num].context->sp->xPSR = (1 << 24);
    // 新規スレッドは基本フレーム (FP 無効) の Thread/PSP へ復帰する。
    // 0 のままだと初回復帰で bx 0 → 即 HardFault。必ず明示的に種を入れる。
    thread_table[thread_num].context->exc_return = 0xFFFFFFFD;
    // 呼び出しスタックは create 時 (スレッドモード) に 1 回だけ確保する。以後 SVC
    // 例外ハンドラ内 (METHOD_CALL / トランポリン) では malloc しない。
    thread_table[thread_num].call_stack.frames =
        (method_call_stack_t *)malloc(sizeof(method_call_stack_t) *
                                      call_stack_t::MAX_DEPTH);
    thread_table[thread_num].call_stack.depth = 0;
    object_table[obj_num].thread_table.insert(thread_num);
  } else {
    panic("this thread is already initialized\ncalling this system call for "
          "\n thread_id: %d\n object_id: %d\n",
          thread_num, obj_num);
  };
}
void export_method(uint32_t obj_num, uint32_t method_num, method_t entry) {
  shizu::object_table[obj_num].method_table[method_num] = entry;
}
void exit_method(uint32_t return_code) {
  svc<(uint32_t)shizu::kernel_object_svc_num::METHOD_EXIT>(return_code, 0, 0, 0,
                                                           0);
};
} // namespace FOR_KERNEL_OBJECT

} // namespace shizu

shizu::context_t *get_current_context() {
  return shizu::thread_table[shizu::cpu_manager::current_thread_id[get_core_num()]]
      .context;
}

// SysTick (banked, 自コア): grant スタック top の期限監視。tickless ワンショット
// なので発火 = 「期限到来」か「24bit を超える期限のチャンク継ぎ」のどちらか。
void systick_grant_handler() {
  const uint32_t core = get_core_num();
  shizu::grant_stack_t &gs = shizu::grant_stacks[core];
  if (gs.depth == 0) { // 早期復帰との競合で grant が消えた後の遅延発火
    systick_hw->csr = 0;
    return;
  }
  const uint64_t deadline = gs.frames[gs.depth - 1].deadline;
  if (time_us_64() >= deadline) {
    systick_hw->csr = 0; // 停止。再装填は PendSV 側 (pop 後の新 top) が判断する
    scb_hw->icsr = M33_ICSR_PENDSVSET_BITS; // 切替本体は最低優先度の PendSV へ
  } else {
    shizu::systick_arm_for(deadline); // チャンク継ぎ足し (>111ms の期限)
  }
}

// PendSV (最低優先度): grant 期限の強制コンテキストスイッチ本体。ガードは
// 自己安定型 — 「depth==0 / top 未期限なら no-op 復帰」なので、早期復帰や
// SysTick との順序競合で余分に発火しても壊れない (PENDSVCLR 不要)。
void pendsv_cpp_handler(shizu::context_t *context) {
  (void)context; // grantee の退避は asm 入口 (CTX_SAVE) が完了済み
  const uint32_t core = get_core_num();
  shizu::grant_stack_t &gs = shizu::grant_stacks[core];
  if (gs.depth == 0)
    return;
  if (gs.frames[gs.depth - 1].deadline > time_us_64()) {
    shizu::systick_arm_for(gs.frames[gs.depth - 1].deadline); // spurious → 再装填
    return;
  }
  const uint32_t grantee = shizu::cpu_manager::current_thread_id[core];
  // grantee がカーネルオブジェクトとして走行中 (トランポリン/METHOD_CALL 中) は
  // 切替を延期する — カーネルオブジェクトが自身の内部状態 (method_map / malloc
  // 経路) を原子に保つための方針 (機構はカーネル、原子性はオブジェクトの方針)。
  if (shizu::object_table[shizu::thread_table[grantee].object_id].state ==
      shizu::object_t::state_t::KERNEL_OBJECT) {
    shizu::systick_arm_for(time_us_64() + shizu::GRANT_RETRY_US);
    return;
  }
  shizu::grant_pop_to_grantor(core, shizu::grant_end::EXPIRED);
}

__always_inline void set_return_code(uint32_t return_code,
                                     shizu::exception_frame_t &context) {
  context.r0 = return_code;
};

void set_return_code_as_syscall_num(shizu::exception_frame_t &stack_frame) {
  stack_frame.r0 = ((*(uint16_t *)(stack_frame.pc - 2)) & 0xff);
}

void trap() {
  while (1) {
    printf("trapped\n");
    sleep_ms(1000);
  }
}

// READY→RUNNING を CAS で原子的に claim する (SWITCH_THREAD / GRANT_CPU 共用)。
// これが「2 コアが同一 context_t を走らせない」根 = context の所有権ロックそのもの。
// ACQUIRE で以後の context 読みを claim 後に固定。
static shizu::switch_error try_claim(shizu::thread_t &t, uint32_t core) {
  if (!(t.affinity & (1u << core)))
    return shizu::switch_error::BAD_AFFINITY;
  uint32_t expected = (uint32_t)shizu::thread_t::state_t::READY;
  if (!__atomic_compare_exchange_n((uint32_t *)&t.state, &expected,
                                   (uint32_t)shizu::thread_t::state_t::RUNNING,
                                   false, __ATOMIC_ACQUIRE, __ATOMIC_RELAXED))
    return shizu::switch_error::NOT_READY;
  return shizu::switch_error::OK;
}

// 本当はcontext_t* svc_cpp_handler(context_t
// *context)として変更の有無に応じてcontextを変更した方が早い
void svc_cpp_handler(shizu::context_t *context) {
  shizu::exception_frame_t *stack_frame = context->sp;
  uint32_t arg0 = stack_frame->r0, arg1 = stack_frame->r1,
           arg2 = stack_frame->r2, arg3 = stack_frame->r3,
           r12 = stack_frame->r12, lr = stack_frame->lr, pc = stack_frame->pc;
  shizu::thread_t &current_thread =
      shizu::thread_table[shizu::cpu_manager::current_thread_id[get_core_num()]];
  shizu::kernel_object_svc_num svc_num =
      (shizu::kernel_object_svc_num)(*(uint16_t *)(pc - 2) & 0xff);
  /*{

    if (svc_num != shizu::kernel_object_svc_num::GET_THREAD_STATE) {
      // svc_cpp_handler の最初に挿入
      shizu::context_t *ctx = get_current_context();
      uintptr_t ctx_addr = reinterpret_cast<uintptr_t>(ctx);

      // ASM が書き込んでいるオフセットを使って PSP を取得（offset は asm
      // と合わせる）
      uintptr_t saved_psp = 0;
      memcpy(&saved_psp, reinterpret_cast<void *>(ctx_addr + 32),
             sizeof(saved_psp)); // <-- 確認: asm のオフセット32と一致するか

      uintptr_t current_psp;
      asm volatile("mrs %0, PSP" : "=r"(current_psp));

      // ダンプ（printf 使用例）
      printf("DBG_CTX: ctx=%lx ctx_addr=%lx saved_psp=%lx current_psp=%lx "
             "ctx->sp=%lx\n",
             (uint32_t)ctx, (uint32_t)ctx_addr, (uint32_t)saved_psp,
             (uint32_t)current_psp, (uint32_t)ctx->sp);

            uint32_t *stacked = reinterpret_cast<uint32_t *>(saved_psp);
            if (stacked) {
              printf("DBG_STACKED: [%08x %08x %08x %08x]\n", stacked[0],
         stacked[1], stacked[2], stacked[3]);
            }


      uint32_t *stacked = reinterpret_cast<uint32_t *>(ctx->sp);
      uint32_t maybe_svc = stacked[0];
      if (maybe_svc > 1000) { // svc id は小さいはず。閾値は実装で調整
        printf("ctx=%lx stacked=%lx maybe_svc=%lx stacked[1]=%lx\n",
               (uint32_t)ctx, (uint32_t)stacked, maybe_svc, stacked[1]);
      }
    }
  }*/

  if (shizu::object_table[current_thread.object_id].state ==
      shizu::object_t::state_t::KERNEL_OBJECT) {

    shizu::kernel_object_svc_num svc_num =
        (shizu::kernel_object_svc_num)(*(uint16_t *)(pc - 2) & 0xff);

    switch (svc_num) {
    case shizu::kernel_object_svc_num::METHOD_CALL: {
      // arg0=対象オブジェクト ID, arg1=メソッド番号, arg2/arg3=引数。
      // 呼び出し元フレームを call_stack に積み、pc を対象の method_table[arg1] へ
      // 張り替える。lr は ::trap (戻ってきてはいけない。復帰は METHOD_EXIT 経由のみ)。
      // current object id を対象へ移すので、メソッド本体は対象オブジェクトの権限で
      // 同一スレッド上を走る (= 保護されたサブルーチン呼び出し)。
      // printf("call to %lx,%lx\n", arg0, arg1);
      // 呼び先が未 export (nullptr) のまま飛ぶと pc=0 → 戻りで trap に落ち、
      // 原因の特定が困難になる。ここで検出して呼び出し元/先を明示して止める。
      // (典型例: スレッド番号の昇順で先に走ったオブジェクトが、まだ main が
      //  走っていないオブジェクトのメソッドを呼んだ場合)
      if (shizu::object_table[arg0].method_table[arg1] == nullptr) {
        panic("METHOD_CALL to unexported method\n"
              " caller obj: %lu\n callee obj: %lu\n method: %lu\n",
              (unsigned long)current_thread.object_id, (unsigned long)arg0,
              (unsigned long)arg1);
      }
      // NOTE(FPU): stack_frame は 8 語の basic exception_frame_t を*その場で*
      // コピーする。拡張(FP)フレームでもこの構造体はハードウェアスタック上の
      // 元の位置に居続けなければならない (フレームを別アドレスへ再配置しない)。
      // FP 拡張分 (S0-S15/FPSCR) は PSP 上に残り、call_stack.context.sp が
      // 指す位置は不変なので、ここは basic 部のみのコピーで正しい。
      if (current_thread.call_stack.full()) {
        panic("call_stack overflow (METHOD_CALL)\n caller obj: %lu\n callee "
              "obj: %lu\n method: %lu\n",
              (unsigned long)current_thread.object_id, (unsigned long)arg0,
              (unsigned long)arg1);
      }
      current_thread.call_stack.push(
          {.stack_frame = *stack_frame,
           .context = *context,
           .caller_object_id = current_thread.object_id});
      /*
        current_thread.call_stack.push(
        {.stack_frame = *stack_frame,
         .context = *context,
         .caller_object_id = current_thread.object_id});
      */
      // printf("pc: %lx\n", stack_frame->pc);
      stack_frame->pc = (uint32_t)shizu::object_table[arg0].method_table[arg1];
      stack_frame->lr = (uint32_t)::trap;
      stack_frame->r0 = current_thread.object_id;
      stack_frame->r1 = current_thread.thread_id;
      stack_frame->r2 = arg2;
      stack_frame->r3 = arg3;
      stack_frame->r12 = r12;
      current_thread.object_id = arg0;
      // printf("pc: %lx\n", stack_frame->pc);
      break;
    }
    case shizu::kernel_object_svc_num::METHOD_EXIT: {
      // call_stack を (arg1+1) 段 pop して元のフレーム/オブジェクト ID を復元する。
      // 復元したフレームの r1 に arg0 (メソッド戻り値) を載せて呼び出し元へ返す。
      shizu::method_call_stack_t call_stack_top;
      for (size_t i = 0; i < arg1 + 1; i++) {
        if (current_thread.call_stack.empty()) {
          panic("call_stack empty");
        } else {
          call_stack_top = current_thread.call_stack.top();
          current_thread.call_stack.pop();
        }
        // NOTE(FPU): basic フレームを元のスタック位置へ書き戻すだけ。フレームは
        // 決して別アドレスへ再配置しないこと (拡張 FP フレームの S0-S15 が同じ
        // PSP 位置に残っており、sp を張り替えると FP 分と乖離する)。
        *get_current_context() = call_stack_top.context;
        stack_frame = call_stack_top.context.sp;
        *stack_frame = call_stack_top.stack_frame;
        current_thread.object_id = call_stack_top.caller_object_id;
        stack_frame->r0 = 0;
        stack_frame->r1 = arg0;
      }

      break;
    }
    case shizu::kernel_object_svc_num::SET_SVC_HANDLER: {
      shizu::cpu_manager::svc_handler_info.entry_point = (shizu::method_t)arg0;
      shizu::cpu_manager::svc_handler_info.handling_object_id =
          shizu::thread_table[shizu::cpu_manager::current_thread_id[get_core_num()]]
              .object_id;
      break;
    }

    case shizu::kernel_object_svc_num::GET_CURRENT_THREAD_ID: {
      stack_frame->r1 = shizu::cpu_manager::current_thread_id[get_core_num()];
      stack_frame->r0 = 0;
      break;
    }
    case shizu::kernel_object_svc_num::SWITCH_THREAD: { // = TRY_SWITCH_THREAD
      // check + claim + switch を 1 SVC に畳んだ原子操作。r0 に switch_error を返す。
      // 「READY を見てから別 SVC で切替」を 2 コアでやると TOCTOU レースになるので、
      // ここで CAS 込みの単一操作にしておく (単一コアでも意味論は等価)。
      const uint32_t core = get_core_num();
      const uint32_t cur = shizu::cpu_manager::current_thread_id[core];
      // grant アクティブ中の switch 発行 = grantee の早期復帰。対象 (arg0) は無視して
      // 最寄りの grantor へ 1 段 pop する (時限移譲は「呼び出し」であり、grantee の
      // yield は「早期 return」)。grantee は READY へ戻り、再開時の r0 は OK。
      if (shizu::grant_stacks[core].depth != 0) {
        stack_frame->r0 = (uint32_t)shizu::switch_error::OK;
        shizu::grant_pop_to_grantor(core, shizu::grant_end::YIELDED);
        break;
      }
      if (arg0 == cur) {
        stack_frame->r0 = (uint32_t)shizu::switch_error::OK; // 自スレッドへは no-op
        break;
      }
      // claim: READY→RUNNING の CAS (try_claim)。これが「2 コアが同一 context_t を
      // 走らせない」根 = context の所有権ロックそのもの (別途 LOCK_CONTEXT は不要)。
      shizu::switch_error claim_result =
          try_claim(shizu::thread_table[arg0], core);
      if (claim_result != shizu::switch_error::OK) {
        stack_frame->r0 = (uint32_t)claim_result;
        break;
      }
      shizu::cpu_manager::current_thread_id[core] = arg0;
      // 旧スレッドを release。RELEASE store 以降、他コアが cur を claim してよい
      // (SVC 入口 asm が cur の context 退避を完了済みなので自己完結)。
      __atomic_store_n((uint32_t *)&shizu::thread_table[cur].state,
                       (uint32_t)shizu::thread_t::state_t::READY,
                       __ATOMIC_RELEASE);
      stack_frame->r0 = (uint32_t)shizu::switch_error::OK;
      break;
    }
    case shizu::kernel_object_svc_num::GRANT_CPU: {
      // 時限実行権移譲: arg0=対象 tid, arg1=最大実行時間[µs]。対象を claim して
      // 切替え、期限到来 (SysTick→PendSV) か対象の yield/switch (早期復帰) で
      // この SVC から戻る。戻り: r0=grant_error, r1=grant_end。
      const uint32_t core = get_core_num();
      const uint32_t cur = shizu::cpu_manager::current_thread_id[core];
      shizu::grant_stack_t &gs = shizu::grant_stacks[core];
      if (arg0 == cur) { // 自分への移譲は no-op (即 YIELDED 扱い)
        stack_frame->r0 = (uint32_t)shizu::grant_error::OK;
        stack_frame->r1 = (uint32_t)shizu::grant_end::YIELDED;
        break;
      }
      if (gs.depth >= shizu::grant_stack_t::MAX_DEPTH) {
        stack_frame->r0 = (uint32_t)shizu::grant_error::DEPTH;
        break;
      }
      shizu::switch_error claim_result =
          try_claim(shizu::thread_table[arg0], core);
      if (claim_result != shizu::switch_error::OK) {
        // grant_error の先頭 3 値は switch_error と同値なのでそのまま写せる。
        stack_frame->r0 = (uint32_t)claim_result;
        break;
      }
      // 期限はネストの外側 deadline でクランプ (over-time の構造的禁止)。
      uint64_t deadline = time_us_64() + (uint64_t)arg1;
      if (gs.depth > 0 && gs.frames[gs.depth - 1].deadline < deadline)
        deadline = gs.frames[gs.depth - 1].deadline;
      gs.frames[gs.depth++] = {cur, deadline};
      // grantor の復帰値を先に用意: r0=OK 確定。r1 (EXPIRED/YIELDED) は復帰経路
      // (pendsv_cpp_handler / SWITCH_THREAD インターセプト) が書く。
      stack_frame->r0 = (uint32_t)shizu::grant_error::OK;
      // grantor は WAIT_GRANT — READY でないので他コアに claim されず、復帰は
      // このコアの pop 経路のみ (単独所有につき plain store で足りる)。
      shizu::thread_table[cur].state = shizu::thread_t::state_t::WAIT_GRANT;
      shizu::cpu_manager::current_thread_id[core] = arg0;
      shizu::systick_arm_for(deadline);
      break;
    }
    case shizu::kernel_object_svc_num::GET_THREAD_STATE: {
      stack_frame->r1 = (uint32_t)shizu::thread_table[arg0].state;
      stack_frame->r0 = 0;
      break;
    }
    case shizu::kernel_object_svc_num::GET_OBJECT_ID_FROM_THREAD_ID: {
      stack_frame->r1 = (uint32_t)shizu::thread_table[arg0].object_id;
      stack_frame->r0 = 0;
      break;
    }
    default:
      break;
    }
  } else {
    // 発行元が一般オブジェクトの場合: 直接は処理せず、登録済みの
    // SVC ハンドラ (= カーネルオブジェクトの kernel_obj_svc_handler) へ
    // トランポリンする。現フレームを積み、pc をハンドラ入口へ、戻り先 lr を
    // exit_method へ張り替え、current object をハンドラの所有者 (カーネルオブジェクト)
    // へ移す。元の svc 番号 / 呼び出し元 obj / 呼び出し元 thread を callee-saved な
    // r4/r5/r6 に載せて渡す (ハンドラ側はこれを引数として受け取る)。
    uint32_t caller_obj_num = current_thread.object_id;
    uint32_t caller_thread_id = current_thread.thread_id;
    if (current_thread.call_stack.full()) {
      panic("call_stack overflow (svc trampoline)\n caller obj: %lu\n thread: "
            "%lu\n",
            (unsigned long)caller_obj_num, (unsigned long)caller_thread_id);
    }
    current_thread.call_stack.push(
        {.stack_frame = *stack_frame,
         .context = *context,
         .caller_object_id = current_thread.object_id});
    stack_frame->pc =
        (uint32_t)shizu::cpu_manager::svc_handler_info.entry_point;
    stack_frame->lr = (uint32_t)shizu::FOR_KERNEL_OBJECT::exit_method;
    current_thread.object_id =
        shizu::cpu_manager::svc_handler_info.handling_object_id;
    current_thread.context->r4 = (uint32_t)svc_num;
    current_thread.context->r5 = caller_obj_num;
    current_thread.context->r6 = caller_thread_id;
  }
}