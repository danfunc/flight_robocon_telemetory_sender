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
#include <hardware/regs/addressmap.h>   // XIP_BASE/XIP_END/SRAM_END (MPU region)
#include <hardware/structs/mpu.h>       // mpu_hw (banked per-core)
#include <hardware/structs/scb.h>       // scb_hw->icsr (PendSV pend)
#include <hardware/structs/systick.h>   // systick_hw (banked per-core)
#include <cstdarg>
#include <hardware/sync.h> // __dmb (panic スロットの publish 順序)
#include <kernel.hpp>
#include <panic_ring.hpp>
#include <pico/stdlib.h>
#include <shizu.hpp>

static_assert(offsetof(shizu::context_t, sp) == 32,
              "context_t layout mismatch: psp offset");
// svc_asm_handler.S の .equ CTX_EXC_RETURN / CTX_FP と一致していること。
static_assert(offsetof(shizu::context_t, exc_return) == 36,
              "context_t layout mismatch: exc_return offset (asm CTX_EXC_RETURN)");
static_assert(offsetof(shizu::context_t, fp) == 40,
              "context_t layout mismatch: fp offset (asm CTX_FP)");
static_assert(offsetof(shizu::context_t, psplim) == 104,
              "context_t layout mismatch: psplim offset (asm CTX_PSPLIM)");
static_assert(offsetof(shizu::context_t, control) == 108,
              "context_t layout mismatch: control offset (asm CTX_CONTROL)");
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

// ---------------------------------------------------------------------------
//  カーネル専用データプール (kernel.hpp 冒頭のコメント参照) — malloc ヒープ
//  (__end__ 以降) と物理的に分離した .bss 常駐領域。MPU Step1 で不特権オブジェクトへ
//  渡す region から自然に外れる (リンカ配置変更なし)。
// ---------------------------------------------------------------------------
// context_t: thread_id で直引きする固定 128 スロット。.bss ゼロ初期化 = 旧
// `new context_t()` (POD の value-init = ゼロ初期化) と同じ既定値なので挙動不変。
static context_t g_context_pool[128];

context_t *kernel_context_for(uint32_t thread_id) {
  return &g_context_pool[thread_id];
}

// call_stack フレーム用のカーネルアリーナ (bump allocator, 解放なし = 現行 malloc
// 運用と同じ「スレッド寿命 = プロセス寿命、解放パス無し」の前提を継承)。実運用の
// スレッド数 (本番構成で ~10、selftest 込みでも ~30 程度) に対して 32 スレッド分の
// 余裕を静的確保し、超過時だけ malloc へフォールバックする (性能でなく安全側: 想定を
// 超えた場合に panic させず、分離の恩恵をその分だけ諦めて動作を継続する)。
// ★32 → 20 (2026-08-13)。call_stack_t::MAX_DEPTH を 16→32 に倍増したので、枠を
// 据え置くとアリーナも倍 (86KB→172KB) になり、8KB/本 のスレッドスタックを malloc する
// ヒープが 200KB→116KB まで削られて 14 本で尽きる。実スレッド数は本番構成 ~10 /
// selftest 込みで ~15 なので、枠を実数へ寄せて総量をほぼ据え置く (20×32×168B=107KB)。
// これを超えた分は下の malloc フォールバックが受ける (警告を 1 行出して動作継続)。
// 呼び出しフレーム用のカーネルアリーナは撤去した (2026-08-13)。退避先はスレッド
// 自身のスタック (include/kernel.hpp の call_frame_hdr_t の幾何を参照)。実際に
// ネストした分しか消費せず、溢れは PSPLIM が捕まえる。旧実装は 20 スレッド ×
// 32 段 × 168B = 105KB を .bss へ予約していた。

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

// ---------------------------------------------------------------------------
//  panic リング (panic_ring.hpp) — noinit RAM なのでリセットを跨いで残る
// ---------------------------------------------------------------------------
panic_slot_t __uninitialized_ram(panic_slots)[2];
#if SHIZU_SVC_DELEGATION
// svc 委譲の計装リング (noinit = リセットを跨いで残る)。
svc_trace_t __uninitialized_ram(svc_trace);
// 記録は最小コスト (ロック無し・printf 無し)。固まる直前の数件が残ればよい。
void svc_trace_record(uint32_t num, uint32_t obj, uint32_t in_handler,
                      bool primitive, bool trampoline, uint32_t pc) {
  if (svc_trace.magic != SVC_TRACE_MAGIC) {
    svc_trace.magic = SVC_TRACE_MAGIC;
    svc_trace.n = 0;
  }
  svc_trace_t::entry_t &e = svc_trace.e[svc_trace.n % SVC_TRACE_SLOTS];
  e.num = (uint16_t)num;
  e.obj = (uint8_t)obj;
  e.flags = (in_handler ? 1u : 0u) | (primitive ? 2u : 0u) | (trampoline ? 4u : 0u);
  e.pc = pc;
  __dmb();
  svc_trace.n = svc_trace.n + 1;
}

void svc_trace_boot_report() {
  if (svc_trace.magic != SVC_TRACE_MAGIC)
    return;
  const uint32_t n = svc_trace.n;
  const uint32_t shown = n < SVC_TRACE_SLOTS ? n : SVC_TRACE_SLOTS;
  printf("[SVCTRACE] 前回の残骸 %lu 件 (総 %lu):\n", (unsigned long)shown,
         (unsigned long)n);
  for (uint32_t i = 0; i < shown; ++i) {
    const uint32_t idx = (n - shown + i) % SVC_TRACE_SLOTS;
    const svc_trace_t::entry_t &e = svc_trace.e[idx];
    printf("  #%lu num=%u obj=%u %s%s%s pc=%08lx\n", (unsigned long)(n - shown + i),
           (unsigned)e.num, (unsigned)e.obj,
           (e.flags & 1) ? "in_handler " : "",
           (e.flags & 2) ? "primitive " : "",
           (e.flags & 4) ? "->trampoline" : "->direct", (unsigned long)e.pc);
  }
  svc_trace.magic = 0;
}
#endif

static void panic_slot_print(const panic_slot_t &s, const char *tag) {
  printf("[PANIC-RING] %s core%lu exc=%lu thr=%lu t=%llums: %.*s\n", tag,
         (unsigned long)s.core, (unsigned long)s.exception,
         (unsigned long)s.thread_id, (unsigned long long)(s.time_us / 1000),
         (int)sizeof(s.msg), s.msg);
}

void panic_ring_boot_report() {
  for (int c = 0; c < 2; ++c) {
    if (panic_slots[c].magic == PANIC_MAGIC)
      panic_slot_print(panic_slots[c], "(前回実行の残骸)");
    panic_slots[c].magic = 0;
    panic_slots[c].seq = 0;
  }
}

void panic_ring_poll_report() {
  static uint32_t reported_seq[2] = {0, 0};
  for (int c = 0; c < 2; ++c) {
    if (panic_slots[c].magic == PANIC_MAGIC &&
        panic_slots[c].seq != reported_seq[c]) {
      reported_seq[c] = panic_slots[c].seq;
      panic_slot_print(panic_slots[c], "LIVE");
    }
  }
}

} // namespace shizu (一時的に閉じる: shizu_panic は C リンケージのグローバル)

// panic() の差し替え先 (CMakeLists: PICO_PANIC_FUNCTION=shizu_panic)。
// ロック/stdio を一切使わず自コアのスロットへ記録して戻る (戻った先で SDK の
// bkpt ループ → HardFault ダンプは従来どおり)。vsnprintf は pico_printf の
// 実装でロック・malloc を使わない。
extern "C" void shizu_panic(const char *fmt, ...) {
  const uint32_t core = get_core_num();
  shizu::panic_slot_t &s = shizu::panic_slots[core];
  s.core = core;
  s.exception = __get_current_exception();
  s.thread_id = shizu::cpu_manager::current_thread_id[core];
  s.time_us = time_us_64();
  if (fmt) {
    va_list args;
    va_start(args, fmt);
    vsnprintf(s.msg, sizeof(s.msg), fmt, args);
    va_end(args);
  } else {
    s.msg[0] = '\0';
  }
  s.seq = s.seq + 1;
  __dmb();
  s.magic = shizu::PANIC_MAGIC; // 最後に有効化 (途中クラッシュで半端を残さない)
}

namespace shizu {

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

// ---------------------------------------------------------------------------
//  MPU Step0: W^X (HANDOFF §6 の段 0 の残り半分。PSPLIM とセット)
// ---------------------------------------------------------------------------
//  目的はバグ封じ込め: (i) スタック/ヒープ上に化けた関数ポインタ経由で「データを
//  実行」する事故を XN で即 fault に、(ii) flash (XIP 窓) への誤ストアを RO で即
//  fault にする。PMSAv8 は有効 region の overlap 禁止なので「広い RW に穴あけ」は
//  できない — RAM 常駐コード (.data 内 time_critical) を避けるため region はヒープ
//  先頭 (__end__) から張る。scratch_x/y (コア MSP スタック) はコード無し (map で
//  確認済み) なので SRAM 終端まで丸ごと XN に含める。
//  PRIVDEFENA=1: region 外 (静的データ / ペリフェラル / SIO / USB RAM) は特権
//  default map のまま = 現行コードは無傷。全スレッド特権のまま (nPRIV 分離は
//  Step1)。DMA は MPU を素通りする (バスマスタ) — DMA 設定をカーネル専有に保つ
//  方針とセットで成立する保護であることに注意。
//  MemManage は個別有効化せず HardFault へエスカレートさせる (PSPLIM と同じ扱い、
//  main.cpp の hardfault_handler がダンプを出す)。
#if SHIZU_MPU_WX
extern "C" char __end__[]; // リンカ: 静的データ終端 = ヒープ先頭
namespace {
// v8-M MPU region を 1 本張る。base/limit は 32B 粒度 (inclusive limit)。
// ap: RBAR.AP (0b01=RW 全特権, 0b11=RO 全特権)。xn: 1=実行禁止。
// attridx: MAIR0 の属性スロット番号。
void mpu_set_region(uint32_t idx, uint32_t base, uint32_t limit_incl,
                    uint32_t ap, uint32_t xn, uint32_t attridx) {
  mpu_hw->rnr = idx;
  mpu_hw->rbar = (base & ~0x1Fu) | (ap << 1) | xn;
  mpu_hw->rlar = (limit_incl & ~0x1Fu) | (attridx << 1) | 1u;
}
} // namespace
void mpu_init_this_core() {
  // MAIR: attr0 = Normal WB read/write-allocate (XIP キャッシュ経路の妥当な属性)、
  //       attr1 = Normal non-cacheable (SRAM。M33 に D キャッシュは無い)。
  mpu_hw->mair[0] = 0xFFu | (0x44u << 8);
  // region0: XIP flash 全窓 = RO + X。実行と読みは今まで通り、ストアは即 fault。
  // (flash_range_program は QMI/ROM 経由で XIP 窓へストアしないので併用可能。)
  mpu_set_region(0, XIP_BASE, XIP_END - 32u, 0b11, 0, 0);
  // region1: ヒープ先頭〜SRAM 終端 = RW + XN。全スレッドスタック (malloc 産) と
  // ヒープ、scratch_x/y の MSP スタックからのコード実行を禁じる。
  const uint32_t heap = ((uint32_t)__end__ + 31u) & ~31u;
  mpu_set_region(1, heap, SRAM_END - 32u, 0b01, 1, 1);
  // 残り region は明示無効化 (ブートローダ等の残骸を引き継がない保険)。
  for (uint32_t i = 2; i < 8; ++i) {
    mpu_hw->rnr = i;
    mpu_hw->rlar = 0;
  }
  // PRIVDEFENA | ENABLE。HFNMIENA=0 (HardFault ハンドラは MPU 制約なしで走らせる)。
  mpu_hw->ctrl = M33_MPU_CTRL_PRIVDEFENA_BITS | M33_MPU_CTRL_ENABLE_BITS;
  __dsb();
  __isb();
}
#else
void mpu_init_this_core() {}
#endif

void cpu_manager::init() {
  exception_set_exclusive_handler(SVCALL_EXCEPTION, svc_asm_handler);
  // grant 期限強制の土台。RAM ベクタテーブルは両コア共有なので登録はここ (core0)
  // の 1 回だけ (core1 で再登録すると exclusive 二重登録 panic)。
  exception_set_exclusive_handler(PENDSV_EXCEPTION, pendsv_asm_handler);
  exception_set_exclusive_handler(SYSTICK_EXCEPTION, systick_grant_handler);
  g_cycles_per_us = clock_get_hz(clk_sys) / 1000000u;
  init_exception_priorities();
  mpu_init_this_core(); // MPU は per-core banked。core1 は init_core1 が呼ぶ
}

cpu_manager::svc_handler_descriptor cpu_manager::svc_handler_info{nullptr, 0};


// MPU Step1 プロトタイプ: obj_id が現在の「実行中オブジェクト」であるとき、
// CONTROL レジスタが持つべき値 (nPRIV ビット) を object_table[obj_id].unprivileged
// から導出する。svc_cpp_handler の METHOD_CALL/トランポリンと create_thread が使う
// 唯一の導出点 (「特権は current-object の属性」という不変条件をここに集約する)。
static inline uint32_t control_for_object(uint32_t obj_id) {
  return object_table[obj_id].unprivileged ? CONTROL_UNPRIV_PSP
                                           : CONTROL_PRIV_PSP;
}

// exit_method の svc が「戻ってきてしまった」= 巻き戻しがカーネルの検算で弾かれた
// ときの落ち先 (naked な exit_method から b で飛ぶ)。r0/r1 に METHOD_EXIT の戻り
// (exit_error と実際のネスト数) がそのまま乗っているので、そのまま診断に使える。
// 巻き戻せない以上ここから復帰する先が無いので panic する。
extern "C" [[noreturn]] void exit_method_failed(uint32_t err,
                                                uint32_t actual_depth) {
  // exit_method / delegate_exit_method の svc が「戻ってきてしまった」= 巻き戻しが
  // カーネルの検算で弾かれたときの落ち先。
  // err: 1=DEPTH_MISMATCH (申告したネスト数が実際と違う) / 2=BAD_COUNT。
  //
  // ★**panic しない**。段数の申告はオブジェクト側の責任なので、間違えた者だけが
  //   止まるべきで、系全体を道連れにするのは方針として誤り (CPU 予算の超過が
  //   guest から取り上げるだけで系を殺さないのと同じ)。
  //   ・shizu_panic() は panic スロットへ記録して**戻る**だけなので、診断は残る
  //     (ビーコン IRQ の panic_ring_poll_report が拾って印字する)。
  //   ・その後このスレッドは yield ループに入って永久に実務へ戻らない = 隔離。
  //     他のスレッド/コアは動き続ける。
  shizu_panic("exit unwind rejected: err=%lu actual_depth=%lu (thread parked)\n",
              (unsigned long)err, (unsigned long)actual_depth);
  while (true) {
    // obj_api の YIELD (旧 r0 ディスパッチ: 即値 0 / r0=0)。一般オブジェクトからも
    // カーネルオブジェクトからも撃てる唯一の共通経路。
    asm volatile("movs r0, #0\n"
                 "movs r1, #0\n"
                 "movs r2, #0\n"
                 "movs r3, #0\n"
                 "svc  0\n" ::: "r0", "r1", "r2", "r3", "memory");
  }
}

// ---------------------------------------------------------------------------
//  ★ARCH SEAM — アトミックな枠取り
// ---------------------------------------------------------------------------
//  「UNINITIALIZED なら RESERVED にする」を不可分に行う。**ここが ISA/ボード依存の
//  継ぎ目**で、移植時に差し替える点。同一ベンダの 2 チップで答えが真逆になる:
//    ・RP2350 (ARMv8-M): LDREX/STREX (= __atomic 系) を使う。SIO ハードウェア
//      スピンロックは **E2 erratum で使えない**
//    ・RP2040 (ARMv6-M): **LDREX/STREX がそもそも無い**。マルチコアでは SIO
//      ハードウェアスピンロックを使うしかない
//  → 新実装では `arch` テンプレートのパラメータとして外から与えること
//     (docs/SHIZUKU_REDESIGN_PORTABILITY.md §5.1)。ここは RP2350 用の実装。
static bool arch_claim_thread_slot(thread_t &t) {
  uint32_t expected = (uint32_t)thread_t::state_t::UNINITIALIZED;
  return __atomic_compare_exchange_n(
      (uint32_t *)&t.state, &expected, (uint32_t)thread_t::state_t::RESERVED,
      false, __ATOMIC_ACQUIRE, __ATOMIC_RELAXED);
}

namespace FOR_KERNEL_OBJECT {

void create_object(uint32_t obj_num, uint32_t thread_num, method_t entry) {
  shizu::create_object(obj_num);
  shizu::FOR_KERNEL_OBJECT::create_thread(obj_num, thread_num, entry);
}

// オブジェクトのみ生成 (スレッド無し)。起動は別途 async_call(id, main) で行う。
void create_object(uint32_t obj_num) { shizu::create_object(obj_num); }

// 空きスレッドスロット (1..127, 0 はカーネルオブジェクト予約) を自動確保して entry を
// 非同期起動する。戻り = 割り当てた thread_id。
uint32_t async_call(uint32_t obj_num, method_t entry, uint32_t arg,
                    uint32_t affinity, uint32_t budget_us) {
  for (uint32_t t = 1; t < 128; ++t) {
    // ★「空いているか見てから作る」は 2 コアで TOCTOU になる。両コアが同じ枠を
    //   UNINITIALIZED と観測すると、片方は create_thread の二重初期化検査で
    //   **panic して系ごと落ちる** (I-9 違反)。CAS で枠を先に予約する。
    if (arch_claim_thread_slot(thread_table[t])) {
      shizu::FOR_KERNEL_OBJECT::create_thread(obj_num, t, entry, arg, affinity,
                                              budget_us);
      return t;
    }
  }
  panic("async_call: no free thread slot (all 127 in use)");
  return 0;
}

void create_thread(uint32_t obj_num, uint32_t thread_num, method_t entry,
                   uint32_t arg, uint32_t affinity, uint32_t budget_us) {
  // 枠は「RESERVED 済み (async_call が CAS で取った)」か「これから CAS で取る
  // (明示 thread_num 指定の経路)」のどちらか。どちらでもなければ二重初期化。
  if (thread_table[thread_num].state == thread_t::state_t::RESERVED ||
      arch_claim_thread_slot(thread_table[thread_num])) {
    thread_t &t = thread_table[thread_num];
    t.object_id = obj_num;
    // context_t はカーネル専用の固定プールから (malloc しない。kernel.hpp 冒頭
    // コメント参照)。.bss ゼロ初期化済みなので旧 `new context_t()` と初期値は等価。
    t.context = kernel_context_for(thread_num);
    // BLE/CYW43 (btstack) の送信・暗号化パスは深く、2KB ではオーバーフローして
    // HardFault する。1 スレッドあたり 8KB へ拡張 (RP2350 は RAM に余裕あり)。
    constexpr uint32_t THREAD_STACK_SIZE = 8192;
    uint32_t stack_base = (uint32_t)malloc(THREAD_STACK_SIZE);
    t.context->sp =
        (exception_frame_t *)((stack_base + THREAD_STACK_SIZE -
                               sizeof(exception_frame_t)) &
                              ~((uint32_t)0xfL));
    // PSPLIM = スタック底 (8B アライン上げ)。PSP がここを下回ると即フォルト。
    t.context->psplim = (stack_base + 7u) & ~7u;
    t.context->sp->pc = (uint32_t)entry;
    t.context->sp->lr = (uint32_t)shizu::FOR_KERNEL_OBJECT::exit_method;
    t.context->sp->r0 = arg; // entry の第1引数
    t.context->sp->r1 = 0;
    t.context->sp->r2 = 0;
    t.context->sp->r3 = 0;
    t.context->sp->r12 = 0;
    t.context->sp->xPSR = (1 << 24);
    // 新規スレッドは基本フレーム (FP 無効) の Thread/PSP へ復帰する。
    // 0 のままだと初回復帰で bx 0 → 即 HardFault。必ず明示的に種を入れる。
    t.context->exc_return = 0xFFFFFFFD;
    // MPU Step1 プロトタイプ: このスレッドの「現在オブジェクト」= 生成先 obj_num の
    // 特権属性から CONTROL 初期値を確定する (既定は全オブジェクト特権なので不変)。
    t.context->control = control_for_object(obj_num);
    // 呼び出しスタックはカーネル専用アリーナから create 時 (スレッドモード) に
    // 1 回だけ確保する。以後 SVC 例外ハンドラ内 (METHOD_CALL / トランポリン) では
    // malloc しない (アリーナ超過時のみ内部でフォールバックする)。
    t.call_stack.top = 0; // 退避先はスレッドスタック (事前確保なし)
    t.call_stack.depth = 0;
    object_table[obj_num].thread_table.insert(thread_num);
    // 生成時オプション (0 / BUDGET_KEEP = thread_table_init の既定を維持)。
    // READY 公開前に確定させることが本質: 別コア行き (affinity=CORE1 等) の
    // スレッドが「既定 core0 のまま走り出してから移動」する隙間を作らない。
    if (affinity != 0)
      t.affinity = affinity & AFFINITY_ALL;
    if (budget_us != BUDGET_KEEP)
      t.grant_budget_us = budget_us;
    // READY は最後に release で公開。claim 側 (try_claim の ACQUIRE CAS) が READY を
    // 観測した時点で、上の初期化 (context/スタック/affinity/budget) が全て可視になる
    // — 旧実装は初期化冒頭で READY にしており、他コア affinity だと途中初期化のまま
    // claim され得た (これまで顕在化しなかったのは既定 affinity が生成コア=core0 で
    // 生成中は core0 が busy だったため)。
    __atomic_store_n((uint32_t *)&t.state, (uint32_t)thread_t::state_t::READY,
                     __ATOMIC_RELEASE);
  } else {
    panic("this thread is already initialized\ncalling this system call for "
          "\n thread_id: %d\n object_id: %d\n",
          thread_num, obj_num);
  };
}
void export_method(uint32_t obj_num, uint32_t method_num, method_t entry) {
  shizu::object_table[obj_num].method_table[method_num] = entry;
}
// ハンドラ活性化 / スレッドエントリの lr に載る「落ちてきたときの戻り口」。
// r0 = 戻り値 (C の return 値がそのまま乗る), r7 = カーネルが活性化時に打った
// 「今のネスト数」。r7 をそのまま arg3 に載せるので、この経路の 1 段 pop も
// カーネル側の検算を通る (旧実装は無検査で、段数がズレると無音でロックアップした)。
// naked なのは r7 を確実に捕まえるため — 通常関数だとプロローグで壊され得る。
__attribute__((naked)) void exit_method(uint32_t return_code) {
  (void)return_code;
  asm volatile("mov  r1, #0\n"  // arg1 = 追加 pop 段数 0 → 1 枚だけ落とす
               "mov  r2, #0\n"  // arg2 = エラーコード (成功)
               "mov  r3, r7\n"  // arg3 = 申告する「今のネスト数」
               "svc  %[num]\n"  // 戻ってきた = 検算で弾かれた (成功なら戻らない)
               "b    exit_method_failed\n"
               :
               : [num] "i"((uint32_t)shizu::kernel_object_svc_num::METHOD_EXIT)
               :);
};

// 委譲サブハンドラ (一般オブジェクト) 用の戻り口。生のカーネルプリミティブ (201) は
// 一般オブジェクトからは撃てない (撃つとトランポリンに乗る) ので、**obj_api の
// EXIT_METHOD** を使う。これはカーネルオブジェクトのハンドラ側で
// METHOD_EXIT(値, 追加1段) に変換される = 「この exit 自身のトランポリン枠 + 自分の
// 活性化枠」の 2 枚巻き戻し。段数はカーネルが arg3 の申告と突き合わせて検算する。
// ★これが「一般オブジェクトに特権を渡さずに戻る」ための唯一の口。ここを exit_method
// (生 201) にすると一般オブジェクトがカーネルプリミティブを叩けることになり、
// SET_SVC_HANDLER 等のバイパスに繋がる。
__attribute__((naked)) void delegate_exit_method(uint32_t return_code) {
  (void)return_code;
  asm volatile("mov  r1, r0\n"   // obj_api の引数は r1..r3 (r0 は予約)
               "mov  r0, #0\n"
               "mov  r2, #0\n"
               "mov  r3, #0\n"
               "svc  %[num]\n"   // obj_api::EXIT_METHOD (即値ディスパッチ)
               "b    exit_method_failed\n"
               :
               : [num] "i"((uint32_t)shizu::obj_api_EXIT_METHOD_NUM)
               :);
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


// ---------------------------------------------------------------------------
//  呼び出しフレームの push/pop — 退避先はスレッド自身のスタック
//  幾何とFP の扱いは include/kernel.hpp の call_frame_hdr_t のコメントを参照。
// ---------------------------------------------------------------------------
// 積む。成功で true。失敗 (PSPLIM に届く) は false — 呼び出し側は panic せず
// エラー復帰させること。成功時 *frame_io は「書き換え用フレーム」を指すよう更新する。
static bool call_frame_push(shizu::thread_t &t, shizu::context_t *ctx,
                            shizu::exception_frame_t **frame_io,
                            uint32_t caller_obj) {
  // EXC_RETURN bit4=0 → FP 拡張フレーム (基本 32B + S0-S15/FPSCR/予約 = 104B)。
  const bool has_fp = (ctx->exc_return & 0x10u) == 0;
  const uint32_t fs = has_fp ? 104u : 32u;
  uint32_t total = (uint32_t)sizeof(shizu::call_frame_hdr_t) +
                   (has_fp ? 64u : 0u) + fs;
  total = (total + 7u) & ~7u; // 8B アライン維持 (xPSR bit9 のパディング整合)
  const uint32_t sp_old = (uint32_t)ctx->sp; // = X - fs (元の例外フレーム位置)
  const uint32_t X = sp_old + fs;            // 呼び出し元の SVC 直前の PSP
  const uint32_t snap = X - total;           // 退避域の先頭
  // ★8B の余裕。呼び出し元の SVC 発行時に SP が 8B 境界でないと、ハードウェアが
  //   フレームにパディングを入れて xPSR bit9 を立て、**例外復帰時に SP へ +4 余分に
  //   足す**。それを見込まないと呼び先の開始 PSP が退避域へ 4B 食い込み、直後の
  //   プッシュでヘッダ先頭 (prev) を潰して連鎖が壊れる。
  const uint32_t new_sp = snap - fs - 8;     // 書き換え用フレームの置き場
  // PSPLIM 手前で止める (呼び先のプロローグぶんの余裕も見る)。ここで false を返せば
  // 呼び出し側がエラーを返すので、スタック不足が無音ロックアップにならない。
  if (ctx->psplim != 0 && new_sp < ctx->psplim + 64u)
    return false;
  shizu::call_frame_hdr_t *h = (shizu::call_frame_hdr_t *)snap;
  h->prev = t.call_stack.top;
  h->total_bytes = total;
  h->frame_bytes = fs;
  h->caller_object_id = caller_obj;
  h->control = ctx->control;
  h->exc_return = ctx->exc_return;
  h->has_fp = has_fp ? 1u : 0u;
  h->pad = 0;
  memcpy(h->r4_r11, &ctx->r4, sizeof(h->r4_r11));
#if SHIZU_SVC_DELEGATION
  h->in_handler = ctx->in_handler;
  h->inv_svc_num = ctx->inv_svc_num;
  h->inv_caller_obj = ctx->inv_caller_obj;
  h->inv_caller_thread = ctx->inv_caller_thread;
#endif
  // FP 活性のときだけ S16-S31 の枠を使う (非活性なら 64B ぶん退避域が小さい)。
  if (has_fp)
    memcpy((void *)(snap + sizeof(shizu::call_frame_hdr_t)), ctx->fp, 64);
  // ★元の例外フレーム [X-fs, X) は退避域の一番上にそのまま残す = 一切動かさない。
  //   (FP 拡張の S0-S15 と乖離させないための最重要点。) 書き換えるのは下へ複製した
  //   作業コピーの方で、呼び先はそれを pop して PSP=snap から下へ伸びる。
  memcpy((void *)new_sp, (const void *)sp_old, fs);
  ctx->sp = (shizu::exception_frame_t *)new_sp;
  *frame_io = ctx->sp;
  t.call_stack.top = snap;
  t.call_stack.depth++;
  return true;
}

// 1 枚戻す。空なら false。元の例外フレームは退避域の中に生きているので指すだけ。
static bool call_frame_pop(shizu::thread_t &t, shizu::context_t *ctx,
                           shizu::exception_frame_t **frame_io) {
  if (t.call_stack.top == 0)
    return false;
  shizu::call_frame_hdr_t *h = (shizu::call_frame_hdr_t *)t.call_stack.top;
  // ★幾何は push 時に記録した値を読み戻す (再計算しない)。呼び先の実行中に FP 状態が
  //   変わっても push/pop で寸法がズレない。
  const uint32_t X = t.call_stack.top + h->total_bytes;
  memcpy(&ctx->r4, h->r4_r11, sizeof(h->r4_r11));
  ctx->control = h->control;
  ctx->exc_return = h->exc_return;
#if SHIZU_SVC_DELEGATION
  ctx->in_handler = h->in_handler;
  ctx->inv_svc_num = h->inv_svc_num;
  ctx->inv_caller_obj = h->inv_caller_obj;
  ctx->inv_caller_thread = h->inv_caller_thread;
#endif
  if (h->has_fp)
    memcpy(ctx->fp,
           (const void *)(t.call_stack.top + sizeof(shizu::call_frame_hdr_t)),
           64);
  t.object_id = h->caller_object_id;
  ctx->sp = (shizu::exception_frame_t *)(X - h->frame_bytes);
  *frame_io = ctx->sp;
  t.call_stack.top = h->prev;
  t.call_stack.depth--;
  return true;
}

// 最内フレームに保存されている「発行元の例外フレーム」。REDISPATCH_SVC が元の
// syscall の r0..r3 を回収するのに使う。
static shizu::exception_frame_t *
call_frame_saved_frame(const shizu::call_stack_t &cs) {
  if (cs.top == 0)
    return nullptr;
  shizu::call_frame_hdr_t *h = (shizu::call_frame_hdr_t *)cs.top;
  return (shizu::exception_frame_t *)(cs.top + h->total_bytes - h->frame_bytes);
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

#if SHIZU_SVC_DELEGATION
  // ハンドラ活性化から撃たれた svc は「**カーネルオブジェクト宛**の svc」と見做す。
  // ★発行元をカーネルオブジェクト**扱いにする**のとは違う (object_id は書き換えない)。
  //   宛先がカーネルオブジェクトなら、その届け先は 2 つある:
  //     ・カーネルが**プリミティブとして実装している番号** → ここで直接実行
  //       (これによりハンドラは METHOD_EXIT を自分で叩け、戻るためだけの svc と
  //        その余分なフレームが消える = 1 枚 pop)
  //     ・それ以外 → カーネルオブジェクトの**登録ハンドラ** (obj_api の実装) へ
  //       トランポリン。これを落とすとハンドラから obj_api も他サブシステムへの
  //       委譲も撃てなくなる (実機で多段が塞がった原因はこれだった)。
  //   番号は「経路を決めた後のプリミティブ選択」に使うだけで、経路を番号で分けては
  //   いない — 選べなかったときの既定を「黙殺」から「委譲」に戻しただけ。
  // 即値 0 は旧 ABI (r0 ディスパッチ, YIELD 等) なので プリミティブ扱いしない。
  const uint32_t imm_num = (uint32_t)(*(uint16_t *)(pc - 2) & 0xff);
#endif
  // 発行元の「種別」で経路が決まる (svc 番号では決めない — 冒頭の不変条件)。
  // ★経路は「発行元オブジェクトの種別」だけで決まる。ハンドラとして走っているか
  // (in_handler) は**一切考慮しない**。
  // 【2026-08-13 削除】以前はここに `|| (in_handler && to_kernel_primitive)` があり、
  // 委譲サブハンドラ (= ルート表に登録されただけの**一般オブジェクト**) が 200..206/208 を
  // 生で叩けた。これは「svc を捌く役を任される」が「カーネル特権を得る」と同義になる
  // セキュリティポリシーのバイパスで、とくに SET_SVC_HANDLER(202) は系全体の svc
  // ハンドラを自分に差し替えられる = 全オブジェクトの syscall 乗っ取りだった。
  // この昇格は「ハンドラが戻るための METHOD_EXIT を自分で叩けるように」= 戻り口の
  // 自己参照問題を特権で殴るために入っていたが、**巻き戻す層数を明示的に渡す API**
  // (METHOD_EXIT の arg1 と、arg3 のネスト数申告による検算) がある今は不要。
  // 委譲サブハンドラは普通の一般オブジェクトとしてトランポリンを経由し、
  // delegate_exit_method (obj_api EXIT_METHOD = 2 枚巻き戻し) で戻る。
  if (shizu::object_table[current_thread.object_id].state ==
      shizu::object_t::state_t::KERNEL_OBJECT) {
#if SHIZU_SVC_DELEGATION
    shizu::svc_trace_record(imm_num, current_thread.object_id,
                            context->in_handler, false, false, pc);
#endif

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
      // 原因の特定が困難になる。従来はここで panic 停止していたが、affinity で
      // オブジェクトのコアを動かすと「スレッド番号昇順の初回実行順」による export
      // 順の暗黙保証が崩れる (典型例: 先に走ったオブジェクトが、まだ main が走って
      // いないオブジェクトのメソッドを呼ぶ)。pc は張り替えず call_error を r0 で
      // 呼び出し元へ返し、呼び出し元が yield → 再試行できるようにする。
      {
        shizu::call_error ce = shizu::call_error::OK;
        if (arg0 >= 128 || arg1 >= 128 ||
            shizu::object_table[arg0].state ==
                shizu::object_t::state_t::UNINITIALIZED)
          ce = shizu::call_error::BAD_OBJECT;
        else if (shizu::object_table[arg0].method_table[arg1] == nullptr)
          ce = shizu::call_error::UNDECLARED_METHOD;
        if (ce != shizu::call_error::OK) {
          stack_frame->r0 = (uint32_t)ce; // この svc をその場でエラー復帰させる
          stack_frame->r1 = 0;
          break;
        }
      }
      // NOTE(FPU): stack_frame は 8 語の basic exception_frame_t を*その場で*
      // コピーする。拡張(FP)フレームでもこの構造体はハードウェアスタック上の
      // 元の位置に居続けなければならない (フレームを別アドレスへ再配置しない)。
      // FP 拡張分 (S0-S15/FPSCR) は PSP 上に残り、call_stack.context.sp が
      // 指す位置は不変なので、ここは basic 部のみのコピーで正しい。
      // 退避先はスレッドスタック。積めない (PSPLIM 手前) ときは panic せず
      // call_error で返す — 呼び出し元が対処できる (無音ロックアップにしない)。
      if (!call_frame_push(current_thread, context, &stack_frame,
                           current_thread.object_id)) {
        stack_frame->r0 = (uint32_t)shizu::call_error::NO_STACK;
        stack_frame->r1 = 0;
        break;
      }
      // printf("pc: %lx\n", stack_frame->pc);
      stack_frame->pc = (uint32_t)shizu::object_table[arg0].method_table[arg1];
      stack_frame->lr = (uint32_t)::trap;
      stack_frame->r0 = current_thread.object_id;
      stack_frame->r1 = current_thread.thread_id;
      stack_frame->r2 = arg2;
      stack_frame->r3 = arg3;
      stack_frame->r12 = r12;
      current_thread.object_id = arg0;
      // MPU Step1 プロトタイプ: 呼び先オブジェクトの特権属性へ CONTROL を張り替える
      // (この svc は例外ハンドラ内 = Handler mode なので MSR CONTROL は無条件で許可
      // される。実際に効くのは次の例外復帰から)。呼び出し元の値は上で push した
      // call_stack スナップショット (= 張り替え前の *context) に残っているので、
      // METHOD_EXIT 側は明示コード無しでスナップショット復元だけで元に戻る。
      context->control = shizu::control_for_object(arg0);
#if SHIZU_SVC_DELEGATION
      // ★ハンドラ状態は METHOD_CALL では伝播させない。伝播させると「ハンドラから
      // 呼ばれた普通のメソッド」まで暗黙に特権化してしまう (この機構の一番鋭い刃)。
      context->in_handler = 0;
#endif
      // printf("pc: %lx\n", stack_frame->pc);
      break;
    }
    case shizu::kernel_object_svc_num::METHOD_EXIT: {
      // call_stack を (arg1+1) 段 pop して元のフレーム/オブジェクト ID を復元する。
      // 復元したフレームの r1 に arg0 (メソッド戻り値)、r0 に arg2 (エラーコード、
      // 0 = 成功 — 既存呼び出し元は arg2=0 なのでワイヤ互換) を載せて返す。
      //
      // ★段数の検算 (arg3 = 発行側が申告する「今のネスト数」)。落とす枚数は発行側の
      //   自由 (任意段数) だが、発行側が想定している呼び出し文脈と実際がズレていたら
      //   巻き戻してはいけない — ズレたまま落とすと「無関係な祖先が偽の戻り値で
      //   再開する」という最悪の壊れ方をする (無音ロックアップの正体)。カーネルは
      //   実際の深さと突き合わせ、合わなければ 1 段も落とさずエラーで返す。
      //   発行側 (exit_method / kernel_obj_svc_handler) も自分が持つ深さを申告する
      //   ことで同じ検算に参加している = 両側チェック。
      const uint32_t depth_now = current_thread.call_stack.depth;
      const uint32_t pops = arg1 + 1; // arg1 = 追加段数
      if (depth_now == 0) {
        // 戻り先が 1 枚も無い (スレッドの main が return した等)。**panic しない** —
        // 発行元の例外フレームは生きているのでエラーを返せる。系全体を落とすのは
        // 「超過は超過した者にだけ当たる」という資源管理の方針に反する。
        stack_frame->r0 = (uint32_t)shizu::exit_error::BAD_COUNT;
        stack_frame->r1 = 0;
        break;
      }
      if (arg3 != 0 && arg3 != depth_now) {
        stack_frame->r0 = (uint32_t)shizu::exit_error::DEPTH_MISMATCH;
        stack_frame->r1 = depth_now; // 実際の深さを返す (発行側の自己診断用)
        break;
      }
      if (pops == 0 || pops > depth_now) {
        stack_frame->r0 = (uint32_t)shizu::exit_error::BAD_COUNT;
        stack_frame->r1 = depth_now;
        break;
      }
      // NOTE(FPU): 元の例外フレームは退避域の中に**元の位置のまま**生きているので
      // 書き戻しも再配置も不要 (call_frame_pop は ctx->sp をそこへ向けるだけ)。
      for (uint32_t i = 0; i < pops; i++)
        call_frame_pop(current_thread, context, &stack_frame);
      stack_frame->r0 = arg2;
      stack_frame->r1 = arg0;

      break;
    }
    case shizu::kernel_object_svc_num::SET_SVC_HANDLER: {
      shizu::cpu_manager::svc_handler_info.entry_point = (shizu::method_t)arg0;
      shizu::cpu_manager::svc_handler_info.handling_object_id =
          shizu::thread_table[shizu::cpu_manager::current_thread_id[get_core_num()]]
              .object_id;
      break;
    }

#if SHIZU_SVC_DELEGATION
    case shizu::kernel_object_svc_num::REDISPATCH_SVC: {
      // arg0 = サブハンドラ entry, arg1 = 担当オブジェクト。
      // 今の syscall を「そのオブジェクトのハンドラ」として実行し直す。カーネルは
      // svc 番号を見ない — 誰に回すかは発行元 (カーネルオブジェクトのハンドラ) の判断。
      // 現フレームを積み、pc/object を張り替え、r4/r5/r6 に**元の呼び出し情報**を載せる
      // (kernel_obj_svc_handler と同じ ABI)。サブハンドラは in_handler=1 で走るので
      // METHOD_EXIT を直接叩け、戻りは 1 枚 pop で足りる。
      if (arg1 >= 128 || arg0 == 0 ||
          shizu::object_table[arg1].state ==
              shizu::object_t::state_t::UNINITIALIZED) {
        stack_frame->r0 = 1; // 対象不正 — 呼び出し元 (ハンドラ) がエラーを返せる
        break;
      }
      if (current_thread.call_stack.empty()) {
        // ハンドラ活性化はトランポリンが 1 枚積んだ上で走るので、ここが空なのは
        // 「ハンドラでない誰か」が REDISPATCH を撃った = 使い方の誤り。
        stack_frame->r0 = 3;
        break;
      }
      const uint32_t inv_num = context->inv_svc_num;
      const uint32_t inv_obj = context->inv_caller_obj;
      const uint32_t inv_thr = context->inv_caller_thread;
      // ★元の syscall の引数 (r0..r3) を回収する。**この svc の引数ではない**。
      // 今の stack_frame->r0..r3 は REDISPATCH 自身の引数 (entry / 担当 obj) に
      // 上書きされているので、そのまま飛ばすとサブハンドラは元の引数の代わりに
      // 担当オブジェクト ID を受け取る。トランポリンは r0..r3 を書き換えないため、
      // 直前に積まれた発行元フレーム (= 今の top) に元の引数が生きている。
      // 【実測 2026-08-13】これが無いと最小プローブが arg=35 のはずが arg=27
      // (= 担当 obj の ID) を受け取り、ネストでは「減らない引数」で終了条件に
      // 到達せず**無限相互再帰** → call_stack とスレッドスタックを食い潰して
      // IRQ ごと死ぬ無言ロックアップになっていた。
      const shizu::exception_frame_t *orig =
          call_frame_saved_frame(current_thread.call_stack);
      const uint32_t orig_r0 = orig->r0, orig_r1 = orig->r1,
                     orig_r2 = orig->r2, orig_r3 = orig->r3;
      if (!call_frame_push(current_thread, context, &stack_frame,
                           current_thread.object_id)) {
        stack_frame->r0 = 2; // スタック不足。panic でなくエラーで返す
        break;
      }
      stack_frame->r0 = orig_r0;
      stack_frame->r1 = orig_r1;
      stack_frame->r2 = orig_r2;
      stack_frame->r3 = orig_r3;
      stack_frame->pc = arg0;
      // ★サブハンドラは一般オブジェクトとして走るので、戻り口も一般オブジェクト用。
      stack_frame->lr =
          (uint32_t)shizu::FOR_KERNEL_OBJECT::delegate_exit_method;
      current_thread.object_id = arg1;
      context->control = shizu::control_for_object(arg1);
      context->in_handler = 1; // サブハンドラも「ハンドラとして」走る
      context->inv_svc_num = inv_num;
      context->inv_caller_obj = inv_obj;
      context->inv_caller_thread = inv_thr;
      current_thread.context->r4 = inv_num;
      current_thread.context->r5 = inv_obj;
      current_thread.context->r6 = inv_thr;
      // サブハンドラにも「今のネスト数」を打つ (トランポリンと同じ規約)。委譲で
      // 何段挟まっても、各層は自分が入った時点の深さを申告して巻き戻せる。
      current_thread.context->r7 = current_thread.call_stack.depth;
      break;
    }
#endif
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
      // ★ここに来るのは**信頼された発行元** (カーネルオブジェクト、または信頼活性化)
      // だけ。存在しないプリミティブ番号を撃つのは**カーネル自身の不変条件の破れ**
      // なので panic してよい (§14: panic はカーネル自身が壊れた場合に限る)。
      // 【2026-08-17】ここが `break` で**黙って捨てていた**ため、カーネルオブジェクト
      // から撃った obj_api 番号 (24 = SET_OBJECT_UNPRIVILEGED) が無音で消え、
      // 「非特権で動いた」と誤認する計測事故を起こした。無音の握り潰しは禁止。
      panic("unknown kernel primitive svc %lu\n obj: %lu\n thread: %lu\n pc: %08lx\n"
            " (obj_api の番号をカーネルオブジェクトから撃っていないか?)\n",
            (unsigned long)svc_num, (unsigned long)current_thread.object_id,
            (unsigned long)current_thread.thread_id, (unsigned long)pc);
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
#if SHIZU_SVC_DELEGATION
    shizu::svc_trace_record(imm_num, caller_obj_num, context->in_handler,
                            false, true, pc);
#endif

    // 既定の宛先 = 従来の単一ハンドラ (カーネルオブジェクト)。
    shizu::method_t handler_entry =
        (shizu::method_t)shizu::cpu_manager::svc_handler_info.entry_point;
    uint32_t handler_obj = shizu::cpu_manager::svc_handler_info.handling_object_id;
    if (handler_entry == nullptr) {
      panic("no svc handler for num %lu\n caller obj: %lu\n thread: %lu\n",
            (unsigned long)svc_num, (unsigned long)caller_obj_num,
            (unsigned long)caller_thread_id);
    }
    // 積めない (PSPLIM 手前) ときは panic せず、フレームを積まずに r0 へエラーを
    // 返して復帰する。一般オブジェクトの再帰でカーネルを落とせないようにする。
    if (!call_frame_push(current_thread, context, &stack_frame,
                         current_thread.object_id)) {
      stack_frame->r0 = (uint32_t)shizu::call_error::NO_STACK;
      stack_frame->r1 = 0;
      return; // トランポリンは switch の外なので break ではなく return
    }
    stack_frame->pc = (uint32_t)handler_entry;
    stack_frame->lr = (uint32_t)shizu::FOR_KERNEL_OBJECT::exit_method;
    current_thread.object_id = handler_obj;
    // MPU Step1: ハンドラ所有オブジェクト (通常カーネルオブジェクト=特権) の値へ。
    // METHOD_CALL 分岐と同じ規約 — push は上で完了済みなので、戻り側 (EXIT_METHOD →
    // METHOD_EXIT) は call_stack のスナップショット復元だけで元の呼び出し元の
    // 特権に戻る (明示コード不要)。
    context->control = shizu::control_for_object(handler_obj);
    current_thread.context->r4 = (uint32_t)svc_num;
    current_thread.context->r5 = caller_obj_num;
    current_thread.context->r6 = caller_thread_id;
    // ★「今のネスト数」を打つ: この活性化が走り出す時点の call_stack 深さ。ハンドラは
    // これを METHOD_EXIT の arg3 に載せ返すことで、落とす段数がカーネルの持つ実際の
    // 深さと突き合わされる (段数の申告を鵜呑みにしない = 両側チェック)。r7 なのは
    // ハンドラ shim が既に {r4-r7} を積んでおり、第 7 引数としてそのまま届くため。
    current_thread.context->r7 = current_thread.call_stack.depth;
#if SHIZU_SVC_DELEGATION
    // このハンドラ活性化は「svc ハンドラとして」走る (上の判定が同等に扱う)。
    // 呼び出し情報も記録しておき、委譲 (REDISPATCH_SVC) がサブハンドラへ同じ材料を
    // 渡せるようにする。どちらも context_t なので call_stack の復元で自動的に戻る。
    context->in_handler = 1;
    context->inv_svc_num = (uint32_t)svc_num;
    context->inv_caller_obj = caller_obj_num;
    context->inv_caller_thread = caller_thread_id;
#endif
  }
}