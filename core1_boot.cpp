// ===========================================================================
//  core1_boot — core1 を Shizuku カーネルとして起動し、センサ I/O を SENSOR_IO
//               スレッド (core1 ピン留め) として走らせる
// ===========================================================================
//  init_core1 が core1 を idle スレッド (カーネルオブジェクトのスレッド) として
//  立ち上げ、core0 の thread0 と同じ共通 scheduler_idle_loop を回す。SENSOR_IO
//  (affinity=core1) は仕事の無い間、次のセンサ締切まで sleep してこのスケジューラへ
//  CPU を返す協調スレッド (旧: ベアメタル専有ループ)。センサ優先はそのままに、
//  余剰時間を将来 core1 affinity の他スレッドへ貸せる形 (AMP → 優先的占有)。
//
//  【要点】
//  ・idle は必ず object 0 (カーネルオブジェクト) のスレッドにする。scheduler_idle_loop
//    は SWITCH_THREAD/GRANT_CPU を生 svc で発行するため、一般オブジェクトのスレッド
//    から呼ぶとトランポリンへ誤ルーティングされて panic する (init_core1 内コメント)。
//  ・SENSOR_IO の sleep/yield は SVC トランポリン経由 = カーネル SVC 経路を core1 も
//    通る。SVC 直接処理 (SWITCH/SLEEP/claim) は per-core 状態 + CAS で 2 コア安全。
//    method_map/memory_map (std::map) に触る API は core1 からは呼ばないこと (フラット
//    表化までの暫定規律)。
//  ・RP2350 は core1 も core0 の RAM ベクタテーブルを共有する (pico_multicore が
//    scb_hw->vtor を core1 へ渡す) ので、SVC ハンドラの再登録は不要 (再登録すると
//    exclusive ハンドラ二重登録で panic する)。FPU (CPACR) と例外優先度 (SHPR, banked)
//    は per-core なので init_core1 で設定する。
// ===========================================================================
#include <core_ring.hpp> // sensor_io_main (SENSOR_IO スレッド entry)
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <kernel.hpp>
#include <obj_api.hpp>
#include <object_id.hpp>
#include <pico/multicore.h>
#include <pico/stdlib.h>

namespace shizu {
namespace {

// idle スレッドは init_core1 が直接 (create_thread を通さず) 仕立てるので tid を高位に
// 予約する。async_call の自動割当 (1 から昇順) と衝突しない範囲 (実働スレッド << 100)。
constexpr uint32_t CORE1_IDLE_TID = 100;

} // namespace

// core0 (kernel_object_main) から呼ぶ。SENSOR_IO オブジェクト + センサ I/O スレッドを
// 作り (create は core1 起動前・yield 前なので core0 が SENSOR_IO を拾う窓は無い)、
// core1 をカーネル入口で起動する。idle が起動直後に SENSOR_IO を claim して core1 上で
// センサ I/O ループを走らせる (= 旧 core1_io と同じ「core1 専有センサ I/O」を Shizuku
// スレッドとして実現)。
void core1_kernel_launch() {
  // SENSOR_IO = create_object + async_call (空き tid 自動確保)。返った tid に affinity を
  // 立てて core1 ピン留めする。ここは thread0 が yield/switch する前 (逐次) なので、
  // affinity を立てる前に core0 が拾う窓は無い。
  FOR_KERNEL_OBJECT::create_object((uint32_t)object_ids::SENSOR_IO);
  uint32_t tid = FOR_KERNEL_OBJECT::async_call((uint32_t)object_ids::SENSOR_IO,
                                               (method_t)sensor_io_main);
  thread_table[tid].affinity = AFFINITY_CORE1;
  // SENSOR_IO は core1 の優先テナント = バトン組 (budget 無制限)。仕事は I2C 読みの
  // バースト (≤~2.4ms) で自発的に sleep へ戻る設計なので期限強制は不要。budget 付きに
  // すると I2C バースト途中の期限切れ/再 host が起きるだけで守るものが無い。
  thread_table[tid].grant_budget_us = 0;
  multicore_launch_core1(init_core1);
}

// core1 のカーネル入口。per-core 初期化 (FPU) + 自分を idle スレッドに仕立てて
// idle ループへ入る (core0 の set_current_context_as_kernel_init と同型)。
[[noreturn]] void init_core1() {
  // FPU (CPACR CP10/CP11 フルアクセス) を core1 でも有効化する保険。SDK の per-core
  // init が立てていれば冪等。core1 スレッドは整数のみだが svc_asm_handler の条件 FP
  // 退避と将来の FP スレッドのために揃えておく。
  *(volatile uint32_t *)0xE000ED88 |= (0xFu << 20);
  __asm volatile("dsb; isb" ::: "memory");

  // 例外優先度 (SVC > SysTick > PendSV)。SHPR は per-core banked なので core1 でも
  // 設定が要る。ハンドラ登録自体は共有ベクタテーブル経由で core0 の分が効いている
  // (再登録は exclusive 二重登録 panic するのでしない — 上記コメント参照)。
  init_exception_priorities();

  void *psp = (void *)(((uint32_t)malloc(4096) + 4096) & ~0xFu);
  thread_t &idle = thread_table[CORE1_IDLE_TID];
  // idle は必ず「カーネルオブジェクト (id=0) のスレッド」にする。scheduler_idle_loop は
  // SWITCH_THREAD/GRANT_CPU を生 svc で発行するため、一般オブジェクトから呼ぶと
  // トランポリンへ誤ルーティングされ、r0(=切替先 tid) が obj_api::svc_num として誤解釈
  // される (実障害: tid2=CREATE_OBJECT → create_object(0) → 「object already
  // initialized」panic → core1 死亡)。core0 の thread0 と同じ条件に揃えるのが本質。
  idle.object_id = (uint32_t)object_ids::KERNEL_OBJECT;
  object_table[(uint32_t)object_ids::KERNEL_OBJECT].thread_table.insert(
      CORE1_IDLE_TID);
  idle.thread_id = CORE1_IDLE_TID;
  idle.context = new context_t();
  idle.context->sp = (exception_frame_t *)psp; // 初回 yield で実 PSP に上書きされる種
  idle.context->exc_return = 0xFFFFFFFD;        // 基本フレーム/PSP へ復帰
  idle.call_stack.frames = (method_call_stack_t *)malloc(
      sizeof(method_call_stack_t) * call_stack_t::MAX_DEPTH);
  idle.call_stack.depth = 0;
  idle.affinity = AFFINITY_CORE1;
  idle.grant_budget_us = 0; // core1 idle もバトン組 (thread0 と同じ理由)
  idle.state = thread_t::state_t::RUNNING;
  cpu_manager::current_thread_id[1] = CORE1_IDLE_TID; // friend アクセス

  // PSP へ切替え、Thread モードで PSP を使う (SPSEL=1) → 共通スケジューラ idle ループへ。
  uintptr_t CONTROL_MASK = 1u << 1;
  __asm volatile("MSR PSP,%[psp];"
                 "MSR CONTROL,%[ctl];"
                 "isb;"
                 :
                 : [psp] "r"(psp), [ctl] "r"(CONTROL_MASK)
                 : "memory");
  // core0 の thread0 と同じ共通実装。初回 sched_pick_next で READY な SENSOR_IO
  // (affinity=core1) をバトンで拾い、SENSOR_IO が idle 時に sleep で戻るたびに回る。
  // 使用率 (cpu_busy_us[1]) もこのループの会計から出る。
  scheduler_idle_loop(CORE1_IDLE_TID);
}

} // namespace shizu
