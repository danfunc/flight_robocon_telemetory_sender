// ===========================================================================
//  core1_boot — core1 を Shizuku カーネルとして起動し、センサ I/O を SENSOR_IO
//               スレッド (core1 ピン留め) として走らせる
// ===========================================================================
//  init_core1 が core1 を idle スレッドとして立ち上げ、idle が起動直後の 1 回の
//  yield で SENSOR_IO (affinity=core1) を claim する。SENSOR_IO (= 旧 core1_io の
//  センサ I/O ループ) は以後 yield しないので core1 を専有する。旧ベアメタル core1_io
//  と等価だが、Shizuku のスケジューラ/claim を通って core1 に載る点だけが違う。
//  既存スレッドは全て affinity=core0 なので core1 は触らない (AMP をアフィニティで実現)。
//
//  【要点】
//  ・SENSOR_IO は yield/SVC を呼ばないので、YIELD/claim の共有経路 (method_map /
//    memory_map / stream_table) に一切触れず、core0 と競合しない。stream への push は
//    SVC を通らないライブラリなので core1 から直接叩ける。
//  ・RP2350 は core1 も core0 の RAM ベクタテーブルを共有する (pico_multicore が
//    scb_hw->vtor を core1 へ渡す) ので、SVC ハンドラの再登録は不要 (再登録すると
//    exclusive ハンドラ二重登録で panic する)。FPU (CPACR) だけ保険で立てる。
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

// idle: init_core1 が core1 の初期スレッドとして走らせる。起動直後の 1 回の yield で
// READY な SENSOR_IO (affinity=core1) を claim させる。SENSOR_IO は以後 yield しない
// ので、以降 core1 は SENSOR_IO 専有になる (idle は二度と走らない bootstrap)。
[[noreturn]] void core1_idle_main() {
  while (true) {
    obj_api::yield();
    tight_loop_contents();
  }
}

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
  idle.object_id = (uint32_t)object_ids::SENSOR_IO; // SENSOR_IO の bootstrap スレッド
  idle.thread_id = CORE1_IDLE_TID;
  idle.context = new context_t();
  idle.context->sp = (exception_frame_t *)psp; // 初回 yield で実 PSP に上書きされる種
  idle.context->exc_return = 0xFFFFFFFD;        // 基本フレーム/PSP へ復帰
  idle.call_stack.frames = (method_call_stack_t *)malloc(
      sizeof(method_call_stack_t) * call_stack_t::MAX_DEPTH);
  idle.call_stack.depth = 0;
  idle.affinity = AFFINITY_CORE1;
  idle.state = thread_t::state_t::RUNNING;
  cpu_manager::current_thread_id[1] = CORE1_IDLE_TID; // friend アクセス

  // PSP へ切替え、Thread モードで PSP を使う (SPSEL=1) → idle ループへ。
  uintptr_t CONTROL_MASK = 1u << 1;
  __asm volatile("MSR PSP,%[psp];"
                 "MSR CONTROL,%[ctl];"
                 "isb;"
                 :
                 : [psp] "r"(psp), [ctl] "r"(CONTROL_MASK)
                 : "memory");
  core1_idle_main();
}

} // namespace shizu
