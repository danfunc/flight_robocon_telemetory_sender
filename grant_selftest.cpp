// ===========================================================================
//  grant_selftest — 時限実行権移譲 (run_for / GRANT_CPU) の end-to-end 実動確認
// ===========================================================================
//  GRANT_TEST オブジェクトに driver + worker 群を置き、以下を実機で検証する:
//    A: 期限強制 (EXPIRED)   — yield しない busy スレッドが期限で強制返却される
//    B: 早期復帰 (YIELDED)   — grantee の yield で即 grantor へ戻る
//    C: エラー経路           — NOT_READY / BAD_AFFINITY / DEPTH (8 段ネスト上限)
//    D: ネスト + クランプ    — 内側期限が外側残時間へクランプされ、内→外の順で復帰
//    E: 長期限               — >111ms で SysTick 24bit チャンク継ぎ足し経路を通す
//  worker は grantee として走る間 malloc/printf を呼ばない (v1 の規律)。記録は
//  volatile グローバルへ書き、printf は driver が grant の外で行う。
//  SHIZU_GRANT_SELFTEST で有効化。
// ===========================================================================
#include <cstdint>
#include <cstdio>
#include <kernel.hpp>
#include <obj_api.hpp>
#include <object_id.hpp>
#include <pico/stdlib.h>

namespace shizu {
namespace {

enum phase_t : uint32_t {
  PH_IDLE = 0,
  PH_BUSY,  // A/E: busy_worker が yield せず回る
  PH_YIELD, // B: yield_worker が 500µs 後に yield する
  PH_CLAMP, // D: mid_worker が内側 run_for を発行する
  PH_NEST,  // C: nest_worker 連鎖で DEPTH 上限を踏む
};
volatile uint32_t g_phase = PH_IDLE;

// worker たちの tid (launch 時に確定)
uint32_t g_busy_tid, g_yield_tid, g_affinity_tid, g_mid_tid, g_nest_tid[8];
constexpr uint32_t UNUSED_TID = 120; // どこにも create していないスロット (NOT_READY 用)

volatile uint32_t g_busy_counter = 0; // busy_worker の生存カウンタ (実走確認)

// D (クランプ) の記録: mid が内側 run_for の結果と「driver より先に復帰したか」を残す
volatile uint32_t g_mid_err = 0xFFFF, g_mid_reason = 0xFFFF, g_mid_elapsed = 0;
volatile bool g_mid_done = false, g_driver_resumed = false,
              g_mid_before_driver = false;

// C (DEPTH) の記録: 各段の run_for 結果
volatile uint32_t g_nest_err[8];
volatile bool g_nest_done[8];

// A/D/E の的: phase 中は yield せず回り続ける (期限強制でしか戻れない)。
// grantee 規律: ロック持ちライブラリ (malloc/printf) は呼ばない。
void busy_worker(uint32_t) {
  while (true) {
    if (g_phase == PH_BUSY || g_phase == PH_CLAMP) {
      g_busy_counter = g_busy_counter + 1;
    } else {
      obj_api::yield();
    }
  }
}

// B の的: phase 中は 500µs 走ってから yield → 早期復帰 (YIELDED) を踏む。
void yield_worker(uint32_t) {
  while (true) {
    if (g_phase == PH_YIELD) {
      const uint64_t t0 = time_us_64();
      while (time_us_64() - t0 < 500)
        tight_loop_contents();
      obj_api::yield(); // grant 中の yield = grantor への早期復帰
    } else {
      obj_api::yield();
    }
  }
}

// BAD_AFFINITY の的: affinity=CORE1 だが core1 は SENSOR_IO 専有なので永久に READY の
// まま走らない。core0 の driver から run_for すると affinity 検査で弾かれる。
void affinity_worker(uint32_t) {
  while (true)
    obj_api::yield();
}

// D の中間段: 外側残 (~2ms) より長い 10ms を要求 → クランプされ ~2ms で EXPIRED。
// 復帰順 (mid が driver より先) も記録する。
void mid_worker(uint32_t) {
  while (true) {
    if (g_phase == PH_CLAMP && !g_mid_done) {
      g_mid_done = true;
      const uint64_t t0 = time_us_64();
      obj_api::run_for_result_t r = obj_api::run_for(g_busy_tid, 10000);
      g_mid_elapsed = (uint32_t)(time_us_64() - t0);
      g_mid_err = (uint32_t)r.error;
      g_mid_reason = (uint32_t)r.reason;
      g_mid_before_driver = !g_driver_resumed;
      obj_api::yield();
    } else {
      obj_api::yield();
    }
  }
}

// C (DEPTH) の連鎖段: idx<7 は次段へ run_for、idx==7 は depth=8 で発行して DEPTH を
// 踏む。各段は結果を記録して yield → 早期復帰 (YIELDED) で 1 段ずつ巻き戻る。
void nest_worker(uint32_t idx) {
  while (true) {
    if (g_phase == PH_NEST && !g_nest_done[idx]) {
      g_nest_done[idx] = true;
      obj_api::run_for_result_t r =
          (idx < 7) ? obj_api::run_for(g_nest_tid[idx + 1], 50000)
                    : obj_api::run_for(g_busy_tid, 50000); // depth 上限 → DEPTH
      g_nest_err[idx] = (uint32_t)r.error;
      obj_api::yield();
    } else {
      obj_api::yield();
    }
  }
}

void report(const char *name, bool pass, uint64_t elapsed_us, uint32_t err,
            uint32_t reason) {
  printf("[GRANTTEST] %s: %s (elapsed=%lluus err=%lu reason=%lu)\n", name,
         pass ? "PASS" : "FAIL", (unsigned long long)elapsed_us,
         (unsigned long)err, (unsigned long)reason);
}

// テストの司令塔。grant の外でだけ printf する。
void driver(uint32_t) {
  obj_api::sleep_us(3000000); // 他オブジェクト (BLE 等) の初期化が済むまで待つ
  printf("[GRANTTEST] start (busy=%lu yield=%lu mid=%lu)\n",
         (unsigned long)g_busy_tid, (unsigned long)g_yield_tid,
         (unsigned long)g_mid_tid);
  bool all = true;

  // ---- A: 期限強制 (EXPIRED)。busy は yield しないので PendSV でしか戻らない --
  const uint32_t count_before = g_busy_counter;
  g_phase = PH_BUSY;
  uint64_t t0 = time_us_64();
  obj_api::run_for_result_t r = obj_api::run_for(g_busy_tid, 3000);
  uint64_t elapsed = time_us_64() - t0;
  g_phase = PH_IDLE;
  const bool ran = (g_busy_counter != count_before);
  bool pass = r.is_ok() && r.reason == grant_end::EXPIRED && ran &&
              elapsed >= 2900 && elapsed <= 5000;
  report("A expire", pass, elapsed, (uint32_t)r.error, (uint32_t)r.reason);
  all = all && pass;

  // ---- B: 早期復帰 (YIELDED)。500µs で grantee が yield する ------------------
  g_phase = PH_YIELD;
  t0 = time_us_64();
  r = obj_api::run_for(g_yield_tid, 5000);
  elapsed = time_us_64() - t0;
  g_phase = PH_IDLE;
  pass = r.is_ok() && r.reason == grant_end::YIELDED && elapsed >= 450 &&
         elapsed <= 2000;
  report("B yield", pass, elapsed, (uint32_t)r.error, (uint32_t)r.reason);
  all = all && pass;

  // ---- C1: NOT_READY (未生成スロット) / C2: BAD_AFFINITY (core1 ピン) ---------
  r = obj_api::run_for(UNUSED_TID, 1000);
  pass = (r.error == grant_error::NOT_READY);
  report("C1 not_ready", pass, 0, (uint32_t)r.error, (uint32_t)r.reason);
  all = all && pass;
  r = obj_api::run_for(g_affinity_tid, 1000);
  pass = (r.error == grant_error::BAD_AFFINITY);
  report("C2 bad_affinity", pass, 0, (uint32_t)r.error, (uint32_t)r.reason);
  all = all && pass;

  // ---- C3: DEPTH。8 段ネスト連鎖の最深段が DEPTH を踏み、YIELDED で巻き戻る ---
  for (int i = 0; i < 8; ++i) {
    g_nest_err[i] = 0xFFFF;
    g_nest_done[i] = false;
  }
  g_phase = PH_NEST;
  t0 = time_us_64();
  r = obj_api::run_for(g_nest_tid[0], 50000);
  elapsed = time_us_64() - t0;
  g_phase = PH_IDLE;
  pass = r.is_ok() && r.reason == grant_end::YIELDED;
  for (int i = 0; i < 7; ++i) // 中間段は OK (次段へ移譲できた)
    pass = pass && (g_nest_err[i] == (uint32_t)grant_error::OK);
  pass = pass && (g_nest_err[7] == (uint32_t)grant_error::DEPTH); // 最深段
  report("C3 depth", pass, elapsed, g_nest_err[7], (uint32_t)r.reason);
  all = all && pass;

  // ---- D: ネスト + クランプ。mid の内側 10ms 要求が外側残 ~2ms に切られる -----
  g_mid_done = false;
  g_driver_resumed = false;
  g_phase = PH_CLAMP;
  t0 = time_us_64();
  r = obj_api::run_for(g_mid_tid, 2000);
  elapsed = time_us_64() - t0;
  g_driver_resumed = true;
  g_phase = PH_IDLE;
  // 内側: クランプされ ~2ms で EXPIRED。外側: 直後に期限切れ (EXPIRED)、mid が
  // yield に先に到達していれば YIELDED もあり得る。復帰順 (mid → driver) は必須。
  pass = r.is_ok() && g_mid_err == (uint32_t)grant_error::OK &&
         g_mid_reason == (uint32_t)grant_end::EXPIRED && g_mid_before_driver &&
         g_mid_elapsed >= 1500 && g_mid_elapsed <= 3500 && elapsed >= 1900 &&
         elapsed <= 5000;
  report("D clamp", pass, elapsed, g_mid_err, g_mid_reason);
  printf("[GRANTTEST]   (inner elapsed=%luus mid_first=%d outer_reason=%lu)\n",
         (unsigned long)g_mid_elapsed, g_mid_before_driver ? 1 : 0,
         (unsigned long)r.reason);
  all = all && pass;

  // ---- E: 長期限 (>111ms)。SysTick 24bit チャンク継ぎ足し経路 -----------------
  g_phase = PH_BUSY;
  t0 = time_us_64();
  r = obj_api::run_for(g_busy_tid, 200000);
  elapsed = time_us_64() - t0;
  g_phase = PH_IDLE;
  pass = r.is_ok() && r.reason == grant_end::EXPIRED && elapsed >= 199000 &&
         elapsed <= 220000;
  report("E long", pass, elapsed, (uint32_t)r.error, (uint32_t)r.reason);
  all = all && pass;

  printf("[GRANTTEST] ==== %s ====\n", all ? "ALL PASS" : "SOME FAILED");
  while (true)
    obj_api::sleep_us(1000000);
}

} // namespace

// core0 (kernel_object_main) から呼ぶ。GRANT_TEST オブジェクト + driver/worker 群を
// 作る (driver 以外は全て「phase 外では即 yield」の待機ループなので通常運転を乱さない)。
void grant_selftest_launch() {
  FOR_KERNEL_OBJECT::create_object((uint32_t)object_ids::GRANT_TEST);
  const uint32_t obj = (uint32_t)object_ids::GRANT_TEST;
  g_busy_tid = FOR_KERNEL_OBJECT::async_call(obj, (method_t)busy_worker);
  g_yield_tid = FOR_KERNEL_OBJECT::async_call(obj, (method_t)yield_worker);
  g_affinity_tid = FOR_KERNEL_OBJECT::async_call(obj, (method_t)affinity_worker);
  thread_table[g_affinity_tid].affinity = AFFINITY_CORE1; // BAD_AFFINITY の的
  g_mid_tid = FOR_KERNEL_OBJECT::async_call(obj, (method_t)mid_worker);
  for (uint32_t i = 0; i < 8; ++i)
    g_nest_tid[i] = FOR_KERNEL_OBJECT::async_call(obj, (method_t)nest_worker, i);
  FOR_KERNEL_OBJECT::async_call(obj, (method_t)driver);
}

} // namespace shizu
