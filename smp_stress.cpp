// ===========================================================================
//  smp_stress — カーネル SVC 経路の 2 コア同時進入ストレス
// ===========================================================================
//  「SENSOR_IO で動いた」を SMP 安全性の完了判定にしない (Fable レビュー) ための
//  合成負荷。SLEEP/SWITCH (トランポリン + バトン + grant host) の SVC を両コアから
//  同時多発させ、レース/クラッシュを数分単位で炙り出す:
//    ・c0 worker ×2 : core0 ピン、budget 付き (grant host 経路)
//    ・c1 worker ×2 : core1 ピン、budget 付き (core1 の SVC/バトン経路。SENSOR_IO
//                     と同居し、core1 scheduler_idle_loop の混雑も再現)
//    ・mig worker ×1: affinity=ALL (跨コア移動。FP/EXC_RETURN のコア独立性を検査)
//  各 worker は sleep_us(素数 µs) → カウンタ++ を回すだけ。reporter (core0) が 2s
//  ごとに全カウンタを印字する。**全カウンタが前進し続け、panic/freeze が出ない**
//  ことが合格条件 (panic は panic_ring 経由でビーコンに出る)。
//  SHIZU_SMP_STRESS で有効化。
// ===========================================================================
#include <cstdint>
#include <cstdio>
#include <kernel.hpp>
#include <obj_api.hpp>
#include <object_id.hpp>
#include <pico/stdlib.h>

namespace shizu {
namespace {

// カウンタ: [0..1]=core0 ピン, [2..3]=core1 ピン, [4]=移動 (affinity ALL)
volatile uint32_t g_cnt[5];

// sleep 周期 [µs]。素数で位相が揃わないようにし、SVC の衝突タイミングを掃く。
constexpr uint32_t PERIODS[5] = {311, 502, 293, 701, 409};

void stress_worker(uint32_t idx) {
  while (true) {
    obj_api::sleep_us(PERIODS[idx]);
    g_cnt[idx] = g_cnt[idx] + 1;
  }
}

void reporter(uint32_t) {
  uint32_t prev[5] = {};
  while (true) {
    obj_api::sleep_us(2000000);
    printf("[SMPSTRESS] c0=%lu,%lu c1=%lu,%lu mig=%lu %s\n",
           (unsigned long)g_cnt[0], (unsigned long)g_cnt[1],
           (unsigned long)g_cnt[2], (unsigned long)g_cnt[3],
           (unsigned long)g_cnt[4],
           (g_cnt[0] > prev[0] && g_cnt[1] > prev[1] && g_cnt[2] > prev[2] &&
            g_cnt[3] > prev[3] && g_cnt[4] > prev[4])
               ? "ADVANCING"
               : "**STALLED**");
    for (int i = 0; i < 5; ++i)
      prev[i] = g_cnt[i];
  }
}

} // namespace

// core0 (kernel_object_main) から呼ぶ。core1 ピン組は SENSOR_IO 起動後 (= 逐次、
// yield 前) に affinity を立てるので、立てる前に走り出す窓は無い。
void smp_stress_launch() {
  FOR_KERNEL_OBJECT::create_object((uint32_t)object_ids::SMP_STRESS);
  const uint32_t obj = (uint32_t)object_ids::SMP_STRESS;
  // core0 ピン ×2 (既定 affinity=CORE0, budget=3ms)
  FOR_KERNEL_OBJECT::async_call(obj, (method_t)stress_worker, 0);
  FOR_KERNEL_OBJECT::async_call(obj, (method_t)stress_worker, 1);
  // core1 ピン ×2
  uint32_t t2 = FOR_KERNEL_OBJECT::async_call(obj, (method_t)stress_worker, 2);
  uint32_t t3 = FOR_KERNEL_OBJECT::async_call(obj, (method_t)stress_worker, 3);
  thread_table[t2].affinity = AFFINITY_CORE1;
  thread_table[t3].affinity = AFFINITY_CORE1;
  // 跨コア移動 ×1 (どちらのコアの scheduler にも claim される)
  uint32_t t4 = FOR_KERNEL_OBJECT::async_call(obj, (method_t)stress_worker, 4);
  thread_table[t4].affinity = AFFINITY_ALL;
  // reporter (core0)
  FOR_KERNEL_OBJECT::async_call(obj, (method_t)reporter, 0);
}

} // namespace shizu
