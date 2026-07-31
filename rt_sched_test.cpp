// ===========================================================================
//  rt_sched_test — RT スケジューラの「締切ジッタ under 敵対負荷」実地検証
// ===========================================================================
//  スループット測定 (host の B ブラスト) は blast 中にセンサを止めるので競合ゼロの
//  帯域しか見ておらず、RT スケジューラの本業 (BLE/センサ/制御を締切を割らずに捌く)
//  を検証していない。こちらは**欠けているセンサ負荷をソフトで合成**し、締切の裾を
//  device 側だけで測る (センサ実機もホスト接続も不要):
//
//    ・victim ×N : 代表レート (1kHz/200Hz/100Hz…) の周期タスク。絶対グリッドで起き、
//                  「起床が締切から何 µs 遅れたか (late)」を毎周期記録し、少量の CPU
//                  バーンで仕事を模擬する。late の最大 = スケジューラの応答ジッタ。
//    ・hog ×M    : yield しない無限ループ。BLE と同じ core1 に既定 budget(3ms) で置く。
//                  凍結ウォッチドッグが 3ms ごとに取り上げないと victim が餓死する的。
//    ・reporter  : core0。3 秒待って arm し、2s ごとに victim ジッタ表と hog 生存を印字。
//
//  【合格の読み方】
//    ・core1 victim: max_late が**ホグ budget(3ms)程度で頭打ち**、かつ ticks が期待値
//      付近を維持 (平均レートを保って catch-up している)。period < budget の 1kHz は
//      毎締切は守れない (物理限界: 3ms ホグ > 1ms 周期) が、overrun は有界で rate は維持。
//      period > budget の 100Hz は max_late << period でほぼ全締切を守るはず。
//    ・core0 victim: ホグが居ないコアなので **near-zero ジッタ** = スケジューラがコア間で
//      分離できている (core1 のホグが core0 を汚さない) 証拠。
//    ・hog: reporter が ADVANCING を出し続ける (ホグも餓死せず 3ms スライスをもらえ、
//      系が凍結していない)。max_late が青天井 / ticks 崩壊 / STALLED は RT 失敗。
//
//  RT 応答の粒度は grant budget (SHIZU_DEFAULT_GRANT_BUDGET_US=3ms) そのもの。この
//  試験はその粒度を可視化する — 細かくしたいなら budget を下げる (プリエンプト増と交換)。
//
//  併用: BLE を残したビルドで本試験を回すと、host の ping/B ブラストは victim+hog が
//  走ったまま測られる (blast はセンサを止めるだけで本試験の合成負荷は止めない) ので、
//  「競合下の BLE スループット/RTT」も同時に取れる — 前述の穴がこれで埋まる。
//  注意: センサ非接続だと SENSOR_IO は init 失敗で 20Hz へ退避し概ね sleep するので、
//  core1 に小さなノイズを足すが有界。厳密測定が要るなら別途センサ経路を外すこと。
//
//  SHIZU_RT_SCHED_TEST で有効化 (既定 OFF)。
// ===========================================================================
#include <cstdint>
#include <cstdio>
#include <kernel.hpp>
#include <obj_api.hpp>
#include <object_id.hpp>
#include <pico/stdlib.h>

namespace shizu {
namespace {

// ---- 合成する周期負荷 (センサの代わり) --------------------------------------
struct victim_cfg {
  uint32_t period_us; // 絶対グリッド周期
  uint32_t work_us;   // 1 周期あたりの CPU バーン (fusion 等の仕事を模擬。budget 未満)
  uint32_t affinity;  // AFFINITY_CORE0 / CORE1
  const char *name;
};
// core1 (BLE と同居) に budget を跨ぐ 3 レート + core0 に対照 1 本。
constexpr victim_cfg VICTIMS[] = {
    {1000, 150, AFFINITY_CORE1, "1kHz-c1"},   // period < budget: 毎締切は不可(物理)
    {5000, 200, AFFINITY_CORE1, "200Hz-c1"},  // period ~ budget 境界
    {10000, 300, AFFINITY_CORE1, "100Hz-c1"}, // period > budget: ほぼ守れるはず
    {40000, 400, AFFINITY_CORE0, "25Hz-c0"},  // 対照 (ホグ無しコア = near-zero)
};
constexpr uint32_t NUM_VICTIMS = sizeof(VICTIMS) / sizeof(VICTIMS[0]);
constexpr uint32_t HOG_COUNT = 1; // core1 の never-yield ホグ数 (増やして飽和度を上げる)

struct victim_stat {
  volatile uint32_t ticks;          // 完了した周期数 (累積。rate 維持の確認)
  volatile uint32_t overruns;       // late > period だった回数 (累積)
  volatile uint32_t max_late_us;    // 起床遅れの全期間最大 (= 応答ジッタ)
  volatile uint32_t win_max_late_us;// 窓内最大 (reporter が読んで 0 に戻す)
};
victim_stat g_vstat[NUM_VICTIMS];
volatile uint32_t g_hog[HOG_COUNT];
volatile bool g_armed = false; // reporter が初期化待ちの後に立てる

// ---- victim: 締切ジッタを自己計測する周期タスク ------------------------------
void victim(uint32_t idx) {
  const victim_cfg &c = VICTIMS[idx];
  victim_stat &s = g_vstat[idx];
  while (!g_armed)
    obj_api::yield(); // arm 前は普通に譲る (boot/init を邪魔しない)

  uint64_t next = time_us_64();
  while (true) {
    next += c.period_us;
    obj_api::yield_until_us(next); // 締切まで sleep + 最後だけ spin (±数µs)
    uint64_t now = time_us_64();
    int64_t late = (int64_t)(now - next); // 起床がグリッドからどれだけ遅れたか
    if (late < 0)
      late = 0;
    uint32_t lu = (uint32_t)late;
    s.ticks = s.ticks + 1;
    if (lu > s.max_late_us)
      s.max_late_us = lu;
    if (lu > s.win_max_late_us)
      s.win_max_late_us = lu;
    if (late > (int64_t)c.period_us)
      s.overruns = s.overruns + 1;
    // 1 周期以上遅れたらグリッドを現在へ寄せ直す (catch-up の連射を抑える。次周回で
    // 必ず yield_until_us を通るので CPU を独占しない — overrun-resync-must-still-yield)。
    if (now > next + c.period_us)
      next = now;
    // 仕事を模擬 (この間は CPU を握る。work_us は budget 3ms 未満に収めてある)。
    uint64_t w0 = time_us_64();
    while (time_us_64() - w0 < c.work_us)
      tight_loop_contents();
  }
}

// ---- hog: 締切を持たない CPU 独占者 (敵役)。arm 後は yield しない。-------------
void hog(uint32_t idx) {
  while (!g_armed)
    obj_api::yield();
  while (true)
    g_hog[idx] = g_hog[idx] + 1; // grantee 規律: malloc/printf は呼ばない
}

// ---- reporter: arm と結果印字 (core0。grant の外なので printf 可) -------------
void reporter(uint32_t) {
  obj_api::sleep_us(3000000); // 他オブジェクト (BLE/センサ) の初期化完了を待つ
  g_armed = true;
  printf("[RTTEST] armed: %lu victims + %lu hog(s) on core1, hog budget=%dus. "
         "PASS: core1 max_late bounded ~budget & ticks~exp; core0 near-zero; "
         "hog ADVANCING.\n",
         (unsigned long)NUM_VICTIMS, (unsigned long)HOG_COUNT,
         SHIZU_DEFAULT_GRANT_BUDGET_US);

  uint32_t prev_ticks[NUM_VICTIMS] = {};
  uint32_t prev_hog[HOG_COUNT] = {};
  constexpr uint32_t WIN_US = 2000000;
  while (true) {
    obj_api::sleep_us(WIN_US);
    for (uint32_t i = 0; i < NUM_VICTIMS; ++i) {
      victim_stat &s = g_vstat[i];
      uint32_t t = s.ticks;
      uint32_t d = t - prev_ticks[i];
      prev_ticks[i] = t;
      uint32_t exp = WIN_US / VICTIMS[i].period_us;
      uint32_t wl = s.win_max_late_us;
      s.win_max_late_us = 0; // 窓リセット
      printf("[RTTEST] %-9s late(win)=%luus late(max)=%luus overrun=%lu "
             "ticks=%lu/%lu\n",
             VICTIMS[i].name, (unsigned long)wl, (unsigned long)s.max_late_us,
             (unsigned long)s.overruns, (unsigned long)d, (unsigned long)exp);
    }
    for (uint32_t h = 0; h < HOG_COUNT; ++h) {
      uint32_t cc = g_hog[h];
      uint32_t dh = cc - prev_hog[h];
      prev_hog[h] = cc;
      printf("[RTTEST] hog%lu +%lu/2s %s\n", (unsigned long)h,
             (unsigned long)dh, dh ? "ADVANCING" : "**STALLED**");
    }
  }
}

} // namespace

// core0 (kernel_object_main) から呼ぶ。affinity は async_call 直後・kernel_object_main
// が yield する前に立てるので、既定 affinity のまま走り出す窓は無い (smp_stress と同じ)。
void rt_sched_test_launch() {
  FOR_KERNEL_OBJECT::create_object((uint32_t)object_ids::RT_SCHED_TEST);
  const uint32_t obj = (uint32_t)object_ids::RT_SCHED_TEST;
  for (uint32_t i = 0; i < NUM_VICTIMS; ++i) {
    uint32_t tid = FOR_KERNEL_OBJECT::async_call(obj, (method_t)victim, i);
    thread_table[tid].affinity = VICTIMS[i].affinity; // 既定 budget(3ms)のまま
  }
  for (uint32_t h = 0; h < HOG_COUNT; ++h) {
    uint32_t tid = FOR_KERNEL_OBJECT::async_call(obj, (method_t)hog, h);
    thread_table[tid].affinity = AFFINITY_CORE1; // BLE と同じコアで取り合わせる
  }
  FOR_KERNEL_OBJECT::async_call(obj, (method_t)reporter); // reporter は core0 既定
}

} // namespace shizu
