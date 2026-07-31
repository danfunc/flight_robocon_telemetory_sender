// ===========================================================================
//  busy_load — 「CPU ホグがスケジューラに取り上げられ、BLE を妨げない」の実地検証
// ===========================================================================
//  grant_selftest は run_for/GRANT_CPU の機構 (期限強制) を単体で確認する。こちらは
//  その機構が**通常運転の BLE と同居して効く**ことを end-to-end で見るための合成負荷:
//    ・busy_worker ×1 : **yield しない無限ループ**を BLE と同じ core1 にピン留めし、
//                       既定 budget (3ms) で走らせる。凍結ウォッチドッグが 3ms ごとに
//                       PendSV で取り上げ、core1 の BLE (budget0=無限バトン) と
//                       SENSOR_IO へ CPU を戻す — この「取り上げ」が実運用条件下でも
//                       効くことが検証対象。budget0 にするとバトンを返さず BLE を
//                       餓死させる (= やってはいけない設定) ので、必ず既定 budget。
//    ・reporter ×1    : core0。3 秒待って (BLE/センサの初期化完了を待つ) からホグを
//                       arm し、以後 2s ごとに生存カウンタを印字する。
//  合格条件は 2 つ、観測点が分かれている:
//    (1) ホグ側     — reporter が "ADVANCING" を出し続ける (= ホグが 3ms スライスを
//                     もらえている & 系が凍結していない)。"STALLED" は凍結か餓死。
//    (2) BLE 側     — BLE_UART が 1s ごとに出す tx 統計の ctrl_lat_max が跳ねない/
//                     ダウンリンク・ブラスト (TELEMETRY 'B') のスループットが素の状態と
//                     同等。こちらは BLE_UART 側の既存計装で見る (本 TU は BLE に依存
//                     しないよう ctrl_lat を参照しない — SHIZU_NO_BLE ビルドでも通る)。
//  SHIZU_BUSY_LOAD で有効化 (既定 OFF)。
// ===========================================================================
#include <cstdint>
#include <cstdio>
#include <kernel.hpp>
#include <obj_api.hpp>
#include <object_id.hpp>
#include <pico/stdlib.h>

namespace shizu {
namespace {

volatile bool g_armed = false;        // reporter が初期化待ちの後に立てる
volatile uint32_t g_busy_counter = 0; // ホグの生存カウンタ (実走 & 非凍結の証拠)

// core1 の CPU ホグ。arm 後は yield せず回り続ける (期限強制でしか戻れない)。
// grantee 規律: ロック持ちライブラリ (malloc/printf) は呼ばない — カウンタ++ のみ。
void busy_worker(uint32_t) {
  while (true) {
    if (g_armed) {
      g_busy_counter = g_busy_counter + 1;
    } else {
      obj_api::yield(); // arm 前は普通に譲る (BLE/センサ初期化を邪魔しない)
    }
  }
}

// core0 の観測係。ホグ arm と生存監視だけ (grant の外なので printf 可)。
void reporter(uint32_t) {
  obj_api::sleep_us(3000000); // 他オブジェクト (BLE/センサ) の初期化完了を待つ
  g_armed = true;
  printf("[BUSYLOAD] armed: core1 never-yield hog @ budget=%dus. Watch BLE_UART "
         "ctrl_lat / blast throughput for interference.\n",
         SHIZU_DEFAULT_GRANT_BUDGET_US);
  uint32_t prev = g_busy_counter;
  while (true) {
    obj_api::sleep_us(2000000);
    uint32_t now = g_busy_counter;
    uint32_t delta = now - prev; // u32 引き算はラップ安全
    prev = now;
    printf("[BUSYLOAD] hog counter=%lu (+%lu/2s) %s\n", (unsigned long)now,
           (unsigned long)delta, delta ? "ADVANCING" : "**STALLED**");
  }
}

} // namespace

// core0 (kernel_object_main) から呼ぶ。ホグは core1 ピン (BLE と同居)、既定 budget。
// core1 ピンの affinity は async_call 直後・kernel_object_main が yield する前に立てる
// ので、既定 affinity(core0) のまま走り出す窓は無い (smp_stress と同じ逐次生成規律)。
void busy_load_launch() {
  FOR_KERNEL_OBJECT::create_object((uint32_t)object_ids::BUSY_LOAD);
  const uint32_t obj = (uint32_t)object_ids::BUSY_LOAD;
  uint32_t hog = FOR_KERNEL_OBJECT::async_call(obj, (method_t)busy_worker);
  thread_table[hog].affinity = AFFINITY_CORE1; // BLE と同じコアで取り合わせる
  // budget は既定 (SHIZU_DEFAULT_GRANT_BUDGET_US) のまま — ここを 0 にしないこと。
  FOR_KERNEL_OBJECT::async_call(obj, (method_t)reporter); // reporter は core0 既定
}

} // namespace shizu
