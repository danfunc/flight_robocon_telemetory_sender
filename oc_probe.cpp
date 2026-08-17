// CPU クロックの上限探索 (cmake -DSHIZU_OC_PROBE=ON、既定 OFF)。
//
// なぜ「1 回のブートで階段を登る」のか
// -----------------------------------
// 素朴には各周波数でビルドして焼けばよいが、**落ちた瞬間に物理 BOOTSEL 送り**になる
// (memory: wedged-firmware-is-unrecoverable — 固まると printf も picotool 復旧も効かない)。
// そこで 1 回のブートで 300→600MHz を順に試し、各段の結果を**その場で印字して排出**する。
// 途中で固まっても「最後に印字された段」までは分かる。
//
// さらに固まっても人手が要らないようにする 2 段構え:
//   1. **ウォッチドッグ**を階段の前に張る → 固まれば自動リブート (電源を抜かなくてよい)
//   2. **noinit RAM のフラグ** (`attempting_khz`) → リブート後に「前回どこで死んだか」が
//      分かり、**登り直さない**。だから次のブートは安全なクロックで正常起動し、
//      picotool で普通に焼き直せる。BOOTSEL 押しが要らない。
//
// ★終わったら必ず「ビルドが想定しているクロック (SHIZU_SYS_CLK_KHZ)」へ戻す。
//   CYW43 の PIO 分周はコンパイル時定数なので、探索の途中の周波数のまま本編を起動すると
//   BLE が壊れる (これも固定分周の連鎖)。このプローブはあくまで診断用。
//
// ★限界の意味に注意: ここで測るのは **core0 単独・センサ/BLE 停止・室温** の上限。
//   実運用 (両コア稼働 + 周辺 IRQ + 機体内の温度) の上限はこれより低い。
//   得られるのは「天井の位置」であって「常用してよい値」ではない。
#include <cstdint>
#include <cstdio>

#include <hardware/adc.h>
#include <hardware/clocks.h>
#include <hardware/regs/addressmap.h>
#include <hardware/structs/qmi.h>
#include <hardware/sync.h>
#include <hardware/vreg.h>
#include <hardware/watchdog.h>
#include <pico/stdlib.h>

#include <flash_clock.hpp>
#include <object_headers/SHIZUKU_USB.hpp>
#include <oc_probe.hpp>

namespace shizu {
namespace {

// ---- 階段 -------------------------------------------------------------------
// 300 は既知良 (基準チェックサムをここで採る)。以降 50MHz 刻み。
// ★PLL の制約: VCO = clk_sys × postdiv が **12MHz の整数倍** かつ 750..1600MHz で
//   なければ作れない。だから 350/440/550MHz などは「作れない」と弾かれる
//   (set_sys_clock_khz の第2引数 false で panic させずに判定している)。
//   400 OK / 450 ハングが分かったので、その間を作れる値で刻む。
// ★作れる周波数は「VCO = clk_sys × postdiv が 12MHz の整数倍かつ 750..1600MHz」。
//   postdiv=3 のとき clk_sys が 4 の倍数なら成立するので 12MHz 刻みで並べられる。
//   作れない値 (350/440/550 等) は set_sys_clock_khz(f,false) が false を返すので
//   panic させずにスキップする。
const uint32_t LADDER[] = {300000, 324000, 348000, 372000, 396000,
                           408000, 420000, 432000, 450000, 500000, 600000};
constexpr uint32_t CHECK_BYTES = 256u * 1024u; // フラッシュ照合の範囲
constexpr uint32_t WD_MS = 4000;               // 1 段あたりの猶予

// ---- リセットを跨ぐ状態 -----------------------------------------------------
// ★.uninitialized_data は通常リセット/ウォッチドッグリセットを跨いで残る
//   (このプロジェクトの panic_ring / svc_trace と同じ仕掛け)。BOOTSEL 経由では
//   bootrom が SRAM 下位を使うので消えるが、それは「人が介入した」ケースなので構わない。
struct oc_state {
  uint32_t magic;
  uint32_t attempting_khz; // 非 0 = この周波数を試している最中に落ちた
  uint32_t max_good_khz;   // 検証を通った最高周波数
};
constexpr uint32_t OC_MAGIC = 0x0C0C5AA5;
__attribute__((section(".uninitialized_data"))) oc_state g_oc;

// ---- 検証 (すべて RAM 常駐) -------------------------------------------------
// フラッシュ読みの照合。nocache 窓なのでキャッシュに騙されない。
uint32_t __no_inline_not_in_flash_func(flash_sum)(uint32_t bytes) {
  const volatile uint32_t *p =
      (const volatile uint32_t *)XIP_NOCACHE_NOALLOC_BASE;
  const uint32_t n = bytes / 4u;
  uint32_t s = 0xffffffffu;
  for (uint32_t i = 0; i < n; ++i)
    s = ((s >> 31) | (s << 1)) ^ p[i];
  return s;
}

// CPU + SRAM の健全性。フラッシュが読めても演算やレジスタ/SRAM が化ける可能性は
// 別なので独立に見る。答えが決まっているので 1 bit でも狂えば検出できる。
uint32_t __no_inline_not_in_flash_func(compute_check)() {
  static uint32_t buf[256]; // .bss (SRAM)
  uint32_t x = 0x12345678u;
  for (uint32_t i = 0; i < 256; ++i) {
    x = x * 1664525u + 1013904223u;
    buf[i] = x;
  }
  uint32_t acc = 0;
  for (uint32_t r = 0; r < 64; ++r)
    for (uint32_t i = 0; i < 256; ++i) {
      acc = ((acc << 5) | (acc >> 27)) ^ (buf[i] + r);
      acc = acc * 2654435761u + 1u;
    }
  return acc;
}

int32_t die_temp_x10_local() {
  // ADC は clk_usb 由来なので clk_sys を変えても換算は不変。ここでは
  // main.cpp と同じ式を使う (adc は main 側で init 済み前提にせず自前で見る)。
  adc_init();
  adc_set_temp_sensor_enabled(true);
  adc_select_input(ADC_TEMPERATURE_CHANNEL_NUM);
  const uint32_t raw = adc_read();
  const int32_t v_uv = (int32_t)((raw * 3300000ull) / 4096ull);
  return 270 - (int32_t)(((int64_t)(v_uv - 706000) * 10) / 1721);
}

void drain() {
#if SHIZU_USB_DRIVER
  shizuku_usb_emergency_drain();
#endif
}

} // namespace

void oc_probe_run() {
  // ---- 前回ハングの検出 ----------------------------------------------------
  if (g_oc.magic == OC_MAGIC && g_oc.attempting_khz != 0) {
    printf("[OC] !! previous boot HUNG while attempting %lu kHz "
           "(max verified good = %lu kHz)\n",
           (unsigned long)g_oc.attempting_khz,
           (unsigned long)g_oc.max_good_khz);
    printf("[OC] ladder skipped this boot (recovered by watchdog). "
           "clear with a power cycle or BOOTSEL to re-run.\n");
    g_oc.attempting_khz = 0; // 次からは普通に起動する
    watchdog_disable();
    drain();
    return;
  }
  if (g_oc.magic != OC_MAGIC) {
    g_oc.magic = OC_MAGIC;
    g_oc.max_good_khz = 0;
  }
  g_oc.attempting_khz = 0;

  // ---- 電圧を上げる --------------------------------------------------------
  // ★1.30V は SDK が「ここまでは追加操作なし」としている上限
  //   (これより上は POWMAN_VREG_CTRL_DISABLE_VOLTAGE_LIMIT が必要)。
  // ★探索電圧は cmake -DSHIZU_OC_PROBE_MV=1200/1250/1300 で選ぶ。
  //   1.30V は SDK が「ここまでは追加操作なし」としている上限
  //   (これより上は POWMAN_VREG_CTRL_DISABLE_VOLTAGE_LIMIT が必要)。
  //   ★出荷電圧 (1.20V) での上限も測ること — 常用構成の余裕はそこで決まる。
  //     1.30V の天井を見て「余裕がある」と判断するのは誤り。
#if SHIZU_OC_PROBE_MV >= 1300
  vreg_set_voltage(VREG_VOLTAGE_1_30);
#elif SHIZU_OC_PROBE_MV >= 1250
  vreg_set_voltage(VREG_VOLTAGE_1_25);
#else
  vreg_set_voltage(VREG_VOLTAGE_1_20);
#endif
  busy_wait_us(5000); // レギュレータ安定待ち (1.20V のときの 2ms より余裕を見る)
  printf("[OC] ==== CPU clock ladder @ vreg %d mV ====\n", SHIZU_OC_PROBE_MV);
  printf("[OC] note: core0 only, sensors/BLE not started, room temp. "
         "The real limit under dual-core load will be lower.\n");
  drain();

  uint32_t expect_sum = 0, expect_acc = 0;

  for (uint32_t i = 0; i < count_of(LADDER); ++i) {
    const uint32_t khz = LADDER[i];

    // ★固まっても人手が要らないように、クロックを上げる**前**に張る。
    watchdog_enable(WD_MS, false);
    g_oc.attempting_khz = khz; // ここで落ちたら次のブートが気づく
    __dmb();

    // 分周比を先に上げておく (上げた後の clk_sys でフラッシュが定格超過にならないように)。
    // ★段数でなく遅延 [ps] から導出する。探索は clk_sys を 2 倍近く振るので、
    //   固定段数だと「速い段でだけ窓から外れる」ことになり天井を誤判定する。
    flash_clock_apply_for(khz * 1000u, SHIZU_FLASH_TARGET_KHZ * 1000u,
                          flash_rxdelay_for(khz * 1000u, SHIZU_FLASH_RXDELAY_PS));
    if (!set_sys_clock_khz(khz, false)) {
      printf("[OC] %lu kHz: PLL cannot make this frequency, skipped\n",
             (unsigned long)khz);
      g_oc.attempting_khz = 0;
      watchdog_update();
      drain();
      continue;
    }
    // clk_sys が確定してから目標へ合わせ直す (分周は導出)。
    flash_clock_apply(
        SHIZU_FLASH_TARGET_KHZ * 1000u,
        flash_rxdelay_for(clock_get_hz(clk_sys), SHIZU_FLASH_RXDELAY_PS));
    watchdog_update();

    const uint32_t actual = clock_get_hz(clk_sys);
    const uint32_t fsum = flash_sum(CHECK_BYTES);
    const uint32_t acc = compute_check();
    watchdog_update();
    const int32_t t10 = die_temp_x10_local();

    if (i == 0) { // 300MHz = 既知良。ここで基準を採る
      expect_sum = fsum;
      expect_acc = acc;
    }
    const bool ok = (fsum == expect_sum) && (acc == expect_acc);
    printf("[OC] %lu kHz: actual=%lu Hz flash=%lu Hz die=%ld.%ldC "
           "fsum=%08lx acc=%08lx %s\n",
           (unsigned long)khz, (unsigned long)actual,
           (unsigned long)flash_clock_hz(), (long)(t10 / 10),
           (long)(t10 < 0 ? -t10 % 10 : t10 % 10), (unsigned long)fsum,
           (unsigned long)acc,
           ok ? "OK" : "**CORRUPT** (計算 or フラッシュ読みが化けた)");
    drain();
    watchdog_update();

    if (!ok) {
      // 生きてはいるがデータが化けた = ここが上限。これ以上は上げない。
      g_oc.attempting_khz = 0;
      break;
    }
    g_oc.max_good_khz = khz;
    g_oc.attempting_khz = 0; // この段は生き延びた
    __dmb();
  }

  watchdog_disable();

  // ---- ビルドが想定しているクロックへ必ず戻す ------------------------------
  // ★CYW43 の PIO 分周はコンパイル時定数。探索途中の周波数のまま本編を起動すると
  //   BLE が壊れる。プローブは診断専用。
  vreg_set_voltage((enum vreg_voltage)SHIZU_VREG_ENUM);
  busy_wait_us(2000);
  flash_clock_apply_for(
      SHIZU_SYS_CLK_KHZ * 1000u, SHIZU_FLASH_TARGET_KHZ * 1000u,
      flash_rxdelay_for(SHIZU_SYS_CLK_KHZ * 1000u, SHIZU_FLASH_RXDELAY_PS));
  set_sys_clock_khz(SHIZU_SYS_CLK_KHZ, true);
  flash_clock_apply(
      SHIZU_FLASH_TARGET_KHZ * 1000u,
      flash_rxdelay_for(clock_get_hz(clk_sys), SHIZU_FLASH_RXDELAY_PS));

  printf("[OC] ==== max verified good = %lu kHz ====\n",
         (unsigned long)g_oc.max_good_khz);
  printf("[OC] restored to build clock %lu kHz (vreg 1.20V), flash=%lu Hz\n",
         (unsigned long)SHIZU_SYS_CLK_KHZ, (unsigned long)flash_clock_hz());
  drain();
}

} // namespace shizu
