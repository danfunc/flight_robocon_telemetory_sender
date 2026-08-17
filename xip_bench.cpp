// XIP (QSPI フラッシュ) 速度ベンチ + 読み違い検出。
//
// ねらい: `PICO_FLASH_SPI_CLKDIV` / `PICO_FLASH_SPI_RXDELAY` を上げたときに
//   (a) 本当に速くなっているのか、(b) 静かにデータを読み違えていないか、
// を **1 回のブートで全組み合わせぶん** 測る。
//
// なぜ実行時スイープなのか
// ------------------------
// この 2 定数は boot2 (`boot2_w25q080.S` の INIT_M0_TIMING) のコンパイル時定数だが、
// 実体は QMI の `M0_TIMING` レジスタの CLKDIV[7:0] / RXDELAY[10:8] フィールドでしか
// なく、**実行時に書き換えられる**。boot2 と違うのは COOLDOWN 等の他フィールドだけで、
// それらは触らないので「再ビルドして焼き直す」のと等価。20 通りを 6 回の
// 焼き直しでなく 1 ブートで測れる。決まった値は最後に CMake 側 (boot2) へ焼き付ける。
//
// ★安全順序 (ここが本質)
// ----------------------
// 速すぎる設定は「落ちる」のではなく **静かにデータを読み違える**。しかもそれが
// 命令フェッチなら、そのまま暴走 → 物理 BOOTSEL 送り (memory: wedged-firmware-is-
// unrecoverable)。なので 1 設定ぶんの試験はすべて **RAM 上のコード** で行い、
//
//   1. タイミング変更 (RAM)
//   2. **nocache エイリアスでデータだけ読んで**チェックサム照合 (RAM ループ)
//   3. 通ったらキャッシュ経由でも照合 (ラインフィル = バースト読みの検証)
//   4. **両方通ったときだけ** フラッシュ上のコードを呼ぶ命令フェッチ試験へ進む
//   5. どう転んでも既知良のタイミングへ戻し、キャッシュを捨ててから戻る
//
// の順に進む。2 で外れた設定は「データを読み違えた」と記録するだけで、その設定の
// ままフラッシュ命令を実行しない。3 で外れた場合もキャッシュに毒 (壊れたライン) が
// 残るので、失敗・成功にかかわらず必ず invalidate してから復帰する。
//
// ★「起動した = OK」で判断しないこと
// 読み違えはランダムかつ無症状で潜伏する。だから速度と**必ず組で**チェックサムを
// 見る。照合値は「ブート時のタイミング (= boot2 が設定した既知良の値)」で測った
// ものを基準にし、同じ値をホスト側でも main.bin から再計算できるようにしてある
// (tools/xip_bench_check.py)。
//
//   cmake -DSHIZU_XIP_BENCH=ON ...   で有効化 (既定 OFF)。
#include <cstdint>
#include <cstdio>

#include <hardware/clocks.h>
#include <hardware/regs/addressmap.h>
#include <hardware/structs/qmi.h>
#include <hardware/sync.h>
#include <hardware/timer.h>
#include <hardware/xip_cache.h>
#include <pico/stdlib.h>

#include <object_headers/SHIZUKU_USB.hpp> // shizuku_usb_emergency_drain
#include <xip_bench.hpp>

namespace {

typedef uint32_t (*xf_t)(uint32_t);
#include "xip_bench_fns.inc" // xf_0..xf_N-1 (128B 境界に散らした的) と xf_flash_table

// ---- 調律つまみ -------------------------------------------------------------
// チェックサム対象。バイナリ先頭から。実データ (高エントロピー) の範囲に収める。
constexpr uint32_t REGION_BYTES = 256u * 1024u;
// 命令フェッチ試験の呼び出し回数。1 回 ≈ 100-300ns なので数十 ms。
constexpr uint32_t IFETCH_ITERS = 200000u;
// IRQ を止める時間を 1 設定あたり ~50ms に抑えるための上限でもある
// (USB CDC は数十 ms の無応答なら再送で吸収される)。

// 命令フェッチ試験の飛び先テーブル。**RAM に置く** — フラッシュ上の
// xf_flash_table を毎回引くと、測りたい命令フェッチにデータ読みが混ざる。
xf_t g_xf_ram[XIP_BENCH_NFN];

struct probe_result {
  uint32_t sum_nc;  // nocache エイリアス経由のチェックサム
  uint32_t sum_c;   // キャッシュ経由のチェックサム
  uint32_t us_nc;   // 同、所要 µs
  uint32_t us_c;    //
  uint32_t us_if;   // 命令フェッチ試験の所要 µs (未実行なら 0)
  uint32_t acc_if;  // 同、累算結果 (命令列が壊れれば変わる)
  uint32_t timing;  // 実際に書き込んだ M0_TIMING 全体 (自己申告でなく読み戻し)
};

// ---- チェックサム -----------------------------------------------------------
// CRC32 は使わない: テーブル引き 8 演算/byte が 300MHz でも ~37MB/s 相当で、
// フラッシュ読み (~20-60MB/s) と同じ桁になってしまい**速度差を潰す**。
// ror+xor なら 2 命令/word ≈ 400MB/s 以上で、律速は確実にフラッシュ側になる。
// 位置依存なので誤読の検出力も (単純 XOR と違って) 十分ある。
// ホスト側は tools/xip_bench_check.py が同じ式で main.bin から再計算する。
uint32_t __no_inline_not_in_flash_func(rot_sum)(uintptr_t base, uint32_t bytes) {
  const volatile uint32_t *p = (const volatile uint32_t *)base;
  const uint32_t n = bytes / 4u;
  uint32_t s = 0xffffffffu;
  for (uint32_t i = 0; i < n; ++i) {
    uint32_t w = p[i];
    s = ((s >> 31) | (s << 1)) ^ w;
  }
  return s;
}

// ---- QMI タイミングの実行時変更 --------------------------------------------
// 必ず RAM から。書き換え直後に nocache から 1 語読んで新設定を確定させる。
void __no_inline_not_in_flash_func(qmi_write_timing)(uint32_t timing) {
  qmi_hw->m[0].timing = timing;
  (void)*(const volatile uint32_t *)XIP_NOCACHE_NOALLOC_BASE;
  __compiler_memory_barrier();
}

uint32_t timing_with(uint32_t base, uint32_t clkdiv, uint32_t rxdelay) {
  return (base & ~(QMI_M0_TIMING_CLKDIV_BITS | QMI_M0_TIMING_RXDELAY_BITS)) |
         (clkdiv << QMI_M0_TIMING_CLKDIV_LSB) |
         (rxdelay << QMI_M0_TIMING_RXDELAY_LSB);
}

// ---- 1 設定ぶんの試験 (全部 RAM 上で完結させる) -----------------------------
// safe_timing = 既知良 (boot2 が設定した値)。expect = その設定で測った基準値。
void __no_inline_not_in_flash_func(probe_one)(uint32_t clkdiv, uint32_t rxdelay,
                                              uint32_t safe_timing,
                                              uint32_t expect,
                                              probe_result *r) {
  const uint32_t save = save_and_disable_interrupts();

  qmi_write_timing(timing_with(safe_timing, clkdiv, rxdelay));
  r->timing = qmi_hw->m[0].timing; // 自己申告でなく読み戻す

  // (1) データだけ・キャッシュを通さず読む。ここで外れる設定はコードを踏ませない。
  uint32_t t0 = timer_hw->timerawl;
  r->sum_nc = rot_sum(XIP_NOCACHE_NOALLOC_BASE, REGION_BYTES);
  r->us_nc = timer_hw->timerawl - t0;

  // (2) キャッシュ経由 (= 8B ラインフィルのバースト読み)。nocache が通っても
  //     バーストで崩れる設定があり得るので別に見る。
  xip_cache_invalidate_all(); // この関数自体が RAM 常駐 (SDK)
  t0 = timer_hw->timerawl;
  r->sum_c = rot_sum(XIP_BASE, REGION_BYTES);
  r->us_c = timer_hw->timerawl - t0;

  // (3) 両方一致したときだけ、フラッシュ上のコードを呼ぶ。
  r->us_if = 0;
  r->acc_if = 0;
  if (r->sum_nc == expect && r->sum_c == expect) {
    xip_cache_invalidate_all();
    uint32_t idx = 12345u, acc = 0u;
    t0 = timer_hw->timerawl;
    for (uint32_t i = 0; i < IFETCH_ITERS; ++i) {
      idx = idx * 1103515245u + 12345u;
      acc = g_xf_ram[(idx >> 16) & (XIP_BENCH_NFN - 1u)](acc);
    }
    r->us_if = timer_hw->timerawl - t0;
    r->acc_if = acc;
  }

  // (4) 何があっても既知良へ戻し、毒入りキャッシュを捨ててから帰る。
  qmi_write_timing(safe_timing);
  xip_cache_invalidate_all();
  restore_interrupts(save);
}

// bytes/µs → 100 倍した MB/s (整数で印字するため)
uint32_t mbps_x100(uint32_t bytes, uint32_t us) {
  if (!us)
    return 0;
  return (uint32_t)((uint64_t)bytes * 100u / us);
}

void print_row(const char *tag, uint32_t clkdiv, uint32_t rxdelay,
               uint32_t sys_hz, const probe_result &r, uint32_t expect) {
  const uint32_t f_khz = clkdiv ? (sys_hz / 1000u) / clkdiv : 0;
  const uint32_t nc = mbps_x100(REGION_BYTES, r.us_nc);
  const uint32_t c = mbps_x100(REGION_BYTES, r.us_c);
  const bool ok = (r.sum_nc == expect) && (r.sum_c == expect);
  if (!ok) {
    printf("[XIP] %s div=%lu rx=%lu flash=%lu.%03luMHz  BAD READ "
           "sum_nc=%08lx sum_c=%08lx (expect %08lx)\n",
           tag, (unsigned long)clkdiv, (unsigned long)rxdelay,
           (unsigned long)(f_khz / 1000), (unsigned long)(f_khz % 1000),
           (unsigned long)r.sum_nc, (unsigned long)r.sum_c,
           (unsigned long)expect);
    return;
  }
  // ns/call は µs*1000/回数。IFETCH_ITERS=200k なので µs → ns は 5 倍。
  const uint32_t ns_call = r.us_if ? (uint32_t)((uint64_t)r.us_if * 1000u / IFETCH_ITERS) : 0;
  const uint32_t ps_call = r.us_if ? (uint32_t)(((uint64_t)r.us_if * 1000000u / IFETCH_ITERS) % 1000u) : 0;
  printf("[XIP] %s div=%lu rx=%lu flash=%lu.%03luMHz  ok  "
         "seq_nc=%lu.%02luMB/s seq_c=%lu.%02luMB/s ifetch=%lu.%03luns/call "
         "acc=%08lx timing=%08lx\n",
         tag, (unsigned long)clkdiv, (unsigned long)rxdelay,
         (unsigned long)(f_khz / 1000), (unsigned long)(f_khz % 1000),
         (unsigned long)(nc / 100), (unsigned long)(nc % 100),
         (unsigned long)(c / 100), (unsigned long)(c % 100),
         (unsigned long)ns_call, (unsigned long)ps_call,
         (unsigned long)r.acc_if, (unsigned long)r.timing);
}

// スイープ表。div=2 (300MHz なら 150MHz = W25Q の定格 133MHz 超) も入れてあるが、
// 上記の安全順序でデータ照合が先に落ちるだけなので暴走しない。
struct sweep_entry {
  uint8_t clkdiv;
  uint8_t rxdelay;
};
// ★候補構成 (div=3 と div=4) では RXDELAY を **0..7 全域** 舐める。窓の「端がどこか」
//   を知らないとマージンを語れない — 「rx=2 は通る」だけでは、そこが窓の中央なのか
//   崖の 1 歩手前なのか区別できない。RXDELAY は 3bit なので 0..7 が全域。
const sweep_entry SWEEP[] = {
    {6, 2},
    {4, 0}, {4, 1}, {4, 2}, {4, 3}, {4, 4}, {4, 5}, {4, 6}, {4, 7},
    {3, 0}, {3, 1}, {3, 2}, {3, 3}, {3, 4}, {3, 5}, {3, 6}, {3, 7},
    {2, 1}, {2, 2}, {2, 3}, {2, 4},
};

} // namespace

extern "C" {
extern char __flash_binary_start;
extern char __flash_binary_end;
}

namespace shizu {

void xip_bench_run() {
  for (uint32_t i = 0; i < XIP_BENCH_NFN; ++i)
    g_xf_ram[i] = xf_flash_table[i];

  const uint32_t sys_hz = clock_get_hz(clk_sys);
  const uint32_t boot_timing = qmi_hw->m[0].timing;
  const uint32_t boot_div = (boot_timing & QMI_M0_TIMING_CLKDIV_BITS) >>
                            QMI_M0_TIMING_CLKDIV_LSB;
  const uint32_t boot_rx = (boot_timing & QMI_M0_TIMING_RXDELAY_BITS) >>
                           QMI_M0_TIMING_RXDELAY_LSB;
  const uint32_t bin_bytes =
      (uint32_t)(&__flash_binary_end - &__flash_binary_start);

  printf("[XIP] ==== flash QSPI sweep ====\n");
  printf("[XIP] sys=%lu Hz  boot2 timing=%08lx (div=%lu rx=%lu -> %lu kHz)  "
         "region=%luKB  image=%lu B  cache=%uKB/%uB-line  iters=%lu\n",
         (unsigned long)sys_hz, (unsigned long)boot_timing,
         (unsigned long)boot_div, (unsigned long)boot_rx,
         (unsigned long)(boot_div ? (sys_hz / 1000u) / boot_div : 0),
         (unsigned long)(REGION_BYTES / 1024), (unsigned long)bin_bytes,
         (unsigned)(XIP_CACHE_SIZE / 1024), (unsigned)XIP_CACHE_LINE_SIZE,
         (unsigned long)IFETCH_ITERS);

  // 基準値は「boot2 が設定した既知良のタイミング」で測る。以後の全設定はこれと
  // 突き合わせる。ホストからも tools/xip_bench_check.py で再計算できる。
  probe_result base{};
  probe_one(boot_div, boot_rx, boot_timing, 0u /*照合はまだできない*/, &base);
  const uint32_t expect = base.sum_nc;
  if (base.sum_c != expect) {
    // 起動しているのだからここは一致するはず。外れたらベンチ側の欠陥を疑う。
    printf("[XIP] !! baseline self-inconsistent: nc=%08lx c=%08lx\n",
           (unsigned long)expect, (unsigned long)base.sum_c);
  }
  printf("[XIP] baseline checksum=%08lx over %lu bytes @0x%08lx "
         "(host: tools/xip_bench_check.py <main.bin>)\n",
         (unsigned long)expect, (unsigned long)REGION_BYTES,
         (unsigned long)XIP_BASE);
  // 基準値が出た時点でもう一度だけ基準設定を測り直す (ifetch の acc も得る)。
  probe_one(boot_div, boot_rx, boot_timing, expect, &base);
  print_row("BASE", boot_div, boot_rx, sys_hz, base, expect);
#if SHIZU_USB_DRIVER
  // ★1 行ずつ確実に出す。どこかで暴走しても「直前まで何が通ったか」が残るように。
  shizu::shizuku_usb_emergency_drain();
#endif

  for (const sweep_entry &e : SWEEP) {
    probe_result r{};
    probe_one(e.clkdiv, e.rxdelay, boot_timing, expect, &r);
    print_row("SWP ", e.clkdiv, e.rxdelay, sys_hz, r, expect);
    if (r.us_if && r.acc_if != base.acc_if)
      printf("[XIP] !! ifetch acc mismatch: %08lx vs base %08lx (命令列を"
             "読み違えている)\n",
             (unsigned long)r.acc_if, (unsigned long)base.acc_if);
#if SHIZU_USB_DRIVER
    shizu::shizuku_usb_emergency_drain();
#endif
  }
  printf("[XIP] ==== sweep done, timing restored to %08lx ====\n",
         (unsigned long)qmi_hw->m[0].timing);
#if SHIZU_USB_DRIVER
  shizu::shizuku_usb_emergency_drain();
#endif
}

} // namespace shizu
