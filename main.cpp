
#include "hardware/structs/scb.h"
#include <cstdio>
#include <cstdlib>
#include <hardware/exception.h>
#include <hardware/adc.h>  // ダイ温度 (内蔵温度センサ)
#include <hardware/gpio.h>
#include <hardware/clocks.h> // set_sys_clock_khz (オーバークロック)
#include <hardware/vreg.h>   // vreg_set_voltage (同上)
#include <core_ring.hpp> // core1_io_launch (センサ I/O の core1 分離)
#include <flash_clock.hpp> // XIP フラッシュのクロックを clk_sys から導出する
#include <panic_ring.hpp> // panic の noinit RAM リング (boot/beacon で印字)
#include <object_headers/SHIZUKU_USB.hpp> // 非ブロッキング printf (shizuku_usb_install)
#include <pico/stdio.h>
#include <pico/stdlib.h>
#include <pico/time.h>
#include <shizu.hpp>
#include <time.h>
#include <oc_probe.hpp> // CPU クロック上限探索 (SHIZU_OC_PROBE)
#include <xip_bench.hpp> // XIP (フラッシュ QSPI) 速度スイープ (SHIZU_XIP_BENCH)
#include <tusb.h> // HardFault 中に USB CDC を手動ポンプするため

// RP2350 内蔵温度センサ (ADC 最終チャネル) からダイ温度を 0.1℃ 単位で読む。
// データシートの換算: T = 27 - (V - 0.706) / 0.001721、V = raw * 3.3 / 4096。
// ADC は clk_usb (48MHz) 由来なので sys クロックを変えても換算式は変わらない。
// ★このプロジェクトは他に ADC を使っていないので (grep 済み)、チャネル選択を
//   奪い合う相手が居ない。もし ADC を使うコードを足すなら排他を考えること。
static int32_t die_temp_x10() {
  static bool inited = false;
  if (!inited) {
    adc_init();
    adc_set_temp_sensor_enabled(true);
    inited = true;
  }
  adc_select_input(ADC_TEMPERATURE_CHANNEL_NUM);
  const uint32_t raw = adc_read(); // 12bit
  // 整数演算のまま 0.1℃ 単位へ。V_uV = raw * 3300000 / 4096。
  const int32_t v_uv = (int32_t)((raw * 3300000ull) / 4096ull);
  // T_x10 = 270 - (v_uv - 706000) * 10 / 1721
  return 270 - (int32_t)(((int64_t)(v_uv - 706000) * 10) / 1721);
}

void *get_psp() {
  void *psp;
  asm volatile("MRS %0, PSP" : "=r"(psp));
  return psp;
}
void *get_msp() {
  void *msp;
  asm volatile("MRS %0, MSP" : "=r"(msp));
  return msp;
}

void test_svc_handler() {
  printf("test_svc_handler\n");
  //  sleep_ms(100);
}

void hardfault_handler() {

  uint32_t volatile exc_lr;
  asm volatile("mov %[lr], lr" : [lr] "=r"(exc_lr));
  // EXC_RETURN bit2: 0=MSP, 1=PSP。例外発生時のスタックポインタを選び、
  // そこに積まれた例外フレーム {r0,r1,r2,r3,r12,lr,pc,xPSR} を読む。
  uint32_t *sf = (exc_lr & 0x4) ? (uint32_t *)get_psp() : (uint32_t *)get_msp();
  while (1) {
    printf("HardFault: exc_lr=%lx psp=%p msp=%p\n", exc_lr, get_psp(),
           get_msp());
    printf("  r0=%08lx r1=%08lx r2=%08lx r3=%08lx\n", sf[0], sf[1], sf[2],
           sf[3]);
    printf("  r12=%08lx lr=%08lx pc=%08lx xpsr=%08lx\n", sf[4], sf[5], sf[6],
           sf[7]);
    // HardFault は最高優先度なので stdio_usb の低優先度 IRQ (tud_task) が走れず、
    // printf がホストへ届かない=「無言フリーズ」に見える。ここで手動ポンプして
    // ダンプを届ける (busy_wait はタイマ IRQ 不要なので sleep_ms の代わりに使う)。
    for (int i = 0; i < 1000; ++i) {
      tud_task();
      busy_wait_us(1000);
    }
  }
}

// ---------------------------------------------------------------------------
//  生存ビーコン (フリーズ箇所の特定用デバッグ計装)
// ---------------------------------------------------------------------------
// 協調スケジューラでは 1 スレッドが yield せず回り続けると全スレッドが止まり、
// ログが完全に沈黙する。このビーコンはタイマ IRQ (スレッドとは独立に走る) から
// 5 秒ごとに「今 RUNNING のスレッド」と「割り込まれた地点の pc」を出す。
// フリーズ時に同じ pc が出続ければ、そのアドレスが無限ループの現場
// (arm-none-eabi-addr2line -e build/main.elf <pc> で行番号に変換できる)。
// 何も出なくなった場合は IRQ ごと止まる系 (HardFault 連鎖 / 割り込み禁止での
// ハング) と切り分けられる。
// poll/evt は BLE_UART_DRIVER の生存カウンタ: poll が進まない=BLE スレッド停止、
// poll は進むが evt が進まない=HCI イベント経路 (CYW43↔host) の停止、と読む。
#if SHIZU_NO_BLE
// Pico 2 (無印) テストビルド: BLE_UART_DRIVER がリンクされないため、ビーコンが読む
// カウンタと ble_tx の計装変数 (TELEMETRY の ble_send_prio が stamp する) をここで
// 実体化する (常に 0 = BLE 非搭載の意)。
namespace shizu {
volatile uint32_t ble_dbg_poll = 0, ble_dbg_evt = 0;
namespace ble_tx {
volatile uint64_t ctrl_enq_us = 0;
volatile uint32_t ctrl_lat_last_us = 0;
volatile uint32_t ctrl_lat_max_us = 0;
} // namespace ble_tx
}
#else
namespace shizu {
extern volatile uint32_t ble_dbg_poll, ble_dbg_evt;
}
#endif
static bool beacon_cb(repeating_timer_t *) {
  // panic リング監視: どちらかのコアが panic していたらメッセージを印字する
  // (panic した core1 自身は stdio に書けないが、core0 のこの IRQ は生きている)。
  shizu::panic_ring_poll_report();

  uint32_t running_thread = 0xFFFFFFFF, running_obj = 0xFFFFFFFF;
  for (uint32_t i = 0; i < 128; ++i) {
    if (shizu::thread_table[i].state == shizu::thread_t::state_t::RUNNING) {
      running_thread = i;
      running_obj = shizu::thread_table[i].object_id;
      break;
    }
  }
  uint32_t psp;
  asm volatile("MRS %0, PSP" : "=r"(psp));
  const uint32_t *f = (const uint32_t *)psp; // 割り込まれたスレッドの例外フレーム
  // ★ダイ温度を常に出す。300MHz + vreg 1.20V の常用で唯一未検証なのが「長時間の
  //   熱安定性」で、しかも触って熱くないことは根拠にならない (ダイと筐体表面は別、
  //   密閉した機体では風も無い)。ADC は clk_usb 由来なので sys クロックを変えても
  //   換算式は変わらない。10倍した整数で出す (printf の %f を避ける)。
  //   これで飛行を含む実運用の温度がログに勝手に残る。
  const int32_t t10 = die_temp_x10(); // 1 回だけ読む (2 回読むと桁が食い違う)
  printf("[BEACON] thr=%lu obj=%lu pc=%08lx lr=%08lx ble_poll=%lu evt=%lu "
         "die=%ld.%ldC\n",
         (unsigned long)running_thread, (unsigned long)running_obj, f[6], f[5],
         (unsigned long)shizu::ble_dbg_poll, (unsigned long)shizu::ble_dbg_evt,
         (long)(t10 / 10), (long)(t10 < 0 ? -t10 % 10 : t10 % 10));
  // フラッシュのタイミングが起動時に設定した値からずれていないか見張る。
  // ずれる経路は一つ: フラッシュ書き込み (BTstack のペアリング鍵保存など) の後、
  // SDK が ROM の flash_enter_cmd_xip() で XIP を張り直して ROM 既定へ戻すため。
  // 壊れはしない (ROM 既定は常に安全側) が、**黙って遅くなる**ので必ず表に出す。
  // ここで書き直さないのは、他コアが XIP 転送中に M0_TIMING を触るのが安全でない
  // ため。直すなら core1 が止まっている flash_safe_execute の中で行うこと。
  if (shizu::flash_clock_drifted())
    printf("[FLASH] timing drifted -> %lu Hz (flash write reset XIP setup)\n",
           (unsigned long)shizu::flash_clock_hz());
#if SHIZU_USB_DRIVER
  // ★詰まり検出 → 最終手段の同期排出。SHIZU_USB_DRIVER では printf はリングへ積む
  // だけで、排出は SHIZUKU_USB スレッドが行う。系が固まるとその排出も止まるので、
  // **panic を含めて一切表に出なくなる** (実際にこれで無音ロックアップの原因究明が
  // 何度も空振りした)。このビーコンはタイマ IRQ 駆動でスレッドが死んでも生きている
  // ので、ここから救出する。
  // 「溜まっていて、かつ前回のビーコンから 1 バイトも減っていない」= 排出スレッドが
  // 走れていない、と判定する。健全時は必ず減っている (5s あれば必ず捌ける) ので、
  // 通常運転で IRQ から tud_task を叩いて排出スレッドと競合することはない。
  {
    static uint32_t prev_used = 0;
    const uint32_t used = shizu::shizuku_usb_ring_used();
    if (used != 0 && used == prev_used)
      shizu::shizuku_usb_emergency_drain();
    prev_used = used;
  }
#endif

  // per-core CPU 使用率 = Δcpu_busy_us/Δwall (この 5s 窓)。cpu_busy_us は各コアの共通
  // scheduler_idle_loop が「スケジューラが実務へ渡して戻ってきた時間」を足したもの。
  static uint64_t prev_busy[2] = {0, 0};
  static uint64_t prev_t = 0;
  uint64_t now64 = time_us_64();
  if (prev_t != 0) {
    uint64_t wall = now64 - prev_t;
    for (int c = 0; c < 2; ++c) {
      uint64_t busy = shizu::cpu_busy_us[c];
      uint64_t d_busy = busy - prev_busy[c];
      if (d_busy > wall)
        d_busy = wall; // クランプ (境界のズレ対策)
      uint32_t util_x10 = wall ? (uint32_t)(1000ull * d_busy / wall) : 0;
      printf("[CPU] core%d util=%lu.%lu%%\n", c, (unsigned long)(util_x10 / 10),
             (unsigned long)(util_x10 % 10));
      prev_busy[c] = busy;
    }
  }
  prev_t = now64;
  return true;
}

int main(int argc, char const *argv[]) {
#if defined(SHIZU_SYS_CLK_KHZ)
  // ★ stdio_init_all より**前**に上げること (UART ボーレートは clk_peri から算出
  //   されるため)。順序は「電圧を上げる → 安定待ち → クロックを上げる」。
  //   フラッシュ分周は下の flash_clock_apply_for が目標周波数から導出する。
  //   カーネル側の µs 換算 (g_cycles_per_us) は cpu_manager::init が
  //   clock_get_hz(clk_sys) から取るので、ここで変えれば自動で追従する。
  //   SysTick は 24bit なので 1 チャンクの上限が 300MHz では ~56ms に縮むが、
  //   systick_arm_for がチャンク継ぎをするので期限自体は変わらない。
  // ★コア電圧も直書きしない (SHIZU_VREG_MV、既定 1200)。実測の天井は
  //   1.20V=372MHz / 1.30V=432MHz なので 300MHz は 1.20V で 24% の余裕がある。
  //   上げても +16% の天井しか買えず、恒久的に消費電力と素子ストレスを払うだけ。
  //   熱は律速していない (全負荷 120s で 40.2℃ 飽和) ので上げる理由が無い。
  vreg_set_voltage((enum vreg_voltage)SHIZU_VREG_ENUM);
  busy_wait_us(2000); // レギュレータ安定待ち
  // ★clk_sys を上げる**前に**、上げた後の clk_sys を前提にフラッシュ分周を決めて
  //   おく。QMI の分周比は固定なので、先に clk_sys だけ上げるとフラッシュ速度が
  //   そのまま比例して跳ね上がる (150→300MHz で 50→100MHz になっていた)。
  //   分周比を先に大きくしておけば、その瞬間に定格を超えることはない。
  shizu::flash_clock_apply_for(
      SHIZU_SYS_CLK_KHZ * 1000u, SHIZU_FLASH_TARGET_KHZ * 1000u,
      shizu::flash_rxdelay_for(SHIZU_SYS_CLK_KHZ * 1000u,
                               SHIZU_FLASH_RXDELAY_PS));
  set_sys_clock_khz(SHIZU_SYS_CLK_KHZ, true);
#endif
  // ---- clk_peri を SPI/UART に足る値へ引き上げる ---------------------------
  // ★`set_sys_clock_khz` は clk_peri を PLL_USB (48MHz) へ張り替えてしまう。48MHz だと
  //   SPI の上限が clk_peri/2 = 24MHz (spi_set_baudrate の prescale 最小 2)。
  //   RP2350 の clk_peri は分周器付き (2bit) なので、PLL_SYS から分周して
  //   「CPU 300MHz のまま clk_peri 150MHz」にできる。150MHz は RP2350 の定格 sys
  //   クロックと同値 = 周辺としても定格内。SPI 上限は 75MHz に上がる。
  // ★必ず stdio_init_all より前に行うこと (UART ボーレートは clk_peri から算出される)。
  //   分周比は固定値を書かず clk_sys から導出する。
#if SHIZU_PERI_TARGET_KHZ > 0
  {
    const uint32_t sys_hz = clock_get_hz(clk_sys);
    const uint32_t want = SHIZU_PERI_TARGET_KHZ * 1000u;
    uint32_t div = (sys_hz + want - 1u) / want; // 目標を超えない最小の分周比
    if (div < 1u)
      div = 1u;
    if (div > 4u)
      div = 4u; // CLK_PERI_DIV_INT は 2bit (1..3、0 は 4 の意)
    clock_configure_int_divider(clk_peri, 0,
                                CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
                                sys_hz, div);
  }
#endif
  // clk_sys 確定後にあらためて適用する (OC しないビルドではここが唯一の適用点)。
  // 分周比 = ceil(clk_sys / target) を実行時に導出する — 固定値は書かない。
  // rxdelay も clk_sys から導出する (段数は半サイクル単位なので固定値は書けない)。
  shizu::flash_clock_apply(
      SHIZU_FLASH_TARGET_KHZ * 1000u,
      shizu::flash_rxdelay_for(clock_get_hz(clk_sys), SHIZU_FLASH_RXDELAY_PS));
  stdio_init_all();
#if SHIZU_USB_DRIVER
  // ★stdio_init_all の直後に差し替える。以後 printf は「リングへ積んで即 return」に
  // なり、どのスレッド/コア/IRQ から呼んでも待たない (SDK 実装は呼び出し元スレッドで
  // 1000ms/500ms 待つ = BLE 接続時フリーズの実測原因)。排出は SHIZUKU_USB の
  // スレッドが後から始めるので、それまでの起動ログはリングに溜まる。
  shizu::shizuku_usb_install();
#endif
  exception_set_exclusive_handler(HARDFAULT_EXCEPTION, hardfault_handler);
  sleep_ms(2000);
  // 前回実行の panic 残骸 (noinit RAM はリセットを跨ぐ) があれば印字してクリア。
  shizu::panic_ring_boot_report();
#if SHIZU_SVC_DELEGATION
  shizu::svc_trace_boot_report(); // 前回ハング時の svc 経路 (noinit RAM に残る)
#endif
  // ★実クロックを必ず印字する。「設定したつもり」で確認しないと、変わっていない
  //   状態の観測を有効な結果として扱ってしまう (2026-08-17 に CONTROL の
  //   自己申告を省いて非特権化を誤認した教訓)。
  printf("[CLK] sys=%lu Hz peri=%lu Hz flash=%lu Hz rx=%lu (%lu ps) "
         "vreg=%d mV\n",
         (unsigned long)clock_get_hz(clk_sys),
         (unsigned long)clock_get_hz(clk_peri),
         (unsigned long)shizu::flash_clock_hz(),
         (unsigned long)shizu::flash_rxdelay_for(clock_get_hz(clk_sys),
                                                SHIZU_FLASH_RXDELAY_PS),
         (unsigned long)SHIZU_FLASH_RXDELAY_PS, SHIZU_VREG_MV);
#if SHIZU_USB_DRIVER
  // panic 残骸は Shizuku 起動前 (排出スレッド不在) に出るので、ここだけは同期排出で
  // 確実に見せる。可視性 > RT の文脈。
  shizu::shizuku_usb_emergency_drain();
#endif
#if SHIZU_FLASH_DRIFT_PROBE
  shizu::flash_clock_drift_probe();
#if SHIZU_USB_DRIVER
  shizu::shizuku_usb_emergency_drain();
#endif
#endif
#if SHIZU_OC_PROBE
  // ★core1 もセンサも BLE もまだ動いていないここで測る。終わったらビルドが想定する
  //   クロックへ戻すので、以降の起動は通常どおり。
  shizu::oc_probe_run();
#endif
#if SHIZU_XIP_BENCH
  // ★ここ (Shizuku 起動前・core1 起動前・ビーコンタイマ登録前) で測る。
  //   XIP バスを取り合う相手が居ない状態でないと、フラッシュ速度の差が
  //   他コアの待ちに埋もれる。
  shizu::xip_bench_run();
#endif
  // 生存ビーコン開始 (5s 周期、タイマ IRQ 駆動なのでスレッドが固まっても出続ける)。
  static repeating_timer_t beacon_timer;
  add_repeating_timer_ms(-5000, beacon_cb, nullptr, &beacon_timer);
  // core1 のセンサ I/O ループを起動 (I2C と BNO055/BME280 は core1 が専有)。
  // cyw43/BTstack の初期化は core0 の Shizuku スレッド (BLE_UART_DRIVER) 内で
  // 後から走るが、poll モードで core1 を使わないため順序上の競合はない。
  // Shizuku 起動前に立ち上げておけば、ドライバスレッドが動き出す頃には
  // リングへレコードが流れ始めている。
#if !SHIZU_CORE1_KERNEL_POC
  // 従来経路: core1 はベアメタルのセンサ I/O ループ。
  shizu::core1_io_launch();
#else
  // デュアルコア: core1 は Shizuku カーネルが取り、センサ I/O は SENSOR_IO スレッド
  // (core1 ピン留め) として走る。ベアメタル core1_io は起動しない (kernel_object_main
  // が core1_kernel_launch で SENSOR_IO を立ち上げる)。SHIZU_CORE1_KERNEL_POC は
  // include/kernel.hpp で定義。
  printf("[main] dual-core: core1 = Shizuku kernel + SENSOR_IO (sensors ON)\n");
#endif
#if SHIZU_FLASH_DRIFT_PROBE
  // ★core1 が走り出してから測る。停止中だと flash_safe_execute は
  //   「他コアを気にしなくてよい」経路に落ちて判定が変わる。
  sleep_ms(200);
  shizu::flash_clock_safe_exec_probe();
#if SHIZU_USB_DRIVER
  shizu::shizuku_usb_emergency_drain();
#endif
#endif
  shizu::init();
  while (1) {
    sleep_ms(500);
    printf("no_return\n");
  }
  return 0;
}
