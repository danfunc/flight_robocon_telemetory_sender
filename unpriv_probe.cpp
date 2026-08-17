// ===========================================================================
//  unpriv_probe — 非特権実行 (CONTROL.nPRIV=1) の最小プローブ
// ===========================================================================
//  目的: 「特権遷移機構が正しいか」だけを、他の全要因から切り離して確かめる。
//
//  ★背景 (2026-08-17): MPU Step1 は 2026-07-20 に FLIGHT_CONTROLLER を非特権化して
//    実機 HardFault で失敗した。しかしあれは**一番複雑なオブジェクトで機構を試した**
//    ため、「機構が壊れている」のか「そのオブジェクトが非特権に耐えない」のかを
//    区別できていなかった (svc 委譲を 6 段ネストでいきなり試して原因を絞れなかったのと
//    同じ方法論の誤り)。
//
//  ★MPU 設定を読むと、後者である可能性が高い:
//      region0 = XIP flash 全窓, AP=0b11 (RO 全特権レベル), X   → 非特権でも実行可
//      region1 = __end__〜SRAM 終端, AP=0b01 (RW 全特権レベル), XN → 非特権でも RW 可
//      region 外は PRIVDEFENA=1 により **特権のみ**
//    そして `.bss` は丸ごと __end__ の**下** (実測: __bss_end__ == __end__ == 0x200390ac)。
//    → **非特権オブジェクトが自分のグローバル/static に触った瞬間にフォールトする。**
//      ペリフェラル (GPIO/I2C/UART/USB)、SIO (get_core_num の CPUID)、タイマ
//      (time_us_64) も region 外なので同様。
//
//  したがってこのプローブは「region1 (ヒープ) だけで完結する」ように作る:
//    ・グローバル/static を**一切持たない**。状態は malloc したブロックへの
//      ポインタを生成時引数 (r0) で受け取り、それ経由でのみ触る
//    ・標準ライブラリを一切呼ばない (printf / time_us_64 / get_core_num すべて禁止)
//    ・yield は **生の svc 命令**で撃つ。SVC 命令自体は非特権でも実行できる —
//      むしろこれが特権遷移機構の検証そのもの
//    ・スレッドスタックは malloc 産 = region1 なので非特権 RW で触れる
//    ・コードは flash = region0 なので非特権で実行できる
//
//  → **MPU の設定を一切変えずに、特権遷移機構だけを切り出して検証できる。**
//     通れば「機構は正しく、失敗の原因は stdlib とグローバル」が確定する。
//     通らなければ問題は本当に CONTROL.nPRIV の遷移側にあり、探索範囲が桁で狭まる。
//
//  ★実オブジェクトを非特権化する条件は別途ある: ドライバの private global を
//    `.bss` から**オブジェクトごとの arena (ヒープ上)** へ移すこと。arena は region1 に
//    入るので自動的に非特権アクセス可能になる。docs/SHIZUKU_DESIGN.md §11.2 参照。
// ===========================================================================
#include <cstdint>
#include <kernel.hpp>

#if SHIZU_UNPRIV_PROBE

#include <cstdlib>
#include <hardware/exception.h>
#include <log.hpp>
#include <obj_api.hpp>
#include <object_id.hpp>
#include <shizu.hpp>

namespace shizu {
namespace {

// ---- MemManage フォールトの仮ハンドラ ---------------------------------------
// 非特権が region 外 (特権のみ) に触ったときの「仮の」処理。
//   ・違反を数え、最後のアドレスを記録する
//   ・**フォールトした命令をスキップして続行する** (PC を命令長ぶん進める)
// ★制限: ストアなら「書き込みが落ちる」= 実質 0 埋めと同じ意味論になるが、
//   **ロードでは行き先レジスタが元の値のまま残る** (0 にはならない)。
//   ロードを 0 埋めするには命令デコードして Rt を特定する必要があり、それは
//   本物の実装の仕事。ここは「落ちずに観測を続ける」ための足場。
volatile uint32_t g_mm_faults = 0;   // .bss (特権のみ触れる。ハンドラは特権なので可)
volatile uint32_t g_mm_last_addr = 0;
volatile uint32_t g_mm_last_pc = 0;

constexpr uint32_t SCB_CFSR  = 0xE000ED28u; // 下位バイト = MMFSR
constexpr uint32_t SCB_MMFAR = 0xE000ED34u;
constexpr uint32_t SCB_SHCSR = 0xE000ED24u;
constexpr uint32_t MEMFAULTENA = 1u << 16;

extern "C" void shizu_memmanage_handler() {
  uint32_t psp;
  asm volatile("mrs %0, psp" : "=r"(psp));
  uint32_t *f = (uint32_t *)psp; // r0,r1,r2,r3,r12,lr,pc,xPSR
  const uint32_t pc = f[6];
  volatile uint32_t *cfsr = (volatile uint32_t *)SCB_CFSR;
  const uint32_t mmfsr = *cfsr & 0xFFu;
  g_mm_last_addr = (mmfsr & (1u << 7)) // MMARVALID
                       ? *(volatile uint32_t *)SCB_MMFAR
                       : 0xFFFFFFFFu;
  g_mm_last_pc = pc;
  g_mm_faults = g_mm_faults + 1;
  *cfsr = mmfsr; // write-1-to-clear
  // Thumb-2: 上位 5bit が 0b11101/11110/11111 なら 32bit 命令。
  const uint16_t hw = *(volatile uint16_t *)pc;
  f[6] = pc + (((hw >> 11) >= 0x1Du) ? 4u : 2u);
}

// ---- 非特権で走る本体 -------------------------------------------------------
// ★この関数の中で触ってよいのは「引数で渡されたポインタの先」と「自分のスタック」
//   だけ。グローバル参照・関数呼び出し (stdlib) を足した瞬間に意味が変わる。
// 引数 st: [0]=ticks [1]=started [2]=違反を試す対象アドレス (0 なら試さない)
[[noreturn]] void probe_main(uint32_t state_ptr) {
  volatile uint32_t *st = (volatile uint32_t *)state_ptr;
  // ★自分が本当に非特権で走っているかを自己申告させる。MRS CONTROL は非特権でも
  //   読める (書き込みだけが無視される)。bit0 = nPRIV。
  //   これが 0 なら「非特権で動いた」という観測は全て無意味になる。
  uint32_t ctrl;
  asm volatile("mrs %0, control" : "=r"(ctrl));
  st[3] = ctrl;
  st[1] = 1; // 到達した印
  uint32_t round = 0;
  while (true) {
    // 固定量の計算 + スタックアクセス (MPU チェックは load/store にかかるので、
    // メモリを触る仕事を入れないと特権/非特権の差が測れない)。
    volatile uint32_t work[8];
    uint32_t acc = st[0];
    for (uint32_t i = 0; i < 64; ++i) {
      work[i & 7u] = acc + i;
      acc += work[(i + 3u) & 7u];
    }
    st[0] = st[0] + 1;
    // 一定周期で「特権のみの領域」へストアしてみる (仮ハンドラの動作確認)。
    // 書き込みが落ちるだけで続行できるはず。
    if (st[2] != 0 && (++round & 0x3FFu) == 0)
      *(volatile uint32_t *)st[2] = 0xDEADBEEFu;
    asm volatile("movs r0, #0\n"
                 "movs r1, #0\n"
                 "movs r2, #0\n"
                 "movs r3, #0\n"
                 "svc  0\n" ::: "r0", "r1", "r2", "r3", "memory");
  }
}

// ---- 特権のまま走る観測側 ---------------------------------------------------
// [0..2]=非特権プローブ  [4..6]=特権プローブ (同一コード・同一負荷)
[[noreturn]] void reporter_main(uint32_t state_ptr) {
  volatile uint32_t *st = (volatile uint32_t *)state_ptr;
  uint32_t lu = 0, lp = 0, rounds = 0;
  while (true) {
    obj_api::yield_us(2000000);
    const uint32_t u = st[0], p = st[4];
    const uint32_t du = u - lu, dp = p - lp;
    ++rounds;
    log::printf("[UNPRIV] r=%lu unpriv=%lu(+%lu) priv=%lu(+%lu) ratio=%lu.%02lu%% "
                "CONTROL(u=%lx p=%lx) mmfault=%lu addr=%08lx pc=%08lx => %s\n",
                (unsigned long)rounds, (unsigned long)u, (unsigned long)du,
                (unsigned long)p, (unsigned long)dp,
                (unsigned long)(dp ? du * 100u / dp : 0u),
                (unsigned long)(dp ? (du * 10000u / dp) % 100u : 0u),
                (unsigned long)st[3], (unsigned long)st[7],
                (unsigned long)g_mm_faults, (unsigned long)g_mm_last_addr,
                (unsigned long)g_mm_last_pc,
                (st[1] && du) ? "UNPRIV RUNNING" : "STALLED/NOT ENTERED");
    lu = u; lp = p;
  }
}

} // namespace

void unpriv_probe_launch() {
  // 状態はヒープへ (region1 = 非特権 RW)。[0..2]=非特権 / [4..6]=特権
  volatile uint32_t *st = (volatile uint32_t *)malloc(8 * sizeof(uint32_t));
  if (st == nullptr)
    return;
  for (uint32_t i = 0; i < 8; ++i)
    st[i] = 0;
  // 非特権プローブに「特権のみの領域」のアドレスを渡す。
  // ★対象は **SIO の CPUID (0xd0000000)** を使う:
  //   ・region0 (XIP) にも region1 (SRAM ヒープ) にも**絶対に入らない** →
  //     PRIVDEFENA により非特権からは必ず MemManage になるはず
  //   ・**読み出し専用レジスタ**なので、万一フォールトせず通ってしまっても
  //     書き込みは無視される = 副作用ゼロ (他の動作に影響しない)
  //   ・`get_core_num()` が読むレジスタそのもので、現実的なケースでもある
  //   【前回の失敗】記録用変数 `g_mm_last_addr` 自身を対象にしてしまい、
  //   (i) 書き込みが通って記録が 0xDEADBEEF で潰れ、(ii) `.bss` 末尾は __end__ の
  //   32B 切り上げで region1 に入り込み得る、の 2 点で計測が壊れた。対象と記録先は
  //   必ず分けること。
  st[2] = 0xd0000000u;

  // MemManage を有効化 (無効だと HardFault へエスカレートしてハンドラが呼ばれない)。
  *(volatile uint32_t *)SCB_SHCSR |= MEMFAULTENA;
  exception_set_exclusive_handler(MEMMANAGE_EXCEPTION, shizu_memmanage_handler);

  // ★順序が重要: unprivileged フラグは create_thread より**前**に立てる。
  FOR_KERNEL_OBJECT::create_object((uint32_t)object_ids::UNPRIV_PROBE);
  // ★obj_api の svc を使ってはいけない。この関数は**カーネルオブジェクトとして**
  //   走るので、obj_api 番号 (24) は「発行元 = カーネルオブジェクト」判定でプリミティブ
  //   側の switch に入り、そこに 24 は無いので **default で黙って捨てられる**
  //   (実測 2026-08-17: これに気づかず CONTROL=2 = 特権のまま「非特権で動いた」と
  //   誤認した)。カーネルオブジェクトからは表を直接書く。
  object_table[(uint32_t)object_ids::UNPRIV_PROBE].unprivileged = true;
  FOR_KERNEL_OBJECT::async_call((uint32_t)object_ids::UNPRIV_PROBE,
                                (method_t)probe_main,
                                (uint32_t)(uintptr_t)&st[0]);

  // ★A/B: まったく同じコード・同じ負荷を**特権のまま**走らせる。同一実行・同一
  //   クロックでの比較なので、ビルドや温度の差が入らない。違反は試さない (st[6]=0)。
  FOR_KERNEL_OBJECT::create_object((uint32_t)object_ids::UNPRIV_PRIV_REF);
  FOR_KERNEL_OBJECT::async_call((uint32_t)object_ids::UNPRIV_PRIV_REF,
                                (method_t)probe_main,
                                (uint32_t)(uintptr_t)&st[4]);

  // 観測側は特権のまま (別オブジェクト)。
  FOR_KERNEL_OBJECT::create_object((uint32_t)object_ids::UNPRIV_REPORTER);
  FOR_KERNEL_OBJECT::async_call((uint32_t)object_ids::UNPRIV_REPORTER,
                                (method_t)reporter_main,
                                (uint32_t)(uintptr_t)&st[0]);
}

} // namespace shizu

#endif // SHIZU_UNPRIV_PROBE
