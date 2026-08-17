#pragma once
#include <cstdint>

// XIP (QSPI フラッシュ) のクロックを **clk_sys から導出して** 設定する。
//
// なぜ要るのか
// ------------
// SDK 2.2 / RP2350 の既定ビルドには **boot2 が入っていない** (リンカスクリプトの
// `.boot2` は optional で、実際 main.elf に存在しない)。つまり
// `PICO_FLASH_SPI_CLKDIV` / `PICO_FLASH_SPI_RXDELAY` は**効かない死んだつまみ**で、
// QMI の M0_TIMING はブート ROM が設定した値 (実測 div=3 / rx=2 固定) のまま走る。
//
// 分周比が固定ということは **フラッシュ速度が clk_sys に比例して動く**:
//   clk_sys 150MHz → フラッシュ 50MHz  (既定ビルドはずっとこれだった)
//   clk_sys 300MHz → フラッシュ 100MHz (OC ビルドは偶然ここに居ただけ)
// 前者は遅すぎ、後者は「意図して選んだ値」ではない。どちらも黙って決まっている。
//
// だから分周比を固定値で書くのではなく、**目標周波数を宣言して clk_sys から
// 分周比を導出する**。clk_sys を変えても意味が変わらない唯一の書き方。
namespace shizu {

// clk_sys=sys_hz のときに target_hz を**超えない**最小の分周比を選んで書き込む。
// 戻り値は実際のフラッシュ周波数 [Hz]。RAM 常駐 (XIP を止めずに書き換えるため)。
// ★clk_sys を上げる前に「上げた後の sys_hz」でこれを呼んでおくこと。分周比を先に
//   上げておけば、クロックが上がった瞬間にフラッシュが定格超過になるのを防げる。
uint32_t flash_clock_apply_for(uint32_t sys_hz, uint32_t target_hz,
                               uint32_t rxdelay);
// 現在の clk_sys を使う版。
uint32_t flash_clock_apply(uint32_t target_hz, uint32_t rxdelay);

// 現在の M0_TIMING から読み戻したフラッシュ周波数 [Hz] (0 = 分周比 0 = 異常)。
uint32_t flash_clock_hz();

// RXDELAY を「遅延の**物理量** (ps)」から導出する。
// ★RXDELAY のレジスタ単位は clk_sys の**半サイクル**なので、固定値を書くと
//   「そのクロックのときだけ正しい値」になる — フラッシュ分周・CYW43 分周と
//   まったく同じ罠。同じ往復遅延を保つには clk_sys に比例した段数が必要:
//     300MHz: 半サイクル 1.67ns → 5ns = 3 段
//     150MHz: 半サイクル 3.33ns → 5ns = 2 段 (ここで 3 を書くと窓 0..3 の上端になる)
// 実測した窓 (flash_clock.cpp 冒頭) を ns に直すと、上限はどのクロックでも ~11ns、
// 下限だけがフラッシュ速度に応じて上がる (100MHz で ≥1.7ns、150MHz で ≥3.3ns)。
// 5ns はその全ケースで窓の内側に入る。
uint32_t flash_rxdelay_for(uint32_t sys_hz, uint32_t delay_ps);

// 最後に flash_clock_apply* が書いた値から M0_TIMING がずれていたら true。
// ★フラッシュ書き込み (BTstack のペアリング鍵保存など) を行うと、SDK は
//   ROM の flash_enter_cmd_xip() で XIP を張り直すため **ROM 既定 (div=3 rx=2) へ
//   戻る**。壊れはしないが黙って遅くなる。ビーコンがこれを見て報告する。
//   直すなら再設定は flash_safe_execute の中 (core1 が止まっている場所) で行うこと
//   — 他コアが XIP 転送中に M0_TIMING を書き換えるのは安全でない。
bool flash_clock_drifted();

#if SHIZU_FLASH_DRIFT_PROBE
// 上の「フラッシュ操作後に ROM 既定へ戻る」を実測で確かめる (cmake
// -DSHIZU_FLASH_DRIFT_PROBE=ON)。書き込みは行わない — flash_do_cmd が
// flash_range_erase と同じ XIP 再入経路を通るため。JEDEC ID も印字する。
void flash_clock_drift_probe();
// BTstack の書き込み経路 (flash_safe_execute) が実際に到達するかを実測する。
// ★core1 が動き出した後に呼ぶこと。
void flash_clock_safe_exec_probe();
#endif

} // namespace shizu
