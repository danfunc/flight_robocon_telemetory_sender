#pragma once
// XIP (QSPI フラッシュ) 速度スイープ。cmake -DSHIZU_XIP_BENCH=ON のときだけ
// ビルドされ、main() から Shizuku 起動前に 1 度だけ呼ばれる。詳細は xip_bench.cpp。
namespace shizu {
void xip_bench_run();
}
