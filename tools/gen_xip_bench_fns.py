#!/usr/bin/env python3
"""xip_bench_fns.inc を生成する (リポジトリ直下へ出力)。

XIP (フラッシュ実行) の命令フェッチ・ストールを測るには、**XIP キャッシュ (RP2350 =
16KB / 8B ライン) に収まらない量のコード**を散らして、ランダム順に呼ぶ必要がある。
小さいループはキャッシュに載ってしまい、フラッシュ速度を変えても差が出ない
(2026-08-17 の probe がまさにそれだった)。

    python3 tools/gen_xip_bench_fns.py     # → xip_bench_fns.inc
"""
import os

N = 512
ALIGN = 128  # N * ALIGN = 64KB のスパン = キャッシュ 16KB の 4 倍

out = ["// 自動生成 (tools/gen_xip_bench_fns.py)。手で編集しないこと。\n",
       "// %d 個の関数を %dB 境界へ並べ、.text 上に %dKB の的を作る。\n"
       % (N, ALIGN, N * ALIGN // 1024),
       "// XIP キャッシュ (16KB) より十分大きいので、ランダム順に呼べば\n",
       "// ほぼ毎回ラインフィル = 命令フェッチのストールを測れる。\n\n",
       "#define XIP_BENCH_NFN %d\n" % N,
       "#define XIP_BENCH_FN_ALIGN %d\n\n" % ALIGN]
for i in range(N):
    out.append("static uint32_t __attribute__((noinline, aligned(%d)))\n"
               "xf_%d(uint32_t x) { return (x ^ %uu) * 2654435761u + %uu; }\n"
               % (ALIGN, i, (i * 2654435761) & 0xffffffff, i + 1))
out.append("\nstatic xf_t const xf_flash_table[XIP_BENCH_NFN] = {\n")
for i in range(0, N, 8):
    out.append("    " + ", ".join("xf_%d" % j for j in range(i, min(i + 8, N))) + ",\n")
out.append("};\n")

dst = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "xip_bench_fns.inc")
with open(dst, "w") as f:
    f.write("".join(out))
print("wrote", os.path.normpath(dst))
