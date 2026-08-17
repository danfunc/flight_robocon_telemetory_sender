#!/usr/bin/env python3
"""main.bin から [XIP] baseline checksum を再計算する。

    python3 tools/xip_bench_check.py build_xipbench/main.bin

ファームが印字する `[XIP] baseline checksum=XXXXXXXX` と一致することを確認する。
一致すれば「基準値そのものが正しい (= 基準を測ったブート時タイミングでも
読み違えていない)」ことまで言える。ファーム内の比較だけだと、基準自体が壊れて
いた場合に全設定が仲良く同じ壊れ方をして「全部 ok」に見えてしまう。

アルゴリズムは xip_bench.cpp の rot_sum と同じ:
    s = 0xffffffff;  各 32bit LE ワード w について  s = rol(s,1) ^ w
"""
import sys

REGION_BYTES = 256 * 1024  # xip_bench.cpp の REGION_BYTES と揃えること


def rot_sum(buf):
    s = 0xFFFFFFFF
    for i in range(0, len(buf), 4):
        w = int.from_bytes(buf[i:i + 4], "little")
        s = (((s >> 31) | (s << 1)) & 0xFFFFFFFF) ^ w
    return s


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    with open(sys.argv[1], "rb") as f:
        buf = f.read()
    if len(buf) < REGION_BYTES:
        print(f"main.bin が {len(buf)} B しかない ({REGION_BYTES} B 必要)。"
              " xip_bench.cpp の REGION_BYTES を下げるか、-DSHIZU_XIP_BENCH=ON で"
              " ビルドしたイメージを渡すこと。")
        return 2
    print(f"[XIP] baseline checksum={rot_sum(buf[:REGION_BYTES]):08x}"
          f"  ({REGION_BYTES} bytes of {sys.argv[1]}, image={len(buf)} B)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
