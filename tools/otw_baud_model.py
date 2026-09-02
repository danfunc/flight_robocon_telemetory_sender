#!/usr/bin/env python3
"""実測したチャンク破損率から、実効速度が最大になる baud を選ぶ。

★これは**順位付けの道具であって予測器ではない**。固定費 (ラウンドあたりの
  中継の張り直し) と flash の所要時間は実測値をそのまま入れてあるので、
  絶対値は当たらなくてもよいが、baud 間の**優劣の向き**は当たるはず。

使い方: 各 baud で 1 回ずつ転送し、device が出す
    NEED n=<k> (of 109, ok=.. bad=.. round=1)
の 1 ラウンド目の値を拾って、下の MEASURED へ書き写して実行する。

  python3 tools/otw_baud_model.py

★測り方の作法 (CLAUDE.md): baud を固めて回さず 1 巡ずつ交互に、1 回ごとに
  冷ます。中央値で判断し、σ/中央値 が 3% を超えたら冷却不足を疑う。
★制御が壊れる baud (NEED 行そのものが読めない) はその時点で失格。速くても
  「どれを再送すべきか」が分からなければ再送機構ごと成立しない。
"""

# ---- 実測で埋める欄 --------------------------------------------------------
# baud -> 1 ラウンド目で欠けたチャンク数 (of TOTAL_CHUNKS)。None = 未測定。
# ★2026-09-02 時点で分かっているのは「320000 以上は Pico 自身が返すテキスト
#   まで化ける」ということだけ。当時は再送が無かったので即死したが、
#   チャンク再送があるなら「多少化けても速い方が勝つ」帯があるかもしれない。
#   ただし**制御 (NEED 行) が読めるかは別問題**なので、そこは実測で見ること。
MEASURED = {
    115200: None,
    230400: None,
    300000: None,   # 2026-09-02 の既定。当時は「常に成功」
    345600: None,
    460800: None,
    921600: None,
    1000000: None,
}

# ---- 実測済みの定数 --------------------------------------------------------
IMAGE_BYTES = 445312        # 像 (展開後)
WIRE_BYTES = 302226         # XNOR で線に乗る量 (圧縮 + チャンクヘッダ 16B/4KB)
TOTAL_CHUNKS = 109
# 1 チャンクあたりの flash 所要 (2026-09-02 実測: erase 4284ms/109 blk、
# program 2286ms/109)。★再送されたチャンクは**消し直して書き直す**ので、
# 再送のたびにこの分が丸ごと乗る。
FLASH_MS_PER_CHUNK = (4284 + 2286) / TOTAL_CHUNKS
INFLATE_MS_TOTAL = 420
# ラウンドあたりの固定費: 中継を畳んで baud を戻し、OTANEED を叩いて
# 一覧を読み、もう一度 ubridge を張り直すまで。★このうち 3 秒は「2 回連続
# ブリッジのタイミング問題」への実用対処 (根本原因は未特定)。ここが
# 縮められれば攻めた baud が一気に有利になるので、**最も効く宿題**。
ROUND_OVERHEAD_S = 3.5


def estimate(baud: int, bad_first_round: int, max_rounds: int = 5):
    """(総時間 s, 実効速度 kB/s, ラウンド数) を返す。

    1 ラウンドで欠ける割合 p は baud に固有と見なし、再送でも同じ割合で
    欠けるとして幾何級数で積む (実測 1 点から外挿するので、ラウンドが
    多いほど当てにならない — 順位付けにだけ使うこと)。
    """
    p = bad_first_round / TOTAL_CHUNKS
    bytes_per_s = baud / 10.0        # 8N1 = 10 bit/byte
    remaining = TOTAL_CHUNKS
    total_s = 0.0
    written = 0
    rounds = 0
    while remaining > 0 and rounds < max_rounds:
        rounds += 1
        frac = remaining / TOTAL_CHUNKS
        total_s += (WIRE_BYTES * frac) / bytes_per_s
        got = remaining * (1.0 - p)
        written += got
        total_s += ROUND_OVERHEAD_S
        remaining = remaining - got
        if remaining < 0.5:
            remaining = 0
    total_s += written * FLASH_MS_PER_CHUNK / 1000.0
    total_s += INFLATE_MS_TOTAL / 1000.0
    return total_s, IMAGE_BYTES / total_s / 1024.0, rounds


def main() -> int:
    known = {b: v for b, v in MEASURED.items() if v is not None}
    if not known:
        print(__doc__)
        print("MEASURED がまだ空。実機で 1 ラウンド目の欠損数を採ってから実行する。")
        print("\n参考: 欠損率を仮に置いたときの見え方 (実測ではない):")
        for baud in (300000, 460800, 921600, 1000000):
            for bad in (0, 1, 5, 20):
                s, kb, r = estimate(baud, bad)
                print(f"  baud={baud:>7} 欠損={bad:>3}/109 -> "
                      f"{s:5.1f}s  {kb:5.1f} kB/s  rounds={r}")
        return 0
    print(f"{'baud':>8} {'欠損/109':>9} {'総時間':>8} {'実効':>10} {'rounds':>7}")
    best = None
    for baud, bad in sorted(known.items()):
        s, kb, r = estimate(baud, bad)
        print(f"{baud:>8} {bad:>9} {s:>7.1f}s {kb:>8.1f} kB/s {r:>7}")
        if best is None or kb > best[1]:
            best = (baud, kb)
    print(f"\n実効速度が最大なのは {best[0]} baud ({best[1]:.1f} kB/s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
