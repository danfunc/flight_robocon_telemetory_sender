#!/bin/bash
# ===========================================================================
#  OTW (有線) だけで焼く — 経路を落とさない
# ===========================================================================
#  ★★`:flash` (自動選択) と違い、**失敗しても BLE へ落ちない**。
#    これは省略のためではなく安全のため: 2026-09-02 に、OTW が失敗した直後に
#    自動で Pico を REBOOT して BLE OTA を始めた結果、転送が stall して
#    **BLE 広告停止 + UART PING 無応答**の状態に陥り、物理的な電源再投入
#    でしか戻せなくなった。経路を勝手に乗り換えるのは、うまくいけば便利だが、
#    失敗している最中の板にもう一手加える操作でもある。**どの経路で焼くかを
#    人が決めたいときのための口**がこれ。
#
#  ★この経路は **picotool を要らない** ところに価値がある。picotool だけは
#    CMake の島 (libusb にリンクする) で Bazel から素直に持てないが、OTW は
#    Python と USB CDC だけで完結するので、ビルドから書き込みまでを
#    Bazel の中で閉じられる。BOOTSEL は板に手が届いている状況でしか使わない
#    ので、そこだけホストの picotool に頼るのは釣り合っている。
#
#  使い方:
#    bazel run //firmware_bazel:otw                 # 焼いて本体へ移す
#    bazel run //firmware_bazel:otw -- --baud=310000
set -uo pipefail

ELF="${1:?usage: otw.sh <path-to-elf> [options]}"
shift

PYTHON="${PYTHON:-$(command -v python3)}"
if [ -z "$PYTHON" ]; then
  echo "[otw] python3 が見つかりません" >&2
  exit 1
fi

echo "[otw] $ELF を有線 (XIAO 経由) で焼く"
if "$PYTHON" tools/otw_via_safety.py "$ELF" --commit "$@"; then
  echo "[otw] 完了"
  exit 0
fi

# ★★ここで BLE へ落ちない。次に何をするかは人が決める。
echo "[otw] 失敗した。**自動では次の手を打たない**"
echo "[otw]   - 板が生きているか先に確かめること (XIAO へ 'SEND PING')"
echo "[otw]   - 転送段階の失敗なら本体は無傷 (commit まで到達していない)"
echo "[otw]   - BLE で焼くなら: bazel run //firmware_bazel:ota"
echo "[otw]   - BOOTSEL なら  : 板を BOOTSEL に入れて bazel run //firmware_bazel:flash"
exit 1
