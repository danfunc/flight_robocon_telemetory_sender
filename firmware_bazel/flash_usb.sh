#!/bin/bash
# ===========================================================================
#  卓上用: USB (BOOTSEL) で最短で焼く
# ===========================================================================
#  ★★`:flash` (自動選択) と違い、**動いている Pico を問答無用で BOOTSEL へ
#    落として焼く** (picotool -f)。`:flash` が -f を使わないのは「動作中の
#    飛行制御を勝手に落とす操作を自動化しない」ためで、**機体に載っている
#    ときは正しい**。だが卓上で自動操縦を開発している最中はその配慮が
#    足を引っ張るだけなので、口を分ける。
#
#  ★2026-09-02 実測: BOOTSEL 経由は**約 8 秒**で、この日 一度も失敗しなかった。
#    OTW は通っても 16〜30 秒で、同じ日に何度も転んでいる。**卓上で USB が
#    繋がっているなら、これが最速かつ最確実。** OTW/OTA が存在する理由は
#    「機体に組み上がって USB に手が届かない」ときのためであって、速さではない。
#
#  ★飛行機に載せた状態では使わないこと。名前を分けてあるのはそのため。
set -uo pipefail

ELF="${1:?usage: flash_usb.sh <path-to-elf>}"
# bazel run の runfiles からでも絶対パスで渡せるように解決する
case "$ELF" in
  /*) ;;
  *) ELF="$PWD/$ELF" ;;
esac

PICOTOOL="${PICOTOOL:-}"
if [ -z "$PICOTOOL" ]; then
  for c in /opt/picotool/bin/picotool "${HOME:-}/.pico-sdk/picotool/bin/picotool" \
           /opt/homebrew/bin/picotool "$(command -v picotool 2>/dev/null || true)"; do
    [ -n "$c" ] && [ -x "$c" ] && PICOTOOL="$c" && break
  done
fi
if [ -z "$PICOTOOL" ]; then
  echo "[flash_usb] picotool が見つかりません" >&2
  exit 1
fi
echo "[flash_usb] picotool: $PICOTOOL ($("$PICOTOOL" version 2>/dev/null | head -1))"

# ★★対象を名指しする。安全装置の XIAO (RP2040) も USB に居るので、
#   指定しないと picotool が「単一のデバイスを狙え」と言って止まる。
#   RP2350 を型で選ぶ。
TARGET_ARGS=""
if "$PICOTOOL" info 2>&1 | grep -q "RP2350 device at bus \([0-9]*\), address \([0-9]*\)"; then
  BUS=$("$PICOTOOL" info 2>&1 | sed -n 's/.*RP2350 device at bus \([0-9]*\), address \([0-9]*\).*/\1/p' | head -1)
  ADDR=$("$PICOTOOL" info 2>&1 | sed -n 's/.*RP2350 device at bus \([0-9]*\), address \([0-9]*\).*/\2/p' | head -1)
  TARGET_ARGS="--bus $BUS --address $ADDR"
  echo "[flash_usb] RP2350 を bus $BUS address $ADDR で狙う"
fi

# shellcheck disable=SC2086
if "$PICOTOOL" load -x -v -u "$ELF" -t elf -f $TARGET_ARGS; then
  echo "[flash_usb] 完了"
  exit 0
fi
echo "[flash_usb] 失敗した。"
echo "[flash_usb]   - Pico の USB が母艦に繋がっているか"
echo "[flash_usb]   - 既に BOOTSEL に居るなら -f は要らない (picotool info で確認)"
echo "[flash_usb]   - 機体に載っていて USB に手が届かないなら //firmware_bazel:otw か :ota"
exit 1
