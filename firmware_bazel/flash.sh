#!/bin/bash
# ===========================================================================
#  Pico 2 W への書き込み — 使える手を自動で選ぶ
# ===========================================================================
#  優先順: BOOTSEL → OTW (有線) → OTA (無線)。上から試して、駄目なら次へ。
#
#  ★なぜこの順番か:
#    1. **BOOTSEL** — commit 段階が無いので**半端に壊れない**。転送が途中で
#       死んでも、もう一度 BOOTSEL に入れれば必ず戻せる。いちばん安全。
#       ★ただし「既に BOOTSEL に入っている板」しか対象にしない。動いている
#         飛行制御を勝手に BOOTSEL へ叩き落とすのは、書き込み手段の選択が
#         勝手にやってよい操作ではない。だから -f (強制再起動) は使わない。
#    2. **OTW** — 有線。BLE の 1 本しかない接続を占有しないので GDB と喧嘩しない。
#    3. **OTA** — 無線。機体が組み上がって USB に手が届かなくても届く。
#
#  ★★経路を移る前に**状態を整える**のもこの層の仕事。前の試行の残骸が
#    受信側に残っていると、次の手の先頭バイトがそれに食われる
#    (PING が FPING として届いた実績あり)。
set -uo pipefail

ELF="${1:?usage: flash.sh <path-to-elf>}"
PYTHON="${PYTHON:-$(command -v python3)}"
PICOTOOL="${PICOTOOL:-}"
if [ -z "$PICOTOOL" ]; then
  for c in /opt/picotool/bin/picotool "${HOME:-}/.pico-sdk/picotool/bin/picotool" \
           /opt/homebrew/bin/picotool "$(command -v picotool 2>/dev/null || true)"; do
    [ -n "$c" ] && [ -x "$c" ] && PICOTOOL="$c" && break
  done
fi

say() { echo "[flash] $*"; }

# XIAO へ 1 行送る。見つからなければ黙って何もしない。
# ★シェルだけで書く (python を挟まない)。ここは「経路を整える」ためのつまみで
#   あって、依存を増やす場所ではない。
xiao_send() {
  local port
  port="$(ls /dev/cu.usbmodem* 2>/dev/null | head -1)"
  [ -n "$port" ] || return 0
  stty -f "$port" 115200 raw -echo >/dev/null 2>&1 || return 0
  { printf '\n%s\n' "$1" > "$port"; } >/dev/null 2>&1 || true
  sleep 1
}

# ★書き込み前に安全装置を黙らせる。定期ハートビートに Pico が答えると、
#   その返事が BLE と UART を埋めて **どちらの経路も通らなくなる**
#   (2026-09-01 に実際に両方塞がった)。相手のファームが古いときほど効く。
quiet_on()  { xiao_send "QUIET ON"; }
quiet_off() { xiao_send "QUIET OFF"; }
reset_pico() { say "Pico を再起動して状態を作り直す"; xiao_send "reboot"; sleep 6; }

# ---- 1) BOOTSEL -----------------------------------------------------------
if [ -n "$PICOTOOL" ] && "$PICOTOOL" info >/dev/null 2>&1; then
  say "BOOTSEL のデバイスを見つけた → picotool で焼く"
  if "$PICOTOOL" load -x -v -u "$ELF" -t elf; then
    say "BOOTSEL 経由で完了"
    exit 0
  fi
  say "BOOTSEL での書き込みに失敗。次の手へ"
fi

# ---- 2) OTW (安全装置の XIAO 経由、有線) -----------------------------------
if ls /dev/cu.usbmodem* >/dev/null 2>&1; then
  say "XIAO の USB CDC を見つけた → OTW (有線) を試す"
  quiet_on
  if "$PYTHON" tools/otw_via_safety.py "$ELF" --commit; then
    quiet_off
    say "OTW 経由で完了"
    exit 0
  fi
  say "OTW に失敗。次の手へ (転送段階の失敗なら本体は無傷)"
  reset_pico
else
  say "XIAO の USB CDC が見えない → OTW は使えない"
fi

# ---- 3) OTA (BLE, 無線) ---------------------------------------------------
say "OTA (BLE) を試す"
quiet_on
if "$PYTHON" tools/ota_send.py "$ELF" --commit; then
  quiet_off
  say "OTA 経由で完了"
  exit 0
fi
quiet_off

say "すべての経路で失敗した。"
say "  - BOOTSEL: 板を BOOTSEL に入れて再実行するのが最も確実"
say "  - OTW    : XIAO が USB に見えているか / Pico と UART が繋がっているか"
say "  - OTA    : BLE で 'Shizuku UART' が見えているか (GDB ブリッジが接続を"
say "             掴んでいると 'device not found' になる)"
exit 1
