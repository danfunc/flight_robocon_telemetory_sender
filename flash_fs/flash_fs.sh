#!/bin/bash
set -euo pipefail

PICOTOOL="${PICOTOOL:-/opt/picotool/bin/picotool}"
if [ ! -x "$PICOTOOL" ]; then
  PICOTOOL="$(which picotool 2>/dev/null || true)"
fi

if [ -z "$PICOTOOL" ] || [ ! -x "$PICOTOOL" ]; then
  echo "Error: picotool not found" >&2
  exit 1
fi

UF2_PATH="$1"

echo "=== Flashing Flash FS ($UF2_PATH) to device at 0x00200000 (kernel untouched) ==="
"$PICOTOOL" load -f "$UF2_PATH"
echo "=== Flash FS update complete! ==="
