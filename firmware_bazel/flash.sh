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

echo "=== Flashing $UF2_PATH to device via USB (picotool) ==="
"$PICOTOOL" load -f "$UF2_PATH"
echo "=== Flash complete! ==="
