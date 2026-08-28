#!/bin/bash
set -euo pipefail

# Find python3 with bleak/pyserial
PYTHON="${PYTHON:-/opt/homebrew/bin/python3}"
if [ ! -x "$PYTHON" ]; then
  PYTHON="$(which python3)"
fi

# Run ota_send.py with the target image and --commit
exec "$PYTHON" tools/ota_send.py "$@" --commit
