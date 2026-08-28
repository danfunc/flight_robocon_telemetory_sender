#!/bin/bash
set -euo pipefail

PYTHON="${PYTHON:-/opt/homebrew/bin/python3}"
if [ ! -x "$PYTHON" ]; then
  PYTHON="$(which python3)"
fi

exec "$PYTHON" tools/gdb_ble_bridge.py "$@"
