#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
PY="$PWD/Python_Environment/venv/bin/python"
if [ ! -s Python_Environment/ENVIRONMENT_READY.json ] || [ ! -x "$PY" ]; then
    echo 'Run bash ./PREPARE_PYTHON_ENVIRONMENT.sh first.' >&2
    exit 1
fi
if [ "$(id -u)" -eq 0 ]; then
    exec "$PY" -B ./launch_offline.py "$@"
else
    exec sudo env PYTHONDONTWRITEBYTECODE=1 "$PY" -B ./launch_offline.py "$@"
fi
