#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$0")"
PY="$PWD/Python_Environment/venv/bin/python"
[[ -x "$PY" && -s "$PWD/Python_Environment/ENVIRONMENT_READY.json" ]] || { echo '先执行 bash ./PREPARE_PYTHON_ENVIRONMENT.sh' >&2; exit 2; }
exec "$PY" -B ./launch_cameras.py --mode preview
