#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$0")"
PY="$PWD/Python_Environment/venv/bin/python"
[[ -x "$PY" && -s "$PWD/Python_Environment/ENVIRONMENT_READY.json" ]] || { printf '先执行 bash ./PREPARE_PYTHON_ENVIRONMENT.sh，保留原 Python 环境直到备份成功。\n' >&2; exit 2; }
exec "$PY" -B ./launch_decision.py
