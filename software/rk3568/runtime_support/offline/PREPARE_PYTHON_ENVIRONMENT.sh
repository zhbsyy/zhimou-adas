#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
BOOTSTRAP_PY=${BOOTSTRAP_PY:-/usr/bin/python3}
exec "$BOOTSTRAP_PY" -B ./Python_Environment/prepare_environment.py "${1:-/home/linaro/yolov8_deploy/.venv-offline}"
