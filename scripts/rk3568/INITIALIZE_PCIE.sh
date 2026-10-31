#!/usr/bin/env bash
set -euo pipefail
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PROFILE=${1:-decision}
case "$PROFILE" in decision|four_camera) ;; *) echo 'Use decision or four_camera' >&2; exit 2;; esac
exec /usr/bin/python3 -B "$HERE/../project.py" rk --profile "$PROFILE" --action init-pcie
