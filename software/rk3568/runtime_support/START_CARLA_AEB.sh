#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$0")"
PY=/home/linaro/yolov8_deploy/.venv-offline/bin/python
"$PY" -B check_start.py
set -o pipefail
sudo env PYTHONDONTWRITEBYTECODE=1 BOARD_DEP_DIR="$PWD" YOLO_DIR="$PWD" \
 "$PY" -u run_carla_aeb.py --source carla --carla_url http://192.168.2.10:8081/stream \
 --carla_decode_backend mpp_rga_cached --pc_ip 192.168.2.10 \
 --model "$PWD/Models/A_rk3568_int8.rknn" \
 --classes "$PWD/Models/classes_11.txt" --size 640 \
 --lane_model "$PWD/Models/R15_256x448_patched_native_int8.rknn" \
 --p2_mpp_dir "$PWD/Mpp_Decoder" --preflight_video "$PWD/Selfcheck_Video/video_20260929_112507.mp4" \
 --lane_height 256 --lane_width 448 --lane_input_mode uint8_nhwc \
 --infer_every 1 --lane_hz 30 --q_depth 1 --quality 85 --port 18084 \
 --runtime_backend native_buffered --cached_display --publication_callback --lane_argmax planar \
 --run_seconds 0 --warmup_seconds 10 2>&1 | tee ./latest_run.log
