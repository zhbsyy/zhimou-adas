#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$0")/Mpp_Decoder"
g++ -std=c++17 -O2 -Wall -Wextra -I/usr/include/rga mpp_native_stage.cpp -o mpp_native_stage.new -lrga $(pkg-config --cflags --libs gstreamer-app-1.0 gstreamer-video-1.0 gstreamer-allocators-1.0)
chmod u+x mpp_native_stage.new
mv -f -- mpp_native_stage.new mpp_native_stage
echo 'MPP_DECODER_READY: local module only; original board decoder untouched'
