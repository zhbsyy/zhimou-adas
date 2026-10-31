#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
carla_source.py —— RK3568 板端 CARLA HTTP MJPEG 视频流拉取器
===========================================================
对外接口形态与 PcieFrameSource 保持 100% 兼容：
  - start()
  - read_bgr() -> np.ndarray (720, 1280, 3) uint8 BGR
  - close()

从 PC 端 HTTP MJPEG 视频流（如 http://192.168.10.1:8081/stream）持续拉取最新帧，
具备后台线程拉帧、粘包/分包重组、断线自动重连与帧率统计功能。
"""

import os
import sys
import threading
import time
import urllib.request
from typing import Optional

import cv2
import numpy as np

WIDTH, HEIGHT = 1280, 720


class CarlaFrameSource:
    """拉取 PC 端 CARLA 四宫格 HTTP MJPEG 视频流并提供 BGR 图像接口。"""

    def __init__(self, stream_url: str = "http://192.168.10.1:8081/stream",
                 width: int = WIDTH, height: int = HEIGHT, timeout: float = 3.0):
        self.stream_url = stream_url
        self.width = width
        self.height = height
        self.timeout = timeout

        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._frame_ready = threading.Condition(self._lock)

        self._latest_bgr: Optional[np.ndarray] = None
        self._frame_count = 0
        self.fps = 0.0
        self.connected = False
        self.last_frame_time = 0.0

    @property
    def mode(self) -> str:
        return "carla_http_mjpeg"

    def start(self) -> None:
        """启动后台拉流线程。"""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def _worker(self) -> None:
        """后台持续拉取 HTTP MJPEG 流，自动处理断线重连。"""
        while self._running:
            resp = None
            try:
                req = urllib.request.Request(self.stream_url, headers={"User-Agent": "RK3568_ADAS/1.0"})
                resp = urllib.request.urlopen(req, timeout=self.timeout)
                self.connected = True
                stream_bytes = b""
                t_last = time.time()

                while self._running:
                    chunk = resp.read(8192)
                    if not chunk:
                        break
                    stream_bytes += chunk

                    # 查找 JPEG 帧头 (0xFF 0xD8) 和帧尾 (0xFF 0xD9)
                    start = stream_bytes.find(b"\xff\xd8")
                    end = stream_bytes.find(b"\xff\xd9", start + 2) if start != -1 else -1

                    if start != -1 and end != -1:
                        jpg_data = stream_bytes[start:end + 2]
                        stream_bytes = stream_bytes[end + 2:]

                        # 解码 JPEG 为 BGR 图像
                        arr = np.frombuffer(jpg_data, dtype=np.uint8)
                        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)

                        if frame is not None:
                            if frame.shape[0] != self.height or frame.shape[1] != self.width:
                                frame = cv2.resize(frame, (self.width, self.height))

                            now = time.time()
                            span = now - t_last
                            t_last = now
                            if span > 0:
                                self.fps = 0.9 * self.fps + 0.1 / span

                            with self._frame_ready:
                                self._latest_bgr = frame
                                self._frame_count += 1
                                self.last_frame_time = now
                                self._frame_ready.notify_all()

            except Exception:
                self.connected = False
                time.sleep(1.0)  # 断线后休眠 1s 优雅重连
            finally:
                if resp is not None:
                    try:
                        resp.close()
                    except Exception:
                        pass

    def read_bgr(self) -> np.ndarray:
        """读取最新的一帧 BGR 图像 (720, 1280, 3)。若暂无帧则返回黑色默认帧。"""
        with self._lock:
            if self._latest_bgr is not None:
                return self._latest_bgr.copy()

        # 兜底默认帧
        blank = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        cv2.putText(blank, "WAITING FOR CARLA STREAM...", (self.width // 4, self.height // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)
        return blank

    def read_new_bgr(self, last_sequence: int, timeout: float = 0.2):
        """等待新解码帧，返回 (BGR, 序号, 解码完成时刻)；超时返回空帧。"""
        deadline = time.monotonic() + max(0.0, timeout)
        with self._frame_ready:
            while self._running and self._frame_count <= last_sequence:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._frame_ready.wait(remaining)
            if self._frame_count <= last_sequence or self._latest_bgr is None:
                return None, last_sequence, 0.0
            return self._latest_bgr.copy(), self._frame_count, self.last_frame_time

    def close(self) -> None:
        """停止拉流。"""
        self._running = False
        with self._frame_ready:
            self._frame_ready.notify_all()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self._thread = None


if __name__ == "__main__":
    url = sys.argv[1] if len(sys.argv) > 1 else "http://127.0.0.1:8081/stream"
    src = CarlaFrameSource(url)
    src.start()
    print(f"[CarlaSource] 尝试连接: {url}")
    try:
        for _ in range(5):
            time.sleep(0.5)
            f = src.read_bgr()
            print(f"帧规格: {f.shape}, 帧率: {src.fps:.1f}, 已连上: {src.connected}")
    finally:
        src.close()
