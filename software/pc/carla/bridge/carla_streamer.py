#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
carla_streamer.py —— CARLA 四向环视相机采集与 HTTP MJPEG 推流器
============================================================
负责在 CARLA 仿真车上挂载 4 路相机（前/右/后/左），拼接为 1280x720 的 2x2 四宫格，
并通过标准 HTTP MJPEG (端口 8081) 提供视频流服务供 RK3568 板端拉取推理。

四向相机几何参数沿用 ADAS_Camera_Domain 实测标定参数：
  - front: (2.05, 0.0, 1.75), yaw=0°
  - right: (0.0, 1.15, 1.65), yaw=90°
  - rear:  (-2.05, 0.0, 1.75), yaw=180°
  - left:  (0.0, -1.15, 1.65), yaw=-90°
每格 640x360, FOV 60°
"""

import argparse
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Dict, Optional

import cv2
import numpy as np


class CarlaMultiViewStreamer:
    """CARLA 四向环视相机挂载、拼图与 HTTP MJPEG 推流服务。"""

    def __init__(self, host: str = "127.0.0.1", port: int = 2000,
                 http_port: int = 8081, width: int = 1280, height: int = 720,
                 quality: int = 80) -> None:
        self.host = host
        self.port = port
        self.http_port = http_port
        self.width = width
        self.height = height
        self.quality = quality
        self.tile_w = width // 2   # 640
        self.tile_h = height // 2  # 360

        self.carla_client = None
        self.carla_cameras = []
        self._frames: Dict[str, np.ndarray] = {}
        self._lock = threading.Lock()
        self._running = False

        self._latest_jpeg: Optional[bytes] = None
        self._latest_bgr: Optional[np.ndarray] = None
        self._frame_seq = 0
        self._fps = 0.0
        self._last_tick = time.time()

        self._http_server: Optional[ThreadingHTTPServer] = None
        self._http_thread: Optional[threading.Thread] = None

    def _generate_test_pattern(self) -> np.ndarray:
        """无 CARLA 仿真环境时生成高对比度 4 象限测试图。"""
        canvas = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        now_str = time.strftime("%H:%M:%S") + f".{int(time.time()*1000)%1000:03d}"

        # 4象限基底色: TL(前-红灰) TR(右-绿灰) BL(后-蓝灰) BR(左-黄灰)
        canvas[0:self.tile_h, 0:self.tile_w] = (30, 30, 80)
        canvas[0:self.tile_h, self.tile_w:] = (30, 80, 30)
        canvas[self.tile_h:, 0:self.tile_w] = (80, 30, 30)
        canvas[self.tile_h:, self.tile_w:] = (30, 80, 80)

        # 分割线
        cv2.line(canvas, (self.tile_w, 0), (self.tile_w, self.height), (200, 200, 200), 2)
        cv2.line(canvas, (0, self.tile_h), (self.width, self.tile_h), (200, 200, 200), 2)

        views = [
            ("FRONT", 20, 40),
            ("RIGHT", self.tile_w + 20, 40),
            ("REAR", 20, self.tile_h + 40),
            ("LEFT", self.tile_w + 20, self.tile_h + 40),
        ]
        for name, tx, ty in views:
            cv2.putText(canvas, f"CARLA VIEW: {name}", (tx, ty),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(canvas, f"{self.tile_w}x{self.tile_h} FOV:60", (tx, ty + 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1, cv2.LINE_AA)
            cv2.putText(canvas, now_str, (tx, ty + 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1, cv2.LINE_AA)

        return canvas

    def attach_to_carla(self, carla_client=None) -> bool:
        """连接 CARLA 并为 ego_vehicle 挂载四向相机。"""
        try:
            import carla
        except ImportError:
            print("[CarlaStreamer]  当前环境未安装 carla 库，推流器将以测试图源运行。")
            return False

        try:
            if carla_client is None:
                self.carla_client = carla.Client(self.host, self.port)
                self.carla_client.set_timeout(4.0)
            else:
                self.carla_client = carla_client

            world = self.carla_client.get_world()
            vehicles = [a for a in world.get_actors().filter("vehicle.*")
                        if a.attributes.get("role_name") == "ego_vehicle"]
            if not vehicles:
                vehicles = list(world.get_actors().filter("vehicle.*"))
            if not vehicles:
                print(f"[CarlaStreamer]  CARLA [{self.host}:{self.port}] 未找到自车，等待自车生成。")
                return False

            ego = vehicles[0]
            views = (
                ("front", 2.05, 0.0, 1.75, 0.0),
                ("right", 0.0, 1.15, 1.65, 90.0),
                ("rear", -2.05, 0.0, 1.75, 180.0),
                ("left", 0.0, -1.15, 1.65, -90.0),
            )
            roles = {f"adas_{name}_camera" for name, *_ in views}
            for a in world.get_actors().filter("sensor.camera.rgb"):
                if a.attributes.get("role_name") in roles:
                    try:
                        if getattr(a, "is_alive", True):
                            a.destroy()
                    except Exception:
                        pass

            self.carla_cameras.clear()
            for name, x, y, z, yaw in views:
                bp = world.get_blueprint_library().find("sensor.camera.rgb")
                bp.set_attribute("image_size_x", str(self.tile_w))
                bp.set_attribute("image_size_y", str(self.tile_h))
                bp.set_attribute("fov", "60")
                bp.set_attribute("role_name", f"adas_{name}_camera")
                tf = carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(yaw=yaw))
                cam = world.spawn_actor(bp, tf, attach_to=ego)
                self.carla_cameras.append(cam)

                def _make_cb(view_name):
                    def _on_img(img):
                        arr = np.frombuffer(img.raw_data, dtype=np.uint8)
                        bgr = arr.reshape((img.height, img.width, 4))[:, :, :3]
                        with self._lock:
                            self._frames[view_name] = bgr.copy()
                    return _on_img

                cam.listen(_make_cb(name))

            print(f"[CarlaStreamer] [OK] 成功为自车挂载四向环视相机 (4x {self.tile_w}x{self.tile_h})")
            return True
        except Exception as e:
            print(f"[CarlaStreamer] [ERR] 连接挂载相机失败: {e}")
            return False

    def get_latest_mosaic(self) -> np.ndarray:
        """获取最新的四宫格 1280x720 图像。"""
        with self._lock:
            names = ("front", "right", "rear", "left")
            if all(k in self._frames for k in names):
                frames = {k: self._frames[k].copy() for k in names}
            else:
                frames = None

        if frames is None:
            return self._generate_test_pattern()

        mosaic = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        mosaic[0:self.tile_h, 0:self.tile_w] = frames["front"]
        mosaic[0:self.tile_h, self.tile_w:] = frames["right"]
        mosaic[self.tile_h:, 0:self.tile_w] = frames["rear"]
        mosaic[self.tile_h:, self.tile_w:] = frames["left"]

        # 添加视角角标标注
        cv2.putText(mosaic, "FRONT", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(mosaic, "RIGHT", (self.tile_w + 10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(mosaic, "REAR", (10, self.tile_h + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(mosaic, "LEFT", (self.tile_w + 10, self.tile_h + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        return mosaic

    def _server_loop(self) -> None:
        """后台高频刷新 JPEG 帧。"""
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.quality]
        while self._running:
            t0 = time.time()
            mosaic = self.get_latest_mosaic()
            ok, buf = cv2.imencode(".jpg", mosaic, encode_param)
            if ok:
                with self._lock:
                    self._latest_jpeg = buf.tobytes()
                    self._latest_bgr = mosaic
                    self._frame_seq += 1

            t1 = time.time()
            elapsed = t1 - self._last_tick
            self._last_tick = t1
            if elapsed > 0:
                self._fps = 0.9 * self._fps + 0.1 / elapsed
            time.sleep(max(0.01, 0.05 - (t1 - t0)))

    def start_http_server(self) -> None:
        """启动 HTTP MJPEG 推流服务。"""
        if self._running:
            return
        self._running = True

        # 启动抓帧刷新线程
        threading.Thread(target=self._server_loop, daemon=True).start()

        streamer_ref = self

        class MJPEGHandler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path in ("/", "/stream", "/video", "/video_feed"):
                    self.send_response(200)
                    self.send_header("Age", "0")
                    self.send_header("Cache-Control", "no-cache, private")
                    self.send_header("Pragma", "no-cache")
                    self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
                    self.end_headers()
                    try:
                        while streamer_ref._running:
                            jpeg = streamer_ref._latest_jpeg
                            if jpeg:
                                self.wfile.write(b"--FRAME\r\n")
                                self.send_header("Content-Type", "image/jpeg")
                                self.send_header("Content-Length", str(len(jpeg)))
                                self.end_headers()
                                self.wfile.write(jpeg)
                                self.wfile.write(b"\r\n")
                            time.sleep(0.04)
                    except (ConnectionResetError, BrokenPipeError):
                        pass
                elif self.path in ("/frame", "/snapshot.jpg"):
                    jpeg = streamer_ref._latest_jpeg
                    if jpeg:
                        self.send_response(200)
                        self.send_header("Content-Type", "image/jpeg")
                        self.send_header("Content-Length", str(len(jpeg)))
                        self.end_headers()
                        self.wfile.write(jpeg)
                    else:
                        self.send_error(503, "No frame available")
                elif self.path == "/status":
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.end_headers()
                    status = f'{{"fps": {streamer_ref._fps:.1f}, "seq": {streamer_ref._frame_seq}, "cameras": {len(streamer_ref.carla_cameras)}}}'
                    self.wfile.write(status.encode("utf-8"))
                else:
                    self.send_error(404)

            def log_message(self, format, *args):
                pass  # 静默高频访问日志

        self._http_server = ThreadingHTTPServer(("0.0.0.0", self.http_port), MJPEGHandler)
        self._http_thread = threading.Thread(target=self._http_server.serve_forever, daemon=True)
        self._http_thread.start()
        print(f"[CarlaStreamer] HTTP MJPEG 视频流已启动: http://0.0.0.0:{self.http_port}/stream")

    def stop(self) -> None:
        """停止推流并释放相机。"""
        self._running = False
        if self._http_server:
            try:
                self._http_server.shutdown()
                self._http_server.server_close()
            except Exception:
                pass
            self._http_server = None
        for cam in self.carla_cameras:
            try:
                cam.stop()
                if getattr(cam, "is_alive", True):
                    cam.destroy()
            except Exception:
                pass
        self.carla_cameras.clear()


def main():
    parser = argparse.ArgumentParser(description="CARLA 4-Camera MJPEG Streamer")
    parser.add_argument("--carla-host", default="127.0.0.1", help="CARLA Host IP")
    parser.add_argument("--carla-port", type=int, default=2000, help="CARLA Port")
    parser.add_argument("--http-port", type=int, default=8081, help="HTTP MJPEG Stream Port")
    args = parser.parse_args()

    streamer = CarlaMultiViewStreamer(host=args.carla_host, port=args.carla_port, http_port=args.http_port)
    streamer.attach_to_carla()
    streamer.start_http_server()

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n[CarlaStreamer] 收到退出信号...")
    finally:
        streamer.stop()


if __name__ == "__main__":
    main()
