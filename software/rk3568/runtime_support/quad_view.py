#!/usr/bin/env python3
"""四路环视预览：FPGA 两路（前/后） + 2 个 USB 摄像头（左/右） → 1280x720 画布 → MJPEG 网页。

设计原则
--------
* **不动 FPGA**：级的四宫格帧里，左上=本板左目、左下=级联板左目，直接裁象限即可。
* **不动生产主程序**：这是独立程序，跑在另一个端口（默认 8083），
  但**会独占 PCIe 采集设备**，所以必须先把 `pcie_stream.py` 停掉再运行本程序。
* 复用工程既有模块：`pcie_frame.PcieFrameSource`（抓帧+隔行修复+LUT）、
  `hw_jpeg.JpegEncoder`（VPU 硬编，失败自动回退 cv2）、`usb_source.UsbCameraSource`。
* 画布仍是 **1280x720**，每格 640x360，因此下游（硬编/网页/统计）与原方案一致。

布局（默认）
-----------
    左上 TL = 前 (FPGA 本板左目)      右上 TR = 右 (USB#1)
    左下 BL = 后 (FPGA 级联板左目)    右下 BR = 左 (USB#2)

用法
----
    cd /userdata/cascade_camera_color_20261029
    bash ./START_QUAD.sh   # 按 USB 物理端口识别、应用单路配置
    # 浏览器打开 http://<板子IP>:8083/     统计 http://<板子IP>:8083/stat

    # 先只验 FPGA 两格（不插 USB）：
    sudo /home/linaro/yolov8_deploy/.venv-offline/bin/python quad_view.py
    # 关掉硬编做对照：
    sudo /home/linaro/yolov8_deploy/.venv-offline/bin/python quad_view.py --no-hw-jpeg
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from types import SimpleNamespace
from color_profiles import ColorProfiles, handle_http

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from pcie_frame import PcieFrameSource                      # noqa: E402
from camera_ports import (FRONT_CROP, REAR_CROP, TILE_WIDTH, TILE_HEIGHT, open_usb_camera)  # noqa: E402

try:
    from hw_jpeg import JpegEncoder                         # noqa: E402
except Exception:                                           # pragma: no cover
    JpegEncoder = None

TILE_W, TILE_H = TILE_WIDTH, TILE_HEIGHT
CANVAS_W, CANVAS_H = TILE_W * 2, TILE_H * 2

# 四个格子的左上角坐标
POS = {"tl": (0, 0), "tr": (TILE_W, 0), "bl": (0, TILE_H), "br": (TILE_W, TILE_H)}

PROFILES = None
STOP = threading.Event()
STATS = {"compose_fps": 0.0, "encode_fps": 0.0, "encode_ms": 0.0, "hw": "off",
         "frames": 0, "errors": 0}
LATEST = {"jpg": None, "seq": 0, "t": 0.0}


# --------------------------------------------------------------------- 抓帧线程
class PcieSource:
    """跑 PcieFrameSource，只保留最新一帧（与 USB 源同样的语义）。"""

    def __init__(self, cap_reader: str, label: str = "pcie", fix_interlace: bool = True):
        self.src = PcieFrameSource(cap_reader, fix_interlace=fix_interlace)
        self.label = label
        self.lock = threading.Lock()
        self.frame = None
        self.t = 0.0
        self.count = 0
        self.errors = 0
        self.fps = 0.0
        self._last_t = None
        self._thread = None

    def start(self):
        self._thread = threading.Thread(target=self._run, name="pcie", daemon=True)
        self._thread.start()

    def _run(self):
        while not STOP.is_set():
            try:
                bgr = self.src.read_bgr()
            except Exception as exc:
                with self.lock:
                    self.errors += 1
                print(f"[pcie] 读帧异常: {type(exc).__name__}: {exc}", file=sys.stderr)
                time.sleep(0.5)
                continue
            if bgr is None:
                continue
            now = time.perf_counter()
            PROFILES.observe(bgr, self.count + 1, time.monotonic())
            dt = now - (self._last_t or now)
            self._last_t = now
            with self.lock:
                self.frame = bgr
                self.t = now
                self.count += 1
                if dt > 0:
                    inst = 1.0 / dt
                    self.fps = inst if self.fps == 0 else 0.9 * self.fps + 0.1 * inst

    def latest(self):
        with self.lock:
            return self.frame, self.t, self.fps

    def snapshot(self):
        with self.lock:
            age = (time.perf_counter() - self.t) * 1000 if self.t else None
            return dict(label=self.label, fps=round(self.fps, 2), frames=self.count,
                        errors=self.errors, age_ms=round(age, 1) if age else None)

    def close(self):
        try:
            self.src.close()
        except Exception:
            pass


# --------------------------------------------------------------------- 合成
def fit_tile(img, w=TILE_W, h=TILE_H):
    """把任意尺寸的图贴成 w*h；已是目标尺寸则原样返回（不拷贝）。"""
    if img is None:
        return None
    ih, iw = img.shape[:2]
    if iw == w and ih == h:
        return img
    interp = cv2.INTER_AREA if (iw > w or ih > h) else cv2.INTER_LINEAR
    return cv2.resize(img, (w, h), interpolation=interp)


def crop_center_169(img):
    """居中裁成 16:9（用于 320x240/640x480 这类 4:3 摄像头，贴格不变形）。"""
    if img is None:
        return None
    h, w = img.shape[:2]
    if w * 9 == h * 16:
        return img
    if w * 9 > h * 16:                    # 太宽 → 裁左右
        tw = (h * 16) // 9
        x0 = (w - tw) // 2
        return img[:, x0:x0 + tw]
    th = (w * 9) // 16                    # 太高 → 裁上下
    y0 = (h - th) // 2
    return img[y0:y0 + th, :]


def crop(frame, x, y, w, h):
    if frame is None:
        return None
    fh, fw = frame.shape[:2]
    x2, y2 = min(x + w, fw), min(y + h, fh)
    if x >= fw or y >= fh or x2 <= x or y2 <= y:
        return None
    return frame[y:y2, x:x2]


def draw_label(canvas, pos, text, sub, alarm=False):
    x, y = pos
    color = (0, 0, 255) if alarm else (0, 255, 0)
    cv2.rectangle(canvas, (x, y), (x + TILE_W - 1, y + TILE_H - 1), (70, 70, 70), 1)
    cv2.rectangle(canvas, (x, y), (x + 150, y + 26), (0, 0, 0), -1)
    cv2.putText(canvas, text, (x + 6, y + 19), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)
    if sub:
        cv2.rectangle(canvas, (x, y + TILE_H - 24), (x + 260, y + TILE_H), (0, 0, 0), -1)
        cv2.putText(canvas, sub, (x + 6, y + TILE_H - 7), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, (200, 200, 200), 1, cv2.LINE_AA)


def placeholder(canvas, pos, text="NO SIGNAL"):
    x, y = pos
    canvas[y:y + TILE_H, x:x + TILE_W] = (32, 32, 32)
    cv2.putText(canvas, text, (x + 150, y + TILE_H // 2), cv2.FONT_HERSHEY_SIMPLEX,
                1.0, (90, 90, 220), 2, cv2.LINE_AA)


# ----------------------------------------------------------- 合成自检用假帧
def fake_pcie(n):
    """造一张 1280x720，并在四个象限写上各自的通道名——用来肉眼确认裁剪位置对不对。"""
    img = np.full((CANVAS_H, CANVAS_W, 3), 28, np.uint8)
    names = [(0, 0, "Q0 / TL  LOCAL-LEFT"), (TILE_W, 0, "Q1 / TR  LOCAL-RIGHT"),
             (0, TILE_H, "Q2 / BL  REMOTE-LEFT"), (TILE_W, TILE_H, "Q3 / BR  REMOTE-RIGHT")]
    for x, y, t in names:
        cv2.rectangle(img, (x + 8, y + 8), (x + TILE_W - 8, y + TILE_H - 8), (80, 80, 80), 2)
        cv2.putText(img, t, (x + 26, y + 52), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (220, 220, 220), 2)
    bx = (n * 7) % CANVAS_W
    cv2.line(img, (bx, 0), (bx, CANVAS_H), (0, 180, 0), 2)
    return img


def fake_usb(n, name):
    img = np.full((TILE_H, TILE_W, 3), 40, np.uint8)
    cv2.putText(img, f"USB {name}", (24, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 220, 255), 2)
    bx = (n * 11) % TILE_W
    cv2.line(img, (bx, 0), (bx, TILE_H), (255, 120, 0), 3)
    return img


# --------------------------------------------------------------------- HTTP
class Handler(BaseHTTPRequestHandler):
    page = ("<!doctype html><meta charset=utf-8><title>四路环视</title>"
            "<style>body{margin:0;background:#111;color:#ddd;font:14px/1.5 monospace}"
            "img{width:100%;max-width:1280px;display:block}</style>"
            "<h3 style='margin:8px'>四路环视 · 前(FPGA) 后(FPGA) 左(USB) 右(USB)</h3>"
            "<p style='margin:8px'>颜色配置自动保存/加载。校准时用同一张白纸覆盖每格中央40%矩形；前后分别校准。</p>"
            "<img src=/stream><pre id=s style='margin:8px'></pre>"
            "<script>setInterval(async()=>{s.textContent=await (await fetch('/stat')).text()},1000)</script>")

    def log_message(self, *a):        # 静音访问日志
        pass

    def do_POST(self):
        handle_http(self, PROFILES)

    def do_GET(self):
        if self.path.startswith('/camera/'):
            handle_http(self, PROFILES)
            return
        if self.path.startswith("/stream"):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            last = -1
            try:
                while not STOP.is_set():
                    if LATEST["seq"] != last and LATEST["jpg"] is not None:
                        last = LATEST["seq"]
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                         b"Content-Length: %d\r\n\r\n" % len(LATEST["jpg"]))
                        self.wfile.write(LATEST["jpg"])
                        self.wfile.write(b"\r\n")
                    else:
                        time.sleep(0.002)
            except (BrokenPipeError, ConnectionResetError):
                pass
            return
        if self.path.startswith("/stat"):
            body = json.dumps(STATS, ensure_ascii=False, indent=2).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        body = self.page.encode()
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


# --------------------------------------------------------------------- main
def parse_rect(s):
    x, y, w, h = (int(v) for v in s.replace("x", ",").split(","))
    return x, y, w, h


def main():
    ap = argparse.ArgumentParser(description="四路环视预览（FPGA 前/后 + USB 左/右）")
    ap.add_argument("--cap-reader", default=os.path.join(HERE, "..", "Camera_Input", "cap_reader.py"))
    ap.add_argument("--usb-right", default="", help="右视 USB 设备，如 /dev/video0")
    ap.add_argument("--usb-left", default="", help="左视 USB 设备，如 /dev/video2")
    ap.add_argument("--usb-size", default="640x360")
    ap.add_argument("--usb-fps", type=int, default=20)
    ap.add_argument("--usb-crop", action="store_true",
                    help="USB 帧先居中裁成 16:9 再贴格（配合 320x240/640x480 用，画面不变形）")
    ap.add_argument("--front-crop", default=",".join(map(str, FRONT_CROP)),
                    help="前视在 PCIe 帧中的 x,y,w,h")
    ap.add_argument("--rear-crop", default=",".join(map(str, REAR_CROP)),
                    help="后视在 PCIe 帧中的 x,y,w,h")
    ap.add_argument("--color-profile", default=os.path.join(HERE, "..", "Configuration", "camera_profiles.json"))
    ap.add_argument("--fps", type=float, default=20.0, help="合成/编码目标帧率")
    ap.add_argument("--quality", type=int, default=85)
    ap.add_argument("--port", type=int, default=8083)
    ap.add_argument("--delay", type=float, default=1e-5, help="CAP_DELAY（抓帧行间延时）")
    ap.add_argument("--no-hw-jpeg", action="store_true")
    ap.add_argument("--no-pcie", action="store_true",
                    help="不打开 PCIe（只显示 USB 两格），用来判断 PCIe 抓帧是否在抢 CPU")
    ap.add_argument("--opencv-threads", type=int, default=1,
                    help="OpenCV 内部线程数。多线程会和采集线程抢 GIL/CPU，默认压到 1")
    ap.add_argument("--no-pcie-fix-interlace", action="store_true",
                    help="关掉主机侧隔行修复（若位流已修好可省一份 CPU）")
    ap.add_argument("--selftest", action="store_true",
                    help="不用 FPGA/USB，用合成画面自检：验证裁剪位置/合成/硬编/网页")
    ap.add_argument("--duration", type=float, default=0.0, help=">0 时自动退出（秒）")
    args = ap.parse_args()
    if (args.usb_left and args.usb_right and
            os.path.realpath(args.usb_left) == os.path.realpath(args.usb_right)):
        ap.error("LEFT 与 RIGHT 指向同一 USB 设备，拒绝启动")
    global PROFILES
    if parse_rect(args.front_crop) != FRONT_CROP or parse_rect(args.rear_crop) != REAR_CROP:
        ap.error("This calibrated profile is bound to the fixed front/rear mosaic tiles")
    PROFILES = ColorProfiles(args.color_profile)
    STATS["camera_color"] = PROFILES.description()
    print("[持久颜色配置] " + json.dumps(PROFILES.description(), ensure_ascii=False), flush=True)

    # 关键调优：OpenCV 默认会开多线程（通常 4），与我们的采集线程抢 CPU/GIL，
    # 实测会把 USB 采集从 20fps 压到 8~15fps。压到 1 之后各帧更可预测。
    try:
        cv2.setNumThreads(max(1, args.opencv_threads))
        print(f"[quad_view] cv2.setNumThreads({max(1, args.opencv_threads)})")
    except Exception:
        pass

    os.environ["CAP_DELAY"] = str(args.delay)
    uw, uh = (int(v) for v in args.usb_size.lower().split("x"))
    fc, rc = parse_rect(args.front_crop), parse_rect(args.rear_crop)
    fake = args.selftest

    pcie = None
    if fake:
        print("[quad_view] ★ 自检模式：用合成画面，不打开 PCIe / USB")
    elif args.no_pcie:
        print("[quad_view] ★ --no-pcie：不打开 PCIe，只跑 USB（FPGA 两格显示 NO SIGNAL）")
    else:
        print(f"[quad_view] PCIe 抓帧: {args.cap_reader}  (CAP_DELAY={args.delay}, "
              f"fix_interlace={not args.no_pcie_fix_interlace})")
        pcie = PcieSource(args.cap_reader, fix_interlace=not args.no_pcie_fix_interlace)
        pcie.start()

    usb = {}
    if not fake:
        if args.usb_right:
            usb["right"] = open_usb_camera("right", device=args.usb_right,
                                           width=uw, height=uh, fps=args.usb_fps)
            usb["right"].start()
            print(f"[quad_view] 右视 USB: {args.usb_right} @ {args.usb_size}")
        if args.usb_left:
            usb["left"] = open_usb_camera("left", device=args.usb_left,
                                          width=uw, height=uh, fps=args.usb_fps)
            usb["left"].start()
            print(f"[quad_view] 左视 USB: {args.usb_left} @ {args.usb_size}")
        if not usb:
            print("[quad_view] 未指定 USB（--usb-left/--usb-right），这两格显示 NO SIGNAL")

    enc = None
    if not args.no_hw_jpeg and JpegEncoder is not None:
        try:
            enc = JpegEncoder(width=CANVAS_W, height=CANVAS_H, quality=args.quality, verbose=True)
            print("[quad_view] 使用 VPU 硬编（mppjpegenc）")
        except Exception as exc:
            print(f"[quad_view] 硬编初始化失败，回退 cv2: {exc}", file=sys.stderr)

    srv = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    threading.Thread(target=srv.serve_forever, name="http", daemon=True).start()
    print(f"[quad_view] 打开 http://<板子IP>:{args.port}/   统计 /stat")
    print("[quad_view] ⚠ 本程序独占 PCIe，请先停掉 pcie_stream.py。Ctrl+C 退出。")

    canvas = np.zeros((CANVAS_H, CANVAS_W, 3), np.uint8)
    usb_seq = {k: 0 for k in usb}
    usb_last = {k: None for k in usb}
    n, t0, t_last = 0, time.perf_counter(), time.perf_counter()
    fps_ema, enc_ema = 0.0, 0.0
    period = 1.0 / max(args.fps, 1.0)

    def on_signal(*_):
        STOP.set()
    for s in (signal.SIGINT, signal.SIGTERM):
        signal.signal(s, on_signal)

    try:
        while not STOP.is_set():
            loop_t = time.perf_counter()
            # 取最新一帧（不排队、不等待旧帧）
            if fake:
                usb_last["right"] = SimpleNamespace(bgr=fake_usb(n, "RIGHT"), sequence=n)
                usb_last["left"] = SimpleNamespace(bgr=fake_usb(n, "LEFT"), sequence=n)
            else:
                for k, src in usb.items():
                    f = src.read_newer(usb_seq[k], timeout=0.0)
                    if f is not None:
                        usb_seq[k], usb_last[k] = f.sequence, f
            if fake:
                pframe, pt, pfps = fake_pcie(n), time.perf_counter(), args.fps
            elif pcie is not None:
                pframe, pt, pfps = pcie.latest()
            else:
                pframe, pt, pfps = None, 0.0, 0.0

            canvas[:] = 0
            def usb_tile(k):
                f = usb_last.get(k)
                if f is None:
                    return None
                img = f.bgr
                if args.usb_crop:
                    img = crop_center_169(img)
                return fit_tile(img)

            front_tile, rear_tile = (PROFILES.views(pframe) if pframe is not None else (None, None))
            description = PROFILES.published_description()
            STATS["camera_color"] = description
            STATS["rear_swap_rb"] = description["cameras"]["rear"]["swap_rb"]
            STATS["front_gains_rgb"] = description["cameras"]["front"]["gains_rgb"]
            STATS["rear_gains_rgb"] = description["cameras"]["rear"]["gains_rgb"]
            tiles = {
                "tl": (front_tile, "FRONT  FPGA", None),
                "tr": (usb_tile("right"), "RIGHT  USB", None),
                "bl": (rear_tile, "REAR   FPGA", None),
                "br": (usb_tile("left"), "LEFT   USB", None),
            }
            for pos_name, (img, text, alarm) in tiles.items():
                pos = POS[pos_name]
                if img is None:
                    placeholder(canvas, pos)
                else:
                    canvas[pos[1]:pos[1] + TILE_H, pos[0]:pos[0] + TILE_W] = fit_tile(img)
            # 标签/帧龄放在贴图之后，避免被覆盖
            for pos_name, (img, text, alarm) in tiles.items():
                pos = POS[pos_name]
                if fake:
                    sub = "synthetic"
                elif pos_name in ("tl", "bl"):
                    sub = (f"pcie {pfps:.1f}fps age {(time.perf_counter() - pt) * 1000:.0f}ms"
                           if pt else "no signal")
                else:
                    k = "right" if pos_name == "tr" else "left"
                    s = usb[k].snapshot() if k in usb else {}
                    sub = (f"{s.get('fps', 0)}fps age {s.get('last_frame_age_ms')}ms"
                           if s else "未连接")
                draw_label(canvas, pos, text, sub, alarm)

            # 编码
            t_enc = time.perf_counter()
            jpg = enc.encode_bgr(canvas) if enc is not None else None
            used_hw = jpg is not None
            if jpg is None:
                ok, buf = cv2.imencode(".jpg", canvas, [int(cv2.IMWRITE_JPEG_QUALITY), args.quality])
                jpg = buf.tobytes() if ok else None
            enc_ms = (time.perf_counter() - t_enc) * 1000
            if jpg is not None:
                LATEST["jpg"], LATEST["seq"], LATEST["t"] = jpg, LATEST["seq"] + 1, time.perf_counter()

            n += 1
            now = time.perf_counter()
            if now - t_last > 0:
                inst = 1.0 / (now - t_last)
                fps_ema = inst if fps_ema == 0 else 0.9 * fps_ema + 0.1 * inst
                enc_ema = enc_ms if enc_ema == 0 else 0.9 * enc_ema + 0.1 * enc_ms
            t_last = now
            STATS.update(compose_fps=round(fps_ema, 2), encode_ms=round(enc_ema, 2),
                         encode_fps=round(n / max(now - t0, 1e-6), 2),
                         hw=("on" if used_hw else ("off" if enc is None else "broken")),
                         frames=n,
                         errors=(0 if pcie is None else pcie.snapshot()["errors"]),
                         selftest=fake, no_pcie=(pcie is None and not fake),
                         pcie=(dict(label="selftest", fps=args.fps) if fake else
                               (dict(label="disabled") if pcie is None else pcie.snapshot())),
                         usb={} if fake else {k: v.snapshot() for k, v in usb.items()},
                         jpeg_bytes=len(jpg or b""))

            if n % 40 == 0:
                print(f"[quad_view] frames={n} compose={fps_ema:.1f}fps enc={enc_ema:.1f}ms "
                      f"hw={'on' if used_hw else 'off'} pcie={pfps:.1f}fps")
            if args.duration and (time.perf_counter() - t0) >= args.duration:
                break
            sleep = period - (time.perf_counter() - loop_t)
            if sleep > 0:
                time.sleep(sleep)
    finally:
        STOP.set()
        for s in usb.values():
            s.close()
        if pcie is not None:
            pcie.close()
        if enc is not None:
            try:
                print("[quad_view] 编码统计: " + enc.report())
            except Exception:
                pass
            enc.close()
        try:
            srv.shutdown()
        except Exception:
            pass
        print(f"[quad_view] 退出：合成 {n} 帧"
              + ("" if pcie is None else f"，pcie {pcie.snapshot()}"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
