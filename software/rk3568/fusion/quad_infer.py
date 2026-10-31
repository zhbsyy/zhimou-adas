#!/usr/bin/env python3
"""Cascaded physical four-view inference, isolated from frozen decision software."""
import argparse
from collections import deque
import json
from pathlib import Path
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

import cv2
import numpy as np

import pcie_post
from cached_display import prepare_masks, apply_masks_inplace
from camera_ports import open_usb_camera, resolve_usb_devices
from color_profiles import ColorProfiles, handle_http
from detection_policy import CLASS10_MIN_CONFIDENCE, OTHER_MIN_CONFIDENCE, TILES, tile_detections
from jpeg_lifecycle import JoinedJpegEncoder
from pcie_frame import PcieFrameSource
from pipeline import CameraFrame, Latest, make_mosaic, fresh, result_valid
from postprocess_candidate import lane_argmax_planar
from r15_front_fast import R15Front
from runtime_backend import runtime_factory, check_model

HERE = Path(__file__).resolve().parent


class Metrics:
    def __init__(self):
        self.lock = threading.Lock()
        self.counts, self.ms, self.rates = {}, {}, {}
        self.errors = deque(maxlen=20)

    def add(self, name, count=1, **milliseconds):
        with self.lock:
            self.counts[name] = self.counts.get(name, 0) + count
            for key, value in milliseconds.items():
                old = self.ms.get(key)
                self.ms[key] = value if old is None else .9 * old + .1 * value

    def snapshot(self):
        with self.lock:
            return {"counts": dict(self.counts), "fps_5s": dict(self.rates),
                    "stage_ema_ms": {k: round(v, 2) for k, v in self.ms.items()},
                    "errors": list(self.errors)}

    def fail(self, stage, exc):
        with self.lock:
            self.errors.append({"stage": stage, "error": repr(exc)})
        print("[ERROR %s] %r" % (stage, exc), flush=True)


class Application:
    def __init__(self, args):
        self.args, self.stop = args, threading.Event()
        self.profiles = ColorProfiles(args.color_profile)
        if self.profiles.data["revision"] < 2:
            raise ValueError("Use the accepted front/rear color profile (revision >= 2)")
        self.names = Path(args.classes).read_text(encoding="utf-8-sig").splitlines()
        if self.names != ["person", "rider", "car", "truck", "bus", "train", "motor",
                          "bike", "traffic light", "traffic sign", "crosswalk"]:
            raise ValueError("Unexpected class order; class10 must be crosswalk")
        self.model_hashes = {"det": check_model(args.model), "lane": check_model(args.lane_model)}
        self.metrics = Metrics()
        self.npu_lock = threading.Lock()
        self.pcie, self.mosaics, self.detections, self.lanes = [Latest() for _ in range(4)]
        self.rendered, self.jpeg = Latest(), Latest()
        self.threads, self.usb, self.encoder = [], {}, None
        self.decoder, self.detector, self.lane = None, None, None
        self.server = None

    def worker(self, name, function):
        def guarded():
            try:
                function()
            except Exception as exc:
                if not self.stop.is_set():
                    self.metrics.fail(name, exc)
                    self.stop.set()
        thread = threading.Thread(target=guarded, name=name, daemon=True)
        self.threads.append(thread)
        thread.start()

    def capture(self):
        sequence = 0
        while not self.stop.is_set():
            image = self.decoder.read_bgr()
            if self.stop.is_set():
                break
            if image is None:
                raise EOFError("PCIe reader returned no frame")
            if not np.any(image):
                self.metrics.add("pcie_empty_warmup")
                continue  # Initial all-zero FPGA frame must not become valid video.
            received = time.perf_counter()
            sequence += 1
            self.profiles.observe(image, sequence, received)
            front, rear = self.profiles.views(image)
            front.flags.writeable = rear.flags.writeable = False
            self.pcie.publish({"front": CameraFrame(front, sequence, received),
                               "rear": CameraFrame(rear, sequence, received)})
            self.metrics.add("pcie_capture", convert_ms=self.decoder.convert_ms)

    def compose(self):
        last_signature, sequence = None, 0
        deadline = time.perf_counter()
        while not self.stop.is_set():
            _, pair = self.pcie.get()
            frames = dict(pair or {})
            for name, source in self.usb.items():
                frame = source.read_newer(0, timeout=0)
                if frame is not None:
                    frames[name] = CameraFrame(frame.bgr, frame.sequence, frame.received_mono)
            now = time.perf_counter()
            signature = tuple((name, f.sequence, fresh(f.captured, now)) for name, f in sorted(frames.items()))
            if signature != last_signature:
                last_signature, sequence = signature, sequence + 1
                t0 = time.perf_counter()
                self.mosaics.publish(make_mosaic(frames, sequence, self.profiles.data["revision"], now))
                self.metrics.add("compose", compose_ms=(time.perf_counter()-t0)*1000)
            deadline += 1 / self.args.video_fps
            self.stop.wait(max(0, deadline-time.perf_counter()))
            deadline = max(deadline, time.perf_counter() - 1 / self.args.video_fps)

    def detect(self):
        version = 0
        while not self.stop.is_set():
            new, mosaic = self.mosaics.newer(version)
            if mosaic is None:
                continue
            self.metrics.add("det_input_skipped", max(0, new-version-1))
            version = new
            now = time.perf_counter()
            valid = {n for n, origin in mosaic.origins.items() if fresh(origin[1], now)}
            if not valid:
                continue
            t0 = time.perf_counter()
            tensor, transform = pcie_post.preprocess_fast(mosaic.image, 640)
            pre = time.perf_counter()
            with self.npu_lock:
                acquired = time.perf_counter()
                outputs = self.detector.inference(inputs=[tensor], data_format="nhwc")
            called = time.perf_counter()
            if outputs is None or len(outputs) != 9 or any(not np.isfinite(a).all() for a in outputs):
                raise ValueError("Invalid A detector outputs")
            dets = pcie_post.decode_auto(outputs, size=640, nc=11, confidence=OTHER_MIN_CONFIDENCE)
            dets = pcie_post.to_original(dets, transform)
            tiles = tile_detections(dets, valid)
            for rows in tiles.values():
                rows.flags.writeable = False
            self.detections.publish({"sequence": mosaic.sequence, "origins": mosaic.origins,
                "color_revision": mosaic.color_revision, "tiles": tiles})
            self.metrics.add("det", det_pre_ms=(pre-t0)*1000,
                det_lock_wait_ms=(acquired-pre)*1000, det_call_ms=(called-acquired)*1000,
                det_post_ms=(time.perf_counter()-called)*1000)

    def segment(self):
        version, last_capture = 0, 0
        while not self.stop.is_set():
            new, mosaic = self.mosaics.newer(version)
            if mosaic is None:
                continue
            version = new
            origin = mosaic.origins.get("front")
            if not origin or origin[0] == last_capture or not fresh(origin[1], time.perf_counter()):
                continue
            last_capture = origin[0]
            direct, lane, timing = self.lane.infer(mosaic.image[:360, :640])
            prepared = prepare_masks((direct, lane))
            self.lanes.publish({"sequence": mosaic.sequence, "origins": {"front": origin},
                "color_revision": mosaic.color_revision, "prepared": prepared})
            self.metrics.add("lane", **{"lane_"+key: timing[key] for key in
                ("pre_ms", "lock_wait_ms", "call_ms", "post_ms")})

    def render_frame(self, mosaic, now):
        # Only this owned copy is drawn on; clean capture/model inputs are immutable.
        image = mosaic.image.copy()
        _, det = self.detections.get()
        _, lane = self.lanes.get()
        if result_valid(lane, mosaic, "front", now):
            apply_masks_inplace(image[:360, :640], lane["prepared"])
        for name, (x1, y1, x2, y2) in TILES.items():
            valid = name in mosaic.origins and fresh(mosaic.origins[name][1], now)
            if result_valid(det, mosaic, name, now):
                for bx1, by1, bx2, by2, score, cid in det["tiles"][name]:
                    cid = int(cid)
                    color = ((37*cid+40) % 220, (71*cid+180) % 220, (111*cid+60) % 220)
                    p = (round(float(bx1)), round(float(by1)))
                    cv2.rectangle(image, p, (round(float(bx2)), round(float(by2))), color, 2)
                    cv2.putText(image, "%s %.2f" % (self.names[cid], score),
                        (p[0], max(y1+18, p[1]-5)), cv2.FONT_HERSHEY_SIMPLEX, .5, color, 1, cv2.LINE_AA)
            cv2.rectangle(image, (x1, y1), (x1+195, y1+25), (0, 0, 0), -1)
            cv2.putText(image, name.upper()+ (" FPGA" if name in ("front", "rear") else " USB"),
                (x1+5, y1+19), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 255, 0) if valid else (0, 0, 255), 2)
            if not valid:
                cv2.putText(image, "NO FRESH SIGNAL", (x1+140, y1+180), cv2.FONT_HERSHEY_SIMPLEX,
                            .8, (0, 0, 255), 2)
        with self.metrics.lock:
            rates = dict(self.metrics.rates)
        label = "det %.1fHz | R15 %.1fHz Front | class10 >= %.2f" % (rates.get("det", 0), rates.get("lane", 0), CLASS10_MIN_CONFIDENCE)
        cv2.rectangle(image, (0, 698), (640, 719), (0, 0, 0), -1)
        cv2.putText(image, label, (5, 713), cv2.FONT_HERSHEY_SIMPLEX, .44, (230, 230, 230), 1)
        return image

    def render(self):
        version = 0
        while not self.stop.is_set():
            new, mosaic = self.mosaics.newer(version)
            if mosaic is None:
                continue
            self.metrics.add("render_input_skipped", max(0, new-version-1))
            version = new
            t0 = time.perf_counter()
            self.rendered.publish(self.render_frame(mosaic, t0))
            self.metrics.add("render", render_ms=(time.perf_counter()-t0)*1000)

    def publish_jpeg(self, jpeg):
        self.jpeg.publish((jpeg, time.perf_counter()))
        self.metrics.add("jpeg")

    def encode(self):
        version = 0
        software_only = False
        while not self.stop.is_set():
            new, image = self.rendered.newer(version)
            if image is None:
                continue
            self.metrics.add("encode_input_skipped", max(0, new-version-1))
            version = new
            # Never collect a second time: hardware reader publishes each JPEG once.
            if not software_only and (self.encoder.broken or not self.encoder.submit_bgr(image)):
                # End hardware publication before enabling software: never interleave
                # delayed old hardware JPEGs with newer software JPEGs.
                self.encoder.close()
                software_only = True
            if software_only:
                jpeg = self.encoder._sw_fallback(image)
                if jpeg:
                    self.publish_jpeg(jpeg)
                else:
                    raise RuntimeError("Both hardware and software JPEG failed")
            self.metrics.add("encode_submit")

    def status(self, include_boxes=False):
        now = time.perf_counter()
        _, mosaic = self.mosaics.get()
        _, det = self.detections.get()
        _, lane = self.lanes.get()
        _, jpg = self.jpeg.get()
        profile = self.profiles.description()
        profile["scope"] = "same corrected input used by detector, R15 and video"
        result = {"schema": "cascade_dual_model.v1", "source": "physical_pcie_and_two_usb",
            "models": {"det": self.args.model, "lane": self.args.lane_model, "sha256": self.model_hashes},
            "class10_min_confidence": CLASS10_MIN_CONFIDENCE, "class10_name": "crosswalk",
            "other_min_confidence": OTHER_MIN_CONFIDENCE, "threshold_override_supported": False,
            "detector_scope": "one 640x640 inference on full four-view mosaic",
            "lane_scope": "Front only, 256x448 uint8 NHWC", "camera_color": profile,
            "mosaic_sequence": mosaic.sequence if mosaic else None,
            "det_sequence": det["sequence"] if det else None, "lane_sequence": lane["sequence"] if lane else None,
            "jpeg_publish_age_ms": round((now-jpg[1])*1000, 1) if jpg else None,
            "jpeg_size": [1280, 720], "runtime": "native_buffered, shared NPU call lock",
            "queues": "single latest item per stage; encoder input depth=1",
            "latency_note": "JPEG publish age is not matched-frame end-to-end latency; capture counters do not prove optical freshness",
            "running": not self.stop.is_set(), **self.metrics.snapshot()}
        result["cameras"] = {name: {"capture_seq": mosaic.origins[name][0],
            "age_ms": round((now-mosaic.origins[name][1])*1000, 1)}
            if mosaic and name in mosaic.origins else {"capture_seq": None, "age_ms": None} for name in TILES}
        result["usb"] = {name: source.snapshot() for name, source in self.usb.items()}
        result["encoder"] = dict(self.encoder.stats) if self.encoder else {}
        if include_boxes:
            result["detections"] = {name: det["tiles"][name].tolist() if mosaic and result_valid(det, mosaic, name, now) else [] for name in TILES}
        return result

    def report(self):
        previous, started = {}, time.perf_counter()
        while not self.stop.wait(5):
            now = time.perf_counter()
            current = self.metrics.snapshot()["counts"]
            rates = {name: round((value-previous.get(name, 0))/(now-started), 2)
                     for name, value in current.items() if not name.endswith("skipped")}
            with self.metrics.lock:
                self.metrics.rates = rates
            print("[四路双模型 5s] " + json.dumps({"fps": rates, "ms": self.metrics.snapshot()["stage_ema_ms"],
                "skipped": {n: v for n, v in current.items() if n.endswith("skipped")}}, ensure_ascii=False), flush=True)
            previous, started = current, now

    def run(self):
        try:
            factory = runtime_factory("native_buffered")
            self.detector = factory()
            self.detector.load_rknn(self.args.model)
            self.detector.init_runtime()
            self.lane = R15Front(self.args.lane_model, .6, 256, "uint8_nhwc", 448, factory)
            self.lane.npu_lock, self.lane.lane_argmax = self.npu_lock, lane_argmax_planar
            mapping = resolve_usb_devices()
            self.usb = {n: open_usb_camera(n, device=dev) for n, dev in mapping.items()}
            self.decoder = PcieFrameSource(getattr(self.args, "cap_reader", str(HERE.parent/"Camera_Input/cap_reader.py")), fix_interlace=True)
            self.decoder.start()
            self.encoder = JoinedJpegEncoder(1280, 720, quality=85, fps=20, depth=1, on_jpeg=self.publish_jpeg)
            for source in self.usb.values():
                source.start()
            self.server = ThreadingHTTPServer(("0.0.0.0", self.args.port), make_handler(self))
            for name, function in (("pcie", self.capture), ("compose", self.compose), ("det", self.detect),
                ("lane", self.segment), ("render", self.render), ("jpeg", self.encode), ("report", self.report),
                ("http", lambda: self.server.serve_forever(poll_interval=.2))):
                self.worker(name, function)
            print("CASCADE_DUAL_READY " + json.dumps({"stream": "http://192.168.2.3:%d/stream" % self.args.port,
                "class10": CLASS10_MIN_CONFIDENCE, "color_revision": self.profiles.data["revision"],
                "models": self.model_hashes}, ensure_ascii=False), flush=True)
            self.stop.wait(self.args.seconds if self.args.seconds > 0 else None)
        finally:
            self.stop.set()
            if self.server:
                self.server.shutdown()
                self.server.server_close()
            if self.decoder:
                self.decoder.close()
            for source in self.usb.values():
                source.close()
            for thread in self.threads:
                thread.join(timeout=5)
            if any(t.is_alive() for t in self.threads):
                raise RuntimeError("Worker did not exit; runtime context kept alive")
            if self.encoder:
                self.encoder.close()
            if self.detector:
                self.detector.release()
            if self.lane:
                self.lane.close()
            print("CASCADE_DUAL_SUMMARY " + json.dumps(self.status(), ensure_ascii=False), flush=True)
        if self.metrics.snapshot()["errors"]:
            raise SystemExit(1)


def make_handler(app):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def setup(self):
            super().setup()
            self.connection.settimeout(3)

        def do_GET(self):
            path = urlsplit(self.path).path
            if path.startswith("/camera/"):
                handle_http(self, app.profiles)
                return
            if path == "/stream":
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                version = 0
                try:
                    while not app.stop.is_set():
                        version, item = app.jpeg.newer(version)
                        if item is None:
                            continue
                        jpg = item[0]
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(jpg))
                        self.wfile.write(jpg)
                        self.wfile.write(b"\r\n")
                except (OSError, TimeoutError):
                    pass
                return
            if path in ("/stat", "/health", "/results"):
                body, mime = json.dumps(app.status(path == "/results"), ensure_ascii=False).encode(), "application/json"
            elif path == "/snapshot.jpg":
                _, item = app.jpeg.get()
                if item is None:
                    self.send_error(503, "Waiting for JPEG")
                    return
                body, mime = item[0], "image/jpeg"
            elif path == "/":
                body = ("<!doctype html><meta charset=utf-8><title>级联四路双模型</title>"
                    "<style>body{margin:8px;background:#111;color:white}img{width:100%;max-width:1280px}</style>"
                    f"<h3>四路双模型 · A INT8 全画面检测 / R15 INT8 前视 · class10 ≥ {CLASS10_MIN_CONFIDENCE:.2f}</h3>"
                    "<img src='/stream'><pre id=s></pre><script>setInterval(async()=>{s.textContent="
                    "JSON.stringify(await(await fetch('/stat')).json(),null,2)},1000)</script>").encode()
                mime = "text/html; charset=utf-8"
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", mime)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", default=str(HERE.parent/"Models/A_rk3568_int8.rknn"))
    parser.add_argument("--lane-model", default=str(HERE.parent/"Models/R15_256x448_patched_native_int8.rknn"))
    parser.add_argument("--classes", default=str(HERE.parent/"Models/classes_11.txt"))
    parser.add_argument("--color-profile", default=str(HERE.parent/"Configuration/camera_profiles.json"))
    parser.add_argument("--cap-reader", default=str(HERE.parent/"Camera_Input/cap_reader.py"))
    parser.add_argument("--port", type=int, default=8083)
    parser.add_argument("--video-fps", type=int, choices=range(1, 31), default=20)
    parser.add_argument("--seconds", type=float, default=0, help="0 runs until Ctrl+C")
    args = parser.parse_args()
    cv2.setNumThreads(1)
    app = Application(args)
    signal.signal(signal.SIGINT, lambda *_: app.stop.set())
    signal.signal(signal.SIGTERM, lambda *_: app.stop.set())
    app.run()


if __name__ == "__main__":
    main()
