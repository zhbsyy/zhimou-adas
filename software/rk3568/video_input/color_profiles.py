"""Persistent per-camera input correction; calibration uses declared neutral paper.

No road-scene gray-world estimation, no JPEG sampling and no chained legacy gains.
The raw PCIe mosaic is never modified. LUTs are rebuilt only on profile changes.
"""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json
import math
import os
import threading
import time

import cv2
import numpy as np

SCHEMA = "cascade_camera_color.v1"
BINDING = "sfp_tx_camera_frame_fix_20261014+sfp_rx_no_overlay_20261014"
CAMERAS = {"front": {"crop": [0, 0, 640, 360], "swap_rb": False},
           "rear": {"crop": [0, 360, 640, 360], "swap_rb": True}}


def default_profile():
    return {"schema": SCHEMA, "binding": BINDING, "revision": 0,
            "cameras": {name: {**cfg, "gains_rgb": [1., 1., 1.],
                                 "status": "neutral_reference_required"}
                        for name, cfg in CAMERAS.items()}}


def validate(data):
    if data.get("schema") != SCHEMA or data.get("binding") != BINDING:
        raise ValueError("Color profile does not match this cascaded camera layout")
    if type(data.get("revision")) is not int or data["revision"] < 0:
        raise ValueError("Invalid profile revision")
    if set(data.get("cameras", {})) != set(CAMERAS):
        raise ValueError("Both front and rear profiles are required")
    for name, cfg in CAMERAS.items():
        row = data["cameras"][name]
        if row.get("crop") != cfg["crop"] or type(row.get("swap_rb")) is not bool or row["swap_rb"] != cfg["swap_rb"]:
            raise ValueError("Camera crop/channel order changed: " + name)
        gains = row.get("gains_rgb")
        if (not isinstance(gains, list) or len(gains) != 3 or
                any(type(v) not in (int, float) or not math.isfinite(v) or not .5 <= v <= 2 for v in gains)):
            raise ValueError("Invalid RGB gains for " + name)
    return data


def make_lut(gains):
    if all(abs(v - 1) < 1e-8 for v in gains):
        return None
    levels = np.arange(256, dtype=np.float32)
    lut = np.empty((256, 1, 3), np.uint8)
    for c, gain in enumerate(reversed(gains)):
        lut[:, 0, c] = np.clip(np.rint(levels * gain), 0, 255).astype(np.uint8)
    return lut


def neutral_statistics(image, roi):
    if (len(roi) != 4 or any(type(v) not in (int, float) or not math.isfinite(v) or not 0 <= v <= 1 for v in roi)
            or roi[0] >= roi[2] or roi[1] >= roi[3]):
        raise ValueError("Invalid neutral-paper ROI")
    h, w = image.shape[:2]
    x1, y1, x2, y2 = [round(v * s) for v, s in zip(roi, (w, h, w, h))]
    patch = image[y1:y2, x1:x2]
    if min(patch.shape[:2]) < 20:
        raise ValueError("Neutral-paper ROI is too small")
    pixels = patch.reshape(-1, 3).astype(np.float32)
    median = np.median(pixels, axis=0)
    spread = np.percentile(pixels, 90, axis=0) - np.percentile(pixels, 10, axis=0)
    if median.min() < 40 or np.mean(np.any(pixels >= 248, axis=1)) > .05:
        raise ValueError("Paper is too dark or clipped; avoid bright windows and reflections")
    if np.any(spread / median > .25):
        raise ValueError("Paper ROI is not uniform; fill the central rectangle with plain white/gray paper")
    return {"median_bgr": median.tolist(), "spread_bgr": spread.tolist(), "roi": list(roi)}


def estimate(samples):
    if len(samples) < 5 or len({s["seq"] for s in samples}) < 5:
        raise ValueError("Need at least five different capture sequence numbers")
    medians = np.asarray([s["median_bgr"] for s in samples], dtype=np.float64)
    median = np.median(medians, axis=0)
    # Reject a moving paper / changing exposure instead of freezing unstable AWB.
    if np.any(np.ptp(medians, axis=0) / median > .12):
        raise ValueError("Neutral reference is changing; hold the paper steady and wait for exposure to settle")
    target = float(median.mean())
    gains = target / median
    if gains.min() < .5 or gains.max() > 2:
        raise ValueError("Imbalance exceeds bounded neutral gains; check channel format and lighting")
    return {"gains_rgb": [round(float(v), 6) for v in gains[::-1]],
            "reference": {"median_bgr": median.tolist(), "target": target,
                          "sample_count": len(samples), "first_seq": samples[0]["seq"],
                          "last_seq": samples[-1]["seq"], "roi": samples[0]["roi"],
                          "stage": "after_fixed_channel_order_before_gain_or_overlay"},
            "status": "neutral_calibrated_color_targets_pending"}


class ColorProfiles:
    def __init__(self, path):
        self.path = Path(path).resolve()
        self.lock = threading.RLock()
        self.calibration_lock = threading.Lock()
        self.latest = None
        self.epoch = os.urandom(12).hex()
        self.data = validate(json.loads(self.path.read_text(encoding="utf-8-sig")))
        self.luts = {n: make_lut(r["gains_rgb"]) for n, r in self.data["cameras"].items()}
        self._description = self._describe()

    def observe(self, image, sequence, captured_mono):
        if image.shape != (720, 1280, 3) or image.dtype != np.uint8:
            raise ValueError("Expected 1280x720 BGR PCIe mosaic")
        # Capture publishes a new owned image. Keep a reference, never draw on it.
        with self.lock:
            self.latest = image, int(sequence), float(captured_mono)

    @staticmethod
    def raw_tile(image, name):
        cfg = CAMERAS[name]
        x, y, w, h = cfg["crop"]
        tile = image[y:y+h, x:x+w]
        return cv2.cvtColor(tile, cv2.COLOR_BGR2RGB) if cfg["swap_rb"] else tile

    def views(self, image):
        # One snapshot per frame prevents half-old/half-new front/rear parameters.
        with self.lock:
            luts = self.luts
        outputs = []
        for name in CAMERAS:
            tile = self.raw_tile(image, name)
            outputs.append(tile if luts[name] is None else cv2.LUT(tile, luts[name]))
        return tuple(outputs)

    def _describe(self):
        result = json.loads(json.dumps(self.data))
        result.update(profile_file=str(self.path), source_epoch=self.epoch,
                      scope="shared per-camera input transform; currently used by four-view preview",
                      legacy_environment_gains=False)
        return result

    def description(self):
        with self.lock:
            return json.loads(json.dumps(self._description))

    def published_description(self):
        # Immutable by contract; the render loop only publishes/serializes it.
        # Replaced once per calibration, no per-frame profile deep copies.
        with self.lock:
            return self._description

    def snapshot(self, name):
        if name not in CAMERAS:
            raise ValueError("Expected front or rear")
        with self.lock:
            item = self.latest
        if item is None:
            raise ValueError("No live PCIe frame yet")
        image, seq, captured = item
        age = time.monotonic() - captured
        if age < 0 or age > .75:
            raise ValueError("PCIe frame is stale; calibration refused")
        return self.raw_tile(image, name), seq, age

    def calibrate(self, name, roi, confirmed):
        if confirmed is not True:
            raise ValueError("Declare that the ROI is filled with neutral paper")
        if name not in CAMERAS:
            raise ValueError("Expected front or rear")
        if not self.calibration_lock.acquire(blocking=False):
            raise ValueError("Another calibration is already running")
        try:
            samples, seen = [], set()
            deadline = time.monotonic() + 3
            while len(samples) < 9 and time.monotonic() < deadline:
                image, seq, _ = self.snapshot(name)
                if seq not in seen:
                    samples.append({"seq": seq, **neutral_statistics(image, roi)})
                    seen.add(seq)
                time.sleep(.10)
            result = estimate(samples)
            with self.lock:
                updated = json.loads(json.dumps(self.data))
                updated["revision"] += 1
                updated["cameras"][name].update(result)
                updated["cameras"][name]["reference"].update(source_epoch=self.epoch,
                    recorded_utc=datetime.now(timezone.utc).isoformat(),
                    recorded_board_clock_note="Board clock may be wrong; revision identifies order")
                validate(updated)
                blob = (json.dumps(updated, ensure_ascii=False, indent=2) + "\n").encode("utf-8")
                history = self.path.parent / "profile_history"
                history.mkdir(exist_ok=True)
                old = self.path.read_bytes()
                backup = history / ("before_revision_%d_%s.json" % (updated["revision"], hashlib.sha256(old).hexdigest()[:12]))
                if not backup.exists():
                    backup.write_bytes(old)
                temporary = self.path.with_name(self.path.name + ".pending")
                try:
                    with temporary.open("wb") as out:
                        out.write(blob)
                        out.flush()
                        os.fsync(out.fileno())
                    os.replace(temporary, self.path)
                finally:
                    if temporary.exists():
                        temporary.unlink()
                self.data = updated
                self.luts = {n: make_lut(r["gains_rgb"]) for n, r in updated["cameras"].items()}
                self._description = self._describe()
            return {"camera": name, "revision": updated["revision"], "saved": str(self.path),
                    "applied_without_restart": True, **result}
        finally:
            self.calibration_lock.release()


def handle_http(handler, profiles):
    """Only called for /camera/*; POST is local-only calibration, never arbitrary file I/O."""
    from urllib.parse import urlsplit, parse_qs
    url = urlsplit(handler.path)
    status, mime = 200, "application/json; charset=utf-8"
    headers = {}
    try:
        if handler.command == "GET" and url.path == "/camera/info":
            body = json.dumps(profiles.description(), ensure_ascii=False).encode()
        elif handler.command == "GET" and url.path == "/camera/raw.png":
            name = parse_qs(url.query).get("camera", ["front"])[0]
            image, seq, age = profiles.snapshot(name)
            ok, png = cv2.imencode(".png", image)
            if not ok:
                raise ValueError("PNG encoding failed")
            body, mime = png.tobytes(), "image/png"
            headers = {"X-Frame-Sequence": str(seq), "X-Frame-Age-Ms": str(round(age * 1000, 2)),
                       "X-Color-Stage": "fixed-channel-order-before-gain-and-overlay"}
        elif handler.command == "POST" and url.path == "/camera/calibrate":
            if handler.client_address[0] not in ("127.0.0.1", "::1"):
                raise ValueError("Run calibration locally on the RK3568")
            length = int(handler.headers.get("Content-Length", "0"))
            if not 0 < length <= 2048:
                raise ValueError("Invalid calibration request length")
            request = json.loads(handler.rfile.read(length))
            body = json.dumps(profiles.calibrate(request.get("camera"),
                request.get("roi", [.3, .3, .7, .7]), request.get("confirm_neutral")), ensure_ascii=False).encode()
        else:
            status, body = 404, b'{"error":"unknown camera endpoint"}'
    except (ValueError, TypeError, KeyError, OSError) as exc:
        status, body = 422, json.dumps({"error": str(exc)}, ensure_ascii=False).encode()
    handler.send_response(status)
    handler.send_header("Content-Type", mime)
    handler.send_header("Content-Length", str(len(body)))
    handler.send_header("Cache-Control", "no-store")
    for key, value in headers.items():
        handler.send_header(key, value)
    handler.end_headers()
    handler.wfile.write(body)
