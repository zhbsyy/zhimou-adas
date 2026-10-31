"""Explicit camera-input RGB gains. Never infer gains from arbitrary road scenes."""
from pathlib import Path
import json
import math

import cv2
import numpy as np


class CameraInputColor:
    def __init__(self, path=None):
        self.profile = {"schema": "camera_input_color.v1", "gains_rgb": [1., 1., 1.],
                        "status": "identity_no_camera_calibration", "source": "pcie"}
        if path:
            with open(path, encoding="utf-8-sig") as stream:
                self.profile = json.load(stream)
            if self.profile.get("schema") != "camera_input_color.v1" or self.profile.get("source") != "pcie":
                raise ValueError("Expected a PCIe camera_input_color.v1 profile")
        gains = self.profile.get("gains_rgb")
        if (not isinstance(gains, list) or len(gains) != 3 or
                any(type(g) not in (int, float) or not math.isfinite(g) or not .5 <= g <= 2 for g in gains)):
            raise ValueError("Camera RGB gains must be three finite numbers in [0.5, 2]")
        self.description = {"schema": "camera_input_color.v1", "gains_rgb": gains,
                            "profile_file": str(Path(path).resolve()) if path else None,
                            "status": str(self.profile.get("status", "explicit_profile"))[:80],
                            "scope": "camera input before detection, segmentation, HSV and preview"}
        self.lut = None
        if any(abs(g-1) > 1e-6 for g in gains):
            # Same BGR LUT arithmetic as the previous camera_config.make_gain_lut.
            values = np.arange(256, dtype=np.float32)
            self.lut = np.empty((256, 1, 3), np.uint8)
            for channel, gain in enumerate(reversed(gains)):
                self.lut[:, 0, channel] = np.clip(np.rint(values*gain), 0, 255).astype(np.uint8)

    def apply(self, bgr):
        return bgr if self.lut is None else cv2.LUT(bgr, self.lut)


def estimate_neutral_gains(bgr, roi):
    """Requires a user-declared neutral sheet. This is not gray-world auto-WB."""
    if (len(roi) != 4 or any(not math.isfinite(v) or not 0 <= v <= 1 for v in roi)
            or roi[0] >= roi[2] or roi[1] >= roi[3]):
        raise ValueError("Use normalized x1,y1,x2,y2 for the neutral-paper ROI")
    h, w = bgr.shape[:2]
    x1, y1, x2, y2 = round(roi[0]*w), round(roi[1]*h), round(roi[2]*w), round(roi[3]*h)
    tile = bgr[y1:y2, x1:x2]
    if min(tile.shape[:2], default=0) < 20:
        raise ValueError("Neutral-paper ROI is too small")
    pixels = tile.reshape(-1, 3).astype(np.float32)
    median = np.median(pixels, axis=0)
    spread = np.percentile(pixels, 90, axis=0)-np.percentile(pixels, 10, axis=0)
    if median.min() < 35 or np.mean(np.any(pixels >= 248, axis=1)) > .10:
        raise ValueError("Neutral reference is too dark or clipped; avoid bright windows")
    if np.any(spread/np.maximum(median, 1) > .30):
        raise ValueError("ROI is not uniform; fill it with the plain neutral sheet")
    target = float(median.mean())
    gains_bgr = target/median
    if gains_bgr.min() < .5 or gains_bgr.max() > 2:
        raise ValueError("Color imbalance is too large for bounded white gains; check format/lighting")
    return {"schema": "camera_input_color.v1", "source": "pcie",
            "status": "neutral_reference_estimate_pending_red_green_scene_check",
            "gains_rgb": [round(float(v), 6) for v in gains_bgr[::-1]],
            "reference": {"roi_normalized": list(roi), "median_bgr": median.tolist(),
                          "p90_minus_p10_bgr": spread.tolist(), "target": target},
            "limits": "Diagonal white gains only; cannot repair channel permutations, lost detail or arbitrary hue errors."}
