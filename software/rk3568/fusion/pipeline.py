"""Owned immutable frames + one-item latest mailboxes; no video backlog."""
from dataclasses import dataclass
import threading
import time

import cv2
import numpy as np

from detection_policy import TILES

SOURCE_TTL = .75
RESULT_TTL = .50


class Latest:
    def __init__(self):
        self.condition = threading.Condition()
        self.version, self.value = 0, None

    def publish(self, value):
        with self.condition:
            self.version += 1
            self.value = value
            self.condition.notify_all()

    def get(self):
        with self.condition:
            return self.version, self.value

    def newer(self, version, timeout=.2):
        with self.condition:
            self.condition.wait_for(lambda: self.version > version, timeout)
            return self.version, self.value if self.version > version else None


@dataclass(frozen=True)
class CameraFrame:
    image: np.ndarray
    sequence: int
    captured: float


@dataclass(frozen=True)
class Mosaic:
    image: np.ndarray
    sequence: int
    # name -> (physical capture sequence, host receive monotonic timestamp)
    origins: dict
    color_revision: int


def fresh(captured, now, ttl=SOURCE_TTL):
    return 0 <= now - captured <= ttl


def fit_usb(image):
    h, w = image.shape[:2]
    if w * 9 > h * 16:
        width = h * 16 // 9
        image = image[:, (w-width)//2:(w-width)//2+width]
    elif w * 9 < h * 16:
        height = w * 9 // 16
        image = image[(h-height)//2:(h-height)//2+height]
    return cv2.resize(image, (640, 360), interpolation=cv2.INTER_LINEAR)


def make_mosaic(frames, sequence, color_revision, now=None):
    now = time.perf_counter() if now is None else now
    image = np.full((720, 1280, 3), 24, np.uint8)
    origins = {}
    for name, (x1, y1, x2, y2) in TILES.items():
        frame = frames.get(name)
        if frame is not None and fresh(frame.captured, now):
            pixels = fit_usb(frame.image) if name in ("right", "left") else frame.image
            if pixels.shape != (360, 640, 3) or pixels.dtype != np.uint8:
                raise ValueError("Invalid tile format: " + name)
            image[y1:y2, x1:x2] = pixels
            origins[name] = (frame.sequence, frame.captured)
    image.flags.writeable = False
    return Mosaic(image, sequence, origins, color_revision)


def result_valid(result, mosaic, camera, now):
    if result is None or result["color_revision"] != mosaic.color_revision:
        return False
    origin = result["origins"].get(camera)
    current = mosaic.origins.get(camera)
    return bool(origin and current and origin[0] <= current[0]
                and fresh(origin[1], now, RESULT_TTL) and fresh(current[1], now))
