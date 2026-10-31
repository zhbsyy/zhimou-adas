"""R15 direct/lane inference on one CARLA front camera tile.

The original RKNN expects RGB NCHW float32 in [0, 1]. The new conversion
trial expects RGB NHWC uint8 and scales by 1/255 inside RKNN. Select the
input contract explicitly; never infer it from a model filename.
"""
import time

import cv2
import numpy as np


def lane_argmax_numpy(lane_logits):
    return lane_logits.argmax(axis=0).astype(np.uint8)


def filter_lane_confidence_sparse(lane_idx, lane_logits, confidence):
    """Exact softmax confidence test only where the winning class is a lane.

    Background pixels will remain background regardless of their confidence,
    so exponentiating three logits for them is unnecessary. The mask is
    modified in place and remains pixel-identical to the dense implementation.
    """
    ys, xs = np.nonzero(lane_idx)
    if ys.size == 0:
        return lane_idx
    # The cropped logits are often non-contiguous. Coordinate indexing avoids
    # making a full-image copy just to flatten them.
    selected = lane_logits[:, ys, xs]
    peak = selected.max(axis=0)
    probability = 1.0 / np.exp(selected - peak[None, :]).sum(axis=0)
    low_confidence = probability < confidence
    lane_idx[ys[low_confidence], xs[low_confidence]] = 0
    return lane_idx


class R15Front:
    def __init__(self, model_path, confidence=0.6, height=640,
                 input_mode="float_nchw", width=640, runtime_factory=None):
        if runtime_factory is None:
            from rknnlite.api import RKNNLite
            runtime_factory = RKNNLite

        if not 0.0 < confidence < 1.0:
            raise ValueError("lane confidence must be between 0 and 1")
        if (height, width) not in ((256, 448), (384, 640), (640, 640)):
            raise ValueError("Unsupported R15 input dimensions")
        if input_mode not in ("float_nchw", "uint8_nhwc"):
            raise ValueError("unsupported R15 input mode: " + input_mode)
        self.confidence = confidence
        self.height = height
        self.width = width
        self.input_mode = input_mode
        self.npu_lock = None
        self.lane_argmax = lane_argmax_numpy
        self.rk = runtime_factory()
        if self.rk.load_rknn(model_path) != 0:
            raise RuntimeError("R15 load_rknn failed: " + model_path)
        if self.rk.init_runtime() != 0:
            self.rk.release()
            raise RuntimeError("R15 init_runtime failed")

    def preprocess(self, front):
        h, w = front.shape[:2]
        ratio = min(self.width / w, self.height / h)
        nw, nh = round(w * ratio), round(h * ratio)
        left = round((self.width - nw) / 2 - 0.1)
        top = round((self.height - nh) / 2 - 0.1)
        canvas = np.full((self.height, self.width, 3), 114, dtype=np.uint8)
        canvas[top:top + nh, left:left + nw] = cv2.resize(
            front, (nw, nh), interpolation=cv2.INTER_LINEAR)
        rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
        if self.input_mode == "float_nchw":
            tensor = np.ascontiguousarray(
                rgb.transpose(2, 0, 1)[None].astype(np.float32) / 255.0)
        else:
            tensor = np.ascontiguousarray(rgb[None])
        return tensor, (left, top, nw, nh, w, h)

    def _channels(self, array, channels):
        arr = np.asarray(array)
        if arr.shape == (1, channels, self.height, self.width):
            return arr[0]
        if arr.shape == (1, self.height, self.width, channels):
            return arr[0].transpose(2, 0, 1)
        raise ValueError("R15 output shape %s; expected %d channels at %dx%d" %
                         (arr.shape, channels, self.height, self.width))

    def infer(self, front):
        start = time.perf_counter()
        tensor, box = self.preprocess(front)
        pre_ms = (time.perf_counter() - start) * 1000
        t0 = time.perf_counter()
        data_format = "nchw" if self.input_mode == "float_nchw" else "nhwc"
        lock_acquired = t0
        if self.npu_lock is None:
            outs = self.rk.inference(inputs=[tensor], data_format=data_format)
        else:
            with self.npu_lock:
                lock_acquired = time.perf_counter()
                outs = self.rk.inference(inputs=[tensor], data_format=data_format)
        call_end = time.perf_counter()
        infer_ms = (call_end - t0) * 1000
        lock_wait_ms = (lock_acquired - t0) * 1000
        call_ms = (call_end - lock_acquired) * 1000
        if outs is None or len(outs) != 2:
            raise ValueError("R15 needs two outputs, got %s" %
                             (None if outs is None else len(outs)))
        direct = self._channels(outs[0], 2)
        lane = self._channels(outs[1], 3)
        if not np.isfinite(direct).all() or not np.isfinite(lane).all():
            raise ValueError("R15 output has NaN/Inf")
        if not np.any(direct) and not np.any(lane):
            raise RuntimeError("R15 returned all-zero logits; RKNN inference may have failed")
        left, top, nw, nh, w, h = box
        direct = direct[:, top:top + nh, left:left + nw]
        lane = lane[:, top:top + nh, left:left + nw]
        direct_mask = (direct[1] > direct[0]).astype(np.uint8)
        lane_idx = self.lane_argmax(lane)
        # Same softmax decision as the original, evaluated only for lane-class
        # winners. Most road pixels are background, so this avoids a dense
        # 3-channel exponential over the complete image.
        filter_lane_confidence_sparse(lane_idx, lane, self.confidence)
        if (nw, nh) != (w, h):
            direct_mask = cv2.resize(direct_mask, (w, h), interpolation=cv2.INTER_NEAREST)
            lane_idx = cv2.resize(lane_idx, (w, h), interpolation=cv2.INTER_NEAREST)
        post_ms = (time.perf_counter() - t0) * 1000 - infer_ms
        return direct_mask, lane_idx, {
            "pre_ms": pre_ms,
            "infer_ms": infer_ms,
            "lock_wait_ms": lock_wait_ms,
            "call_ms": call_ms,
            "post_ms": post_ms,
            "direct_pixels": int(np.count_nonzero(direct_mask)),
            "solid_pixels": int(np.count_nonzero(lane_idx == 1)),
            "dashed_pixels": int(np.count_nonzero(lane_idx == 2)),
            "direct_logits_range": [float(direct.min()), float(direct.max())],
            "lane_logits_range": [float(lane.min()), float(lane.max())],
        }

    def close(self):
        self.rk.release()


_COLOR_TILE_CACHE = {}


def _color_tile(shape, color):
    key = (shape[0], shape[1], color)
    tile = _COLOR_TILE_CACHE.get(key)
    if tile is None:
        tile = np.empty((shape[0], shape[1], 3), dtype=np.uint8)
        tile[:] = color
        _COLOR_TILE_CACHE[key] = tile
    return tile


def draw_front(front, direct_mask, lane_mask):
    if direct_mask.shape != front.shape[:2] or lane_mask.shape != front.shape[:2]:
        raise ValueError("R15 masks must match the front tile size")
    vis = front.copy()
    blended = np.empty_like(vis)
    for mask, color, alpha in ((direct_mask == 1, (30, 170, 30), 0.25),
                               (lane_mask == 1, (0, 0, 255), 0.70),
                               (lane_mask == 2, (0, 255, 255), 0.70)):
        if mask.any():
            cv2.addWeighted(vis, 1.0 - alpha, _color_tile(vis.shape, color),
                            alpha, 0.0, dst=blended)
            cv2.copyTo(blended, mask.astype(np.uint8), vis)
    return vis


class LiveLaneModel(R15Front):
    """Compatibility adapter for the current CARLA four-camera HIL script."""

    def __init__(self, model_path, _legacy_config_path=None, confidence=None,
                 height=640, input_mode="float_nchw", width=640, runtime_factory=None):
        super().__init__(model_path, 0.6 if confidence is None else confidence,
                         height, input_mode, width, runtime_factory)
        self._printed = False

    def infer(self, front):
        direct, lane, timing = super().infer(front)
        if not self._printed:
            print("[R15 first frame] Front=%s direct=%s lane=%s stats=%s" %
                  (front.shape, direct.shape, lane.shape, timing), flush=True)
            self._printed = True
        timing["inference_ms"] = timing["infer_ms"]
        return (direct, lane), timing


def draw_lane_mask(front, masks, _detections=None):
    direct, lane = masks
    return draw_front(front, direct, lane), {
        "direct_area": int(np.count_nonzero(direct)),
        "solid_line": int(np.count_nonzero(lane == 1)),
        "dashed_line": int(np.count_nonzero(lane == 2)),
    }
