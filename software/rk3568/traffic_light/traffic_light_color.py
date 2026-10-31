"""HSV traffic-light state after YOLO. Input boxes are original-image xyxy.

Dependencies: numpy, OpenCV. Scores are rule scores, not probabilities.
Tracking is a lightweight geometric association, not a general tracking model.
"""
import math
from dataclasses import dataclass, field

import cv2
import numpy as np


@dataclass
class ColorConfig:
    red_ranges: tuple = ((0, 10), (170, 179))
    yellow_ranges: tuple = ((15, 35),)
    green_ranges: tuple = ((40, 95),)
    min_saturation: int = 90
    min_value: int = 120
    min_pixels: int = 4
    min_box_side: int = 5
    min_area_ratio: float = 0.003
    min_contrast: float = 15.0
    ambiguity_ratio: float = 0.55
    max_roi_side: int = 192


@dataclass
class StateConfig:
    confirm_samples: int = 2
    confirm_seconds: float = 0.05
    max_sample_gap: float = 0.75
    hold_seconds: float = 0.30
    track_ttl: float = 0.75
    min_iou: float = 0.15
    max_center_distance: float = 0.65
    max_area_ratio: float = 2.5


@dataclass
class ColorResult:
    color: str = "unknown"
    score: float = 0.0
    scores: dict = field(default_factory=dict)
    reason: str = "no_color"
    roi_size: tuple = (0, 0)
    diagnostics: dict = field(default_factory=dict)


def clipped_box(box, shape):
    coords = np.asarray(box, dtype=float)
    if coords.shape != (4,) or not np.isfinite(coords).all():
        return None
    h, w = shape[:2]
    x1, y1 = np.floor(coords[:2]).astype(int)
    x2, y2 = np.ceil(coords[2:]).astype(int)
    x1, x2 = np.clip((x1, x2), 0, w)
    y1, y2 = np.clip((y1, y2), 0, h)
    return (int(x1), int(y1), int(x2), int(y2)) if x2 > x1 and y2 > y1 else None


def classify_color(frame, box, config=None):
    """Find concentrated bright colored components inside an unannotated ROI.

    Handles both red hue ranges and arrow-shaped lamps; no fixed lamp order.
    Dark, tiny, white overexposed or conflicting lamps return unknown.
    """
    cfg = config or ColorConfig()
    box = clipped_box(box, frame.shape)
    if box is None:
        return ColorResult(reason="invalid_box")
    x1, y1, x2, y2 = box
    roi = frame[y1:y2, x1:x2]
    h, w = roi.shape[:2]
    result = ColorResult(roi_size=(w, h))
    if min(h, w) < cfg.min_box_side:
        result.reason = "too_small"
        return result
    if max(h, w) > cfg.max_roi_side:
        scale = cfg.max_roi_side / float(max(h, w))
        roi = cv2.resize(roi, (max(1, round(w * scale)), max(1, round(h * scale))),
                         interpolation=cv2.INTER_AREA)
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    n_pixels = hsv.shape[0] * hsv.shape[1]
    result.diagnostics = {
        "analysis_size": [int(hsv.shape[1]), int(hsv.shape[0])],
        "thresholds": {"min_saturation": cfg.min_saturation,
                       "min_value": cfg.min_value, "min_pixels": cfg.min_pixels,
                       "min_area_ratio": cfg.min_area_ratio,
                       "min_contrast": cfg.min_contrast},
        "hue_histogram_10deg_bins": np.bincount(
            (hsv[:, :, 0] // 10).ravel(), minlength=18).tolist(),
        "hue_count_chroma_floor": 20,
        "colors": {},
    }
    ranges = {"red": cfg.red_ranges, "yellow": cfg.yellow_ranges,
              "green": cfg.green_ranges}
    for color, hue_ranges in ranges.items():
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        hue_mask = np.zeros_like(mask)
        for lo, hi in hue_ranges:
            hue_mask |= cv2.inRange(hsv[:, :, 0], lo, hi)
            mask |= cv2.inRange(hsv, (lo, cfg.min_saturation, cfg.min_value),
                                (hi, 255, 255))
        # OpenCV assigns hue=0 to gray/white pixels. Exclude these from the
        # diagnostic hue count, without changing the actual classification mask.
        hue_mask &= cv2.inRange(hsv[:, :, 1], 20, 255)
        # Avoid an opening filter: it can erase small distant lamp pixels.
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        detail = {"hue_pixels": int(cv2.countNonZero(hue_mask)),
                  "hsv_pixels": int(cv2.countNonZero(mask)),
                  "components": int(count-1), "largest_component": 0,
                  "rejected_area": 0, "rejected_shape": 0,
                  "rejected_background": 0, "rejected_contrast": 0,
                  "accepted_components": 0, "max_contrast": None}
        best = 0.0
        for component in range(1, count):
            area = int(stats[component, cv2.CC_STAT_AREA])
            detail["largest_component"] = max(detail["largest_component"], area)
            if area < max(cfg.min_pixels, cfg.min_area_ratio * n_pixels):
                detail["rejected_area"] += 1
                continue
            cw = int(stats[component, cv2.CC_STAT_WIDTH])
            ch = int(stats[component, cv2.CC_STAT_HEIGHT])
            if min(cw, ch) / max(cw, ch) < 0.15:
                detail["rejected_shape"] += 1
                continue
            region = labels == component
            value = float(hsv[:, :, 2][region].mean())
            saturation = float(hsv[:, :, 1][region].mean())
            ring = cv2.dilate(region.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
            ring &= ~region
            # Exclude other colored lamps from local background comparison.
            background = hsv[:, :, 2][ring & (mask == 0)]
            if not background.size:
                detail["rejected_background"] += 1
                continue
            contrast = value - float(np.median(background))
            detail["max_contrast"] = round(max(contrast, detail["max_contrast"]
                                               if detail["max_contrast"] is not None
                                               else contrast), 2)
            if contrast < cfg.min_contrast:
                detail["rejected_contrast"] += 1
                continue
            detail["accepted_components"] += 1
            score = (area / float(n_pixels)) * (saturation / 255.0) * (value / 255.0)
            score *= min(1.0, contrast / 60.0)
            best = max(best, score)
        result.scores[color] = round(best, 6)
        result.diagnostics["colors"][color] = detail
    ranked = sorted(result.scores.items(), key=lambda item: item[1], reverse=True)
    color, winner = ranked[0]
    result.score = float(winner)
    if winner <= 0:
        result.reason = "no_bright_colored_component"
        details = list(result.diagnostics["colors"].values())
        result.diagnostics["failure_stage"] = (
            "hue_outside_signal_ranges" if not sum(d["hue_pixels"] for d in details)
            else "saturation_or_value_below_threshold" if not sum(d["hsv_pixels"] for d in details)
            else "component_area_shape_or_contrast")
    elif ranked[1][1] >= winner * cfg.ambiguity_ratio:
        result.reason = "ambiguous_colors"
    else:
        result.color = color
        result.reason = "ok"
    return result


def box_iou(a, b):
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    intersection = max(0, x2 - x1) * max(0, y2 - y1)
    aa = (a[2] - a[0]) * (a[3] - a[1])
    ba = (b[2] - b[0]) * (b[3] - b[1])
    return intersection / max(aa + ba - intersection, 1e-9)


@dataclass
class _Track:
    track_id: int
    box: tuple
    last_seen: float
    stable: str = "unknown"
    last_support: float = -math.inf
    candidate: str = "unknown"
    candidate_since: float = 0.0
    candidate_count: int = 0
    last_candidate: float = -math.inf

    def observe(self, raw, timestamp, cfg):
        if raw != "unknown" and raw == self.stable:
            self.last_support = timestamp
            self.candidate = "unknown"
            self.candidate_count = 0
            return self.stable, "confirmed"
        if timestamp - self.last_support > cfg.hold_seconds:
            self.stable = "unknown"
        if raw == "unknown":
            self.candidate = "unknown"
            self.candidate_count = 0
            return self.stable, "held" if self.stable != "unknown" else "unknown"
        # Contrary evidence invalidates the old state, even if the next sample
        # is unknown; otherwise an old red/green state could be resurrected.
        self.stable = "unknown"
        if raw != self.candidate or timestamp - self.last_candidate > cfg.max_sample_gap:
            self.candidate = raw
            self.candidate_since = timestamp
            self.candidate_count = 1
        elif timestamp > self.last_candidate:
            self.candidate_count += 1
        self.last_candidate = timestamp
        if (self.candidate_count >= cfg.confirm_samples and
                timestamp - self.candidate_since + 1e-9 >= cfg.confirm_seconds):
            self.stable = raw
            self.last_support = timestamp
            self.candidate = "unknown"
            self.candidate_count = 0
            return self.stable, "confirmed"
        # Do not display the old color as current when contrary evidence appears.
        return "unknown", "pending"


class TrafficLightColorEngine:
    """Call once per YOLO result, including empty detections, with source time.

    dets: N x 6 rows [x1,y1,x2,y2,confidence,class_id]; names: class order.
    Only current detections are returned; lost boxes are never drawn as current.
    """
    def __init__(self, color_config=None, state_config=None):
        self.color_config = color_config or ColorConfig()
        self.state_config = state_config or StateConfig()
        validate_config(self.color_config, self.state_config)
        self.tracks = {}
        self.next_id = 1
        self.last_timestamp = -math.inf

    def reset(self):
        self.tracks.clear()
        self.next_id = 1
        self.last_timestamp = -math.inf

    def update(self, frame, detections, names, timestamp, traffic_light_ids=None):
        timestamp = float(timestamp)
        if not math.isfinite(timestamp) or timestamp < self.last_timestamp:
            raise ValueError("Use finite nondecreasing source timestamps; reset on a new clip")
        self.last_timestamp = timestamp
        cfg = self.state_config
        self.tracks = {i: t for i, t in self.tracks.items()
                       if timestamp - t.last_seen <= cfg.track_ttl}
        if traffic_light_ids is None:
            traffic_light_ids = {i for i, name in enumerate(names)
                                 if str(name).strip().lower().replace("_", " ").replace("-", " ")
                                 in ("traffic light", "traffic lights", "trafficlight", "信号灯", "交通灯")}
        dets = np.asarray(detections, dtype=float)
        if dets.size == 0:
            return []
        if dets.ndim != 2 or dets.shape[1] != 6:
            raise ValueError("Expected N x 6 xyxy/confidence/class detections")
        current = []
        for det_index, det in enumerate(dets):
            if not np.isfinite(det).all() or int(det[5]) not in traffic_light_ids:
                continue
            box = clipped_box(det[:4], frame.shape)
            if box:
                current.append((det_index, det, box))
        # Geometric assignment does not use color: switching must preserve IDs.
        candidates = []
        for index, (_, _, box) in enumerate(current):
            bw, bh = box[2] - box[0], box[3] - box[1]
            center = np.array(((box[0] + box[2]) / 2, (box[1] + box[3]) / 2))
            for track_id, track in self.tracks.items():
                old = track.box
                ow, oh = old[2] - old[0], old[3] - old[1]
                area_ratio = max(bw * bh / (ow * oh), ow * oh / (bw * bh))
                if area_ratio > cfg.max_area_ratio:
                    continue
                old_center = np.array(((old[0] + old[2]) / 2, (old[1] + old[3]) / 2))
                distance = float(np.linalg.norm(center - old_center)) / max(math.hypot(ow, oh), 1)
                overlap = box_iou(box, old)
                if overlap >= cfg.min_iou or distance <= cfg.max_center_distance:
                    candidates.append((overlap - 0.25 * distance, index, track_id))
        assigned, used = {}, set()
        for _, index, track_id in sorted(candidates, reverse=True):
            if index not in assigned and track_id not in used:
                assigned[index] = track_id
                used.add(track_id)
        results = []
        for index, (det_index, det, box) in enumerate(current):
            if index in assigned:
                track = self.tracks[assigned[index]]
            else:
                track = _Track(self.next_id, box, timestamp)
                self.tracks[self.next_id] = track
                self.next_id += 1
            track.box, track.last_seen = box, timestamp
            raw = classify_color(frame, box, self.color_config)
            stable, status = track.observe(raw.color, timestamp, cfg)
            results.append({"detection_index": det_index, "track_id": track.track_id,
                            "box": list(box), "det_confidence": float(det[4]),
                            "raw_color": raw.color, "stable_color": stable,
                            "status": status, "score": raw.score, "scores": raw.scores,
                            "reason": raw.reason, "roi_size": list(raw.roi_size),
                            "diagnostics": raw.diagnostics})
        return results


def validate_config(color, state):
    for ranges in (color.red_ranges, color.yellow_ranges, color.green_ranges):
        if not ranges or any(len(pair) != 2 or not 0 <= pair[0] <= pair[1] <= 179 for pair in ranges):
            raise ValueError("Hue ranges must be ordered pairs between 0 and 179")
    if not 0 <= color.min_saturation <= 255 or not 0 <= color.min_value <= 255:
        raise ValueError("S/V thresholds must be between 0 and 255")
    if (color.min_pixels < 1 or color.min_box_side < 1 or color.max_roi_side < 8 or
            not 0 < color.min_area_ratio <= 1 or not 0 < color.ambiguity_ratio < 1 or
            not 0 <= color.min_contrast <= 255):
        raise ValueError("Invalid color component thresholds")
    if (state.confirm_samples < 1 or state.confirm_seconds < 0 or state.max_sample_gap <= 0 or
            state.hold_seconds < 0 or state.track_ttl <= 0 or not 0 <= state.min_iou <= 1 or
            state.max_center_distance < 0 or state.max_area_ratio < 1):
        raise ValueError("Invalid temporal/tracking thresholds")
    for config in (color, state):
        if any(isinstance(value, (int, float)) and not math.isfinite(value)
               for value in vars(config).values()):
            raise ValueError("Config values must be finite")


def load_config(path=None):
    import json
    if not path:
        return ColorConfig(), StateConfig()
    with open(path, encoding="utf-8-sig") as stream:
        data = json.load(stream)
    if set(data) - {"color", "state"}:
        raise ValueError("Config accepts only color and state sections")
    color, state = ColorConfig(**data.get("color", {})), StateConfig(**data.get("state", {}))
    validate_config(color, state)
    return color, state


def draw_results(frame, detections, names, results, title=""):
    """Draw on a copy, after all color crops have been read."""
    vis = frame.copy()
    by_index = {item["detection_index"]: item for item in results}
    palette = {"red": (0, 0, 255), "yellow": (0, 230, 255),
               "green": (0, 220, 0), "unknown": (180, 180, 180)}
    for index, det in enumerate(detections):
        box = clipped_box(det[:4], frame.shape)
        if box is None:
            continue
        x1, y1, x2, y2 = box
        class_id = int(det[5])
        label = names[class_id] if 0 <= class_id < len(names) else str(class_id)
        color = (210, 160, 80)
        if index in by_index:
            item = by_index[index]
            color = palette[item["stable_color"]]
            label = "TL #%d %s raw:%s %s" % (item["track_id"], item["stable_color"].upper(),
                                             item["raw_color"], item["status"])
        else:
            label += " %.2f" % det[4]
        cv2.rectangle(vis, (x1, y1), (x2 - 1, y2 - 1), color, 2)
        cv2.putText(vis, label, (x1, max(16, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    color, 1, cv2.LINE_AA)
    if title:
        cv2.rectangle(vis, (0, 0), (vis.shape[1], 26), (15, 15, 15), -1)
        cv2.putText(vis, title, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (255, 255, 255), 1, cv2.LINE_AA)
    return vis
