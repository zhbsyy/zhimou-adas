"""HSV traffic-light state after YOLO. Input boxes are original-image xyxy.

Dependencies: numpy, OpenCV. Forced-mode probabilities are normalized heuristic
scores, not calibrated probabilities of a correct real-world light state.
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
    min_mean_value: float = 175.0
    min_component_aspect: float = 0.45
    min_circularity: float = 0.45
    roi_padding: float = 0.25
    max_center_outside: float = 0.20
    min_pixels: int = 4
    min_box_side: int = 5
    min_area_ratio: float = 0.003
    min_contrast: float = 15.0
    ambiguity_ratio: float = 0.55
    max_roi_side: int = 192
    decision_mode: str = "conservative"
    candidates: tuple = ("red", "yellow", "green")
    force_hue_sigma: float = 8.0
    force_top_fraction: float = 0.05


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
    duplicate_lamp_distance: float = 0.60
    force_smoothing_seconds: float = 0.15


@dataclass
class ColorResult:
    color: str = "unknown"
    score: float = 0.0
    scores: dict = field(default_factory=dict)
    reason: str = "no_color"
    roi_size: tuple = (0, 0)
    lamp_box: tuple = ()
    probabilities: dict = field(default_factory=dict)
    evidence_strength: float = 0.0
    forced_choice: bool = False


def classify_forced(frame, box, cfg):
    """Fixed-candidate argmax for any valid ROI, even tiny/dim/white ROIs.

    Soft hue likelihoods use circular distance to each existing hue interval.
    Saturation/brightness weight colored evidence continuously; there is no
    component area, circularity, contrast or hard S/V rejection in this mode.
    An evidence-free ROI gets uniform probabilities and an explicit tie choice.
    No padding: nearby buildings/lamps must not enter merely to force a color.
    """
    x1, y1, x2, y2 = box
    roi = frame[y1:y2, x1:x2]
    h, w = roi.shape[:2]
    if max(h, w) > cfg.max_roi_side:
        scale = cfg.max_roi_side / float(max(h, w))
        roi = cv2.resize(roi, (max(1, round(w * scale)), max(1, round(h * scale))),
                         interpolation=cv2.INTER_AREA)
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV).astype(np.float32)
    hue = hsv[:, :, 0]
    saturation, value = hsv[:, :, 1] / 255.0, hsv[:, :, 2] / 255.0
    weight = saturation ** 1.5 * value ** 1.5
    count = max(1, int(math.ceil(hue.size * cfg.force_top_fraction)))
    ranges = {"red": cfg.red_ranges, "yellow": cfg.yellow_ranges, "green": cfg.green_ranges}
    scores = {}
    for color in cfg.candidates:
        distance = np.full(hue.shape, 90.0, np.float32)
        for lo, hi in ranges[color]:
            dlo, dhi = np.abs(hue - lo), np.abs(hue - hi)
            d = np.minimum(np.minimum(dlo, 180.0 - dlo), np.minimum(dhi, 180.0 - dhi))
            d[(hue >= lo) & (hue <= hi)] = 0
            distance = np.minimum(distance, d)
        response = (np.exp(-0.5 * (distance / cfg.force_hue_sigma) ** 2) * weight).ravel()
        scores[color] = float(np.partition(response, response.size - count)[-count:].mean())
    total = sum(scores.values())
    probabilities = ({c: s / total for c, s in scores.items()} if total > 0 else
                     {c: 1.0 / len(cfg.candidates) for c in cfg.candidates})
    winner = max(cfg.candidates, key=lambda c: probabilities[c])
    return ColorResult(color=winner, score=probabilities[winner], scores=scores,
                       reason="forced_argmax" if total > 0 else "forced_no_color_evidence",
                       roi_size=(w, h), probabilities=probabilities,
                       evidence_strength=max(scores.values()), forced_choice=True)


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

    Handles both red hue ranges and horizontal/vertical layouts.
    Default component geometry is intended for round vehicle lamps.
    Conservative mode rejects dark, tiny, white or conflicting lamps. Forced
    mode instead returns an argmax candidate for every valid nonempty ROI.
    """
    cfg = config or ColorConfig()
    box = clipped_box(box, frame.shape)
    if box is None:
        return ColorResult(reason="invalid_box")
    if cfg.decision_mode == "forced":
        return classify_forced(frame, box, cfg)
    x1, y1, x2, y2 = box
    h, w = y2 - y1, x2 - x1
    result = ColorResult(roi_size=(w, h))
    if min(h, w) < cfg.min_box_side:
        result.reason = "too_small"
        return result
    # Partial face boxes often cut through the active lamp. Recover nearby
    # pixels, but require its center to belong to the original detection.
    padding = round(max(h, w) * cfg.roi_padding)
    rx1, ry1, rx2, ry2 = clipped_box((x1 - padding, y1 - padding,
                                     x2 + padding, y2 + padding), frame.shape)
    roi = frame[ry1:ry2, rx1:rx2]
    rh, rw = roi.shape[:2]
    scale = 1.0
    if max(rh, rw) > cfg.max_roi_side:
        scale = cfg.max_roi_side / float(max(rh, rw))
        roi = cv2.resize(roi, (max(1, round(rw * scale)), max(1, round(rh * scale))),
                         interpolation=cv2.INTER_AREA)
    sx, sy = roi.shape[1] / rw, roi.shape[0] / rh
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    n_pixels = hsv.shape[0] * hsv.shape[1]
    ranges = {"red": cfg.red_ranges, "yellow": cfg.yellow_ranges,
              "green": cfg.green_ranges}
    component_boxes = {}
    for color, hue_ranges in ranges.items():
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lo, hi in hue_ranges:
            mask |= cv2.inRange(hsv, (lo, cfg.min_saturation, cfg.min_value),
                                (hi, 255, 255))
        # Avoid an opening filter: it can erase small distant lamp pixels.
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        best = 0.0
        best_box = ()
        for component in range(1, count):
            area = int(stats[component, cv2.CC_STAT_AREA])
            if area < max(cfg.min_pixels, cfg.min_area_ratio * n_pixels):
                continue
            cw = int(stats[component, cv2.CC_STAT_WIDTH])
            ch = int(stats[component, cv2.CC_STAT_HEIGHT])
            if min(cw, ch) / max(cw, ch) < cfg.min_component_aspect:
                continue
            cx = int(stats[component, cv2.CC_STAT_LEFT])
            cy = int(stats[component, cv2.CC_STAT_TOP])
            lamp_box = (rx1 + cx / sx, ry1 + cy / sy,
                        rx1 + (cx + cw) / sx, ry1 + (cy + ch) / sy)
            center_x = (lamp_box[0] + lamp_box[2]) / 2
            center_y = (lamp_box[1] + lamp_box[3]) / 2
            tolerance = min(h, w) * cfg.max_center_outside
            if not (x1 - tolerance <= center_x <= x2 + tolerance and
                    y1 - tolerance <= center_y <= y2 + tolerance):
                continue
            region = labels == component
            value = float(hsv[:, :, 2][region].mean())
            saturation = float(hsv[:, :, 1][region].mean())
            if value < cfg.min_mean_value:
                continue
            contours, _ = cv2.findContours(region.astype(np.uint8), cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
            contour = max(contours, key=cv2.contourArea)
            perimeter = cv2.arcLength(contour, True)
            circularity = 4 * math.pi * cv2.contourArea(contour) / max(perimeter ** 2, 1)
            if circularity < cfg.min_circularity:
                continue
            ring = cv2.dilate(region.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
            ring &= ~region
            # Exclude other colored lamps from local background comparison.
            background = hsv[:, :, 2][ring & (mask == 0)]
            if not background.size:
                continue
            contrast = value - float(np.median(background))
            if contrast < cfg.min_contrast:
                continue
            score = (area / float(n_pixels)) * (saturation / 255.0) * (value / 255.0)
            score *= min(1.0, contrast / 60.0)
            if score > best:
                best, best_box = score, lamp_box
        result.scores[color] = round(best, 6)
        component_boxes[color] = best_box
    ranked = sorted(result.scores.items(), key=lambda item: item[1], reverse=True)
    color, winner = ranked[0]
    result.score = float(winner)
    if winner <= 0:
        result.reason = "no_bright_colored_component"
    elif ranked[1][1] >= winner * cfg.ambiguity_ratio:
        result.reason = "ambiguous_colors"
    else:
        result.color = color
        result.reason = "ok"
        result.lamp_box = component_boxes[color]
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
    lamp_box: tuple = ()
    color_probabilities: dict = field(default_factory=dict)
    probability_time: float = -math.inf

    def observe_forced(self, raw, timestamp, cfg, candidates):
        """EMA over this target only; returns a candidate on the first frame."""
        no_evidence = raw.evidence_strength == 0.0
        if not self.color_probabilities:
            self.color_probabilities = dict(raw.probabilities)
        elif timestamp > self.probability_time:
            dt = timestamp - self.probability_time
            alpha = 1.0 if cfg.force_smoothing_seconds == 0 else -math.expm1(-dt / cfg.force_smoothing_seconds)
            self.color_probabilities = {c: (1 - alpha) * self.color_probabilities[c] + alpha * raw.probabilities[c]
                                        for c in candidates}
        # No-color frames have a uniform distribution: previous preference
        # decays rather than accumulating invented confidence. History belongs
        # only to this track; update() expires tracks using the existing TTL.
        self.probability_time = timestamp
        best = max(self.color_probabilities.values())
        tied = [c for c in candidates if best - self.color_probabilities[c] <= 1e-12]
        self.stable = self.stable if self.stable in tied else tied[0]
        return self.stable, "forced_no_evidence" if no_evidence else "forced"

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
    def __init__(self, color_config=None, state_config=None, classifier=None):
        self.color_config = color_config or ColorConfig()
        self.classifier = classifier or classify_color
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
                current.append((det_index, det, box, self.classifier(frame, box, self.color_config)))
        # Forced ROI votes have no verified lamp center. Do not merge distinct
        # lights by an invented emitter position; YOLO NMS still applies upstream.
        if self.color_config.decision_mode != "forced":
            current = self._merge_duplicates(current, frame.shape)
        # Geometric assignment does not use color: switching must preserve IDs.
        candidates = []
        for index, (_, _, box, raw) in enumerate(current):
            bw, bh = box[2] - box[0], box[3] - box[1]
            center = np.array(((box[0] + box[2]) / 2, (box[1] + box[3]) / 2))
            for track_id, track in self.tracks.items():
                old = track.box
                ow, oh = old[2] - old[0], old[3] - old[1]
                area_ratio = max(bw * bh / (ow * oh), ow * oh / (bw * bh))
                lamp_match = bool(raw.lamp_box and track.lamp_box and
                                  self._same_lamp(raw.lamp_box, track.lamp_box))
                if area_ratio > cfg.max_area_ratio and not lamp_match:
                    continue
                old_center = np.array(((old[0] + old[2]) / 2, (old[1] + old[3]) / 2))
                distance = float(np.linalg.norm(center - old_center)) / max(math.hypot(ow, oh), 1)
                overlap = box_iou(box, old)
                if lamp_match or overlap >= cfg.min_iou or distance <= cfg.max_center_distance:
                    priority = 2 if raw.color != "unknown" else 0
                    candidates.append((priority + int(lamp_match) + overlap - 0.25 * distance, index, track_id))
        assigned, used = {}, set()
        for _, index, track_id in sorted(candidates, reverse=True):
            if index not in assigned and track_id not in used:
                assigned[index] = track_id
                used.add(track_id)
        results = []
        for index, (det_index, det, box, raw) in enumerate(current):
            if index in assigned:
                track = self.tracks[assigned[index]]
            else:
                track = _Track(self.next_id, box, timestamp)
                self.tracks[self.next_id] = track
                self.next_id += 1
            track.box, track.last_seen = box, timestamp
            if raw.lamp_box:
                track.lamp_box = raw.lamp_box
            if raw.forced_choice:
                stable, status = track.observe_forced(raw, timestamp, cfg, self.color_config.candidates)
            else:
                stable, status = track.observe(raw.color, timestamp, cfg)
            probabilities = dict(track.color_probabilities) if raw.forced_choice else {}
            ordered = sorted(probabilities.values(), reverse=True)
            results.append({"detection_index": det_index, "track_id": track.track_id,
                            "box": list(box), "det_confidence": float(det[4]),
                            "raw_color": raw.color, "stable_color": stable,
                            "status": status, "score": raw.score, "scores": raw.scores,
                            "reason": raw.reason, "roi_size": list(raw.roi_size),
                            "diagnostics": getattr(raw, "diagnostics", {}),
                            "lamp_box": list(raw.lamp_box or track.lamp_box),
                            "forced_choice": raw.forced_choice, "evidence_strength": raw.evidence_strength,
                            "raw_probabilities": raw.probabilities, "probabilities": probabilities,
                            "candidate_margin": ordered[0] - ordered[1] if len(ordered) > 1 else None,
                            "probability_kind": "normalized_heuristic_not_calibrated" if raw.forced_choice else None})
        # A held fragment and a current full head can refer to the same emitter.
        # Prefer current evidence; keep the hidden track alive until its TTL.
        accepted = []
        for item in sorted(results, key=lambda item: item["status"] == "held"):
            duplicate = any(item["stable_color"] != "unknown" and
                            item["stable_color"] == other["stable_color"] and
                            item["lamp_box"] and other["lamp_box"] and
                            self._same_lamp(item["lamp_box"], other["lamp_box"])
                            for other in accepted)
            if not duplicate:
                accepted.append(item)
        return sorted(accepted, key=lambda item: item["detection_index"])

    def _same_lamp(self, a, b):
        center_a = np.array(((a[0] + a[2]) / 2, (a[1] + a[3]) / 2))
        center_b = np.array(((b[0] + b[2]) / 2, (b[1] + b[3]) / 2))
        diagonal = max(math.hypot(a[2] - a[0], a[3] - a[1]),
                       math.hypot(b[2] - b[0], b[3] - b[1]))
        return np.linalg.norm(center_a - center_b) <= self.state_config.duplicate_lamp_distance * diagonal

    def _merge_duplicates(self, current, shape):
        """Associate duplicate face boxes by the SAME active lamp, not box IoU.

        Different neighboring active lamps remain separate even when their
        detection boxes overlap. No image coordinates are hard-coded.
        """
        groups = []
        known = [item for item in current if item[3].color != "unknown"]
        unknown = [item for item in current if item[3].color == "unknown"]
        for item in sorted(known, key=lambda item: item[3].score, reverse=True):
            raw = item[3]
            lamp = raw.lamp_box
            center = np.array(((lamp[0] + lamp[2]) / 2, (lamp[1] + lamp[3]) / 2))
            diagonal = math.hypot(lamp[2] - lamp[0], lamp[3] - lamp[1])
            match = None
            for group in groups:
                other = group[0][3]
                if other.color != raw.color:
                    continue
                other_lamp = other.lamp_box
                other_center = np.array(((other_lamp[0] + other_lamp[2]) / 2,
                                         (other_lamp[1] + other_lamp[3]) / 2))
                other_diagonal = math.hypot(other_lamp[2] - other_lamp[0], other_lamp[3] - other_lamp[1])
                if np.linalg.norm(center - other_center) <= self.state_config.duplicate_lamp_distance * max(diagonal, other_diagonal):
                    match = group
                    break
            if match is None:
                groups.append([item])
            else:
                match.append(item)
        merged = []
        for group in groups:
            index, det, _, _ = min(group, key=lambda item: item[0])
            raw = group[0][3]
            all_boxes = [item[2] for item in group] + [item[3].lamp_box for item in group]
            box = clipped_box((min(b[0] for b in all_boxes), min(b[1] for b in all_boxes),
                               max(b[2] for b in all_boxes), max(b[3] for b in all_boxes)), shape)
            merged.append((index, det, box, raw))
        # Unknown boxes overlapping an accepted head should not steal its ID.
        for item in unknown:
            box = item[2]
            area = (box[2] - box[0]) * (box[3] - box[1])
            overlaps = False
            for _, _, other, _ in merged:
                intersection = (max(0, min(box[2], other[2]) - max(box[0], other[0])) *
                                max(0, min(box[3], other[3]) - max(box[1], other[1])))
                if intersection / max(area, 1) >= 0.5:
                    overlaps = True
                    break
            if not overlaps:
                merged.append(item)
        return sorted(merged, key=lambda item: item[0])


def validate_config(color, state):
    if color.decision_mode not in ("conservative", "forced"):
        raise ValueError("decision_mode must be conservative or forced")
    if (not isinstance(color.candidates, (list, tuple)) or len(color.candidates) not in (2, 3) or
            len(set(color.candidates)) != len(color.candidates) or
            not set(color.candidates) <= {"red", "yellow", "green"}):
        raise ValueError("candidates must be 2 or 3 distinct red/yellow/green names")
    if color.force_hue_sigma <= 0 or not 0 < color.force_top_fraction <= 1 or state.force_smoothing_seconds < 0:
        raise ValueError("Invalid forced classification/smoothing parameters")
    for ranges in (color.red_ranges, color.yellow_ranges, color.green_ranges):
        if not ranges or any(len(pair) != 2 or not 0 <= pair[0] <= pair[1] <= 179 for pair in ranges):
            raise ValueError("Hue ranges must be ordered pairs between 0 and 179")
    if not 0 <= color.min_saturation <= 255 or not 0 <= color.min_value <= 255:
        raise ValueError("S/V thresholds must be between 0 and 255")
    if (not 0 <= color.min_mean_value <= 255 or not 0 <= color.min_circularity <= 1 or
            not 0 < color.min_component_aspect <= 1 or not 0 <= color.roi_padding <= 1 or
            not 0 <= color.max_center_outside <= 1):
        raise ValueError("Invalid lamp geometry/brightness thresholds")
    if (color.min_pixels < 1 or color.min_box_side < 1 or color.max_roi_side < 8 or
            not 0 < color.min_area_ratio <= 1 or not 0 < color.ambiguity_ratio < 1 or
            not 0 <= color.min_contrast <= 255):
        raise ValueError("Invalid color component thresholds")
    if (state.confirm_samples < 1 or state.confirm_seconds < 0 or state.max_sample_gap <= 0 or
            state.hold_seconds < 0 or state.track_ttl <= 0 or not 0 <= state.min_iou <= 1 or
            state.max_center_distance < 0 or state.max_area_ratio < 1 or
            state.duplicate_lamp_distance <= 0):
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


def draw_results(frame, detections, names, results, title="", display="clean", show_unknown=False):
    """Draw on a copy, after all color crops have been read."""
    vis = frame.copy()
    palette = {"red": (0, 0, 255), "yellow": (0, 230, 255),
               "green": (0, 220, 0), "unknown": (180, 180, 180)}
    rows = []
    for item in results:
        if display != "debug" and item["stable_color"] == "unknown" and not show_unknown:
            continue
        rows.append((item["box"], item, ""))
    if display in ("all", "debug"):
        for index, det in enumerate(detections):
            class_id = int(det[5])
            label = names[class_id] if 0 <= class_id < len(names) else str(class_id)
            normalized = str(label).lower().replace("_", " ").replace("-", " ")
            if normalized in ("traffic light", "traffic lights", "trafficlight", "信号灯", "交通灯"):
                continue
            rows.append((det[:4], None, "%s %.2f" % (label, det[4])))
    for coordinates, item, label in rows:
        box = clipped_box(coordinates, frame.shape)
        if box is None:
            continue
        x1, y1, x2, y2 = box
        color = (210, 160, 80)
        if item is not None:
            color = palette[item["stable_color"]]
            label = item["stable_color"].upper()
            if item.get("forced_choice"):
                label += "*"
            if item["status"] == "held":
                label += " ?"
            if display == "debug":
                label = "TL #%d %s raw:%s %s" % (item["track_id"], item["stable_color"].upper(),
                                                 item["raw_color"], item["status"])
        cv2.rectangle(vis, (x1, y1), (x2 - 1, y2 - 1), color, 2)
        font_scale = max(0.55, frame.shape[0] / 1080 * 0.70)
        thickness = 1 if frame.shape[0] < 900 else 2
        (tw, th), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)
        text_y = max(th + 5, y1 - 5)
        cv2.rectangle(vis, (x1, text_y - th - 3),
                      (min(vis.shape[1] - 1, x1 + tw + 4), text_y + baseline), (15, 15, 15), -1)
        cv2.putText(vis, label, (x1 + 2, text_y), cv2.FONT_HERSHEY_SIMPLEX, font_scale,
                    color, thickness, cv2.LINE_AA)
    if title and display == "debug":
        cv2.rectangle(vis, (0, 0), (vis.shape[1], 26), (15, 15, 15), -1)
        cv2.putText(vis, title, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (255, 255, 255), 1, cv2.LINE_AA)
    return vis
