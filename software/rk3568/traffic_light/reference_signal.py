"""Adapt the user's traffic_light reference to the existing RK summary API.

The provided component/padding/dedup/association/confirmation implementation is
retained. Its forced soft-score formula supplies weak-color candidates, with
explicit evidence checks before entering the ordinary two-sample state machine.
"""
from dataclasses import replace
from pathlib import Path
import math

import cv2
import numpy as np

from signal_reference import traffic_light_color as reference


class HybridClassifier:
    def __init__(self):
        source = Path(__file__).with_name('signal_reference')
        self.soft, _ = reference.load_config(source / 'hsv_config_forced_ryg.json')
        self.calls = {'conservative': 0, 'soft_fallback': 0, 'soft_accepted': 0}

    def __call__(self, frame, box, cfg):
        self.calls['conservative'] += 1
        raw = reference.classify_color(frame, box, cfg)
        detail = {'method': 'reference_conservative', 'reference_reason': raw.reason,
                  'soft_candidate': None, 'soft_weights': {},
                  'weights_are_calibrated_probabilities': False}
        raw.diagnostics = detail
        if raw.color != 'unknown':
            return raw
        self.calls['soft_fallback'] += 1
        soft = reference.classify_forced(frame, box, self.soft)
        ordered = sorted(soft.probabilities.values(), reverse=True)
        margin = ordered[0] - ordered[1]
        detail.update(method='reference_soft_candidate', soft_candidate=soft.color,
                      soft_weights=dict(soft.probabilities), soft_scores=dict(soft.scores),
                      evidence_strength=soft.evidence_strength, candidate_margin=margin,
                      decision_gate='not_evaluated')
        if raw.reason == 'ambiguous_colors':
            detail['decision_gate'] = 'conflicting_lamps'
            return raw
        if soft.evidence_strength < .003:
            detail['decision_gate'] = 'no_or_weak_chroma_evidence'
            return raw
        if soft.score < .65 or margin < .35:
            detail['decision_gate'] = 'ambiguous_soft_weights'
            return raw
        lamp, reason = self.lamp_evidence(frame, box, soft.color)
        detail['decision_gate'] = reason
        if lamp is None:
            return raw
        self.calls['soft_accepted'] += 1
        accepted = reference.ColorResult(color=soft.color, score=soft.score, scores=soft.scores,
            reason='soft_lamp_evidence', roi_size=raw.roi_size, lamp_box=lamp,
            probabilities=soft.probabilities, evidence_strength=soft.evidence_strength,
            forced_choice=False)
        accepted.diagnostics = detail
        return accepted

    def lamp_evidence(self, frame, box, color):
        """Native colored support, lamp-like geometry and local contrast.

        No blue-to-green remapping; no enlarging tiny ROIs to invent evidence.
        Yellow keeps the reference's brightness floor to reject matte housings.
        """
        x1, y1, x2, y2 = box
        h, w = y2-y1, x2-x1
        if min(h, w) < 3:
            return None, 'native_roi_too_small'
        roi = frame[y1:y2, x1:x2]
        if max(h, w) > 192:
            scale = 192 / max(h, w)
            roi = cv2.resize(roi, (max(1, round(w*scale)), max(1, round(h*scale))), interpolation=cv2.INTER_AREA)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], np.uint8)
        ranges = {'red': self.soft.red_ranges, 'yellow': self.soft.yellow_ranges,
                  'green': self.soft.green_ranges}
        for lo, hi in ranges[color]:
            mask |= cv2.inRange(hsv, (lo, 30, 55), (hi, 255, 255))
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        best, best_box = 0., None
        for i in range(1, count):
            area = int(stats[i, cv2.CC_STAT_AREA])
            if area < 3 or area / mask.size > .40:
                continue
            cw, ch = int(stats[i, cv2.CC_STAT_WIDTH]), int(stats[i, cv2.CC_STAT_HEIGHT])
            if min(cw, ch) / max(cw, ch) < .35:
                continue
            region = labels == i
            contours, _ = cv2.findContours(region.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contour = max(contours, key=cv2.contourArea)
            perimeter = cv2.arcLength(contour, True)
            if 4*math.pi*cv2.contourArea(contour) / max(perimeter**2, 1) < .30:
                continue
            value = float(hsv[:, :, 2][region].mean())
            if color == 'yellow' and value < 175:
                continue
            ring = (cv2.dilate(region.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0) & ~region
            background = hsv[:, :, 2][ring & (mask == 0)]
            if not background.size or value-float(np.median(background)) < 8:
                continue
            score = area*value
            if score > best:
                best = score
                left, top = stats[i, cv2.CC_STAT_LEFT], stats[i, cv2.CC_STAT_TOP]
                sx, sy = w / roi.shape[1], h / roi.shape[0]
                best_box = (x1+left*sx, y1+top*sy, x1+(left+cw)*sx, y1+(top+ch)*sy)
        return (best_box, 'supported_native_lamp') if best_box else (None, 'no_supported_native_lamp')


def make_engine():
    source = Path(__file__).with_name('signal_reference')
    color, state = reference.load_config(source / 'hsv_config.json')
    # Keep the reference brightness/geometry filters against yellow housings;
    # soft fallback handles weak, localized colored emitters separately.
    color = replace(color, min_saturation=60, min_value=90, min_pixels=3, min_box_side=4,
                    min_contrast=12)
    state = replace(state, hold_seconds=0)
    return reference.TrafficLightColorEngine(color, state, classifier=HybridClassifier())


def make_forced_rg_engine():
    """Use the supplied reference's red/green argmax for every valid box.

    Scores are relative HSV evidence, not calibrated correctness probabilities.
    Zero smoothing makes the displayed color the current ROI's highest score;
    an evidence-free tie follows the supplied candidate order (red first).
    """
    source = Path(__file__).with_name('signal_reference')
    color, state = reference.load_config(source / 'hsv_config_forced_rg.json')
    state = replace(state, force_smoothing_seconds=0)
    return reference.TrafficLightColorEngine(color, state)


def classify_pixel_ratio(frame, box, cfg):
    """Count native red/green pixels strictly inside the detector's clipped ROI."""
    x1, y1, x2, y2 = box
    roi = frame[y1:y2, x1:x2]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    counts = {}
    for name, ranges in (('red', cfg.red_ranges), ('green', cfg.green_ranges)):
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in ranges:
            mask |= cv2.inRange(hsv, (lo, cfg.min_saturation, cfg.min_value), (hi, 255, 255))
        counts[name] = int(cv2.countNonZero(mask))
    pixels = int(hsv.shape[0] * hsv.shape[1])
    scores = {name: count / pixels for name, count in counts.items()}
    total = counts['red'] + counts['green']
    weights = ({name: count / total for name, count in counts.items()} if total else
               {'red': .5, 'green': .5})
    winner = 'green' if counts['green'] > counts['red'] else 'red'
    result = reference.ColorResult(color=winner, score=weights[winner], scores=scores,
        reason='pixel_ratio_argmax' if total else 'no_red_green_pixels_tie_red',
        roi_size=(roi.shape[1], roi.shape[0]), probabilities=weights,
        evidence_strength=total / pixels, forced_choice=True)
    result.diagnostics = {'method': 'native_roi_red_green_pixel_ratio',
        'red_pixels': counts['red'], 'green_pixels': counts['green'], 'roi_pixels': pixels,
        'red_ratio': scores['red'], 'green_ratio': scores['green'],
        'min_saturation': cfg.min_saturation, 'min_value': cfg.min_value,
        'red_hue_ranges': cfg.red_ranges, 'green_hue_ranges': cfg.green_ranges,
        'tie_policy': 'red', 'evidence_free': total == 0,
        'resized': False, 'padding': 0, 'weights_are_calibrated_probabilities': False}
    return result


class PixelRatioEngine(reference.TrafficLightColorEngine):
    def update(self, frame, detections, names, timestamp):
        results = super().update(frame, detections, names, timestamp)
        for result in results:
            # Count the current ROI. A 0:0 tie must not inherit yesterday's or
            # the previous frame's green just because the track survived.
            result['stable_color'] = result['raw_color']
            result['probabilities'] = dict(result['raw_probabilities'])
            result['status'] = 'pixel_ratio'
            result['probability_kind'] = 'relative_pixel_counts_not_calibrated'
        return results


def make_pixel_ratio_engine():
    source = Path(__file__).with_name('signal_reference')
    color, state = reference.load_config(source / 'hsv_config_forced_rg.json')
    color = replace(color, min_saturation=50, min_value=60)
    state = replace(state, force_smoothing_seconds=0)
    return PixelRatioEngine(color, state, classifier=classify_pixel_ratio)
