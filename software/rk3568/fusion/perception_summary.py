"""CPU summaries from existing unannotated BGR/detections and R15 uint8 masks."""
import cv2
import numpy as np

from traffic_light_color import TrafficLightColorEngine, ColorConfig
from boundary_ratio import extract_boundaries_ratio


def unknown(reason):
    return {"kind": "unknown", "valid": False, "reason": reason, "points": []}


def extract_boundaries(direct, lane, config=None):
    cfg = {"anchor_x": .5, "row_start": .50, "row_end": .94,
           "rows": 24, "min_samples": 5, "dominance": .8, **(config or {})}
    if direct.shape != lane.shape or lane.ndim != 2 or min(lane.shape) < 16:
        raise ValueError("Expected matching 2D direct/lane masks")
    # Summaries are evaluated at a bounded resolution, not full-frame FPS.
    if lane.shape[1] > 448:
        h = max(16, round(lane.shape[0]*448/lane.shape[1]))
        direct = cv2.resize(direct, (448, h), interpolation=cv2.INTER_NEAREST)
        lane = cv2.resize(lane, (448, h), interpolation=cv2.INTER_NEAREST)
    h, w = lane.shape
    if cfg.get("classification", "consensus") == "length_ratio":
        return extract_boundaries_ratio(direct,lane,cfg)
    anchor = int(cfg["anchor_x"]*w)
    points = {"left": [], "right": []}
    rows = np.linspace(int(cfg["row_end"]*(h-1)), int(cfg["row_start"]*(h-1)), cfg["rows"]).astype(int)
    for y in np.unique(rows)[::-1]:
        y0, y1 = max(0, y-1), min(h, y+2)
        road = direct[y0:y1, max(0, anchor-int(.08*w)):min(w, anchor+int(.08*w)+1)]
        if road.size == 0 or np.mean(road == 1) < .25:
            continue
        band = lane[y0:y1]
        marked = np.any((band == 1) | (band == 2), axis=0)
        edges = np.diff(np.pad(marked.astype(np.int8), (1, 1)))
        runs = list(zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)))
        for side in ("left", "right"):
            candidates = [(a, b) for a, b in runs if
                          (int(.03*w) <= anchor-(a+b)/2 <= .47*w if side == "left"
                           else int(.03*w) <= (a+b)/2-anchor <= .47*w)]
            candidates.sort(key=lambda r: abs((r[0]+r[1])/2-anchor))
            if not candidates:
                continue
            a, b = candidates[0]  # nearest line to ego center, never global majority
            x = (a+b)/2
            old = points[side]
            if old:
                last_x, last_y, _ = old[-1]
                predicted = last_x
                if len(old) > 1:
                    prev_x, prev_y, _ = old[-2]
                    predicted += (last_x-prev_x)*(y-last_y)/min(-1, last_y-prev_y)
                if abs(x-predicted) > .08*w+.6*w*abs(y-last_y)/h:
                    continue  # don't jump to a neighboring solid line in a dashed gap
            labels = band[:, a:b]
            solid, dashed = np.count_nonzero(labels == 1), np.count_nonzero(labels == 2)
            label = 1 if solid >= dashed else 2
            if max(solid, dashed)/max(1, solid+dashed) < cfg["dominance"]:
                label = 0
            old.append((float(x), int(y), label))
    out = {}
    for side, found in points.items():
        if len(found) < cfg["min_samples"]:
            out[side] = unknown("insufficient_ego_boundary_samples")
            out[side]["samples"] = len(found)
            continue
        labels = np.array([v[2] for v in found])
        solid, dashed = float(np.mean(labels == 1)), float(np.mean(labels == 2))
        kind = "solid" if solid >= cfg["dominance"] else "dashed" if dashed >= cfg["dominance"] else "mixed"
        out[side] = {"kind": kind, "valid": kind in ("solid", "dashed"),
                     "reason": "ok" if kind != "mixed" else "conflicting_boundary_classes",
                     "samples": len(found), "solid_fraction": round(solid, 3),
                     "dashed_fraction": round(dashed, 3),
                     "points": [[round(x/w, 4), round(y/h, 4)] for x,y,_ in found]}
    return out


class BoundaryTracker:
    def __init__(self, confirm=2, max_gap=.75, dashed_exit=.3):
        self.confirm, self.max_gap = confirm, max_gap
        self.dashed_exit = dashed_exit
        self.reset()

    def reset(self):
        self.candidates = {}
        self.confirmed = {}

    def update(self, result, seq, stamp):
        out = {}
        for side in ("left", "right"):
            row = dict(result[side])
            measured_kind = row["kind"]
            previous = self.candidates.get(side)
            if previous is None or not 0 <= stamp-previous[2] <= self.max_gap or not row["valid"]:
                self.confirmed.pop(side,None)
            if (row.get("method") == "length_ratio" and row["valid"] and
                    row["kind"] == "solid" and
                    self.confirmed.get(side) == "dashed" and
                    row["dashed_fraction"] >= self.dashed_exit):
                row["kind"] = "dashed"
                row["hysteresis"] = True
            count = 0
            if row["valid"]:
                count = (previous[3]+1 if previous and previous[0] == row["kind"] and
                         seq > previous[1] and 0 <= stamp-previous[2] <= self.max_gap else 1)
            self.candidates[side] = (row["kind"], seq, stamp, count)
            row["raw_kind"] = measured_kind
            row["confirm_samples"] = count
            if not row["valid"] or count < self.confirm:
                row.update(kind="unknown", valid=False,
                           reason=row["reason"] if not row["valid"] else "pending_confirmation")
            else:
                self.confirmed[side] = row["kind"]
            out[side] = row
        if "selection" in result:
            out["selection"] = result["selection"]
        return out


class SignalSelector:
    def __init__(self, config=None):
        self.config = config or {}
        # Screen recapture can reduce saturation and brightness. Keep the same
        # hue ranges, local contrast check and temporal confirmation; blue and
        # white must never be re-labelled as green to increase a success rate.
        color = (ColorConfig(min_saturation=60, min_value=90, min_pixels=3,
                             min_box_side=4, min_contrast=12)
                 if self.config.get("screen_capture_color") else None)
        self.pixel_ratio = self.config.get("signal_algorithm") == "traffic_light_pixel_ratio_rg"
        self.forced_rg = self.pixel_ratio or self.config.get("signal_algorithm") == "traffic_light_reference_forced_rg"
        self.reference_hybrid = self.config.get("signal_algorithm") == "traffic_light_reference_hybrid"
        if self.pixel_ratio:
            from reference_signal import make_pixel_ratio_engine
            self.engine = make_pixel_ratio_engine()
        elif self.forced_rg:
            from reference_signal import make_forced_rg_engine
            self.engine = make_forced_rg_engine()
        elif self.reference_hybrid:
            from reference_signal import make_engine
            self.engine = make_engine()
        else:
            self.engine = TrafficLightColorEngine(color_config=color)

    def reset(self):
        self.engine.reset()

    def update(self, frame, detections, names, stamp, lane):
        results = self.engine.update(frame, detections, names, stamp)
        rois = self.config.get("signal_rois", {})
        rect = rois.get(lane, rois.get("default", [0, 0, 1, .75]))
        h, w = frame.shape[:2]
        x1, y1, x2, y2 = rect
        eligible = []
        for result in results:
            box = result["box"]
            cx, cy = (box[0]+box[2])/2/w, (box[1]+box[3])/2/h
            if x1 <= cx <= x2 and y1 <= cy <= y2:
                eligible.append(result)
        if self.forced_rg:
            return self._forced_summary(results, eligible, rect, lane)
        unanimous = False
        if len(eligible) > 1 and self.config.get("unanimous_lights"):
            colors = {r["stable_color"] for r in eligible}
            unanimous = (len(colors) == 1 and next(iter(colors)) in ("red", "yellow", "green")
                         and all(r["raw_color"] == r["stable_color"] for r in eligible))
        if len(eligible) != 1 and not unanimous:
            return {"color": "unknown", "valid": False,
                    "reason": "no_related_light" if not eligible else "multiple_lights_require_scene_roi",
                    "eligible_lights": len(eligible), "lane": lane, "selection_roi": rect,
                    "candidates": [{k:r[k] for k in ("box", "raw_color", "stable_color", "reason", "diagnostics")}
                                   for r in eligible[:8]]}
        light = eligible[0]
        color = light["stable_color"]
        return {"color": color, "raw_color": light["raw_color"],
                "valid": color in ("red", "yellow", "green"), "reason": light["reason"],
                "tracking_status": light["status"], "track_id": light["track_id"],
                "box": light["box"], "lane": lane, "selection_roi": rect,
                "eligible_lights": len(eligible), "det_confidence": light["det_confidence"],
                "selection_policy": "all_selected_lights_agree" if unanimous else "unique_light",
                "roi_size": light["roi_size"], "color_scores": light["scores"],
                "color_diagnostics": light["diagnostics"],
                "color_algorithm": "traffic_light_reference_hybrid" if self.reference_hybrid else "legacy_hsv",
                "soft_candidate": light["diagnostics"].get("soft_candidate"),
                "soft_weights": light["diagnostics"].get("soft_weights", {}),
                "lamp_box": light.get("lamp_box", []),
                "color_score": light["score"], "score_is_probability": False}

    def _forced_summary(self, results, eligible, rect, lane):
        algorithm = "traffic_light_pixel_ratio_rg" if self.pixel_ratio else "traffic_light_reference_forced_rg"
        if not results:
            return {"color": "unknown", "valid": False, "reason": "no_related_light",
                    "eligible_lights": 0, "lane": lane, "selection_roi": rect,
                    "candidates": [], "color_algorithm": algorithm}
        # Scene ROI defines preference, not another UNKNOWN gate. For multiple
        # heads use detector confidence, then area and location to break ties.
        pool = eligible or results
        light = max(pool, key=lambda r: (r["det_confidence"],
                    (r["box"][2]-r["box"][0])*(r["box"][3]-r["box"][1]),
                    -r["box"][0], -r["box"][1]))
        probabilities = light["probabilities"]
        color = light["stable_color"]
        return {"color": color, "raw_color": light["raw_color"], "valid": True,
                "reason": light["reason"], "tracking_status": light["status"],
                "track_id": light["track_id"], "box": light["box"], "lane": lane,
                "selection_roi": rect, "eligible_lights": len(eligible),
                "detected_lights": len(results), "det_confidence": light["det_confidence"],
                "selection_policy": "highest_detector_confidence_in_roi" if eligible else
                                    "highest_detector_confidence_fallback",
                "roi_size": light["roi_size"], "color_scores": light["scores"],
                "color_probabilities": probabilities, "color_score": probabilities[color],
                "score_is_probability": False, "forced_choice": True,
                "probability_kind": light["probability_kind"],
                "color_algorithm": algorithm,
                "soft_candidate": color, "soft_weights": probabilities,
                "color_diagnostics": {"method": "reference_forced_rg",
                    "evidence_strength": light["evidence_strength"],
                    "candidate_margin": light["candidate_margin"],
                    "weights_are_calibrated_probabilities": False, **light.get("diagnostics", {})},
                "candidates": [{k: r[k] for k in ("box", "raw_color", "stable_color",
                    "det_confidence", "reason", "scores", "probabilities", "forced_choice", "diagnostics")}
                    for r in results]}
