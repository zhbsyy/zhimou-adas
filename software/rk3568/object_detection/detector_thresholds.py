"""Per-class filtering after a low-floor decode; classes and weights unchanged."""
import numpy as np


def traffic_light_ids(names):
    return {i for i, name in enumerate(names)
            if str(name).strip().lower().replace('_', ' ').replace('-', ' ')
            in ('traffic light', 'traffic lights', 'trafficlight', '信号灯', '交通灯')}


def filter_scores(detections, names, base=.25, light=.15, crosswalk=.5):
    if not detections.size:
        return detections
    limits = np.full(len(detections), base, np.float32)
    for cls in traffic_light_ids(names):
        limits[detections[:, 5] == cls] = min(base, light)
    limits[detections[:, 5] == 10] = max(base, crosswalk)
    return detections[detections[:, 4] >= limits]
