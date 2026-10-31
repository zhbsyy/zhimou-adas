"""Fixed detector policy, applied before results enter display/HTTP state."""
import numpy as np

CLASS10_MIN_CONFIDENCE = 0.40  # crosswalk: intentionally no CLI/environment override
OTHER_MIN_CONFIDENCE = 0.25
TILES = {"front": (0, 0, 640, 360), "right": (640, 0, 1280, 360),
         "rear": (0, 360, 640, 720), "left": (640, 360, 1280, 720)}


def filter_detections(detections):
    dets = np.asarray(detections, dtype=np.float32).reshape(-1, 6)
    if not len(dets):
        return dets.copy()
    thresholds = np.where(dets[:, 5] == 10, CLASS10_MIN_CONFIDENCE, OTHER_MIN_CONFIDENCE)
    valid = (np.isfinite(dets).all(axis=1) & (dets[:, 4] >= thresholds)
             & (dets[:, 4] <= 1) & (dets[:, 5] >= 0) & (dets[:, 5] <= 10)
             & (dets[:, 5] == np.floor(dets[:, 5]))
             & (dets[:, 2] > dets[:, 0]) & (dets[:, 3] > dets[:, 1]))
    return dets[valid].copy()


def tile_detections(detections, valid_cameras):
    """Clip within one camera; discard proposals mainly spanning mosaic seams."""
    rows = {name: [] for name in TILES}
    for box in filter_detections(detections):
        cx, cy = (box[:2] + box[2:4]) / 2
        for name, (x1, y1, x2, y2) in TILES.items():
            if name not in valid_cameras or not (x1 <= cx < x2 and y1 <= cy < y2):
                continue
            clipped = box.copy()
            clipped[[0, 2]] = np.clip(clipped[[0, 2]], x1, x2 - 1)
            clipped[[1, 3]] = np.clip(clipped[[1, 3]], y1, y2 - 1)
            area = (box[2] - box[0]) * (box[3] - box[1])
            inside = (clipped[2] - clipped[0]) * (clipped[3] - clipped[1])
            if inside > 0 and inside >= .8 * area:
                rows[name].append(clipped)
            break
    return {name: np.asarray(items, np.float32).reshape(-1, 6) for name, items in rows.items()}
