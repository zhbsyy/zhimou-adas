"""Diagnostic-only pixel-equivalent bounding-box blend; not installed in pipeline."""
import cv2
import numpy as np


def draw_front_roi(front, direct_mask, lane_mask):
    if direct_mask.shape != front.shape[:2] or lane_mask.shape != front.shape[:2]:
        raise ValueError("R15 masks must match the front tile size")
    vis = front.copy()
    for mask, color, alpha in ((direct_mask == 1, (30, 170, 30), .25),
                               (lane_mask == 1, (0, 0, 255), .70),
                               (lane_mask == 2, (0, 255, 255), .70)):
        mask8 = mask.astype(np.uint8)
        x, y, w, h = cv2.boundingRect(mask8)
        if not w or not h:
            continue
        roi = vis[y:y+h, x:x+w]
        tile = np.empty_like(roi); tile[:] = color
        blended = cv2.addWeighted(roi, 1-alpha, tile, alpha, 0)
        cv2.copyTo(blended, mask8[y:y+h, x:x+w], roi)
    return vis


def draw_lane_mask_roi(front, masks, _detections=None):
    direct, lane = masks
    return draw_front_roi(front, direct, lane), {
        "direct_area": int(np.count_nonzero(direct)),
        "solid_line": int(np.count_nonzero(lane == 1)),
        "dashed_line": int(np.count_nonzero(lane == 2)),
    }
