"""Pixel-identical display work on a private render frame, with immutable masks.

No cached source pixels: each render still blends the newest BGR image.
Only mask geometry/constant colors/counts are cached between lane results.
"""
import cv2
import numpy as np


def prepare_masks(masks):
    direct, lane = masks
    if direct.shape != lane.shape or direct.ndim != 2:
        raise ValueError("Two same-size two-dimensional masks required")
    layers = []
    for mask, color, alpha in ((direct == 1, (30, 170, 30), .25),
                               (lane == 1, (0, 0, 255), .70),
                               (lane == 2, (0, 255, 255), .70)):
        mask8 = mask.astype(np.uint8)
        x, y, w, h = cv2.boundingRect(mask8)
        if not w or not h:
            continue
        tile = np.empty((h, w, 3), np.uint8)
        tile[:] = color
        layers.append((x, y, w, h, mask8[y:y+h, x:x+w].copy(), tile, alpha))
    return {"shape": direct.shape, "layers": layers,
            "counts": {"direct_area": int(np.count_nonzero(direct)),
                       "solid_line": int(np.count_nonzero(lane == 1)),
                       "dashed_line": int(np.count_nonzero(lane == 2))}}


def apply_masks_inplace(front, prepared):
    if front.shape[:2] != prepared["shape"]:
        raise ValueError("Prepared mask shape mismatch")
    for x, y, w, h, mask, tile, alpha in prepared["layers"]:
        roi = front[y:y+h, x:x+w]
        blended = cv2.addWeighted(roi, 1-alpha, tile, alpha, 0)
        cv2.copyTo(blended, mask, roi)
    return prepared["counts"]


def begin_frame(image):
    """One owned canvas: copy the new image once, reserve the title in advance."""
    if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("Expected uint8 BGR image")
    h, w = image.shape[:2]
    canvas = np.empty((h + 35, w, 3), np.uint8)
    canvas[:35] = 25
    canvas[35:] = image
    return canvas, canvas[35:]


def draw_owned(image, detections, names, title, canvas=None):
    """Same OpenCV operations/order as current pcie_post.draw, on owned input."""
    if canvas is None:
        if not image.flags.owndata or image.dtype != np.uint8:
            raise ValueError("draw_owned requires a private uint8 render copy")
    else:
        if (not canvas.flags.owndata or canvas.dtype != np.uint8 or
                canvas.shape != (image.shape[0] + 35, image.shape[1], 3) or
                image.ctypes.data != canvas[35:].ctypes.data or image.strides != canvas[35:].strides):
            raise ValueError("Expected the exact image view of a private title canvas")
    target = image
    for x1, y1, x2, y2, confidence, class_id in detections:
        class_id = int(class_id)
        color = ((37 * class_id + 40) % 220, (71 * class_id + 180) % 220,
                 (111 * class_id + 60) % 220)
        p1 = (round(float(x1)), round(float(y1)))
        p2 = (round(float(x2)), round(float(y2)))
        cv2.rectangle(target, p1, p2, color, 2)
        cv2.putText(target, "%s %.2f" % (names[class_id], confidence),
                    (p1[0], max(18, p1[1] - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
    if canvas is None:
        canvas = cv2.copyMakeBorder(target, 35, 0, 0, 0, cv2.BORDER_CONSTANT, value=(25, 25, 25))
    cv2.putText(canvas, title, (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 1)
    return canvas
