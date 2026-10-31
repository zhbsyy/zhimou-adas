"""NumPy postprocessing for Rockchip-style YOLOv8 nine-output detection.

Output order per stride 8/16/32: regression logits, sigmoid class scores,
clipped class-score sum. DFL and NMS run on CPU, outside the ONNX graph.
"""

import cv2
import numpy as np


def preprocess(image, size=640, pad_value=114):
    h, w = image.shape[:2]
    ratio = min(size / h, size / w)
    resized_w, resized_h = round(w * ratio), round(h * ratio)
    half_w, half_h = (size - resized_w) / 2, (size - resized_h) / 2
    left, top = round(half_w - 0.1), round(half_h - 0.1)
    right, bottom = round(half_w + 0.1), round(half_h + 0.1)
    resized = cv2.resize(image, (resized_w, resized_h), interpolation=cv2.INTER_LINEAR)
    padded = cv2.copyMakeBorder(resized, top, bottom, left, right,
                                cv2.BORDER_CONSTANT, value=(pad_value,) * 3)
    rgb = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB)
    tensor = np.ascontiguousarray(rgb.transpose(2, 0, 1)[None], dtype=np.float32) / 255.0
    return tensor, {"ratio": ratio, "left": left, "top": top, "shape": [h, w]}


def decode(outputs, size=640, nc=11, reg_max=16):
    if len(outputs) != 9:
        raise ValueError(f"Expected 9 outputs, received {len(outputs)}")
    boxes, scores = [], []
    for branch, stride in enumerate((8, 16, 32)):
        logits, classes, score_sum = outputs[branch * 3:branch * 3 + 3]
        grid_size = size // stride
        assert logits.shape == (1, 4 * reg_max, grid_size, grid_size)
        assert classes.shape == (1, nc, grid_size, grid_size)
        assert score_sum.shape == (1, 1, grid_size, grid_size)
        distribution = logits.reshape(1, 4, reg_max, grid_size, grid_size)
        distribution = distribution - distribution.max(axis=2, keepdims=True)
        distribution = np.exp(distribution)
        distribution /= distribution.sum(axis=2, keepdims=True)
        distances = (distribution * np.arange(reg_max, dtype=np.float32)
                     .reshape(1, 1, reg_max, 1, 1)).sum(axis=2)[0]
        yy, xx = np.meshgrid(np.arange(grid_size, dtype=np.float32),
                             np.arange(grid_size, dtype=np.float32), indexing="ij")
        xyxy = np.stack((xx + 0.5 - distances[0], yy + 0.5 - distances[1],
                         xx + 0.5 + distances[2], yy + 0.5 + distances[3]), axis=-1) * stride
        boxes.append(xyxy.reshape(-1, 4))
        scores.append(classes[0].transpose(1, 2, 0).reshape(-1, nc))
    return np.concatenate(boxes), np.concatenate(scores)


def pairwise_iou(box, others):
    lo = np.maximum(box[:2], others[:, :2])
    hi = np.minimum(box[2:], others[:, 2:])
    inter = np.maximum(hi - lo, 0).prod(axis=1)
    area = np.maximum(box[2:] - box[:2], 0).prod()
    other_area = np.maximum(others[:, 2:] - others[:, :2], 0).prod(axis=1)
    return inter / np.maximum(area + other_area - inter, 1e-9)


def nms(boxes, scores, confidence=0.25, iou_threshold=0.45, max_det=300):
    labels = scores.argmax(axis=1)
    confidences = scores.max(axis=1)
    selected = np.flatnonzero(confidences >= confidence)
    kept = []
    for class_id in np.unique(labels[selected]):
        indices = selected[labels[selected] == class_id]
        indices = indices[np.argsort(-confidences[indices], kind="stable")]
        while indices.size:
            chosen = indices[0]
            kept.append(chosen)
            indices = indices[1:]
            indices = indices[pairwise_iou(boxes[chosen], boxes[indices]) <= iou_threshold]
    kept = sorted(kept, key=lambda i: -float(confidences[i]))[:max_det]
    return np.array([[*boxes[i], confidences[i], labels[i]] for i in kept],
                    dtype=np.float32).reshape(-1, 6)


def to_original(detections, transform):
    result = detections.copy()
    result[:, [0, 2]] = (result[:, [0, 2]] - transform["left"]) / transform["ratio"]
    result[:, [1, 3]] = (result[:, [1, 3]] - transform["top"]) / transform["ratio"]
    h, w = transform["shape"]
    result[:, [0, 2]] = np.clip(result[:, [0, 2]], 0, w)
    result[:, [1, 3]] = np.clip(result[:, [1, 3]], 0, h)
    return result


def draw(image, detections, names, title):
    canvas = image.copy()
    for x1, y1, x2, y2, confidence, class_id in detections:
        class_id = int(class_id)
        color = ((37 * class_id + 40) % 220, (71 * class_id + 180) % 220,
                 (111 * class_id + 60) % 220)
        p1, p2 = (round(float(x1)), round(float(y1))), (round(float(x2)), round(float(y2)))
        cv2.rectangle(canvas, p1, p2, color, 2)
        cv2.putText(canvas, f"{names[class_id]} {confidence:.3f}",
                    (p1[0], max(18, p1[1] - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
    canvas = cv2.copyMakeBorder(canvas, 35, 0, 0, 0, cv2.BORDER_CONSTANT, value=(25, 25, 25))
    cv2.putText(canvas, title, (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 1)
    return canvas
