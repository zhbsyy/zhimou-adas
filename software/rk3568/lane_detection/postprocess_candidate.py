"""Three finite FP32 planes -> first winning class, with uint8 output.

The surrounding R15 code still checks finite/nonzero outputs and applies the
original sparse softmax, confidence threshold and nearest-neighbor resize.
This changes no logits and does not bypass any RKNN operator.
"""
import statistics
import time

import numpy as np


def lane_argmax_planar(lane_logits):
    """Match np.argmax(axis=0), including first-index ties, for finite logits."""
    if lane_logits.ndim != 3 or lane_logits.shape[0] != 3:
        raise ValueError("Expected three lane-logit planes")
    result = (lane_logits[1] > lane_logits[0]).astype(np.uint8)
    win2 = lane_logits[2] > lane_logits[0]
    np.logical_and(win2, lane_logits[2] > lane_logits[1], out=win2)
    np.copyto(result, np.uint8(2), where=win2)
    return result


def compare_masks(lane_logits, confidence, size, lane_module):
    """Both the confidence-filtered mask and the resized mask must match."""
    import cv2
    a = lane_module.lane_argmax_numpy(lane_logits)
    b = lane_argmax_planar(lane_logits)
    if not np.array_equal(a, b):
        raise RuntimeError("Planar lane argmax differs; live trial stopped")
    lane_module.filter_lane_confidence_sparse(a, lane_logits, confidence)
    lane_module.filter_lane_confidence_sparse(b, lane_logits, confidence)
    if not np.array_equal(a, b):
        raise RuntimeError("Confidence-filtered lane masks differ; live trial stopped")
    a = cv2.resize(a, size, interpolation=cv2.INTER_NEAREST)
    b = cv2.resize(b, size, interpolation=cv2.INTER_NEAREST)
    if not np.array_equal(a, b):
        raise RuntimeError("Resized lane masks differ; live trial stopped")
    return a


def measure_argmax_pair(logits, lane_module, repeats=24):
    """Balanced CPU microbenchmark only; excludes NPU/softmax/video/render."""
    functions = (lane_module.lane_argmax_numpy, lane_argmax_planar)
    timings = [[], []]
    for fn in functions:
        fn(logits)
    for round_id in range(repeats):
        for index in ((0, 1) if round_id % 2 == 0 else (1, 0)):
            started = time.perf_counter()
            result = functions[index](logits)
            timings[index].append((time.perf_counter() - started) * 1000)
            del result
    return {
        "numpy_argmax_median_ms": statistics.median(timings[0]),
        "planar_argmax_median_ms": statistics.median(timings[1]),
        "calls_per_method": repeats,
        "note": "CPU argmax only, before live capture; not full-postprocess or pipeline FPS",
    }


def verify_model_masks(path, video, factory, lane_module, confidence):
    """Check actual unchanged RKNN outputs before opening capture or HTTP."""
    import cv2
    cap = cv2.VideoCapture(video)
    try:
        ok, frame = cap.read() if cap.isOpened() else (False, None)
        if not ok:
            raise RuntimeError("Lane mask preflight cannot read existing video: " + video)
    finally:
        cap.release()
    frame = cv2.resize(frame, (640, 360), interpolation=cv2.INTER_AREA)
    prep = lane_module.R15Front.__new__(lane_module.R15Front)
    prep.height, prep.width, prep.input_mode = 256, 448, "uint8_nhwc"
    actual, box = prep.preprocess(frame)
    h, w = actual.shape[1:3]
    ramp = np.arange(h*w*3, dtype=np.uint32).reshape(1, h, w, 3).astype(np.uint8)
    runtime = factory()
    rows = []
    try:
        if runtime.load_rknn(path) or runtime.init_runtime():
            raise RuntimeError("Lane mask preflight runtime initialization failed")
        for name, tensor in (("flat114", np.full_like(actual, 114)),
                             ("ramp", ramp), ("video_frame", actual)):
            outputs = runtime.inference(inputs=[tensor], data_format="nhwc")
            if outputs is None or len(outputs) != 2:
                raise RuntimeError("Lane mask preflight needs two outputs")
            logits = prep._channels(outputs[1], 3)
            if not np.isfinite(logits).all():
                raise RuntimeError("Lane preflight logits are not finite")
            left, top, nw, nh, out_w, out_h = box
            logits = logits[:, top:top+nh, left:left+nw]
            mask = compare_masks(logits, confidence, (out_w, out_h), lane_module)
            row = {"input": name, "exact_mask_equal": True,
                   "mask_shape": list(mask.shape), "confidence": confidence,
                   "solid_pixels": int(np.count_nonzero(mask == 1)),
                   "dashed_pixels": int(np.count_nonzero(mask == 2))}
            row.update(measure_argmax_pair(logits, lane_module))
            rows.append(row)
    finally:
        runtime.release()
    return rows
