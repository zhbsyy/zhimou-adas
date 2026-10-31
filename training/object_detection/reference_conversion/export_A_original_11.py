"""Export the local A_original.pt 11-class checkpoint to a static nine-output ONNX.

No installed Ultralytics source or checkpoint is modified. A copied model's
Detect.forward is adapted in memory, following Rockchip's nine-output contract:
https://github.com/airockchip/ultralytics_yolov8/blob/main/ultralytics/nn/modules/head.py
"""

import argparse
import copy
import hashlib
import json
import os
import platform
import shutil
import types
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parent
# Keep Ultralytics' incidental settings/cache writes inside this deployment folder.
os.environ["YOLO_CONFIG_DIR"] = str(ROOT / ".ultralytics")

import cv2
import numpy as np
import onnx
import onnxruntime as ort
import torch
import ultralytics
from ultralytics import YOLO
from ultralytics.nn.modules import Detect

from postprocess import decode, draw, nms, pairwise_iou, preprocess, to_original

SOURCE_ROOT = Path("D:/deeplearning/ultralytics-8.3.163")
EXPECTED_CLASSES = ["person", "rider", "car", "truck", "bus", "train", "motor", "bike",
                    "traffic light", "traffic sign", "crosswalk"]
OUTPUT_NAMES = [f"{kind}_s{stride}" for stride in (8, 16, 32)
                for kind in ("reg", "cls", "score_sum")]


def rockchip_forward(self, features):
    outputs = []
    for level in range(self.nl):
        regression = self.cv2[level](features[level])
        probabilities = self.cv3[level](features[level]).sigmoid()
        outputs.extend((regression, probabilities,
                        probabilities.sum(dim=1, keepdim=True).clamp(0, 1)))
    return tuple(outputs)


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def numerical_check(reference, candidate, atol, rtol):
    assert reference.shape == candidate.shape
    error = np.abs(reference - candidate)
    return {"shape": list(reference.shape), "max_abs_error": float(error.max()),
            "mean_abs_error": float(error.mean()), "atol": atol, "rtol": rtol,
            "passed": bool(np.allclose(reference, candidate, atol=atol, rtol=rtol))}


def match_detections(reference, candidate):
    remaining = set(range(len(candidate)))
    matches = []
    for det in reference:
        choices = [i for i in remaining if int(candidate[i, 5]) == int(det[5])]
        if not choices:
            return {"passed": False, "reason": "Missing same-class detection", "matches": matches}
        overlaps = pairwise_iou(det[:4], candidate[choices, :4])
        best = int(np.argmax(overlaps))
        index = choices[best]
        remaining.remove(index)
        matches.append({"class_id": int(det[5]), "iou": float(overlaps[best]),
                        "max_box_error_px": float(np.abs(det[:4] - candidate[index, :4]).max()),
                        "confidence_error": float(abs(det[4] - candidate[index, 4]))})
    passed = not remaining and len(reference) > 0 and all(
        m["iou"] >= 0.995 and m["max_box_error_px"] <= 0.2 and m["confidence_error"] <= 0.001
        for m in matches)
    return {"passed": passed, "reference_count": len(reference),
            "candidate_count": len(candidate), "matches": matches}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--weights", type=Path, default=SOURCE_ROOT / "A_original.pt")
    parser.add_argument("--image", type=Path, default=SOURCE_ROOT / "ultralytics/assets/bus.jpg")
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts")
    args = parser.parse_args()
    for path in (args.weights, args.image):
        if not path.is_file():
            raise FileNotFoundError(path)
    args.output.mkdir(parents=True, exist_ok=True)
    model_path = args.output / "A_original_rkhead9_640.onnx"
    if model_path.exists():
        raise FileExistsError(model_path)
    # A previous successful report must not be mistaken for the current run.
    report_path = args.output / "verification.json"
    report_path.write_text('{"status":"running","passed":false}', encoding="utf-8")
    torch.set_num_threads(4)
    torch.manual_seed(0)
    original = YOLO(str(args.weights)).model.cpu().float().eval()
    names = [original.names[i] for i in range(len(original.names))]
    if names != EXPECTED_CLASSES:
        raise ValueError(f"Unexpected class mapping: {names}")
    head = original.model[-1]
    if type(head) is not Detect or head.nc != 11 or head.nl != 3 or head.reg_max != 16:
        raise ValueError("This exporter requires the verified three-scale YOLOv8 Detect head.")
    if getattr(head, "end2end", False) or head.stride.tolist() != [8.0, 16.0, 32.0]:
        raise ValueError("Unsupported head stride or end-to-end mode.")
    adapted = copy.deepcopy(original)
    adapted_head = adapted.model[-1]
    adapted_head.forward = types.MethodType(rockchip_forward, adapted_head)
    image = cv2.imdecode(np.fromfile(str(args.image), dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"Unreadable image: {args.image}")
    input_array, transform = preprocess(image)
    assert input_array.shape == (1, 3, 640, 640)
    tensor = torch.from_numpy(input_array)
    print("Running original and adapted PyTorch models on identical 640x640 input...", flush=True)
    with torch.inference_mode():
        prediction = original(tensor)
        if isinstance(prediction, tuple):
            prediction = prediction[0]
        native = prediction[0].numpy().T
        raw_torch = [value.numpy() for value in adapted(tensor)]
    native_boxes = np.concatenate((native[:, :2] - native[:, 2:4] / 2,
                                    native[:, :2] + native[:, 2:4] / 2), axis=1)
    native_scores = native[:, 4:]
    adapted_boxes, adapted_scores = decode(raw_torch)
    adaptation_checks = {
        "decoded_boxes": numerical_check(native_boxes, adapted_boxes, 0.002, 1e-5),
        "class_probabilities": numerical_check(native_scores, adapted_scores, 1e-6, 1e-5),
    }
    if not all(v["passed"] for v in adaptation_checks.values()):
        raise AssertionError(f"Head adaptation changed predictions: {adaptation_checks}")

    print(f"Exporting opset 12, static 1x3x640x640, nine outputs: {model_path}", flush=True)
    with torch.no_grad():
        torch.onnx.export(adapted, tensor, str(model_path), input_names=["images"],
                          output_names=OUTPUT_NAMES, opset_version=12, dynamic_axes=None,
                          do_constant_folding=True, training=torch.onnx.TrainingMode.EVAL,
                          dynamo=False)
    graph = onnx.load(str(model_path))
    onnx.checker.check_model(graph, full_check=True)
    shapes = [[d.dim_value for d in value.type.tensor_type.shape.dim]
              for value in graph.graph.output]
    expected_shapes = [[1, channels, 640 // stride, 640 // stride]
                       for stride in (8, 16, 32) for channels in (64, 11, 1)]
    assert [d.dim_value for d in graph.graph.input[0].type.tensor_type.shape.dim] == [1, 3, 640, 640]
    assert shapes == expected_shapes, shapes
    assert not any(node.op_type in ("NonMaxSuppression", "Softmax") for node in graph.graph.node)
    metadata = {
        "task": "detect", "names": json.dumps(dict(enumerate(names))),
        "stride": "32", "imgsz": "[640, 640]", "batch": "1",
        "output_contract": "stride 8,16,32: regression logits, sigmoid classes, clipped class sum",
        "preprocessing": "RGB; fixed letterbox 640; pad=114; NCHW float32 divided by 255",
        "source_weights_sha256": sha256(args.weights),
    }
    onnx.helper.set_model_props(graph, metadata)
    onnx.save(graph, str(model_path))
    print("Running ONNX Runtime CPU and numerical comparisons...", flush=True)
    options = ort.SessionOptions()
    options.intra_op_num_threads = 4
    options.inter_op_num_threads = 1
    session = ort.InferenceSession(str(model_path), sess_options=options,
                                   providers=["CPUExecutionProvider"])
    raw_onnx = session.run(OUTPUT_NAMES, {"images": input_array})
    raw_checks = {name: numerical_check(a, b, 0.0002, 0.001)
                  for name, a, b in zip(OUTPUT_NAMES, raw_torch, raw_onnx)}
    onnx_boxes, onnx_scores = decode(raw_onnx)
    native_dets = to_original(nms(native_boxes, native_scores), transform)
    onnx_dets = to_original(nms(onnx_boxes, onnx_scores), transform)
    detection_check = match_detections(native_dets, onnx_dets)
    passed = all(v["passed"] for v in raw_checks.values()) and detection_check["passed"]
    panels = []
    for filename, dets, title in (("pytorch.jpg", native_dets, "Original PyTorch (fixed 640)"),
                                   ("onnx.jpg", onnx_dets, "RK-optimized ONNX (fixed 640)")):
        panel = draw(image, dets, names, title)
        panels.append(panel)
        ok, encoded = cv2.imencode(".jpg", panel)
        assert ok
        (args.output / filename).write_bytes(encoded.tobytes())
    ok, comparison = cv2.imencode(".jpg", np.concatenate(panels, axis=1))
    assert ok
    (args.output / "comparison.jpg").write_bytes(comparison.tobytes())
    shutil.copyfile(args.image, args.output / "test_image.jpg")
    (args.output / "classes.txt").write_text("\n".join(names) + "\n", encoding="utf-8")
    manifest = {
        "onnx_file": model_path.name, "onnx_sha256": sha256(model_path), "opset": 12,
        "input": {"name": "images", "shape": [1, 3, 640, 640], "dtype": "float32"},
        "outputs": [{"name": name, "shape": shape} for name, shape in zip(OUTPUT_NAMES, shapes)],
        "classes": names, "reg_max": 16, "strides": [8, 16, 32],
        "preprocess": {"color": "RGB", "letterbox": True, "padding": 114,
                       "onnx_scale": "divide by 255", "resize": "OpenCV INTER_LINEAR"},
        "postprocess": {"confidence": 0.25, "nms_iou": 0.45, "class_aware": True},
        "rknn_config": {"target_platform": "rk3568", "mean_values": [[0, 0, 0]],
                        "std_values": [[255, 255, 255]]},
        "rknn_input_note": "Use RGB uint8 input with the above RKNN config; do not divide by 255 again.",
        "rk_board_validation": "NOT RUN; this artifact is ONNX, not an RKNN model",
        "source_weights_sha256": sha256(args.weights),
        "adapter_reference": "https://github.com/airockchip/ultralytics_yolov8/blob/main/ultralytics/nn/modules/head.py",
    }
    (args.output / "deployment_manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    report = {
        "status": "completed", "passed": passed, "onnx_checker_passed": True,
        "source_image": str(args.image), "source_image_sha256": sha256(args.image),
        "model_sha256": sha256(model_path), "input_shape": list(input_array.shape),
        "versions": {"python": platform.python_version(), "torch": torch.__version__,
                     "ultralytics": ultralytics.__version__, "numpy": np.__version__,
                     "onnx": onnx.__version__, "onnxruntime": ort.__version__},
        "adaptation_checks": adaptation_checks, "onnx_output_checks": raw_checks,
        "detection_check": detection_check,
        "pytorch_detections_xyxy_conf_class": native_dets.tolist(),
        "onnx_detections_xyxy_conf_class": onnx_dets.tolist(),
        "pytorch_counts": dict(Counter(names[int(d[5])] for d in native_dets)),
        "onnx_counts": dict(Counter(names[int(d[5])] for d in onnx_dets)),
        "scope": "One-image FP32 export equivalence only; not mAP, INT8 accuracy, or RK3568 performance",
    }
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps({"passed": passed, "counts": report["onnx_counts"],
                      "largest_raw_error": max(v["max_abs_error"] for v in raw_checks.values()),
                      "detection_check": detection_check}, indent=2), flush=True)
    print(f"Artifacts: {args.output}")
    if not passed:
        raise AssertionError(f"Verification failed. Inspect {report_path}")


if __name__ == "__main__":
    main()
