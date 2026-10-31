"""Offline video -> existing YOLO -> HSV + per-target temporal state.

RKNN uses the board's existing postprocess.py interface unchanged.
ONNX supports standard YOLOv8 single output or Rockchip 6/9 tensor heads.
ROI backend is only for isolated color-pipeline checks, not YOLO validation.
"""
import argparse
import ast
import csv
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import time

import cv2
import numpy as np

from traffic_light_color import TrafficLightColorEngine, draw_results, load_config


def letterbox(frame, size):
    h, w = frame.shape[:2]
    scale = min(size / float(w), size / float(h))
    nw, nh = round(w * scale), round(h * scale)
    resized = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_LINEAR)
    left, top = (size - nw) // 2, (size - nh) // 2
    canvas = cv2.copyMakeBorder(resized, top, size - nh - top, left, size - nw - left,
                                cv2.BORDER_CONSTANT, value=(114, 114, 114))
    return cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB), (nw / w, nh / h, left, top)


def decode_onnx(outputs, nc, size, confidence, iou_threshold):
    if len(outputs) == 1:
        pred = np.asarray(outputs[0], dtype=np.float32)
        if pred.ndim != 3 or pred.shape[0] != 1:
            raise ValueError("Expected standard YOLOv8 [1,4+nc,N] output")
        pred = pred[0]
        if pred.shape[0] == nc + 4:
            pred = pred.T
        elif pred.shape[1] != nc + 4:
            raise ValueError("YOLO output does not match class count; supply matching --classes")
        xywh, scores = pred[:, :4], pred[:, 4:]
        boxes = np.concatenate((xywh[:, :2] - xywh[:, 2:] / 2,
                                xywh[:, :2] + xywh[:, 2:] / 2), axis=1)
    elif len(outputs) in (6, 9):
        step = len(outputs) // 3
        box_parts, class_parts = [], []
        for index in range(3):
            reg, cls = (np.asarray(v, dtype=np.float32) for v in outputs[index * step:index * step + 2])
            if (reg.ndim != 4 or cls.ndim != 4 or reg.shape[0] != 1 or
                    cls.shape[:2] != (1, nc) or reg.shape[1] % 4 or
                    reg.shape[2:] != cls.shape[2:]):
                raise ValueError("Unsupported head layout; expected NCHW DFL and sigmoid classes")
            gh, gw = reg.shape[2:]
            bins = reg.shape[1] // 4
            p = reg.reshape(1, 4, bins, gh, gw)
            p = np.exp(p - p.max(axis=2, keepdims=True))
            p /= p.sum(axis=2, keepdims=True)
            distance = (p * np.arange(bins, dtype=np.float32)[None, None, :, None, None]).sum(axis=2)[0]
            gy, gx = np.mgrid[:gh, :gw]
            sx, sy = size / gw, size / gh
            boxes_i = np.stack(((gx + .5 - distance[0]) * sx,
                                (gy + .5 - distance[1]) * sy,
                                (gx + .5 + distance[2]) * sx,
                                (gy + .5 + distance[3]) * sy), axis=-1).reshape(-1, 4)
            box_parts.append(boxes_i)
            class_parts.append(cls[0].transpose(1, 2, 0).reshape(-1, nc))
        boxes, scores = np.concatenate(box_parts), np.concatenate(class_parts)
    else:
        raise ValueError("Unsupported YOLO output count: %d" % len(outputs))
    if not np.isfinite(scores).all() or np.any(scores < 0) or np.any(scores > 1.0001):
        raise ValueError("Expected finite sigmoid class probabilities, not logits")
    class_ids = scores.argmax(axis=1)
    best = scores[np.arange(len(scores)), class_ids]
    keep = ((best >= confidence) & np.isfinite(boxes).all(axis=1) &
            (boxes[:, 2] > boxes[:, 0]) & (boxes[:, 3] > boxes[:, 1]))
    boxes, best, class_ids = boxes[keep], best[keep], class_ids[keep]
    picked = []
    for class_id in np.unique(class_ids):
        indices = np.flatnonzero(class_ids == class_id)
        xywh = boxes[indices].copy()
        xywh[:, 2:] -= xywh[:, :2]
        selected = cv2.dnn.NMSBoxes(xywh.tolist(), best[indices].tolist(),
                                    confidence, iou_threshold)
        if len(selected):
            picked.extend(indices[np.asarray(selected).reshape(-1)].tolist())
    picked = sorted(picked, key=lambda i: best[i], reverse=True)[:300]
    if not picked:
        return np.empty((0, 6), dtype=np.float32)
    return np.column_stack((boxes[picked], best[picked], class_ids[picked])).astype(np.float32)


class OnnxDetector:
    def __init__(self, args, names):
        import onnxruntime as ort
        options = ort.SessionOptions()
        options.intra_op_num_threads = args.threads
        self.session = ort.InferenceSession(args.model, sess_options=options,
                                           providers=["CPUExecutionProvider"])
        inputs = self.session.get_inputs()
        if len(inputs) != 1:
            raise ValueError("Expected one ONNX image input")
        self.input = inputs[0]
        shape = self.input.shape
        if (len(shape) != 4 or shape[:2] != [1, 3] or
                not isinstance(shape[2], int) or shape[2] != shape[3] or
                self.input.type != "tensor(float)"):
            raise ValueError("Expected static square float32 NCHW ONNX model")
        self.size = shape[2]
        self.args, self.names = args, names
        metadata = self.session.get_modelmeta().custom_metadata_map
        if "names" in metadata:
            model_names = ast.literal_eval(metadata["names"])
            ordered = [model_names[i] for i in range(len(model_names))] if isinstance(model_names, dict) else list(model_names)
            if ordered != names:
                raise ValueError("--classes order differs from ONNX metadata names")

    def infer(self, frame):
        start = time.perf_counter()
        rgb, transform = letterbox(frame, self.size)
        tensor = np.ascontiguousarray(rgb.transpose(2, 0, 1)[None], dtype=np.float32) / 255.0
        pre = time.perf_counter()
        outputs = self.session.run(None, {self.input.name: tensor})
        inf = time.perf_counter()
        dets = decode_onnx(outputs, len(self.names), self.size, self.args.conf, self.args.iou)
        if dets.size:
            sx, sy, left, top = transform
            dets[:, [0, 2]] = (dets[:, [0, 2]] - left) / sx
            dets[:, [1, 3]] = (dets[:, [1, 3]] - top) / sy
            dets[:, [0, 2]] = dets[:, [0, 2]].clip(0, frame.shape[1])
            dets[:, [1, 3]] = dets[:, [1, 3]].clip(0, frame.shape[0])
        end = time.perf_counter()
        return dets, {"preprocess_ms": (pre - start) * 1000,
                      "inference_ms": (inf - pre) * 1000,
                      "postprocess_ms": (end - inf) * 1000}

    def close(self):
        pass


class RknnDetector:
    def __init__(self, args, names):
        path = Path(args.yolo_dir).expanduser().resolve() / "postprocess.py"
        if not path.is_file():
            raise FileNotFoundError("Existing board postprocess.py missing: %s; use --yolo-dir" % path)
        sys.path.insert(0, str(path.parent))
        spec = importlib.util.spec_from_file_location("existing_yolo_postprocess", str(path))
        self.post = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.post)
        from rknnlite.api import RKNNLite
        self.rk = RKNNLite()
        self.args, self.names = args, names
        try:
            if self.rk.load_rknn(args.model) != 0:
                raise RuntimeError("load_rknn failed")
            if self.rk.init_runtime() != 0:
                raise RuntimeError("init_runtime failed")
        except Exception:
            self.rk.release()
            raise

    def infer(self, frame):
        start = time.perf_counter()
        tensor, transform = self.post.preprocess(frame, size=self.args.size)
        # Same uint8 RGB NHWC conversion as the existing yolo_pcie.py.
        image = (np.clip(tensor, 0, 1)[0].transpose(1, 2, 0) * 255 + .5).astype(np.uint8)
        image = np.ascontiguousarray(image[None])
        pre = time.perf_counter()
        outputs = self.rk.inference(inputs=[image], data_format="nhwc")
        inf = time.perf_counter()
        boxes, scores = self.post.decode(outputs, size=self.args.size, nc=len(self.names))
        dets = self.post.nms(boxes, scores, confidence=self.args.conf)
        if dets.size:
            dets = self.post.to_original(dets, transform)
        end = time.perf_counter()
        return dets, {"preprocess_ms": (pre - start) * 1000,
                      "inference_ms": (inf - pre) * 1000,
                      "postprocess_ms": (end - inf) * 1000}

    def close(self):
        self.rk.release()


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--video", required=True, help="Input video file, not camera index")
    parser.add_argument("--model", help="Existing .rknn or .onnx model; required except ROI mode")
    parser.add_argument("--backend", choices=("auto", "rknn", "onnx", "roi"), default="auto")
    parser.add_argument("--classes", default=str(Path(__file__).with_name("classes_bdd100k.txt")))
    parser.add_argument("--yolo-dir", default=os.environ.get("YOLO_DIR", "/home/linaro/yolov8_deploy"))
    parser.add_argument("--out", default="video_result", help="New output directory")
    parser.add_argument("--config", default=str(Path(__file__).with_name("hsv_config.json")))
    parser.add_argument("--color-mode", choices=("conservative", "forced"),
                        help="Override config: forced always selects a candidate for a valid detected ROI")
    parser.add_argument("--color-candidates", choices=("red", "yellow", "green"), nargs="+",
                        help="Forced-mode candidate set, e.g. red green or red yellow green")
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--iou", type=float, default=0.45, help="ONNX NMS IoU")
    parser.add_argument("--size", type=int, default=640, help="RKNN input size; must match model")
    parser.add_argument("--threads", type=int, default=2, help="ONNX CPU threads")
    parser.add_argument("--stride", type=int, default=1, help="Process every Nth source frame")
    parser.add_argument("--max-frames", type=int, default=0, help="Maximum processed frames; 0=all")
    parser.add_argument("--fps", type=float, help="Override input FPS when video metadata is wrong")
    parser.add_argument("--roi", action="append", type=float, nargs=4, metavar=("X1", "Y1", "X2", "Y2"),
                        help="Fixed box for --backend roi only; repeat for multiple lamps")
    parser.add_argument("--debug-every", type=int, default=0, help="Save raw lamp crops every N processed frames")
    parser.add_argument("--no-video", action="store_true", help="Only logs; omit video encoding")
    parser.add_argument("--show", action="store_true", help="Optional local preview; Q stops")
    parser.add_argument("--display", choices=("clean", "all", "debug"), default="clean",
                        help="clean: colors only; all: other classes too; debug: IDs/raw/timings")
    parser.add_argument("--show-unknown", action="store_true", help="Draw uncertain lamps in clean/all modes")
    args = parser.parse_args()
    if args.stride < 1 or args.max_frames < 0 or args.debug_every < 0 or args.threads < 1:
        parser.error("stride/threads must be positive; frame/debug counts nonnegative")
    if args.size < 32 or args.size % 32 or not 0 < args.conf < 1 or not 0 < args.iou < 1:
        parser.error("size must be a positive multiple of 32; conf/iou must be between 0 and 1")
    if args.backend == "auto":
        suffix = Path(args.model or "").suffix.lower()
        if suffix not in (".rknn", ".onnx"):
            parser.error("Provide --model .rknn/.onnx, or explicitly choose --backend roi")
        args.backend = suffix[1:]
    if args.backend == "roi":
        if not args.roi or args.model:
            parser.error("ROI mode needs --roi and does not take --model")
        if any(not all(math.isfinite(v) for v in box) or box[2] <= box[0] or box[3] <= box[1]
               for box in args.roi):
            parser.error("Each ROI must be finite positive-size xyxy")
    elif args.roi or not args.model or not Path(args.model).is_file():
        parser.error("YOLO mode needs an existing --model and does not use --roi")
    if not Path(args.video).is_file():
        parser.error("Video file does not exist: " + args.video)
    if args.fps is not None and (not math.isfinite(args.fps) or args.fps <= 0):
        parser.error("--fps must be finite and positive")
    return args


def main():
    args = arguments()
    color_cfg, state_cfg = load_config(args.config)
    if args.color_mode:
        color_cfg.decision_mode = args.color_mode
    if args.color_candidates:
        color_cfg.candidates = tuple(args.color_candidates)
    engine = TrafficLightColorEngine(color_cfg, state_cfg)
    if args.backend == "roi":
        names = ["traffic light"]
        print("[ROI mode] COLOR ONLY: fixed boxes, no YOLO inference")
    else:
        names = [line.strip() for line in Path(args.classes).read_text(encoding="utf-8-sig").splitlines()
                 if line.strip()]
        normalized = {name.lower().replace("_", " ").replace("-", " ") for name in names}
        if not normalized.intersection({"traffic light", "traffic lights", "trafficlight", "信号灯", "交通灯"}):
            raise ValueError("Class list has no traffic light class; check class order/names")
    output = Path(args.out).resolve()
    if output.exists() and any(output.iterdir()):
        raise FileExistsError("Output directory is not empty; choose a new --out: %s" % output)
    output.mkdir(parents=True, exist_ok=True)
    cap, writer, detector = None, None, None
    processed, source_index, light_count = 0, 0, 0
    color_counts, raw_counts, stage_sum = {}, {}, {}
    aborted = False
    error_message = None
    wall_start = None
    try:
        cap = cv2.VideoCapture(str(Path(args.video).resolve()))
        if not cap.isOpened():
            raise RuntimeError("Cannot decode video; check OpenCV video codec support")
        fps = args.fps if args.fps is not None else cap.get(cv2.CAP_PROP_FPS)
        if not math.isfinite(fps) or fps <= 0:
            raise ValueError("Invalid source FPS; supply --fps")
        if args.backend == "onnx":
            detector = OnnxDetector(args, names)
        elif args.backend == "rknn":
            detector = RknnDetector(args, names)
        print("[Input] %s | %.3f source fps | %s | stride=%d" %
              (args.video, fps, args.backend, args.stride))
        print("[Color] %s | candidates=%s | normalized heuristic scores, not calibrated accuracy" %
              (color_cfg.decision_mode, ",".join(color_cfg.candidates)))
        wall_start = time.perf_counter()
        fieldnames = ["frame", "video_seconds", "track_id", "raw_color", "stable_color", "status",
                      "det_confidence", "score", "reason", "forced_choice", "evidence_strength",
                      "candidate_margin", "p_red", "p_yellow", "p_green", "x1", "y1", "x2", "y2"]
        with (output / "frames.jsonl").open("w", encoding="utf-8") as logs, \
                (output / "lights.csv").open("w", encoding="utf-8-sig", newline="") as table:
            csv_writer = csv.DictWriter(table, fieldnames=fieldnames)
            csv_writer.writeheader()
            while not args.max_frames or processed < args.max_frames:
                read_start = time.perf_counter()
                ok, frame = cap.read()
                read_end = time.perf_counter()
                if not ok:
                    break
                index = source_index
                source_index += 1
                if index % args.stride:
                    continue
                # Fixed-rate video source time, independent of processing speed.
                timestamp = index / fps
                if detector:
                    detections, stages = detector.infer(frame)
                else:
                    detections = np.asarray([list(box) + [1.0, 0] for box in args.roi], dtype=np.float32)
                    stages = {"preprocess_ms": 0.0, "inference_ms": 0.0, "postprocess_ms": 0.0}
                color_start = time.perf_counter()
                results = engine.update(frame, detections, names, timestamp)
                color_end = time.perf_counter()
                stages["decode_video_ms"] = (read_end - read_start) * 1000
                stages["color_tracking_ms"] = (color_end - color_start) * 1000
                for item in results:
                    for counts, key in ((color_counts, "stable_color"), (raw_counts, "raw_color")):
                        counts[item[key]] = counts.get(item[key], 0) + 1
                    row = {key: item[key] for key in fieldnames if key in item}
                    row.update(frame=index, video_seconds=round(timestamp, 6))
                    row.update(zip(("x1", "y1", "x2", "y2"), item["box"]))
                    row.update({"p_" + c: item["probabilities"].get(c, "") for c in ("red", "yellow", "green")})
                    csv_writer.writerow(row)
                    if args.debug_every and processed % args.debug_every == 0:
                        x1, y1, x2, y2 = item["box"]
                        crop_dir = output / "crops"
                        crop_dir.mkdir(exist_ok=True)
                        cv2.imwrite(str(crop_dir / ("f%06d_id%d_%s.jpg" %
                                    (index, item["track_id"], item["raw_color"]))), frame[y1:y2, x1:x2])
                light_count += len(results)
                vis_start = time.perf_counter()
                vis = None
                if not args.no_video or args.show:
                    title = "src %.2fs | YOLO %.1fms | HSV+track %.1fms | TL %d" % (
                        timestamp, stages["inference_ms"], stages["color_tracking_ms"], len(results))
                    vis = draw_results(frame, detections, names, results, title,
                                       display=args.display, show_unknown=args.show_unknown)
                if not args.no_video:
                    if writer is None:
                        h, w = frame.shape[:2]
                        writer = cv2.VideoWriter(str(output / "annotated.avi"),
                                                 cv2.VideoWriter_fourcc(*"MJPG"), fps / args.stride, (w, h))
                        if not writer.isOpened():
                            raise RuntimeError("MJPG output unavailable; use --no-video to test logs")
                    writer.write(vis)
                stages["render_encode_ms"] = (time.perf_counter() - vis_start) * 1000
                record = {"frame": index, "video_seconds": timestamp, "detections": len(detections),
                          "yolo_detections": np.asarray(detections).tolist(),
                          "traffic_lights": results, "timings_ms": stages}
                logs.write(json.dumps(record, ensure_ascii=False) + "\n")
                for name, milliseconds in stages.items():
                    stage_sum[name] = stage_sum.get(name, 0) + milliseconds
                processed += 1
                if processed % 30 == 0:
                    rate = processed / max(time.perf_counter() - wall_start, 1e-9)
                    print("[Progress] %d frames | %.2f video s | %.2f processed fps | TL %d" %
                          (processed, timestamp, rate, light_count), flush=True)
                if args.show:
                    cv2.imshow("YOLO + traffic light color", vis)
                    if cv2.waitKey(1) & 0xff == ord("q"):
                        aborted = True
                        break
        if not processed:
            raise RuntimeError("No readable frames in video")
    except KeyboardInterrupt:
        aborted = True
        print("Stopped; preserving completed frames")
    except Exception as exc:
        error_message = str(exc)
        raise
    finally:
        elapsed = time.perf_counter() - wall_start if wall_start is not None else 0.0
        if cap is not None:
            cap.release()
        if writer is not None:
            writer.release()
        if detector is not None:
            detector.close()
        if args.show:
            cv2.destroyAllWindows()
        summary = {"backend": args.backend, "video": str(Path(args.video).resolve()), "model": args.model,
                   "classes": names, "processed_frames": processed, "decoded_source_frames": source_index,
                   "source_fps": locals().get("fps"), "stride": args.stride,
                   "display": args.display,
                   "timebase": "frame_index / source_fps (constant-rate video)",
                   "processing_wall_seconds": elapsed,
                   "processing_fps": processed / elapsed if elapsed > 0 else 0,
                   "traffic_light_observations": light_count, "raw_color_counts": raw_counts,
                   "stable_color_counts": color_counts,
                   "average_ms": {key: value / max(processed, 1) for key, value in stage_sum.items()},
                   "aborted": aborted, "error": error_message,
                   "validation_note": "Counts are not accuracy. ROI mode does not test YOLO. Forced probabilities are normalized heuristic scores, not calibrated accuracy.",
                   "config": {"color": vars(color_cfg), "state": vars(state_cfg)}}
        (output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print("[Done] %d frames | %.2f actual processing fps | %s" %
          (processed, summary["processing_fps"], output))
    print("[Average ms] " + json.dumps(summary["average_ms"]))
    if not light_count:
        print("[Notice] No traffic lights detected: color recognition could not be evaluated.")


if __name__ == "__main__":
    main()
