"""Process a local single-camera video into a finite annotated video file.

This batch tool never starts HTTP or OIB and does not read the CARLA stream.
It reuses the same detector and R15 preprocessing/postprocessing as the live
candidate. It is intended for visual assessment, not CARLA FPS comparison.
"""

from __future__ import annotations

import argparse
import json
import itertools
import traceback
from pathlib import Path
import shutil
import time

import cv2
import numpy as np

import offline_pipeline as base


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--video", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--model", required=True)
    parser.add_argument("--classes", required=True)
    parser.add_argument("--lane_model", required=True)
    parser.add_argument("--size", type=int, default=640)
    parser.add_argument("--lane_height", type=int, default=256)
    parser.add_argument("--lane_width", type=int, default=448)
    parser.add_argument("--lane_input_mode", default="uint8_nhwc")
    parser.add_argument("--lane_conf", type=float, default=0.6)
    parser.add_argument("--stride", type=int, default=1,
                        help="process every Nth source frame; output FPS is source FPS/N")
    parser.add_argument("--start", type=float, default=0.0)
    parser.add_argument("--seconds", type=float, default=0.0)
    parser.add_argument("--max_width", type=int, default=1280)
    parser.add_argument("--class10_conf", type=float, default=0.60,
                        help="crosswalk minimum confidence; all other classes remain 0.25")
    args = parser.parse_args()
    if args.stride < 1 or args.seconds < 0 or args.start < 0 or (args.max_width != 0 and args.max_width < 320):
        parser.error("invalid stride/start/seconds/max_width")
    if not 0.25 <= args.class10_conf <= 1.0:
        parser.error("class10_conf must be in [0.25, 1.0]")
    source = Path(args.video)
    output = Path(args.output)
    if not source.is_file():
        parser.error("input video missing: %s" % source)
    if output.exists():
        parser.error("output already exists: %s" % output)
    if output.suffix.lower() not in (".mp4", ".avi"):
        parser.error("output must be .mp4 or .avi")
    output.parent.mkdir(parents=True, exist_ok=True)
    free = shutil.disk_usage(output.parent).free
    if free < 64 * 1024 * 1024:
        parser.error("less than 64 MiB free near output; choose another directory")

    cap = cv2.VideoCapture(str(source))
    if not cap.isOpened():
        raise RuntimeError("OpenCV cannot decode video: %s" % source)
    fps = cap.get(cv2.CAP_PROP_FPS)
    if not 1.0 <= fps <= 120.0:
        raise RuntimeError("invalid source FPS: %s" % fps)
    source_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    if args.start:
        cap.set(cv2.CAP_PROP_POS_MSEC, args.start * 1000.0)
    names = [line.strip() for line in Path(args.classes).read_text(encoding="utf-8").splitlines()
             if line.strip()]
    if len(names) != 11 or names[10] != "crosswalk":
        cap.release()
        raise RuntimeError("expected the frozen 11-class order with class10=crosswalk")
    from rknnlite.api import RKNNLite
    detector = RKNNLite()
    lane = None
    writer = None
    counts = {"source_read": 0, "processed": 0, "detections": 0,
              "direct_pixels": 0, "solid_pixels": 0, "dashed_pixels": 0}
    times = {"det_pre_ms": 0.0, "det_infer_ms": 0.0, "det_post_ms": 0.0,
             "lane_pre_ms": 0.0, "lane_infer_ms": 0.0,
             "lane_post_ms": 0.0, "draw_ms": 0.0}
    began = time.monotonic()
    stop_reason = "duration" if args.seconds else "source_eof"
    error = None
    try:
        if detector.load_rknn(args.model) != 0 or detector.init_runtime() != 0:
            raise RuntimeError("detector RKNN load/init failed")
        probe = np.full((1, args.size, args.size, 3), 114, np.uint8)
        outs = detector.inference(inputs=[probe], data_format="nhwc")
        cells = (args.size // 8) ** 2
        if outs is None or len(outs) < 9 or np.asarray(outs[1]).size != cells * len(names):
            raise RuntimeError("detector outputs do not match classes/size")
        lane = base.LiveLaneModel(args.lane_model, None, args.lane_conf,
                                  args.lane_height, args.lane_input_mode,
                                  args.lane_width)
        limit_read = round(args.seconds * fps) if args.seconds else None
        for frame_number in (range(limit_read) if limit_read is not None else itertools.count()):
            ok, frame = cap.read()
            if not ok or frame is None:
                expected_remaining = max(0, source_frames - round(args.start * fps))
                stop_reason = "decode_ended_early" if source_frames and counts["source_read"] < expected_remaining - 1 else "source_eof"
                break
            counts["source_read"] += 1
            if frame_number % args.stride:
                continue
            if args.max_width and frame.shape[1] > args.max_width:
                new_h = round(frame.shape[0] * args.max_width / frame.shape[1])
                frame = cv2.resize(frame, (args.max_width, new_h),
                                   interpolation=cv2.INTER_AREA)

            t = time.perf_counter()
            tensor, transform = base.rknn_input(frame, args.size)
            times["det_pre_ms"] += (time.perf_counter() - t) * 1000
            t = time.perf_counter()
            outs = detector.inference(inputs=[tensor], data_format="nhwc")
            times["det_infer_ms"] += (time.perf_counter() - t) * 1000
            if outs is None:
                raise RuntimeError("detector returned no outputs")
            t = time.perf_counter()
            dets = base.decode_auto(outs, size=args.size, nc=len(names),
                                    confidence=0.25)
            if dets.size:
                dets = base.to_original(dets, transform)
                dets = dets[(dets[:, 5] != 10) | (dets[:, 4] >= args.class10_conf)]
            times["det_post_ms"] += (time.perf_counter() - t) * 1000

            masks, timing = lane.infer(frame)
            times["lane_pre_ms"] += timing["pre_ms"]
            times["lane_infer_ms"] += timing["call_ms"]
            times["lane_post_ms"] += timing["post_ms"]
            counts["detections"] += len(dets)
            counts["direct_pixels"] += timing["direct_pixels"]
            counts["solid_pixels"] += timing["solid_pixels"]
            counts["dashed_pixels"] += timing["dashed_pixels"]

            t = time.perf_counter()
            marked, _ = base.draw_lane_mask(frame, masks)
            title = "OFFLINE | t=%.1fs | det=%d | R15" % (
                args.start + frame_number / fps, len(dets))
            vis = base.draw(marked, dets, names, title)
            times["draw_ms"] += (time.perf_counter() - t) * 1000
            h, w = vis.shape[:2]
            if h % 2 or w % 2:
                vis = cv2.copyMakeBorder(vis, 0, h % 2, 0, w % 2,
                                          cv2.BORDER_CONSTANT, value=(25, 25, 25))
            if writer is None:
                codec = "mp4v" if output.suffix.lower() == ".mp4" else "MJPG"
                fourcc = cv2.VideoWriter_fourcc(*codec)
                writer = cv2.VideoWriter(str(output), fourcc, fps / args.stride,
                                         (vis.shape[1], vis.shape[0]))
                if not writer.isOpened():
                    raise RuntimeError("OpenCV cannot open video writer for %s" % output)
                print("[OFFLINE_OUTPUT] %s | %dx%d | %.2f FPS" %
                      (output, vis.shape[1], vis.shape[0], fps / args.stride),
                      flush=True)
            writer.write(vis)
            counts["processed"] += 1
            if counts["processed"] % 20 == 0:
                print("[OFFLINE_PROGRESS] source=%d processed=%d det=%d" %
                      (counts["source_read"], counts["processed"], len(dets)),
                      flush=True)
                if shutil.disk_usage(output.parent).free < 64 * 1024 * 1024:
                    stop_reason = "low_disk_space"
                    break
    except KeyboardInterrupt:
        stop_reason = "interrupted"
    except Exception as exc:
        stop_reason = "error"
        error = str(exc)
        traceback.print_exc()
    finally:
        cap.release()
        if writer is not None:
            writer.release()
        if lane is not None:
            lane.close()
        detector.release()
    elapsed = time.monotonic() - began
    n = max(counts["processed"], 1)
    report = {"input": str(source), "output": str(output),
              "output_bytes": output.stat().st_size if output.exists() else 0,
              "class10_min_confidence": args.class10_conf, "other_min_confidence": 0.25,
              "lane_confidence": args.lane_conf, "stride": args.stride, "start_s": args.start,
              "requested_seconds": args.seconds, "source_reported_frames": source_frames,
              "source_reported_duration_s": source_frames / fps,
              "output_duration_s": counts["processed"] / (fps / args.stride),
              "input_order": "detector then R15, both from the same unannotated frame",
              "audio_included": False, "error": error,
              "complete": stop_reason in ("source_eof", "duration") and counts["processed"] > 0,
              "source_fps": fps, "output_fps": fps / args.stride,
              "elapsed_wall_s": round(elapsed, 3),
              "processed_per_wall_s": round(counts["processed"] / max(elapsed, 1e-6), 3),
              "stop_reason": stop_reason, **counts,
              "mean_ms": {key: round(value / n, 3) for key, value in times.items()}}
    output.with_suffix(".summary.json").write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    print("OFFLINE_SUMMARY " + json.dumps(report, ensure_ascii=False), flush=True)
    if not report["complete"]:
        raise SystemExit(2 if stop_reason != "error" else 1)


if __name__ == "__main__":
    main()
