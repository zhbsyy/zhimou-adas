"""CARLA HIL: independent video, detection and Front segmentation workers.

This is an experimental, versioned alternative to pcie_stream_r15.py. It
reuses that file's RKNN decoding, JPEG encoder, HTTP handler and OIB link.
CARLA mosaic or a local video file can be used. Detection and R15 access the RK3568
NPU under one lock; their CPU preprocessing/postprocessing can overlap.
"""
import argparse
import os
import queue
import sys
import threading
import time
from http.server import ThreadingHTTPServer

import numpy as np

import pcie_stream_r15_profile as base
from runtime_backend import check_model, runtime_factory, verify_outputs
from trial_metrics import TrialMetrics


NPU_LOCK = threading.Lock()
STATE_LOCK = threading.Lock()
ACTIVE_SOURCE = None
STATE = {
    "dets": np.zeros((0, 6), np.float32),
    "det_at": 0.0,
    "masks": None,
    "lane_at": 0.0,
    "lane_shape": None,
}


def ema(key, value):
    old = base.STATS.get(key, 0.0)
    base.STATS[key] = value if old <= 0 else 0.9 * old + 0.1 * value


def capture_worker(args, q_det, q_lane, q_raw):
    global ACTIVE_SOURCE
    source = None
    seq = 0
    last_video_loop = 0
    last = time.monotonic()
    try:
        if args.source == "video":
            from video_source import VideoFrameSource
            source = VideoFrameSource(args.video, loop=args.video_loop,
                                      max_width=args.video_max_width)
        elif args.carla_decode_backend == "mpp_rga_cached":
            # Reuse the already board-tested P2 MPP -> RGA adapter unchanged.
            # Its read_new_bgr contract matches CarlaFrameSource.
            sys.path.insert(0, args.p2_mpp_dir)
            from p2_native_mpp_source import CarlaMppFrameAdapter
            source = CarlaMppFrameAdapter(
                args.carla_url,
                program=os.path.join(args.p2_mpp_dir, "mpp_native_stage"))
        else:
            source = base.CarlaFrameSource(stream_url=args.carla_url)
        source.start()
        ACTIVE_SOURCE = source
        while not base.STOP:
            bgr, seq, captured_at = source.read_new_bgr(seq, timeout=0.2)
            if bgr is None:
                if getattr(source, "eof", False):
                    print("[本地视频] 到达文件末尾", flush=True)
                    base.STOP = True
                    break
                continue
            if args.source == "video" and source.loops != last_video_loop:
                last_video_loop = source.loops
                with STATE_LOCK:
                    STATE["dets"] = np.zeros((0, 6), np.float32)
                    STATE["det_at"] = 0.0
                    STATE["masks"] = None
                    STATE["lane_at"] = 0.0
                    STATE["lane_shape"] = None
            now = time.monotonic()
            ema("cap_fps", 1.0 / max(now - last, 1e-6))
            last = now
            base.STATS["source"] = args.source
            base.STATS["cap_count"] += 1
            if not base.STATS["count_start"]:
                base.STATS["count_start"] = now
            item = (bgr, captured_at)
            base.put_latest(q_det, item)
            base.put_latest(q_lane, item)
            base.put_latest(q_raw, item)
    except Exception as exc:
        base.STATS["source_error"] = str(exc)
        print("[异步采集失败] %s" % exc, flush=True)
        base.STOP = True
    finally:
        ACTIVE_SOURCE = None
        try:
            if source is not None:
                source.close()
        except Exception:
            pass


def detector_worker(args, names, rknn, q_det):
    count = 0
    try:
        while not base.STOP:
            try:
                bgr, captured_at = q_det.get(timeout=0.5)
            except queue.Empty:
                continue
            count += 1
            if (count - 1) % args.infer_every:
                continue
            started = time.perf_counter()
            tensor, transform = base.rknn_input(bgr, args.size)
            ema("det_pre_ms", (time.perf_counter() - started) * 1e3)
            started = time.perf_counter()
            with NPU_LOCK:
                lock_acquired = time.perf_counter()
                outs = rknn.inference(inputs=[tensor], data_format="nhwc")
            call_end = time.perf_counter()
            if outs is None:
                raise RuntimeError("detector inference returned None")
            ema("inf_ms", (call_end - started) * 1e3)
            ema("det_lock_ms", (lock_acquired - started) * 1e3)
            ema("det_call_ms", (call_end - lock_acquired) * 1e3)
            started = time.perf_counter()
            dets = base.decode_auto(outs, size=args.size, nc=len(names),
                                    confidence=args.conf)
            if dets.size:
                dets = base.to_original(dets, transform)
            ema("det_post_ms", (time.perf_counter() - started) * 1e3)
            with STATE_LOCK:
                STATE["dets"] = dets.copy()
                STATE["det_at"] = time.monotonic()
            base.STATS["det"] = len(dets)
            base.STATS["det_count"] += 1
            base.STATS["inf_age_ms"] = (time.time() - captured_at) * 1e3
            # CARLA provides no stereo range. Preserve the original OIB
            # detection reporting without inventing distance or lane decisions.
            if base.OIB_LINK is not None:
                started = time.perf_counter()
                try:
                    base.OIB_LINK.evaluate_and_report(
                        dets=dets, stereo_pairs=[], tl_state=None)
                except Exception as exc:
                    base.STATS["oib_err"] = str(exc)[:80]
                ema("oib_ms", (time.perf_counter() - started) * 1e3)
    except Exception as exc:
        base.STATS["detector_error"] = str(exc)[:100]
        print("[检测线程停止] %s" % exc, flush=True)
        # A missing detector must not leave OIB sending stale decisions.
        if base.OIB_LINK is not None:
            try:
                base.OIB_LINK.stop()
            except Exception:
                pass
            base.OIB_LINK = None
        base.STOP = True


def lane_worker(args, lane_model, q_lane):
    period = 1.0 / args.lane_hz
    next_due = 0.0
    try:
        while not base.STOP:
            try:
                bgr, _captured_at = q_lane.get(timeout=0.5)
            except queue.Empty:
                continue
            now = time.monotonic()
            if now < next_due:
                continue
            next_due = now + period
            front = (bgr if args.source == "video" and args.video_layout == "single"
                     else bgr[:bgr.shape[0] // 2, :bgr.shape[1] // 2])
            masks, timing = lane_model.infer(front)
            with STATE_LOCK:
                STATE["masks"] = masks
                STATE["lane_at"] = time.monotonic()
                STATE["lane_shape"] = front.shape[:2]
            base.STATS["lane_ms"] = timing["infer_ms"]
            ema("lane_lock_ms", timing["lock_wait_ms"])
            ema("lane_call_ms", timing["call_ms"])
            base.STATS["r15_pre_ms"] = timing["pre_ms"]
            base.STATS["r15_post_ms"] = timing["post_ms"]
            base.STATS["direct_pixels"] = timing["direct_pixels"]
            base.STATS["solid_pixels"] = timing["solid_pixels"]
            base.STATS["dashed_pixels"] = timing["dashed_pixels"]
            base.STATS["lane_count"] += 1
            base.STATS["lane_status"] = "on"
    except Exception as exc:
        base.STATS["lane_status"] = "error: %s" % str(exc)[:80]
        print("[R15 线程停止，视频与检测继续] %s" % exc, flush=True)
        with STATE_LOCK:
            STATE["masks"] = None


def render_worker(args, names, q_raw, q_vis):
    empty = np.zeros((0, 6), np.float32)
    cached_masks, cached_overlay = None, None
    try:
        while not base.STOP:
            try:
                bgr, captured_at = q_raw.get(timeout=0.5)
            except queue.Empty:
                continue
            now = time.monotonic()
            with STATE_LOCK:
                dets = STATE["dets"].copy()
                det_at = STATE["det_at"]
                masks = STATE["masks"]
                lane_at = STATE["lane_at"]
                lane_shape = STATE["lane_shape"]
            if now - det_at > args.det_ttl:
                dets = empty
            frame = bgr.copy()
            h, w = frame.shape[:2]
            single = args.source == "video" and args.video_layout == "single"
            front = frame if single else frame[:h // 2, :w // 2]
            if masks is not None and lane_shape == front.shape[:2] and (
                    now - lane_at <= args.lane_ttl):
                started = time.perf_counter()
                if args.cached_display:
                    from cached_display import prepare_masks, apply_masks_inplace
                    if masks is not cached_masks:
                        cached_overlay = prepare_masks(masks)
                        cached_masks = masks
                    counts = apply_masks_inplace(front, cached_overlay)
                else:
                    marked, counts = base.draw_lane_mask(front, masks)
                    if single:
                        frame = marked
                    else:
                        frame[:h // 2, :w // 2] = marked
                base.STATS["lane_pixels"] = counts["solid_line"] + counts["dashed_line"]
                ema("lane_draw_ms", (time.perf_counter() - started) * 1e3)
            else:
                base.STATS["lane_pixels"] = 0
            title = ("%s | %s | cap %.1f | det %.1fHz | lane %.1fHz | "
                     "det_age %.0fms | lane_age %.0fms"
                     % (os.path.basename(args.model), args.source.upper(), base.STATS["cap_fps"],
                        base.STATS["inf_fps"], base.STATS["lane_fps"],
                        max(0.0, now - det_at) * 1e3 if det_at else -1,
                        max(0.0, now - lane_at) * 1e3 if lane_at else -1))
            started = time.perf_counter()
            if args.cached_display:
                from cached_display import draw_owned
                vis = draw_owned(frame, dets, names, title)
            else:
                vis = base.draw(frame, dets, names, title)
            ema("vis_draw_ms", (time.perf_counter() - started) * 1e3)
            base.put_latest(q_vis, (vis, captured_at))
            base.STATS["processed_count"] += 1
    except Exception as exc:
        base.STATS["render_error"] = str(exc)
        print("[异步绘制失败] %s" % exc, flush=True)
        base.STOP = True


def reporter_worker(args):
    previous = {
        "cap_count": 0, "det_count": 0, "lane_count": 0,
        "processed_count": 0, "stream_count": 0,
    }
    last = time.monotonic()
    last_hw_produced = 0
    last_hw_collected = 0
    last_hw_submitted = 0
    while not base.STOP:
        time.sleep(5)
        now = time.monotonic()
        delta = max(now - last, 1e-6)
        rates = {}
        for key in previous:
            count = base.STATS[key]
            rates[key] = (count - previous[key]) / delta
            previous[key] = count
        last = now
        base.STATS["inf_fps"] = rates["det_count"]
        base.STATS["lane_fps"] = rates["lane_count"]
        base.STATS["process_fps"] = rates["processed_count"]
        print("[异步 5s] 采集 %.1f | 检测 %.1f (等锁 %.1fms RKNN调用 %.1fms) | "
              "R15 %.1f (等锁 %.1fms RKNN调用 %.1fms 后处理 %.1fms) | "
              "绘图 %.1f (掩膜 %.1fms 目标 %.1fms) | JPEG %.1f | "
              "丢帧 %d | 目标 %d | lane %s"
              % (rates["cap_count"], rates["det_count"],
                 base.STATS.get("det_lock_ms", 0.0), base.STATS.get("det_call_ms", 0.0),
                 rates["lane_count"], base.STATS.get("lane_lock_ms", 0.0),
                 base.STATS.get("lane_call_ms", 0.0), base.STATS["r15_post_ms"],
                 rates["processed_count"], base.STATS["lane_draw_ms"],
                 base.STATS["vis_draw_ms"], rates["stream_count"],
                 base.STATS["drop"], base.STATS["det"],
                 base.STATS["lane_status"]), flush=True)
        source = ACTIVE_SOURCE
        if args.source == "carla" and source is not None and hasattr(source, "snapshot"):
            status = source.snapshot()
            print("[MPP源 5s] 解码 %d | 跳帧 %d | 解码错误 %d | 重连 %d | "
                  "连接 %s | 最近帧 %.1fms | 错误 %s"
                  % (status["decoded_count"], status["native_frames_skipped"],
                     status["decode_failures"], status["reconnects"],
                     status["connected"], status["last_frame_age_ms"] or -1.0,
                     status["last_error"] or "none"), flush=True)
        if base.HW_ENC is not None:
            hw = base.HW_ENC
            produced = hw.stats["produced"]
            collected = hw.stats["hw"]
            submitted = hw.stats["submitted"]
            print("[编码 5s] 提交 %.1f | 硬件产出 %.1f | 取回 %.1f | "
                  "实际发布 %.1f | 输出队列 %d | 输入队列 %d"
                  % ((submitted - last_hw_submitted) / delta,
                     (produced - last_hw_produced) / delta,
                     (collected - last_hw_collected) / delta,
                     rates["stream_count"], hw._out_q.qsize(),
                     hw._in_q.qsize() if hw._in_q is not None else 0),
                  flush=True)
            last_hw_produced = produced
            last_hw_collected = collected
            last_hw_submitted = submitted
        for name, runtime in getattr(args, "trial_runtimes", {}).items():
            if getattr(runtime, "last", None):
                print("[原生接口 %s] %s" % (name, runtime.last), flush=True)


def main():
    parser = argparse.ArgumentParser(description="CARLA or local video async dual RKNN trial")
    parser.add_argument("--source", choices=("carla", "video"), default="carla")
    parser.add_argument("--video", help="local MP4 path, required with --source video")
    parser.add_argument("--video_layout", choices=("single", "quad"), default="single",
                        help="single camera or FRONT/RIGHT/REAR/LEFT 2x2 mosaic")
    parser.add_argument("--video_loop", action="store_true",
                        help="replay from the beginning at EOF for browser inspection")
    parser.add_argument("--video_max_width", type=int, default=1280,
                        help="limit local video width before model input and preview")
    parser.add_argument("--carla_url", default="http://192.168.2.10:8081/stream")
    parser.add_argument("--carla_decode_backend",
                        choices=("opencv_cpu", "mpp_rga_cached"),
                        default="opencv_cpu")
    parser.add_argument("--p2_mpp_dir",
                        default="/home/linaro/p2_carla_integrated_candidate_20261017")
    parser.add_argument("--pc_ip", default="192.168.2.10")
    parser.add_argument("--model", required=True)
    parser.add_argument("--classes", required=True)
    parser.add_argument("--size", type=int, default=640)
    parser.add_argument("--lane_model", required=True)
    parser.add_argument("--lane_height", type=int, choices=(256, 384, 640), default=384)
    parser.add_argument("--lane_width", type=int, choices=(448, 640), default=640)
    parser.add_argument("--lane_input_mode", choices=("uint8_nhwc", "float_nchw"),
                        default="uint8_nhwc")
    parser.add_argument("--lane_conf", type=float, default=0.6)
    parser.add_argument("--lane_hz", type=float, default=2.0)
    parser.add_argument("--infer_every", type=int, default=1)
    parser.add_argument("--det_ttl", type=float, default=0.5)
    parser.add_argument("--lane_ttl", type=float, default=1.0)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--quality", type=int, default=85)
    parser.add_argument("--encode_scale", type=float, default=1.0)
    parser.add_argument("--q_depth", type=int, default=1)
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--oib_port", type=int, default=8888)
    parser.add_argument("--no_oib", action="store_true")
    parser.add_argument("--runtime_backend", choices=("lite", "native_float", "native_buffered"), default="lite")
    parser.add_argument("--roi_overlay", action="store_true")
    parser.add_argument("--cached_display", action="store_true")
    parser.add_argument("--publication_callback", action="store_true")
    parser.add_argument("--lane_argmax", choices=("numpy", "planar"), default="numpy")
    parser.add_argument("--run_seconds", type=float, default=35.0,
                        help="finite live trial duration, excluding preflight; 0 means until Ctrl+C")
    parser.add_argument("--warmup_seconds", type=float, default=10.0)
    parser.add_argument("--preflight_video", default="/home/linaro/yolov8_deploy/videos/video_20260929_112507.mp4")
    args = parser.parse_args()
    if args.infer_every < 1 or args.lane_hz <= 0 or args.q_depth < 1 or args.video_max_width < 320:
        parser.error("infer_every, lane_hz and q_depth must be positive")
    if (args.lane_height, args.lane_width) not in ((256, 448), (384, 640), (640, 640)):
        parser.error("lane dimensions must be 256x448, 384x640 or 640x640")
    if args.source == "video" and not args.video:
        parser.error("--video is required with --source video")
    if args.source == "video" and not args.no_oib:
        parser.error("local video replay requires --no_oib; recorded frames must not control CARLA")
    if args.run_seconds < 0 or args.warmup_seconds < 0 or (
            args.run_seconds and args.run_seconds < args.warmup_seconds + 10):
        parser.error("finite trial needs at least 10 measured seconds after warmup")
    if args.roi_overlay and args.cached_display:
        parser.error("Choose one display candidate at a time")
    if args.lane_argmax == "planar" and args.runtime_backend != "native_buffered":
        parser.error("planar comparison uses the verified native_buffered publish pipeline")
    if args.runtime_backend.startswith("native_") and (
            args.size != 640 or (args.lane_height, args.lane_width) != (256, 448) or
            args.lane_input_mode != "uint8_nhwc"):
        parser.error("native trial is pinned to current 640 detector and uint8 NHWC 256x448 R15")
    if args.source == "carla" and args.carla_decode_backend == "opencv_cpu" and not base.HAVE_CARLA:
        parser.error("CARLA source unavailable: %s" % base._CARLA_IMPORT_ERR)
    if args.source == "carla" and args.carla_decode_backend == "mpp_rga_cached":
        if not all(os.path.isfile(os.path.join(args.p2_mpp_dir, name)) for name in
                   ("p2_native_mpp_source.py", "p2_fresh_source.py")):
            parser.error("P2 MPP Python adapter is missing")
        if not os.access(os.path.join(args.p2_mpp_dir, "mpp_native_stage"), os.X_OK):
            parser.error("P2 MPP binary is missing or not executable")
    names = [line.strip() for line in open(args.classes, encoding="utf-8") if line.strip()]
    for path in (args.model, args.lane_model):
        check_model(path)
    if args.runtime_backend.startswith("native_"):
        import json
        import pcie_post
        import r15_front_fast
        verified = verify_outputs({"det": args.model, "lane": args.lane_model},
                                  pcie_post, r15_front_fast, args.preflight_video,
                                  backend=args.runtime_backend)
        print("[原生预检] " + json.dumps(verified, ensure_ascii=False), flush=True)
    factory = runtime_factory(args.runtime_backend)
    if args.lane_argmax == "planar":
        import json
        import r15_front_fast
        from postprocess_candidate import verify_model_masks
        mask_checks = verify_model_masks(args.lane_model, args.preflight_video,
                                        factory, r15_front_fast, args.lane_conf)
        print("[R15 掩膜预检] " + json.dumps(mask_checks, ensure_ascii=False), flush=True)
    detector = factory()
    if detector.load_rknn(args.model) != 0 or detector.init_runtime() != 0:
        raise RuntimeError("detector RKNN load/init failed")
    probe = np.full((1, args.size, args.size, 3), 114, np.uint8)
    outputs = detector.inference(inputs=[probe], data_format="nhwc")
    cells = (args.size // 8) ** 2
    if outputs is None or len(outputs) < 9:
        raise RuntimeError("detector preflight: expected nine outputs")
    classes, remainder = divmod(np.asarray(outputs[1]).size, cells)
    if remainder or classes != len(names):
        raise RuntimeError("detector preflight: model %d classes, file %d" %
                           (classes, len(names)))
    print("[检测类别] %d classes, final class = %s" %
          (classes, names[-1]), flush=True)
    lane_model = base.LiveLaneModel(args.lane_model, None, args.lane_conf,
                                    args.lane_height, args.lane_input_mode,
                                    args.lane_width, runtime_factory=factory)
    lane_model.npu_lock = NPU_LOCK
    if args.lane_argmax == "planar":
        from postprocess_candidate import lane_argmax_planar
        lane_model.lane_argmax = lane_argmax_planar
    base.STATS["lane_argmax"] = args.lane_argmax
    args.trial_runtimes = {"det": detector, "lane": lane_model.rk}
    if args.roi_overlay:
        from overlay_roi_candidate import draw_lane_mask_roi
        base.draw_lane_mask = draw_lane_mask_roi
    print("[独立候选] runtime=%s roi_overlay=%s cached_display=%s shared_lock=on duration=%.1fs" %
          (args.runtime_backend, args.roi_overlay, args.cached_display, args.run_seconds), flush=True)
    print("[R15 CPU后处理] lane_argmax=%s" % args.lane_argmax, flush=True)
    base.STATS["lane_status"] = "ready"
    base.STOP = False
    base.HW_ENC = (base.JpegEncoder(width=None, height=None, quality=args.quality,
                                   verbose=True) if base.HAVE_HW_JPEG else None)
    base.STATS["hw"] = "on" if base.HW_ENC is not None else "off"
    if args.publication_callback:
        from publication_candidate import install_callback, submit_worker
        install_callback(base)
        def encode_target(arguments, frames):
            return submit_worker(base, arguments, frames)
    else:
        encode_target = base.encode_thread
    if not args.no_oib:
        if not base.HAVE_OIB:
            raise RuntimeError("OIB link unavailable: %s" % base._OIB_IMPORT_ERR)
        base.OIB_LINK = base.OibLink(pc_ip=args.pc_ip, oib_port=args.oib_port,
                                     listen_port=8892)
        base.OIB_LINK.start()
    q_det = queue.Queue(maxsize=args.q_depth)
    q_lane = queue.Queue(maxsize=args.q_depth)
    q_raw = queue.Queue(maxsize=args.q_depth)
    q_vis = queue.Queue(maxsize=args.q_depth)
    # The base hardware encoder and HTTP handler retain the same endpoints.
    args.fps_print = 0
    workers = (
        (capture_worker, (args, q_det, q_lane, q_raw)),
        (detector_worker, (args, names, detector, q_det)),
        (lane_worker, (args, lane_model, q_lane)),
        (render_worker, (args, names, q_raw, q_vis)),
        (encode_target, (args, q_vis)),
        (reporter_worker, (args,)),
    )
    server = ThreadingHTTPServer(("0.0.0.0", args.port), base.Handler)
    for target, params in workers:
        threading.Thread(target=target, args=params, daemon=True).start()
    meter = TrialMetrics(base.STATS, warmup=args.warmup_seconds)
    timer = None
    if args.run_seconds:
        timer = threading.Timer(args.run_seconds, server.shutdown)
        timer.daemon = True
        timer.start()
    print("[异步双模型] 检测输入全部画面；R15 处理 Front；视频与推理分离；输入 %s"
          % (args.carla_decode_backend if args.source == "carla" else
             "video/%s" % args.video_layout), flush=True)
    print("[地址] http://<板子IP>:%d/stream  |  /stat" % args.port, flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        base.STOP = True
        if timer is not None:
            timer.cancel()
        trial_errors = []
        for key in ("source_error", "render_error", "publication_error"):
            if base.STATS.get(key):
                trial_errors.append(key + ": " + base.STATS[key])
        if base.STATS.get("detector_error"):
            trial_errors.append(base.STATS["detector_error"])
        if str(base.STATS.get("lane_status", "")).startswith("error"):
            trial_errors.append(base.STATS["lane_status"])
        if not base.STATS["cap_count"]:
            trial_errors.append("No captured input frames")
        meter.finish(args.runtime_backend, args.roi_overlay, trial_errors,
                     display_mode="cached" if args.cached_display else "roi" if args.roi_overlay else "original",
                     publication_mode="hardware_callback" if args.publication_callback else "submit_collect_latest")
        try:
            server.shutdown()
        except Exception:
            pass
        if base.OIB_LINK is not None:
            try:
                base.OIB_LINK.stop()
            except Exception:
                pass
        if base.HW_ENC is not None:
            try:
                print("[编码统计] " + base.HW_ENC.report(), flush=True)
                base.HW_ENC.close()
            except Exception:
                pass
        time.sleep(0.3)
        print("已停止", flush=True)
        os._exit(1 if trial_errors else 0)


if __name__ == "__main__":
    main()
