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
import json
from http.server import ThreadingHTTPServer

import numpy as np
import cv2

import pcie_stream_r15_profile as base
from runtime_backend import check_model, runtime_factory, verify_outputs
from trial_metrics import TrialMetrics
from decision_service import DecisionService, load_config
from detector_thresholds import filter_scores


NPU_LOCK = threading.Lock()
STATE_LOCK = threading.Lock()
ACTIVE_SOURCE = None
DECISIONS = None
STATE = {
    "dets": np.zeros((0, 6), np.float32),
    "det_at": 0.0,
    "masks": None,
    "lane_at": 0.0,
    "lane_shape": None,
    "det_epoch": None,
    "lane_epoch": None,
}


def single_view(args):
    return args.source in ("pcie", "usb") or (args.source == "video" and args.video_layout == "single")


def front_rect(args, frame):
    h, w = frame.shape[:2]
    return (0, 0, w, h) if single_view(args) else (0, 0, w//2, h//2)


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
        camera_color = None
        if args.source == "pcie":
            from camera_input_color import CameraInputColor
            camera_color = CameraInputColor(getattr(args, "camera_color_profile", None))
            print("[相机输入颜色] "+json.dumps(camera_color.description, ensure_ascii=False), flush=True)
            from single_sources import PcieSingleSource
            if getattr(args,'stereo_capture',False):
                from stereo_source import PcieStereoSource
                source=PcieStereoSource(args.pcie_dir)
            else:
                source = PcieSingleSource(args.pcie_dir, tuple(args.front_crop))
        elif args.source == "usb":
            from single_sources import UsbSingleSource
            source = UsbSingleSource(args.usb_device)
        elif args.source == "video":
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
        epoch = DECISIONS.begin_source(args.source)
        while not base.STOP:
            stereo_right=None
            if args.source=='pcie' and getattr(args,'stereo_capture',False):
                bgr,stereo_right,seq,captured_at=source.read_new_stereo(seq,timeout=.2)
            else:
                bgr, seq, captured_at = source.read_new_bgr(seq, timeout=0.2)
            if bgr is None:
                if getattr(source, "eof", False):
                    print("[本地视频] 到达文件末尾", flush=True)
                    base.STOP = True
                    break
                continue
            if args.source == "video" and source.loops != last_video_loop:
                last_video_loop = source.loops
                epoch = DECISIONS.begin_source(args.source)
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
            raw_bgr = bgr
            if camera_color is not None:
                started = time.perf_counter()
                bgr = camera_color.apply(raw_bgr)
                ema("camera_color_ms", (time.perf_counter()-started)*1000)
            item = DECISIONS.frame(bgr, int(seq), epoch, received_mono=now, captured_at=captured_at,
                                   raw_bgr=raw_bgr if camera_color is not None else None,
                                   camera_color=camera_color.description if camera_color is not None else None,
                                   stereo_right=stereo_right,stereo_left_raw=raw_bgr if stereo_right is not None else None)
            if item is None:
                continue
            base.put_latest(q_det, item)
            base.put_latest(q_lane, item)
            base.put_latest(q_raw, item)
    except Exception as exc:
        DECISIONS.core.source_failed(str(exc))
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
                packet = q_det.get(timeout=0.5)
                bgr, captured_at = packet.bgr, packet.captured_at
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
                                    confidence=min(args.conf, .15))
            if dets.size:
                dets = base.to_original(dets, transform)
                # class10 (crosswalk): filter before render and decision summaries.
                dets = filter_scores(dets, names, base=args.conf)
            ema("det_post_ms", (time.perf_counter() - started) * 1e3)
            with STATE_LOCK:
                STATE["dets"] = dets.copy()
                STATE["det_at"] = packet.received_mono
                STATE["det_epoch"] = packet.epoch
            DECISIONS.submit_detection(packet, dets, names, front_rect(args, bgr))
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
        DECISIONS.core.source_failed("detector_failed: "+str(exc))
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
                packet = q_lane.get(timeout=0.5)
                bgr = packet.bgr
            except queue.Empty:
                continue
            now = time.monotonic()
            if now < next_due:
                continue
            next_due = now + period
            x, y, w, h = front_rect(args, bgr)
            front = bgr[y:y+h, x:x+w]
            masks, timing = lane_model.infer(front)
            DECISIONS.submit_lane(packet, masks)
            with STATE_LOCK:
                STATE["masks"] = masks
                STATE["lane_at"] = packet.received_mono
                STATE["lane_epoch"] = packet.epoch
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
        DECISIONS.core.source_failed("lane_failed: "+str(exc))
        base.STOP = True
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
                packet = q_raw.get(timeout=0.5)
                bgr, captured_at = packet.bgr, packet.captured_at
            except queue.Empty:
                continue
            now = time.monotonic()
            with STATE_LOCK:
                dets = STATE["dets"].copy()
                det_at = STATE["det_at"]
                masks = STATE["masks"]
                lane_at = STATE["lane_at"]
                lane_shape = STATE["lane_shape"]
                det_epoch = STATE["det_epoch"]
                lane_epoch = STATE["lane_epoch"]
            if now - det_at > args.det_ttl or det_epoch != packet.epoch:
                dets = empty
            canvas = None
            if args.cached_display:
                from cached_display import begin_frame
                canvas, frame = begin_frame(bgr)
            else:
                frame = bgr.copy()
            h, w = frame.shape[:2]
            single = single_view(args)
            front = frame if single else frame[:h // 2, :w // 2]
            if masks is not None and lane_shape == front.shape[:2] and (
                    now - lane_at <= args.lane_ttl and lane_epoch == packet.epoch):
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
            # Results remain tagged with their detector source frame. Never draw
            # old/other-source distances after a restart or queue backlog.
            summary=DECISIONS.snapshot()
            obstacle=summary['perception']['obstacles']
            if obstacle.get('fresh') and (obstacle.get('age_ms') or 0)<=300:
                for target in (obstacle.get('range') or {}).get('targets',[]):
                    if target.get('distance_m') is None:continue
                    x1,y1,x2,y2=map(int,target['box'])
                    tag='Z' if getattr(args,'aeb_mode','video_demo')=='stereo_distance' else 'surface'
                    text='%s %.2fm #%s' % (tag,target['distance_m'],obstacle['frame_seq'])
                    cv2.putText(frame,text,(max(0,x1),min(h-5,max(14,y2+14))),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,255,0),1,cv2.LINE_AA)
            started = time.perf_counter()
            if args.cached_display:
                from cached_display import draw_owned
                vis = draw_owned(frame, dets, names, title, canvas=canvas)
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
        summary = DECISIONS.snapshot()
        perception = summary["perception"]
        metrics = summary["metrics"]
        print("[RK摘要 5s] 灯=%s 左=%s 右=%s | 灯色 %.2fms 边界 %.2fms 规则 %.3fms | 错误 %d" % (
            perception["signal"]["color"], perception["boundaries"]["left"]["kind"],
            perception["boundaries"]["right"]["kind"], metrics["signal_ms"],
            metrics["boundaries_ms"], metrics["rule_ms"], metrics["summary_errors"]), flush=True)
        for name, runtime in getattr(args, "trial_runtimes", {}).items():
            if getattr(runtime, "last", None):
                print("[原生接口 %s] %s" % (name, runtime.last), flush=True)


def main():
    global DECISIONS
    parser = argparse.ArgumentParser(description="CARLA or local video async dual RKNN trial")
    parser.add_argument("--source", choices=("carla", "video", "pcie", "usb"), default="video")
    parser.add_argument("--pcie_dir", default="/home/linaro/board_files")
    parser.add_argument("--front_crop", type=int, nargs=4, default=(0,0,640,360))
    parser.add_argument("--camera_color_profile", default=None,
                        help="explicit PCIe input RGB gain JSON; never applies to video/USB/CARLA")
    parser.add_argument("--usb_device")
    parser.add_argument("--decision_config", default=os.path.join(os.path.dirname(__file__), "decision_config.json"))
    parser.add_argument("--decision_bind", default="0.0.0.0")
    parser.add_argument("--decision_port", type=int, default=18088)
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
    parser.add_argument("--port", type=int, default=18089)
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
    parser.add_argument('--stereo_capture',action='store_true')
    parser.add_argument('--stereo_calibration')
    parser.add_argument('--stereo_calibration_confirmed',action='store_true')
    parser.add_argument('--aeb_mode',choices=('video_demo','stereo_distance'),default='video_demo')
    args = parser.parse_args()
    base.STATS["runtime_profile"] = {"version": "stereo_decision.v0.1.3", "HD": False, "class10_min_conf": max(args.conf,.5), "light_candidate_conf": min(args.conf,.15), "other_class_conf": args.conf, "alarm": "automatic_conflict_and_3s_display", "driving_policy": "normal_drive_red_or_AEB_stop", "render_canvas": "one_allocation_one_input_copy"}
    if not args.no_oib:
        parser.error("Decision mode requires --no_oib; this interface supplies stationary simulation feedback only")
    if args.source == "usb" and not args.usb_device:
        parser.error("--usb_device is required")
    if min(args.front_crop) < 0 or min(args.front_crop[2:]) <= 0:
        parser.error("front_crop must be x y positive_width positive_height")
    if args.decision_port == args.port:
        parser.error("Video and decision interfaces need different ports")
    decision_config = load_config(args.decision_config)
    if args.source=='carla' or (args.source=='video' and args.video_layout!='single'):
        parser.error('This candidate requires one front view for pedestrian rules; use the retained CARLA/four-view entry for mosaics')
    if args.stereo_capture and (args.source!='pcie' or not args.stereo_calibration):
        parser.error('Stereo requires PCIe two-eye bitstream and calibration path')
    if args.aeb_mode=='stereo_distance' and not args.stereo_capture:
        parser.error('Metric AEB requires the independent two-eye capture source')
    decision_config['aeb']={**decision_config.get('aeb',{}),'enabled':True,'mode':args.aeb_mode}
    decision_config['stereo']={'enabled':args.stereo_capture,'calibration':args.stereo_calibration,
                                'confirmed':args.stereo_calibration_confirmed}
    base.STATS['runtime_profile'].update(stereo_capture=args.stereo_capture,aeb_mode=args.aeb_mode,
        display_eye='CMOS1',extra_NPU_calls=0,exposure_synchronized=False)
    base.STATS["runtime_profile"]["signal_algorithm"] = decision_config.get("scene", {}).get("signal_algorithm", "legacy_hsv")
    if args.camera_color_profile:
        if args.source != "pcie":
            parser.error("camera_color_profile is for the physical PCIe camera only")
        from camera_input_color import CameraInputColor
        CameraInputColor(args.camera_color_profile)  # validate before opening capture/NPU
    base.PAGE = ("<!doctype html><meta charset=utf-8><title>单路感知</title>"
                 "<style>body{background:#101827;color:white;font:18px sans-serif}img{max-width:100%%}</style>"
                 "<h2>单路感知视频</h2><p>灯色、左右边界与按钮请求：端口 %d；当前画面来自RK模型处理。</p>"
                 "<img src='/stream'>" % args.decision_port).encode("utf-8")
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
        import pcie_post
        import r15_front_fast
        verified = verify_outputs({"det": args.model, "lane": args.lane_model},
                                  pcie_post, r15_front_fast, args.preflight_video,
                                  backend=args.runtime_backend)
        print("[原生预检] " + json.dumps(verified, ensure_ascii=False), flush=True)
    factory = runtime_factory(args.runtime_backend)
    if args.lane_argmax == "planar":
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
    print("[检测阈值] base=%.2f | class10_min=0.50 | class10_effective=%.2f" %
          (args.conf, max(args.conf, 0.50)), flush=True)
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
    DECISIONS = DecisionService(decision_config, stats_provider=lambda: base.STATS)
    DECISIONS.start(args.decision_bind, args.decision_port, os.environ.get("RK_DECISION_TOKEN"))
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
    try:
        server = ThreadingHTTPServer(("0.0.0.0", args.port), base.Handler)
    except Exception:
        DECISIONS.close()
        raise
    threads = []
    for target, params in workers:
        thread = threading.Thread(target=target, args=params, daemon=True)
        thread.start()
        threads.append(thread)
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
    print("[RK规则接口] http://<板子IP>:%d/snapshot  |  POST /command" % args.decision_port, flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        base.STOP = True
        DECISIONS.core.source_failed("vision_stopped")
        source = ACTIVE_SOURCE
        if source is not None:
            try:
                source.close()
            except Exception:
                pass
        for thread in threads:
            thread.join(timeout=3)
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
        detector.release()
        lane_model.close()
        print("DECISION_SUMMARY " + json.dumps(DECISIONS.snapshot(), ensure_ascii=False), flush=True)
        DECISIONS.close()
        time.sleep(0.3)
        print("已停止", flush=True)
        os._exit(1 if trial_errors else 0)


if __name__ == "__main__":
    main()
