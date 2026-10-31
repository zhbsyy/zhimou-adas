"""Bounded asynchronous summaries and a separate JSON/HTTP command interface."""
from collections import deque
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import base64
import json
import threading
import time
from urllib.parse import parse_qs, urlsplit

import numpy as np
import cv2

from decision_core import CommandError, DecisionCore
from pedestrian_aeb import PedestrianMonitor, validate_config
from perception_summary import BoundaryTracker, SignalSelector, extract_boundaries


@dataclass(frozen=True)
class FramePacket:
    bgr: np.ndarray
    seq: int
    epoch: str
    received_mono: float
    captured_at: float  # legacy preview timestamp, not exposure-time evidence
    raw_bgr: object = None  # optional immutable image before camera-input gains
    camera_color: object = None
    stereo_right: object = None
    stereo_left_raw: object = None


def load_config(path):
    with open(path, encoding="utf-8-sig") as stream:
        config = json.load(stream)
    if config.get("scene", {}).get("signal_algorithm", "legacy_hsv") not in (
            "legacy_hsv", "traffic_light_reference_hybrid", "traffic_light_reference_forced_rg",
            "traffic_light_pixel_ratio_rg"):
        raise ValueError("Unsupported signal_algorithm for decision mode")
    for rect in config.get("scene", {}).get("signal_rois", {}).values():
        if (not isinstance(rect, list) or len(rect) != 4 or
                not all(type(v) in (int, float) and np.isfinite(v) and 0 <= v <= 1 for v in rect)
                or rect[0] >= rect[2] or rect[1] >= rect[3]):
            raise ValueError("Signal ROIs must be normalized x1,y1,x2,y2 rectangles")
    boundary = config.get("boundaries", {})
    for key in ("rows", "min_samples", "confirm_samples", "min_marked_pixels"):
        if key in boundary and (type(boundary[key]) is not int or not 1 <= boundary[key] <= 64):
            raise ValueError("Invalid boundary integer: "+key)
    if not 0 < boundary.get("row_start", .5) < boundary.get("row_end", .94) <= 1:
        raise ValueError("Invalid boundary row range")
    if not .2 <= boundary.get("anchor_x", .5) <= .8 or not .6 <= boundary.get("dominance", .8) <= 1:
        raise ValueError("Invalid ego anchor or class dominance")
    if boundary.get("classification","consensus") not in ("consensus","length_ratio"):
        raise ValueError("Invalid boundary classification")
    if boundary.get("anchor_mode","frame_center") not in ("frame_center","road_center"):
        raise ValueError("Invalid boundary anchor_mode")
    enter,leave=boundary.get("dashed_ratio_threshold",.4),boundary.get("dashed_exit_ratio",.3)
    if not (0 < leave <= enter < 1):
        raise ValueError("Expected 0 < dashed_exit_ratio <= dashed_ratio_threshold < 1")
    for key,default in (("min_length_ratio",.06),("min_vertical_span_ratio",.04),
                        ("min_road_area_ratio",.002),("max_boundary_width_ratio",.06)):
        value=boundary.get(key,default)
        if type(value) not in (int,float) or not np.isfinite(value) or not 0 < value < .5:
            raise ValueError("Invalid boundary ratio: "+key)
    validate_config(config.get("aeb"))
    return config


class DecisionService:
    def __init__(self, config=None, stats_provider=None, evidence_mode="live"):
        self.config = config or {}
        self.core = DecisionCore(self.config.get("rules"), aeb_config=self.config.get("aeb"))
        self.stats_provider = stats_provider or (lambda: {})
        self.evidence_mode = evidence_mode
        self.pending = {}
        self.collision_pending = None
        self.collision_worker = None
        self.ranger = None
        stereo=self.config.get("stereo",{})
        if stereo.get("enabled"):
            from stereo_roi_range import RoiStereoRanger
            self.ranger=RoiStereoRanger(stereo["calibration"],stereo.get("confirmed",False))
        self.cv = threading.Condition()
        self.stop_event = threading.Event()
        self.metrics = {"submitted_signal": 0, "submitted_boundaries": 0,
                        "replaced_signal": 0, "replaced_boundaries": 0,
                        "processed_signal": 0, "processed_boundaries": 0,
                        "signal_ms": 0, "boundaries_ms": 0, "rule_ms": 0,
                        "summary_errors": 0, "last_error": None}
        self.metrics.update(submitted_collision=0,replaced_collision=0,processed_collision=0,collision_ms=0,stereo_ms=0)
        self.metric_lock = threading.Lock()
        self.server = None
        self.worker = None
        self.signal_debug_lock = threading.Lock()
        self.signal_debug_sample = None
        self.camera_debug_packet = None

    def begin_source(self, source):
        epoch = self.core.reset_source(source, self.evidence_mode)
        with self.cv:
            self.pending.clear()
            self.collision_pending=None
        with self.signal_debug_lock:
            self.signal_debug_sample = None
            self.camera_debug_packet = None
        return epoch

    def frame(self, image, seq, epoch, received_mono=None, captured_at=None,
              raw_bgr=None, camera_color=None, stereo_right=None, stereo_left_raw=None):
        stamp = time.monotonic() if received_mono is None else received_mono
        if not self.core.note_frame(epoch, seq, stamp):
            return None
        packet = FramePacket(image, seq, epoch, stamp,
                             time.time() if captured_at is None else captured_at, raw_bgr, camera_color, stereo_right, stereo_left_raw)
        with self.signal_debug_lock:
            self.camera_debug_packet = packet
        return packet

    def _submit(self, key, item):
        if item[0].epoch != self.core.epoch:
            return
        with self.cv:
            with self.metric_lock:
                self.metrics["submitted_"+key] += 1
                if key in self.pending:
                    self.metrics["replaced_"+key] += 1
            self.pending[key] = item
            self.cv.notify_all()

    def submit_detection(self, packet, detections, names, front_rect=None):
        # Capture/renderer treats packet.bgr as immutable. Keep a reference;
        # only the small box array needs an independent copy.
        h, w = packet.bgr.shape[:2]
        self._submit("signal", (packet, np.asarray(detections).copy(), tuple(names),
                                front_rect or (0, 0, w, h), self.core.context_snapshot()))
        if self.config.get('aeb',{}).get('enabled') or self.ranger is not None:
            with self.cv:
                with self.metric_lock:
                    self.metrics['submitted_collision']+=1
                    if self.collision_pending is not None:self.metrics['replaced_collision']+=1
                self.collision_pending=(packet,np.asarray(detections).copy(),tuple(names))
                self.cv.notify_all()

    def submit_lane(self, packet, masks):
        # These uint8 masks are owned postprocess outputs, never borrowed RKNN floats.
        self._submit("boundaries", (packet, masks))

    def _record(self, key, elapsed):
        with self.metric_lock:
            self.metrics["processed_"+key] += 1
            old = self.metrics[key+"_ms"]
            self.metrics[key+"_ms"] = elapsed if old == 0 else .9*old+.1*elapsed

    def _collision_work(self):
        monitor=PedestrianMonitor(self.config.get('aeb'))
        epoch=None
        while not self.stop_event.is_set():
            with self.cv:
                self.cv.wait_for(lambda:self.collision_pending is not None or self.stop_event.is_set(),.1)
                item=self.collision_pending;self.collision_pending=None
            if item is None:continue
            packet,dets,names=item
            if packet.epoch!=self.core.epoch:continue
            if time.monotonic()-packet.received_mono>monitor.config['ttl_s']:continue
            if epoch!=packet.epoch:
                monitor.reset()
                if self.ranger:self.ranger.tracks=[]
                epoch=packet.epoch
            started=time.perf_counter()
            try:
                ranges=None
                if self.ranger is not None:
                    t=time.perf_counter()
                    ranges=self.ranger.measure(packet.stereo_left_raw,packet.stereo_right,dets,names,packet.received_mono)
                    with self.metric_lock:self.metrics['stereo_ms']=(time.perf_counter()-t)*1000
                result=monitor.update(dets,names,packet.bgr.shape,packet.received_mono,ranges)
                self.core.publish('obstacles',packet.epoch,packet.seq,packet.received_mono,result)
            except Exception as exc:
                self.core.publish('obstacles',packet.epoch,packet.seq,packet.received_mono,
                    {'valid':False,'level':'unknown','distance_m':None,'reason':'pedestrian_summary_error'})
                with self.metric_lock:
                    self.metrics['summary_errors']+=1;self.metrics['last_error']=str(exc)[:160]
            self._record('collision',(time.perf_counter()-started)*1000)

    def _work(self):
        selector = SignalSelector(self.config.get("scene"))
        boundary_cfg = dict(self.config.get("boundaries", {}))
        tracker = BoundaryTracker(boundary_cfg.pop("confirm_samples", 2),
                                  dashed_exit=boundary_cfg.get("dashed_exit_ratio",.3))
        last_epoch, last_revision, last_kind = None, None, None
        while not self.stop_event.is_set():
            job = None
            with self.cv:
                if not self.pending:
                    self.cv.wait(.05)
                keys = list(self.pending)
                if keys:
                    key = next((k for k in keys if k != last_kind), keys[0])
                    job = (key, self.pending.pop(key))
            if job:
                key, item = job
                packet = item[0]
                last_kind = key
                if packet.epoch != self.core.epoch:
                    continue
                if packet.epoch != last_epoch:
                    selector.reset()
                    tracker.reset()
                    last_epoch, last_revision = packet.epoch, None
                started = time.perf_counter()
                try:
                    if key == "signal":
                        _, dets, names, rect, context = item
                        if context["revision"] != last_revision:
                            selector.reset()
                            last_revision = context["revision"]
                        x, y, w, h = rect
                        front = packet.bgr[y:y+h, x:x+w]
                        if front.shape[:2] != (h, w):
                            raise ValueError("Front ROI outside source frame")
                        if dets.size:
                            cx, cy = (dets[:,0]+dets[:,2])/2, (dets[:,1]+dets[:,3])/2
                            dets = dets[(cx >= x) & (cx < x+w) & (cy >= y) & (cy < y+h)].copy()
                            dets[:, [0,2]] -= x
                            dets[:, [1,3]] -= y
                        result = selector.update(front, dets, names, packet.received_mono, context["lane"])
                        self.core.publish("signal", packet.epoch, packet.seq, packet.received_mono,
                                          result, context_revision=context["revision"])
                        # Keep a view of the immutable raw frame, not the rendered JPEG.
                        # No PNG encoding or file write occurs in the summary worker.
                        with self.signal_debug_lock:
                            self.signal_debug_sample = (packet, context["revision"], front, result)
                    else:
                        result = extract_boundaries(*item[1], config=boundary_cfg)
                        result = tracker.update(result, packet.seq, packet.received_mono)
                        self.core.publish("boundaries", packet.epoch, packet.seq, packet.received_mono, result)
                    self._record(key, (time.perf_counter()-started)*1000)
                except Exception as exc:
                    with self.metric_lock:
                        self.metrics["summary_errors"] += 1
                        self.metrics["last_error"] = str(exc)[:160]
                    result = ({"color": "unknown", "valid": False, "reason": "summary_error"}
                              if key == "signal" else
                              {side: {"kind": "unknown", "valid": False, "reason": "summary_error"}
                               for side in ("left", "right")})
                    self.core.publish(key, packet.epoch, packet.seq, packet.received_mono, result,
                                      context_revision=item[4]["revision"] if key == "signal" else None)
            started = time.perf_counter()
            self.core.snapshot()  # expire requests even if no HTTP client is polling
            with self.metric_lock:
                self.metrics["rule_ms"] = (time.perf_counter()-started)*1000

    def snapshot(self):
        result = self.core.snapshot()
        with self.metric_lock:
            result["metrics"] = dict(self.metrics)
        # The existing statistics are numeric scalars; do not copy frames/masks.
        result["vision"] = {k: v for k,v in self.stats_provider().items()
                            if k in ("cap_fps", "inf_fps", "lane_fps", "enc_fps", "det_count", "lane_count", "stream_count", "camera_color_ms")}
        with self.signal_debug_lock:
            packet = self.camera_debug_packet
        result["camera_input"] = packet.camera_color if packet and packet.epoch == result["source_epoch"] else None
        result["runtime_config"] = dict(self.stats_provider().get("runtime_profile", {}))
        result["active_rules"] = dict(self.core.config)
        result["active_boundaries"] = dict(self.config.get("boundaries", {}))
        result["active_aeb"] = dict(self.core.aeb_config)
        result["stereo_runtime"] = self.ranger.describe() if self.ranger else None
        return result

    def camera_debug(self, include_image=False):
        current = self.core.snapshot()
        with self.signal_debug_lock:
            packet = self.camera_debug_packet
        if (not packet or packet.epoch != current["source_epoch"] or
                not current["perception"]["source_valid"] or
                not 0 <= time.monotonic()-packet.received_mono <= self.core.config["source_ttl_s"]):
            return 503, {"error": "no_fresh_camera_frame"}
        raw = packet.raw_bgr if packet.raw_bgr is not None else packet.bgr
        if raw.ndim != 3 or raw.shape[2] != 3 or raw.dtype != np.uint8 or raw.shape[0]*raw.shape[1] > 1024*1024:
            return 413, {"error": "unsupported_debug_frame_size_or_format"}
        value = {"schema": "rk_camera_debug.v1", "source_epoch": packet.epoch,
                 "frame_seq": packet.seq, "source": current["perception"]["source"],
                 "evidence_mode": current["perception"]["evidence_mode"],
                 "shape": list(raw.shape), "format": "uint8 BGR",
                 "stage": "before_input_gains_and_all_overlays",
                 "camera_color": packet.camera_color}
        if include_image:
            ok, png = cv2.imencode(".png", raw, [cv2.IMWRITE_PNG_COMPRESSION, 3])
            if not ok:
                return 500, {"error": "camera_encode_failed"}
            value["raw_png_base64"] = base64.b64encode(png).decode("ascii")
        return 200, value

    def signal_debug(self, include_image=False, requested_epoch=None, requested_seq=None):
        """Fresh, matched-frame evidence. This never changes perception or rules."""
        current = self.core.snapshot()
        signal = current["perception"]["signal"]
        with self.signal_debug_lock:
            sample = self.signal_debug_sample
        if not sample or not signal["fresh"]:
            return 503, {"error": "no_fresh_signal_sample"}, None
        packet, revision, front, result = sample
        if (packet.epoch != current["source_epoch"] or
                packet.seq != signal["frame_seq"] or
                revision != current["context"]["revision"]):
            return 409, {"error": "signal_sample_changed_retry"}, None
        if ((requested_epoch is not None and requested_epoch != packet.epoch) or
                (requested_seq is not None and requested_seq != str(packet.seq))):
            return 409, {"error": "requested_signal_sample_expired_retry"}, None
        value = {"schema": "rk_signal_debug.v1", "source_epoch": packet.epoch,
                 "frame_seq": packet.seq, "source": current["perception"]["source"],
                 "evidence_mode": current["perception"]["evidence_mode"],
                 "unannotated_bgr": True, "signal": signal,
                 "camera_color": packet.camera_color,
                 "stage": "after_input_gains_before_overlays",
                 "note": "ROI is before boxes, labels and lane overlays; color score is not probability."}
        box = result.get("box")
        if not box:
            value.update(roi_url=None, reason="no_unique_selected_light")
            return 200, value, None
        x1, y1, x2, y2 = box
        roi = front[y1:y2, x1:x2]
        # Prevent oversized debug responses if a nonstandard source is supplied.
        if not roi.size or roi.shape[0]*roi.shape[1] > 1024*1024:
            return 413, {"error": "invalid_or_oversized_signal_roi"}, None
        value["roi_url"] = "/signal_roi.png"
        value["roi_size"] = [int(roi.shape[1]), int(roi.shape[0])]
        image = None
        if include_image:
            ok, encoded = cv2.imencode(".png", roi, [cv2.IMWRITE_PNG_COMPRESSION, 3])
            if not ok:
                return 500, {"error": "roi_encode_failed"}, None
            image = encoded.tobytes()
        return 200, value, image

    def start(self, host="0.0.0.0", port=18088, token=None):
        service = self

        class Handler(BaseHTTPRequestHandler):
            def setup(self):
                super().setup()
                self.connection.settimeout(3)

            def reply(self, code, value):
                payload = json.dumps(value, ensure_ascii=False, allow_nan=False).encode("utf-8")
                self.send_response(code)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(payload)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(payload)

            def do_GET(self):
                url = urlsplit(self.path)
                if url.path == "/camera_debug":
                    code, value = service.camera_debug(parse_qs(url.query).get("image") == ["1"])
                    self.reply(code, value)
                elif url.path in ("/signal_debug", "/signal_roi.png"):
                    query = parse_qs(url.query)
                    code, value, png = service.signal_debug(
                        include_image=(url.path == "/signal_roi.png" or
                                       query.get("image") == ["1"]),
                        requested_epoch=query.get("epoch", [None])[0],
                        requested_seq=query.get("seq", [None])[0])
                    if url.path == "/signal_roi.png" and code == 200 and png is None:
                        code = 404
                    if code == 200 and png is not None and url.path == "/signal_roi.png":
                        self.send_response(200)
                        self.send_header("Content-Type", "image/png")
                        self.send_header("Content-Length", str(len(png)))
                        self.send_header("Cache-Control", "no-store")
                        self.send_header("X-Source-Epoch", value["source_epoch"])
                        self.send_header("X-Frame-Seq", str(value["frame_seq"]))
                        self.end_headers()
                        self.wfile.write(png)
                    else:
                        if code == 200 and png is not None:
                            value["roi_png_base64"] = base64.b64encode(png).decode("ascii")
                        self.reply(code, value)
                elif url.path in ("/snapshot", "/perception", "/decision", "/health"):
                    snapshot = service.snapshot()
                    if url.path == "/health":
                        value = {"service": "rk_decision.v1", "ready": True,
                                 "evidence_mode": service.evidence_mode,
                                 "perception_valid": snapshot["perception"]["source_valid"],
                                 "simulation_only": True, "source_epoch": snapshot["source_epoch"]}
                    else:
                        value = snapshot if url.path == "/snapshot" else {
                            "schema": snapshot["schema"], "source_epoch": snapshot["source_epoch"],
                            url.path[1:]: snapshot[url.path[1:]]}
                    self.reply(200, value)
                elif url.path == "/events":
                    try:
                        after = int(parse_qs(url.query).get("after", ["0"])[0])
                        if after < 0:
                            raise ValueError()
                    except ValueError:
                        self.reply(400, {"error": "after must be a nonnegative integer"})
                        return
                    self.reply(200, service.core.events_after(after))
                elif url.path == "/":
                    payload = PAGE.encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(payload)))
                    self.end_headers()
                    self.wfile.write(payload)
                else:
                    self.reply(404, {"error": "not_found"})

            def do_POST(self):
                if self.path != "/command":
                    self.reply(404, {"error": "not_found"})
                    return
                if token and self.headers.get("X-Decision-Token") != token:
                    self.reply(403, {"error": "invalid_token"})
                    return
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                    if not 0 < length <= 4096 or self.headers.get("Transfer-Encoding"):
                        raise CommandError("Expected Content-Length between 1 and 4096")
                    if self.headers.get_content_type() != "application/json":
                        raise CommandError("Use application/json")
                    body = json.loads(self.rfile.read(length).decode("utf-8"),
                                      parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))
                    self.reply(200, service.core.command(body))
                except CommandError as exc:
                    self.reply(exc.status, {"error": str(exc)})
                except (ValueError, UnicodeError, TimeoutError):
                    self.reply(400, {"error": "invalid_json_or_body"})

            def log_message(self, *args):
                pass

        self.server = ThreadingHTTPServer((host, port), Handler)
        self.server.daemon_threads = True
        self.worker = threading.Thread(target=self._work, name="decision-summary", daemon=True)
        self.worker.start()
        self.collision_worker=threading.Thread(target=self._collision_work,name="pedestrian-summary",daemon=True)
        self.collision_worker.start()
        threading.Thread(target=self.server.serve_forever, name="decision-http", daemon=True).start()
        return self.server.server_address

    def close(self):
        self.core.source_failed("decision_service_stopped")
        self.stop_event.set()
        with self.cv:
            self.cv.notify_all()
        if self.server:
            self.server.shutdown()
            self.server.server_close()
        if self.worker:
            self.worker.join(timeout=2)
        if self.collision_worker:
            self.collision_worker.join(timeout=2)


PAGE = """<!doctype html><html lang=zh-CN><meta charset=utf-8>
<title>RK决策接口</title><style>body{background:#101827;color:#e6edf5;font:16px sans-serif;margin:24px}button,select{padding:12px;margin:5px;font-size:16px}pre{white-space:pre-wrap}#result{font-size:25px;color:#82efcf}</style>
<h1>RK感知与决策</h1><p>按钮向RK提交请求，由RK返回判断；此页尚未连接CARLA。灯色和线型来自RK结果，不能在此手工设置。</p>
<select id=scenario><option value=intersection>路口</option><option value=lane_change>变道</option></select>
<select id=lane><option value=straight>直行车道</option><option value=left>左转车道</option><option value=right>右转车道</option></select>
<button onclick="send('set_context')">应用场景/车道</button><br>
<button onclick="send('move')">行驶请求</button><button onclick="send('left')">左变道请求</button><button onclick="send('right')">右变道请求</button><button onclick="send('stop')">停止</button><span>报警自动触发；行人制动优先</span>
<div id=result>等待RK结果</div><pre id=info></pre><script>
const client='web_'+Math.random().toString(36).slice(2);let seq=0,current=null,busy=false;
async function refresh(){try{const r=await fetch('/snapshot',{cache:'no-store'});current=await r.json();document.querySelector('#result').textContent=current.decision.message;document.querySelector('#info').textContent=JSON.stringify(current,null,2);}catch(e){current=null;document.querySelector('#result').textContent='RK接口断开，停止反馈';}}
async function send(action){if(busy||!current)return;busy=true;const n=++seq;const body={client_id:client,request_id:'r'+n,seq:n,source_epoch:current.source_epoch,action};if(action==='set_context'){body.scenario=document.querySelector('#scenario').value;body.lane=document.querySelector('#lane').value;}try{const r=await fetch('/command',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});const j=await r.json();if(!r.ok)alert(j.error);await refresh();}finally{busy=false;}}
refresh();setInterval(refresh,300);</script></html>"""
