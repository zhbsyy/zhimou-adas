#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pcie_stream.py (v2-HW) ——  PCIe 抓帧 + YOLOv8 + 网页直播（**硬件 JPEG 编码版**）
==================================================================================

本文件基于你「最快最稳定」的 board_files/pcie_stream.py 改动，**默认行为只改一处**：
    把 cv2.imencode（软件 JPEG，实测 ~42.7 ms/帧、独占一个核）
    换成 mppjpegenc（RK3568 VPU 硬编，预计几 ms、几乎不占 CPU）

除了编码器，其余（抓帧线程 / 推理线程 / 队列策略 / 网页 / 退出方式）与你原来的完全一致。
配套文件：hw_jpeg.py（必须放在同一个目录里）

用法（和以前一样，只是多了两个开关）：
    sudo ~/yolov8_deploy/.venv-offline/bin/python pcie_stream.py --infer_every 2
    加 --no_hw_jpeg     → 退回原来的 cv2 软件编码（硬编出问题时的退路）
    加 --q_depth 1      → 队列深度 2→1，立刻再省约一帧延迟（默认 2，保持原行为）

启动时会打印一行 `[编码] 硬件 JPEG(...)` 或 `[编码] cv2 软件 JPEG`，一眼就能确认走的哪条路。
"""
import argparse
import os
import queue
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
YOLO_DIR = os.environ.get("YOLO_DIR", "/home/linaro/yolov8_deploy")
BOARD_DEP_DIR = os.environ.get("BOARD_DEP_DIR", "/home/linaro/03_RK3568_Perception")
# Keep candidate modules first; import unchanged capture/postprocess/OIB modules
# from the stable directory without copying or editing those files.
for root in (YOLO_DIR, BOARD_DEP_DIR, HERE):
    sys.path.insert(0, root)

from pcie_frame import PcieFrameSource                              # noqa: E402
from pcie_post import (preprocess_fast, decode_auto, to_original, draw,
                       guess_size_from_model)   # noqa: E402
from r15_front_fast import LiveLaneModel, draw_lane_mask        # noqa: E402

# ---------------------------------------------------------------- [HW] 硬编
try:
    from hw_jpeg import JpegEncoder
    HAVE_HW_JPEG = True
except Exception as _e:                     # 缺文件/缺依赖都不能让推流挂掉
    JpegEncoder = None
    HAVE_HW_JPEG = False
    _HW_IMPORT_ERR = _e
else:
    _HW_IMPORT_ERR = None

HW_ENC = None           # 全局编码器，main() 里创建

# ------------------------------------------------------ 双目测距（可选）
try:
    from stereo_range import StereoRanger
    HAVE_RANGE = True
except Exception as _re:
    StereoRanger = None
    HAVE_RANGE = False
    _RANGE_IMPORT_ERR = _re
else:
    _RANGE_IMPORT_ERR = None

RANGER = None           # 全局测距器，main() 里创建

# ------------------------------------------------------ 红绿灯颜色（可选）
try:
    sys.path.insert(0, YOLO_DIR)
    from traffic_light_video_demo.traffic_light_color import (
        TrafficLightColorEngine as _TLEngine,
        draw_results as _tl_draw,
        load_config as _tl_loadcfg)
    HAVE_TL = True
except Exception:
    _TLEngine = _tl_draw = _tl_loadcfg = None
    HAVE_TL = False

TL_ENGINE = None        # 全局颜色引擎，main() 里创建

# ------------------------------------------------------ CARLA 视频源（可选）
try:
    from carla_source import CarlaFrameSource
    HAVE_CARLA = True
except Exception as _ce:
    CarlaFrameSource = None
    HAVE_CARLA = False
    _CARLA_IMPORT_ERR = _ce
else:
    _CARLA_IMPORT_ERR = None

# ------------------------------------------------------ OIB 中枢通信与安全决策（可选）
try:
    from oib_link import OibLink
    HAVE_OIB = True
except Exception as _oe:
    OibLink = None
    HAVE_OIB = False
    _OIB_IMPORT_ERR = _oe
else:
    _OIB_IMPORT_ERR = None

OIB_LINK = None

LATEST = {"jpeg": None, "seq": 0}
STOP = False
STATS = {"cap_fps": 0.0, "cap_pcie_fps": 0.0, "cap_carla_fps": 0.0, "inf_fps": 0.0, "inf_ms": 0.0,
         "enc_fps": 0.0, "det": 0, "drop": 0, "enc_ms": 0.0, "hw": "?", "source": "pcie",
         "lane_ms": 0.0, "lane_fps": 0.0, "lane_pixels": 0, "lane_status": "off",
         "direct_pixels": 0, "solid_pixels": 0, "dashed_pixels": 0,
         "r15_pre_ms": 0.0, "r15_post_ms": 0.0,
         "det_pre_ms": 0.0, "det_post_ms": 0.0, "lane_draw_ms": 0.0,
         "vis_draw_ms": 0.0, "oib_ms": 0.0, "loop_ms": 0.0,
         "process_fps": 0.0, "count_start": 0.0,
         "cap_count": 0, "det_count": 0, "lane_count": 0,
         "processed_count": 0, "stream_count": 0}

PAGE = (b"<!doctype html><html><head><meta charset='utf-8'><title>FPGA PCIe YOLOv8</title>"
        b"<style>body{margin:0;background:#111;color:#eee;font-family:sans-serif}"
        b"img{width:100%;display:block}"
        b"#clk{font:700 30px monospace;color:#6f6;padding:4px 10px;letter-spacing:1px}"
        b"#tip{font:13px sans-serif;color:#aaa;padding:0 10px 6px}</style></head><body>"
        b"<div style='padding:6px 10px'>FPGA \xe5\x8f\x8c\xe7\x9b\xae\xe7\x9b\xb8\xe6\x9c\xba + YOLOv8 "
        b"&nbsp;|&nbsp; <a style='color:#6cf' href='/stream'>/stream</a>"
        b" &nbsp;|&nbsp; <a style='color:#6cf' href='/stat'>/stat</a></div>"
        #  端到端延迟测量：绿字是本机当前时间；画面顶部横幅里烧的是"采集时刻"。
        #   截一张图，用两者相减 = 采集→显示 的真实延迟（含浏览器缓冲）。
        b"<div id='clk'>--:--:--.---</div>"
        b"<div id='tip'>\xe5\xaf\xb9\xe6\xaf\x94\xef\xbc\x9a\xe7\xbb\xbf\xe5\xad\x97\xe6\x97\xb6\xe9\x97\xb4"
        b" \xe4\xb8\x8e\xe7\x94\xbb\xe9\x9d\xa2\xe9\xa1\xb6\xe9\x83\xa8\xe6\xa8\xaa\xe5\xb9\x85\xe9\x87\x8c"
        b"\xe7\x9a\x84\xe6\x97\xb6\xe9\x97\xb4\xe7\x9b\xb8\xe5\x87\x8f = \xe7\xab\xaf\xe5\x88\xb0\xe7\xab\xaf"
        b"\xe5\xbb\xb6\xe8\xbf\x9f</div>"
        b"<img src='/stream'><script>"
        b"var c=document.getElementById('clk');"
        b"function p(n,w){n=String(n);while(n.length<w)n='0'+n;return n}"
        # 用 NTP 式校时把"板子的时钟"搬到网页上显示（误差≈往返的一半，局域网里几毫秒）。
        # 于是：这个绿字 与 画面横幅里烧的采集时刻 属于同一时钟源，截图相减就是端到端延迟。
        b"var sL=0,sT=0,ok=false;"
        b"function sync(){var t0=Date.now();"
        b"fetch('/lat',{cache:'no-store'}).then(function(r){return r.json()}).then(function(j){"
        b"var t1=Date.now(),rtt=t1-t0;sL=j.l+rtt/2;sT=t1;ok=true;}).catch(function(){});}"
        b"function tick(){if(!ok)return;"
        b"var v=(sL+(Date.now()-sT))%86400000;if(v<0)v+=86400000;"
        b"var h=Math.floor(v/3600000),m=Math.floor(v/60000)%60,"
        b"s=Math.floor(v/1000)%60,ms=Math.floor(v)%1000;"
        b"c.textContent=p(h,2)+':'+p(m,2)+':'+p(s,2)+'.'+p(ms,3);}"
        b"sync();setInterval(sync,3000);setInterval(tick,20);"
        b"</script></body></html>")


def rknn_input(bgr, size=640):
    """BGR -> uint8 NHWC（一步到位，比 float 往返路线快约 15 倍）"""
    return preprocess_fast(bgr, size=size)


def put_latest(q, item):
    """队列满时丢掉最旧的（保证总是最新的帧）"""
    try:
        q.put_nowait(item)
    except queue.Full:
        try:
            q.get_nowait()
            STATS["drop"] += 1
        except queue.Empty:
            pass
        try:
            q.put_nowait(item)
        except queue.Full:
            pass


# ---------------------------------------------------------------- 线程1：抓帧
def capture_thread(args, q_frame):
    pcie_src = None
    carla_src = None
    last_carla_sequence = 0

    if args.source in ("pcie", "dual"):
        pcie_src = PcieFrameSource(os.path.join(HERE, "cap_reader.py"))
        try:
            pcie_src.start()
        except Exception as e:
            sys.stderr.write("[capture] PCIe 采集启动异常: %s\n" % e)
            pcie_src = None

    if args.source in ("carla", "dual"):
        if CarlaFrameSource is not None:
            carla_src = CarlaFrameSource(stream_url=args.carla_url)
            carla_src.start()
        else:
            sys.stderr.write("[capture] 未找到 CarlaFrameSource 模块\n")

    n, t0, fps = 0, time.time(), 0.0
    while not STOP:
        # dual 模式下根据 OIB_LINK 的 perception_source 动态决定活跃源
        active_source = args.source
        if args.source == "dual" and OIB_LINK is not None:
            active_source = getattr(OIB_LINK, "perception_source", "pcie")

        STATS["source"] = active_source
        bgr = None
        t_cap = time.time()
        tag = active_source

        if active_source == "carla" and carla_src is not None:
            bgr, last_carla_sequence, frame_time = carla_src.read_new_bgr(
                last_carla_sequence, timeout=0.2)
            if bgr is not None:
                t_cap = frame_time
            tag = "carla"
            STATS["cap_carla_fps"] = carla_src.fps
        elif pcie_src is not None:
            bgr = pcie_src.read_bgr()
            tag = "pcie"
            if carla_src is not None:
                STATS["cap_carla_fps"] = carla_src.fps
        elif carla_src is not None:
            bgr, last_carla_sequence, frame_time = carla_src.read_new_bgr(
                last_carla_sequence, timeout=0.2)
            if bgr is not None:
                t_cap = frame_time
            tag = "carla"

        if bgr is not None:
            if not STATS["count_start"]:
                STATS["count_start"] = time.monotonic()
            STATS["cap_count"] += 1
            put_latest(q_frame, (bgr, t_cap, tag))
            n += 1
            t1 = time.time()
            fps = 0.9 * fps + 0.1 / max(t1 - t0, 1e-6)
            t0 = t1
            STATS["cap_fps"] = fps
            if tag == "pcie":
                STATS["cap_pcie_fps"] = fps
        else:
            time.sleep(0.01)

    if pcie_src is not None:
        try: pcie_src.close()
        except Exception: pass
    if carla_src is not None:
        try: carla_src.close()
        except Exception: pass


# ------------------------------------------------- 线程2：推理 + 画框（最重）
def infer_thread(args, names, rknn, lane_model, q_frame, q_vis):
    import cv2
    n, t0, fps = 0, time.time(), 0.0
    process_t0, process_fps = time.monotonic(), 0.0
    last_dets = np.zeros((0, 6), dtype=np.float32)
    lane_mask = None
    lane_shape = None
    lane_last_time = None
    lane_active = lane_model is not None
    while not STOP:
        try:
            item = q_frame.get(timeout=0.5)
        except queue.Empty:
            continue
        if len(item) == 3:
            bgr, t_cap, src_tag = item
        else:
            bgr, t_cap = item
            src_tag = "pcie"
        loop_started = time.perf_counter()

        if n % max(args.infer_every, 1) == 0:
            t_pre0 = time.perf_counter()
            inp, tr = rknn_input(bgr, args.size)
            pre_ms = (time.perf_counter() - t_pre0) * 1e3
            STATS["det_pre_ms"] = 0.9 * STATS["det_pre_ms"] + 0.1 * pre_ms
            t_i0 = time.perf_counter()
            outs = rknn.inference(inputs=[inp], data_format="nhwc")
            dt_i = (time.perf_counter() - t_i0) * 1e3
            STATS["inf_ms"] = 0.9 * STATS.get("inf_ms", dt_i) + 0.1 * dt_i
            t_post0 = time.perf_counter()
            dets = decode_auto(outs, size=args.size, nc=len(names),
                               confidence=args.conf)
            if dets.size:
                dets = to_original(dets, tr)
            STATS["det_count"] += 1
            post_ms = (time.perf_counter() - t_post0) * 1e3
            STATS["det_post_ms"] = 0.9 * STATS["det_post_ms"] + 0.1 * post_ms
            last_dets = dets
            t1 = time.time()
            fps = 0.9 * fps + 0.1 / max(t1 - t0, 1e-6)
            t0 = t1
        else:
            dets = last_dets
        n += 1
        STATS["inf_fps"] = fps
        STATS["det"] = len(dets)
        STATS["inf_age_ms"] = (time.time() - t_cap) * 1e3

        # CARLA's 2x2 mosaic: only the top-left tile is the forward road view.
        # The two RKNN contexts run sequentially in this thread, as in dual_video.py.
        vis_source = bgr
        if lane_active and src_tag == "carla":
            tile_h, tile_w = bgr.shape[0] // 2, bgr.shape[1] // 2
            front = bgr[:tile_h, :tile_w]
            if lane_shape != front.shape[:2]:
                lane_mask = None
                lane_shape = front.shape[:2]
            if lane_mask is None or (n - 1) % args.lane_every == 0:
                try:
                    lane_mask, lane_timing = lane_model.infer(front)
                    STATS["lane_count"] += 1
                    STATS["lane_ms"] = lane_timing["inference_ms"]
                    STATS["r15_pre_ms"] = lane_timing["pre_ms"]
                    STATS["r15_post_ms"] = lane_timing["post_ms"]
                    STATS["direct_pixels"] = lane_timing["direct_pixels"]
                    STATS["solid_pixels"] = lane_timing["solid_pixels"]
                    STATS["dashed_pixels"] = lane_timing["dashed_pixels"]
                    now = time.monotonic()
                    if lane_last_time is not None:
                        dt = now - lane_last_time
                        STATS["lane_fps"] = 1.0 / dt if dt > 0 else 0.0
                    lane_last_time = now
                    STATS["lane_status"] = "on"
                except Exception as err:
                    lane_active = False
                    lane_mask = None
                    STATS["lane_status"] = "error: %s" % str(err)[:80]
                    print("[车道线] 推理停止，YOLO 继续运行: %s" % err, flush=True)
            if lane_mask is not None:
                try:
                    front_dets = dets[(dets[:, 0] < tile_w) & (dets[:, 1] < tile_h)].copy()
                    if len(front_dets):
                        front_dets[:, [0, 2]] = np.clip(front_dets[:, [0, 2]], 0, tile_w)
                        front_dets[:, [1, 3]] = np.clip(front_dets[:, [1, 3]], 0, tile_h)
                    vis_source = bgr.copy()
                    t_lane_draw = time.perf_counter()
                    marked, counts = draw_lane_mask(front, lane_mask, front_dets)
                    STATS["lane_draw_ms"] = 0.9 * STATS["lane_draw_ms"] + 0.1 * (
                        (time.perf_counter() - t_lane_draw) * 1e3)
                    vis_source[:tile_h, :tile_w] = marked
                    STATS["lane_pixels"] = counts["solid_line"] + counts["dashed_line"]
                except Exception as err:
                    lane_active = False
                    lane_mask = None
                    vis_source = bgr
                    STATS["lane_status"] = "error: %s" % str(err)[:80]
                    print("[车道线] 绘制停止，YOLO 继续运行: %s" % err, flush=True)
        else:
            lane_mask = None
            STATS["lane_pixels"] = 0
            if lane_model is not None and lane_active:
                STATS["lane_status"] = "idle"

        lt = time.localtime(t_cap)
        title = "%s | [%s] #%d | cap %.1f | detHz %.1f | det %d | %02d:%02d:%02d.%03d" % (
            os.path.basename(args.model), src_tag.upper(), n, STATS["cap_fps"], fps, len(dets),
            lt.tm_hour, lt.tm_min, lt.tm_sec, int((t_cap % 1) * 1000))
        if lane_model is not None:
            title += " | lane %s %.0fms" % (STATS["lane_status"], STATS["lane_ms"])

        # ---- 双目测距（可选）：仅在 pcie 源下启用！carla 源下必须禁用以防假双目误触发 AEB ----
        pairs = []
        if src_tag == "pcie" and RANGER is not None and len(dets):
            try:
                pairs = RANGER.solve_pairs(bgr, dets)
                STATS["range_n"] = len(pairs)
                zs = [r["Z"] for _, r in pairs if not r.get("suspect")]
                STATS["range_z"] = float(np.median(zs)) if zs else 0.0
                STATS["range_mv"] = sum(1 for _, r in pairs if r.get("moving"))
            except Exception as e:
                STATS["range_err"] = str(e)[:60]
                STATS["range_z"] = 0.0
        else:
            STATS["range_n"] = 0
            STATS["range_z"] = 0.0
            STATS["range_mv"] = 0

        # ---- 红绿灯颜色（可选）----
        tl_str = None
        t_vis_draw = time.perf_counter()
        if TL_ENGINE is not None:
            try:
                res = TL_ENGINE.update(bgr, dets, names, time.monotonic())
                vis = _tl_draw(vis_source, dets, names, res, title)
                STATS["tl_n"] = len(res)
                if res and isinstance(res, list):
                    tl_str = res[0].get("color", "")
            except Exception:
                vis = draw(vis_source, dets, names, title)
        else:
            vis = draw(vis_source, dets, names, title)
        STATS["vis_draw_ms"] = 0.9 * STATS["vis_draw_ms"] + 0.1 * (
            (time.perf_counter() - t_vis_draw) * 1e3)

        if pairs and src_tag == "pcie":
            RANGER.annotate(vis, dets, pairs)

        # ---- OIB 通信与自主安全决策上报 (UDP 8888) ----
        t_oib = time.perf_counter()
        if OIB_LINK is not None:
            try:
                formatted_pairs = []
                for det, r in pairs:
                    if r.get("suspect"):
                        continue
                    formatted_pairs.append({
                        "Z": r.get("Z"),
                        "X": r.get("X", 0.0),
                        "class": r.get("name", "obstacle"),
                    })
                OIB_LINK.evaluate_and_report(dets=dets, stereo_pairs=formatted_pairs, tl_state=tl_str)
            except Exception as e:
                STATS["oib_err"] = str(e)[:40]
        STATS["oib_ms"] = 0.9 * STATS["oib_ms"] + 0.1 * (
            (time.perf_counter() - t_oib) * 1e3)

        put_latest(q_vis, (vis, t_cap))
        STATS["processed_count"] += 1
        STATS["loop_ms"] = 0.9 * STATS["loop_ms"] + 0.1 * (
            (time.perf_counter() - loop_started) * 1e3)
        now = time.monotonic()
        process_fps = 0.9 * process_fps + 0.1 / max(now - process_t0, 1e-6)
        process_t0 = now
        STATS["process_fps"] = process_fps
        continue


# ------------------------------------------------------------- 线程3：编码推流
def encode_thread(args, q_vis):
    """[HW] 只改了这一段：cv2.imencode → HW_ENC.encode_bgr（失败自动退回 cv2）"""
    import cv2
    n, t0, fps = 0, time.time(), 0.0
    ema_ms = 0.0
    while not STOP:
        try:
            vis, t_cap = q_vis.get(timeout=0.5)        #  带上帧的时间戳
        except queue.Empty:
            continue
        STATS["q_wait_ms"] = (time.time() - t_cap) * 1e3    # 从采集到手的等待（含排队+推理）
        if args.encode_scale != 1.0:
            h, w = vis.shape[:2]
            vis = cv2.resize(vis, (int(w * args.encode_scale), int(h * args.encode_scale)))

        t_enc = time.time()
        jpg = None
        if HW_ENC is not None:
            jpg = HW_ENC.encode_bgr(vis)            # [HW] 投递 + 取最新一张
            # 注意：返回 None 只代表"这一帧还没编出来"，**不要**因此去软编
            # （那就白干两遍）。只有硬编被判死（.broken）才回退 cv2。
        if jpg is None and (HW_ENC is None or HW_ENC.broken):
            ok, buf = cv2.imencode(".jpg", vis, [cv2.IMWRITE_JPEG_QUALITY, args.quality])
            jpg = buf.tobytes() if ok else None
        if jpg:
            LATEST["jpeg"] = jpg
            LATEST["seq"] += 1
            STATS["stream_count"] += 1
            #  帧龄：从"这一帧数据在主机侧到手"到"JPEG 发布出去"的总耗时
            #   这就是【管道内延迟】。用户看到的延迟 = 它 + 浏览器 MJPEG 缓冲。
            age = (time.time() - t_cap) * 1e3
            STATS["age_ms"] = 0.9 * STATS.get("age_ms", age) + 0.1 * age
            STATS["age_min_ms"] = min(STATS.get("age_min_ms", age), age)

        dt = (time.time() - t_enc) * 1e3
        ema_ms = 0.9 * ema_ms + 0.1 * dt
        STATS["enc_ms"] = ema_ms
        if HW_ENC is not None:
            st = HW_ENC.stats
            STATS["hw"] = "on" if st["hw"] else ("broken" if HW_ENC.broken else "waiting")
            STATS["enc_hw_frames"] = st["produced"]
            STATS["enc_sw_frames"] = st["sw"]
            STATS["enc_submitted"] = st["submitted"]

        n += 1
        t1 = time.time()
        fps = 0.9 * fps + 0.1 / max(t1 - t0, 1e-6)
        t0 = t1
        STATS["enc_fps"] = fps
        if args.fps_print and n % args.fps_print == 0:
            print("[流水线] 推流 %5.1f fps | 抓帧 %5.1f | 检测前向 %5.1f ms(检测更新 %.1f fps) "
                  "| 编码 %5.1f ms(%s 产出%d/投%d) | 帧龄 %.0f ms%s | 目标 %d | 丢帧 %d"
                  % (fps, STATS["cap_fps"],
                     STATS.get("inf_ms", 0.0), STATS["inf_fps"], ema_ms, STATS["hw"],
                     STATS.get("enc_hw_frames", 0), STATS.get("enc_submitted", 0),
                     STATS.get("age_ms", 0.0),
                     (" | 测距 %d 个 中位 %.2fm(运动%d)" % (STATS.get("range_n", 0), STATS.get("range_z", 0.0) / 1000.0, STATS.get("range_mv", 0))) if RANGER else "",
                     STATS["det"], STATS["drop"]))
            print("[细分] 检测预处理 %.1f ms | 检测解码 %.1f ms | R15 前向 %.1f ms "
                  "| R15 后处理 %.1f ms | 掩膜叠加 %.1f ms | 画框 %.1f ms "
                  "| OIB %.1f ms | 单帧处理 %.1f ms (%.1f fps) | R15 更新 %.1f fps"
                  % (STATS["det_pre_ms"], STATS["det_post_ms"],
                     STATS["lane_ms"], STATS["r15_post_ms"],
                     STATS["lane_draw_ms"], STATS["vis_draw_ms"],
                     STATS["oib_ms"], STATS["loop_ms"],
                     STATS["process_fps"], STATS["lane_fps"]))
            elapsed = max(time.monotonic() - STATS["count_start"], 1e-6)
            print("[实测累计 %.1fs] 采集 %.1f fps (%d) | 检测更新 %.1f fps (%d) "
                  "| R15 更新 %.1f fps (%d) | 处理 %.1f fps (%d) "
                  "| JPEG 发布 %.1f fps (%d)"
                  % (elapsed, STATS["cap_count"] / elapsed, STATS["cap_count"],
                     STATS["det_count"] / elapsed, STATS["det_count"],
                     STATS["lane_count"] / elapsed, STATS["lane_count"],
                     STATS["processed_count"] / elapsed, STATS["processed_count"],
                     STATS["stream_count"] / elapsed, STATS["stream_count"]))


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path.startswith("/stream"):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            last = -1
            try:
                while not STOP:
                    if LATEST["seq"] != last and LATEST["jpeg"]:
                        last = LATEST["seq"]
                        jpg = LATEST["jpeg"]
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                         b"Content-Length: %d\r\n\r\n" % len(jpg))
                        self.wfile.write(jpg)
                        self.wfile.write(b"\r\n")
                    else:
                        time.sleep(0.002)
            except (BrokenPipeError, ConnectionResetError):
                pass
        elif self.path.startswith("/lat"):
            #  端到端延迟测量用：把【板子的时间】告诉网页，网页用它自己的时钟推进显示。
            #   这样"网页上的时钟"和"烧在画面里的采集时刻"是同一个时钟源，可以直接相减。
            #   e = 板子的 epoch 毫秒；l = 板子本地时间"从零点起的毫秒数"
            now = time.time()
            lt = time.localtime(now)
            local_ms = (((lt.tm_hour * 60 + lt.tm_min) * 60 + lt.tm_sec) * 1000
                        + int((now % 1) * 1000))
            body = ('{"e":%d,"l":%d,"n":%d}'
                    % (int(now * 1000), local_ms, LATEST["seq"])).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path.startswith("/stat"):
            body = ("capture_fps=%.1f\nnpu_fps=%.1f\nencode_fps=%.1f\n"
                    "encode_ms=%.1f\nencode_hw=%s\nencode_hw_produced=%d\n"
                    "encode_submitted=%d\nencode_sw_frames=%d\n"
                    "npu_infer_ms=%.1f\n"
                    "frame_age_ms=%.1f\nqueue_wait_ms=%.1f\n"
                    "infer_age_ms=%.1f\nage_min_ms=%.1f\n"
                    "lane_status=%s\nlane_infer_ms=%.1f\nlane_fps=%.1f\nlane_pixels=%d\n"
                    "r15_pre_ms=%.1f\nr15_post_ms=%.1f\ndirect_pixels=%d\n"
                    "solid_pixels=%d\ndashed_pixels=%d\n"
                    "range_pairs=%d\nrange_median_mm=%.0f\ntl_tracks=%d\ndet=%d\ndrop=%d\nseq=%d\n"
                    % (STATS["cap_fps"], STATS["inf_fps"], STATS["enc_fps"],
                       STATS["enc_ms"], STATS["hw"],
                       STATS.get("enc_hw_frames", 0), STATS.get("enc_submitted", 0), STATS.get("enc_sw_frames", 0),
                       STATS.get("inf_ms", 0.0),
                       STATS.get("age_ms", 0.0), STATS.get("q_wait_ms", 0.0),
                       STATS.get("inf_age_ms", 0.0), STATS.get("age_min_ms", 0.0),
                       STATS["lane_status"], STATS["lane_ms"], STATS["lane_fps"], STATS["lane_pixels"],
                       STATS["r15_pre_ms"], STATS["r15_post_ms"], STATS["direct_pixels"],
                       STATS["solid_pixels"], STATS["dashed_pixels"],
                       STATS.get("range_n", 0), STATS.get("range_z", 0.0),
                       STATS.get("tl_n", 0),
                       STATS["det"], STATS["drop"], LATEST["seq"])).encode()
            body += ("detector_update_fps=%.1f\nprocess_fps=%.1f\n"
                     "detector_pre_ms=%.1f\ndetector_decode_ms=%.1f\n"
                     "lane_draw_ms=%.1f\nvisual_draw_ms=%.1f\n"
                     "oib_ms=%.1f\nprocess_ms=%.1f\n"
                     % (STATS["inf_fps"], STATS["process_fps"],
                        STATS["det_pre_ms"], STATS["det_post_ms"],
                        STATS["lane_draw_ms"], STATS["vis_draw_ms"],
                        STATS["oib_ms"], STATS["loop_ms"])).encode()
            elapsed = (max(time.monotonic() - STATS["count_start"], 1e-6)
                       if STATS["count_start"] else 1e-6)
            body += ("measured_seconds=%.1f\n"
                     "capture_count=%d\ndetector_count=%d\nlane_count=%d\n"
                     "processed_count=%d\nstream_count=%d\n"
                     "detector_measured_fps=%.2f\nlane_measured_fps=%.2f\n"
                     % (elapsed, STATS["cap_count"], STATS["det_count"],
                        STATS["lane_count"], STATS["processed_count"],
                        STATS["stream_count"], STATS["det_count"] / elapsed,
                        STATS["lane_count"] / elapsed)).encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/plain")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        else:
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(PAGE)))
            self.end_headers()
            self.wfile.write(PAGE)

    def log_message(self, *a):
        pass


def main():
    global HW_ENC, STOP
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default=os.path.join(
        YOLO_DIR, "models", "bdd100k_yolov8n_rk3568_i8.rknn"))
    ap.add_argument("--classes", default=os.path.join(YOLO_DIR, "models", "classes.txt"))
    ap.add_argument("--lane_model", default="",
                    help="R15 direct2/lane3 640x640 RKNN；仅分析 CARLA 四宫格左上角 Front")
    ap.add_argument("--lane_height", type=int, choices=(384, 640), default=640,
                    help="R15 模型固定输入高度；宽度固定 640")
    ap.add_argument("--lane_input_mode", choices=("float_nchw", "uint8_nhwc"),
                    default="float_nchw",
                    help="旧 RKNN 用 float_nchw；新 mean/std=255 转换版用 uint8_nhwc")
    ap.add_argument("--lane_config", default=os.path.join(HERE, "model_config.json"),
                    help="兼容旧命令的参数，R15 不使用")
    ap.add_argument("--lane_every", type=int, default=3,
                    help="每 N 帧运行一次车道线模型，其他帧复用最近掩膜（默认 3）")
    ap.add_argument("--lane_conf", type=float, default=None,
                    help="覆盖标线配置中的像素置信度")
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--size", type=int, default=0, help="0=按模型文件名自动识别")
    ap.add_argument("--delay", type=float, default=1e-5,
                    help="抓帧每行延时(秒)，精确忙等。默认 30µs；"
                         "太小(如 0)会让画面撕裂；太大则帧率下降")
    ap.add_argument("--infer_every", type=int, default=1)
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--quality", type=int, default=85)
    ap.add_argument("--encode_scale", type=float, default=1.0,
                    help="推流缩放，如 0.75（960x540，编码更快）")
    ap.add_argument("--fps_print", type=int, default=30, help="每 N 帧打印一次状态（0=关）")
    # ---------------- [HW] 新增两个开关 ----------------
    ap.add_argument("--hw_jpeg", dest="hw_jpeg", action="store_true", default=True,
                    help="用 RK3568 VPU 硬件 JPEG 编码（默认开）")
    ap.add_argument("--no_hw_jpeg", dest="hw_jpeg", action="store_false",
                    help="退回 cv2 软件 JPEG")
    ap.add_argument("--q_depth", type=int, default=2,
                    help="队列深度；改成 1 可省约一帧延迟（默认 2，与原版一致）")
    # ---------------- 双目测距 / 红绿灯颜色（可选） ----------------
    ap.add_argument("--range", dest="do_range", action="store_true",
                    help="开启双目测距标注（需要标定文件 + stereo_range.py）")
    ap.add_argument("--calib", default=os.path.join(HERE, "stereo_8params.json"),
                    help="标定文件路径（stereo_calibrate.py 生成）")
    ap.add_argument("--tl", dest="do_tl", action="store_true",
                    help="开启红绿灯颜色识别（需要 traffic_light_video_demo 在 YOLO_DIR 下）")
    ap.add_argument("--tl_config", default="",
                    help="红绿灯 HSV 配置文件（留空用模块自带默认）")
    ap.add_argument("--source", default="dual", choices=["pcie", "carla", "dual"],
                    help="输入源选择: dual(默认双源并行采集，跟随 PC 指令动态切换) / pcie(仅物理双目) / carla(仅仿真四向)")
    ap.add_argument("--carla_url", default="http://192.168.10.1:8081/stream",
                    help="PC 端 CARLA HTTP MJPEG 视频流地址")
    ap.add_argument("--pc_ip", default="192.168.10.1",
                    help="PC 端 IP 地址 (用于 UDP 8888 决策上行与 8892 监听)")
    ap.add_argument("--oib_port", type=int, default=8888,
                    help="PC HIL 系统 UDP 决策监听端口 (默认 8888)")
    ap.add_argument("--no_oib", action="store_true",
                    help="关闭 OIB 通信与决策上报")
    args = ap.parse_args()
    if args.lane_every < 1:
        ap.error("--lane_every 必须 >= 1")

    if args.size <= 0:
        args.size = guess_size_from_model(args.model)
    os.environ["CAP_DELAY"] = str(args.delay)
    names = [l.strip() for l in open(args.classes) if l.strip()]

    # ---- 初始化测距器 / 颜色引擎 ----
    global RANGER, TL_ENGINE
    if args.do_range:
        if not HAVE_RANGE:
            print("[测距] 不可用：%s（需要 stereo_range.py 与 board_files 同目录）" % _RANGE_IMPORT_ERR)
        elif not os.path.isfile(args.calib):
            print("[测距] 找不到标定文件：%s" % args.calib)
        else:
            try:
                RANGER = StereoRanger(args.calib)
                STATS["range_n"] = 0
                STATS["range_z"] = 0.0
                print("[测距] 已启用  f=%.2fpx  B=%.2fmm  重投影=%.3fpx  |dy|max=%.2fpx"
                      % (RANGER.f, RANGER.B, RANGER.rms, RANGER.dy_cal))
            except Exception as e:
                print("[测距] 初始化失败：%s" % e)
    if args.do_tl:
        if not HAVE_TL:
            print("[红绿灯] 不可用：找不到 traffic_light_video_demo/traffic_light_color.py")
        else:
            try:
                cc, sc = _tl_loadcfg(args.tl_config) if args.tl_config else (None, None)
                TL_ENGINE = _TLEngine(cc, sc)
                STATS["tl_n"] = 0
                print("[红绿灯] 已启用（HSV 颜色 + 每目标时序稳定）")
            except Exception as e:
                print("[红绿灯] 初始化失败：%s" % e)

    from rknnlite.api import RKNNLite
    rknn = RKNNLite()
    if rknn.load_rknn(args.model) != 0:
        sys.exit("load_rknn 失败")
    if rknn.init_runtime() != 0:
        sys.exit("init_runtime 失败")
    print("[模型] %s   (输入尺寸 %d)" % (os.path.basename(args.model), args.size))

    # The new detector has 11 classes. Stop before OIB startup if a stale
    # 10-class file would make decode_fast() fail inside the inference thread.
    probe = np.full((1, args.size, args.size, 3), 114, dtype=np.uint8)
    probe_out = rknn.inference(inputs=[probe], data_format="nhwc")
    if probe_out is None or len(probe_out) < 9:
        sys.exit("detector preflight failed: expected 9 output tensors")
    cells = (args.size // 8) ** 2
    nc_actual, remainder = divmod(np.asarray(probe_out[1]).size, cells)
    if remainder or nc_actual != len(names):
        sys.exit("detector has %d classes but --classes contains %d; OIB not started" %
                 (nc_actual, len(names)))
    print("[检测类别] %d classes, final class = %s" %
          (nc_actual, names[-1]), flush=True)

    lane_model = None
    if args.lane_model:
        try:
            lane_model = LiveLaneModel(args.lane_model, args.lane_config,
                                       args.lane_conf, args.lane_height,
                                       args.lane_input_mode)
        except Exception as err:
            rknn.release()
            sys.exit("车道线模型初始化失败: %s" % err)
        STATS["lane_status"] = "ready"
        print("[R15 direct+lane] %s  %dx640 %s 每 %d 帧推理一次，仅 CARLA Front" %
              (os.path.basename(args.lane_model), args.lane_height,
               args.lane_input_mode, args.lane_every))

    # ---------------- [HW] 创建硬件编码器 ----------------
    if args.hw_jpeg and HAVE_HW_JPEG:
        # 尺寸传 None：按第一帧自动配置（draw() 加了 35 行标题栏 → 实际是 1280x755）
        HW_ENC = JpegEncoder(width=None, height=None, quality=args.quality, verbose=True)
        STATS["hw"] = "on"
        print("[编码] 硬件 JPEG（mppjpegenc，尺寸按第一帧自动配置）")
    else:
        HW_ENC = None
        STATS["hw"] = "off"
        if args.hw_jpeg and not HAVE_HW_JPEG:
            print("[编码] 硬件 JPEG 不可用（%s）→ 用 cv2 软件编码" % _HW_IMPORT_ERR)
        else:
            print("[编码] cv2 软件 JPEG（--no_hw_jpeg）")

    print("[参数] delay=%.0fus infer_every=%d quality=%d encode_scale=%.2f q_depth=%d  (三级流水线)"
          % (args.delay * 1e6, args.infer_every, args.quality, args.encode_scale, args.q_depth))

    # ---------------- [OIB] 启动与 PC HIL 的通信中枢 ----------------
    global OIB_LINK
    if not args.no_oib and HAVE_OIB:
        try:
            OIB_LINK = OibLink(pc_ip=args.pc_ip, oib_port=args.oib_port, listen_port=8892)
            OIB_LINK.start()
            print("[OIB] 智驾决策通信已启动: UDP 上行 -> %s:%d, 下行 <- 8892" % (args.pc_ip, args.oib_port))
        except Exception as e:
            print("[OIB] 初始化失败: %s" % e)
            OIB_LINK = None

    q_frame = queue.Queue(maxsize=args.q_depth)      # [HW] 原来固定是 2
    q_vis = queue.Queue(maxsize=args.q_depth)        # [HW]
    threading.Thread(target=capture_thread, args=(args, q_frame), daemon=True).start()
    threading.Thread(target=infer_thread, args=(args, names, rknn, lane_model, q_frame, q_vis),
                     daemon=True).start()
    threading.Thread(target=encode_thread, args=(args, q_vis), daemon=True).start()

    srv = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    print("\n==================================================")
    print(" 实时画面:  http://<板子IP>:%d/" % args.port)
    print(" 纯视频流:  http://<板子IP>:%d/stream" % args.port)
    print(" 状态查询:  http://<板子IP>:%d/stat" % args.port)
    print(" 按 Ctrl+C 结束")
    print("==================================================\n")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        STOP = True
        try:
            srv.shutdown()
        except Exception:
            pass
        if OIB_LINK is not None:
            try:
                OIB_LINK.stop()
            except Exception:
                pass
        if HW_ENC is not None:
            try:
                print("[编码统计] " + HW_ENC.report())
                HW_ENC.close()
            except Exception:
                pass
        time.sleep(0.3)
        print("\n已停止")
        os._exit(0)          # 直接退出，避免线程/HTTP 句柄析构时 Segmentation fault


if __name__ == "__main__":
    main()






