"""Publish every parsed hardware JPEG without waiting for another submission.

The hardware reader only swaps immutable bytes/counters. HTTP writes remain
in existing handler threads; no codec settings or image pixels are changed.
"""
import queue
import threading
import time

import cv2


def install_callback(base):
    if base.HW_ENC is None:
        raise RuntimeError("Publication trial needs the existing hardware encoder")
    last = [None]
    publication_lock = threading.Lock()
    base.STATS["publication_mode"] = "hardware_callback"
    base.STATS["age_ms"] = -1.0
    base.STATS["age_min_ms"] = -1.0
    base.STATS["frame_age_valid"] = False

    def publish(jpg):
        if not jpg:
            return
        with publication_lock:
            now = time.monotonic()
            base.LATEST.update(jpeg=jpg, seq=base.LATEST["seq"]+1)
            base.STATS["stream_count"] += 1
            if last[0] is not None:
                dt = max(now-last[0], 1e-6)
                old = base.STATS.get("enc_fps", 0.0)
                base.STATS["enc_fps"] = 1/dt if old <= 0 else .9*old+.1/dt
            last[0] = now

    def checked_publish(jpg):
        try:
            publish(jpg)
        except Exception as exc:
            base.STATS["publication_error"] = str(exc)
            base.STOP = True
            raise

    base.HW_ENC.on_jpeg = checked_publish
    return checked_publish


def submit_worker(base, args, q_vis):
    """Keep one bounded input queue; the existing reader publishes outputs."""
    try:
        publish = base.HW_ENC.on_jpeg
        while not base.STOP:
            try:
                vis, captured_at = q_vis.get(timeout=.5)
            except queue.Empty:
                continue
            base.STATS["q_wait_ms"] = (time.time()-captured_at)*1000
            if args.encode_scale != 1.0:
                h, w = vis.shape[:2]
                vis = cv2.resize(vis, (int(w*args.encode_scale), int(h*args.encode_scale)))
            started = time.perf_counter()
            submitted = base.HW_ENC.submit_bgr(vis)
            if not submitted and base.HW_ENC.broken:
                # Existing fallback only when hardware is broken, never merely
                # because an asynchronous frame has not been produced yet.
                publish(base.HW_ENC._sw_fallback(vis))
            elapsed_ms = (time.perf_counter()-started)*1000
            old = base.STATS.get("enc_ms", 0.0)
            base.STATS["enc_ms"] = elapsed_ms if old <= 0 else .9*old+.1*elapsed_ms
            stats = base.HW_ENC.stats
            base.STATS["hw"] = "on" if stats["hw"] else "broken" if base.HW_ENC.broken else "waiting"
            base.STATS["enc_hw_frames"] = stats["produced"]
            base.STATS["enc_sw_frames"] = stats["sw"]
            base.STATS["enc_submitted"] = stats["submitted"]
    except Exception as exc:
        base.STATS["publication_error"] = str(exc)
        print("[JPEG publication stopped] %s" % exc, flush=True)
        base.STOP = True
