"""Opt-in CARLA MPP/RGA source with a small shared-memory latest-frame slot.

The native process receives MJPEG, decodes with MPP, converts NV12 with RGA,
and publishes BGR into /dev/shm. Python copies only the newest completed frame
under a file lock; no raw video travels through a Unix pipe. Direct mode imports
the MPP DMA-BUF into RGA. Cached output is an opt-in comparison mode.
"""
import fcntl
import json
import mmap
import os
import struct
import subprocess
import tempfile
import threading
import time

import numpy as np

from p2_fresh_source import SourceFrame


HEADER = struct.Struct("<8sIIQQ")
MAGIC = b"ADASMJPG"


class NativeMppCarlaSource:
    def __init__(self, url, width=1280, height=720, program=None, reconnect_delay=.5,
                 direct_dmabuf=False, cached_output=False):
        self.url = url
        self.width, self.height = width, height
        self.program = os.path.abspath(program or os.path.join(os.path.dirname(__file__),
                                                               "mpp_native_stage"))
        if not os.access(self.program, os.X_OK):
            raise RuntimeError("native MPP program is missing or not executable: " + self.program)
        if width <= 0 or height <= 0 or width % 16 or height % 2:
            raise ValueError("native MPP/RGA needs positive width divisible by 16 and even height")
        self.reconnect_delay = reconnect_delay
        self.direct_dmabuf = direct_dmabuf
        if cached_output and not direct_dmabuf:
            raise ValueError("cached output requires direct DMA-BUF mode")
        self.cached_output = cached_output
        self.condition = threading.Condition()
        self.stop_event = threading.Event()
        self.thread = None
        self.proc = None
        self.latest = None
        self.stats = dict(decoded_count=0, native_frames_skipped=0,
                          parser_dropped=None, discarded_bytes=None,
                          decode_failures=0, decode_ms=None, shm_copy_caller_cpu_s=0.0,
                          native_report=None, connected=False,
                          last_error="", reconnects=0,
                          note=("MPP appsink + RGA + shared BGR; " +
                                ("MPP DMA-BUF imported directly" if direct_dmabuf else
                                 "NV12 copied into DMA-HEAP") +
                                ("; cached BGR DMA heap" if cached_output else "") +
                                "; age starts at local publish"))

    def start(self):
        if self.thread is not None:
            raise RuntimeError("source instances are single-start")
        self.thread = threading.Thread(target=self._worker, name="carla_mpp_native", daemon=True)
        self.thread.start()

    def _worker(self):
        frame_size = self.width * self.height * 3
        shared_size = HEADER.size + frame_size
        while not self.stop_event.is_set():
            proc = None
            file = None
            path = None
            view = None
            stderr_thread = None
            diagnostics = []
            last_native_seq = 0
            try:
                file = tempfile.NamedTemporaryFile(prefix="adas_mpp_", dir="/dev/shm", delete=False)
                path = file.name
                file.truncate(shared_size)
                file.flush()
                view = mmap.mmap(file.fileno(), shared_size, access=mmap.ACCESS_READ)
                native_mode = ("publish_direct_cached" if self.cached_output else
                               "publish_direct" if self.direct_dmabuf else "publish")
                proc = subprocess.Popen([self.program, native_mode, self.url, "0", path],
                                        stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                        stderr=subprocess.PIPE, text=True)
                with self.condition:
                    self.proc = proc
                def drain_errors():
                    for line in proc.stderr:
                        if line.strip():
                            diagnostics.append(line.strip()[-300:])
                            if len(diagnostics) > 8:
                                del diagnostics[0]
                stderr_thread = threading.Thread(target=drain_errors, daemon=True)
                stderr_thread.start()
                while not self.stop_event.is_set():
                    if proc.poll() is not None:
                        break
                    fcntl.flock(file.fileno(), fcntl.LOCK_SH)
                    try:
                        magic, width, height, native_seq, mono_ns = HEADER.unpack_from(view)
                        if magic == MAGIC and native_seq > last_native_seq:
                            if (width, height) != (self.width, self.height):
                                raise RuntimeError("native frame geometry changed")
                            copy_cpu_begin = time.thread_time()
                            raw = bytes(view[HEADER.size:shared_size])
                            copy_cpu = time.thread_time()-copy_cpu_begin
                        else:
                            raw = None
                    finally:
                        fcntl.flock(file.fileno(), fcntl.LOCK_UN)
                    if raw is not None:
                        image = np.frombuffer(raw, np.uint8).reshape(self.height, self.width, 3)
                        skipped = max(0, native_seq-last_native_seq-1)
                        last_native_seq = native_seq
                        with self.condition:
                            self.stats["native_frames_skipped"] += skipped
                            self.stats["shm_copy_caller_cpu_s"] += copy_cpu
                            self.stats["decoded_count"] += 1
                            self.stats["connected"] = True
                            self.stats["last_error"] = ""
                            self.latest = SourceFrame(image, self.stats["decoded_count"],
                                                      mono_ns / 1e9)
                            self.condition.notify_all()
                    self.stop_event.wait(.005)
                if not self.stop_event.is_set():
                    report = proc.stdout.read()[-1000:]
                    raise RuntimeError("native MPP exited rc=%s; %s; %s" %
                                       (proc.returncode, report, "; ".join(diagnostics)))
            except Exception as exc:
                with self.condition:
                    if not self.stop_event.is_set():
                        self.stats["decode_failures"] += 1
                        self.stats["reconnects"] += 1
                        self.stats["last_error"] = type(exc).__name__ + ": " + str(exc)
            finally:
                if proc is not None:
                    if proc.poll() is None:
                        proc.terminate()
                    try:
                        proc.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                        proc.wait(timeout=3)
                    if proc.stdout is not None:
                        report_text = proc.stdout.read().strip()
                        if report_text:
                            try:
                                report = json.loads(report_text.splitlines()[-1])
                            except (ValueError, TypeError):
                                report = {"parse_error": report_text[-300:]}
                            with self.condition:
                                self.stats["native_report"] = report
                        proc.stdout.close()
                    if stderr_thread is not None:
                        stderr_thread.join(timeout=1)
                    if proc.stderr is not None:
                        proc.stderr.close()
                if view is not None:
                    view.close()
                if file is not None:
                    file.close()
                if path is not None:
                    os.unlink(path)
                with self.condition:
                    self.proc = None
                    self.stats["connected"] = False
            self.stop_event.wait(self.reconnect_delay)

    def read_newer(self, last_sequence, timeout=.2):
        deadline = time.perf_counter() + timeout
        with self.condition:
            while not self.stop_event.is_set():
                if self.latest is not None and self.latest.sequence > last_sequence:
                    return self.latest
                remaining = deadline - time.perf_counter()
                if remaining <= 0:
                    return None
                self.condition.wait(remaining)
        return None

    def snapshot(self):
        with self.condition:
            out = dict(self.stats)
            out["last_frame_age_ms"] = (round((time.perf_counter() - self.latest.received_mono) * 1000, 2)
                                        if self.latest else None)
            return out

    def close(self):
        self.stop_event.set()
        with self.condition:
            self.condition.notify_all()
            proc = self.proc
        if proc is not None and proc.poll() is None:
            proc.terminate()
        if self.thread is not None and self.thread is not threading.current_thread():
            self.thread.join(timeout=5)
            if self.thread.is_alive():
                raise RuntimeError("native MPP worker did not exit")


class CarlaMppFrameAdapter:
    """Expose the same new-frame contract as the stable CARLA source.

    The timestamp denotes locally available BGR, matching the software source's
    decode-complete origin. It does not include HTTP reception or hardware decode.
    """

    def __init__(self, stream_url, program, width=1280, height=720):
        self.source = NativeMppCarlaSource(
            stream_url, width, height, program=program,
            direct_dmabuf=True, cached_output=True)
        self.fps = 0.0
        self._last_read = None

    def start(self):
        self.source.start()

    def read_new_bgr(self, last_sequence, timeout=0.2):
        frame = self.source.read_newer(last_sequence, timeout=timeout)
        if frame is None:
            return None, last_sequence, 0.0
        now_mono = time.perf_counter()
        publish_lag = now_mono - frame.received_mono
        if publish_lag < -0.01 or publish_lag > 2.0:
            raise RuntimeError("native frame timestamp is stale or uses another clock")
        if self._last_read is not None and now_mono > self._last_read:
            rate = 1.0 / (now_mono - self._last_read)
            self.fps = rate if self.fps == 0 else .9 * self.fps + .1 * rate
        self._last_read = now_mono
        # Existing pcie_stream.py uses epoch time for its overlay and age.
        local_bgr_epoch = time.time() - publish_lag
        return frame.bgr, frame.sequence, local_bgr_epoch

    def snapshot(self):
        return self.source.snapshot()

    def close(self):
        self.source.close()
