"""Latest-frame MJPEG source. Times use perf_counter, never the remote PC clock.

SOI/EOI framing is intended for CARLA/OpenCV MJPEG (no nested JPEG thumbnails).
It counts new decoded arrivals, not unique sensor exposures or CARLA world ticks.
"""
from dataclasses import dataclass
import socket
import threading
import time
import urllib.request

import cv2
import numpy as np


class LatestJpegParser:
    def __init__(self, max_buffer_bytes=8*1024*1024):
        if max_buffer_bytes < 1024:
            raise ValueError("buffer too small")
        self.limit = max_buffer_bytes
        self.buffer = bytearray()
        self.dropped = 0
        self.discarded_bytes = 0
        self.completed = 0

    def feed(self, chunk):
        self.buffer.extend(chunk)
        if len(self.buffer) > self.limit:
            # Bounded memory even for a broken HTTP server or missing EOI.
            tail = self.buffer[-self.limit:]
            self.discarded_bytes += len(self.buffer)-len(tail)
            self.buffer = tail
        latest = None
        while True:
            start = self.buffer.find(b"\xff\xd8")
            if start < 0:
                # A split SOI can begin with the final 0xff byte.
                keep = 1 if self.buffer.endswith(b"\xff") else 0
                self.discarded_bytes += len(self.buffer)-keep
                self.buffer = self.buffer[-1:] if keep else bytearray()
                break
            if start:
                self.discarded_bytes += start
                del self.buffer[:start]
            end = self.buffer.find(b"\xff\xd9",2)
            if end < 0:
                break
            current = bytes(self.buffer[:end+2])
            del self.buffer[:end+2]
            if latest is not None:
                self.dropped += 1
            self.completed += 1
            latest = current
        return latest


@dataclass(frozen=True)
class SourceFrame:
    bgr: np.ndarray
    sequence: int
    received_mono: float


class CarlaSource:
    def __init__(self, url, timeout=3, max_buffer_bytes=8*1024*1024):
        self.url, self.timeout, self.max_buffer = url, timeout, max_buffer_bytes
        self.condition = threading.Condition()
        self.stop_event = threading.Event()
        self.thread = None
        self.response = None
        self.latest = None
        self.stats = dict(decoded_count=0, parser_dropped=0, discarded_bytes=0,
                          decode_failures=0, decode_ms=0, decode_caller_cpu_s=0.0, connected=False,
                          last_error="", reconnects=0)

    def start(self):
        if self.thread is not None:
            raise RuntimeError("source instances are single-start; create a new instance after close")
        self.thread = threading.Thread(target=self.worker, name="carla_http", daemon=True)
        self.thread.start()

    def worker(self):
        while not self.stop_event.is_set():
            response = None
            parser = LatestJpegParser(self.max_buffer)
            try:
                req = urllib.request.Request(self.url,headers={"User-Agent":"RK3568-optimization-trial","Cache-Control":"no-cache"})
                response = urllib.request.urlopen(req, timeout=self.timeout)
                with self.condition:
                    self.response = response
                    self.stats["connected"] = True
                reader = getattr(response,"read1",response.read)
                while not self.stop_event.is_set():
                    chunk = reader(262144)
                    if not chunk:
                        raise EOFError("MJPEG stream closed")
                    previous_drops, previous_bytes = parser.dropped, parser.discarded_bytes
                    jpeg = parser.feed(chunk)
                    with self.condition:
                        self.stats["parser_dropped"] += parser.dropped-previous_drops
                        self.stats["discarded_bytes"] += parser.discarded_bytes-previous_bytes
                    if jpeg is None:
                        continue
                    begin = time.perf_counter()
                    cpu_begin = time.thread_time()
                    image = cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR)
                    now = time.perf_counter()
                    decode_cpu = time.thread_time()-cpu_begin
                    with self.condition:
                        self.stats["decode_caller_cpu_s"] += decode_cpu
                        if image is None:
                            self.stats["decode_failures"] += 1
                            continue
                        image.flags.writeable = False
                        self.stats["decoded_count"] += 1
                        self.stats["decode_ms"] = round((now-begin)*1000,3)
                        self.stats["last_error"] = ""
                        self.latest = SourceFrame(image,self.stats["decoded_count"],now)
                        self.condition.notify_all()
            except Exception as exc:
                with self.condition:
                    if not self.stop_event.is_set():
                        self.stats["last_error"] = type(exc).__name()+": "+str(exc)
                        self.stats["reconnects"] += 1
            finally:
                with self.condition:
                    self.stats["connected"] = False
                    self.response = None
                if response is not None:
                    response.close()
            self.stop_event.wait(.5)

    def read_newer(self, last_sequence, timeout=.2):
        deadline = time.perf_counter()+timeout
        with self.condition:
            while not self.stop_event.is_set():
                if self.latest is not None and self.latest.sequence > last_sequence:
                    return self.latest
                wait = deadline-time.perf_counter()
                if wait <= 0:
                    return None
                self.condition.wait(wait)
        return None

    def snapshot(self):
        with self.condition:
            out = dict(self.stats)
            out["last_frame_age_ms"] = round((time.perf_counter()-self.latest.received_mono)*1000,2) if self.latest else None
            return out

    def close(self):
        self.stop_event.set()
        with self.condition:
            self.condition.notify_all()
            response = self.response
        if response is not None:
            # Interrupt a blocking read where CPython exposes its socket; the HTTP
            # timeout remains the fallback. The worker owns response.close().
            try:
                response.fp.raw._sock.shutdown(socket.SHUT_RDWR)
            except (AttributeError,OSError):
                pass
        if self.thread is not None and self.thread is not threading.current_thread():
            self.thread.join(self.timeout+1)
            if self.thread.is_alive():
                raise RuntimeError("HTTP worker did not exit within timeout")
