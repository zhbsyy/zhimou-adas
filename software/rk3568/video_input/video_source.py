"""Local MP4 reader with the same read_new_bgr contract as the CARLA sources."""

import time

import cv2


class VideoFrameSource:
    def __init__(self, path, loop=False, max_width=1280):
        self.path = path
        self.loop = loop
        self.max_width = max_width
        self.capture = None
        self.fps = 0.0
        self.frames = 0
        self.loops = 0
        self.eof = False
        self._next_at = None
        self._first_frame = None

    def start(self):
        cap = cv2.VideoCapture(self.path)
        if not cap.isOpened():
            raise RuntimeError("cannot open video: %s" % self.path)
        ok, frame = cap.read()
        if not ok or frame is None or frame.ndim != 3 or frame.shape[2] != 3:
            cap.release()
            raise RuntimeError("video has no decodable BGR frames: %s" % self.path)
        self._first_frame = frame
        fps = cap.get(cv2.CAP_PROP_FPS)
        self.fps = fps if 1.0 <= fps <= 120.0 else 25.0
        self.capture = cap
        self._next_at = time.monotonic()
        out_w = min(frame.shape[1], self.max_width)
        out_h = round(frame.shape[0] * out_w / frame.shape[1])
        print("[本地视频] %s | 源 %dx%d | 输出 %dx%d | %.2f FPS | 循环 %s" %
              (self.path, frame.shape[1], frame.shape[0], out_w, out_h,
               self.fps, self.loop),
              flush=True)

    def read_new_bgr(self, previous_seq, timeout=0.2):
        del previous_seq, timeout
        if self.eof:
            return None, self.frames, None
        delay = self._next_at - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        if self._first_frame is not None:
            ok, frame = True, self._first_frame
            self._first_frame = None
        else:
            ok, frame = self.capture.read()
        if not ok and self.loop:
            if self.capture.set(cv2.CAP_PROP_POS_FRAMES, 0):
                ok, frame = self.capture.read()
            if not ok:
                self.capture.release()
                self.capture = cv2.VideoCapture(self.path)
                ok, frame = self.capture.read()
            self._next_at = time.monotonic()
            if ok:
                self.loops += 1
        if not ok or frame is None:
            self.eof = True
            return None, self.frames, None
        if frame.shape[1] > self.max_width:
            out_h = round(frame.shape[0] * self.max_width / frame.shape[1])
            frame = cv2.resize(frame, (self.max_width, out_h),
                               interpolation=cv2.INTER_AREA)
        self.frames += 1
        now = time.monotonic()
        self._next_at = max(self._next_at + 1.0 / self.fps, now)
        return frame, self.frames, time.time()

    def close(self):
        if self.capture is not None:
            self.capture.release()
            self.capture = None
