"""Single-view adapters. Reuse board PCIe decoder; never alter camera controls."""
import importlib.util
import os
from pathlib import Path
import threading
import time

import cv2


class PcieSingleSource:
    def __init__(self, directory, roi=(0, 0, 640, 360)):
        self.directory = Path(directory)
        self.roi = roi
        self.source = None
        self.thread = None
        self.stop_event = threading.Event()
        self.cv = threading.Condition()
        self.latest = (None, 0, None)
        self.error = None
        self.eof = False
        self.loops = 0

    def start(self):
        from pcie_preflight import require_pcie_ready
        require_pcie_ready()  # Never launch the legacy reader with BAR1=0.
        path = self.directory / "pcie_frame.py"
        spec = importlib.util.spec_from_file_location("decision_board_pcie_frame", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.source = module.PcieFrameSource(str(self.directory / "cap_reader.py"))
        self.source.start()
        self.thread = threading.Thread(target=self._read, daemon=True)
        self.thread.start()

    def _read(self):
        seq = 0
        try:
            while not self.stop_event.is_set():
                full = self.source.read_bgr()
                x, y, w, h = self.roi
                single = full[y:y+h, x:x+w]
                if single.shape[:2] != (h, w):
                    raise ValueError("PCIe front crop does not fit the decoded frame")
                seq += 1
                with self.cv:
                    self.latest = (single.copy(), seq, time.time())
                    self.cv.notify_all()
        except Exception as exc:
            self.error = str(exc)
            with self.cv:
                self.cv.notify_all()

    def read_new_bgr(self, previous_seq, timeout=.2):
        with self.cv:
            self.cv.wait_for(lambda: self.latest[1] > previous_seq or self.error is not None, timeout)
            if self.error:
                raise RuntimeError(self.error)
            if self.latest[1] <= previous_seq:
                return None, previous_seq, None
            return self.latest

    def close(self):
        self.stop_event.set()
        if self.source:
            self.source.close()
        if self.thread:
            self.thread.join(timeout=2)


class UsbSingleSource(PcieSingleSource):
    def __init__(self, device):
        super().__init__(".")
        self.device = device

    def start(self):
        if not self.device or not Path(self.device).exists():
            raise ValueError("Specify the actual USB capture node or by-path symlink")
        self.source = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not self.source.isOpened():
            raise RuntimeError("Cannot open USB capture source")
        self.thread = threading.Thread(target=self._read, daemon=True)
        self.thread.start()

    def _read(self):
        seq = 0
        try:
            while not self.stop_event.is_set():
                ok, frame = self.source.read()
                if not ok or frame is None:
                    raise RuntimeError("USB source stopped delivering frames")
                seq += 1
                with self.cv:
                    self.latest = (frame, seq, time.time())
                    self.cv.notify_all()
        except Exception as exc:
            self.error = str(exc)
            with self.cv:
                self.cv.notify_all()

    def close(self):
        self.stop_event.set()
        if self.source is not None:
            self.source.release()
        if self.thread:
            self.thread.join(timeout=2)
