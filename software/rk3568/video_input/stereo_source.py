"""Two real eyes in one legacy DMA payload, one eye presented to RKNN/UI."""
import time
import numpy as np
from single_sources import PcieSingleSource


class PcieStereoSource(PcieSingleSource):
    def __init__(self, directory):
        super().__init__(directory)
        self.latest = (None, None, 0, None)

    def _read(self):
        seq = 0
        try:
            while not self.stop_event.is_set():
                full = self.source.read_bgr()
                if full.shape != (720, 1280, 3) or full.dtype != np.uint8:
                    raise ValueError('Stereo requires the legacy 1280x720 BGR DMA layout')
                # One owned copy for both eyes from the SAME decoded payload.
                # Lifetime extends through both worker queues. No later in-place edits.
                pair = full[:360, :1280].copy()
                pair.flags.writeable = False
                seq += 1
                with self.cv:
                    self.latest = (pair[:, :640], pair[:, 640:], seq, time.time())
                    self.cv.notify_all()
        except Exception as exc:
            self.error = str(exc)
            with self.cv:
                self.cv.notify_all()

    def read_new_stereo(self, previous_seq, timeout=.2):
        with self.cv:
            self.cv.wait_for(lambda: self.latest[2] > previous_seq or self.error is not None, timeout)
            if self.error:
                raise RuntimeError(self.error)
            if self.latest[2] <= previous_seq:
                return None, None, previous_seq, None
            return self.latest


def validate_pair(left, right):
    if (left is None or right is None or left.shape != (360, 640, 3) or
            right.shape != left.shape or left.dtype != np.uint8 or right.dtype != np.uint8):
        return False, 'missing_or_wrong_size_pair'
    if np.array_equal(left, right):
        return False, 'identical_eye_replicas'
    for eye in (left, right):
        if float(eye.std()) < 2:
            return False, 'blank_or_untextured_eye'
    return True, 'two_distinct_images_not_proof_of_exposure_sync'
