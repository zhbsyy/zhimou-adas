"""Rejection evidence and raw matched-frame HTTP pixels, without real NPU claims."""
import base64
import json
from pathlib import Path
import sys
import time
import unittest
import urllib.error
import urllib.request

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from decision_service import DecisionService
from traffic_light_color import classify_color


class ColorEvidenceTests(unittest.TestCase):
    def make_roi(self, color, radius=2):
        frame = np.full((12, 28, 3), 12, np.uint8)
        cv2.circle(frame, (21, 6), radius, color, -1)
        return frame

    def test_tiny_red_green_yellow_still_classified(self):
        for name, bgr in (("red", (0, 0, 245)), ("green", (0, 245, 0)),
                          ("yellow", (0, 245, 245))):
            with self.subTest(name=name):
                result = classify_color(self.make_roi(bgr), [0, 0, 28, 12])
                self.assertEqual(result.color, name)
                self.assertGreater(result.diagnostics["colors"][name]["accepted_components"], 0)

    def test_blue_is_not_green_and_reports_hue_failure(self):
        result = classify_color(self.make_roi((245, 0, 0)), [0, 0, 28, 12])
        self.assertEqual(result.color, "unknown")
        self.assertEqual(result.diagnostics["failure_stage"], "hue_outside_signal_ranges")

    def test_dim_and_single_pixel_remain_unknown(self):
        dim = classify_color(self.make_roi((0, 60, 0)), [0, 0, 28, 12])
        self.assertEqual(dim.color, "unknown")
        self.assertEqual(dim.diagnostics["failure_stage"], "saturation_or_value_below_threshold")
        tiny = classify_color(self.make_roi((0, 250, 0), 0), [0, 0, 28, 12])
        self.assertEqual(tiny.color, "unknown")
        self.assertEqual(tiny.diagnostics["colors"]["green"]["rejected_area"], 1)

    def test_conflicting_colors_and_white_do_not_allow(self):
        frame = self.make_roi((0, 245, 0))
        cv2.circle(frame, (6, 6), 2, (0, 0, 245), -1)
        self.assertEqual(classify_color(frame, [0, 0, 28, 12]).reason, "ambiguous_colors")
        white = classify_color(self.make_roi((250, 250, 250)), [0, 0, 28, 12])
        self.assertEqual(white.color, "unknown")


class RawSignalHttpTests(unittest.TestCase):
    def setUp(self):
        self.service = DecisionService(evidence_mode="synthetic_test")
        host, port = self.service.start("127.0.0.1", 0)
        self.url = "http://%s:%s" % (host, port)
        self.epoch = self.service.begin_source("test")
        self.frame = np.full((40, 64, 3), 12, np.uint8)
        self.frame[12:22, 24:46] = (18, 27, 39)
        cv2.circle(self.frame, (40, 17), 2, (0, 240, 0), -1)
        self.dets = np.array([[24, 12, 46, 22, .8, 0]], np.float32)

    def tearDown(self):
        self.service.close()

    def get(self, path):
        with urllib.request.urlopen(self.url+path, timeout=2) as r:
            return json.load(r)

    def submit(self, detections=None):
        packet = self.service.frame(self.frame, 1, self.epoch)
        self.service.submit_detection(packet, self.dets if detections is None else detections,
                                      ["traffic light"])
        deadline = time.monotonic()+1
        while time.monotonic() < deadline:
            if self.service.snapshot()["metrics"]["processed_signal"]:
                return
            time.sleep(.005)
        self.fail("summary worker did not complete")

    def test_single_reply_has_exact_raw_pixels_and_no_inference(self):
        self.submit()
        before = self.service.snapshot()["metrics"]["processed_signal"]
        info = self.get("/signal_debug?image=1")
        image = cv2.imdecode(np.frombuffer(base64.b64decode(info["roi_png_base64"]), np.uint8),
                             cv2.IMREAD_COLOR)
        np.testing.assert_array_equal(image, self.frame[12:22, 24:46])
        self.assertEqual(info["source_epoch"], self.epoch)
        self.assertEqual(info["frame_seq"], 1)
        self.assertIn("color_diagnostics", info["signal"])
        self.assertEqual(before, self.service.snapshot()["metrics"]["processed_signal"])
        # One new sample alone still cannot grant green permission.
        self.assertFalse(info["signal"]["valid"])

    def test_png_mismatched_epoch_and_reset_fail_closed(self):
        self.submit()
        with urllib.request.urlopen(self.url+"/signal_roi.png", timeout=2) as r:
            self.assertEqual(r.headers["X-Source-Epoch"], self.epoch)
            image = cv2.imdecode(np.frombuffer(r.read(), np.uint8), cv2.IMREAD_COLOR)
        np.testing.assert_array_equal(image, self.frame[12:22, 24:46])
        with self.assertRaises(urllib.error.HTTPError) as mismatch:
            self.get("/signal_debug?epoch=old&seq=1")
        self.assertEqual(mismatch.exception.code, 409)
        self.service.begin_source("new")
        with self.assertRaises(urllib.error.HTTPError) as reset:
            self.get("/signal_debug?image=1")
        self.assertEqual(reset.exception.code, 503)

    def test_no_unique_target_does_not_export_a_random_light(self):
        self.submit(np.vstack([self.dets, self.dets]))
        info = self.get("/signal_debug?image=1")
        self.assertIsNone(info["roi_url"])
        self.assertNotIn("roi_png_base64", info)
        self.assertEqual(info["signal"]["reason"], "multiple_lights_require_scene_roi")

    def test_source_error_disables_raw_evidence(self):
        self.submit()
        self.service.core.source_failed("disconnected")
        with self.assertRaises(urllib.error.HTTPError) as err:
            self.get("/signal_debug?image=1")
        self.assertEqual(err.exception.code, 503)


if __name__ == "__main__":
    unittest.main(verbosity=2)
