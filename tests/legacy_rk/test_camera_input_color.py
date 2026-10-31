"""Input gains, neutral reference rejection, and unchanged capture metadata."""
import base64
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch
from collections import defaultdict
import queue

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from camera_input_color import CameraInputColor, estimate_neutral_gains
import calibrate_camera_color
from decision_service import DecisionService
from traffic_light_color import classify_color


class CameraColorTests(unittest.TestCase):
    def profile(self, folder, gains):
        p = Path(folder)/"color.json"
        p.write_text(json.dumps({"schema": "camera_input_color.v1", "source": "pcie",
                                 "gains_rgb": gains}), encoding="utf-8")
        return str(p)

    def test_identity_and_explicit_gain_keep_raw_owned_by_capture(self):
        raw = np.full((30, 40, 3), (100, 100, 100), np.uint8)
        self.assertIs(CameraInputColor().apply(raw), raw)
        with tempfile.TemporaryDirectory() as folder:
            color = CameraInputColor(self.profile(folder, [1, 1.19, .94]))
            adjusted = color.apply(raw)
        np.testing.assert_array_equal(adjusted[0, 0], [94, 119, 100])
        np.testing.assert_array_equal(raw[0, 0], [100, 100, 100])
        self.assertEqual(adjusted.shape, raw.shape)

    def test_neutral_reference_balances_diagonal_cast(self):
        raw = np.full((80, 120, 3), (180, 130, 190), np.uint8)
        profile = estimate_neutral_gains(raw, [.2, .2, .8, .8])
        with tempfile.TemporaryDirectory() as folder:
            color = CameraInputColor(self.profile(folder, profile["gains_rgb"]))
            adjusted = color.apply(raw)
        self.assertLessEqual(int(adjusted[0, 0].max())-int(adjusted[0, 0].min()), 1)

    def test_bad_reference_and_profile_rejected(self):
        for value in ((15, 15, 15), (255, 255, 255), (230, 40, 230)):
            with self.subTest(reference=value), self.assertRaises(ValueError):
                estimate_neutral_gains(np.full((80, 120, 3), value, np.uint8), [.2, .2, .8, .8])
        textured = np.full((80, 120, 3), 180, np.uint8)
        textured[::2] = 50
        with self.assertRaises(ValueError):
            estimate_neutral_gains(textured, [.2, .2, .8, .8])
        with tempfile.TemporaryDirectory() as folder:
            for gains in ([1, 0, 1], [1, float("nan"), 1], [1, 1]):
                with self.subTest(gains=gains), self.assertRaises(ValueError):
                    CameraInputColor(self.profile(folder, gains))

    def test_white_balance_does_not_turn_blue_into_green(self):
        image = np.full((20, 40, 3), 15, np.uint8)
        cv2.circle(image, (30, 10), 3, (230, 0, 0), -1)
        with tempfile.TemporaryDirectory() as folder:
            color = CameraInputColor(self.profile(folder, [1, 1.19, .94]))
            self.assertEqual(classify_color(color.apply(image), [0, 0, 40, 20]).color, "unknown")

    def test_raw_debug_remains_before_gain_without_lights(self):
        service = DecisionService(evidence_mode="live")
        try:
            epoch = service.begin_source("pcie")
            raw = np.full((40, 60, 3), (100, 80, 110), np.uint8)
            corrected = np.full_like(raw, 97)
            service.frame(corrected, 1, epoch, raw_bgr=raw,
                          camera_color={"gains_rgb": [1, 1.2, 1]})
            code, sample = service.camera_debug(True)
            self.assertEqual(code, 200)
            png = cv2.imdecode(np.frombuffer(base64.b64decode(sample["raw_png_base64"]), np.uint8), cv2.IMREAD_COLOR)
            np.testing.assert_array_equal(png, raw)
            self.assertEqual(service.snapshot()["camera_input"]["gains_rgb"], [1, 1.2, 1])
            service.core.source_failed("unplugged")
            self.assertEqual(service.camera_debug(True)[0], 503)
        finally:
            service.close()

    def test_calibration_cli_saves_local_profile_and_retains_previous(self):
        raw = np.full((80, 120, 3), (180, 130, 190), np.uint8)
        ok, png = cv2.imencode(".png", raw)
        self.assertTrue(ok)
        data = {"schema": "rk_camera_debug.v1", "source": "pcie", "evidence_mode": "live",
                "stage": "before_input_gains_and_all_overlays", "shape": list(raw.shape),
                "raw_png_base64": base64.b64encode(png).decode("ascii"),
                "source_epoch": "declared_fixture_not_hardware", "frame_seq": 12}
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder)/"camera_color_profile.local.json"
            out.write_text("previous fixture profile", encoding="utf-8")
            argv = ["calibrate_camera_color.py", "--confirm-neutral", "--output", str(out)]
            with patch.object(sys, "argv", argv), patch.object(
                    calibrate_camera_color.urllib.request, "urlopen",
                    return_value=io.BytesIO(json.dumps(data).encode())):
                calibrate_camera_color.main()
            profile = CameraInputColor(out)
            adjusted = profile.apply(raw)
            self.assertLessEqual(int(adjusted[0, 0].max())-int(adjusted[0, 0].min()), 1)
            self.assertEqual(next(Path(folder).glob("*.before_*.json")).read_text(encoding="utf-8"),
                             "previous fixture profile")

    def test_capture_queues_share_one_corrected_frame_and_original(self):
        saved_base = sys.modules.get("pcie_stream_r15_profile")
        saved_sources = sys.modules.get("single_sources")
        base = types.ModuleType("pcie_stream_r15_profile")
        base.STOP = False
        base.OIB_LINK = None
        base.STATS = defaultdict(float)
        base.put_latest = lambda q, item: q.put_nowait(item)
        sys.modules["pcie_stream_r15_profile"] = base
        raw = np.full((40, 60, 3), 100, np.uint8)
        class Source:
            error = None
            eof = False
            count = 0
            def __init__(self, *a): pass
            def start(self): pass
            def close(self): pass
            def read_new_bgr(self, seq, timeout):
                self.count += 1
                if self.count > 1:
                    self.eof = True
                    return None, 1, None
                return raw, 1, 123.
        sys.modules["single_sources"] = types.SimpleNamespace(PcieSingleSource=Source)
        service = DecisionService()
        try:
            spec = importlib.util.spec_from_file_location("camera_capture_test", Path(__file__).resolve().parents[1]/"run_vision.py")
            vision = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(vision)
            vision.DECISIONS = service
            with tempfile.TemporaryDirectory() as folder:
                args = types.SimpleNamespace(source="pcie", pcie_dir="fixture", front_crop=[0, 0, 60, 40],
                                             camera_color_profile=self.profile(folder, [1, 1.19, .94]))
                qs = [queue.Queue() for _ in range(3)]
                vision.capture_worker(args, *qs)
            packets = [q.get_nowait() for q in qs]
            self.assertTrue(all(p is packets[0] for p in packets))
            self.assertIs(packets[0].raw_bgr, raw)
            np.testing.assert_array_equal(packets[0].bgr[0, 0], [94, 119, 100])
            np.testing.assert_array_equal(raw[0, 0], [100, 100, 100])
        finally:
            service.close()
            for name, old in (("pcie_stream_r15_profile", saved_base), ("single_sources", saved_sources)):
                if old is None:
                    sys.modules.pop(name, None)
                else:
                    sys.modules[name] = old


if __name__ == "__main__":
    unittest.main(verbosity=2)
