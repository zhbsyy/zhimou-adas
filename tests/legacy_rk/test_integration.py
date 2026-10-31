"""CPU-only contract checks. These do not measure RK3568 inference or optical FPS."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
from http.server import ThreadingHTTPServer
from urllib.request import build_opener, ProxyHandler

import cv2
import numpy as np

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import quad_infer as q
from color_profiles import ColorProfiles
from detection_policy import filter_detections, tile_detections
from pipeline import CameraFrame, Latest, make_mosaic, result_valid
from jpeg_lifecycle import JoinedJpegEncoder
from r15_front_fast import R15Front
from runtime_backend import NativeFloatRuntime


class Integration(unittest.TestCase):
    def setUp(self):
        cv2.setNumThreads(1)

    def test_class10_boundary_and_other_classes(self):
        boxes = [[2, 3, 40, 60, score, cid] for cid, score in
                 [(10, .3999), (10, .40), (10, .59), (10, .91), (0, .25), (8, .249), (8, .30)]]
        found = filter_detections(boxes)
        np.testing.assert_array_equal(found[:, 5], [10, 10, 10, 0, 8])
        self.assertTrue(np.all(found[found[:, 5] == 10, 4] >= .40))

    def test_invalid_and_seam_boxes(self):
        boxes = [[10, 10, 20, 30, .9, 10], [650, 10, 700, 30, .8, 0],
                 [600, 10, 680, 50, .9, 2], [10, 10, 20, 30, float("nan"), 10],
                 [10, 10, 20, 30, .9, 11], [10, 10, 20, 30, .9, 2.1]]
        rows = tile_detections(boxes, {"front"})
        self.assertEqual(len(rows["front"]), 1)
        self.assertTrue(all(not len(v) for k, v in rows.items() if k != "front"))

    def test_latest_has_one_item_and_independent_consumers(self):
        slot = Latest()
        for n in range(100):
            slot.publish(n)
        self.assertEqual(slot.newer(0, 0), (100, 99))
        self.assertEqual(slot.newer(50, 0), (100, 99))
        self.assertEqual(slot.newer(100, 0), (100, None))

    def test_layout_and_ownership(self):
        frames = {n: CameraFrame(np.full((360, 640, 3), k, np.uint8), 1, 10)
                  for n, k in [("front", 30), ("right", 60), ("rear", 90), ("left", 120)]}
        mosaic = make_mosaic(frames, 1, 2, 10.1)
        self.assertFalse(mosaic.image.flags.writeable)
        self.assertEqual([int(mosaic.image[y, x, 0]) for y, x in
                          [(100, 100), (100, 900), (500, 100), (500, 900)]], [30, 60, 90, 120])
        render = mosaic.image.copy()
        render[:] = 0
        self.assertEqual(int(frames["front"].image[0, 0, 0]), 30)
        self.assertEqual(int(mosaic.image[0, 0, 0]), 30)
        self.assertFalse(make_mosaic(frames, 2, 2, 11).origins)

    def test_calibrated_model_inputs(self):
        profile = ColorProfiles(HERE.parent/"Configuration/camera_profiles.json")
        raw = np.empty((720, 1280, 3), np.uint8)
        raw[:] = [100, 120, 140]
        baseline = raw.copy()
        front, rear = profile.views(raw)
        # Distinct rear fixed channel order is applied before RGB model conversion.
        self.assertEqual(front[0, 0].tolist(), [98, 134, 130])
        self.assertEqual(rear[0, 0].tolist(), [140, 113, 106])
        mosaic = make_mosaic({"front": CameraFrame(front, 1, 10),
                              "rear": CameraFrame(rear, 1, 10)}, 1, 2, 10)
        tensor, _ = q.pcie_post.preprocess_fast(mosaic.image, 640)
        self.assertEqual(tensor[0, 200, 100].tolist(), [130, 134, 98])
        lane = R15Front.__new__(R15Front)
        lane.height, lane.width, lane.input_mode = 256, 448, "uint8_nhwc"
        lane_input, _ = lane.preprocess(front)
        self.assertEqual(lane_input.shape, (1, 256, 448, 3))
        self.assertEqual(lane_input[0, 100, 100].tolist(), [130, 134, 98])
        np.testing.assert_array_equal(raw, baseline)

    def test_stale_result_and_color_revision(self):
        mosaic = make_mosaic({"front": CameraFrame(np.zeros((360, 640, 3), np.uint8), 20, 10.2)}, 7, 2, 10.2)
        result = {"color_revision": 2, "origins": {"front": (19, 10)}}
        self.assertTrue(result_valid(result, mosaic, "front", 10.3))
        self.assertFalse(result_valid(result, mosaic, "front", 10.6))
        result["color_revision"] = 1
        self.assertFalse(result_valid(result, mosaic, "front", 10.3))

    def test_detector_to_render_and_http(self):
        from types import SimpleNamespace
        args = SimpleNamespace(color_profile=str(HERE.parent/"Configuration/camera_profiles.json"),
            classes=str(HERE.parent/"Models/classes_11.txt"), model="mock_det", lane_model="mock_lane")
        with patch.object(q, "check_model", return_value="TEST_ONLY"):
            app = q.Application(args)
        app.detector = SimpleNamespace(inference=lambda **kw: [np.ones(1, np.float32)]*9)
        now = time.perf_counter()
        clean = np.full((360, 640, 3), 180, np.uint8)
        mosaic = make_mosaic({"front": CameraFrame(clean, 12, now)}, 5, 2, now)
        predictions = np.asarray([[10, 145, 50, 175, .39, 10],
                                  [100, 145, 150, 175, .41, 10],
                                  [200, 145, 250, 175, .3, 0]], np.float32)
        with patch.object(q.pcie_post, "decode_auto", return_value=predictions):
            worker = threading.Thread(target=app.detect)
            app.mosaics.publish(mosaic)
            worker.start()
            _, result = app.detections.newer(0, 2)
            app.stop.set()
            worker.join(2)
        self.assertIsNotNone(result)
        self.assertEqual(len(result["tiles"]["front"]), 2)
        baseline = mosaic.image.copy()
        # Render twice from the clean mosaic: no accumulation / stale pixel cache.
        first = app.render_frame(mosaic, now+.1)
        np.testing.assert_array_equal(app.render_frame(mosaic, now+.1), first)
        np.testing.assert_array_equal(mosaic.image, baseline)
        self.assertFalse(np.array_equal(first, baseline))
        server = ThreadingHTTPServer(("127.0.0.1", 0), q.make_handler(app))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            opener = build_opener(ProxyHandler({}))
            body = json.loads(opener.open("http://127.0.0.1:%d/results" % server.server_port, timeout=3).read())
            self.assertEqual(body["class10_min_confidence"], .40)
            self.assertFalse(body["threshold_override_supported"])
            rows = body["detections"]["front"]
            self.assertEqual(len(rows), 2)
            self.assertTrue(all(row[4] >= .4 for row in rows if row[5] == 10))
        finally:
            server.shutdown()
            server.server_close()
            thread.join(2)

    def test_jpeg_workers_join_and_callback_publication(self):
        """A real child pipe emits minimal JPEG markers; no MPP performance claim."""
        with tempfile.TemporaryDirectory(dir=HERE/"tests") as tmp:
            child = Path(tmp)/"child.py"
            child.write_text("import os\nwhile os.read(0, 1):\n os.write(1, b'\\xff\\xd8hello\\xff\\xd9')\n", encoding="utf-8")
            proc = subprocess.Popen([sys.executable, "-u", str(child)], stdin=subprocess.PIPE,
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)
            seen, ready = [], threading.Event()
            def callback(jpg):
                seen.append(jpg)
                ready.set()
            encoder = JoinedJpegEncoder(verbose=False, depth=1, on_jpeg=callback)
            import queue
            encoder._in_q = queue.Queue(maxsize=1)
            encoder._proc = proc
            encoder._reader = threading.Thread(target=encoder._read_loop, args=(proc,))
            encoder._writer = threading.Thread(target=encoder._write_loop, args=(proc,))
            workers = encoder._reader, encoder._writer
            for worker in workers:
                worker.start()
            try:
                encoder._in_q.put(b"x")
                self.assertTrue(ready.wait(3))
                self.assertEqual(seen, [b"\xff\xd8hello\xff\xd9"])
                self.assertEqual(encoder._out_q.qsize(), 0)
            finally:
                encoder.close()
            self.assertTrue(all(not worker.is_alive() for worker in workers))
            self.assertIsNotNone(proc.poll())

    def test_all_stage_smoke_with_synthetic_runtime(self):
        """Run production workers together, substituting only hardware boundaries."""
        from types import SimpleNamespace
        from contextlib import redirect_stdout
        import io
        class Runtime:
            def load_rknn(self, path):
                self.lane = "lane" in path
                return 0
            def init_runtime(self):
                return 0
            def release(self):
                pass
            def inference(self, inputs, data_format):
                self_test.assertEqual(inputs[0].shape, (1, 256, 448, 3) if self.lane else (1, 640, 640, 3))
                self_test.assertEqual(inputs[0].dtype, np.uint8)
                time.sleep(.003)
                if self.lane:
                    direct = np.full((1, 2, 256, 448), -2, np.float32)
                    direct[:, 1, 100:] = 3
                    lane = np.zeros((1, 3, 256, 448), np.float32)
                    lane[:, 1, 100:, 80:83] = 8
                    lane[:, 2, 100:, 360:363] = 8
                    return [direct, lane]
                outputs = []
                for gs in (80, 40, 20):
                    box = np.zeros((1, 64, gs, gs), np.float32)
                    cls = np.zeros((1, 11, gs, gs), np.float32)
                    total = np.zeros((1, 1, gs, gs), np.float32)
                    if gs == 80:
                        cls[0, 10, 30, 20], total[0, 0, 30, 20] = .7, .7
                    outputs.extend([box, cls, total])
                return outputs
        class Decoder:
            convert_ms = .1
            def __init__(self, *args, **kwargs):
                pass
            def start(self):
                pass
            def close(self):
                pass
            def read_bgr(self):
                time.sleep(.01)
                return np.full((720, 1280, 3), 110, np.uint8)
        class USB:
            def start(self):
                pass
            def close(self):
                pass
            def read_newer(self, *args, **kwargs):
                return SimpleNamespace(bgr=np.full((240, 320, 3), 130, np.uint8), sequence=1,
                                       received_mono=time.perf_counter())
            def snapshot(self):
                return {"test_only": True}
        class Encoder:
            broken = False
            def __init__(self, *args, on_jpeg, **kwargs):
                self.publish, self.stats = on_jpeg, {}
            def submit_bgr(self, image):
                ok, jpg = cv2.imencode(".jpg", image)
                self_test.assertTrue(ok)
                self.publish(jpg.tobytes())
                return True
            def close(self):
                pass
        self_test = self
        args = SimpleNamespace(color_profile=str(HERE.parent/"Configuration/camera_profiles.json"),
            classes=str(HERE.parent/"Models/classes_11.txt"), model="mock_det", lane_model="mock_lane", port=0,
            video_fps=20, seconds=.5)
        with patch.object(q, "check_model", return_value="TEST_ONLY"), \
             patch.object(q, "runtime_factory", return_value=Runtime), \
             patch.object(q, "PcieFrameSource", Decoder), \
             patch.object(q, "resolve_usb_devices", return_value={"left": "left", "right": "right"}), \
             patch.object(q, "open_usb_camera", side_effect=lambda *args, **kwargs: USB()), \
             patch.object(q, "JoinedJpegEncoder", Encoder), redirect_stdout(io.StringIO()):
            app = q.Application(args)
            app.run()
        self.assertFalse(app.metrics.snapshot()["errors"])
        for stage in ("pcie_capture", "compose", "det", "lane", "render", "jpeg"):
            self.assertGreater(app.metrics.snapshot()["counts"].get(stage, 0), 0, stage)
        _, item = app.jpeg.get()
        image = cv2.imdecode(np.frombuffer(item[0], np.uint8), cv2.IMREAD_COLOR)
        self.assertEqual(image.shape, (720, 1280, 3))
        self.assertTrue(all(not thread.is_alive() for thread in app.threads))


if __name__ == "__main__":
    unittest.main()
