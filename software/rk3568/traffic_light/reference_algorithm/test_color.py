"""Behavior checks on artificial lamps, tracking and YOLO decoding.

Synthetic checks cannot establish accuracy on actual road video.
Run: python -m unittest -v test_color
"""
import unittest

import cv2
import numpy as np

from traffic_light_color import TrafficLightColorEngine, classify_color, draw_results
from video_test import decode_onnx, letterbox


NAMES = ["traffic light", "car"]
BOX_A = (20, 30, 60, 130)
BOX_B = (160, 30, 200, 130)
COLORS = {"red": (0, 0, 255), "green": (0, 255, 0),
          "yellow": (0, 255, 255), "white": (255, 255, 255), "dark": (0, 0, 65)}


def scene(a="red", b="green"):
    frame = np.full((160, 240, 3), 30, dtype=np.uint8)
    for box, color in ((BOX_A, a), (BOX_B, b)):
        x1, y1, x2, y2 = box
        frame[y1:y2, x1:x2] = 10
        if color in COLORS:
            cv2.circle(frame, ((x1 + x2) // 2, y1 + 22), 8, COLORS[color], -1)
    return frame


def detections(reverse=False):
    values = [list(BOX_A) + [.9, 0], list(BOX_B) + [.8, 0]]
    return np.asarray(values[::-1] if reverse else values, dtype=np.float32)


class ColorTests(unittest.TestCase):
    def test_three_colors(self):
        for color in ("red", "yellow", "green"):
            with self.subTest(color=color):
                self.assertEqual(classify_color(scene(color), BOX_A).color, color)

    def test_red_high_hue_range(self):
        frame = scene("none")
        bgr = cv2.cvtColor(np.uint8([[[175, 230, 250]]]), cv2.COLOR_HSV2BGR)[0, 0]
        cv2.circle(frame, (40, 52), 8, tuple(int(v) for v in bgr), -1)
        self.assertEqual(classify_color(frame, BOX_A).color, "red")

    def test_dark_white_and_empty_unknown(self):
        for color in ("dark", "white", "none"):
            with self.subTest(color=color):
                self.assertEqual(classify_color(scene(color), BOX_A).color, "unknown")

    def test_conflicting_lamps_unknown(self):
        frame = scene("red")
        cv2.circle(frame, (40, 100), 8, COLORS["green"], -1)
        self.assertEqual(classify_color(frame, BOX_A).reason, "ambiguous_colors")

    def test_bad_and_tiny_boxes(self):
        for box in ((20, 30, 20, 40), (-8, -8, -2, -2), (0, 0, 4, 4), (np.nan, 0, 2, 3)):
            self.assertEqual(classify_color(scene(), box).color, "unknown")

    def test_matte_yellow_surface_is_not_lit_lamp(self):
        frame = scene("none")
        cv2.circle(frame, (40, 52), 10, (25, 130, 150), -1)
        self.assertEqual(classify_color(frame, BOX_A).color, "unknown")

    def test_padding_recovers_cut_lamp_but_not_neighbor(self):
        frame = scene()
        self.assertEqual(classify_color(frame, (20, 53, 60, 130)).color, "red")
        self.assertEqual(classify_color(frame, (70, 30, 110, 130)).color, "unknown")

    def test_no_input_mutation(self):
        frame = scene()
        before = frame.copy()
        engine = TrafficLightColorEngine()
        result = engine.update(frame, detections(), NAMES, 0)
        draw_results(frame, detections(), NAMES, result)
        np.testing.assert_array_equal(frame, before)


class TemporalTests(unittest.TestCase):
    def setUp(self):
        self.engine = TrafficLightColorEngine()

    def update(self, a="red", b="green", t=0, reverse=False):
        return self.engine.update(scene(a, b), detections(reverse), NAMES, t)

    def test_two_targets_and_order_change(self):
        first = self.update(t=0)
        self.assertEqual([x["stable_color"] for x in first], ["unknown", "unknown"])
        second = self.update(t=.1, reverse=True)
        self.assertEqual([x["stable_color"] for x in second], ["green", "red"])
        self.assertEqual([x["track_id"] for x in second], [first[1]["track_id"], first[0]["track_id"]])

    def test_switch_does_not_show_old_color_as_current(self):
        self.update(t=0)
        old = self.update(t=.1)[0]
        pending = self.update(a="green", t=.2)[0]
        self.assertEqual(pending["stable_color"], "unknown")
        self.assertEqual(pending["status"], "pending")
        confirmed = self.update(a="green", t=.3)[0]
        self.assertEqual(confirmed["stable_color"], "green")
        self.assertEqual(confirmed["track_id"], old["track_id"])

    def test_short_unknown_held_then_expires(self):
        self.update(t=0)
        self.update(t=.1)
        held = self.update(a="white", t=.2)[0]
        self.assertEqual((held["stable_color"], held["status"]), ("red", "held"))
        self.assertEqual(self.update(a="white", t=.5)[0]["stable_color"], "unknown")

    def test_contrary_evidence_does_not_resurrect_old_state(self):
        self.update(t=0)
        self.update(t=.1)
        self.update(a="green", t=.2)
        result = self.update(a="white", t=.25)[0]
        self.assertEqual(result["stable_color"], "unknown")

    def test_low_fps_current_evidence_stays_confirmed(self):
        self.update(t=0)
        self.assertEqual(self.update(t=.5)[0]["stable_color"], "red")
        self.assertEqual(self.update(t=1.0)[0]["stable_color"], "red")

    def test_long_gaps_create_new_track(self):
        old = self.update(t=0)[0]["track_id"]
        new = self.update(t=2)[0]
        self.assertNotEqual(old, new["track_id"])
        self.assertEqual(new["stable_color"], "unknown")

    def test_empty_and_nontraffic_detections(self):
        self.update(t=0)
        self.assertEqual(self.engine.update(scene(), np.empty((0, 6)), NAMES, .1), [])
        car = np.asarray([list(BOX_A) + [.9, 1]])
        self.assertEqual(self.engine.update(scene(), car, NAMES, .2), [])

    def test_same_emitter_is_merged(self):
        dets = np.asarray([list(BOX_A) + [.9, 0], [18, 28, 63, 133, .8, 0],
                           list(BOX_B) + [.8, 0]])
        result = self.engine.update(scene(), dets, NAMES, 0)
        self.assertEqual(len(result), 2)
        result = self.engine.update(scene(), dets, NAMES, .1)
        self.assertEqual([item["stable_color"] for item in result], ["red", "green"])

    def test_face_box_size_change_does_not_duplicate_emitter(self):
        full = np.asarray([list(BOX_A) + [.9, 0]])
        self.engine.update(scene(), full, NAMES, 0)
        self.engine.update(scene(), full, NAMES, .1)
        changed = np.asarray([[20, 65, 60, 130, .9, 0], [31, 43, 49, 61, .8, 0]])
        result = self.engine.update(scene(), changed, NAMES, .2)
        self.assertEqual(sum(item["stable_color"] == "red" for item in result), 1)
        self.assertEqual(next(item for item in result if item["stable_color"] == "red")["status"], "confirmed")

    def test_duplicate_time_does_not_confirm(self):
        self.update(t=0)
        self.assertEqual(self.update(t=0)[0]["stable_color"], "unknown")

    def test_bad_time_and_new_clip_reset(self):
        self.update(t=1)
        with self.assertRaises(ValueError):
            self.update(t=.5)
        self.engine.reset()
        self.assertEqual(self.update(t=0)[0]["track_id"], 1)


class DecoderTests(unittest.TestCase):
    def test_single_output_classwise_nms(self):
        # Two overlapping TL boxes and one same-location car must give 2 results.
        pred = np.array([[[40, 41, 40], [50, 51, 50], [20, 20, 20], [30, 30, 30],
                          [.9, .8, .1], [.1, .2, .85]]], dtype=np.float32)
        dets = decode_onnx([pred], 2, 640, .25, .45)
        self.assertEqual(len(dets), 2)
        self.assertEqual(set(dets[:, 5]), {0, 1})
        np.testing.assert_allclose(dets[0, :4], [30, 35, 50, 65])

    def test_head9_dfl(self):
        outputs = []
        for gs in (8, 4, 2):
            reg = np.full((1, 64, gs, gs), -30, dtype=np.float32)
            reg[:, [1, 17, 33, 49]] = 30  # DFL distance=1 on each side
            cls = np.zeros((1, 2, gs, gs), dtype=np.float32)
            if gs == 8:
                cls[0, 0, 3, 4] = .9
            outputs.extend((reg, cls, cls.sum(axis=1, keepdims=True)))
        dets = decode_onnx(outputs, 2, 64, .25, .45)
        self.assertEqual(len(dets), 1)
        np.testing.assert_allclose(dets[0, :4], [28, 20, 44, 36], atol=1e-5)

    def test_class_count_mismatch_rejected(self):
        with self.assertRaises(ValueError):
            decode_onnx([np.zeros((1, 14, 8400), np.float32)], 2, 640, .25, .45)

    def test_letterbox_geometry(self):
        rgb, (sx, sy, left, top) = letterbox(np.zeros((360, 640, 3), np.uint8), 640)
        self.assertEqual(rgb.shape, (640, 640, 3))
        self.assertEqual((sx, sy, left, top), (1, 1, 0, 140))


if __name__ == "__main__":
    unittest.main()
