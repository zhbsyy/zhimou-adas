"""Fixed-candidate behavior on degraded synthetic ROIs, not road accuracy."""
import math
import unittest
import cv2
import numpy as np
from traffic_light_color import ColorConfig, StateConfig, TrafficLightColorEngine, classify_color, load_config, draw_results
from test_color import scene, detections, BOX_A, BOX_B, NAMES, COLORS


class ForcedColorTests(unittest.TestCase):
    def setUp(self):
        self.cfg = ColorConfig(decision_mode="forced")

    def check_distribution(self, result, colors=("red", "yellow", "green")):
        self.assertEqual(set(result.probabilities), set(colors))
        self.assertAlmostEqual(sum(result.probabilities.values()), 1.0)
        self.assertTrue(all(math.isfinite(v) and 0 <= v <= 1 for v in result.probabilities.values()))
        self.assertIn(result.color, colors)

    def test_three_colors_and_red_wraparound(self):
        for c in ["red", "yellow", "green"]:
            r = classify_color(scene(c), BOX_A, self.cfg)
            self.assertEqual(r.color, c)
            self.check_distribution(r)
        frame = scene("none")
        bgr = cv2.cvtColor(np.uint8([[[179, 255, 200]]]), cv2.COLOR_HSV2BGR)[0, 0]
        cv2.circle(frame, (40, 52), 5, tuple(map(int, bgr)), -1)
        self.assertEqual(classify_color(frame, BOX_A, self.cfg).color, "red")

    def test_dim_and_desaturated_lamps_have_a_vote(self):
        for name, hue in [("red", 0), ("yellow", 25), ("green", 60)]:
            frame = scene("none")
            bgr = cv2.cvtColor(np.uint8([[[hue, 60, 95]]]), cv2.COLOR_HSV2BGR)[0, 0]
            cv2.circle(frame, (40, 52), 4, tuple(map(int, bgr)), -1)
            self.assertEqual(classify_color(frame, BOX_A).color, "unknown")
            r = classify_color(frame, BOX_A, self.cfg)
            self.assertEqual(r.color, name)
            self.assertTrue(r.forced_choice)
            self.check_distribution(r)

    def test_one_pixel_or_tiny_roi_still_selects(self):
        frame = np.zeros((8, 8, 3), np.uint8)
        frame[3, 3] = (0, 80, 0)
        r = classify_color(frame, [3, 3, 4, 4], self.cfg)
        self.assertEqual(r.color, "green")
        self.check_distribution(r)

    def test_blur_resize_and_jpeg_keep_candidates(self):
        for name in ["red", "green", "yellow"]:
            frame = cv2.GaussianBlur(scene(name), (9, 9), 2)
            frame = cv2.resize(cv2.resize(frame, (60, 40)), (240, 160))
            ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 25])
            self.assertTrue(ok)
            frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
            r = classify_color(frame, BOX_A, self.cfg)
            self.assertEqual(r.color, name)
            self.check_distribution(r)

    def test_uniform_black_gray_white_are_explicit_ties(self):
        for value in [0, 100, 255]:
            frame = np.full((16, 16, 3), value, np.uint8)
            r = classify_color(frame, [0, 0, 16, 16], self.cfg)
            self.assertEqual(r.reason, "forced_no_color_evidence")
            self.assertEqual(r.evidence_strength, 0)
            self.assertEqual(r.color, "red")  # configured-order tie, not red evidence
            self.assertTrue(all(abs(p - 1/3) < 1e-10 for p in r.probabilities.values()))

    def test_two_candidates_never_output_yellow(self):
        cfg = ColorConfig(decision_mode="forced", candidates=("red", "green"))
        for name in ["red", "green", "yellow", "white"]:
            r = classify_color(scene(name), BOX_A, cfg)
            self.check_distribution(r, ("red", "green"))
        self.assertEqual(classify_color(scene("green"), BOX_A, cfg).color, "green")

    def test_conflicting_lamps_choose_instead_of_reject(self):
        frame = scene("red")
        cv2.circle(frame, (40, 100), 8, COLORS["green"], -1)
        self.assertEqual(classify_color(frame, BOX_A).color, "unknown")
        self.assertIn(classify_color(frame, BOX_A, self.cfg).color, self.cfg.candidates)

    def test_nearby_color_outside_roi_does_not_vote(self):
        frame = np.zeros((40, 50, 3), np.uint8)
        frame[10:30, 20:30] = (0, 50, 0)
        frame[:, 30:] = (0, 0, 255)
        self.assertEqual(classify_color(frame, [20, 10, 30, 30], self.cfg).color, "green")

    def test_invalid_box_is_not_a_target(self):
        self.assertEqual(classify_color(scene(), [20, 20, 20, 30], self.cfg).reason, "invalid_box")

    def test_first_frame_and_two_targets_no_unknown(self):
        engine = TrafficLightColorEngine(self.cfg)
        first = engine.update(scene(), detections(), NAMES, 0)
        self.assertEqual([r["stable_color"] for r in first], ["red", "green"])
        second = engine.update(scene(), detections(True), NAMES, .1)
        self.assertEqual([r["stable_color"] for r in second], ["green", "red"])
        self.assertEqual(second[0]["track_id"], first[1]["track_id"])
        self.assertTrue(all(r["forced_choice"] for r in second))

    def test_history_tie_stays_per_target_and_uncertainty_increases(self):
        engine = TrafficLightColorEngine(self.cfg)
        a = engine.update(scene("green", "red"), detections(), NAMES, 0)
        b = engine.update(scene("white", "white"), detections(), NAMES, .1)
        self.assertEqual([r["stable_color"] for r in b], ["green", "red"])
        self.assertLess(b[0]["candidate_margin"], a[0]["candidate_margin"])
        self.assertTrue(all(r["evidence_strength"] == 0 for r in b))
        c = engine.update(scene("white", "white"), detections(), NAMES, 2)
        self.assertNotEqual(c[0]["track_id"], a[0]["track_id"])
        self.assertTrue(all(abs(x - 1/3) < 1e-10 for x in c[0]["probabilities"].values()))

    def test_smoothing_then_switch_no_abstention(self):
        engine = TrafficLightColorEngine(self.cfg, StateConfig(force_smoothing_seconds=.15))
        engine.update(scene(), detections(), NAMES, 0)
        same = engine.update(scene("green"), detections(), NAMES, .02)[0]
        later = engine.update(scene("green"), detections(), NAMES, .5)[0]
        self.assertEqual(same["stable_color"], "red")
        self.assertEqual(later["stable_color"], "green")
        self.assertEqual(same["track_id"], later["track_id"])

    def test_no_detection_or_nonlight_does_not_create_color(self):
        engine = TrafficLightColorEngine(self.cfg)
        self.assertEqual(engine.update(scene(), np.empty((0, 6)), NAMES, 0), [])
        car = np.asarray([list(BOX_A) + [.9, 1]])
        self.assertEqual(engine.update(scene(), car, NAMES, .1), [])

    def test_input_unchanged_and_zero_smoothing(self):
        engine = TrafficLightColorEngine(self.cfg, StateConfig(force_smoothing_seconds=0))
        frame = scene();before = frame.copy()
        result = engine.update(frame, detections(), NAMES, 0)
        drawn = draw_results(frame, detections(), NAMES, result)
        np.testing.assert_array_equal(frame, before)
        self.assertFalse(np.array_equal(drawn, before))
        self.assertEqual(engine.update(scene("green"), detections(), NAMES, .01)[0]["stable_color"], "green")

    def test_config_loading_and_validation(self):
        for name in ["hsv_config_forced_ryg.json", "hsv_config_forced_rg.json"]:
            from pathlib import Path
            c, s = load_config(Path(__file__).with_name(name))
            self.assertEqual(c.decision_mode, "forced")
            TrafficLightColorEngine(c, s)
        for cfg in [ColorConfig(decision_mode="bad"), ColorConfig(candidates=("red", "red")),
                    ColorConfig(force_hue_sigma=0), ColorConfig(force_top_fraction=2)]:
            with self.assertRaises(ValueError):TrafficLightColorEngine(cfg)


if __name__ == "__main__":
    unittest.main()
