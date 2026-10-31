"""Native ROI count correctness and single-copy render ownership, without NPU."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from cached_display import begin_frame, draw_owned, prepare_masks, apply_masks_inplace
from perception_summary import SignalSelector


class PixelRatioTests(unittest.TestCase):
    def selector(self):
        return SignalSelector({'signal_algorithm': 'traffic_light_pixel_ratio_rg'})

    def update(self, image, selector=None, stamp=10, box=None):
        h, w = image.shape[:2]
        box = box or [0, 0, w, h]
        return (selector or self.selector()).update(image,
            np.asarray([box + [.9, 0]], np.float32), ['traffic light'], stamp, 'straight')

    def test_count_not_top_brightness_few_green_pixels_cannot_beat_more_red(self):
        image = np.zeros((20, 20, 3), np.uint8)
        image.reshape(-1, 3)[:100] = [0, 0, 70]
        image.reshape(-1, 3)[100:108] = [0, 255, 0]
        out = self.update(image)
        self.assertEqual(out['color'], 'red')
        diag = out['color_diagnostics']
        self.assertEqual((diag['red_pixels'], diag['green_pixels'], diag['roi_pixels']), (100, 8, 400))
        self.assertAlmostEqual(diag['red_ratio'], .25)
        self.assertAlmostEqual(diag['green_ratio'], .02)
        self.assertAlmostEqual(out['color_probabilities']['red'], 100 / 108)

    def test_more_green_pixels_win_even_if_red_pixels_brighter(self):
        image = np.zeros((20, 20, 3), np.uint8)
        image.reshape(-1, 3)[:100] = [0, 70, 0]
        image.reshape(-1, 3)[100:108] = [0, 0, 255]
        self.assertEqual(self.update(image)['color'], 'green')

    def test_red_wraparound_and_thresholds(self):
        hsv = np.zeros((10, 10, 3), np.uint8)
        hsv[:3] = [179, 255, 100]
        hsv[3:5] = [0, 255, 100]
        hsv[5:7] = [60, 255, 100]
        hsv[7:8] = [60, 0, 255]  # white excluded
        hsv[8:] = [60, 255, 20]  # dark excluded
        out = self.update(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR))
        self.assertEqual(out['color_diagnostics']['red_pixels'], 50)
        self.assertEqual(out['color_diagnostics']['green_pixels'], 20)

    def test_blue_and_gray_are_not_counted_as_green(self):
        for color in ([255, 0, 0], [200, 200, 200], [0, 0, 0]):
            out = self.update(np.full((20, 20, 3), color, np.uint8))
            self.assertEqual(out['color'], 'red')
            self.assertEqual(out['reason'], 'no_red_green_pixels_tie_red')
            self.assertEqual(out['color_diagnostics']['green_pixels'], 0)
            self.assertEqual(out['color_diagnostics']['red_pixels'], 0)

    def test_tie_does_not_retain_previous_green(self):
        selector = self.selector()
        self.assertEqual(self.update(np.full((20, 20, 3), [0, 255, 0], np.uint8), selector)['color'], 'green')
        mixed = np.zeros((20, 20, 3), np.uint8)
        mixed[:5] = [0, 255, 0]
        mixed[5:10] = [0, 0, 255]
        self.assertEqual(self.update(mixed, selector, 10.1)['color'], 'red')
        self.assertEqual(self.update(np.zeros_like(mixed), selector, 10.2)['color'], 'red')

    def test_only_detection_roi_pixels_no_padding(self):
        frame = np.full((20, 40, 3), [0, 0, 255], np.uint8)
        frame[5:15, 10:30] = [0, 255, 0]
        out = self.update(frame, box=[10, 5, 30, 15])
        self.assertEqual(out['color'], 'green')
        self.assertEqual(out['color_diagnostics']['roi_pixels'], 200)
        self.assertEqual(out['color_diagnostics']['red_pixels'], 0)

    def test_native_large_roi_is_not_resized_and_source_is_unchanged(self):
        frame = np.zeros((200, 300, 3), np.uint8)
        frame[10, 20] = [0, 255, 0]
        before = frame.copy()
        out = self.update(frame)
        self.assertEqual(out['color_diagnostics']['green_pixels'], 1)
        self.assertEqual(out['color_diagnostics']['roi_pixels'], 60000)
        self.assertFalse(out['color_diagnostics']['resized'])
        np.testing.assert_array_equal(frame, before)


class RenderCanvasTests(unittest.TestCase):
    def test_pixel_identical_to_original_with_masks_boxes_and_title(self):
        image = np.random.default_rng(2).integers(0, 255, (90, 160, 3), dtype=np.uint8)
        before = image.copy()
        direct = np.zeros((90, 160), np.uint8)
        lane = np.zeros_like(direct)
        direct[30:80, 20:145] = 1
        lane[30:80, 35:37] = 1
        lane[30:80, 120:122] = 2
        masks = prepare_masks((direct, lane))
        dets = np.array([[15, 10, 50, 60, .8, 0], [-4, -3, 10, 15, .5, 0]], np.float32)
        old = image.copy()
        apply_masks_inplace(old, masks)
        expected = draw_owned(old, dets, ['person'], 'test title')
        canvas, frame = begin_frame(image)
        apply_masks_inplace(frame, masks)
        with patch.object(cv2, 'copyMakeBorder', side_effect=AssertionError('unnecessary second whole-frame copy')):
            actual = draw_owned(frame, dets, ['person'], 'test title', canvas=canvas)
        self.assertIs(actual, canvas)
        np.testing.assert_array_equal(actual, expected)
        np.testing.assert_array_equal(image, before)

    def test_repeated_frames_never_accumulate_overlays_or_alias_encoder_input(self):
        direct = np.ones((50, 70), np.uint8)
        prepared = prepare_masks((direct, np.zeros_like(direct)))
        previous = None
        for value in (40, 90, 180, 40) * 5:
            raw = np.full((50, 70, 3), value, np.uint8)
            canvas, frame = begin_frame(raw)
            apply_masks_inplace(frame, prepared)
            out = draw_owned(frame, [], [], '', canvas=canvas)
            expected = cv2.addWeighted(raw, .75, np.full_like(raw, [30, 170, 30]), .25, 0)
            np.testing.assert_array_equal(out[35:], expected)
            if previous:
                np.testing.assert_array_equal(previous[0], previous[1])
            previous = (out, out.copy())

    def test_rejects_foreign_canvas_and_unowned_raw(self):
        raw = np.zeros((50, 70, 3), np.uint8)
        canvas, frame = begin_frame(raw)
        with self.assertRaises(ValueError):
            draw_owned(frame, [], [], '', canvas=canvas.copy())
        with self.assertRaises(ValueError):
            draw_owned(raw[:30], [], [], '')


if __name__ == '__main__':
    unittest.main(verbosity=2)
