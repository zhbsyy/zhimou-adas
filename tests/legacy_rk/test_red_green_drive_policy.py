"""Requested red/green forced classification and stationary default-drive policy."""
from pathlib import Path
import copy
import json
import sys
import unittest

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from decision_core import DecisionCore
from decision_service import load_config
from perception_summary import SignalSelector


class ForcedRedGreen(unittest.TestCase):
    def selector(self, **extra):
        return SignalSelector({'signal_algorithm': 'traffic_light_reference_forced_rg', **extra})

    def scene(self, hsv=(60, 255, 245)):
        frame = np.full((100, 160, 3), 12, np.uint8)
        color = cv2.cvtColor(np.uint8([[hsv]]), cv2.COLOR_HSV2BGR)[0, 0]
        cv2.circle(frame, (40, 30), 5, tuple(map(int, color)), -1)
        return frame, np.array([[25, 15, 55, 60, .8, 8]], np.float32)

    def run_selector(self, frame, detections, selector=None, stamp=10):
        names = ['car'] * 11
        names[8] = 'traffic light'
        names[10] = 'crosswalk'
        return (selector or self.selector()).update(frame, detections, names, stamp, 'straight')

    def test_immediate_red_green_wraparound_dim_tiny_and_blue(self):
        for hsv, expected in [((0, 255, 245), 'red'), ((179, 255, 245), 'red'),
                              ((60, 255, 245), 'green'), ((60, 35, 40), 'green'),
                              ((115, 255, 245), 'green')]:
            frame, dets = self.scene(hsv)
            out = self.run_selector(frame, dets)
            self.assertEqual(out['color'], expected)
            self.assertTrue(out['valid'])
            self.assertEqual(set(out['color_probabilities']), {'red', 'green'})
            self.assertAlmostEqual(sum(out['color_probabilities'].values()), 1)
            self.assertEqual(out['color'], max(out['color_probabilities'], key=out['color_probabilities'].get))
        frame, dets = self.scene()
        dets[0, :4] = [40, 30, 41, 31]
        self.assertEqual(self.run_selector(frame, dets)['color'], 'green')

    def test_evidence_free_tie_is_red_with_explicit_equal_weights(self):
        for level in (0, 100, 255):
            frame, dets = self.scene()
            frame[:] = level
            out = self.run_selector(frame, dets)
            self.assertEqual(out['color'], 'red')
            self.assertEqual(out['color_probabilities'], {'red': .5, 'green': .5})
            self.assertEqual(out['reason'], 'forced_no_color_evidence')
            self.assertFalse(out['score_is_probability'])

    def test_switch_has_no_unknown_frame_or_old_color_delay(self):
        selector = self.selector()
        frame, dets = self.scene()
        self.assertEqual(self.run_selector(frame, dets, selector)['color'], 'green')
        red, _ = self.scene((0, 255, 245))
        self.assertEqual(self.run_selector(red, dets, selector, 10.1)['color'], 'red')

    def test_all_boxes_have_color_and_conflicting_heads_have_deterministic_selection(self):
        frame, dets = self.scene()
        cv2.circle(frame, (110, 30), 5, (0, 0, 245), -1)
        dets = np.vstack([dets, [95, 15, 125, 60, .9, 8]])
        out = self.run_selector(frame, dets)
        self.assertEqual(out['color'], 'red')
        self.assertEqual([r['stable_color'] for r in out['candidates']], ['green', 'red'])
        self.assertTrue(all(r['forced_choice'] for r in out['candidates']))

    def test_roi_prefers_relevant_head_but_outside_head_does_not_become_unknown(self):
        frame, dets = self.scene()
        out = self.run_selector(frame, dets, self.selector(signal_rois={'default': [.6, 0, 1, 1]}))
        self.assertEqual(out['color'], 'green')
        self.assertEqual(out['selection_policy'], 'highest_detector_confidence_fallback')

    def test_no_box_invalid_box_and_other_class_do_not_invent_light(self):
        frame, dets = self.scene()
        for rows in ([], [[1, 1, 1, 2, .9, 8]], [[1, 1, 5, 5, .9, 10]],
                     [[float('nan'), 1, 5, 5, .9, 8]]):
            out = self.run_selector(frame, np.asarray(rows, np.float32))
            self.assertFalse(out['valid'])
            self.assertEqual(out['reason'], 'no_related_light')

    def test_original_image_and_detector_boxes_are_unchanged(self):
        frame, dets = self.scene()
        old_frame, old_dets = frame.copy(), dets.copy()
        self.run_selector(frame, dets)
        np.testing.assert_array_equal(frame, old_frame)
        np.testing.assert_array_equal(dets, old_dets)


class DefaultDriving(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        self.n = self.command_seq = 0
        self.config = load_config(ROOT / 'decision_config.json')
        self.core = DecisionCore(self.config['rules'], clock=lambda: self.now,
                                 aeb_config=self.config['aeb'])
        self.epoch = self.core.reset_source('pcie')

    def publish(self, color=None, level='clear', left='solid', right='dashed'):
        self.n += 1
        self.core.note_frame(self.epoch, self.n, self.now)
        if color is not None:
            self.core.publish('signal', self.epoch, self.n, self.now,
                              {'valid': color in ('red', 'green'), 'color': color},
                              self.core.context_revision)
        self.core.publish('obstacles', self.epoch, self.n, self.now,
                          {'valid': True, 'level': level})
        self.core.publish('boundaries', self.epoch, self.n, self.now,
                          {s: {'kind': kind, 'valid': kind in ('solid', 'dashed')}
                           for s, kind in [('left', left), ('right', right)]})

    def command(self, action, **kwargs):
        self.command_seq += 1
        return self.core.command({'client_id': 'policy_test', 'request_id': str(self.command_seq),
            'seq': self.command_seq, 'source_epoch': self.epoch, 'action': action, **kwargs})

    def decision(self):
        return self.core.snapshot()['decision']

    def test_config_explicitly_enables_requested_mode(self):
        self.assertTrue(self.config['rules']['normal_driving_default'])
        self.assertTrue(self.config['rules']['red_only_stop'])
        self.assertEqual(self.config['scene']['signal_algorithm'], 'traffic_light_pixel_ratio_rg')

    def test_live_normal_video_defaults_to_driving_without_light(self):
        self.assertEqual(self.decision()['action'], 'stop')  # no live source yet
        self.publish()
        self.assertEqual(self.decision()['action'], 'move')
        self.assertEqual(self.decision()['reason_code'], 'NO_RED_LIGHT_GO')
        self.assertTrue(self.command('move')['ack']['accepted'])
        self.assertTrue(self.decision()['feedback']['body_locked'])

    def test_no_light_or_expired_light_does_not_veto_forward(self):
        for color in ('unknown', 'green', 'red'):
            self.publish(color)
            self.now += .8
            self.publish()  # new video, no fresh traffic light result
            self.assertTrue(self.command('move')['ack']['accepted'])
            self.assertEqual(self.decision()['action'], 'move')

    def test_normal_driving_does_not_expire_after_three_seconds(self):
        self.publish()
        self.command('move')
        self.now += 20
        self.publish()
        self.assertEqual(self.decision()['action'], 'move')

    def test_red_stops_automatically_without_alarm_and_clear_resumes(self):
        self.publish('red')
        self.assertEqual(self.decision()['reason_code'], 'RED_LIGHT_STOP')
        self.assertIsNone(self.decision()['alarm'])
        self.now += .1
        self.publish('green')
        self.assertEqual(self.decision()['action'], 'move')

    def test_forced_red_request_alarms_and_cannot_bypass_red(self):
        self.publish('red')
        out = self.command('move')
        self.assertFalse(out['ack']['accepted'])
        self.assertEqual(out['snapshot']['decision']['alarm']['code'], 'RED_LIGHT_MOVE_REQUEST')
        self.now += 4
        self.publish('red')
        self.assertEqual(self.decision()['action'], 'stop')
        self.publish('green')
        self.assertEqual(self.decision()['action'], 'move')

    def test_manual_stop_persists_across_new_frames_context_and_emergency_clear(self):
        self.publish()
        self.command('stop')
        self.publish('green')
        self.command('set_context', scenario='lane_change', lane='straight')
        self.publish(level='emergency')
        self.publish(level='clear')
        self.assertEqual(self.decision()['action'], 'stop')
        self.assertTrue(self.command('move')['ack']['accepted'])
        self.assertEqual(self.decision()['action'], 'move')

    def test_aeb_preempts_default_drive_and_remains_latched_until_fresh_clear(self):
        self.publish(level='emergency')
        self.assertEqual(self.decision()['reason_code'], 'PEDESTRIAN_AEB')
        self.assertFalse(self.command('move')['ack']['accepted'])
        self.now += .8
        self.core.note_frame(self.epoch, self.n + 1, self.now)
        self.assertEqual(self.decision()['reason_code'], 'PEDESTRIAN_AEB')
        self.n += 1
        self.publish(level='clear')
        self.assertEqual(self.decision()['action'], 'move')

    def test_three_pedestrian_states_preserved(self):
        for level, action in [('warning', 'move'), ('slow', 'slow'), ('emergency', 'stop')]:
            self.publish(level=level)
            self.assertEqual(self.decision()['action'], action)
            self.assertEqual(self.decision()['aeb']['level'], level)

    def test_solid_rejected_dashed_allowed_and_turn_expires_to_normal_drive(self):
        self.command('set_context', scenario='lane_change', lane='straight')
        self.publish()
        self.assertFalse(self.command('left')['ack']['accepted'])
        self.assertEqual(self.decision()['alarm']['code'], 'SOLID_LINE_CHANGE_REQUEST')
        self.assertTrue(self.command('right')['ack']['accepted'])
        self.assertEqual(self.decision()['feedback']['wheel'], 'right')
        self.now += 4
        self.publish()
        self.assertEqual(self.decision()['action'], 'move')
        self.assertEqual(self.decision()['feedback']['wheel'], 'center')

    def test_unknown_boundary_only_rejects_that_direction_not_forward(self):
        self.command('set_context', scenario='lane_change', lane='straight')
        self.publish(left='unknown')
        self.assertFalse(self.command('left')['ack']['accepted'])
        self.assertEqual(self.decision()['action'], 'move')

    def test_red_applies_in_lane_change_mode_too(self):
        self.command('set_context', scenario='lane_change', lane='straight')
        self.publish('red')
        self.assertFalse(self.command('right')['ack']['accepted'])
        self.assertEqual(self.decision()['action'], 'stop')

    def test_video_failure_still_stops_the_display(self):
        self.publish()
        self.core.source_failed('camera disconnected')
        self.assertEqual(self.decision()['action'], 'stop')
        self.assertFalse(self.command('move')['ack']['accepted'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
