import sys
from pathlib import Path
import unittest

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from decision_core import DecisionCore
from detector_thresholds import filter_scores
from perception_summary import SignalSelector


class Clock:
    now = 10.
    def __call__(self): return self.now


class Rules(unittest.TestCase):
    def setUp(self):
        self.clock = Clock();self.core = DecisionCore({'alarm_auto_clear_s': 3}, clock=self.clock)
        self.epoch = self.core.reset_source('pcie');self.seq = 0;self.commands = 0

    def publish(self, color='green', left='solid', right='dashed'):
        self.seq += 1
        self.core.note_frame(self.epoch, self.seq, self.clock.now)
        self.core.publish('signal', self.epoch, self.seq, self.clock.now,
                          {'color': color, 'valid': color != 'unknown'}, self.core.context_revision)
        self.core.publish('boundaries', self.epoch, self.seq, self.clock.now,
                          {s: {'kind': k, 'valid': k != 'unknown'} for s, k in [('left', left), ('right', right)]})

    def command(self, action, **kwargs):
        self.commands += 1
        return self.core.command({'client_id': 'test', 'request_id': 'r'+str(self.commands),
            'seq': self.commands, 'source_epoch': self.epoch, 'action': action, **kwargs})

    def test_red_normal_wait_has_no_alarm_but_forced_move_automatically_alarms(self):
        self.publish('red')
        self.assertIsNone(self.core.snapshot()['decision']['alarm'])
        reply = self.command('move')
        self.assertFalse(reply['ack']['accepted'])
        self.assertEqual(reply['snapshot']['decision']['feedback']['turn'], 'hazard')
        self.assertTrue(reply['snapshot']['decision']['feedback']['body_locked'])

    def test_alarm_expires_without_confirmation_and_never_resumes(self):
        self.publish('red');self.command('move');self.clock.now += 3.1
        self.publish('green')
        decision = self.core.snapshot()['decision']
        self.assertIsNone(decision['alarm']);self.assertEqual(decision['action'], 'stop')

    def test_new_valid_request_recovers_without_clear_alarm(self):
        self.publish('red');self.command('move');self.clock.now += .1;self.publish('green')
        result = self.command('move')
        self.assertTrue(result['ack']['accepted']);self.assertIsNone(result['snapshot']['decision']['alarm'])

    def test_violation_retrigger_extends_alarm_display(self):
        self.publish('red');self.command('move');self.clock.now += 2.8;self.publish('red');self.command('move')
        self.clock.now += .3;self.publish('red')
        self.assertIsNotNone(self.core.snapshot()['decision']['alarm'])

    def test_solid_rejected_dashed_allowed_without_manual_alarm_button(self):
        self.command('set_context', scenario='lane_change', lane='straight');self.publish()
        self.assertFalse(self.command('left')['ack']['accepted'])
        d = self.command('right')['snapshot']['decision']
        self.assertIsNone(d['alarm']);self.assertEqual(d['feedback']['wheel'], 'right')

    def test_unknown_refuses_without_inventing_red_or_solid_violation(self):
        self.publish('unknown')
        reply = self.command('move')
        self.assertEqual(reply['ack']['reason_code'], 'SIGNAL_UNKNOWN')
        self.assertIsNone(reply['snapshot']['decision']['alarm'])

    def test_stop_cancels_alarm_and_keeps_brake(self):
        self.publish('red');self.command('move');d = self.command('stop')['snapshot']['decision']
        self.assertIsNone(d['alarm']);self.assertTrue(d['feedback']['brake_lamp'])


class Perception(unittest.TestCase):
    def test_class10_and_light_thresholds_are_separate(self):
        names = ['car'] * 11;names[9] = 'traffic light';names[10] = 'crosswalk'
        d = np.array([[0,0,10,10,s,c] for s,c in [(.49,10),(.50,10),(.16,9),(.14,9),(.24,0),(.26,0)]], np.float32)
        self.assertEqual(filter_scores(d,names)[:,5].tolist(), [10,9,0])

    def frame(self, hsv, second=None):
        frame = np.full((60,120,3), 12, np.uint8)
        color = cv2.cvtColor(np.uint8([[hsv]]), cv2.COLOR_HSV2BGR)[0,0].tolist()
        cv2.circle(frame,(30,20),3,color,-1)
        dets = [[20,10,42,30,.2,0]]
        if second is not None:
            color = cv2.cvtColor(np.uint8([[second]]), cv2.COLOR_HSV2BGR)[0,0].tolist()
            cv2.circle(frame,(80,20),3,color,-1);dets.append([70,10,92,30,.2,0])
        return frame,np.asarray(dets,np.float32)

    def confirmed(self, frame, dets, config):
        selector = SignalSelector(config)
        selector.update(frame,dets,['traffic light'],10,'straight')
        return selector.update(frame,dets,['traffic light'],10.15,'straight')

    def test_screen_capture_low_saturation_green_recovers(self):
        f,d = self.frame((60,70,110))
        self.assertFalse(self.confirmed(f,d,{})['valid'])
        self.assertEqual(self.confirmed(f,d,{'screen_capture_color':True})['color'],'green')

    def test_blue_white_dark_and_single_pixel_do_not_become_green(self):
        for hsv in [(110,255,245),(0,0,250),(60,255,35)]:
            f,d = self.frame(hsv)
            self.assertFalse(self.confirmed(f,d,{'screen_capture_color':True})['valid'])
        f,d = self.frame((60,255,240));f[:]=12;f[20,30]=(0,240,0)
        self.assertFalse(self.confirmed(f,d,{'screen_capture_color':True})['valid'])

    def test_multiple_confirmed_lights_only_allow_unanimous_colors(self):
        cfg={'screen_capture_color':True,'unanimous_lights':True}
        f,d=self.frame((60,255,240),(60,255,240))
        self.assertFalse(self.confirmed(f,d,{})['valid'])
        self.assertTrue(self.confirmed(f,d,cfg)['valid'])
        f,d=self.frame((60,255,240),(0,255,240))
        self.assertFalse(self.confirmed(f,d,cfg)['valid'])


if __name__=='__main__': unittest.main(verbosity=2)
