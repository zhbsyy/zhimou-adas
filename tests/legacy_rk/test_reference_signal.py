from pathlib import Path
import sys
import unittest

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from reference_signal import make_engine
from perception_summary import SignalSelector
from decision_core import DecisionCore

BOX=(20,30,60,130)
NAMES=['traffic light','car']


def scene(hsv=(60,255,255)):
    image=np.full((160,240,3),30,np.uint8)
    image[30:130,20:60]=10
    color=cv2.cvtColor(np.uint8([[hsv]]),cv2.COLOR_HSV2BGR)[0,0]
    cv2.circle(image,(40,52),8,tuple(map(int,color)),-1)
    return image


def dets(box=BOX):
    return np.asarray([list(box)+[.8,0]],np.float32)


class ReferenceTests(unittest.TestCase):
    def confirmed(self, frame, detections=None):
        selector=SignalSelector({'signal_algorithm':'traffic_light_reference_hybrid','unanimous_lights':True})
        d=dets() if detections is None else detections
        selector.update(frame,d,NAMES,10,'straight')
        return selector.update(frame,d,NAMES,10.15,'straight')

    def test_three_bright_colors_and_wraparound(self):
        for hue,expected in [(0,'red'),(179,'red'),(25,'yellow'),(60,'green')]:
            result=self.confirmed(scene((hue,255,245)))
            self.assertEqual(result['color'],expected)
            self.assertTrue(result['valid'])

    def test_dim_desaturated_emitters_use_reference_soft_weights(self):
        for hue,expected in [(0,'red'),(60,'green')]:
            result=self.confirmed(scene((hue,60,95)))
            self.assertEqual(result['color'],expected,result)
            self.assertEqual(result['color_diagnostics']['decision_gate'],'supported_native_lamp')
            self.assertEqual(result['soft_candidate'],expected)
            self.assertAlmostEqual(sum(result['soft_weights'].values()),1)

    def test_evidence_free_forced_tie_does_not_enter_rules(self):
        for value in [0,100,255]:
            result=self.confirmed(np.full((160,240,3),value,np.uint8))
            self.assertFalse(result['valid'])
            self.assertEqual(result['color'],'unknown')
            self.assertEqual(result['soft_candidate'],'red')
            self.assertEqual(result['color_diagnostics']['decision_gate'],'no_or_weak_chroma_evidence')

    def test_blue_is_not_green_even_when_forced_candidate_is_green(self):
        result=self.confirmed(scene((115,255,245)))
        self.assertFalse(result['valid'])
        self.assertEqual(result['soft_candidate'],'green')

    def test_single_pixel_does_not_become_a_confirmed_lamp(self):
        f=np.zeros((160,240,3),np.uint8);f[52,40]=(0,80,0)
        self.assertFalse(self.confirmed(f)['valid'])
        tiny=np.asarray([[40,52,41,53,.9,0]],np.float32)
        self.assertFalse(self.confirmed(f,tiny)['valid'])

    def test_same_lamp_multiple_boxes_are_merged(self):
        d=np.asarray([list(BOX)+[.9,0],[18,28,63,133,.8,0]],np.float32)
        result=self.confirmed(scene(),d)
        self.assertTrue(result['valid'])
        self.assertEqual(result['eligible_lights'],1)
        self.assertTrue(result['lamp_box'])

    def test_clipped_lamp_padding_and_neighbor_exclusion(self):
        self.assertEqual(self.confirmed(scene((0,255,245)),dets((20,53,60,130)))['color'],'red')
        self.assertFalse(self.confirmed(scene(),dets((70,30,110,130)))['valid'])

    def test_matte_yellow_housing_is_not_accepted_by_soft_fallback(self):
        f=scene((0,0,0));cv2.circle(f,(40,52),10,(25,130,150),-1)
        result=self.confirmed(f)
        self.assertFalse(result['valid'])
        self.assertEqual(result['soft_candidate'],'yellow')

    def test_conflicting_red_green_remains_unknown(self):
        f=scene((0,255,255));cv2.circle(f,(40,100),8,(0,255,0),-1)
        result=self.confirmed(f)
        self.assertFalse(result['valid'])
        self.assertEqual(result['color_diagnostics']['decision_gate'],'conflicting_lamps')

    def test_no_color_after_green_has_no_green_hold_for_decision(self):
        engine=make_engine()
        engine.update(scene(),dets(),NAMES,10)
        self.assertEqual(engine.update(scene(),dets(),NAMES,10.1)[0]['stable_color'],'green')
        result=engine.update(scene((0,0,255)),dets(),NAMES,10.2)[0]
        self.assertEqual(result['stable_color'],'unknown')
        self.assertFalse(result['forced_choice'])

    def test_switch_confirmation_cannot_reuse_old_green(self):
        engine=make_engine()
        engine.update(scene(),dets(),NAMES,10)
        engine.update(scene(),dets(),NAMES,10.1)
        self.assertEqual(engine.update(scene((0,255,245)),dets(),NAMES,10.2)[0]['stable_color'],'unknown')
        self.assertEqual(engine.update(scene((0,0,255)),dets(),NAMES,10.25)[0]['stable_color'],'unknown')

    def test_two_independent_targets_and_timestamp_reset(self):
        f=scene((0,60,95));f[30:130,160:200]=10;cv2.circle(f,(180,52),8,(20,95,20),-1)
        d=np.asarray([list(BOX)+[.9,0],[160,30,200,130,.8,0]],np.float32)
        engine=make_engine();a=engine.update(f,d,NAMES,10);b=engine.update(f,d[::-1],NAMES,10.15)
        self.assertEqual([r['stable_color'] for r in b],['green','red'])
        self.assertEqual([r['track_id'] for r in b],[a[1]['track_id'],a[0]['track_id']])
        self.assertEqual(engine.update(f,np.empty((0,6)),NAMES,10.3),[])
        engine.reset();self.assertEqual(engine.update(f,d,NAMES,0)[0]['stable_color'],'unknown')

    def test_real_class_order_and_input_not_modified(self):
        frame=scene();before=frame.copy()
        names=['car']*11;names[8]='traffic light';names[10]='crosswalk'
        d=np.asarray([list(BOX)+[.9,8],list(BOX)+[.99,10]],np.float32)
        engine=make_engine();result=engine.update(frame,d,names,10)
        self.assertEqual(len(result),1)
        np.testing.assert_array_equal(frame,before)

    def test_reference_signal_reaches_same_rule_interface(self):
        result=self.confirmed(scene((60,60,95)))
        core=DecisionCore(clock=lambda:10.15);epoch=core.reset_source('pcie')
        core.note_frame(epoch,1,10.15);core.publish('signal',epoch,1,10.15,result,0)
        cmd={'client_id':'test','request_id':'go','seq':1,'source_epoch':epoch,'action':'move'}
        self.assertTrue(core.command(cmd)['ack']['accepted'])
        result=self.confirmed(scene((0,0,255)));core.note_frame(epoch,2,10.15)
        core.publish('signal',epoch,2,10.15,result,0)
        cmd.update(request_id='white',seq=2)
        self.assertFalse(core.command(cmd)['ack']['accepted'])


if __name__=='__main__':unittest.main(verbosity=2)
