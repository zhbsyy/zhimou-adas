"""Synthetic geometry/decision tests, NOT evidence of optical ranging accuracy."""
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from decision_core import DecisionCore
from decision_service import DecisionService
from pedestrian_aeb import PedestrianMonitor, validate_config
from stereo_roi_range import RoiStereoRanger
from stereo_source import PcieStereoSource, validate_pair


def calibration(path):
    K=[[400,0,320],[0,400,180],[0,0,1]];R=np.eye(3).tolist()
    d={'image_size':[640,360],'K1':K,'K2':K,'D1':[0]*5,'D2':[0]*5,'R1':R,'R2':R,
       'P1':[[400,0,320,0],[0,400,180,0],[0,0,1,0]],
       'P2':[[400,0,320,-20000],[0,400,180,0],[0,0,1,0]]}  # translation is mm, as in the legacy calibration
    path.write_text(json.dumps(d),encoding='utf-8');return path


def pair():
    L=np.random.default_rng(8).integers(0,256,(360,640,3),dtype=np.uint8)
    R=np.zeros_like(L);R[:,:-20]=L[:,20:]
    return L,R


def person(height=180,x=320):
    return np.array([[x-25,342-height,x+25,342,.9,0]],np.float32)


class StereoTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.path=calibration(Path(self.temp.name)/'cal.json')
    def tearDown(self):self.temp.cleanup()

    def test_known_disparity_and_lateral_principal_point(self):
        r=RoiStereoRanger(self.path,True);L,R=pair()
        first=r.measure(L,R,person(),['person'],10.)['targets'][0]
        self.assertFalse(first['valid'])
        out=r.measure(L,R,person(),['person'],10.15)['targets'][0]
        self.assertTrue(out['valid'],out)
        self.assertAlmostEqual(out['distance_m'],1.,delta=.02)
        self.assertAlmostEqual(out['lateral_m'],0.,delta=.005)

    def test_unconfirmed_calibration_never_exports_measured_meters(self):
        r=RoiStereoRanger(self.path);L,R=pair()
        r.measure(L,R,person(),['person'],10.)
        out=r.measure(L,R,person(),['person'],10.15)['targets'][0]
        self.assertIsNone(out['distance_m']);self.assertFalse(out['valid'])
        self.assertAlmostEqual(out['candidate_distance_m'],1.,delta=.02)

    def test_replicated_blank_and_wrong_size_rejected(self):
        L,R=pair();r=RoiStereoRanger(self.path)
        for right in (L,np.zeros_like(L),R[:100]):
            self.assertFalse(r.measure(L,right,person(),['person'],10.)['valid'])

    def test_swapped_eyes_and_motion_are_not_valid(self):
        L,R=pair();r=RoiStereoRanger(self.path,True)
        out=r.measure(R,L,person(),['person'],10.)
        self.assertFalse(any(p['valid'] for p in out['targets']))
        r.measure(L,R,person(),['person'],10.15)
        out=r.measure(L,R,person(x=340),['person'],10.30)['targets'][0]
        self.assertFalse(out['valid'])

    def test_old_frame_gap_requires_new_confirmation(self):
        L,R=pair();r=RoiStereoRanger(self.path,True)
        r.measure(L,R,person(),['person'],10.)
        self.assertFalse(r.measure(L,R,person(),['person'],11.)['targets'][0]['valid'])

    def test_capture_pair_owned_together_and_display_only_first_eye(self):
        source=PcieStereoSource('.')
        full=np.zeros((720,1280,3),np.uint8);full[:360,:640]=11;full[:360,640:]=22
        class Raw:
            def read_bgr(self):source.stop_event.set();return full
        source.source=Raw();source._read()
        L,R,seq,_=source.read_new_stereo(0,.01)
        full[:]=0
        self.assertEqual(seq,1);self.assertEqual(L.shape,(360,640,3))
        self.assertEqual(int(L[0,0,0]),11);self.assertEqual(int(R[0,0,0]),22)
        self.assertFalse(L.flags.writeable);self.assertTrue(np.shares_memory(L.base,R.base))


class MonitorTests(unittest.TestCase):
    def make(self,mode='video_demo'):
        return PedestrianMonitor({'enabled':True,'mode':mode})
    def test_video_three_stages_no_metric_distance(self):
        for height,want in ((70,'warning'),(110,'slow'),(180,'emergency')):
            m=self.make();d=person(height)
            m.update(d,['person'],(360,640,3),10.)
            out=m.update(d,['person'],(360,640,3),10.15)
            self.assertEqual(out['level'],want)
            self.assertIsNone(out['distance_m']);self.assertIsNone(out['ttc_s'])
            self.assertFalse(out['metric_distance_used'])
    def test_roadside_nonperson_low_confidence_not_brake(self):
        for d,names in ((person(x=55),['person']),(person(),['car']),
                        (np.array([[290,100,350,340,.3,0]]),['person'])):
            out=self.make().update(d,names,(360,640,3),10.)
            self.assertEqual(out['level'],'clear')
    def test_brake_hold_requires_continuous_fresh_clear(self):
        m=self.make();m.update(person(),['person'],(360,640,3),10.)
        m.update(person(),['person'],(360,640,3),10.15)
        empty=np.empty((0,6))
        for stamp in (10.3,10.6,10.9,11.2):
            self.assertEqual(m.update(empty,['person'],(360,640,3),stamp)['level'],'emergency')
        self.assertEqual(m.update(empty,['person'],(360,640,3),11.6)['level'],'clear')
    def test_missing_updates_do_not_count_as_clear(self):
        m=self.make();m.update(person(),['person'],(360,640,3),10.)
        m.update(person(),['person'],(360,640,3),10.15)
        self.assertEqual(m.update([],['person'],(360,640,3),20.)['level'],'emergency')
    def test_metric_mode_requires_verified_pair_even_with_no_detections(self):
        self.assertFalse(self.make('stereo_distance').update([],['person'],(360,640,3),10.)['valid'])
    def test_metric_threshold_uses_matched_pedestrian_only(self):
        m=self.make('stereo_distance');d=person()
        r={'valid':True,'calibration_confirmed':True,'targets':[{'box':d[0,:4].tolist(),'class_id':0,
              'distance_m':.8,'lateral_m':0.,'valid':True}]}
        m.update(d,['person'],(360,640,3),10.,r)
        out=m.update(d,['person'],(360,640,3),10.15,r)
        self.assertEqual(out['level'],'emergency');self.assertTrue(out['metric_distance_used'])
    def test_bad_thresholds_rejected(self):
        for c in ({'brake_m':10},{'slow_height_ratio':.05},{'ttl_s':float('nan')},{'enabled':'true'}):
            with self.assertRaises(ValueError):validate_config(c)


class ArbitrationTests(unittest.TestCase):
    def setUp(self):
        self.now=100.;self.n=0;self.cmd=0
        self.core=DecisionCore({'alarm_auto_clear_s':3},clock=lambda:self.now,aeb_config={'enabled':True})
        self.epoch=self.core.reset_source('pcie')
    def update(self,level='clear',valid=True):
        self.n+=1;c=self.core;c.note_frame(self.epoch,self.n,self.now)
        c.publish('signal',self.epoch,self.n,self.now,{'valid':True,'color':'green'},0)
        c.publish('obstacles',self.epoch,self.n,self.now,{'level':level,'valid':valid,'mode':'video_demo'})
    def command(self,action):
        self.cmd+=1
        return self.core.command({'client_id':'test','request_id':str(self.cmd),'seq':self.cmd,
            'source_epoch':self.epoch,'action':action})
    def test_aeb_preempts_green_and_cannot_be_cleared_by_buttons(self):
        self.update();self.assertTrue(self.command('move')['ack']['accepted'])
        self.update('emergency')
        for action in ('move','stop','clear_alarm'):
            d=self.command(action)['snapshot']['decision']
            self.assertEqual(d['action'],'stop');self.assertEqual(d['reason_code'],'PEDESTRIAN_AEB')
            self.assertEqual(d['feedback']['turn'],'hazard');self.assertTrue(d['feedback']['brake_lamp'])
        self.now+=4
        self.assertEqual(self.core.snapshot()['decision']['reason_code'],'PEDESTRIAN_AEB')
    def test_clear_does_not_resume_and_expired_obstacles_do_not_add_stop_rule(self):
        self.update('emergency');self.update('clear')
        self.assertEqual(self.core.snapshot()['decision']['action'],'stop')
        self.assertTrue(self.command('move')['ack']['accepted'])
        self.now+=.7
        self.core.note_frame(self.epoch,self.n+1,self.now)
        self.assertTrue(self.command('move')['ack']['accepted'])

    def test_unknown_pedestrian_range_does_not_veto_existing_green_rule(self):
        self.update('unknown',valid=False)
        reply=self.command('move')
        self.assertTrue(reply['ack']['accepted'])
        self.assertEqual(reply['snapshot']['decision']['reason_code'],'GREEN_LIGHT_GO')
        self.update('emergency')
        self.assertFalse(self.command('move')['ack']['accepted'])
    def test_slow_warning_does_not_override_red_light_reason(self):
        self.update('slow')
        self.core.channels['signal']['result']['color']='red'
        d=self.command('move')['snapshot']['decision']
        self.assertEqual(d['action'],'stop');self.assertEqual(d['reason_code'],'RED_LIGHT_MOVE_REQUEST')
    def test_source_epoch_and_context_cannot_remove_emergency(self):
        self.update('emergency')
        body={'client_id':'test','request_id':'ctx','seq':1,'source_epoch':self.epoch,
              'action':'set_context','scenario':'lane_change','lane':'left'}
        self.assertEqual(self.core.command(body)['snapshot']['decision']['aeb']['level'],'emergency')


class ServiceTests(unittest.TestCase):
    def test_actual_async_detector_summary_drives_AEB_without_another_inference(self):
        service=DecisionService({'aeb':{'enabled':True}},evidence_mode='synthetic_test')
        service.start('127.0.0.1',0);epoch=service.begin_source('test')
        try:
            for n in (1,2):
                # Windows monotonic can give both fast synthetic submissions the
                # same clock tick. These are deliberately two different frames.
                stamp=time.monotonic()-(.15 if n==1 else 0)
                packet=service.frame(np.zeros((360,640,3),np.uint8),n,epoch,received_mono=stamp)
                service.submit_detection(packet,person(),['person'])
                deadline=time.monotonic()+1
                while time.monotonic()<deadline:
                    if service.snapshot()['metrics']['processed_collision']>=n:break
                    time.sleep(.01)
            out=service.snapshot()
            self.assertEqual(out['decision']['reason_code'],'PEDESTRIAN_AEB',out)
            self.assertEqual(out['perception']['obstacles']['frame_seq'],2)
            self.assertTrue(out['decision']['simulation_only'])
        finally:service.close()


if __name__=='__main__':unittest.main(verbosity=2)
