import copy
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'PC_Windows'),str(ROOT/'PC_Windows/Console_Application'),'D:/CARLA_download/PythonAPI/carla']
from control_guard import ProgressWatch,SCHEMA
from navigation_session import NavigationSession
from video_bridge import choose_url,RAW,PROCESSED,PCIE


class SessionHealthTests(unittest.TestCase):
    def fixture(self):
        now=time.monotonic()
        s=NS(streamer=NS(health=lambda:{'ready':True}),scene_received=now,
             scene={'valid':True,'actor_id':42},rk_received=now,progress=ProgressWatch())
        s.arbiter=NS(state=NS(perception_source='carla'),epoch=3,
            protocol=NS(session_id='a'*32,run_id='b'*32),report_received=now,
            control_report={'evidence':{'capture_fresh':True,'valid_scene':True,'actor_id':42}})
        s.rk={'source':'carla','lane_count':20,'stream_count':20,
            'carla_aeb':{'schema':SCHEMA,'session_id':'a'*32,'run_id':'b'*32,'epoch':3,'det_report_seq':20}}
        old=copy.deepcopy(s.rk)
        old.update(lane_count=19,stream_count=19)
        old['carla_aeb']['det_report_seq']=19
        s.progress.observe(old,now-.1); s.progress.observe(s.rk,now)
        return s,now

    def test_healthy_processed_video_and_aeb_report(self):
        s,now=self.fixture()
        self.assertEqual(NavigationSession.perception_reason(s,now),'')
        self.assertEqual(choose_url(NS(arbiter=s.arbiter,aeb_session=s)),PROCESSED)

    def test_missing_rk_falls_back_to_raw_and_cannot_arm_aeb(self):
        s,now=self.fixture()
        s.rk_received=now-2
        self.assertIn('RK',NavigationSession.perception_reason(s,now))
        self.assertEqual(choose_url(NS(arbiter=s.arbiter,aeb_session=s)),RAW)

    def test_epoch_actor_and_model_stalls_are_rejected(self):
        s,now=self.fixture()
        s.rk['carla_aeb']['epoch']=2
        self.assertIn('RK',NavigationSession.perception_reason(s,now))
        s,now=self.fixture()
        s.arbiter.control_report['evidence']['actor_id']=43
        self.assertIn('同步',NavigationSession.perception_reason(s,now))
        s,now=self.fixture()
        s.progress.progressed['lane']=now-2
        self.assertEqual(NavigationSession.perception_reason(s,now),'')
        self.assertEqual(choose_url(NS(arbiter=s.arbiter,aeb_session=s)),RAW)

    def test_pcie_preview_does_not_masquerade_as_carla_aeb(self):
        s,now=self.fixture()
        s.arbiter.state.perception_source='pcie'
        self.assertEqual(choose_url(NS(arbiter=s.arbiter,aeb_session=s)),PCIE)
        self.assertIn('CARLA感知源',NavigationSession.perception_reason(s,now))
