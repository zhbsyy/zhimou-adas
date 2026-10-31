import os
os.environ['QT_QPA_PLATFORM']='offscreen'
from pathlib import Path
import queue
import sys
import threading
import time
import types
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'Decision_Panel'))
import candidate_panel as panel
from PyQt5 import QtWidgets


class Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls): cls.app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_manual_alarm_button_removed_and_rejection_visible(self):
        p=panel.DecisionPanel();self.assertNotIn('clear_alarm',p.buttons)
        now=time.monotonic()
        snapshot={'schema':'rk_decision.v1','source_epoch':'test','perception':{
            'evidence_mode':'live','source_valid':True,'source_age_ms':0,
            'signal':{'fresh':True,'valid':False,'reason':'no_related_light'},'boundaries':{}},
            'decision':{'simulation_only':True,'message':'停止请求','action':'stop','arrow':'none','alarm':None,
            'feedback':{'turn':'off','brake_lamp':True,'wheel':'center','body_locked':True}}}
        p.active=True;p.last_save=now+100
        p.transport=types.SimpleNamespace(state=lambda:(snapshot,now,None,False),events=queue.Queue())
        p.video=types.SimpleNamespace(state=lambda:(None,0,0,None))
        p.last_ack={'ack':{'accepted':False,'message':'相关信号灯未知或已过期','reason_code':'SIGNAL_UNKNOWN'}}
        p.last_ack_at=now;p.refresh()
        self.assertIn('SIGNAL_UNKNOWN',p.banner.text())
        self.assertIn('自动',p.auto_alarm.text())
        p.active=False;p.deleteLater()

    def test_emergency_banner_and_no_manual_test_override(self):
        p=panel.DecisionPanel();now=time.monotonic()
        snapshot={'schema':'rk_decision.v1','source_epoch':'test','perception':{
            'evidence_mode':'live','source_valid':True,'source_age_ms':0,'signal':{},'boundaries':{}},
            'decision':{'simulation_only':True,'message':'前方有行人','action':'stop','arrow':'none',
            'alarm':{'message':'前方有行人，紧急制动'},'aeb':{'level':'emergency','mode':'video_demo','distance_m':None},
            'feedback':{'turn':'hazard','brake_lamp':True,'wheel':'center','body_locked':True}}}
        p.active=True;p.last_save=now+100
        p.transport=types.SimpleNamespace(state=lambda:(snapshot,now,None,False),events=queue.Queue())
        p.video=types.SimpleNamespace(state=lambda:(None,0,0,None));p.refresh()
        self.assertIn('紧急制动',p.banner.text());self.assertNotIn('未测量',p.banner.text())
        self.assertFalse(p.self_test.isEnabled())
        from rk_decision_transport import display_feedback
        feedback,error=display_feedback(snapshot,0.)
        self.assertIsNone(error);self.assertTrue(feedback['aeb_emergency'])
        snapshot['decision']['feedback']['brake_lamp']=False
        self.assertIsNotNone(display_feedback(snapshot,0.)[1])
        p.active=False;p.deleteLater()

    def test_self_test_is_owned_by_worker_and_does_not_mutate_RK_desired(self):
        clock=types.SimpleNamespace(now=10.)
        created=[]
        class Session:
            def __init__(self,*args):
                self.lock=threading.Lock();self.frames={};self.turns=[];self.saved=[];self.closed=False;created.append(self)
            def start(self): pass
            def set_feedback(self,turn,brake): self.turns.append((turn,brake))
            def apply_feedback(self): pass
            def status(self): return {'speed_kmh':0,'max_drift_m':0}
            def save_evidence(self,label): self.saved.append(label)
            def close(self): self.closed=True
        worker=panel.VehicleWorker(ROOT/'Results/Test_Fixtures')
        desired=dict(worker.desired)
        class Stop:
            done=False
            def is_set(self): return self.done
            def wait(self,seconds):
                clock.now+=seconds
                if clock.now>16.4:self.done=True
        worker.stop_event=Stop()
        with patch.object(panel.time,'monotonic',side_effect=lambda:clock.now), patch.dict(sys.modules,{'static_vehicle_session':types.SimpleNamespace(DemoSession=Session)}):
            worker.start_self_test();worker._run()
        session=created[0]
        self.assertIn(('left',False),session.turns);self.assertIn(('right',False),session.turns)
        self.assertIn(('hazard',True),session.turns)
        self.assertEqual(set(session.saved),{'SELF_TEST_left','SELF_TEST_right','SELF_TEST_hazard'})
        self.assertEqual(worker.desired,desired);self.assertTrue(session.closed)
        self.assertIsNone(worker.error)

    def make_slow_panel(self,local_age=0.):
        p=panel.DecisionPanel();now=time.monotonic()
        snapshot={'schema':'rk_decision.v1','source_epoch':'test','perception':{
            'evidence_mode':'live','source_valid':True,'source_age_ms':0,'signal':{},'boundaries':{}},
            'active_aeb':{'ttl_s':.6},'decision':{'simulation_only':True,
            'message':'停止请求；前方有行人','action':'stop','arrow':'none','alarm':None,
            'aeb':{'level':'slow','mode':'video_demo','enabled':True,'fresh':True,'valid':True,'age_ms':0},
            'feedback':{'turn':'off','brake_lamp':True,'wheel':'center','body_locked':True}}}
        p.active=True;p.last_save=now+100
        p.transport=types.SimpleNamespace(state=lambda:(snapshot,now-local_age,None,False),events=queue.Queue())
        p.video=types.SimpleNamespace(state=lambda:(None,0,0,None))
        return p,snapshot,now

    def test_slow_notice_survives_stop_and_recent_rejection_without_moving_car(self):
        p,s,now=self.make_slow_panel()
        desired=[]
        p.vehicle=types.SimpleNamespace(set_feedback=lambda value:desired.append(value),state=lambda:({}, {},None,False))
        p.last_ack={'ack':{'accepted':False,'message':'相关信号灯未知','reason_code':'SIGNAL_UNKNOWN'}}
        p.last_ack_at=now;p.refresh()
        self.assertIn('正在减速（模拟）',p.banner.text())
        self.assertIn('SIGNAL_UNKNOWN',p.banner.text())
        self.assertIn('stereo_console.v0.1.5',p.details.text())
        self.assertTrue(desired[-1]['body_locked']);self.assertTrue(desired[-1]['brake_lamp'])
        self.assertEqual(s['decision']['action'],'stop')
        p.active=False;p.deleteLater()

    def test_expired_slow_sample_does_not_claim_current_deceleration(self):
        p,_,_=self.make_slow_panel(local_age=.7);p.refresh()
        self.assertNotIn('正在减速',p.banner.text())
        from rk_decision_transport import display_feedback
        self.assertIsNone(display_feedback(p.transport.state()[0],.7)[1])
        p.active=False;p.deleteLater()

    def test_clear_and_unknown_hide_pedestrian_row_without_adding_brake(self):
        from rk_decision_transport import display_feedback
        for level in ('clear','unknown'):
            p,s,_=self.make_slow_panel()
            s['decision']['aeb'].update(level=level,valid=level=='clear')
            s['decision'].update(action='move',arrow='straight',message='绿灯允许模拟通行')
            s['decision']['feedback']['brake_lamp']=False
            p.refresh()
            self.assertFalse(any(label.text().startswith('行人 / AEB：') for label in p.findChildren(QtWidgets.QLabel)))
            f,error=display_feedback(s,0.)
            self.assertIsNone(error);self.assertFalse(f['brake_lamp']);self.assertEqual(f['turn'],'off')
            self.assertNotIn('未检测',p.banner.text());self.assertNotIn('距离',p.banner.text())
            p.active=False;p.deleteLater()

    def test_three_pedestrian_levels_light_brakes_without_automatic_hazards(self):
        from rk_decision_transport import display_feedback
        for level in ('warning','slow','emergency'):
            p,s,_=self.make_slow_panel()
            d=s['decision'];d['aeb']['level']=level
            d.update(action='stop' if level=='emergency' else 'move',reason_code='STOP_REQUEST')
            d['feedback'].update(turn='hazard' if level=='emergency' else 'off',brake_lamp=level=='emergency')
            result,error=display_feedback(s,0.)
            self.assertIsNone(error);self.assertTrue(result['brake_lamp'])
            self.assertEqual(result['turn'],'off');self.assertEqual(result['wheel'],'center')
            self.assertEqual(result['reaction'],level);self.assertTrue(result['body_locked'])
            p.deleteLater()

    def test_idle_and_explicit_stop_have_separate_brake_lamp_feedback(self):
        from rk_decision_transport import display_feedback
        p,s,_=self.make_slow_panel();s['decision']['aeb']['level']='clear'
        s['decision']['reason_code']='STOP_REQUEST'
        idle,error=display_feedback(s,0.)
        self.assertIsNone(error);self.assertFalse(idle['brake_lamp']);self.assertEqual(idle['turn'],'off')
        s['last_ack']={'accepted':True,'reason_code':'STOP_REQUEST'}
        stopped,error=display_feedback(s,0.)
        self.assertIsNone(error);self.assertTrue(stopped['brake_lamp']);self.assertEqual(stopped['turn'],'off')
        p.deleteLater()

    def test_rule_alarm_and_lost_connection_do_not_blink_both_indicators(self):
        from rk_decision_transport import display_feedback
        p,s,_=self.make_slow_panel();d=s['decision'];d['aeb']['level']='clear'
        d['alarm']={'code':'RED_LIGHT_MOVE_REQUEST'};d['feedback'].update(turn='hazard',brake_lamp=True)
        for snapshot,age in ((s,0.),(None,10.)):
            result,_=display_feedback(snapshot,age)
            self.assertEqual(result['turn'],'off');self.assertTrue(result['brake_lamp'])
            self.assertTrue(result['body_locked'])
        p.deleteLater()

    def test_valid_turns_still_blink_the_requested_side_only(self):
        from rk_decision_transport import display_feedback
        for turn in ('left','right'):
            p,s,_=self.make_slow_panel();d=s['decision'];d['aeb']['level']='clear'
            d.update(action=turn,reason_code='DASHED_LINE_ALLOW')
            d['feedback'].update(turn=turn,wheel=turn,brake_lamp=False)
            result,error=display_feedback(s,0.)
            self.assertIsNone(error);self.assertEqual(result['turn'],turn)
            self.assertEqual(result['wheel'],turn);self.assertFalse(result['brake_lamp'])
            p.deleteLater()


if __name__=='__main__':unittest.main(verbosity=2)
