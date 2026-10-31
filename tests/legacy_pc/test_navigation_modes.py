import copy
from collections import deque
from pathlib import Path
import struct
import sys
import time
from types import SimpleNamespace
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'PC_Windows'),str(ROOT/'PC_Windows/Console_Application'),str(ROOT/'Board_RK3568')]
from navigation_policy import DriveIntent,AebFlow
from navigation_arbiter import NavigationArbiter
from bridge.kinematic_backend import KinematicSimulator
from bridge.protocol_5aa5 import CockpitStreamSplitter,crc16_ccitt
from carla_aeb_link import AebPolicy
from control_guard import ProtocolGate,SCHEMA


def nav_bytes(seq=1,active=True):
    payload=struct.pack('<HHIIIHHBB',10001,1,100,153,60,0,20,int(active),0)
    body=struct.pack('<HHHH',1,1,seq,len(payload))+payload
    return b'\x5a\xa5'+body+struct.pack('<H',crc16_ccitt(body))+b'\xbb\xbb'


def arbiter_stub():
    a=NavigationArbiter.__new__(NavigationArbiter)
    a.kinematic=KinematicSimulator()
    a.intent,a.flow=DriveIntent(),AebFlow()
    a.protocol=ProtocolGate('a'*32)
    a.commands,a.serial_frames=deque(),deque()
    a.aeb_enabled,a.epoch=False,0
    a.session=None
    a.control_report,a.report_received={},0.
    a.control_error=''
    a.nav_received,a.nav_active,a.last_nav_sequence=0.,False,None
    a.log=lambda _:None
    a._dispatch_state=lambda:None
    a.serial=SimpleNamespace(is_connected=lambda:True)
    a.carla=SimpleNamespace(connected=True,trajectory_waypoints=[1],current_route_total_m=153.)
    a._last_cockpit_log_time=0.
    a._current_dest_x,a._current_dest_y=98.8,82.9
    return a


class NavigationTests(unittest.TestCase):
    def step(self,flow,intent,*,enabled=True,emergency=False,clear=False,actor=42,seq=1,t=0.,speed=0.,reason=''):
        return flow.step(enabled=enabled,intent=intent,reason=reason,speed=speed,now=t,
            report={'aeb_triggered':emergency,'det_report_seq':seq,
                    'evidence':{'actor_id':42,'path_clear':clear,'clear_actor_id':actor}})

    def test_aeb_off_ignores_rk_absence_and_emergency(self):
        flow,intent=AebFlow(),DriveIntent()
        intent.gear_request('D')
        self.assertTrue(self.step(flow,intent,enabled=False)['drive'])
        self.assertTrue(self.step(flow,intent,enabled=False,emergency=True)['drive'])

    def test_stop_then_confirm_clear_resumes_same_intent(self):
        f,i=AebFlow(),DriveIntent()
        i.gear_request('D')
        self.assertTrue(self.step(f,i,t=0.,speed=10.)['drive'])
        self.assertTrue(self.step(f,i,emergency=True,t=.1,speed=10.)['brake'])
        for n in range(1,8):
            r=self.step(f,i,clear=False,seq=n,t=.2*n)
            self.assertTrue(r['brake'])
        for n in range(1,8):
            r=self.step(f,i,clear=True,seq=20+n,t=2.+.2*n)
        self.assertTrue(r['drive'])
        self.assertEqual(i.gear,'D')
        self.assertTrue(i.running)

    def test_repeated_or_wrong_actor_clear_cannot_release(self):
        for wrong_actor,repeated in [(43,False),(42,True)]:
            f,i=AebFlow(),DriveIntent()
            i.gear_request('D')
            self.step(f,i,emergency=True)
            for n in range(1,12):
                r=self.step(f,i,clear=True,actor=wrong_actor,seq=1 if repeated else n,t=n*.2)
                self.assertFalse(r['drive'])

    def test_manual_pause_blocks_auto_resume_and_passive_android_d(self):
        f,i=AebFlow(),DriveIntent()
        frame={'gear':'D','simulation_running':True}
        i.remote(frame)
        self.step(f,i,emergency=True)
        i.pause()
        self.assertFalse(i.remote(frame))
        for n in range(1,12):
            self.assertFalse(self.step(f,i,clear=True,seq=n,t=n*.2)['drive'])
        i.gear_request('D')
        for n in range(1,9):
            r=self.step(f,i,clear=True,seq=20+n,t=3+n*.2)
        self.assertTrue(r['drive'])

    def test_fault_does_not_resume_on_reconnect_without_explicit_command(self):
        f,i=AebFlow(),DriveIntent()
        i.gear_request('D')
        self.step(f,i)
        self.assertFalse(self.step(f,i,reason='RK超时',t=.1)['drive'])
        self.assertFalse(self.step(f,i,t=.2)['drive'])
        f.explicit_command()
        self.assertTrue(self.step(f,i,t=.3)['drive'])

    def test_navigation_timeout_also_stops_when_aeb_off(self):
        f,i=AebFlow(),DriveIntent()
        i.gear_request('D')
        self.step(f,i,enabled=False)
        self.assertFalse(self.step(f,i,enabled=False,reason='导航超时')['drive'])

    def test_all_three_original_modes_and_flags_remain(self):
        for mode,cockpit,takeover in [('cockpit_sim',True,False),('perception_debug',False,True),('full_ad',True,True)]:
            a=arbiter_stub()
            a._apply_command('mode',mode,False)
            self.assertEqual(a.state.system_mode,mode)
            self.assertEqual(a.state.cockpit_drive_mode,cockpit)
            self.assertEqual(a.state.takeover_enabled,takeover)
            if mode=='perception_debug':
                self.assertTrue(any(x[0]=='destination' for x in a.commands))
                self.assertTrue(any(x[:2]==('gear','D') for x in a.commands))

    def test_mode_cockpit_aeb_off_needs_no_rk(self):
        a=arbiter_stub()
        a.state.cockpit_drive_mode=True
        a.nav_active,a.nav_received=True,time.monotonic()
        self.assertEqual(a.driving_reason(time.monotonic()),'')
        a.aeb_enabled=True
        self.assertIn('感知',a.driving_reason(time.monotonic()))

    def test_existing_kinematic_fallback_without_carla(self):
        a=arbiter_stub()
        a.carla.connected=False
        a._apply_command('destination',(98.8,82.9,'原运动学导航'),False)
        a.state.cockpit_drive_mode=True
        a.nav_active,a.nav_received=True,time.monotonic()
        self.assertEqual(a.navigation_reason(time.monotonic()),'')
        a.aeb_enabled=True
        self.assertEqual(a.navigation_reason(time.monotonic()),'CARLA未连接')

    def test_serial_reconnect_accepts_restarted_sequence_but_keeps_pause(self):
        a=arbiter_stub()
        a.state.cockpit_drive_mode=True
        splitter=CockpitStreamSplitter()
        a._apply_serial_frame(splitter.feed(nav_bytes(1200))[0],time.monotonic())
        a.flow.has_run=True
        a._on_serial_status('断开',False)
        a._apply_serial_frame(*a.serial_frames.popleft())
        self.assertFalse(a.nav_active)
        a._apply_serial_frame(splitter.feed(nav_bytes(1))[0],time.monotonic())
        self.assertEqual(a.last_nav_sequence,1)
        self.assertTrue(a.flow.fault)

    def test_aeb_off_pcie_display_does_not_gate_navigation(self):
        a=arbiter_stub()
        a.state.cockpit_drive_mode=True
        a.nav_active,a.nav_received=True,time.monotonic()
        a.state.perception_source='pcie'
        self.assertEqual(a.driving_reason(time.monotonic()),'')
        a.aeb_enabled=True
        a.session=SimpleNamespace(perception_reason=lambda _:'AEB需要CARLA感知源')
        self.assertEqual(a.driving_reason(time.monotonic()),'AEB需要CARLA感知源')

    def test_parser_crc_fragmentation_and_sequence_rollover(self):
        splitter=CockpitStreamSplitter()
        data=nav_bytes(65535)
        self.assertEqual(splitter.feed(data[:9]),[])
        frames=splitter.feed(data[9:])
        a=arbiter_stub()
        a._apply_serial_frame(frames[0],time.monotonic())
        old=a.nav_received
        a._apply_serial_frame(frames[0],time.monotonic())
        self.assertEqual(a.nav_received,old)
        next_frame=splitter.feed(nav_bytes(0))[0]
        a._apply_serial_frame(next_frame,time.monotonic())
        self.assertEqual(a.last_nav_sequence,0)
        bad=bytearray(nav_bytes(1)); bad[16]^=1
        self.assertEqual(splitter.feed(bad),[])

    def test_cancelled_navigation_and_malformed_json_do_not_arm(self):
        a=arbiter_stub()
        frame=CockpitStreamSplitter().feed(nav_bytes(active=False))[0]
        a._apply_serial_frame(frame,time.monotonic())
        self.assertFalse(a.nav_active)
        a._apply_serial_frame({'action':'SIMULATOR_STATE','simulation_running':'false','gear':'D'},time.monotonic())
        self.assertFalse(a.intent.running)

    def test_protocol_checks_switch_epoch_and_sequence(self):
        gate=ProtocolGate('a'*32); gate.select_run('b'*32); gate.epoch=3
        packet={'schema':SCHEMA,'source':'carla','session_id':'a'*32,'run_id':'b'*32,'epoch':2,
            'det_report_seq':1,'aeb_triggered':False,'brake_pressure':0.,'evidence':{}}
        self.assertIsNone(gate.accept(packet,'192.168.2.3'))
        packet['epoch']=3
        self.assertEqual(gate.accept(packet,'192.168.2.3'),'control')
        self.assertIsNone(gate.accept(packet,'192.168.2.3'))

    def test_cruise_standby_does_not_cancel_or_refresh_guidance(self):
        a=arbiter_stub()
        splitter=CockpitStreamSplitter()
        guide=splitter.feed(nav_bytes(10))[0]
        a._apply_serial_frame(guide,time.monotonic())
        received=a.nav_received
        cruise=dict(guide,amap_key_type=12205,navigation_active=False,sequence=6000)
        a._apply_serial_frame(cruise,time.monotonic())
        self.assertTrue(a.nav_active)
        self.assertEqual(a.nav_received,received)
        self.assertEqual(a.last_nav_sequence,10)
        a._apply_serial_frame(dict(guide,sequence=11),time.monotonic())
        self.assertEqual(a.last_nav_sequence,11)
        a.state.cockpit_drive_mode=True
        self.assertIn('10秒',a.navigation_reason(a.nav_received+10.1))

    def test_real_guidance_cancel_is_not_undone_by_status_heartbeat(self):
        a=arbiter_stub()
        a._apply_serial_frame({'action':'SIMULATOR_STATE','gear':'D','simulation_running':True},time.monotonic())
        a._apply_serial_frame(CockpitStreamSplitter().feed(nav_bytes(1,active=False))[0],time.monotonic())
        a._apply_serial_frame({'action':'SIMULATOR_STATE','gear':'D','simulation_running':True},time.monotonic())
        self.assertFalse(a.nav_active)
        a._apply_serial_frame(CockpitStreamSplitter().feed(nav_bytes(2,active=True))[0],time.monotonic())
        self.assertTrue(a.nav_active)


class BoardPolicyTests(unittest.TestCase):
    def setUp(self):
        self.dets=[[300,150,330,220,.8,0]]
        self.telemetry={'carla_aeb_enabled':True,'perception_source':'carla','takeover_enabled':True,
            'vehicle':{'speed':20},'carla_aeb_scene':{'valid':True,'actor_id':42,
                'range_source':'CARLA_actor_truth','bbox_front':[300,150,330,220],
                'forward_gap_m':5.,'lateral_m':0.,'clear_state':'not_clear','scene_age_ms':20}}

    def test_emergency_requires_visual_and_switch(self):
        policy=AebPolicy()
        self.assertFalse(policy.evaluate(self.dets,self.telemetry,.05)['aeb_triggered'])
        self.assertTrue(policy.evaluate(self.dets,self.telemetry,.05)['aeb_triggered'])
        self.telemetry['carla_aeb_enabled']=False
        self.assertFalse(policy.evaluate(self.dets,self.telemetry,.05)['aeb_triggered'])

    def test_missing_detection_does_not_equal_positive_clear(self):
        p=AebPolicy().evaluate([],self.telemetry,.05)
        self.assertFalse(p['evidence']['path_clear'])
        self.telemetry['carla_aeb_scene'].update(lateral_m=2.5,clear_state='outside')
        p=AebPolicy().evaluate([],self.telemetry,.05)
        self.assertTrue(p['evidence']['path_clear'])
        self.assertEqual(p['evidence']['clear_actor_id'],42)

    def test_another_in_path_person_vetoes_release(self):
        self.telemetry['carla_aeb_scene'].update(lateral_m=2.5,clear_state='outside',bbox_front=[450,140,470,230])
        p=AebPolicy().evaluate(self.dets,self.telemetry,.05)
        self.assertFalse(p['evidence']['path_clear'])

    def test_stale_or_unknown_scene_never_releases(self):
        for changes,age in [({'valid':False},.05),({'scene_age_ms':500},.05),({},.5)]:
            t=copy.deepcopy(self.telemetry)
            t['carla_aeb_scene'].update(clear_state='completed',**changes)
            self.assertFalse(AebPolicy().evaluate([],t,age)['evidence']['path_clear'])
