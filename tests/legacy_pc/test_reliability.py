from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace as NS
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'PC_Windows'),str(ROOT/'PC_Windows/Console_Application')]
import carla
from navigation_policy import AebFlow,DriveIntent
from navigation_session import NavigationSession
from pedestrian_crossing import crossing_geometry,route_candidates,should_start_crossing


def report(seq,emergency=False,clear=False):
    return {'det_report_seq':seq,'aeb_triggered':emergency,
        'evidence':{'actor_id':42,'clear_actor_id':42,'path_clear':clear}}


class LagRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.flow,self.intent=AebFlow(),DriveIntent()
        self.intent.gear_request('D')
        self.step(0.,1)

    def step(self,t,seq,reason='',transient=False,emergency=False,clear=False):
        return self.flow.step(enabled=True,intent=self.intent,reason=reason,transient=transient,
            report=report(seq,emergency,clear),speed=0.,now=t)

    def test_short_lag_brakes_then_recovers_after_fresh_reports(self):
        self.assertTrue(self.step(.1,2,'场景短时过期',True)['brake'])
        self.assertFalse(self.flow.fault)
        for t,seq in [(.2,3),(.4,4),(.6,5)]:
            self.assertFalse(self.step(t,seq)['drive'])
        self.assertTrue(self.step(.8,6)['drive'])

    def test_sustained_loss_requires_explicit_resume(self):
        self.step(.1,2,'RK失联',True)
        self.assertEqual(self.step(2.2,2,'RK失联',True)['phase'],'fault')
        self.assertFalse(self.step(2.3,3)['drive'])
        self.flow.explicit_command()
        self.assertTrue(self.step(2.4,4)['drive'])

    def test_manual_pause_during_lag_stays_paused(self):
        self.step(.1,2,'短时过期',True)
        self.intent.pause()
        for n in range(3,12):
            self.assertFalse(self.step(n*.2,n)['drive'])

    def test_duplicate_fresh_report_cannot_resume(self):
        self.step(.1,2,'短时过期',True)
        for n in range(1,10):
            self.assertFalse(self.step(.2+n*.2,3)['drive'])

    def test_real_aeb_latch_survives_temporary_data_loss(self):
        self.step(.1,2,emergency=True)
        self.step(.2,3,'短时过期',True)
        for n in range(4,15):
            r=self.step(n*.2,n)
            self.assertFalse(r['drive'])
        for n in range(15,24):
            r=self.step(n*.2,n,clear=True)
        self.assertTrue(r['drive'])


class SceneProducerTests(unittest.TestCase):
    def test_control_lock_cannot_starve_scene_producer(self):
        lock=threading.Lock(); lock.acquire()
        s=NS(operation_lock=lock,stopping=threading.Event(),scene_record=({'valid':False},0.),
             scene_max_sample_ms=0.,update_scene=lambda:{'valid':True,'carla_frame':1})
        thread=threading.Thread(target=lambda:NavigationSession.poll_scene(s)); thread.start()
        try:
            deadline=time.monotonic()+.3
            while not s.scene_record[0].get('valid') and time.monotonic()<deadline:
                time.sleep(.005)
            self.assertTrue(s.scene_record[0]['valid'])
        finally:
            s.stopping.set(); thread.join(1); lock.release()

    def test_repeated_frozen_world_frame_does_not_refresh_age(self):
        old=time.monotonic()-1.
        stopping=threading.Event()
        def once():
            stopping.set()
            return {'valid':True,'carla_frame':7}
        s=NS(stopping=stopping,scene_record=({'valid':True,'carla_frame':7},old),
             scene_max_sample_ms=0.,update_scene=once)
        NavigationSession.poll_scene(s)
        self.assertEqual(s.scene_record[1],old)

    def test_new_route_without_pedestrian_forgets_previous_actor(self):
        backend=NS(connected=True,vehicle=None,_scene_revision=2,_scene_walker=None,_scene_crossing_done=False,
                   world=NS(get_snapshot=lambda:NS(frame=100)))
        s=NS(arbiter=NS(carla=backend),streamer=NS(health=lambda:{'ready':True}),scene_revision=1,
             last_actor=42,last_geometry={'forward_gap_m':5,'lateral_m':0})
        value=NavigationSession.update_scene(s)
        self.assertTrue(value['valid'])
        self.assertIsNone(value['actor_id'])
        self.assertEqual(value['clear_state'],'no_target')


class CrossingGeometryTests(unittest.TestCase):
    def waypoint(self):
        return NS(transform=carla.Transform(carla.Location(0,0,0),carla.Rotation()),lane_width=3.5)

    def world(self,blocked=False):
        return NS(cast_ray=lambda a,b:[NS(label=carla.CityObjectLabel.Buildings)] if blocked else [],
                  ground_projection=lambda p,d:NS(label=carla.CityObjectLabel.Roads,location=carla.Location(p.x,p.y,0)))

    def test_building_across_path_is_rejected(self):
        site,reason=crossing_geometry(self.world(True),self.waypoint())
        self.assertIsNone(site)
        self.assertEqual(reason,'obstacle_in_crossing')

    def test_open_lane_has_two_clear_endpoints(self):
        site,reason=crossing_geometry(self.world(),self.waypoint())
        self.assertFalse(reason)
        self.assertGreater(site['start'].y,2.2)
        self.assertLess(site['target'].y,-2.2)

    def test_high_kerb_or_unknown_ground_is_rejected(self):
        world=self.world()
        world.ground_projection=lambda p,d:NS(label=carla.CityObjectLabel.Sidewalks,location=carla.Location(p.x,p.y,1.))
        self.assertIsNone(crossing_geometry(world,self.waypoint())[0])

    def test_other_street_cannot_start_crossing(self):
        site={'road_yaw':0.,'span':2.2,'walk_speed':1.}
        pose=carla.Transform(carla.Location(0,0,0),carla.Rotation(yaw=0))
        self.assertFalse(should_start_crossing(site,pose,carla.Location(10,30,0),30.,2.4))
        pose.rotation.yaw=90
        self.assertFalse(should_start_crossing(site,pose,carla.Location(0,10,0),30.,2.4))

    def test_parked_car_does_not_send_pedestrian_away(self):
        site={'road_yaw':0.,'span':2.2,'walk_speed':1.}
        pose=carla.Transform()
        self.assertFalse(should_start_crossing(site,pose,carla.Location(8,2.2,0),0.,2.4))

    def test_only_low_speed_aligned_entry_window_can_start(self):
        site={'road_yaw':0.,'span':2.2,'walk_speed':1.,'center':carla.Location(12.5,0,0)}
        pose=carla.Transform()
        self.assertFalse(should_start_crossing(site,pose,carla.Location(12.5,2.2,0),38.,2.4))
        self.assertTrue(should_start_crossing(site,pose,carla.Location(12.5,2.2,0),12.,2.4))
        pose.location.x=6
        self.assertFalse(should_start_crossing(site,pose,carla.Location(12.5,2.2,0),12.,2.4))
