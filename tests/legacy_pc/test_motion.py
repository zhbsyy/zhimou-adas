from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'PC_Windows'))
import carla
from navigation_motion import speed_plan, longitudinal_command, preferred_cockpit_port
from pedestrian_crossing import route_candidates, should_start_crossing


def path(n=100):
    return [NS(transform=carla.Transform(carla.Location(i*1.8,0,0),carla.Rotation()),
               is_junction=False, road_id=1, lane_id=-1, lane_width=3.5) for i in range(n)]


class MotionTests(unittest.TestCase):
    def test_disable_aeb_preserves_uncapped_navigation(self):
        self.assertEqual(speed_plan(None, False), (None, ''))

    def test_approach_and_turn_caps_never_increase_cruise(self):
        p=path()
        b=NS(trajectory_waypoints=p,last_matched_idx=0,_scene_site=None)
        self.assertEqual(speed_plan(b,True)[0],40)
        b._scene_site={'route_arc':[i*1.8 for i in range(100)],'route_s':40.}
        b._scene_crossing_done=False
        self.assertGreater(speed_plan(b,True)[0],12)
        b._scene_site['route_s']=18.
        self.assertEqual(speed_plan(b,True)[0],12)
        b._scene_site=None
        p[8].is_junction=True
        self.assertEqual(speed_plan(b,True)[0],40)
        p[8].transform.rotation.yaw=90
        self.assertAlmostEqual(speed_plan(b,True)[0],15)

    def test_brake_demand_cuts_residual_throttle_immediately(self):
        throttle, brake=longitudinal_command(38,12,.02,.7,0.)
        self.assertEqual(throttle,0)
        self.assertGreater(brake,0)

    def test_pedals_are_mutually_exclusive_under_jitter(self):
        throttle,brake=0.,0.
        for n in range(300):
            throttle,brake=longitudinal_command(10+n%13,12,(.02,.05,.12)[n%3],throttle,brake)
            self.assertFalse(throttle>0 and brake>0)
            self.assertTrue(0<=throttle<=.5 and 0<=brake<=1)

    def test_acceleration_is_limited_by_elapsed_time(self):
        self.assertLess(longitudinal_command(0,20,.02,0,0)[0], .02)
        self.assertAlmostEqual(longitudinal_command(0,20,.2,0,0)[0], .11)

    def test_short_straight_after_turn_is_not_a_crossing_site(self):
        p=path(60)
        for wp in p[:15]:
            wp.is_junction=True
            wp.transform.rotation.yaw=90
        b=NS(trajectory_waypoints=p,last_matched_idx=0,vehicle=NS(get_location=lambda:carla.Location()))
        candidates=route_candidates(b,p[19])
        self.assertTrue(candidates)
        self.assertGreaterEqual(candidates[0][0],23)

    def test_entry_requires_lane_center_and_speed_window(self):
        site={'center':carla.Location(12.5,0,0),'road_yaw':0.}
        pose=carla.Transform()
        for speed in (0.,3.,20.,38.):
            self.assertFalse(should_start_crossing(site,pose,None,speed,2.4))
        self.assertTrue(should_start_crossing(site,pose,None,12.,2.4))
        pose.location.y=2.
        self.assertFalse(should_start_crossing(site,pose,None,12.,2.4))

    def test_com12_selection_never_uses_fpga_or_virtual_port(self):
        ports=[NS(device='COM8',description='USB Serial FTDI'),NS(device='COM9',description='USB Serial FTDI')]
        self.assertIsNone(preferred_cockpit_port(ports))
        ports.append(NS(device='COM12',description='USB-SERIAL CH340K'))
        self.assertEqual(preferred_cockpit_port(ports),'COM12')

    def test_automatic_crossing_is_only_in_route_middle(self):
        p=path()
        b=NS(trajectory_waypoints=p,last_matched_idx=0)
        candidates=route_candidates(b,p[5])
        self.assertTrue(candidates)
        self.assertTrue(all(1/3<=i/(len(p)-1)<=2/3 for i,_ in candidates))
        self.assertLess(abs(candidates[0][0]/(len(p)-1)-.5),.02)
