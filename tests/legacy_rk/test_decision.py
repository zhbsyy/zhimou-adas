import copy
import json
from pathlib import Path
import sys
import time
import unittest
import urllib.error
import urllib.request

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from decision_core import CommandError, DecisionCore
from decision_service import DecisionService, load_config
from perception_summary import BoundaryTracker, SignalSelector, extract_boundaries
from run_fixture import make_frame


class Clock:
    def __init__(self):
        self.now = 100.0
    def __call__(self):
        return self.now


class RuleTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.core = DecisionCore(clock=self.clock)
        self.epoch = self.core.reset_source("video")
        self.seq = 0
        self.frame = 0

    def publish(self, color="green", left="solid", right="dashed"):
        self.frame += 1
        self.core.note_frame(self.epoch, self.frame, self.clock.now)
        self.core.publish("signal", self.epoch, self.frame, self.clock.now,
                          {"color": color, "valid": color != "unknown"}, self.core.context_revision)
        self.core.publish("boundaries", self.epoch, self.frame, self.clock.now,
                          {side:{"kind":kind,"valid":kind in ("solid","dashed")}
                           for side,kind in (("left",left),("right",right))})

    def command(self, action, **kwargs):
        self.seq += 1
        body = {"client_id":"test", "request_id":"r"+str(self.seq), "seq":self.seq,
                "source_epoch":self.epoch,"action":action, **kwargs}
        return self.core.command(body)

    def lane_mode(self):
        self.command("set_context", scenario="lane_change", lane="straight")
        self.publish()

    def test_red_request_alarm_and_zero_actuation(self):
        self.publish("red")
        r = self.command("move")
        self.assertFalse(r["ack"]["accepted"])
        d = r["snapshot"]["decision"]
        self.assertEqual(d["alarm"]["code"], "RED_LIGHT_MOVE_REQUEST")
        self.assertEqual(d["feedback"], {"turn":"hazard","brake_lamp":True,"wheel":"center","body_locked":True})

    def test_ordinary_red_wait_does_not_alarm(self):
        self.publish("red")
        self.assertIsNone(self.core.snapshot()["decision"]["alarm"])

    def test_green_allowed_and_selected_lane_arrow(self):
        self.command("set_context", scenario="intersection", lane="left")
        self.publish()
        d = self.command("move")["snapshot"]["decision"]
        self.assertEqual(d["arrow"], "left")
        self.assertFalse(d["feedback"]["brake_lamp"])

    def test_green_to_red_stops_without_false_forced_alarm(self):
        self.publish()
        self.command("move")
        self.clock.now += .15
        self.publish("red")
        d = self.core.snapshot()["decision"]
        self.assertEqual(d["action"],"stop")
        self.assertIsNone(d["alarm"])
        self.assertEqual(d["reason_code"],"RED_LIGHT_STOP")

    def test_signal_expiry_cancels_go_and_does_not_resume(self):
        self.publish()
        self.command("move")
        self.clock.now += .8
        self.core.note_frame(self.epoch, 2, self.clock.now)
        self.assertEqual(self.core.snapshot()["decision"]["action"],"stop")
        self.publish()
        self.assertEqual(self.core.snapshot()["decision"]["action"],"stop")

    def test_source_stall_cancels_go(self):
        self.publish()
        self.command("move")
        self.clock.now += 1.1
        s = self.core.snapshot()
        self.assertFalse(s["perception"]["source_valid"])
        self.assertEqual(s["decision"]["action"],"stop")

    def test_solid_left_dashed_right(self):
        self.lane_mode()
        self.assertFalse(self.command("left")["ack"]["accepted"])
        self.command("clear_alarm")
        r = self.command("right")
        self.assertTrue(r["ack"]["accepted"])
        self.assertEqual(r["snapshot"]["decision"]["feedback"]["wheel"],"right")

    def test_dashed_left_solid_right(self):
        self.lane_mode()
        self.publish(left="dashed",right="solid")
        self.assertTrue(self.command("left")["ack"]["accepted"])
        self.assertFalse(self.command("right")["ack"]["accepted"])

    def test_unknown_boundary_blocks_without_inventing_violation(self):
        self.lane_mode()
        self.publish(left="unknown")
        r = self.command("left")
        self.assertFalse(r["ack"]["accepted"])
        self.assertIsNone(r["snapshot"]["decision"]["alarm"])

    def test_unknown_signal_and_yellow_wait(self):
        for color in ("unknown","yellow"):
            self.publish(color)
            self.assertFalse(self.command("move")["ack"]["accepted"])

    def test_alarm_clear_is_not_permission(self):
        self.publish("red")
        self.command("move")
        d = self.command("clear_alarm")["snapshot"]["decision"]
        self.assertIsNone(d["alarm"])
        self.assertEqual(d["action"],"stop")
        self.assertFalse(self.command("move")["ack"]["accepted"])

    def test_duplicate_command_does_not_renew_lease(self):
        self.publish()
        body = {"client_id":"test","request_id":"same","seq":1,"source_epoch":self.epoch,"action":"move"}
        self.core.command(body)
        self.clock.now += 2.9
        self.publish()
        self.assertTrue(self.core.command(body)["ack"]["duplicate"])
        self.clock.now += .2
        self.assertEqual(self.core.snapshot()["decision"]["action"],"stop")

    def test_duplicate_id_conflicting_payload_and_reordering(self):
        self.publish()
        self.command("stop")
        for body in (
            {"client_id":"test","request_id":"r1","seq":1,"source_epoch":self.epoch,"action":"move"},
            {"client_id":"test","request_id":"old","seq":1,"source_epoch":self.epoch,"action":"stop"},
        ):
            with self.assertRaises(CommandError):
                self.core.command(body)

    def test_old_epoch_cannot_publish_or_command(self):
        self.publish()
        old = self.epoch
        self.epoch = self.core.reset_source("usb")
        self.assertFalse(self.core.publish("signal",old,2,self.clock.now,{"color":"green","valid":True},0))
        with self.assertRaises(CommandError):
            self.core.command({"client_id":"test","request_id":"old","seq":1,"source_epoch":old,"action":"move"})

    def test_old_context_results_and_duplicate_frames_rejected(self):
        self.publish()
        rev = self.core.context_revision
        self.command("set_context",scenario="intersection",lane="right")
        self.assertFalse(self.core.publish("signal",self.epoch,1,self.clock.now,{"color":"green","valid":True},rev))
        self.assertFalse(self.core.note_frame(self.epoch,1,self.clock.now))
        self.assertFalse(self.command("move")["ack"]["accepted"])

    def test_frame_timestamp_not_completion_defines_freshness(self):
        self.publish()
        old = self.clock.now
        self.clock.now += .9
        self.core.note_frame(self.epoch,2,self.clock.now)
        self.core.publish("signal",self.epoch,2,old,{"color":"green","valid":True},0)
        self.assertFalse(self.command("move")["ack"]["accepted"])

    def test_bad_commands_and_nan(self):
        for action, kwargs in (("invalid",{}),("set_context",{"scenario":"other","lane":"left"}),
                               ("move",{"seq":True}),("move",{"extra":1})):
            with self.assertRaises(CommandError):
                self.command(action,**kwargs)
        self.assertFalse(self.core.note_frame(self.epoch,1,float("nan")))

    def test_lane_change_straight_is_not_red_light_rule(self):
        self.lane_mode()
        self.publish("red")
        self.assertTrue(self.command("move")["ack"]["accepted"])


class SummaryTests(unittest.TestCase):
    def test_nearest_boundaries_not_global_majority(self):
        _,_,masks=make_frame("red",("solid","dashed"))
        masks[1][:,0:90] = 2
        masks[1][:,360:] = 1
        r = extract_boundaries(*masks)
        self.assertEqual(r["left"]["kind"],"solid")
        self.assertEqual(r["right"]["kind"],"dashed")

    def test_swapped_sides_and_no_road(self):
        _,_,masks=make_frame("red",("dashed","solid"))
        r=extract_boundaries(*masks)
        self.assertEqual((r["left"]["kind"],r["right"]["kind"]),("dashed","solid"))
        masks[0][:]=0
        self.assertFalse(extract_boundaries(*masks)["left"]["valid"])

    def test_mixed_and_missing_boundary(self):
        _,_,masks=make_frame("red",("solid","dashed"))
        masks[1][190:,125:129] = 2
        masks[1][:,319:323] = 0
        r=extract_boundaries(*masks)
        self.assertFalse(r["left"]["valid"])
        self.assertFalse(r["right"]["valid"])

    def test_confirmation_resets_on_opposite_evidence(self):
        _,_,masks=make_frame("red",("solid","dashed"))
        r=extract_boundaries(*masks)
        t=BoundaryTracker()
        self.assertFalse(t.update(r,1,10)["left"]["valid"])
        self.assertTrue(t.update(r,2,10.15)["left"]["valid"])
        changed=copy.deepcopy(r);changed["left"]["kind"]="dashed"
        self.assertFalse(t.update(changed,3,10.3)["left"]["valid"])
        self.assertTrue(t.update(changed,4,10.45)["left"]["valid"])

    def test_red_green_unknown_and_multiple_targets(self):
        selector=SignalSelector()
        frame,dets,_=make_frame("red",("solid","dashed"))
        selector.update(frame,dets,["traffic light"],10,"straight")
        r=selector.update(frame,dets,["traffic light"],10.15,"straight")
        self.assertEqual(r["color"],"red")
        green,dets,_=make_frame("green",("solid","dashed"))
        self.assertEqual(selector.update(green,dets,["traffic light"],10.3,"straight")["color"],"unknown")
        self.assertEqual(selector.update(green,dets,["traffic light"],10.45,"straight")["color"],"green")
        self.assertFalse(selector.update(green,np.empty((0,6)),["traffic light"],10.6,"straight")["valid"])
        self.assertFalse(selector.update(green,np.vstack([dets,dets]),["traffic light"],10.75,"straight")["valid"])

    def test_lane_specific_signal_roi(self):
        selector=SignalSelector({"signal_rois":{"left":[0,0,.3,1]}})
        frame,dets,_=make_frame("green",("solid","dashed"))
        self.assertFalse(selector.update(frame,dets,["traffic light"],10,"left")["valid"])


class HttpTests(unittest.TestCase):
    def setUp(self):
        self.service=DecisionService(evidence_mode="synthetic_test")
        host,port=self.service.start("127.0.0.1",0)
        self.url="http://%s:%s"%(host,port)
        self.epoch=self.service.begin_source("test")

    def tearDown(self):
        self.service.close()

    def get(self,path):
        with urllib.request.urlopen(self.url+path,timeout=2) as r:
            return json.load(r)

    def post(self,body):
        req=urllib.request.Request(self.url+"/command",data=json.dumps(body).encode(),headers={"Content-Type":"application/json"})
        with urllib.request.urlopen(req,timeout=2) as r:
            return json.load(r)

    def test_http_real_summary_to_rule_and_duplicate(self):
        frame,dets,masks=make_frame("red",("solid","dashed"))
        for i in range(1,4):
            packet=self.service.frame(frame,i,self.epoch)
            self.service.submit_detection(packet,dets,["traffic light"])
            self.service.submit_lane(packet,masks)
            time.sleep(.10)
        s=self.get("/snapshot")
        self.assertEqual(s["perception"]["signal"]["color"],"red")
        self.assertEqual(s["perception"]["boundaries"]["left"]["kind"],"solid")
        body={"client_id":"http","request_id":"one","seq":1,"source_epoch":self.epoch,"action":"move"}
        self.assertFalse(self.post(body)["ack"]["accepted"])
        self.assertTrue(self.post(body)["ack"]["duplicate"])
        self.assertEqual(self.get("/decision")["decision"]["alarm"]["code"],"RED_LIGHT_MOVE_REQUEST")
        self.assertTrue(self.get("/health")["perception_valid"])

    def test_oversized_bad_json_and_stale_epoch(self):
        for data in (b'{"x": NaN}', b'x'*4100):
            req=urllib.request.Request(self.url+"/command",data=data,headers={"Content-Type":"application/json"})
            with self.assertRaises(urllib.error.HTTPError) as err:
                urllib.request.urlopen(req,timeout=2)
            self.assertEqual(err.exception.code,400)
        body={"client_id":"http","request_id":"old","seq":1,"source_epoch":"bad","action":"move"}
        with self.assertRaises(urllib.error.HTTPError) as err:
            self.post(body)
        self.assertEqual(err.exception.code,409)

    def test_input_error_invalidates_immediately(self):
        self.service.core.source_failed("camera disconnected")
        s=self.get("/snapshot")
        self.assertFalse(s["perception"]["source_valid"])
        self.assertEqual(s["decision"]["action"],"stop")


if __name__=="__main__":
    unittest.main(verbosity=2)
