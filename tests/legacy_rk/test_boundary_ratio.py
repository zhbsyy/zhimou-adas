"""Synthetic mask tests. These do not claim real-road recognition accuracy."""
from copy import deepcopy
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from decision_service import DecisionService,load_config
from perception_summary import BoundaryTracker,extract_boundaries


ROOT=Path(__file__).resolve().parents[1]
CONFIG=load_config(ROOT/'decision_config.json')
CFG=CONFIG['boundaries']


def masks(left=2,right=0):
    road=np.zeros((256,448),np.uint8)
    road[100:250,100:350]=1
    lane=np.zeros_like(road)
    for x,dyellow in ((120,left),(320,right)):
        lane[125:245,x:x+5]=1
        lane[125:245,x:x+dyellow]=2
    return road,lane


class RatioTests(unittest.TestCase):
    def test_forty_percent_dashed_otherwise_solid(self):
        r=extract_boundaries(*masks(2,1),config=CFG)
        self.assertEqual(r['left']['kind'],'dashed')
        self.assertAlmostEqual(r['left']['dashed_fraction'],.4)
        self.assertEqual(r['right']['kind'],'solid')
        self.assertAlmostEqual(r['right']['dashed_fraction'],.2)
        stricter=extract_boundaries(*masks(2,1),config={**CFG,'dashed_ratio_threshold':.400001})
        self.assertEqual(stricter['left']['kind'],'solid')

    def test_configurable_threshold(self):
        r=extract_boundaries(*masks(2,5),config={**CFG,'dashed_ratio_threshold':.5})
        self.assertEqual(r['left']['kind'],'solid')
        self.assertEqual(r['right']['kind'],'dashed')

    def test_short_line_remains_unknown_even_all_yellow(self):
        road,lane=masks(5,5)
        lane[:]=0;lane[222:228,120:125]=2
        r=extract_boundaries(road,lane,CFG)
        self.assertFalse(r['left']['valid'])
        self.assertFalse(r['right']['valid'])

    def test_length_gate_not_only_sample_count(self):
        cfg={**CFG,'min_length_ratio':.45}
        road,lane=masks(5,5)
        lane[:180]=0;lane[210:]=0
        r=extract_boundaries(road,lane,cfg)
        self.assertFalse(r['left']['valid'])
        self.assertEqual(r['left']['reason'],'boundary_too_short')
        self.assertIn('estimated_length_px',r['left'])

    def test_lower_right_video_road_can_be_selected(self):
        road=np.zeros((256,448),np.uint8);lane=np.zeros_like(road)
        road[190:246,295:440]=1
        lane[190:246,288:293]=2
        lane[190:246,439:444]=1
        r=extract_boundaries(road,lane,CFG)
        self.assertEqual(r['left']['kind'],'dashed')
        self.assertEqual(r['right']['kind'],'solid')
        self.assertGreater(r['selection']['anchor_x'],.8)
        old=extract_boundaries(road,lane)
        self.assertFalse(old['left']['valid'])

    def test_missing_road_is_not_filled_by_ratio(self):
        road,lane=masks(5,5);road[:]=0
        r=extract_boundaries(road,lane,CFG)
        self.assertEqual(r['left']['reason'],'no_road_component')
        self.assertFalse(r['left']['valid'])

    def test_two_comparable_roads_are_ambiguous(self):
        road=np.zeros((256,448),np.uint8);lane=np.zeros_like(road)
        road[130:245,20:160]=1;road[130:245,280:420]=1
        r=extract_boundaries(road,lane,CFG)
        self.assertEqual(r['left']['reason'],'ambiguous_road_components')

    def test_nearest_boundary_not_whole_image_yellow_majority(self):
        road,lane=masks(0,0)
        lane[125:245,20:60]=2
        r=extract_boundaries(road,lane,CFG)
        self.assertEqual(r['left']['kind'],'solid')
        self.assertEqual(r['right']['kind'],'solid')

    def test_thick_horizontal_object_is_not_lane_boundary(self):
        road,lane=masks(0,0);lane[:]=0;lane[160:235,10:200]=2
        self.assertFalse(extract_boundaries(road,lane,CFG)['left']['valid'])

    def test_hysteresis_and_short_evidence_invalidate(self):
        t=BoundaryTracker(dashed_exit=.3)
        r=extract_boundaries(*masks(3,0),config=CFG)
        self.assertFalse(t.update(r,1,10)['left']['valid'])
        self.assertTrue(t.update(r,2,10.1)['left']['valid'])
        near=deepcopy(r);near['left'].update(kind='solid',dashed_fraction=.35)
        held=t.update(near,3,10.2)['left']
        self.assertEqual(held['kind'],'dashed')
        self.assertEqual(held['raw_kind'],'solid')
        low=deepcopy(near);low['left']['dashed_fraction']=.2
        self.assertFalse(t.update(low,4,10.3)['left']['valid'])
        self.assertEqual(t.update(low,5,10.4)['left']['kind'],'solid')
        short=deepcopy(r);short['left'].update(kind='unknown',valid=False,reason='boundary_too_short')
        self.assertFalse(t.update(short,6,10.5)['left']['valid'])
        self.assertFalse(t.update(r,7,10.6)['left']['valid'])

    def test_invalid_config_rejected(self):
        for key,value in [('dashed_ratio_threshold',0),('dashed_exit_ratio',.9),
                          ('min_length_ratio',0),('min_length_ratio',float('nan')),
                          ('anchor_mode','guess'),('classification','rgb_threshold')]:
            cfg=deepcopy(CONFIG);cfg['boundaries'][key]=value
            with tempfile.TemporaryDirectory() as folder:
                path=Path(folder)/'config.json';path.write_text(json.dumps(cfg),encoding='utf-8')
                with self.assertRaises(ValueError):load_config(path)

    def test_new_summaries_drive_existing_rules_without_new_inference(self):
        # Isolate legacy boundary rules; combined AEB arbitration is tested separately.
        cfg=deepcopy(CONFIG);cfg['aeb']['enabled']=False
        service=DecisionService(cfg,evidence_mode='synthetic_test')
        service.start('127.0.0.1',0)
        epoch=service.begin_source('synthetic_test')
        frame=np.zeros((256,448,3),np.uint8)
        try:
            service.core.command({'client_id':'ratio_test','request_id':'context','seq':1,
                                  'source_epoch':epoch,'action':'set_context',
                                  'scenario':'lane_change','lane':'straight'})
            for i in range(1,4):
                packet=service.frame(frame,i,epoch)
                service.submit_lane(packet,masks(0,2));time.sleep(.08)
            s=service.snapshot()
            self.assertEqual(s['perception']['boundaries']['left']['kind'],'solid')
            self.assertEqual(s['perception']['boundaries']['right']['kind'],'dashed')
            reply=service.core.command({'client_id':'ratio_test','request_id':'right','seq':2,
                         'source_epoch':epoch,'action':'right'})
            self.assertTrue(reply['ack']['accepted'])
            reply=service.core.command({'client_id':'ratio_test','request_id':'left','seq':3,
                         'source_epoch':epoch,'action':'left'})
            self.assertFalse(reply['ack']['accepted'])
            self.assertEqual(reply['snapshot']['decision']['alarm']['code'],'SOLID_LINE_CHANGE_REQUEST')
        finally:service.close()


if __name__=='__main__':unittest.main(verbosity=2)
