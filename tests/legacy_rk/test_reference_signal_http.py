from pathlib import Path
import json
import sys
import tempfile
import time
import unittest
import urllib.request

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from decision_service import DecisionService,load_config


class ReferenceHttpTests(unittest.TestCase):
    def test_configured_service_debug_and_rule_commands(self):
        cfg=load_config(ROOT/'decision_config.json')
        self.assertEqual(cfg['scene']['signal_algorithm'],'traffic_light_pixel_ratio_rg')
        service=DecisionService(cfg,stats_provider=lambda:{'runtime_profile':{'version':'stable_rules.v1.1'}})
        host,port=service.start('127.0.0.1',0)
        epoch=service.begin_source('test_fixture')
        url='http://%s:%d'%(host,port)
        d=np.asarray([[20,30,60,130,.8,8]],np.float32)
        names=['car']*11;names[8]='traffic light';names[10]='crosswalk'
        seq=0
        def submit(frame, detections=None):
            nonlocal seq
            for i in range(2):
                seq+=1;packet=service.frame(frame,seq,epoch)
                service.submit_detection(packet,d if detections is None else detections,names)
                until=time.monotonic()+1
                while time.monotonic()<until and service.snapshot()['metrics']['processed_signal']<seq:
                    time.sleep(.005)
                if i==0:time.sleep(.07)
        def post(request):
            body={'client_id':'http_test','request_id':'r'+str(request),'seq':request,
                  'source_epoch':epoch,'action':'move'}
            req=urllib.request.Request(url+'/command',data=json.dumps(body).encode(),
                headers={'Content-Type':'application/json'},method='POST')
            with urllib.request.urlopen(req,timeout=2) as response:return json.load(response)
        try:
            frame=np.full((160,240,3),10,np.uint8)
            color=cv2.cvtColor(np.uint8([[[60,60,95]]]),cv2.COLOR_HSV2BGR)[0,0]
            cv2.circle(frame,(40,52),8,tuple(map(int,color)),-1)
            submit(frame)
            with urllib.request.urlopen(url+'/signal_debug',timeout=2) as response:debug=json.load(response)
            self.assertEqual(debug['signal']['color'],'green')
            self.assertEqual(debug['signal']['soft_candidate'],'green')
            self.assertEqual(debug['signal']['color_algorithm'],'traffic_light_pixel_ratio_rg')
            self.assertTrue(post(1)['ack']['accepted'])
            submit(np.full_like(frame,255))
            reply=post(2)
            # No colored pixels is an explicit red tie, never inherited green.
            self.assertFalse(reply['ack']['accepted'])
            self.assertEqual(reply['snapshot']['perception']['signal']['color'],'red')
            diag=reply['snapshot']['perception']['signal']['color_diagnostics']
            self.assertEqual((diag['red_pixels'],diag['green_pixels']),(0,0))
            red=np.full_like(frame,10);cv2.circle(red,(40,52),8,(0,0,245),-1)
            submit(red)
            reply=post(3)
            self.assertFalse(reply['ack']['accepted'])
            self.assertEqual(reply['ack']['reason_code'],'RED_LIGHT_STOP')
            self.assertEqual(reply['snapshot']['decision']['alarm']['code'],'RED_LIGHT_MOVE_REQUEST')
            submit(frame,np.empty((0,6),np.float32))
            reply=post(4)
            self.assertTrue(reply['ack']['accepted'])
            self.assertEqual(reply['ack']['reason_code'],'NO_RED_LIGHT_GO')
            with urllib.request.urlopen(url+'/snapshot',timeout=2) as response:snapshot=json.load(response)
            self.assertEqual(snapshot['runtime_config']['version'],'stable_rules.v1.1')
            self.assertEqual(snapshot['active_boundaries']['dashed_ratio_threshold'],.4)
        finally:service.close()

    def test_forced_display_config_cannot_silently_replace_decision_mode(self):
        cfg=load_config(ROOT/'decision_config.json');cfg['scene']['signal_algorithm']='forced'
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'bad.json';p.write_text(json.dumps(cfg))
            with self.assertRaisesRegex(ValueError,'Unsupported signal_algorithm'):load_config(p)


if __name__=='__main__':unittest.main(verbosity=2)
