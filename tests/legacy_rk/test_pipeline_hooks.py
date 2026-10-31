"""Exercise actual candidate worker hooks with fake RKNN calls, not real NPU claims."""
from collections import defaultdict
import importlib.util
from pathlib import Path
import queue
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from decision_service import DecisionService


class HookTests(unittest.TestCase):
    def setUp(self):
        self.saved = sys.modules.get("pcie_stream_r15_profile")
        self.base=types.ModuleType("pcie_stream_r15_profile")
        self.base.STOP=False
        self.base.OIB_LINK=None
        self.base.STATS=defaultdict(float)
        self.base.put_latest=lambda q,item:q.put_nowait(item)
        self.base.rknn_input=lambda frame,size:(frame[None],None)
        self.base.decode_auto=lambda *a,**kw:np.array([[200,12,240,68,.95,0]],np.float32)
        self.base.to_original=lambda dets,transform:dets
        sys.modules["pcie_stream_r15_profile"]=self.base
        spec=importlib.util.spec_from_file_location("test_candidate_vision",Path(__file__).resolve().parents[1]/"run_vision.py")
        self.vision=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.vision)
        self.service=DecisionService()
        self.vision.DECISIONS=self.service
        self.epoch=self.service.begin_source("video")
        self.packet=self.service.frame(np.zeros((256,448,3),np.uint8),1,self.epoch)

    def tearDown(self):
        self.service.close()
        if self.saved is None:
            sys.modules.pop("pcie_stream_r15_profile",None)
        else:
            sys.modules["pcie_stream_r15_profile"]=self.saved

    def test_detector_one_call_and_packet_metadata(self):
        q=queue.Queue();q.put(self.packet)
        calls=[]
        def infer(**kw):
            calls.append(kw)
            self.base.STOP=True
            return [np.zeros(1)]*9
        runtime=types.SimpleNamespace(inference=infer)
        args=types.SimpleNamespace(infer_every=1,size=640,conf=.25,source="video",video_layout="single")
        self.vision.detector_worker(args,["traffic light"],runtime,q)
        self.assertEqual(len(calls),1)
        self.assertIsNone(self.base.STATS.get("detector_error"))
        item=self.service.pending["signal"]
        self.assertEqual(item[0].seq,1)
        self.assertEqual(item[0].epoch,self.epoch)
        self.assertEqual(item[3],(0,0,448,256))
        self.assertEqual(self.vision.STATE["det_at"],self.packet.received_mono)

    def test_lane_one_call_and_front_crop(self):
        q=queue.Queue();q.put(self.packet)
        calls=[]
        masks=(np.ones((256,448),np.uint8),np.zeros((256,448),np.uint8))
        def infer(front):
            calls.append(front.shape)
            self.base.STOP=True
            timing={k:0.0 for k in ("infer_ms","lock_wait_ms","call_ms","pre_ms","post_ms","direct_pixels","solid_pixels","dashed_pixels")}
            return masks,timing
        args=types.SimpleNamespace(lane_hz=30,source="pcie")
        self.vision.lane_worker(args,types.SimpleNamespace(infer=infer),q)
        self.assertEqual(calls,[(256,448,3)])
        self.assertIn("boundaries",self.service.pending)
        self.assertEqual(self.vision.STATE["lane_epoch"],self.epoch)

    def test_main_cli_reaches_model_validation_and_renders_page(self):
        class BeforeNpu(Exception):
            pass
        with tempfile.TemporaryDirectory() as folder:
            classes=Path(folder)/"classes.txt"
            classes.write_text("traffic light\n",encoding="utf-8")
            for port in (18088,18090):
                with self.subTest(decision_port=port):
                    argv=["run_vision.py","--source","pcie","--no_oib",
                          "--model","det.rknn","--lane_model","lane.rknn",
                          "--classes",str(classes),"--run_seconds","0",
                          "--decision_port",str(port)]
                    with patch.object(sys,"argv",argv), patch.object(
                            self.vision,"check_model",side_effect=BeforeNpu) as check:
                        with self.assertRaises(BeforeNpu):
                            self.vision.main()
                        check.assert_called_once_with("det.rknn")
                    html=self.base.PAGE.decode("utf-8")
                    self.assertIn("max-width:100%}",html)
                    self.assertIn("端口 %d"%port,html)

    def test_video_loop_new_epoch_clears_queued_old_evidence(self):
        class Source:
            loops=0
            eof=False
            count=0
            def __init__(self,*args,**kwargs):pass
            def start(self):pass
            def close(self):pass
            def read_new_bgr(source,previous_seq,timeout):
                source.count+=1
                if source.count==3:
                    source.eof=True
                    return None,2,None
                source.loops=source.count-1
                return np.zeros((32,32,3),np.uint8),source.count,123.0
        video_module=sys.modules.get("video_source")
        sys.modules["video_source"]=types.SimpleNamespace(VideoFrameSource=Source)
        try:
            args=types.SimpleNamespace(source="video",video="fixture",video_loop=True,video_max_width=1280)
            queues=[queue.Queue() for _ in range(3)]
            self.vision.capture_worker(args,*queues)
            first,second=queues[0].get(),queues[0].get()
            self.assertNotEqual(first.epoch,second.epoch)
            self.assertEqual(self.service.core.epoch,second.epoch)
            self.assertFalse(self.service.core.publish("boundaries",first.epoch,1,first.received_mono,{}))
        finally:
            if video_module is None:sys.modules.pop("video_source",None)
            else:sys.modules["video_source"]=video_module


if __name__=="__main__":
    unittest.main(verbosity=2)
