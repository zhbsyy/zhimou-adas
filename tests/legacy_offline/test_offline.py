"""Real small file I/O + fake RKNN; never accesses NPU, PCIe, CARLA or network."""
from pathlib import Path
import importlib.util
import json
import shutil
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import patch
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
CODE = ROOT/'Board_RK3568/Inference_and_Postprocess'
sys.path.insert(0, str(CODE))
import process_video_r15_offline as processor


class FakeRKNN:
    def load_rknn(self, path): return 0
    def init_runtime(self): return 0
    def inference(self, inputs, data_format):
        outputs = [np.zeros((1,), np.float32) for _ in range(9)]
        outputs[1] = np.zeros((80*80*11,), np.float32)
        return outputs
    def release(self): pass


class Tests(unittest.TestCase):
    def run_clip(self, folder, frames=7, extra=(), disk_full=False):
        source, output = folder/'input.mp4', folder/'result.mp4'
        writer = cv2.VideoWriter(str(source), cv2.VideoWriter_fourcc(*'mp4v'), 10.0, (64, 48))
        self.assertTrue(writer.isOpened())
        for i in range(frames):
            writer.write(np.full((48,64,3), 20+i, np.uint8))
        writer.release()
        seen, lane_inputs = [], []

        class FakeLane:
            def __init__(self, *args): pass
            def infer(self, image):
                lane_inputs.append(image.copy())
                zeros = np.zeros(image.shape[:2], np.uint8)
                return (zeros, zeros), {'pre_ms':1,'call_ms':2,'post_ms':3,
                    'direct_pixels':0,'solid_pixels':0,'dashed_pixels':0}
            def close(self): pass

        def masked(frame, masks):
            out = frame.copy()
            out[:] = 250
            return out, {}

        def draw(frame, dets, names, title):
            seen.append(dets.copy())
            return frame

        dets = np.asarray([[0,0,10,10,.59,10], [0,0,10,10,.60,10], [0,0,10,10,.30,0]], np.float32)
        argv = ['offline', '--video', str(source), '--output', str(output),
            '--model','det.rknn','--lane_model','lane.rknn',
            '--classes',str(ROOT/'Board_RK3568/Models/classes_11.txt'), *extra]
        api = types.ModuleType('rknnlite.api'); api.RKNNLite=FakeRKNN
        package = types.ModuleType('rknnlite'); package.api=api
        usage = shutil._ntuple_diskusage(1024**4, 0, 1024**3)
        low = shutil._ntuple_diskusage(1024**4, 1024**4, 0)
        with patch.dict(sys.modules, {'rknnlite':package,'rknnlite.api':api}), \
             patch.object(sys,'argv',argv), \
             patch.object(processor.base,'LiveLaneModel',FakeLane), \
             patch.object(processor.base,'decode_auto',return_value=dets), \
             patch.object(processor.base,'to_original',side_effect=lambda value, transform:value), \
             patch.object(processor.base,'draw_lane_mask',side_effect=masked), \
             patch.object(processor.base,'draw',side_effect=draw), \
             patch.object(processor.shutil,'disk_usage',side_effect=[usage,low] if disk_full else lambda path:usage):
            if disk_full:
                with self.assertRaises(SystemExit) as exit_value:
                    processor.main()
                self.assertEqual(exit_value.exception.code, 2)
            else:
                processor.main()
        report=json.loads(output.with_suffix('.summary.json').read_text(encoding='utf-8'))
        cap=cv2.VideoCapture(str(output))
        try:
            self.assertTrue(cap.isOpened())
            count=int(cap.get(cv2.CAP_PROP_FRAME_COUNT)); fps=cap.get(cv2.CAP_PROP_FPS)
        finally:
            cap.release()
        return report, seen, lane_inputs, count, fps

    def test_full_file_frame_count_and_class10_threshold(self):
        with tempfile.TemporaryDirectory() as d:
            report,seen,lane_inputs,count,fps=self.run_clip(Path(d))
            self.assertTrue(report['complete'])
            self.assertEqual(report['stop_reason'],'source_eof')
            self.assertEqual(report['processed'],7)
            self.assertEqual(count,7)
            self.assertAlmostEqual(fps,10)
            self.assertAlmostEqual(report['output_duration_s'],.7)
            for dets in seen:
                self.assertEqual(dets[:,5].tolist(),[10,0])
            # Drawing changed pixels to 250, but lane must see the raw input.
            self.assertTrue(all(image.max()<100 for image in lane_inputs))

    def test_stride_preserves_sampled_timeline_and_explicit_threshold(self):
        with tempfile.TemporaryDirectory() as d:
            report,seen,_,count,fps=self.run_clip(Path(d),frames=9,extra=('--stride','3','--class10_conf','0.4'))
            self.assertEqual(report['source_read'],9)
            self.assertEqual(count,3)
            self.assertAlmostEqual(report['output_duration_s'],.9)
            self.assertAlmostEqual(fps,10/3,places=2)
            self.assertEqual(len(seen[0]),3)
            self.assertEqual(report['class10_min_confidence'],.4)

    def test_requested_segment_and_incomplete_disk_exit(self):
        with tempfile.TemporaryDirectory() as d:
            report,_,_,count,_=self.run_clip(Path(d),extra=('--seconds','0.2'))
            self.assertEqual(report['stop_reason'],'duration')
            self.assertEqual(count,2)
        with tempfile.TemporaryDirectory() as d:
            report,_,_,count,_=self.run_clip(Path(d),frames=25,disk_full=True)
            self.assertFalse(report['complete'])
            self.assertEqual(report['stop_reason'],'low_disk_space')
            self.assertEqual(count,20)

    def test_relocated_import_has_no_live_pipeline_dependencies(self):
        with tempfile.TemporaryDirectory(prefix='offline relocation ') as d:
            local=Path(d)/'different path with spaces'
            shutil.copytree(CODE,local,ignore=shutil.ignore_patterns('__pycache__'))
            code='''import sys
from pathlib import Path
root=Path(sys.argv[1]);sys.path.insert(0,str(root))
import process_video_r15_offline,offline_pipeline,pcie_post,r15_front_fast
for m in (process_video_r15_offline,offline_pipeline,pcie_post,r15_front_fast):
 assert Path(m.__file__).resolve().is_relative_to(root.resolve())
assert not any(m in sys.modules for m in ('pcie_frame','cap_reader','oib_link','carla_source','pcie_stream_r15_profile'))
'''
            result=subprocess.run([sys.executable,'-I','-B','-c',code,str(local)],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)

    def test_launcher_paths_defaults_and_models_are_independent(self):
        path=ROOT/'Board_RK3568/launch_offline.py'
        spec=importlib.util.spec_from_file_location('offline_launcher_test',path)
        launcher=importlib.util.module_from_spec(spec);spec.loader.exec_module(launcher)
        a=launcher.parse_args([])
        self.assertEqual((a.class10_conf,a.stride,a.seconds),(.6,1,0))
        cmd=launcher.command('python',a,Path('result.mp4'))
        for flag in ('--classes','--model','--lane_model'):
            p=Path(cmd[cmd.index(flag)+1])
            self.assertTrue(p.is_file());self.assertTrue(p.resolve().is_relative_to(ROOT.resolve()))


if __name__=='__main__':
    unittest.main(verbosity=2)
