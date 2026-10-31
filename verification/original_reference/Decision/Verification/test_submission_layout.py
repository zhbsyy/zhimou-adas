"""Submission closure tests. No real PCIe, RKNN inference or CARLA actor writes."""
from pathlib import Path
import hashlib
import importlib.util
import json
import os
import re
import shutil
import struct
import subprocess
import sys
import tempfile
import unittest
import types
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]


class Tests(unittest.TestCase):
    def test_latest_console_retains_frozen_panel_and_separate_ports(self):
        pc=ROOT/'PC_Windows'
        settings=json.loads((pc/'settings.json').read_text(encoding='utf-8'))
        self.assertEqual(settings['start_mode'],'decision')
        self.assertEqual((settings['rk_video_port'],settings['rk_decision_video_port'],settings['rk_decision_port']),(8083,18089,18088))
        for frozen,used in (('candidate_panel.py','rk_decision_panel.py'),('rk_decision_transport.py','rk_decision_transport.py'),('static_vehicle_session.py','static_vehicle_session.py')):
            self.assertEqual((pc/'Decision_Panel'/frozen).read_bytes(),(pc/'Console_Application/ui'/used).read_bytes())

    def test_copied_runtime_sources_match_provenance(self):
        data = json.loads((ROOT/'Verification/PROVENANCE.json').read_text(encoding='utf-8'))
        for record in data['files']:
            path = ROOT/record['destination']
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), record['sha256'], record['destination'])

    def test_both_model_hashes_and_working_capture_binary_are_pinned(self):
        d = json.loads((ROOT/'Verification/FROZEN_VERSION.json').read_text(encoding='utf-8'))
        for name, expected in d['models'].items():
            self.assertEqual(hashlib.sha256((ROOT/'Board_RK3568/Models'/name).read_bytes()).hexdigest(), expected)
        code = (ROOT/'Board_RK3568/Camera_PCIe/cap_reader.py').read_text(encoding='utf-8')
        self.assertIn('while not STOP:', code)
        self.assertNotIn('struct.pack_into(', code)
        self.assertIn('buf.write(b"\\x00" * total)', code)
        library = (ROOT/'Board_RK3568/Camera_PCIe/mmio32.so').read_bytes()
        self.assertEqual(library[:5], b'\x7fELF\x02')
        self.assertEqual(struct.unpack_from('<H', library, 18)[0], 183)  # EM_AARCH64
        self.assertEqual(hashlib.sha256(library).hexdigest(), '911b9d318a362693ee905ad3dcd7714356110707811a1a209404a4a4dfbd1b0c')

    def test_launch_is_continuous_and_all_model_asset_paths_are_inside(self):
        path = ROOT/'Board_RK3568/launch_decision.py'
        spec = importlib.util.spec_from_file_location('final_launch_layout_test', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        args = module.command(Path('/system/independent/python'))
        for flag, expected in {'--size':'640', '--lane_height':'256', '--lane_width':'448',
            '--lane_input_mode':'uint8_nhwc', '--infer_every':'1', '--lane_hz':'30', '--q_depth':'1',
            '--quality':'85', '--runtime_backend':'native_buffered', '--lane_argmax':'planar',
            '--run_seconds':'0', '--warmup_seconds':'10', '--port':'18089', '--decision_port':'18088',
            '--source':'pcie', '--aeb_mode':'video_demo'}.items():
            self.assertEqual(args[args.index(flag)+1], expected, flag)
        for flag in ('--model', '--classes', '--lane_model', '--camera_color_profile',
                     '--stereo_calibration', '--decision_config', '--preflight_video', '--pcie_dir'):
            asset = Path(args[args.index(flag)+1])
            self.assertTrue(asset.resolve().is_relative_to(ROOT.resolve()), flag)
            self.assertTrue(asset.exists(), asset)
        for flag in ('--cached_display', '--publication_callback', '--no_oib', '--stereo_capture'):
            self.assertIn(flag, args)
        env = module.environment()
        self.assertEqual(env['CAP_DELAY'], '0.00001')
        self.assertTrue(all(Path(v).is_relative_to(ROOT) for v in env['PYTHONPATH'].split(os.pathsep)))

    def test_pipeline_imports_from_submission_only(self):
        board = ROOT/'Board_RK3568'
        code = '''import sys,json
from pathlib import Path
b=Path(sys.argv[1])
sys.path[:0]=[str(b/'Inference_and_Rules'),str(b/'Dependencies'),str(b/'Camera_PCIe')]
import os
os.environ.update(BOARD_DEP_DIR=str(b/'Camera_PCIe'),YOLO_DIR=str(b/'Dependencies'))
import run_vision,pcie_post,pcie_frame,decision_service,stereo_source,native_api
mods=[run_vision,pcie_post,pcie_frame,decision_service,stereo_source,native_api]
assert all(Path(m.__file__).resolve().is_relative_to(b.resolve()) for m in mods)
print(json.dumps({m.__name__:m.__file__ for m in mods}))
'''
        result = subprocess.run([sys.executable, '-I', '-B', '-c', code, str(board)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)

    def test_failed_pcie_check_points_to_submission_recovery(self):
        path = ROOT/'Board_RK3568/launch_decision.py'
        spec = importlib.util.spec_from_file_location('final_recovery_test', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        def unavailable():
            raise RuntimeError('PCIE_NOT_READY: run sudo sh /home/linaro/03_RK3568_Perception/pcie_up.sh')
        with patch.dict(sys.modules, {'pcie_preflight': types.SimpleNamespace(require_pcie_ready=unavailable)}):
            with self.assertRaises(RuntimeError) as caught:
                module.local_pcie_ready()
        self.assertIn('INITIALIZE_PCIE.sh', str(caught.exception))
        self.assertNotIn('/home/linaro/03_RK3568_Perception', str(caught.exception))

    def test_PC_launcher_still_works_after_relocation(self):
        # This independent copy cannot resolve old sibling candidate folders.
        # The check imports Qt/CARLA packages, but creates no windows or actors.
        with tempfile.TemporaryDirectory(prefix='decision relocation ') as temporary:
            pc = Path(temporary)/'Different location with spaces/PC_Windows'
            shutil.copytree(ROOT/'PC_Windows', pc, ignore=shutil.ignore_patterns('Results', '__pycache__'))
            result = subprocess.run([sys.executable, '-I', '-B', str(pc/'launch_console.py'), '--check'],
                                    cwd=temporary, capture_output=True, text=True, encoding='utf-8', errors='replace')
            self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
            data = json.loads(result.stdout)
            self.assertEqual(data['PC_panel_version'], 'stereo_console.v0.1.5')
            for path in data['module_paths'].values():
                self.assertTrue(Path(path).resolve().is_relative_to(pc.resolve()), path)

    def test_pds_has_no_missing_or_external_file_reference(self):
        project = ROOT/'FPGA_PG2L50H/Build_Project'
        files = re.findall(r'\(_file "([^"]+)"', (project/'project.pds').read_text())
        self.assertEqual(len(files), 226)
        for name in files:
            path = (project/name).resolve()
            self.assertTrue(path.is_relative_to(project.resolve()), name)
            self.assertTrue(path.is_file(), name)

    def test_no_archives_or_historical_launchers_shipped(self):
        archives = {'.zip', '.rar', '.7z', '.tgz', '.gz', '.tar'}
        self.assertFalse([str(p) for p in ROOT.rglob('*') if p.is_file() and p.suffix.lower() in archives])
        self.assertFalse((ROOT/'Board_RK3568/Inference_and_Rules/run_decision.sh').exists())
        self.assertFalse((ROOT/'Board_RK3568/Inference_and_Rules/run_stereo.sh').exists())


if __name__ == '__main__':
    unittest.main(verbosity=2)
