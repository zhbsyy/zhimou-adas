"""Checks for supplemented files, source lineage and standalone tool imports."""
from pathlib import Path
import csv
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class SupplementTests(unittest.TestCase):
    def test_all_archived_supplement_bytes_match_sources_record(self):
        records = json.loads((ROOT / 'verification/SUPPLEMENT_PROVENANCE.json').read_text(encoding='utf-8'))
        self.assertFalse(records['original_submission_modified'])
        destinations = []
        for row in records['files']:
            p = (ROOT / row['destination']).resolve()
            self.assertTrue(p.is_relative_to(ROOT))
            self.assertEqual(sha(p), row['sha256'], row['destination'])
            destinations.append(row['destination'])
        self.assertEqual(len(destinations), len(set(destinations)))

    def test_training_snapshot_and_model_lineage_hashes(self):
        train = ROOT / 'training/lane_detection'
        run = train / 'runs/R15_external_data_training'
        config = json.loads((train / 'records/final_run/config.json').read_text())
        self.assertEqual(sha(run / 'train_r15_lanefocus.py'), config['trainer_sha256'])
        self.assertEqual(sha(train / 'checkpoints/upstream_small.pth'), config['weight_source_sha256'])
        self.assertEqual(sha(run / 'candidate_checkpoints/targetmix_e15.pt'), config['init_sha256'])
        for split in ('train', 'val'):
            self.assertEqual(sha(run / ('data_v1/' + split + '_manifest.csv')), config['manifest_sha256'][split])
        report = json.loads((train / 'records/export_report.json').read_text())
        self.assertEqual(sha(ROOT / 'models/lane_detection/R15_twinlitenet_small_direct2_lane3_research.pt'), report['checkpoint_sha256'])
        self.assertEqual(sha(ROOT / 'models/lane_detection/onnx/R15_256x448_original.onnx'), report['original_onnx_sha256'])
        self.assertEqual(sha(ROOT / 'models/lane_detection/onnx/R15_256x448_gcn_matmul.onnx'), report['patched_onnx_sha256'])

    def test_calibration_images_and_detector_reference_contract(self):
        import cv2
        calibration = ROOT / 'training/lane_detection/conversion/calibration_v1/input_256x448'
        images = list(calibration.glob('*.png'))
        self.assertEqual(len(images), 160)
        for p in images:
            self.assertEqual(cv2.imread(str(p)).shape, (256, 448, 3))
        source = ROOT / 'training/object_detection/reference_conversion'
        manifest = json.loads((source / 'records/deployment_manifest.json').read_text())
        self.assertEqual(manifest['classes'], (ROOT / 'models/object_detection/classes.txt').read_text().splitlines())
        self.assertEqual(sha(ROOT / 'models/object_detection/reference/A_original.pt'), manifest['source_weights_sha256'])
        self.assertTrue((source / 'vendor/ultralytics/__init__.py').is_file())
        self.assertTrue((source / 'vendor/README.md').is_file())

    def test_three_tools_import_in_isolated_layout_without_live_actions(self):
        base = (ROOT.parent / 'analysis/submission2_validation_20261031').resolve()
        base.mkdir(parents=True, exist_ok=True)
        work = Path(tempfile.mkdtemp(prefix='tools ', dir=base)).resolve()
        try:
            env = dict(os.environ, FPGA_ADAS_RUNTIME_DIR=str(work / 'runtime'))
            for kind in ('cameras', 'brakes', 'inspect-video'):
                result = subprocess.run([sys.executable, '-I', '-B', str(ROOT / 'scripts/project.py'),
                                         'pc-tool', kind, '--check'], env=env,
                                        capture_output=True, text=True, encoding='utf-8', errors='replace')
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                record = json.loads(result.stdout)
                self.assertFalse(record['CARLA_connection_or_actors_started'])
            result = subprocess.run([sys.executable, '-I', '-B', str(ROOT / 'scripts/project.py'),
                                     'pc-tool', 'inspect-video', str(ROOT / 'assets/videos/video_20260929_112507.mp4')],
                                    env=env, capture_output=True, text=True, encoding='utf-8', errors='replace')
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            info = json.loads(result.stdout)
            self.assertEqual(info['size'], [1920, 1080])
            self.assertAlmostEqual(info['duration_s'], 44.9666667, places=3)
        finally:
            assert work.is_relative_to(base) and work != base
            shutil.rmtree(work)


if __name__ == '__main__':
    unittest.main(verbosity=2)
