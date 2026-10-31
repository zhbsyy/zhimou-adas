"""Relocation checks independent of the old project and installed hardware."""
from pathlib import Path
import ast
import hashlib
import importlib.util
import json
import re
import unittest

ROOT = Path(__file__).resolve().parents[1]
BOARD = ROOT/'Board_RK3568'


class CollectedPaths(unittest.TestCase):
    def test_board_command_uses_collected_files(self):
        spec = importlib.util.spec_from_file_location('collected_launcher', BOARD/'launch_cameras.py')
        launcher = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(launcher)
        command = launcher.command(Path('/test/python'))
        flags = ('--model', '--lane-model', '--classes', '--color-profile', '--cap-reader')
        for flag in flags:
            path = Path(command[command.index(flag)+1])
            self.assertTrue(path.is_file(), path)
            self.assertTrue(path.resolve().is_relative_to(BOARD.resolve()))
        self.assertIn('Camera_Input', command[command.index('--cap-reader')+1])
        self.assertEqual(command[command.index('--port')+1], '8083')
        self.assertEqual(command[command.index('--seconds')+1], '0')

    def test_models_and_fixed_thresholds(self):
        freeze = json.loads((ROOT/'Verification/FROZEN_VERSION.json').read_text(encoding='utf-8'))
        for name, expected in freeze['models'].items():
            self.assertEqual(hashlib.sha256((BOARD/'Models'/name).read_bytes()).hexdigest(), expected)
        source = ast.parse((BOARD/'Inference_Four_Views/detection_policy.py').read_text(encoding='utf-8'))
        constants = {node.targets[0].id: ast.literal_eval(node.value) for node in source.body
                     if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name)
                     and node.targets[0].id in ('CLASS10_MIN_CONFIDENCE', 'OTHER_MIN_CONFIDENCE')}
        self.assertEqual(constants, {'CLASS10_MIN_CONFIDENCE': .4, 'OTHER_MIN_CONFIDENCE': .25})
        color = json.loads((BOARD/'Configuration/camera_profiles.json').read_text(encoding='utf-8'))
        self.assertEqual(color['revision'], 2)
        self.assertEqual(color['cameras']['front']['gains_rgb'], [.928571, 1.114286, .975])
        self.assertTrue(color['cameras']['rear']['swap_rb'])

    def test_both_fpga_projects_have_all_local_references(self):
        projects = [('FPGA_Black_PG2L50H_RX', 'project.pds', 226),
                    ('FPGA_Red_PGL50H_TX', 'stereo_sfp_tx.pds', 69)]
        for folder, name, count in projects:
            project = ROOT/folder/'Build_Project'
            refs = re.findall(r'\(_file "([^"]+)"', (project/name).read_text(encoding='utf-8'))
            self.assertEqual(len(refs), count)
            for relative in refs:
                path = project/relative
                self.assertTrue(path.resolve().is_relative_to(project.resolve()))
                self.assertTrue(path.is_file(), relative)
        freeze = json.loads((ROOT/'Verification/FROZEN_VERSION.json').read_text(encoding='utf-8'))
        for folder, row in zip(('FPGA_Black_PG2L50H_RX', 'FPGA_Red_PGL50H_TX'), freeze['fpga']):
            bit = ROOT/folder/'PROGRAM_THIS_BITSTREAM'/row['bitstream']
            self.assertEqual(hashlib.sha256(bit.read_bytes()).hexdigest(), row['sha256'])


if __name__ == '__main__':
    unittest.main()
