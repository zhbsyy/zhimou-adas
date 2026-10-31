from pathlib import Path
import importlib.util
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('collected_color_cli', ROOT/'Board_RK3568/CALIBRATE_COLORS.py')
cli = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cli)


class ColorManifestTests(unittest.TestCase):
    def test_changes_only_profile_entry(self):
        with tempfile.TemporaryDirectory(prefix='color_manifest_test_', dir=ROOT) as folder:
            self.assertEqual(Path(folder).resolve().parent, ROOT.resolve())
            path = Path(folder)/'MANIFEST.sha256'
            other = 'c'*64 + '  Board_RK3568/Inference_Four_Views/quad_infer.py\n'
            path.write_text('a'*64 + '  ' + cli.PROFILE_NAME + '\n' + other, encoding='utf-8')
            cli.sync_manifest(path, 'a'*64, 'b'*64)
            self.assertEqual(path.read_text(encoding='utf-8'), 'b'*64 + '  ' + cli.PROFILE_NAME + '\n' + other)

    def test_changed_baseline_is_refused(self):
        with tempfile.TemporaryDirectory(prefix='color_manifest_test_', dir=ROOT) as folder:
            self.assertEqual(Path(folder).resolve().parent, ROOT.resolve())
            path = Path(folder)/'MANIFEST.sha256'
            original = 'c'*64 + '  ' + cli.PROFILE_NAME + '\n'
            path.write_text(original, encoding='utf-8')
            with self.assertRaises(ValueError):
                cli.sync_manifest(path, 'a'*64, 'b'*64)
            self.assertEqual(path.read_text(encoding='utf-8'), original)

    def test_duplicate_profile_entry_is_refused(self):
        with tempfile.TemporaryDirectory(prefix='color_manifest_test_', dir=ROOT) as folder:
            self.assertEqual(Path(folder).resolve().parent, ROOT.resolve())
            path = Path(folder)/'MANIFEST.sha256'
            original = ('a'*64 + '  ' + cli.PROFILE_NAME + '\n')*2
            path.write_text(original, encoding='utf-8')
            with self.assertRaises(ValueError):
                cli.sync_manifest(path, 'a'*64, 'b'*64)
            self.assertEqual(path.read_text(encoding='utf-8'), original)


if __name__ == '__main__':
    unittest.main()
