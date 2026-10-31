"""Reject unassigned PCI resources before invoking any capture code."""
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pcie_preflight import require_pcie_ready
from single_sources import PcieSingleSource


class PciePreflightTests(unittest.TestCase):
    def fixture(self, folder, start=0xf0204000, command=6):
        path = Path(folder)
        (path/"resource").write_text("0 0 0\n%016x %016x 0000000000000200\n" %
                                     (start, start+4095 if start else 0), encoding="ascii")
        (path/"config").write_bytes(b"\x55\x07\x55\x07"+command.to_bytes(2, "little"))
        return path

    def test_nonzero_enabled_resource_passes(self):
        with tempfile.TemporaryDirectory() as folder:
            self.assertEqual(require_pcie_ready(self.fixture(folder))["bar1"], "0xf0204000")

    def test_zero_resource_and_disabled_commands_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            for address, command in ((0, 6), (0xf0204000, 0), (0xf0204000, 2), (0xf0204000, 4)):
                with self.subTest(address=address, command=command), self.assertRaisesRegex(RuntimeError, "PCIE_NOT_READY"):
                    require_pcie_ready(self.fixture(folder, address, command))

    def test_missing_device_rejected(self):
        with tempfile.TemporaryDirectory() as folder, self.assertRaisesRegex(RuntimeError, "pcie_up.sh"):
            require_pcie_ready(Path(folder)/"not_enumerated")

    def test_source_cannot_import_or_start_capture_after_failed_check(self):
        source = PcieSingleSource("unused")
        with patch("pcie_preflight.require_pcie_ready", side_effect=RuntimeError("PCIE_NOT_READY")), patch(
                "single_sources.importlib.util.spec_from_file_location") as loader:
            with self.assertRaisesRegex(RuntimeError, "PCIE_NOT_READY"):
                source.start()
            loader.assert_not_called()
        self.assertIsNone(source.source)


if __name__ == "__main__":
    unittest.main()
