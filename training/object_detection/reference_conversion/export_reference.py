"""Portable entry for the archived detector comparison exporter."""
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path[:0] = [str(HERE / 'vendor'), str(HERE)]


def main():
    # The original script imports Ultralytics at module scope; set the local path first.
    import export_A_original_11 as exporter
    supplied = sys.argv[1:]
    if not any(x == '--image' or x.startswith('--image=') for x in supplied) and not any(x in ('-h', '--help') for x in supplied):
        raise SystemExit('Pass --image <actual test image>; no original D-drive assets are assumed.')
    if any(x == '--weights' or x.startswith('--weights=') for x in supplied):
        raise SystemExit('This reference entry pins models/object_detection/reference/A_original.pt.')
    sys.argv = [str(HERE / 'export_A_original_11.py'), '--weights',
                str(ROOT / 'models/object_detection/reference/A_original.pt'), *supplied]
    exporter.main()


if __name__ == '__main__':
    main()
