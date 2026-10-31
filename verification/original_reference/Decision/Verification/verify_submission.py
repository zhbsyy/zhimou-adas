"""Offline integrity verification; no hardware, network, actor or model calls."""
from pathlib import Path
import argparse
import hashlib
import json

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--section', choices=('all', 'board', 'pc', 'fpga'), default='all')
    args = parser.parse_args()
    prefixes = {'board': 'Board_RK3568/', 'pc': 'PC_Windows/', 'fpga': 'FPGA_PG2L50H/'}
    checked, failures = 0, []
    for line in (ROOT/'MANIFEST.sha256').read_text(encoding='utf-8').splitlines():
        expected, relative = line.split('  ', 1)
        if args.section != 'all' and not relative.startswith(prefixes[args.section]):
            continue
        path = ROOT/relative
        if not path.resolve().is_relative_to(ROOT.resolve()):
            failures.append(relative + ': outside submission')
            continue
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            failures.append(relative)
        checked += 1
    if not checked:
        failures.append('No files matched requested section')
    print(json.dumps({'section': args.section, 'checked': checked, 'passed': not failures,
                      'failures': failures}, ensure_ascii=False, indent=2))
    if failures:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
