"""Check collected files offline; never touches capture, NPU, network or JTAG."""
from pathlib import Path
import argparse
import hashlib
import json

ROOT = Path(__file__).resolve().parents[1]


def verify(section='all'):
    prefixes = {'board': ('Board_RK3568/',), 'pc': ('PC_Windows/',),
                'fpga': ('FPGA_Black_PG2L50H_RX/', 'FPGA_Red_PGL50H_TX/')}
    checked, failures = 0, []
    for line in (ROOT/'MANIFEST.sha256').read_text(encoding='utf-8').splitlines():
        expected, relative = line.split('  ', 1)
        if section != 'all' and not relative.startswith(prefixes[section]):
            continue
        path = ROOT/relative
        if not path.resolve().is_relative_to(ROOT.resolve()):
            failures.append(relative + ': outside module')
        elif not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            failures.append(relative)
        checked += 1
    if not checked:
        failures.append('No files matched')
    return {'section': section, 'checked': checked, 'passed': not failures, 'failures': failures}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--section', choices=('all', 'board', 'pc', 'fpga'), default='all')
    result = verify(parser.parse_args().section)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if not result['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
