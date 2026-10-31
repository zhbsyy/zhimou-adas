"""Compatibility verifier inside a generated runtime; files remain hash pinned."""
from pathlib import Path
import argparse
import hashlib
import json

ROOT=Path(__file__).resolve().parents[1]


def verify(section='all'):
    checked,failures=0,[]
    for line in (ROOT/'MANIFEST.sha256').read_text(encoding='utf-8').splitlines():
        expected,relative=line.split('  ',1)
        if section=='board' and not relative.startswith('Board_RK3568/'):continue
        p=(ROOT/relative).resolve()
        if not p.is_relative_to(ROOT.resolve()) or not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest()!=expected:
            failures.append(relative)
        checked+=1
    return {'section':section,'checked':checked,'passed':checked>0 and not failures,'failures':failures}


def main():
    p=argparse.ArgumentParser();p.add_argument('--section',default='all');a=p.parse_args()
    result=verify(a.section);print(json.dumps(result,indent=2))
    if not result['passed']:raise SystemExit(1)


if __name__=='__main__':main()
