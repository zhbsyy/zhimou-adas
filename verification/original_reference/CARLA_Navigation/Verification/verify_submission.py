"""Offline hashes only; no control, network or inference."""
from pathlib import Path
import hashlib
import json

ROOT=Path(__file__).resolve().parents[1]


def main():
    checked,failures=0,[]
    for line in (ROOT/'MANIFEST.sha256').read_text(encoding='utf-8').splitlines():
        expected,name=line.split('  ',1)
        path=(ROOT/name).resolve()
        if not path.is_relative_to(ROOT) or not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=expected:
            failures.append(name)
        checked+=1
    print(json.dumps({'checked':checked,'passed':checked>0 and not failures,'failures':failures},indent=2))
    if not checked or failures:
        raise SystemExit(1)


if __name__=='__main__':
    main()
