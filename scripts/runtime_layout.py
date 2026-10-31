"""Project-local source layouts for unchanged legacy relative imports."""
from pathlib import Path
import hashlib
import json
import os
import shutil

ROOT=Path(__file__).resolve().parents[1]
PROFILES=('decision','four_camera','carla_navigation','offline')


def inside(base,relative):
    p=(base/relative).resolve()
    if not p.is_relative_to(base.resolve()):
        raise ValueError('Path outside requested root: '+str(relative))
    return p


def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def layout(name):
    return json.loads((ROOT/'configs/layouts'/('%s.json'%name)).read_text(encoding='utf-8'))['files']


def runtime_root():
    return Path(os.environ.get('FPGA_ADAS_RUNTIME_DIR',str(ROOT/'.runtime'))).resolve()


def verify_sources(name):
    failures=[]
    for destination,item in layout(name).items():
        source=inside(ROOT,item['source'])
        if not source.is_file() or digest(source)!=item['sha256']:
            failures.append(item['source'])
    if failures:raise RuntimeError('Source changed or missing: '+', '.join(sorted(set(failures))))


def materialize(name):
    """Copy only the selected layout; never delete old data or modify algorithms."""
    verify_sources(name)
    if name in ('pc','pc_tests'):target=runtime_root()/'pc'
    elif name.startswith('rk3568_'):target=runtime_root()/'rk3568'/name[len('rk3568_'):]
    elif name.startswith('hardware_'):target=runtime_root()/'hardware'/name[len('hardware_'):]
    else:raise ValueError(name)
    entries=layout(name)
    for relative,item in entries.items():
        src=inside(ROOT,item['source']);dst=inside(target,relative)
        dst.parent.mkdir(parents=True,exist_ok=True)
        if dst.is_file():
            if digest(dst)!=item['sha256']:
                raise RuntimeError('Runtime file was modified; preserve it and stop before rebuilding: '+str(dst))
        else:shutil.copy2(src,dst)
    if name.startswith('rk3568_'):
        verifier=target/'Verification/verify_submission.py'
        verifier.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(ROOT/'scripts/runtime_verify.py',verifier)
        text=''.join(v['sha256']+'  '+k+'\n' for k,v in sorted(entries.items()))
        (target/'MANIFEST.sha256').write_text(text,encoding='utf-8')
    return target


def bind_environment(module):
    """One actual Linux venv shared by all four runtime layouts."""
    source=ROOT/'environment/rk3568'
    if not (source/'ENVIRONMENT_READY.json').is_file() or not (source/'venv/bin/python').is_file():
        raise RuntimeError('First run bash scripts/rk3568/PREPARE_ENVIRONMENT.sh')
    target=module/'Board_RK3568/Python_Environment'
    target.mkdir(parents=True,exist_ok=True)
    dst=target/'venv'
    if dst.is_symlink():
        if dst.resolve()!=(source/'venv').resolve():raise RuntimeError('Unexpected runtime Python symlink: '+str(dst))
    elif dst.exists():raise RuntimeError('Unexpected private runtime venv: '+str(dst))
    else:dst.symlink_to(source/'venv',target_is_directory=True)
    shutil.copy2(source/'ENVIRONMENT_READY.json',target/'ENVIRONMENT_READY.json')
    return source/'venv/bin/python'


def bind_results(module,profile):
    source=ROOT/'results/rk3568'/profile
    source.mkdir(parents=True,exist_ok=True)
    dst=module/'Board_RK3568/Results'
    if dst.is_symlink():
        if dst.resolve()!=source.resolve():raise RuntimeError('Unexpected result symlink')
    elif dst.exists():raise RuntimeError('Runtime Results already exists; preserve it: '+str(dst))
    else:dst.symlink_to(source,target_is_directory=True)
    return source
