"""Offline packaging check. No CARLA/RK connection, serial open or NPU run."""
from pathlib import Path
import ast
import datetime
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def static_files():
    for p in ROOT.rglob('*'):
        parts = p.relative_to(ROOT).parts
        if (p.is_file() and 'Results' not in parts and '__pycache__' not in parts
                and p.suffix not in ('.pyc', '.log') and '.build' not in parts):
            yield p


def main():
    record = {'recorded_local': datetime.datetime.now().astimezone().isoformat(),
              'scope': 'Copied directory + local imports + threshold boundary; no live control or NPU',
              'passed': False}
    temp_parent = Path(tempfile.gettempdir()).resolve()
    scratch = tempfile.TemporaryDirectory(prefix='carla_submission_check_', dir=temp_parent)
    scratch_path = Path(scratch.name).resolve()
    if scratch_path.parent != temp_parent or not scratch_path.name.startswith('carla_submission_check_'):
        raise RuntimeError('Unexpected temporary target')
    try:
        isolated = scratch_path / 'CARLA_Navigation'
        copied = 0
        for src in static_files():
            dst = isolated / src.relative_to(ROOT)
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dst)
            copied += 1
        env = dict(os.environ, PYTHONPATH='', PYTHONDONTWRITEBYTECODE='1')
        pc = subprocess.run([sys.executable, '-B', str(isolated/'PC_Windows/launch_console.py'), '--check'],
                            cwd=scratch_path, env=env, capture_output=True, text=True,
                            encoding='utf-8', errors='replace', timeout=45)
        if pc.returncode:
            raise RuntimeError('Relocated PC import: ' + pc.stdout + pc.stderr)
        pc_data = json.loads(pc.stdout.strip().splitlines()[-1])
        assert Path(pc_data['dashboard']).resolve().is_relative_to(isolated)
        rk = isolated/'Board_RK3568'
        env.update(BOARD_DEP_DIR=str(rk), YOLO_DIR=str(rk))
        code = r'''
import importlib,json,sys
from pathlib import Path
import numpy as np
root=Path(sys.argv[1]).resolve()
sys.path[:0]=[str(root),str(root/'Mpp_Decoder')]
names=['run_carla_aeb','pcie_stream_r15_async','pcie_stream_r15_profile','pcie_post',
       'r15_front_fast','runtime_backend','native_api','hw_jpeg','cached_display',
       'overlay_roi_candidate','postprocess_candidate','publication_candidate',
       'trial_metrics','carla_aeb_link','aeb_status','p2_fresh_source',
       'pcie_frame','frame_align','stereo_range','carla_source','video_source','check_start']
for name in names:
    module=importlib.import_module(name)
    assert Path(module.__file__).resolve().is_relative_to(root),(name,module.__file__)
entry=sys.modules['run_carla_aeb']
entry.pipeline.main=lambda:None
entry.pipeline.base.decode_auto=lambda:np.array([[0,0,10,10,.29,10],[0,0,10,10,.30,10],
                                               [0,0,10,10,.25,0],[0,0,10,10,.31,10]],np.float64)
entry.main()
out=entry.pipeline.base.decode_auto()
assert out[:,4].tolist()==[.30,.25,.31],out
assert entry.pipeline.base.STATS['class10_min_confidence']==.30
print(json.dumps({'owned_imports':len(names),'class10_threshold':.30,
                  'other_class_025_retained':True,'NPU_initialized':False}))
'''
        board = subprocess.run([sys.executable, '-B', '-c', code, str(rk)], cwd=scratch_path,
                               env=env, capture_output=True, text=True, encoding='utf-8',
                               errors='replace', timeout=45)
        if board.returncode:
            raise RuntimeError('Relocated RK imports: ' + board.stdout + board.stderr)
        board_data=json.loads(board.stdout.strip().splitlines()[-1])
        parsed=0
        for path in static_files():
            if path.suffix=='.py':
                ast.parse(path.read_text(encoding='utf-8-sig'),filename=str(path))
                parsed+=1
        package_failures=[]
        package_count=0
        for ln in (rk/'PACKAGE.sha256').read_text(encoding='utf-8').splitlines():
            expected,rel=ln.split('  ',1)
            p=(rk/rel).resolve()
            if not p.is_relative_to(rk) or hashlib.sha256(p.read_bytes()).hexdigest()!=expected:
                package_failures.append(rel)
            package_count+=1
        assert not package_failures,package_failures
        record.update(passed=True,copied_files=copied,python_sources_parsed=parsed,
                      pc_entry_relocated=True,pc_dashboard_resolved_inside_module=True,
                      RK_import_check=board_data,board_package_files_checked=package_count,
                      no_live_services_started=True,installed_python_CARLA_SDK_still_required=True,
                      platform_only_module_not_imported='Mpp_Decoder/p2_native_mpp_source.py requires Linux fcntl; source parsed and hash checked only',
                      note='Imports only on Windows; not an aarch64 MPP build or RKNN inference test.')
    finally:
        assert Path(scratch.name).resolve()==scratch_path and scratch_path.parent==temp_parent
        scratch.cleanup()
    output=ROOT/'Verification/PACKAGING_CHECK.json'
    output.write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(record,ensure_ascii=False))


if __name__=='__main__':
    main()
