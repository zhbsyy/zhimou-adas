"""Portable RK3568 batch launcher. No camera, CARLA, sockets or vehicle control."""
from pathlib import Path
import argparse
import datetime
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parent
MODULE = ROOT.parent


def parse_args(argv=None):
    defaults = json.loads((ROOT/'Configuration/offline_defaults.json').read_text(encoding='utf-8'))
    p = argparse.ArgumentParser(description='Process a single-view file with the frozen detector and R15 on RK3568.')
    p.add_argument('--video', type=Path, default=ROOT/'Input_Videos/video_20260929_112507.mp4')
    p.add_argument('--output_dir', type=Path, default=ROOT/'Results', help='Parent directory on mounted /userdata; each run gets its own subdirectory')
    p.add_argument('--class10_conf', type=float, default=defaults['class10_conf'])
    p.add_argument('--seconds', type=float, default=defaults['seconds'], help='0 = entire remaining video')
    p.add_argument('--start', type=float, default=defaults['start'])
    p.add_argument('--stride', type=int, default=defaults['stride'])
    p.add_argument('--max_width', type=int, default=defaults['max_width'], help='0 keeps source resolution')
    p.add_argument('--check', action='store_true', help='Only check files, hashes and command; works on PC without NPU')
    a = p.parse_args(argv)
    if not 0.25 <= a.class10_conf <= 1 or a.seconds < 0 or a.start < 0 or a.stride < 1:
        p.error('invalid class10_conf/seconds/start/stride')
    if a.max_width and a.max_width < 320:
        p.error('max_width must be 0 or at least 320')
    return a


def command(python, args, output):
    return [str(python), '-B', '-u', str(ROOT/'Inference_and_Postprocess/process_video_r15_offline.py'),
        '--video', str(args.video.resolve()), '--output', str(output),
        '--model', str(ROOT/'Models/A_rk3568_int8.rknn'),
        '--classes', str(ROOT/'Models/classes_11.txt'), '--size', '640',
        '--lane_model', str(ROOT/'Models/R15_256x448_patched_native_int8.rknn'),
        '--lane_height', '256', '--lane_width', '448', '--lane_input_mode', 'uint8_nhwc',
        '--lane_conf', '0.6', '--class10_conf', str(args.class10_conf),
        '--stride', str(args.stride), '--seconds', str(args.seconds),
        '--start', str(args.start), '--max_width', str(args.max_width)]


def verify_board():
    import importlib.util
    path = MODULE/'Verification/verify_submission.py'
    spec = importlib.util.spec_from_file_location('offline_submission_verify', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    result = module.verify('board')
    if not result['passed']:
        raise RuntimeError('Offline files failed integrity verification: '+json.dumps(result))
    return result


def require_idle():
    # Inspect, never terminate another module. NPU jobs must run one at a time.
    pattern = re.compile(r'(run_vision\.py|quad_infer\.py|quad_view\.py|pcie_stream[^ /]*\.py|process_video_r15_offline[^ /]*\.py|run_carla_aeb\.py|probe_stereo\.py)')
    jobs = subprocess.check_output(['ps', '-eo', 'pid=,args='], text=True).splitlines()
    busy = []
    for line in jobs:
        parts = line.strip().split(None, 1)
        if len(parts) == 2 and int(parts[0]) != os.getpid() and pattern.search(parts[1]):
            busy.append(line.strip())
    if busy:
        raise RuntimeError('Stop existing capture/inference normally with Ctrl+C before offline processing:\n'+'\n'.join(busy))


def restore_owner(path):
    if not hasattr(os, 'chown') or not os.environ.get('SUDO_UID'):
        return
    uid, gid = int(os.environ['SUDO_UID']), int(os.environ['SUDO_GID'])
    for p in [path, *path.iterdir()]:
        os.chown(p, uid, gid)


def main(argv=None):
    args = parse_args(argv)
    verified = verify_board()
    if not args.video.is_file():
        raise SystemExit('Input video does not exist: '+str(args.video))
    if args.check:
        print(json.dumps({'verified': verified, 'command': command('Python_Environment/venv/bin/python', args, '<new run>/result.mp4'),
            'uses_network_or_PCIe': False}, ensure_ascii=False, indent=2))
        return
    if sys.platform != 'linux' or not os.path.ismount('/userdata') or not ROOT.is_relative_to(Path('/userdata')):
        raise SystemExit('Copy this module to mounted /userdata and run on RK3568 Linux.')
    output_root = args.output_dir.expanduser().resolve()
    if not output_root.is_relative_to(Path('/userdata')):
        raise SystemExit('output_dir must be under mounted /userdata, not /tmp or /home.')
    require_idle()
    output_root.mkdir(parents=True, exist_ok=True)
    if shutil.disk_usage(output_root).free < 1024**3:
        raise SystemExit('Less than 1 GiB free on /userdata; free space before processing. No existing results deleted.')
    python = ROOT/'Python_Environment/venv/bin/python'
    if not python.is_file() or not (ROOT/'Python_Environment/ENVIRONMENT_READY.json').is_file():
        raise SystemExit('Run PREPARE_PYTHON_ENVIRONMENT.sh first.')
    run = Path(tempfile.mkdtemp(prefix=datetime.datetime.now().strftime('run_%Y%m%d_%H%M%S_'), dir=output_root))
    cmd = command(python, args, run/'result.mp4')
    (run/'invocation.json').write_text(json.dumps({'command': cmd, 'class10_min_confidence': args.class10_conf,
        'other_min_confidence': 0.25, 'input': str(args.video.resolve()), 'verified_board': verified}, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print('OFFLINE_RESULTS='+str(run), flush=True)
    env = dict(os.environ)
    env.pop('PYTHONPATH', None)
    env['PYTHONDONTWRITEBYTECODE'] = '1'
    try:
        with (run/'run.log').open('w', encoding='utf-8', buffering=1) as log:
            with subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                  encoding='utf-8', errors='replace', env=env, cwd=ROOT) as child:
                try:
                    for line in child.stdout:
                        print(line, end='', flush=True)
                        log.write(line)
                except KeyboardInterrupt:
                    # Ctrl+C normally reaches the processor's process group too;
                    # let it close the MP4 and save its partial summary.
                    child.wait()
                code = child.wait()
        print('OFFLINE_EXIT='+str(code), flush=True)
        print('OFFLINE_RESULTS='+str(run), flush=True)
    finally:
        restore_owner(run)
    if code:
        raise SystemExit(code)


if __name__ == '__main__':
    main()
