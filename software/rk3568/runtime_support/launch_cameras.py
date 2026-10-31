"""Relocatable launch paths for the collected four-camera implementation."""
from pathlib import Path
import argparse
import json
import os
import platform
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent


def command(python, mode='inference'):
    if mode == 'inference':
        return [str(python), '-u', str(ROOT/'Inference_Four_Views/quad_infer.py'),
            '--model', str(ROOT/'Models/A_rk3568_int8.rknn'),
            '--lane-model', str(ROOT/'Models/R15_256x448_patched_native_int8.rknn'),
            '--classes', str(ROOT/'Models/classes_11.txt'), '--color-profile', str(ROOT/'Configuration/camera_profiles.json'),
            '--cap-reader', str(ROOT/'Camera_Input/cap_reader.py'), '--port', '8083', '--video-fps', '20', '--seconds', '0']
    sys.path.insert(0, str(ROOT/'Camera_Input'))
    from camera_ports import resolve_usb_devices
    mapping = resolve_usb_devices()
    return [str(python), '-u', str(ROOT/'Preview_Only/quad_view.py'),
        '--cap-reader', str(ROOT/'Camera_Input/cap_reader.py'),
        '--color-profile', str(ROOT/'Configuration/camera_profiles.json'),
        '--usb-right', mapping['right'], '--usb-left', mapping['left'],
        '--usb-size', '320x240', '--usb-fps', '30', '--usb-crop', '--delay', '0.00004', '--port', '8083']


def environment():
    return {'PYTHONDONTWRITEBYTECODE': '1',
        'PYTHONPATH': os.pathsep.join(map(str, (ROOT/'Camera_Input', ROOT/'Inference_Four_Views'))),
        'CAP_DELAY': '0.00004', 'CAP_ROLL_FIX': '0', 'CAP_CHUNK': '0', 'CAP_TRANS': '0'}


def require_idle():
    found = []
    for path in Path('/proc').glob('[0-9]*/cmdline'):
        if int(path.parent.name) == os.getpid():
            continue
        try:
            words = [Path(v.decode(errors='replace')).name for v in path.read_bytes().split(b'\0') if v]
        except OSError:
            continue
        if any(n in ('quad_infer.py', 'quad_view.py', 'run_vision.py', 'cap_reader.py', 'native_reader.py',
                     'cap_fixed.py', 'probe_stereo.py', 'probe.py') or
               n.startswith(('pcie_stream', 'process_video_r15_offline')) and n.endswith('.py') for n in words):
            found.append(path.parent.name)
    if found:
        raise RuntimeError('请先在原终端 Ctrl+C 正常停止采集/推理，PID: ' + ','.join(found))


def local_pcie_ready():
    sys.path.insert(0, str(ROOT/'Inference_Four_Views'))
    from pcie_preflight import require_pcie_ready
    try:
        return require_pcie_ready()
    except RuntimeError as exc:
        raise RuntimeError(str(exc).replace('sudo sh /home/linaro/03_RK3568_Perception/pcie_up.sh',
            'sudo bash "' + str(ROOT/'INITIALIZE_PCIE.sh') + '"')) from exc


def check_files():
    # Usable on the PC as well: no native NPU/capture or external runtime paths.
    for relative in ('Camera_Input/cap_reader.py', 'Configuration/camera_profiles.json',
                     'Models/A_rk3568_int8.rknn', 'Models/R15_256x448_patched_native_int8.rknn', 'Models/classes_11.txt'):
        if not (ROOT/relative).is_file():
            raise FileNotFoundError(ROOT/relative)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=('inference', 'preview'), default='inference')
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--check-idle', action='store_true')
    parser.add_argument('--check-pcie', action='store_true')
    args = parser.parse_args()
    if args.check_idle:
        require_idle()
        return
    if args.check_pcie:
        print('PCIE_PREFLIGHT_OK ' + json.dumps(local_pcie_ready()))
        return
    check_files()
    python = ROOT/'Python_Environment/venv/bin/python'
    if args.check:
        print(json.dumps({'python': str(python), 'command': command(python) if args.mode == 'inference' else
              'Preview resolves two physical USB ports on the board', 'environment': environment(),
              'capture_or_NPU_started': False}, ensure_ascii=False, indent=2))
        return
    if platform.system() != 'Linux' or platform.machine() != 'aarch64':
        parser.error('请在 RK3568 aarch64 Linux 上启动')
    if not os.path.ismount('/userdata') or Path('/userdata') not in ROOT.parents:
        parser.error('请把整个 Cascaded_Four_Camera 放到已挂载的 /userdata 下')
    if not python.is_file() or not (ROOT/'Python_Environment/ENVIRONMENT_READY.json').is_file():
        parser.error('先执行 bash ./PREPARE_PYTHON_ENVIRONMENT.sh 备份离线环境，保留原环境直到成功')
    for name in ('FRONT_GAIN_R', 'FRONT_GAIN_G', 'FRONT_GAIN_B', 'REAR_GAIN_R', 'REAR_GAIN_G', 'REAR_GAIN_B', 'REAR_SWAP_RB'):
        if os.environ.get(name):
            parser.error('请先 unset 旧颜色环境变量：' + name)
    require_idle()
    import shutil
    if not shutil.which('v4l2-ctl'):
        parser.error('Missing v4l2-ctl')
    subprocess.run([str(python), '-B', str(ROOT.parent/'Verification/verify_submission.py'), '--section', 'board'], check=True)
    ready = local_pcie_ready()
    import fcntl
    lock = open('/tmp/cascaded_four_camera_final.lock', 'w')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    result_dir = ROOT/'Results'/('%s_%s_%s' % (args.mode, time.strftime('%Y%m%d_%H%M%S'), os.getpid()))
    result_dir.mkdir(parents=True)
    env = environment()
    invocation = {'mode': args.mode, 'command': command(python, args.mode), 'environment': env, 'pcie': ready}
    (result_dir/'invocation.json').write_text(json.dumps(invocation, ensure_ascii=False, indent=2), encoding='utf-8')
    cmd = ['env'] + [k+'='+v for k,v in env.items()] + invocation['command']
    if os.geteuid() != 0:
        cmd.insert(0, 'sudo')
    print('FOUR_CAMERA_RESULTS=' + str(result_dir), flush=True)
    with (result_dir/'live.log').open('w', encoding='utf-8', buffering=1) as log:
        proc = subprocess.Popen(cmd, cwd=ROOT/('Inference_Four_Views' if args.mode == 'inference' else 'Preview_Only'),
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, encoding='utf-8', errors='replace')
        try:
            while True:
                try:
                    line = proc.stdout.readline()
                except KeyboardInterrupt:
                    continue  # Terminal SIGINT also reaches the foreground capture/inference process.
                if not line:
                    break
                sys.stdout.write(line)
                sys.stdout.flush()
                log.write(line)
            while True:
                try:
                    status = proc.wait()
                    break
                except KeyboardInterrupt:
                    continue
        finally:
            proc.stdout.close()
    (result_dir/'exit_code.txt').write_text(str(status)+'\n', encoding='ascii')
    print('FOUR_CAMERA_EXIT=%s\nFOUR_CAMERA_RESULTS=%s' % (status, result_dir), flush=True)
    raise SystemExit(status)


if __name__ == '__main__':
    main()
