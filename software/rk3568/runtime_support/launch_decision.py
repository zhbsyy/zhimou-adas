"""Relocatable launcher for the frozen v0.1.3 + continuous MMIO32 pipeline."""
from pathlib import Path
import argparse
import json
import os
import platform
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent


def command(python):
    return [str(python), '-u', str(ROOT/'Inference_and_Rules/run_vision.py'),
        '--model', str(ROOT/'Models/A_rk3568_int8.rknn'),
        '--classes', str(ROOT/'Models/classes_11.txt'), '--size', '640',
        '--lane_model', str(ROOT/'Models/R15_256x448_patched_native_int8.rknn'),
        '--lane_height', '256', '--lane_width', '448', '--lane_input_mode', 'uint8_nhwc',
        '--infer_every', '1', '--lane_hz', '30', '--q_depth', '1', '--quality', '85',
        '--runtime_backend', 'native_buffered', '--cached_display', '--publication_callback',
        '--lane_argmax', 'planar', '--run_seconds', '0', '--warmup_seconds', '10',
        '--no_oib', '--port', '18089', '--decision_port', '18088',
        '--source', 'pcie', '--pcie_dir', str(ROOT/'Camera_PCIe'), '--front_crop', '0', '0', '640', '360',
        '--camera_color_profile', str(ROOT/'Configuration/camera_color_profile.json'),
        '--stereo_capture', '--stereo_calibration', str(ROOT/'Configuration/reference_stereo_8params.json'),
        '--aeb_mode', 'video_demo', '--decision_config', str(ROOT/'Inference_and_Rules/decision_config.json'),
        '--preflight_video', str(ROOT/'Startup_Selfcheck_Video/video_20260929_112507.mp4')]


def environment():
    return {'PYTHONDONTWRITEBYTECODE': '1', 'BOARD_DEP_DIR': str(ROOT/'Camera_PCIe'),
        'YOLO_DIR': str(ROOT/'Dependencies'),
        'PYTHONPATH': os.pathsep.join(map(str, (ROOT/'Dependencies', ROOT/'Camera_PCIe'))),
        'CAP_DELAY': '0.00001', 'CAP_ROLL_FIX': '0', 'CAP_CHUNK': '0', 'CAP_TRANS': '0',
        'RK_DECISION_TOKEN': os.environ.get('RK_DECISION_TOKEN', '')}


def require_idle():
    found = []
    for path in Path('/proc').glob('[0-9]*/cmdline'):
        if int(path.parent.name) == os.getpid():
            continue
        try:
            names = [Path(v.decode(errors='replace')).name for v in path.read_bytes().split(b'\0') if v]
        except OSError:
            continue
        if any(n in ('run_vision.py', 'cap_reader.py', 'native_reader.py', 'quad_view.py') or
               n.startswith(('pcie_stream', 'process_video_r15_offline')) and n.endswith('.py') for n in names):
            found.append(path.parent.name)
    if found:
        raise RuntimeError('先在原终端 Ctrl+C 正常停止旧推理/采集，PID: ' + ','.join(found))


def local_pcie_ready():
    sys.path.insert(0, str(ROOT/'Inference_and_Rules'))
    from pcie_preflight import require_pcie_ready
    try:
        return require_pcie_ready()
    except RuntimeError as exc:
        message = str(exc).replace('sudo sh /home/linaro/03_RK3568_Perception/pcie_up.sh',
            'sudo bash "' + str(ROOT/'INITIALIZE_PCIE.sh') + '"')
        raise RuntimeError(message) from exc


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--check', action='store_true', help='resolve and verify only; no capture or NPU')
    parser.add_argument('--check-idle', action='store_true')
    parser.add_argument('--check-pcie', action='store_true', help='read-only resource check; no capture')
    args = parser.parse_args()
    if args.check_idle:
        require_idle()
        return
    if args.check_pcie:
        print('PCIE_PREFLIGHT_OK ' + json.dumps(local_pcie_ready()))
        return
    python = ROOT/'Python_Environment/venv/bin/python'
    env = environment()
    if args.check:
        print(json.dumps({'python': str(python), 'command': command(python), 'environment': env,
                          'capture_or_NPU_started': False}, ensure_ascii=False, indent=2))
        return
    if platform.system() != 'Linux' or platform.machine() != 'aarch64':
        parser.error('请在 RK3568 的 aarch64 Linux 板端运行')
    if not os.path.ismount('/userdata') or Path('/userdata') not in ROOT.parents:
        parser.error('将整个 Decision/Board_RK3568 文件夹放在已挂载的 /userdata 下')
    if not python.is_file() or not (ROOT/'Python_Environment/ENVIRONMENT_READY.json').is_file():
        parser.error('先执行 bash ./PREPARE_PYTHON_ENVIRONMENT.sh 完成运行环境备份')
    require_idle()
    subprocess.run([str(python), str(ROOT.parent/'Verification/verify_submission.py'), '--section', 'board'], check=True)
    try:
        ready = local_pcie_ready()
    except RuntimeError as exc:
        raise SystemExit(str(exc))
    result_root = ROOT/'Results'
    result_root.mkdir(exist_ok=True)
    run = result_root/('run_' + time.strftime('%Y%m%d_%H%M%S') + '_' + str(os.getpid()))
    run.mkdir()
    invocation = {'freeze': 'decision_final_20261027', 'command': command(python), 'environment': env, 'pcie': ready}
    (run/'invocation.json').write_text(json.dumps(invocation, ensure_ascii=False, indent=2), encoding='utf-8')
    print('DECISION_RESULTS=' + str(run), flush=True)
    cmd = ['env'] + [k+'='+v for k,v in env.items()] + command(python)
    if os.geteuid() != 0:
        cmd.insert(0, 'sudo')
    # Logger is outside the model pipeline. No extra camera, NPU or diagnostics.
    with (run/'live.log').open('w', encoding='utf-8', buffering=1) as log:
        proc = subprocess.Popen(cmd, cwd=ROOT/'Inference_and_Rules', stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True, encoding='utf-8', errors='replace')
        try:
            while True:
                try:
                    line = proc.stdout.readline()
                except KeyboardInterrupt:
                    # Terminal SIGINT also reaches the foreground vision process.
                    continue
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
    (run/'exit_code.txt').write_text(str(status)+'\n', encoding='ascii')
    print('DECISION_EXIT=%s\nDECISION_RESULTS=%s' % (status, run), flush=True)
    raise SystemExit(status)


if __name__ == '__main__':
    main()
