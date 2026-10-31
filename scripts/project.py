"""Single project entry: prepare, PC console, RK profiles, CARLA and checks."""
from pathlib import Path
import argparse
import hashlib
import json
import os
import platform
import socket
import subprocess
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parent))
from runtime_layout import ROOT,PROFILES,materialize,bind_environment,bind_results


def settings():return json.loads((ROOT/'configs/pc.json').read_text(encoding='utf-8'))


def verify():
    failures=[];count=0
    for line in (ROOT/'MANIFEST.sha256').read_text(encoding='utf-8').splitlines():
        expected,name=line.split('  ',1);p=(ROOT/name).resolve()
        if not p.is_relative_to(ROOT) or not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest()!=expected:failures.append(name)
        count+=1
    print(json.dumps({'checked':count,'passed':count>0 and not failures,'failures':failures},ensure_ascii=False,indent=2))
    if not count or failures:raise SystemExit(1)


def require_rk():
    if platform.system()!='Linux' or platform.machine()!='aarch64' or not os.path.ismount('/userdata') or not ROOT.is_relative_to(Path('/userdata')):
        raise SystemExit('Run on RK3568 aarch64 Linux after uploading this whole project under mounted /userdata.')


def require_idle():
    if sys.platform!='linux':return
    busy=[]
    names={'run_vision.py','quad_infer.py','quad_view.py','cap_reader.py','native_reader.py','run_carla_aeb.py','process_video_r15_offline.py'}
    for file in Path('/proc').glob('[0-9]*/cmdline'):
        if file.parent.name==str(os.getpid()):continue
        try:words=[Path(x.decode(errors='replace')).name for x in file.read_bytes().split(b'\0') if x]
        except OSError:continue
        if any(w in names or w.startswith('pcie_stream') and w.endswith('.py') for w in words):busy.append(file.parent.name)
    if busy:raise SystemExit('Stop existing RK inference/capture normally with Ctrl+C first; PID='+','.join(busy))


def carla_command(board,python):
    cfg=settings()
    return [str(python),'-B','-u',str(board/'run_carla_aeb.py'),
        '--source','carla','--carla_url','http://%s:%s/stream'%(cfg['pc_ip'],cfg['carla_input_port']),
        '--carla_decode_backend','mpp_rga_cached','--pc_ip',cfg['pc_ip'],
        '--model',str(board/'Models/A_rk3568_int8.rknn'),'--classes',str(board/'Models/classes_11.txt'),'--size','640',
        '--lane_model',str(board/'Models/R15_256x448_patched_native_int8.rknn'),
        '--p2_mpp_dir',str(board/'Mpp_Decoder'),'--preflight_video',str(board/'Selfcheck_Video/video_20260929_112507.mp4'),
        '--lane_height','256','--lane_width','448','--lane_input_mode','uint8_nhwc',
        '--infer_every','1','--lane_hz','30','--q_depth','1','--quality','85','--port','18084',
        '--runtime_backend','native_buffered','--cached_display','--publication_callback','--lane_argmax','planar',
        '--run_seconds','0','--warmup_seconds','10']


def run_rk(profile,action,extra):
    if action!='check':require_rk();require_idle()
    module=materialize('rk3568_'+profile);board=module/'Board_RK3568'
    if action=='check':
        entry={'decision':'launch_decision.py','four_camera':'launch_cameras.py','offline':'launch_offline.py'}.get(profile)
        if entry:subprocess.run([sys.executable,'-B',str(board/entry),'--check'],cwd=board,check=True)
        else:print(json.dumps({'command':carla_command(board,ROOT/'environment/rk3568/venv/bin/python'),'NPU_started':False},ensure_ascii=False,indent=2))
        return
    python=bind_environment(module);results=bind_results(module,profile)
    if action=='init-pcie':
        if profile not in ('decision','four_camera'):raise SystemExit('This mode does not use PCIe.')
        cmd=['bash',str(board/'INITIALIZE_PCIE.sh')]
        if os.geteuid()!=0:cmd.insert(0,'sudo')
        raise SystemExit(subprocess.call(cmd,cwd=board))
    if action=='prepare-decoder':
        if profile!='carla_navigation':raise SystemExit('MPP/RGA decoder preparation is only for CARLA input.')
        raise SystemExit(subprocess.call(['bash',str(board/'PREPARE_DECODER.sh')],cwd=board))
    if profile=='carla_navigation':
        env=dict(os.environ,BOARD_DEP_DIR=str(board),YOLO_DIR=str(board),PYTHONDONTWRITEBYTECODE='1')
        subprocess.run([str(python),'-B',str(board/'check_start.py')],env=env,cwd=board,check=True)
        cmd=carla_command(board,python)
        if os.geteuid()!=0:cmd=['sudo','env','PYTHONDONTWRITEBYTECODE=1','BOARD_DEP_DIR='+str(board),'YOLO_DIR='+str(board)]+cmd
        run=results/time.strftime('run_%Y%m%d_%H%M%S');run.mkdir(exist_ok=False)
        (run/'invocation.json').write_text(json.dumps({'command':cmd},ensure_ascii=False,indent=2),encoding='utf-8')
        print('CARLA_RESULTS='+str(run),flush=True)
        with (run/'live.log').open('w',encoding='utf-8',buffering=1) as log:
            with subprocess.Popen(cmd,cwd=board,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,encoding='utf-8',errors='replace') as child:
                while True:
                    try:line=child.stdout.readline()
                    except KeyboardInterrupt:continue
                    if not line:break
                    print(line,end='',flush=True);log.write(line)
                while True:
                    try:code=child.wait();break
                    except KeyboardInterrupt:continue
        raise SystemExit(code)
    entry={'decision':'START_DECISION.sh','four_camera':'START_INFERENCE.sh','offline':'START_OFFLINE.sh'}[profile]
    print('RESULTS='+str(results),flush=True)
    os.execv('/bin/bash',['bash',str(board/entry),*extra])


def run_pc(profile,check=False):
    cfg=settings();python=Path(cfg['python_executable'])
    cmd=[str(python),'-B',str(ROOT/'software/pc/launch_console.py'),'--profile',profile]
    if check:
        subprocess.run(cmd+['--check'],check=True);return
    if platform.system()!='Windows':raise SystemExit('PC console is for Windows.')
    probe=socket.socket();probe.settimeout(.5)
    try:occupied=probe.connect_ex(('127.0.0.1',18090))==0
    finally:probe.close()
    if occupied:raise SystemExit('A console already uses18090. Close its window normally before starting another profile.')
    subprocess.run(cmd+['--check'],check=True)
    gui=python.with_name('pythonw.exe')
    if not gui.is_file():gui=python
    subprocess.Popen([str(gui),*cmd[1:]],cwd=ROOT,creationflags=subprocess.CREATE_NO_WINDOW)
    print('Console launch requested: '+profile+'; see results/pc for logs.')


def start_carla():
    if platform.system()!='Windows':raise SystemExit('Start CARLA on the Windows PC.')
    cfg=settings();engine=Path(cfg['carla_root'])/'CarlaUE4.exe'
    if not engine.is_file():raise SystemExit('CARLA missing; edit configs/pc.json')
    tasklist=subprocess.check_output(['tasklist','/FO','CSV','/NH','/FI','IMAGENAME eq CarlaUE4*'],text=True,errors='replace')
    if 'CarlaUE4-Win64-Shipping.exe' in tasklist or 'CarlaUE4.exe' in tasklist:
        print('CARLA engine already running.');return
    subprocess.Popen([str(engine),'-quality-level=Low','-RenderOffScreen','-fps=20'],cwd=engine.parent,creationflags=subprocess.CREATE_NO_WINDOW)
    print('CARLA launch requested. Wait for the map before connecting the console cameras.')


def main():
    p=argparse.ArgumentParser(description='FPGA ADAS project entry')
    sub=p.add_subparsers(dest='command',required=True)
    sub.add_parser('verify')
    prep=sub.add_parser('prepare');prep.add_argument('layout')
    env=sub.add_parser('prepare-environment');env.add_argument('--source',default='/home/linaro/yolov8_deploy/.venv-offline')
    rk=sub.add_parser('rk');rk.add_argument('--profile',choices=PROFILES,required=True);rk.add_argument('--action',choices=('run','check','init-pcie','prepare-decoder'),default='run')
    pc=sub.add_parser('pc');pc.add_argument('--profile',choices=PROFILES[:3],required=True);pc.add_argument('--check',action='store_true')
    tool=sub.add_parser('pc-tool');tool.add_argument('kind',choices=('inspect-video','cameras','brakes'));tool.add_argument('video',nargs='?');tool.add_argument('--check',action='store_true')
    sub.add_parser('carla')
    a,extra=p.parse_known_args()
    if extra and a.command!='rk':p.error('Unknown arguments: '+' '.join(extra))
    if a.command=='verify':verify()
    elif a.command=='prepare':print(materialize(a.layout))
    elif a.command=='prepare-environment':
        require_rk();require_idle()
        subprocess.run([sys.executable,'-B',str(ROOT/'environment/rk3568/prepare_environment.py'),a.source],check=True)
    elif a.command=='rk':run_rk(a.profile,a.action,extra)
    elif a.command=='pc':run_pc(a.profile,a.check)
    elif a.command=='pc-tool':
        cmd=[sys.executable,'-B',str(ROOT/'software/pc/run_tool.py'),a.kind]
        if a.video:cmd.append(a.video)
        if a.check:cmd.append('--check')
        raise SystemExit(subprocess.call(cmd,cwd=ROOT))
    elif a.command=='carla':start_carla()


if __name__=='__main__':main()
