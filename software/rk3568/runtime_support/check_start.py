from pathlib import Path
import hashlib
import json
import os
import time
from urllib.request import build_opener, ProxyHandler
from runtime_backend import check_model


def main():
    here = Path(__file__).resolve().parent
    for line in (here/'PACKAGE.sha256').read_text(encoding='utf-8').splitlines():
        digest, name = line.split('  ', 1)
        path = (here/name).resolve()
        if not path.is_relative_to(here) or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            raise RuntimeError('PACKAGE_MISMATCH '+name)
    active = []
    for path in Path('/proc').glob('[0-9]*/cmdline'):
        if int(path.parent.name) == os.getpid():
            continue
        try:
            words = [Path(w.decode(errors='replace')).name for w in path.read_bytes().split(b'\0') if w]
        except OSError:
            continue
        if any(n in ('quad_infer.py','quad_view.py','run_vision.py','run_carla_aeb.py','cap_reader.py') or
                   n.startswith(('pcie_stream','process_video_r15_offline')) and n.endswith('.py') for n in words):
            active.append(path.parent.name)
    if active:
        raise RuntimeError('请先Ctrl+C正常停止其他板端采集/推理，PID='+','.join(active))
    for name in ('A_rk3568_int8.rknn', 'R15_256x448_patched_native_int8.rknn'):
        check_model(here/'Models'/name)
    if not os.access(here/'Mpp_Decoder/mpp_native_stage',os.X_OK):
        raise RuntimeError('请先在本目录运行 bash PREPARE_DECODER.sh，编译本模块的MPP/RGA解码器')
    classes = (here/'Models/classes_11.txt').read_text(encoding='utf-8-sig').splitlines()
    expected = ['person','rider','car','truck','bus','train','motor','bike','traffic light','traffic sign','crosswalk']
    if [line.strip() for line in classes if line.strip()] != expected:
        raise RuntimeError('classes_11.txt类别顺序与当前模型/AEB不匹配')
    opener = build_opener(ProxyHandler({}))
    with opener.open('http://192.168.2.10:8081/status', timeout=4) as response:
        status = json.load(response)
    if status.get('cameras') != 4 or status.get('ready') is not True:
        raise RuntimeError('CARLA四相机推流尚未就绪：'+str(status))
    time.sleep(.3)
    with opener.open('http://192.168.2.10:8081/status',timeout=4) as response:
        next_status = json.load(response)
    if (next_status.get('ready') is not True or next_status.get('seq',0)<=status.get('seq',0)
        or any(next_status.get('counts',{}).get(key,0)<=status.get('counts',{}).get(key,0)
               for key in ('front','right','rear','left'))):
        raise RuntimeError('CARLA相机或视频发布计数未前进')
    print('CARLA_AEB_PREFLIGHT_OK '+json.dumps(status), flush=True)


if __name__ == '__main__':
    main()
