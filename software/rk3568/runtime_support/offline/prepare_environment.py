"""One-time independent copy of the working offline venv; never remove source."""
from pathlib import Path
import json
import os
import shutil
import subprocess
import sys

HERE = Path(__file__).resolve().parent
BOARD = HERE.parent


def main():
    if sys.platform != 'linux' or not os.path.ismount('/userdata') or Path('/userdata') not in HERE.parents:
        raise SystemExit('Run on RK3568 after copying Offline_Video_Processing to mounted /userdata')
    source = Path(sys.argv[1] if len(sys.argv) > 1 else '/home/linaro/yolov8_deploy/.venv-offline').resolve()
    target = HERE/'venv'
    if target.is_relative_to(source) or source.is_relative_to(target):
        raise SystemExit('原环境和目标环境不能相互包含，未复制。')
    if target.exists():
        raise SystemExit('已有 Python_Environment/venv，未覆盖。若 ENVIRONMENT_READY.json 存在可直接启动；否则先保留该目录排查。')
    if not (source/'bin/python').exists() or not (source/'pyvenv.cfg').is_file():
        raise SystemExit('找不到原离线 venv，请保留原目录并用参数传入正确路径。')
    total = sum(p.stat().st_size for p in source.rglob('*') if p.is_file())
    if shutil.disk_usage(HERE).free < total * 1.3 + 256*1024**2:
        raise SystemExit('独立备份空间不足；不会删除或修改原 Python 环境。')
    print('复制离线环境到 ' + str(target), flush=True)
    # Dereference links so site-packages and the interpreter do not keep links
    # into the old project. Linux system libraries/base Python remain prerequisites.
    shutil.copytree(source, target, symlinks=False)
    code = '''import ctypes,json,sys
import cv2,numpy
from pathlib import Path
from rknnlite.api import RKNNLite
import rknnlite
ctypes.CDLL('/usr/lib/librknnrt.so')
old=Path(sys.argv[1]).resolve()
bad=[p for p in sys.path if p and Path(p).resolve().is_relative_to(old)]
assert not bad, ('old venv still referenced',bad)
for m in (cv2,numpy,rknnlite):
    assert not Path(m.__file__).resolve().is_relative_to(old),m.__file__
print('ENV_JSON '+json.dumps({'python':sys.version,'prefix':sys.prefix,'opencv':cv2.__version__,'numpy':numpy.__version__,
 'rknnlite':rknnlite.__file__,'rknnrt':'/usr/lib/librknnrt.so','old_venv_imports':bad}))
'''
    env = dict(os.environ)
    env.pop('PYTHONPATH', None)
    check = subprocess.run([str(target/'bin/python'), '-I', '-B', '-c', code, str(source)],
                           capture_output=True, text=True, env=env)
    (HERE/'environment_check.log').write_text(check.stdout+check.stderr, encoding='utf-8')
    if check.returncode:
        raise SystemExit('备份自检失败，原环境完整保留；查看 environment_check.log，勿删除旧环境。')
    checked = json.loads(next(line[len('ENV_JSON '):] for line in check.stdout.splitlines() if line.startswith('ENV_JSON ')))
    checked.update(source=str(source), copied_environment=str(target), source_unchanged=True,
                   scope='Python packages and executable; board OS/native system libraries remain installed prerequisites')
    (HERE/'ENVIRONMENT_READY.json').write_text(json.dumps(checked, ensure_ascii=False, indent=2), encoding='utf-8')
    print('ENVIRONMENT_READY ' + str(HERE/'ENVIRONMENT_READY.json'), flush=True)


if __name__ == '__main__':
    main()
