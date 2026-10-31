"""Run only original test_*.py cases; never run the live integration scripts."""
from pathlib import Path
import os
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from runtime_layout import materialize


def main():
    pc=materialize('pc');materialize('pc_tests')
    rk=materialize('rk3568_carla_navigation')/'Board_RK3568'
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',
             PYTHONPATH=os.pathsep.join(map(str,(pc/'Console_Application',pc/'Runtime_Support/CARLA_PythonAPI',pc,rk))))
    # Isolated process: Qt offscreen in tests does not affect the live console.
    code=subprocess.call([sys.executable,'-B','-m','unittest','discover','-s',str(pc/'Tests'),'-p','test_*.py','-v'],env=env,cwd=pc)
    raise SystemExit(code)


if __name__=='__main__':main()
