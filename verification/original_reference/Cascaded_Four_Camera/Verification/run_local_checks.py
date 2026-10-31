"""CPU-only software checks, imports and source parsing. Never calls real NPU/JTAG."""
from pathlib import Path
import ast
import json
import os
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
BOARD = ROOT/'Board_RK3568'


def main():
    env = dict(os.environ)
    env['PYTHONDONTWRITEBYTECODE'] = '1'
    env['PYTHONPATH'] = os.pathsep.join(map(str, (BOARD/'Camera_Input', BOARD/'Inference_Four_Views')))
    rows = []
    jobs = [('inference_contracts', ['-m', 'unittest', 'discover', '-s', str(BOARD/'Inference_Four_Views/tests'), '-v']),
            ('collected_paths', ['-m', 'unittest', 'discover', '-s', str(ROOT/'Verification'), '-p', 'test_collected_paths.py', '-v']),
            ('board_launch_paths', [str(BOARD/'launch_cameras.py'), '--check']),
            ('pc_console_imports', [str(ROOT/'PC_Windows/launch_console.py'), '--check'])]
    for name, args in jobs:
        result = subprocess.run([sys.executable, '-B']+args, cwd=ROOT, env=env,
            capture_output=True, text=True, encoding='utf-8', errors='replace', timeout=60)
        rows.append({'check': name, 'exit_code': result.returncode, 'output': result.stdout+result.stderr})
        print(name+': '+str(result.returncode), flush=True)
        if result.returncode:
            print(rows[-1]['output'])
    count = 0
    for path in ROOT.rglob('*.py'):
        if 'Results' not in path.parts and 'venv' not in path.parts:
            ast.parse(path.read_text(encoding='utf-8-sig'), filename=str(path))
            count += 1
    data = {'scope': 'CPU software contracts and relocation, not RK3568 real capture/model or hardware JPEG performance',
            'checks': rows, 'python_sources_parsed': count, 'passed': all(row['exit_code'] == 0 for row in rows),
            'GUI_opened': False, 'board_processes_or_current_console_changed': False}
    (ROOT/'Verification/LOCAL_VALIDATION.json').write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({'passed': data['passed'], 'python_sources_parsed': count}, ensure_ascii=False))
    if not data['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
