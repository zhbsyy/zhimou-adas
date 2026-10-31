"""Run the archived PC tools with the project-local dependency layout."""
from pathlib import Path
import argparse
import importlib.util
import json
import shutil
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts'))
from runtime_layout import materialize


def prepare(kind):
    if kind == 'inspect-video':
        source = ROOT / 'software/pc/tools/inspect_result.py'
        runtime = None
    else:
        runtime = materialize('pc')
        sys.path[:0] = [str(runtime), str(runtime / 'Console_Application'),
                        str(runtime / 'Runtime_Support/CARLA_PythonAPI')]
        source = runtime / {'cameras': 'probe_cameras.py', 'brakes': 'check_actuator.py'}[kind]
    spec = importlib.util.spec_from_file_location('submission_tool_' + kind.replace('-', '_'), source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, runtime


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('kind', choices=('inspect-video', 'cameras', 'brakes'))
    parser.add_argument('video', nargs='?', type=Path)
    parser.add_argument('--check', action='store_true', help='Import dependencies only; no CARLA connection/actors')
    args = parser.parse_args()
    if args.kind == 'inspect-video' and not args.check and args.video is None:
        parser.error('inspect-video requires the downloaded result.mp4 path')
    if args.kind != 'inspect-video' and args.video is not None:
        parser.error('Video argument is only used by inspect-video')
    module, runtime = prepare(args.kind)
    if args.check:
        print(json.dumps({'tool': args.kind, 'module': module.__file__,
                          'CARLA_connection_or_actors_started': False}, ensure_ascii=False))
        return
    if runtime:
        (runtime / 'Results').mkdir(exist_ok=True)
        module.main()
        destination = ROOT / 'results/pc/diagnostics'
        destination.mkdir(parents=True, exist_ok=True)
        names = ('camera_probe.json', 'camera_probe.jpg') if args.kind == 'cameras' else ('PC_ACTUATOR_CHECK.json',)
        for name in names:
            source = runtime / 'Results' / name
            if source.is_file():
                shutil.copy2(source, destination / name)
        print('RESULTS=' + str(destination))
    else:
        # The unchanged inspector owns its original CLI parser.
        sys.argv = [module.__file__, str(args.video.resolve())]
        module.main()


if __name__ == '__main__':
    main()
