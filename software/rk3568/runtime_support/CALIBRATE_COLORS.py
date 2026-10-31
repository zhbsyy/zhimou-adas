"""Calibrate the live four-camera input and keep its startup manifest consistent.

Run on RK while the four-camera program is running and the selected central
ROI contains neutral paper. This client never rewrites capture/inference code.
"""
from pathlib import Path
import argparse
import datetime
import hashlib
import json
import os
import urllib.error
from urllib.request import Request, build_opener, ProxyHandler

ROOT = Path(__file__).resolve().parent
PROFILE_NAME = 'Board_RK3568/Configuration/camera_profiles.json'


def sha256(blob):
    return hashlib.sha256(blob).hexdigest()


def sync_manifest(manifest, old_profile_hash, new_profile_hash):
    """Change precisely one known profile entry; preserve all other entries."""
    manifest = Path(manifest)
    lines = manifest.read_text(encoding='utf-8').splitlines(keepends=True)
    matches = [i for i, line in enumerate(lines)
               if line.rstrip('\r\n').split('  ', 1)[-1] == PROFILE_NAME]
    if len(matches) != 1:
        raise ValueError('Expected exactly one camera-profile entry in MANIFEST.sha256')
    index = matches[0]
    current = lines[index].split('  ', 1)[0]
    if current != old_profile_hash:
        raise ValueError('Manifest changed or profile was already untracked; no manifest update')
    if any(len(value) != 64 or any(c not in '0123456789abcdef' for c in value)
           for value in (old_profile_hash, new_profile_hash)):
        raise ValueError('Invalid profile digest')
    ending = '\r\n' if lines[index].endswith('\r\n') else '\n'
    lines[index] = new_profile_hash + '  ' + PROFILE_NAME + ending
    temporary = manifest.with_name(manifest.name + '.color_pending')
    try:
        with temporary.open('w', encoding='utf-8', newline='') as stream:
            stream.write(''.join(lines))
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, manifest)
    finally:
        if temporary.exists():
            temporary.unlink()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('camera', choices=('front', 'rear'))
    parser.add_argument('--confirm-neutral', action='store_true')
    parser.add_argument('--roi', nargs=4, type=float, default=[.3, .3, .7, .7])
    args = parser.parse_args()
    if not args.confirm_neutral:
        parser.error('Fill the selected central 40% ROI with neutral paper, then pass --confirm-neutral')
    profile = ROOT / 'Configuration/camera_profiles.json'
    manifest = ROOT.parent / 'MANIFEST.sha256'
    opener = build_opener(ProxyHandler({}))
    with opener.open('http://127.0.0.1:8083/camera/info', timeout=4) as response:
        active = json.load(response)
    if Path(active.get('profile_file', '')).resolve() != profile.resolve():
        parser.error('8083 is using a different profile; keep the collected four-camera program running')
    previous = profile.read_bytes()
    old_hash = sha256(previous)
    old_data = json.loads(previous)
    if any(active.get(k) != old_data.get(k) for k in ('schema', 'binding', 'revision', 'cameras')):
        parser.error('In-memory profile and saved profile differ; no calibration requested')
    expected = [line.split('  ', 1)[0] for line in manifest.read_text(encoding='utf-8').splitlines()
                if line.split('  ', 1)[-1] == PROFILE_NAME]
    if expected != [old_hash]:
        parser.error('Saved profile does not match its startup manifest; no calibration requested')
    backup = ROOT / 'Results/Color_Calibration' / (
        datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '_' + args.camera)
    backup.mkdir(parents=True)
    (backup / 'camera_profiles.before.json').write_bytes(previous)
    (backup / 'MANIFEST.before.sha256').write_bytes(manifest.read_bytes())
    request = Request('http://127.0.0.1:8083/camera/calibrate', method='POST',
                      data=json.dumps({'camera': args.camera, 'roi': args.roi,
                                       'confirm_neutral': True}).encode(),
                      headers={'Content-Type': 'application/json'})
    try:
        with opener.open(request, timeout=8) as response:
            result = json.load(response)
    except urllib.error.HTTPError as exc:
        parser.exit(2, 'Calibration refused: ' + exc.read(4096).decode('utf-8', 'replace') + '\n')
    updated = profile.read_bytes()
    saved = json.loads(updated)
    if (Path(result.get('saved', '')).resolve() != profile.resolve()
            or result.get('camera') != args.camera
            or saved.get('revision') != old_data['revision'] + 1
            or result.get('revision') != saved['revision']
            or saved['cameras'][args.camera]['gains_rgb'] != result.get('gains_rgb')):
        raise RuntimeError('Live calibration returned inconsistent save evidence; inspect saved profile and backup')
    other = 'rear' if args.camera == 'front' else 'front'
    if saved['cameras'][other] != old_data['cameras'][other]:
        raise RuntimeError('Other camera changed during calibration; inspect before updating manifest')
    try:
        sync_manifest(manifest, old_hash, sha256(updated))
    except (OSError, ValueError) as exc:
        raise RuntimeError('Color was saved/applied, but manifest sync failed. Do not restart yet. Backup: '
                           + str(backup)) from exc
    (backup / 'camera_profiles.after.json').write_bytes(updated)
    result.update(manifest_updated=True, profile_sha256=sha256(updated), backup=str(backup))
    (backup / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print('CAMERA_CALIBRATED ' + json.dumps(result, ensure_ascii=False))
    print('Saved and applied; next START_INFERENCE.sh reloads this configuration without RGB arguments.')


if __name__ == '__main__':
    main()
