"""Full-sequence, evaluation-only R11.4/R12 comparison on local contest clips.

The videos have no pixel ground truth. Never use their outputs as labels.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import torch

PROJECT = Path(__file__).resolve().parents[2]
ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(PROJECT / 'scripts/training'))
from model_adapter import build_small_three_class

VIDEOS = Path(r'D:\YOLOP\officialVideo')
BASELINE = PROJECT / 'runs/R11_4_multitask_training/selected_model/R11_4_twinlitenet_small_direct2_lane3.pt'
R12 = PROJECT / 'runs/R12_china_carla_autotrain/selected_model/R12_twinlitenet_small_direct2_lane3.pt'
WIDTH, HEIGHT = 640, 384


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def load_model(path: Path, device: torch.device):
    model, _ = build_small_three_class()
    model.out_ll.conv2.act = torch.nn.Identity()
    ckpt = torch.load(path, map_location='cpu', weights_only=False)
    model.load_state_dict(ckpt['model'], strict=True)
    return model.to(device).eval()


def prepare(frame):
    h, w = frame.shape[:2]
    scale = min(HEIGHT / h, WIDTH / w)
    rw, rh = round(w * scale), round(h * scale)
    top, left = (HEIGHT - rh) // 2, (WIDTH - rw) // 2
    image = cv2.resize(frame, (rw, rh), interpolation=cv2.INTER_AREA)
    image = cv2.copyMakeBorder(image, top, HEIGHT-rh-top, left, WIDTH-rw-left,
                               cv2.BORDER_CONSTANT, value=(114, 114, 114))
    x = torch.from_numpy(np.ascontiguousarray(image[:, :, ::-1].transpose(2, 0, 1))).float().div_(255).unsqueeze(0)
    return x, (left, top, rw, rh)


def masks(model, x, device, threshold=0.):
    with torch.inference_mode(), torch.autocast(device_type=device.type, enabled=device.type == 'cuda'):
        da, lane = model(x)
    d = da.argmax(1)[0].byte().cpu().numpy()
    p = lane.float().softmax(1)[0]
    l = p.argmax(0).byte().cpu().numpy()
    conf = p.max(0).values.cpu().numpy()
    if threshold:
        l[(l != 0) & (conf < threshold)] = 0
    return d, l, conf


def crop_to_content(array, box):
    left, top, rw, rh = box
    return array[top:top+rh, left:left+rw]


def render(frame, direct, lane, label):
    # Prediction maps are first cropped to source content; frame is at 640x360 for 1080p inputs.
    h, w = frame.shape[:2]
    d = cv2.resize(direct, (w, h), interpolation=cv2.INTER_NEAREST)
    l = cv2.resize(lane, (w, h), interpolation=cv2.INTER_NEAREST)
    out = frame.copy()
    fg = d == 1
    out[fg] = (out[fg].astype(np.float32) * .58 + np.array([0, 145, 0]) * .42).astype(np.uint8)
    out[l == 1] = (0, 0, 255)
    out[l == 2] = (0, 255, 255)
    cv2.rectangle(out, (0, 0), (w, 29), (20, 20, 20), -1)
    cv2.putText(out, label, (8, 21), cv2.FONT_HERSHEY_SIMPLEX, .58, (255,255,255), 1, cv2.LINE_AA)
    return out


def probe_one(path):
    cap = cv2.VideoCapture(str(path))
    result = {'path': str(path), 'name': path.name, 'bytes': path.stat().st_size,
              'sha256': sha256(path), 'open': cap.isOpened()}
    if cap.isOpened():
        result.update(frames_reported=int(cap.get(cv2.CAP_PROP_FRAME_COUNT)),
                      fps=cap.get(cv2.CAP_PROP_FPS), width=int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                      height=int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                      fourcc=''.join(chr((int(cap.get(cv2.CAP_PROP_FOURCC)) >> 8*i) & 255) for i in range(4)))
        ok, _ = cap.read()
        result['first_frame_decoded'] = ok
    cap.release()
    return result


def run_video(path, models, device, out, snapshot_every=300):
    cap = cv2.VideoCapture(str(path))
    fps = cap.get(cv2.CAP_PROP_FPS)
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    if not cap.isOpened() or fps <= 0:
        return {'status': 'unreadable'}
    stem = path.stem
    output_path = out / f'{stem}_R114_vs_R12.mp4'
    csv_path = out / f'{stem}_frames.csv'
    video = None
    rows = 0
    start = time.perf_counter()
    stats = {'decode_ms': [], 'preprocess_ms': [], 'forward_ms': [], 'postprocess_ms': [], 'encode_ms': []}
    fields = ['video', 'frame', 'time_s', 'direct_r114', 'solid_r114', 'dashed_r114',
              'direct_r12', 'solid_r12', 'dashed_r12', 'bottom8_lane_r114',
              'bottom8_lane_r12', 'pad_bottom_lane_r12', 'decode_ms', 'preprocess_ms',
              'forward_ms', 'postprocess_ms', 'encode_ms']
    with csv_path.open('w', encoding='utf-8', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        while True:
            t0 = time.perf_counter()
            ok, frame = cap.read()
            t1 = time.perf_counter()
            if not ok:
                break
            x, box = prepare(frame)
            x = x.to(device)
            display = cv2.resize(frame, (640, 360), interpolation=cv2.INTER_AREA)
            t2 = time.perf_counter()
            predictions = {}
            for key in ('r114', 'r12'):
                d, l, conf = masks(models[key], x, device, .6 if key == 'r12' else 0.)
                predictions[key] = (d, l, conf)
            if device.type == 'cuda': torch.cuda.synchronize()
            t3 = time.perf_counter()
            r114_d, r114_l, _ = predictions['r114']
            r12_d, r12_l, _ = predictions['r12']
            left, top, rw, rh = box
            dc11, lc11 = crop_to_content(r114_d, box), crop_to_content(r114_l, box)
            dc12, lc12 = crop_to_content(r12_d, box), crop_to_content(r12_l, box)
            a = render(display, dc11, lc11, 'R11.4 baseline')
            b = render(display, dc12, lc12, 'R12 lane t=0.6')
            orig = display.copy()
            cv2.rectangle(orig, (0, 0), (640, 29), (20,20,20), -1)
            cv2.putText(orig, f'{stem} frame {rows}', (8,21), cv2.FONT_HERSHEY_SIMPLEX, .55,(255,255,255),1,cv2.LINE_AA)
            panel = np.hstack([orig, a, b])
            t4 = time.perf_counter()
            if video is None:
                video = cv2.VideoWriter(str(output_path), cv2.VideoWriter_fourcc(*'mp4v'), fps,
                                        (panel.shape[1], panel.shape[0]))
                if not video.isOpened(): raise RuntimeError('Cannot open output VideoWriter: ' + str(output_path))
            video.write(panel)
            if rows % snapshot_every == 0 or rows == total-1:
                cv2.imwrite(str(out / 'snapshots' / f'{stem}_{rows:06d}.jpg'), panel)
            t5 = time.perf_counter()
            values = [1000*(b-a) for a,b in ((t0,t1),(t1,t2),(t2,t3),(t3,t4),(t4,t5))]
            for key,v in zip(stats,values):stats[key].append(v)
            writer.writerow({'video': path.name, 'frame': rows, 'time_s': round(rows/fps,3),
              'direct_r114': int((dc11==1).sum()), 'solid_r114': int((lc11==1).sum()),
              'dashed_r114': int((lc11==2).sum()), 'direct_r12': int((dc12==1).sum()),
              'solid_r12': int((lc12==1).sum()), 'dashed_r12': int((lc12==2).sum()),
              'bottom8_lane_r114': int((lc11[-8:]!=0).sum()),
              'bottom8_lane_r12': int((lc12[-8:]!=0).sum()),
              'pad_bottom_lane_r12': int((r12_l[top+rh:]!=0).sum()),
              **{k:round(v,3) for k,v in zip(stats,values)}})
            rows += 1
            if rows % 300 == 0:
                print(json.dumps({'video': stem, 'frames': rows, 'total': total,
                                  'elapsed_s': round(time.perf_counter()-start,1)}), flush=True)
    cap.release()
    if video:video.release()
    check = cv2.VideoCapture(str(output_path))
    readable = check.isOpened() and check.read()[0]
    check.release()
    return {'status': 'done' if readable and rows else 'output_decode_failed',
            'frames_decoded': rows, 'frames_reported': total, 'fps': fps,
            'elapsed_s': round(time.perf_counter()-start,2),
            'output': str(output_path), 'output_bytes': output_path.stat().st_size if output_path.exists() else 0,
            'csv': str(csv_path), 'output_first_frame_decoded': readable,
            'median_ms': {k: round(float(np.median(v)),2) for k,v in stats.items() if v}}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--probe-only', action='store_true')
    ap.add_argument('--video', action='append', help='specific filename; default all readable contest videos')
    args = ap.parse_args()
    ROOT.mkdir(parents=True, exist_ok=True)
    out = ROOT / 'videos'
    out.mkdir(exist_ok=True)
    (out / 'snapshots').mkdir(exist_ok=True)
    candidates = sorted(VIDEOS.glob('*.mp4'))
    probes = [probe_one(p) for p in candidates]
    (ROOT / 'video_inventory.json').write_text(json.dumps(probes, ensure_ascii=False, indent=2), encoding='utf-8')
    if args.probe_only:return
    cv2.setNumThreads(2)
    torch.set_num_threads(6)
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    models = {'r114': load_model(BASELINE, device), 'r12': load_model(R12, device)}
    results = json.loads((ROOT / 'video_results.json').read_text(encoding='utf-8')) if (ROOT / 'video_results.json').exists() else {}
    requested = set(args.video) if args.video else None
    for item in probes:
        if requested and item['name'] not in requested: continue
        if not item['open'] or not item.get('first_frame_decoded'):
            results[item['name']] = {'status':'unreadable', 'reason':'missing/corrupt MP4 metadata or unsupported codec'}
            continue
        if results.get(item['name'],{}).get('status') == 'done':continue
        results[item['name']] = run_video(Path(item['path']),models,device,out)
        (ROOT / 'video_results.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(results,ensure_ascii=False,indent=2),flush=True)


if __name__ == '__main__':main()
