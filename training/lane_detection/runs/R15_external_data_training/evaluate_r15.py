"""Evaluate a saved checkpoint on the frozen R15 validation manifest."""
import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
import torch

from train_r15 import BASE, SegData, build_small_three_class, score_cm


def confusion(pred, true, n):
    keep = true != 255
    if not keep.any():
        return np.zeros((n, n), np.int64)
    return np.bincount(true[keep].astype(np.int64) * n + pred[keep].astype(np.int64),
                       minlength=n * n).reshape(n, n)


def overlay(image, direct, lane):
    out = image.copy()
    out[direct == 1] = (out[direct == 1] * .6 + np.array([0, 150, 0]) * .4).astype(np.uint8)
    out[lane == 1] = (0, 0, 255)
    out[lane == 2] = (0, 255, 255)
    out[lane == 255] = (255, 0, 255)
    return out


def tolerant_f1(pred, true, valid, radius=5):
    if not true.any():
        return None
    pred = (pred & valid).astype(np.uint8)
    true = (true & valid).astype(np.uint8)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * radius + 1, 2 * radius + 1))
    near_true = cv2.dilate(true, kernel) != 0
    near_pred = cv2.dilate(pred, kernel) != 0
    precision = float((pred & near_true).sum() / pred.sum()) if pred.any() else 0.
    recall = float((true & near_pred).sum() / true.sum())
    return 2 * precision * recall / (precision + recall) if precision + recall else 0.


def group_for(row):
    if 'human_approved' in (row['direct_status'], row['lane_status']):
        return 'human'
    if row['source'].startswith('RLMD'):
        return 'rlmd'
    if row['source'].startswith('CARLA'):
        return 'synthetic'
    if row['source'].startswith('A2D2'):
        return 'a2d2'
    return 'bdd_source'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--checkpoint', type=Path, required=True)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--max-visual-per-group', type=int, default=16)
    ap.add_argument('--only-group', choices=('human', 'rlmd', 'synthetic', 'a2d2', 'bdd_source'))
    ap.add_argument('--lane-threshold', type=float, default=.6,
                    help='Set non-background predictions below this softmax confidence to background')
    args = ap.parse_args()
    if not 0 <= args.lane_threshold <= 1:
        raise ValueError('lane-threshold must be in [0, 1]')
    cv2.setNumThreads(0)
    torch.set_num_threads(6)
    args.out.mkdir(parents=True, exist_ok=False)
    with (BASE / 'data_v1/val_manifest.csv').open(encoding='utf-8', newline='') as f:
        rows = list(csv.DictReader(f))
    if args.only_group:
        rows = [r for r in rows if group_for(r) == args.only_group]
        if not rows:
            raise RuntimeError('No rows in selected group')
    ds = SegData(rows)
    checkpoint = torch.load(args.checkpoint, map_location='cpu', weights_only=False)
    model, _ = build_small_three_class()
    model.out_ll.conv2.act = torch.nn.Identity()
    model.load_state_dict(checkpoint['model'], strict=True)
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model.to(device).eval()
    cms = defaultdict(lambda: {'direct': np.zeros((2, 2), np.int64),
                               'lane': np.zeros((3, 3), np.int64)})
    visuals = defaultdict(int)
    records = []
    with torch.inference_mode():
        for i, row in enumerate(rows):
            item = ds[i]
            x = item['image'][None].to(device)
            with torch.autocast(device_type=device.type, enabled=device.type == 'cuda'):
                dlogit, llogit = model(x)
            dp = dlogit.argmax(1)[0].cpu().numpy().astype(np.uint8)
            lane_prob = torch.softmax(llogit.float(), dim=1)[0]
            lp = lane_prob.argmax(0).cpu().numpy().astype(np.uint8)
            lane_confidence = lane_prob.max(0).values.cpu().numpy()
            lp[(lp != 0) & (lane_confidence < args.lane_threshold)] = 0
            td = item['direct'].numpy().astype(np.uint8)
            tl = item['lane'].numpy().astype(np.uint8)
            group = group_for(row)
            dc = confusion(dp, td, 2)
            lc = confusion(lp, tl, 3)
            cms[group]['direct'] += dc
            cms[group]['lane'] += lc
            dm = score_cm(dc)['1']
            lm = score_cm(lc)
            record = {'sample_id': row['sample_id'], 'group': group,
                      'source': row['source'],
                      'direct_iou': dm['iou'] if dm['support'] else None,
                      'direct_precision': dm['precision'] if dm['support'] else None,
                      'direct_recall': dm['recall'] if dm['support'] else None,
                      'solid_iou': lm['1']['iou'] if lm['1']['support'] else None,
                      'solid_precision': lm['1']['precision'] if lm['1']['support'] else None,
                      'solid_recall': lm['1']['recall'] if lm['1']['support'] else None,
                      'dashed_iou': lm['2']['iou'] if lm['2']['support'] else None,
                      'dashed_precision': lm['2']['precision'] if lm['2']['support'] else None,
                      'dashed_recall': lm['2']['recall'] if lm['2']['support'] else None,
                      'direct_gt_pixels': dm['support'],
                      'solid_gt_pixels': lm['1']['support'],
                      'dashed_gt_pixels': lm['2']['support'],
                      'solid_tolerance5_f1': tolerant_f1(lp == 1, tl == 1, tl != 255),
                      'dashed_tolerance5_f1': tolerant_f1(lp == 2, tl == 2, tl != 255)}
            records.append(record)
            if visuals[group] < args.max_visual_per_group:
                image = np.ascontiguousarray(
                    (item['image'].permute(1, 2, 0).numpy()[:, :, ::-1] * 255).astype(np.uint8))
                panels = [image, overlay(image, td, tl), overlay(image, dp, lp)]
                names = ['original', 'label', 'prediction']
                for panel, name in zip(panels, names):
                    cv2.rectangle(panel, (0, 0), (640, 29), (20, 20, 20), -1)
                    cv2.putText(panel, f'{row["sample_id"]} {name}', (7, 22),
                                cv2.FONT_HERSHEY_SIMPLEX, .53, (255, 255, 255), 1, cv2.LINE_AA)
                out = args.out / 'visuals' / group
                out.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(out / f'{row["sample_id"]}.jpg'), np.hstack(panels))
                visuals[group] += 1
            if (i + 1) % 200 == 0:
                print(json.dumps({'evaluated': i + 1, 'total': len(rows)}), flush=True)
    with (args.out / 'per_image.csv').open('w', encoding='utf-8', newline='') as f:
        w = csv.DictWriter(f, fieldnames=records[0].keys())
        w.writeheader(); w.writerows(records)
    summary = {}
    for group, cm in cms.items():
        direct = score_cm(cm['direct']); lane = score_cm(cm['lane'])
        summary[group] = {'images': sum(r['group'] == group for r in records),
                          'direct_confusion': cm['direct'].tolist(),
                          'lane_confusion': cm['lane'].tolist(),
                          'direct': direct, 'lane': lane,
                          'lane_foreground_macro_iou':
                          float(np.mean([lane[str(i)]['iou'] for i in (1, 2)
                                         if lane[str(i)]['support'] and lane[str(i)]['iou'] is not None]))
                          if any(lane[str(i)]['support'] for i in (1, 2)) else None}
    result = {'checkpoint': str(args.checkpoint), 'checkpoint_epoch': checkpoint.get('epoch'),
              'lane_threshold': args.lane_threshold,
              'device': str(device), 'data_snapshot': str(BASE / 'data_v1/val_manifest.csv'),
              'groups': summary}
    (args.out / 'metrics.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps({'groups': {k: v['images'] for k, v in summary.items()},
                      'device': str(device)}), flush=True)


if __name__ == '__main__':
    main()

