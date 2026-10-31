"""R15 lane-focused trainer preserving R12 direct branch and shared encoder.

All 255 pixels are omitted from both optimization and confusion matrices.
"""
import argparse
import csv
import hashlib
import json
import os
import random
import sys
import time
from collections import Counter
from pathlib import Path

import cv2
import numpy as np
import torch
import torch.nn.functional as F
from torch.utils.data import DataLoader, Dataset, WeightedRandomSampler


BASE = Path(__file__).parent
PROJECT = BASE.parents[1]
sys.path.insert(0, str(PROJECT / 'scripts/training'))
from dataset import letterbox_pair  # noqa: E402
from model_adapter import build_small_three_class  # noqa: E402


def hash_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    with Path(path).open(encoding='utf-8-sig', newline='') as f:
        return list(csv.DictReader(f))


class SegData(Dataset):
    def __init__(self, rows, augment=False):
        self.rows = rows
        self.augment = augment

    def __len__(self):
        return len(self.rows)

    def __getitem__(self, i):
        r = self.rows[i]
        image = cv2.imread(r['image_path'], cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError('image decode: ' + r['sample_id'])
        h, w = image.shape[:2]
        masks = []
        for task in ('direct', 'lane'):
            p = r[task + '_mask_path']
            if not p:
                mask = np.full((h, w), 255, np.uint8)
            else:
                mask = cv2.imread(p, cv2.IMREAD_UNCHANGED)
                if mask is None or mask.shape != (h, w):
                    raise ValueError('mask decode/size: ' + r['sample_id'] + '/' + task)
                if task == 'direct' and r['direct_format'] == 'bdd_original_0_1_2':
                    if not set(np.unique(mask).tolist()) <= {0, 1, 2}:
                        raise ValueError('BDD direct IDs: ' + r['sample_id'])
                    mask = (mask == 1).astype(np.uint8)
            masks.append(mask)
        if self.augment:
            if random.random() < .5:
                image = cv2.flip(image, 1)
                masks = [cv2.flip(m, 1) for m in masks]
            if random.random() < .65:
                image = cv2.convertScaleAbs(image, alpha=random.uniform(.8, 1.2),
                                            beta=random.uniform(-22, 22))
        image, da, lane = letterbox_pair(image, masks[0], masks[1])
        image = torch.from_numpy(np.ascontiguousarray(image[:, :, ::-1].transpose(2, 0, 1))).float().div_(255)
        return {'image': image,
                'direct': torch.from_numpy(np.ascontiguousarray(da)).long(),
                'lane': torch.from_numpy(np.ascontiguousarray(lane)).long(),
                'sample_id': r['sample_id'], 'source': r['source'],
                'direct_status': r['direct_status'], 'lane_status': r['lane_status']}


def cm_update(cm, logits, target):
    pred = logits.argmax(1).detach().cpu().numpy()
    true = target.detach().cpu().numpy()
    keep = true != 255
    if not keep.any():
        return
    n = cm.shape[0]
    cm += np.bincount(true[keep].astype(np.int64) * n + pred[keep].astype(np.int64),
                      minlength=n * n).reshape(n, n)


def score_cm(cm):
    out = {}
    for c in range(cm.shape[0]):
        tp = int(cm[c, c]); actual = int(cm[c].sum()); predicted = int(cm[:, c].sum())
        union = actual + predicted - tp
        out[str(c)] = {'support': actual, 'predicted': predicted,
                       'iou': tp / union if union else None,
                       'precision': tp / predicted if predicted else None,
                       'recall': tp / actual if actual else None}
    return out


def loss_terms(da, ll, target_da, target_ll, lane_weights=(.5, 1.8, 1.4),
               focal_gamma=0., boundary_bg_weight=0.):
    result = []
    logs = {}
    if (target_da != 255).any():
        weights = da.new_tensor([.7, 1.3])
        dloss = F.cross_entropy(da, target_da, weight=weights, ignore_index=255)
        result.append(dloss)
        logs['direct'] = float(dloss.detach())
    if (target_ll != 255).any():
        weights = ll.new_tensor(lane_weights)
        if focal_gamma:
            valid = target_ll != 255
            safe = target_ll.clamp(0, 2)
            ce_map = F.cross_entropy(ll, target_ll, weight=weights,
                                     ignore_index=255, reduction='none')
            probs_for_gt = torch.softmax(ll.float(), 1).gather(1, safe.unsqueeze(1)).squeeze(1)
            normalizer = weights[safe][valid].sum().clamp_min(1)
            lloss = (ce_map[valid] * (1 - probs_for_gt[valid]).pow(focal_gamma)).sum() / normalizer
        else:
            lloss = F.cross_entropy(ll, target_ll, weight=weights, ignore_index=255)
        # Soft class dice gives the thin lane classes useful gradients.
        probs = torch.softmax(ll.float(), 1)
        valid = target_ll != 255
        safe = target_ll.clamp(0, 2)
        target = F.one_hot(safe, 3).permute(0, 3, 1, 2).float()
        p = probs[:, 1:] * valid.unsqueeze(1)
        t = target[:, 1:] * valid.unsqueeze(1)
        intersection = (p * t).sum(dim=(0, 2, 3))
        denom = (p + t).sum(dim=(0, 2, 3))
        present = t.sum(dim=(0, 2, 3)) > 0
        dice = (1 - (2 * intersection + 1) / (denom + 1))[present].mean() if present.any() else lloss * 0
        term = lloss + .35 * dice
        if boundary_bg_weight:
            # This ring is valid labeled background adjacent to actual lane paint.
            # Ignored pixels never enter the ring. It penalizes over-thick masks
            # without shrinking or inventing the positive pixel labels.
            foreground = ((target_ll == 1) | (target_ll == 2)).float().unsqueeze(1)
            nearby = F.max_pool2d(foreground, 7, stride=1, padding=3).squeeze(1) > 0
            ring = nearby & (target_ll == 0)
            if ring.any():
                ring_ce = F.cross_entropy(ll, target_ll, ignore_index=255,
                                          reduction='none')[ring].mean()
                term = term + boundary_bg_weight * ring_ce
                logs['boundary_bg'] = float(ring_ce.detach())
        result.append(term)
        logs['lane'] = float(lloss.detach())
        logs['lane_dice'] = float(dice.detach())
    if not result:
        raise ValueError('No valid supervision')
    return sum(result), logs


def run_epoch(model, loader, device, optimizer=None, scaler=None,
              lane_weights=(.5, 1.8, 1.4), focal_gamma=0., boundary_bg_weight=0.):
    train = optimizer is not None
    model.train(train)
    if train and getattr(model, 'r15_freeze_bn', False):
        for module in model.modules():
            if isinstance(module, torch.nn.modules.batchnorm._BatchNorm):
                module.eval()
    da_cm = np.zeros((2, 2), np.int64)
    ll_cm = np.zeros((3, 3), np.int64)
    total = 0.; batches = 0; terms = Counter(); elapsed = time.perf_counter()
    count = Counter()
    for b in loader:
        x = b['image'].to(device, non_blocking=True)
        td = b['direct'].to(device, non_blocking=True)
        tl = b['lane'].to(device, non_blocking=True)
        if train:
            optimizer.zero_grad(set_to_none=True)
        with torch.set_grad_enabled(train), torch.autocast(device_type=device.type, enabled=device.type == 'cuda'):
            da, ll = model(x)
            loss, lt = loss_terms(da.float(), ll.float(), td, tl, lane_weights,
                                  focal_gamma=focal_gamma,
                                  boundary_bg_weight=boundary_bg_weight)
        if train:
            if not torch.isfinite(loss):
                raise RuntimeError('Nonfinite loss')
            scaler.scale(loss).backward()
            scaler.unscale_(optimizer)
            torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            scaler.step(optimizer)
            scaler.update()
        total += float(loss.detach())
        terms.update(lt)
        batches += 1
        cm_update(da_cm, da, td)
        cm_update(ll_cm, ll, tl)
        count['images'] += len(b['sample_id'])
        count['direct_valid_images'] += int((td != 255).flatten(1).any(1).sum())
        count['lane_valid_images'] += int((tl != 255).flatten(1).any(1).sum())
    return {'loss': total / batches if batches else None,
            'term_mean': {k: v / batches for k, v in terms.items()},
            'batches': batches, 'counts': dict(count), 'seconds': round(time.perf_counter() - elapsed, 2),
            'direct_confusion': da_cm.tolist(), 'lane_confusion': ll_cm.tolist(),
            'direct': score_cm(da_cm), 'lane': score_cm(ll_cm)}


def select_score(result):
    diou = result['direct']['1']['iou'] or 0
    solid = result['lane']['1']['iou'] or 0
    dashed = result['lane']['2']['iou'] or 0
    return (diou + solid + dashed) / 3


def r15_score(results):
    """Prefer held-out real typed-lane gains while retaining direct ability."""
    human = results['human']; rlmd = results['rlmd']; bdd = results['source_original']
    a2d2 = results['a2d2']
    h_direct = human['direct']['1']['iou'] or 0
    b_direct = bdd['direct']['1']['iou'] or 0
    def balance(group):
        solid, dashed = ((group['lane'][str(i)]['iou'] or 0) for i in (1, 2))
        return 2 * solid * dashed / (solid + dashed) if solid + dashed else 0
    h_lane = balance(human)
    r_lane = balance(rlmd)
    a_lane = balance(a2d2)
    return .20 * h_direct + .10 * b_direct + .15 * h_lane + .25 * r_lane + .30 * a_lane


def save_atomic(payload, path):
    temp = path.with_suffix('.tmp')
    torch.save(payload, temp)
    os.replace(temp, path)


def preview(model, rows, device, folder, name):
    folder.mkdir(parents=True, exist_ok=True)
    selected = [r for r in rows if r['direct_status'] == 'human_approved' or r['lane_status'] == 'human_approved'][:6]
    selected += [r for r in rows if r['source'].startswith('CARLA')][:3]
    selected += [r for r in rows if r['source'].startswith('RLMD')][:3]
    selected += [r for r in rows if r['source'].startswith('A2D2')][:6]
    ds = SegData(selected)
    model.eval()
    with torch.no_grad():
        for i, r in enumerate(selected):
            b = ds[i]
            x = b['image'][None].to(device)
            with torch.autocast(device_type=device.type, enabled=device.type == 'cuda'):
                da, ll = model(x)
            image = (b['image'].permute(1, 2, 0).numpy()[:, :, ::-1] * 255).astype(np.uint8).copy()
            dtrue = b['direct'].numpy(); ltrue = b['lane'].numpy()
            dp = da.argmax(1)[0].cpu().numpy(); lp = ll.argmax(1)[0].cpu().numpy()
            def overlay(d, l):
                out = image.copy()
                out[d == 1] = (out[d == 1] * .6 + np.array([0, 150, 0]) * .4).astype(np.uint8)
                out[l == 1] = (0, 0, 255)
                out[l == 2] = (0, 255, 255)
                out[l == 255] = (255, 0, 255)
                return out
            panel = np.hstack([image, overlay(dtrue, ltrue), overlay(dp, lp)])
            cv2.putText(panel, f"{r['sample_id']} | original / label / {name}", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 255, 255), 2)
            cv2.imwrite(str(folder / f'{name}_{i:02d}_{r["sample_id"]}.jpg'), panel)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run', required=True)
    ap.add_argument('--epochs', type=int, default=50)
    ap.add_argument('--batch', type=int, default=12)
    ap.add_argument('--lr', type=float, default=0.0003)
    ap.add_argument('--workers', type=int, default=0)
    ap.add_argument('--patience', type=int, default=20)
    ap.add_argument('--max-train', type=int, default=0)
    ap.add_argument('--overfit', action='store_true')
    ap.add_argument('--eval-only', action='store_true')
    ap.add_argument('--resume', type=Path)
    ap.add_argument('--init-from', type=Path, help='Initialize model only from a prior R11.4 checkpoint')
    ap.add_argument('--data-version', default='data_v1', choices=('data_v1',))
    ap.add_argument('--samples-per-epoch', type=int, default=3000)
    ap.add_argument('--synthetic-fraction', type=float, default=.10)
    ap.add_argument('--human-fraction', type=float, default=.05)
    ap.add_argument('--rlmd-fraction', type=float, default=.25)
    ap.add_argument('--a2d2-fraction', type=float, default=.45)
    ap.add_argument('--freeze-bn', action='store_true',
                    help='Keep R12 running statistics while fine-tuning weights')
    ap.add_argument('--lane-only', action='store_true',
                    help='Train only lane decoder/output; R12 shared/direct tensors stay fixed')
    ap.add_argument('--solid-weight', type=float, default=1.8)
    ap.add_argument('--dashed-weight', type=float, default=1.4)
    ap.add_argument('--background-weight', type=float, default=.5)
    ap.add_argument('--lane-focal-gamma', type=float, default=0.)
    ap.add_argument('--boundary-bg-weight', type=float, default=0.)
    ap.add_argument('--real-only', action='store_true',
                    help='Target-domain refinement: exclude synthetic training images')
    args = ap.parse_args()
    if args.resume and args.init_from:
        raise ValueError('Choose either --resume or --init-from')
    if (min(args.synthetic_fraction, args.human_fraction, args.rlmd_fraction,
            args.a2d2_fraction) < 0 or
            args.synthetic_fraction + args.human_fraction + args.rlmd_fraction +
            args.a2d2_fraction >= 1):
        raise ValueError('Invalid source fractions')
    torch.set_num_threads(6)
    cv2.setNumThreads(0)
    seed = 114
    random.seed(seed); np.random.seed(seed); torch.manual_seed(seed); torch.cuda.manual_seed_all(seed)
    data = BASE / args.data_version
    manifests = {s: data / f'{s}_manifest.csv' for s in ('train', 'val')}
    hashes = {s: hash_file(p) for s, p in manifests.items()}
    train_rows = read(manifests['train']); val_rows = read(manifests['val'])
    if args.real_only:
        train_rows = [r for r in train_rows if not r['source'].startswith('CARLA')]
    if set(r['sample_id'] for r in train_rows) & set(r['sample_id'] for r in val_rows):
        raise ValueError('train/val sample ID leak')
    human_val = [r for r in val_rows if 'human_approved' in (r['direct_status'], r['lane_status'])]
    synthetic_val = [r for r in val_rows if r['source'].startswith('CARLA')]
    rlmd_val = [r for r in val_rows if r['source'].startswith('RLMD')]
    a2d2_val = [r for r in val_rows if r['source'].startswith('A2D2')]
    original_val = [r for r in val_rows if r['direct_status'] == 'source_original_auxiliary']
    if len(human_val) < 10 or not synthetic_val or not rlmd_val or not a2d2_val or not train_rows:
        raise ValueError('Missing validation or training rows')
    if args.overfit:
        positives = [r for r in train_rows if int(r['solid_pixels']) and int(r['dashed_pixels'])
                     and r['direct_mask_path']]
        train_rows = positives[:args.batch]
        synthetic_val = synthetic_val[:8]
        rlmd_val = rlmd_val[:8]
        a2d2_val = a2d2_val[:8]
        original_val = original_val[:8]
    elif args.max_train:
        train_rows = train_rows[:args.max_train]
    run = BASE / args.run
    run.mkdir(exist_ok=False) if not args.resume else run.mkdir(exist_ok=True)
    config = {'epochs': args.epochs, 'batch': args.batch, 'lr': args.lr, 'workers': args.workers,
              'patience': args.patience, 'overfit': args.overfit, 'max_train': args.max_train,
              'samples_per_epoch': args.samples_per_epoch,
              'synthetic_fraction': args.synthetic_fraction,
              'human_fraction': args.human_fraction,
              'rlmd_fraction': args.rlmd_fraction,
              'a2d2_fraction': args.a2d2_fraction,
              'freeze_bn': args.freeze_bn,
              'lane_only': args.lane_only,
              'selection': '0.20 human direct + 0.10 BDD direct + 0.15 human lane harmonic + 0.25 RLMD lane harmonic + 0.30 A2D2 lane harmonic',
              'real_only': args.real_only,
              'lane_class_weights': [args.background_weight,
                                     args.solid_weight, args.dashed_weight],
              'lane_focal_gamma': args.lane_focal_gamma,
              'boundary_bg_weight': args.boundary_bg_weight,
              'lr_groups': {'new_lane_head': 1., 'lane_decoder': .5,
                                                         'shared': .1, 'direct_decoder': .05},
              'seed': seed, 'data_version': args.data_version, 'trainer_sha256': hash_file(__file__),
              'manifest_sha256': hashes,
              'weight_source_sha256': hash_file(PROJECT / 'checkpoints/upstream_small.pth'),
              'init_checkpoint': str(args.init_from) if args.init_from else '',
              'init_sha256': hash_file(args.init_from) if args.init_from else '',
              'train_rows': len(train_rows), 'human_val_rows': len(human_val),
              'synthetic_val_rows': len(synthetic_val),
              'rlmd_val_rows': len(rlmd_val), 'source_val_rows': len(original_val)}
    config['a2d2_val_rows'] = len(a2d2_val)
    if args.resume:
        prior_config = json.loads((run / 'config.json').read_text(encoding='utf-8'))
        config['init_checkpoint'] = prior_config['init_checkpoint']
        config['init_sha256'] = prior_config['init_sha256']
        if prior_config != config:
            raise ValueError('Resume config / manifest / trainer changed')
    (run / 'config.json').write_text(json.dumps(config, indent=2), encoding='utf-8')
    if not args.resume:
        (run / 'trainer_source_snapshot.py').write_bytes(Path(__file__).read_bytes())
        for split, path in manifests.items():
            (run / f'{split}_manifest_snapshot.csv').write_bytes(path.read_bytes())
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model, load_info = build_small_three_class()
    # Only the new randomly initialized 3-class head is made linear. The
    # upstream 2-class drivable head and all reused pretrained tensors stay intact.
    model.out_ll.conv2.act = torch.nn.Identity()
    if args.init_from and not args.resume:
        prior = torch.load(args.init_from, map_location='cpu', weights_only=False)
        model.load_state_dict(prior['model'], strict=True)
    model.r15_freeze_bn = args.freeze_bn
    if args.lane_only:
        for name, parameter in model.named_parameters():
            parameter.requires_grad_(name.startswith(('up_1_ll.', 'up_2_ll.', 'out_ll.')))
    model.to(device)
    (run / 'weight_load.json').write_text(json.dumps(load_info, indent=2), encoding='utf-8')
    grouped = {}
    for name, parameter in model.named_parameters():
        if not parameter.requires_grad:
            continue
        if name.startswith('out_ll.'):
            key, factor = 'new_lane_head', 1.
        elif name.startswith(('up_1_ll.', 'up_2_ll.')):
            key, factor = 'lane_decoder', .5
        elif name.startswith(('up_1_da.', 'up_2_da.', 'out_da.')):
            key, factor = 'direct_decoder', .05
        else:
            key, factor = 'shared', .1
        grouped.setdefault(key, {'params': [], 'lr': args.lr * factor})['params'].append(parameter)
    optimizer = torch.optim.AdamW(list(grouped.values()), weight_decay=1e-4)
    scaler = torch.amp.GradScaler('cuda', enabled=device.type == 'cuda')
    start_epoch = 1; best = -1.; stagnant = 0
    if args.resume:
        ckpt = torch.load(args.resume, map_location='cpu', weights_only=False)
        if ckpt['config'] != config:
            raise ValueError('Resume config / manifests changed')
        model.load_state_dict(ckpt['model']); optimizer.load_state_dict(ckpt['optimizer'])
        scaler.load_state_dict(ckpt['scaler']); start_epoch = ckpt['epoch'] + 1
        best = ckpt['best_score']; stagnant = ckpt['stagnant']
        random.setstate(ckpt['rng_python']); np.random.set_state(ckpt['rng_numpy'])
        torch.set_rng_state(ckpt['rng_torch'])
        if device.type == 'cuda': torch.cuda.set_rng_state_all(ckpt['rng_cuda'])
    ds = SegData(train_rows, augment=not args.overfit)
    if args.overfit:
        sampler = None
    else:
        buckets = Counter()
        categories = []
        for r in train_rows:
            category = ('human' if 'human_approved' in (r['direct_status'], r['lane_status']) else
                        'synthetic' if r['source'].startswith('CARLA') else
                        'rlmd' if r['source'].startswith('RLMD') else
                        'a2d2' if r['source'].startswith('A2D2') else 'bdd')
            categories.append(category)
            buckets[category] += 1
        if min(buckets.values()) == 0 or len(buckets) != 5:
            raise ValueError('Missing source bucket')
        fractions = {'human': args.human_fraction,
                     'synthetic': args.synthetic_fraction,
                     'rlmd': args.rlmd_fraction,
                     'a2d2': args.a2d2_fraction,
                     'bdd': 1 - args.human_fraction - args.synthetic_fraction -
                     args.rlmd_fraction - args.a2d2_fraction}
        weights = [fractions[c] / buckets[c] *
                   (2 if c == 'synthetic' and int(r['solid_pixels']) > 0 else 1)
                   for c, r in zip(categories, train_rows)]
        sampler = WeightedRandomSampler(weights,
                                        num_samples=args.samples_per_epoch,
                                        replacement=True)
    train_loader = DataLoader(ds, batch_size=args.batch, shuffle=sampler is None,
                              sampler=sampler, num_workers=args.workers, pin_memory=device.type == 'cuda')
    val_loaders = {'human': DataLoader(SegData(human_val), batch_size=args.batch, num_workers=args.workers),
                   'synthetic': DataLoader(SegData(synthetic_val), batch_size=args.batch, num_workers=args.workers),
                   'rlmd': DataLoader(SegData(rlmd_val), batch_size=args.batch, num_workers=args.workers),
                   'a2d2': DataLoader(SegData(a2d2_val), batch_size=args.batch, num_workers=args.workers),
                   'source_original': DataLoader(SegData(original_val), batch_size=args.batch, num_workers=args.workers)}
    if start_epoch == 1:
        baseline = {name: run_epoch(model, loader, device,
                                  lane_weights=config['lane_class_weights'],
                                  focal_gamma=args.lane_focal_gamma,
                                  boundary_bg_weight=args.boundary_bg_weight)
                    for name, loader in val_loaders.items() if len(loader.dataset)}
        (run / 'baseline.json').write_text(json.dumps(baseline, indent=2), encoding='utf-8')
        preview(model, human_val + synthetic_val + rlmd_val + a2d2_val, device, run / 'previews', 'before')
        print(json.dumps({'baseline_human': {k: v['direct'].get('1') for k, v in baseline.items()},
                          'train_rows': len(train_rows), 'device': str(device)}), flush=True)
    if args.eval_only:
        return
    for epoch in range(start_epoch, args.epochs + 1):
        started = time.perf_counter()
        train = run_epoch(model, train_loader, device, optimizer, scaler,
                          lane_weights=config['lane_class_weights'],
                          focal_gamma=args.lane_focal_gamma,
                          boundary_bg_weight=args.boundary_bg_weight)
        vals = {name: run_epoch(model, loader, device,
                                lane_weights=config['lane_class_weights'],
                                focal_gamma=args.lane_focal_gamma,
                                boundary_bg_weight=args.boundary_bg_weight)
                for name, loader in val_loaders.items() if len(loader.dataset)}
        score = r15_score(vals)
        improved = score > best + 1e-4
        if improved: best = score; stagnant = 0
        else: stagnant += 1
        record = {'epoch': epoch, 'train': train, 'val': vals,
                  'human_selection_score': score, 'best_score': best,
                  'epoch_seconds': round(time.perf_counter() - started, 2)}
        with (run / 'history.jsonl').open('a', encoding='utf-8') as f:
            f.write(json.dumps(record) + '\n')
        state = {'epoch': epoch, 'model': model.state_dict(), 'optimizer': optimizer.state_dict(),
                 'scaler': scaler.state_dict(), 'best_score': best, 'stagnant': stagnant,
                 'config': config, 'rng_python': random.getstate(), 'rng_numpy': np.random.get_state(),
                 'rng_torch': torch.get_rng_state(),
                 'rng_cuda': torch.cuda.get_rng_state_all() if device.type == 'cuda' else None}
        save_atomic(state, run / 'latest.pt')
        if epoch % 5 == 0:
            (run / 'checkpoints').mkdir(exist_ok=True)
            save_atomic(state, run / 'checkpoints' / f'epoch_{epoch:03d}.pt')
        if improved:
            save_atomic(state, run / 'best.pt')
            preview(model, human_val + synthetic_val + rlmd_val + a2d2_val, device, run / 'previews', 'best')
        h = vals['human']
        print(json.dumps({'epoch': epoch, 'train_loss': train['loss'], 'human_loss': h['loss'],
                          'human_direct_iou': h['direct']['1']['iou'],
                          'human_solid_iou': h['lane']['1']['iou'],
                          'human_dashed_iou': h['lane']['2']['iou'],
                          'rlmd_solid_iou': vals['rlmd']['lane']['1']['iou'],
                          'rlmd_dashed_iou': vals['rlmd']['lane']['2']['iou'],
                          'a2d2_solid_iou': vals['a2d2']['lane']['1']['iou'],
                          'a2d2_dashed_iou': vals['a2d2']['lane']['2']['iou'],
                          'synthetic_score': select_score(vals['synthetic']),
                          'seconds': record['epoch_seconds'], 'best': best}), flush=True)
        if not args.overfit and stagnant >= args.patience and epoch >= 15:
            print('EARLY_STOP patience reached', flush=True)
            break
    best_ckpt = torch.load(run / 'best.pt', map_location='cpu', weights_only=False)
    model.load_state_dict(best_ckpt['model'])
    preview(model, human_val + synthetic_val + rlmd_val + a2d2_val, device, run / 'previews', 'final_best')


if __name__ == '__main__':
    main()
