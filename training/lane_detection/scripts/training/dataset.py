"""Manifest-driven BDD loader. Audit labels are refused until reviewed."""
from __future__ import annotations

import csv
from pathlib import Path

import cv2
import numpy as np
import torch
from torch.utils.data import Dataset


def letterbox_pair(image: np.ndarray, da: np.ndarray, lane: np.ndarray,
                   target: tuple[int,int]=(384,640)) -> tuple[np.ndarray,np.ndarray,np.ndarray]:
    h,w=image.shape[:2]; th,tw=target
    if da.shape!=(h,w) or lane.shape!=(h,w):raise ValueError('mask/image shape mismatch')
    ratio=min(th/h,tw/w)
    rh,rw=int(round(h*ratio)),int(round(w*ratio))
    image=cv2.resize(image,(rw,rh),interpolation=cv2.INTER_AREA)
    da=cv2.resize(da,(rw,rh),interpolation=cv2.INTER_NEAREST)
    lane=cv2.resize(lane,(rw,rh),interpolation=cv2.INTER_NEAREST)
    top=(th-rh)//2;bottom=th-rh-top;left=(tw-rw)//2;right=tw-rw-left
    image=cv2.copyMakeBorder(image,top,bottom,left,right,cv2.BORDER_CONSTANT,value=(114,114,114))
    da=cv2.copyMakeBorder(da,top,bottom,left,right,cv2.BORDER_CONSTANT,value=255)
    lane=cv2.copyMakeBorder(lane,top,bottom,left,right,cv2.BORDER_CONSTANT,value=255)
    return image,da,lane


class ManifestDataset(Dataset):
    def __init__(self, manifest: Path, approved_only: bool=True,
                 da_positive_ids: tuple[int,...]=(1,)) -> None:
        with manifest.open(encoding='utf-8-sig',newline='') as f:rows=list(csv.DictReader(f))
        if approved_only:
            rows=[r for r in rows if r.get('label_review_status')=='approved' and r.get('planned_use') in {'train','val'}]
        rows=[r for r in rows if r.get('da_valid')=='True' or r.get('lane_valid')=='True']
        if not rows:raise ValueError('No reviewed and approved samples in manifest')
        self.rows=rows
        if tuple(da_positive_ids)!=(1,):
            raise ValueError('R10 direct-only supervision requires source ID 1 only')
        self.da_positive_ids=da_positive_ids

    def __len__(self)->int:return len(self.rows)

    def __getitem__(self,index:int):
        row=self.rows[index]
        image=cv2.imread(row['image_path'],cv2.IMREAD_COLOR)
        direct_path=row.get('direct_mask_path') or ''
        lane_path=row.get('lane_v2_path') or row['lane_mask_path']
        da=cv2.imread(direct_path or row['da_original_path'],cv2.IMREAD_UNCHANGED)
        lane=cv2.imread(lane_path,cv2.IMREAD_UNCHANGED)
        if image is None or da is None or lane is None:raise ValueError(row['sample_id']+' missing/undecodable')
        if not set(np.unique(da).tolist())<=({0,1,255} if direct_path else {0,1,2}):raise ValueError('Unexpected DA labels')
        if not set(np.unique(lane).tolist())<={0,1,2,255}:raise ValueError('Unexpected lane labels')
        if not direct_path:
            # Legacy manifest fallback: mirror IDs 1=direct, 2=alternative.
            da=(da==1).astype(np.uint8)
        if row['da_valid']!='True':
            da=np.full_like(da,255)
        if row['lane_valid']!='True':
            lane=np.full_like(lane,255)
        image,da,lane=letterbox_pair(image,da,lane)
        rgb=np.ascontiguousarray(image[:,:,::-1].transpose(2,0,1))
        return {'image':torch.from_numpy(rgb).float().div_(255),
                'da':torch.from_numpy(np.ascontiguousarray(da)).long(),
                'lane':torch.from_numpy(np.ascontiguousarray(lane)).long(),
                'da_valid':torch.tensor(row['da_valid']=='True'),
                'lane_valid':torch.tensor(row['lane_valid']=='True'),
                'sample_id':row['sample_id']}


class TaskApprovedDataset(Dataset):
    """R11 task-level gate. Disabled tasks are all ignore, including on A samples."""

    def __init__(self, manifest: Path, expected_split: str,
                 enabled_tasks: tuple[str, ...] = ('direct',),
                 target: tuple[int, int] = (384, 640)) -> None:
        if expected_split not in {'train', 'val'} or not set(enabled_tasks) <= {'direct', 'lane'}:
            raise ValueError('Unexpected split or task')
        if not enabled_tasks:
            raise ValueError('At least one task must be enabled')
        with Path(manifest).open(encoding='utf-8-sig', newline='') as handle:
            rows = list(csv.DictReader(handle))
        if not rows:
            raise ValueError('Empty approved manifest')
        ids = [row['sample_id'] for row in rows]
        if len(ids) != len(set(ids)):
            raise ValueError('Duplicate sample ID')
        for row in rows:
            if row.get('source_split') != expected_split or row.get('id_verified') != 'True':
                raise ValueError('Split or exact ID verification failed: ' + row['sample_id'])
            if row.get('review_category') == 'C':
                raise ValueError('Rejected sample in approved manifest: ' + row['sample_id'])
            if not any(row.get(task + '_status') == 'approved' for task in enabled_tasks):
                raise ValueError('No enabled approved task: ' + row['sample_id'])
            for task in enabled_tasks:
                if row.get(task + '_status') == 'approved' and row.get(
                        'da_valid' if task == 'direct' else 'lane_valid') != 'True':
                    raise ValueError('Task approval/validity disagreement: ' + row['sample_id'])
        self.rows = rows
        self.enabled_tasks = frozenset(enabled_tasks)
        self.target = target

    def __len__(self) -> int:
        return len(self.rows)

    def __getitem__(self, index: int):
        row = self.rows[index]
        sid = row['sample_id']
        image = cv2.imread(row['image_path'], cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError('Image missing or undecodable: ' + sid)
        h, w = image.shape[:2]
        masks = {}
        for task, key, allowed in (('direct', 'direct_mask_path', {0, 1, 255}),
                                   ('lane', 'lane_mask_path', {0, 1, 2, 255})):
            use = task in self.enabled_tasks and row.get(task + '_status') == 'approved'
            if use:
                mask = cv2.imread(row[key], cv2.IMREAD_UNCHANGED)
                if mask is None or mask.shape != (h, w):
                    raise ValueError(task + ' mask missing or misaligned: ' + sid)
                if mask.ndim != 2 or not set(np.unique(mask).tolist()) <= allowed:
                    raise ValueError(task + ' mask has unexpected IDs: ' + sid)
                if not np.any(mask != 255):
                    raise ValueError(task + ' has no valid pixels: ' + sid)
                masks[task] = mask
            else:
                masks[task] = np.full((h, w), 255, dtype=np.uint8)
        image, da, lane = letterbox_pair(image, masks['direct'], masks['lane'], self.target)
        rgb = np.ascontiguousarray(image[:, :, ::-1].transpose(2, 0, 1))
        return {'image': torch.from_numpy(rgb).float().div_(255),
                'da': torch.from_numpy(np.ascontiguousarray(da)).long(),
                'lane': torch.from_numpy(np.ascontiguousarray(lane)).long(),
                'da_valid': torch.tensor('direct' in self.enabled_tasks and row['direct_status'] == 'approved'),
                'lane_valid': torch.tensor('lane' in self.enabled_tasks and row['lane_status'] == 'approved'),
                'sample_id': sid}
