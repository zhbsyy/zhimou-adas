"""Single-image bbox-preserving transforms with explicit per-image class coverage."""
import sys
sys.dont_write_bytecode=True
from common import *
import random
import cv2
import numpy as np
import torch
from torch.utils.data import Dataset,DataLoader

class DetectionData(Dataset):
    def __init__(self,records,augment=False):self.records=records;self.augment=augment
    def __len__(self):return len(self.records)
    def __getitem__(self,index):
        r=self.records[index];im=cv2.imread(r['image'])
        if im is None:raise ValueError(('Unreadable frozen image',r['id']))
        h,w=im.shape[:2];ratio=min(640/h,640/w);nw,nh=round(w*ratio),round(h*ratio)
        small=cv2.resize(im,(nw,nh),interpolation=cv2.INTER_LINEAR)
        dx,dy=(640-nw)//2,(640-nh)//2
        canvas=np.full((640,640,3),114,np.uint8);canvas[dy:dy+nh,dx:dx+nw]=small
        boxes=np.array([[c,*b] for c,b in r['boxes']],dtype=np.float32).reshape(-1,5)
        if len(boxes):
            boxes[:,1]=(boxes[:,1]*w*ratio+dx)/640;boxes[:,2]=(boxes[:,2]*h*ratio+dy)/640
            boxes[:,3]*=w*ratio/640;boxes[:,4]*=h*ratio/640
        bounds=np.array([dx,dy,dx+w*ratio,dy+h*ratio],dtype=np.float32)
        if self.augment:
            # Photometric changes and one-image reflection preserve whole-instance bbox definition.
            gains=np.random.uniform(-1,1,3)*np.array([.015,.5,.3])+1
            hue,sat,val=cv2.split(cv2.cvtColor(canvas,cv2.COLOR_BGR2HSV));x=np.arange(256)
            hsv=cv2.merge((cv2.LUT(hue,((x*gains[0])%180).astype('uint8')),
                           cv2.LUT(sat,np.clip(x*gains[1],0,255).astype('uint8')),
                           cv2.LUT(val,np.clip(x*gains[2],0,255).astype('uint8'))))
            canvas=cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)
            if random.random()<.5:
                canvas=np.fliplr(canvas)
                if len(boxes):boxes[:,1]=1-boxes[:,1]
                bounds[[0,2]]=640-bounds[[2,0]]
        coverage=torch.zeros(11,dtype=torch.bool);coverage[r['coverage']]=True
        if len(boxes) and not coverage[torch.from_numpy(boxes[:,0].astype(np.int64))].all():raise ValueError('GT outside coverage')
        return {'img':torch.from_numpy(np.ascontiguousarray(canvas[:,:,::-1].transpose(2,0,1))),
                'cls':torch.from_numpy(boxes[:,:1]),'bboxes':torch.from_numpy(boxes[:,1:]),'coverage':coverage,
                'id':r['id'],'im_file':r['image'],'source':r['source'],'bounds':torch.from_numpy(bounds),
                'partition':r['partition'],'native_hw':(h,w),'ratio':ratio,'pad':(dx,dy)}

def collate(items):
    result={k:[r[k] for r in items] for k in ['id','im_file','source','partition','native_hw','ratio','pad']}
    for k in ['img','coverage','bounds']:result[k]=torch.stack([r[k] for r in items])
    for k in ['cls','bboxes']:result[k]=torch.cat([r[k] for r in items],0)
    result['batch_idx']=torch.cat([torch.full((len(r['cls']),),i,dtype=torch.float32) for i,r in enumerate(items)])
    return result

def worker_seed(worker):
    seed=torch.initial_seed()%2**32;np.random.seed(seed);random.seed(seed);cv2.setNumThreads(0);torch.set_num_threads(1)

def loader(records,batch=16,augment=False,workers=0,shuffle=False):
    generator=torch.Generator().manual_seed(SEED)
    return DataLoader(DetectionData(records,augment),batch_size=batch,shuffle=shuffle,num_workers=workers,
       collate_fn=collate,pin_memory=True,worker_init_fn=worker_seed,generator=generator,
       **({'persistent_workers':True,'prefetch_factor':2} if workers else {}))

def cuda_batch(batch):
    out=dict(batch)
    for key in ['img','coverage','cls','bboxes','batch_idx','bounds']:out[key]=batch[key].cuda(non_blocking=True)
    out['img']=out['img'].float().div_(255)
    return out
