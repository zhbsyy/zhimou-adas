"""Same square640 input, top1 class-aware NMS .45/300, coverage-aware AP and fixed operating-point counts."""
import sys
sys.dont_write_bytecode=True
from common import *
from collections import defaultdict
import time
import numpy as np
import torch
import torchvision
from ultralytics.utils.metrics import box_iou,ap_per_class
from ultralytics.utils.ops import xywh2xyxy
from data_runtime import cuda_batch

IOUV=np.linspace(.5,.95,10)

def correct_predictions(pred,gt):
    correct=np.zeros((len(pred),10),dtype=bool)
    if not len(pred) or not len(gt):return correct
    overlaps=box_iou(gt[:,1:5],pred[:,:4])*(gt[:,0:1]==pred[:,5])
    overlaps=overlaps.cpu().numpy()
    # Exact current Ultralytics default greedy IoU matching (no Hungarian).
    for level,threshold in enumerate(IOUV):
        matches=np.array(np.nonzero(overlaps>=threshold)).T
        if len(matches)>1:
            matches=matches[overlaps[matches[:,0],matches[:,1]].argsort()[::-1]]
            matches=matches[np.unique(matches[:,1],return_index=True)[1]]
            matches=matches[np.unique(matches[:,0],return_index=True)[1]]
        if len(matches):correct[matches[:,1].astype(int),level]=True
    return correct

def nms_predictions(decoded,projection='full11'):
    if projection=='old10':decoded=decoded[:,:14]
    elif projection=='crosswalk':
        if decoded.shape[1]<15:return [torch.empty((0,6),device=decoded.device) for _ in range(len(decoded))]
        decoded=torch.cat((decoded[:,:4],decoded[:,14:15]),1)
    # Explicit compiled NMS has no wall-time break that silently leaves later batch images empty.
    # This is the same top1/conf/class-offset/IoU/maxdet math as the locked Ultralytics implementation.
    detections=[]
    for prediction in decoded.transpose(1,2):
        confidence,classes=prediction[:,4:].max(1)
        take=confidence>.001
        boxes=xywh2xyxy(prediction[take,:4]);confidence=confidence[take];classes=classes[take].float()
        if len(boxes):
            keep=torchvision.ops.nms(boxes+classes[:,None]*7680,confidence,.45)[:300]
            result=torch.cat((boxes[keep],confidence[keep,None],classes[keep,None]),1)
        else:result=torch.empty((0,6),device=decoded.device)
        detections.append(result)
    if projection=='crosswalk':
        for d in detections:
            if len(d):d[:,5]=10
    return detections

class Accumulator:
    def __init__(self):self.stats=[];self.fixed=defaultdict(lambda:np.zeros(3,dtype=np.int64));self.images=0;self.covered=np.zeros(11,dtype=int);self.raw_budget_hits=0
    def add(self,pred,gt,coverage):
        self.images+=1;self.covered+=coverage.cpu().numpy().astype(int)
        self.raw_budget_hits+=int(len(pred)>=300)
        # NMS budget/argmax applied to all classes BEFORE unknowns are removed.
        if len(pred):pred=pred[coverage[pred[:,5].long()]]
        if len(gt) and not coverage[gt[:,0].long()].all():raise ValueError('Evaluation GT outside coverage')
        correct=correct_predictions(pred,gt)
        self.stats.append((correct,pred[:,4].cpu().numpy(),pred[:,5].cpu().numpy(),gt[:,0].cpu().numpy()))
        operating=pred[pred[:,4]>.25];fixed_correct=correct_predictions(operating,gt)[:,0]
        for c in torch.where(coverage)[0].tolist():
            selected=operating[:,5].cpu().numpy()==c
            tp=int(fixed_correct[selected].sum());fp=int(selected.sum())-tp;fn=int((gt[:,0]==c).sum())-tp
            self.fixed[c]+=np.array([tp,fp,fn])
    def summary(self):
        correct=np.concatenate([s[0] for s in self.stats],0) if self.stats else np.zeros((0,10),bool)
        conf=np.concatenate([s[1] for s in self.stats]) if self.stats else np.array([])
        pc=np.concatenate([s[2] for s in self.stats]) if self.stats else np.array([])
        gt=np.concatenate([s[3] for s in self.stats]) if self.stats else np.array([])
        valid={}
        if len(gt):
            result=ap_per_class(correct,conf,pc,gt,plot=False,names=dict(enumerate(NAMES)))
            for i,c in enumerate(result[6]):valid[int(c)]={'AP50':float(result[5][i,0]),'AP50_95':float(result[5][i].mean()),
                   'P_at_best_mean_F1':float(result[2][i]),'R_at_best_mean_F1':float(result[3][i])}
        classes={}
        for c in range(11):
            tp,fp,fn=map(int,self.fixed[c]);count=int((gt==c).sum())
            classes[NAMES[c]]={'class_id':c,'covered_images':int(self.covered[c]),'GT':count,**valid.get(c,{'AP50':None if count==0 else 0.,'AP50_95':None if count==0 else 0.}),
                 'P25':tp/(tp+fp) if tp+fp else None,'R25':tp/(tp+fn) if tp+fn else None,'TP25':tp,'FP25':fp,'FN25':fn}
        ap=[r['AP50'] for r in classes.values() if r['AP50'] is not None]
        ap95=[r['AP50_95'] for r in classes.values() if r['AP50_95'] is not None]
        return {'images':self.images,'per_class':classes,'mAP50':float(np.mean(ap)) if ap else None,'mAP50_95':float(np.mean(ap95)) if ap95 else None,
                'at_maxdet300_images_before_coverage_filter':self.raw_budget_hits}

@torch.inference_mode()
def evaluate(model,dataloader,criterion=None,projection='full11',prediction_path=None):
    model.eval();accs=defaultdict(Accumulator);loss=np.zeros(3);batches=0;start=time.perf_counter()
    stream=Path(prediction_path).open('w',encoding='utf-8') if prediction_path else None
    try:
        for cpu in dataloader:
            batch=cuda_batch(cpu)
            with torch.autocast('cuda',enabled=False):
                decoded,raw=model(batch['img'])
                if criterion is not None:
                    terms,_=criterion(raw,batch);loss+=terms.float().cpu().numpy();batches+=len(batch['img'])
            preds=nms_predictions(decoded.float(),projection)
            for i,(pred,source) in enumerate(zip(preds,batch['source'])):
                if len(pred):
                    bounds=batch['bounds'][i];pred[:,0].clamp_(bounds[0],bounds[2]);pred[:,2].clamp_(bounds[0],bounds[2]);pred[:,1].clamp_(bounds[1],bounds[3]);pred[:,3].clamp_(bounds[1],bounds[3])
                    pred=pred[(pred[:,2]>pred[:,0])&(pred[:,3]>pred[:,1])]
                mask=batch['batch_idx']==i
                gt=torch.cat((batch['cls'][mask],xywh2xyxy(batch['bboxes'][mask])*640),1)
                accs[source].add(pred,gt,batch['coverage'][i]);accs['ALL_COVERED'].add(pred,gt,batch['coverage'][i])
                if stream:
                    d=pred.float().cpu().numpy();native=d.copy()
                    if len(native):native[:,[0,2]]-=batch['pad'][i][0];native[:,[1,3]]-=batch['pad'][i][1];native[:,:4]/=batch['ratio'][i]
                    stream.write(json.dumps({'id':batch['id'][i],'source':source,'prediction_native':native.tolist(),'coverage':torch.where(batch['coverage'][i])[0].tolist()})+'\n')
    finally:
        if stream:stream.close()
    return {'sources':{k:a.summary() for k,a in accs.items()},'validation_loss_mean':(loss/max(1,batches)).tolist(),
         'wall_seconds':time.perf_counter()-start,'protocol':{'input':'square640 letterbox no crop','precision':'FP32 model and forward; loss FP32','class_selection':'top1 before NMS',
         'conf_AP':.001,'conf_operating':.25,'nms_iou':.45,'class_aware':True,'max_det':300,'projection':projection,
         'matching':'Ultralytics greedy IoU one-to-one, IoU.50:.05:.95','AP':'installed locked Ultralytics ap_per_class,101-point interpolated integral',
         'unknown':'filter predictions/GT by per-image coverage after full NMS; GT outside coverage rejected'}}

def old_score(result):
    s=result['sources'];return .5*s['BDD100K']['mAP50_95']+.5*s['S2TLD']['per_class']['traffic light']['AP50_95']

def cw_score(result):
    return np.mean([result['sources'][s]['per_class']['crosswalk']['AP50_95'] for s in ['CeyMo','RoadV3_A']])

def retention(result,baseline):
    checks={}
    for source,classes in [('BDD100K',['person','car','traffic light','traffic sign']),('S2TLD',['traffic light'])]:
        for cls in classes:
            a=baseline['sources'][source]['per_class'][cls]['AP50'];b=result['sources'][source]['per_class'][cls]['AP50']
            limit=.03 if source=='S2TLD' else .05;checks[source+'/'+cls]={'baseline':a,'candidate':b,'delta':b-a,'allowed_drop':limit,'pass':b>=a-limit}
    a=baseline['sources']['BDD100K']['mAP50'];b=result['sources']['BDD100K']['mAP50'];checks['BDD_mean_AP50']={'baseline':a,'candidate':b,'delta':b-a,'allowed_drop':.03,'pass':b>=a-.03}
    return {'pass':all(r['pass'] for r in checks.values()),'checks':checks}
