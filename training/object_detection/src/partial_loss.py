"""Coverage-masked classification; standard TAL assignment and box/DFL supervision remain on known GT."""
import sys
sys.dont_write_bytecode=True
from common import *
from types import SimpleNamespace
import torch
import torch.nn.functional as F
from torch import nn
from ultralytics.utils.loss import v8DetectionLoss

class CoveredBCE(nn.Module):
    def __init__(self):super().__init__();self.coverage=None;self.last_active=None
    def forward(self,logits,targets):
        if self.coverage is None or self.coverage.shape!=(logits.shape[0],logits.shape[2]):raise ValueError('Missing aligned class coverage')
        active=self.coverage[:,None,:].expand_as(logits)|(targets>0)
        self.last_active=active.detach()
        # AMP computes BCE in FP32. Keep masked storage FP32 as well rather than casting loss back to half.
        output=torch.zeros_like(logits,dtype=torch.float32)
        output[active]=F.binary_cross_entropy_with_logits(logits[active],targets[active],reduction='none')
        return output

class PartialDetectionLoss(v8DetectionLoss):
    def __init__(self,model):
        original=model.args;model.args=SimpleNamespace(box=7.5,cls=.5,dfl=1.5)
        try:super().__init__(model)
        finally:model.args=original
        if self.nc!=11:raise ValueError('Requires exact standard11-class detection model')
        self.bce=CoveredBCE();self.last_fg=None;self.last_assigned=None
    def get_assigned_targets_and_loss(self,preds,batch):
        cov=batch['coverage'].to(self.device).bool()
        if len(batch['cls']):
            bi=batch['batch_idx'].long();ci=batch['cls'].view(-1).long()
            if not cov[bi,ci].all():raise ValueError('Positive label outside declared coverage')
        self.bce.coverage=cov
        try:
            result=super().get_assigned_targets_and_loss(preds,batch)
            self.last_fg=result[0][0].detach();self.last_assigned=result[0][1].detach()
            return result
        finally:self.bce.coverage=None

def teacher_retention(student,teacher,indices,coverage,foreground):
    """Soft training regularization, not extra GT: high-confidence unknown old-class cells outside known positives."""
    zero=student['scores'].float().sum()*0
    if teacher is None or len(indices)==0:return zero,{'selected_cells':0,'cls':0.,'dfl':0.}
    t_scores=teacher['scores'].float().detach().sigmoid().permute(0,2,1)
    s_scores=student['scores'][indices,:10].float().permute(0,2,1)
    unknown=(~coverage[indices,:10])[:,None,:].expand_as(t_scores)
    confident=(t_scores.masked_fill(~unknown,0).amax(-1)>.5)&(~foreground[indices])
    active=unknown&confident[:,:,None]
    if not active.any():return zero,{'selected_cells':0,'cls':0.,'dfl':0.}
    # Subtract teacher entropy so exact matching distributions have zero diagnostic KL value.
    target=t_scores[active];student_logits=s_scores[active]
    cls=(F.binary_cross_entropy_with_logits(student_logits,target,reduction='none')-
         F.binary_cross_entropy(target.clamp(1e-6,1-1e-6),target,reduction='none')).mean()
    s_dist=student['boxes'][indices].float().view(len(indices),4,16,-1).permute(0,3,1,2)[confident]
    t_dist=teacher['boxes'].float().detach().view(len(indices),4,16,-1).permute(0,3,1,2)[confident]
    dfl=F.kl_div(s_dist.log_softmax(-1),t_dist.softmax(-1),reduction='none').sum(-1).mean()
    return .25*cls+.05*dfl,{'selected_cells':int(confident.sum()),'cls':float(cls.detach()),'dfl':float(dfl.detach())}
