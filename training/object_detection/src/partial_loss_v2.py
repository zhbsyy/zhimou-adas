"""Revision 2: identical coverage/TAL semantics; all detection loss arithmetic in FP32."""
from common import *
import torch
from partial_loss import PartialDetectionLoss as OriginalLoss, teacher_retention


class PartialDetectionLoss(OriginalLoss):
    def get_assigned_targets_and_loss(self, preds, batch):
        # Casting remains differentiable. Assignment, anchors, softmax, CIoU, DFL,
        # BCE and their reductions must not inherit FP16/BF16 autocast.
        with torch.autocast('cuda', enabled=False):
            fp32 = {**preds, 'boxes': preds['boxes'].float(),
                    'scores': preds['scores'].float(),
                    'feats': [f.float() for f in preds['feats']]}
            return super().get_assigned_targets_and_loss(fp32, batch)


def distillation(student, teacher, indices, coverage, foreground):
    with torch.autocast('cuda', enabled=False):
        return teacher_retention(student, teacher, indices, coverage, foreground)
