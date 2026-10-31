"""Original formal-training helper definitions; unused historical v1 main omitted."""
from common import *
from copy import deepcopy
from datetime import datetime
import math
import torch
from torch import nn
import ultralytics

FINAL_KEYS={f'model.22.cv3.{i}.2.{part}' for i in range(3) for part in ['weight','bias']}

def role(name):
    if name in FINAL_KEYS:return 'class_output'
    layer=int(name.split('.')[1])
    if layer<10:return 'backbone'
    if layer<22:return 'neck'
    if '.cv2.' in name:return 'bbox'
    return 'class_hidden'

def configure(model,epoch):
    warm=epoch<=3
    for name,p in model.named_parameters():
        layer=int(name.split('.')[1]);enabled=not warm or name in FINAL_KEYS
        if layer<6 or (layer<10 and epoch<=10) or '.dfl.' in name:enabled=False
        p.requires_grad_(enabled)
    model.train()
    for module in model.modules():
        if isinstance(module,nn.modules.batchnorm._BatchNorm):module.eval()
    return warm

def build_optimizer(model):
    groups=[]
    for kind in ['backbone','neck','bbox','class_hidden','class_output']:
        for decay in [False,True]:
            params=[p for n,p in model.named_parameters() if role(n)==kind and (p.ndim>=2 and not '.dfl.' in n)==decay and not '.dfl.' in n]
            if params:groups.append({'params':params,'lr':0.,'weight_decay':1e-4 if decay else 0.,'role':kind})
    return torch.optim.AdamW(groups,lr=0.,betas=(.9,.999),eps=1e-8)

def set_lr(optimizer,epoch,updates):
    factor=.1+.9*.5*(1+math.cos(math.pi*min(max(epoch-4,0),36)/36))
    base={'backbone':1e-5 if epoch>=11 else 0.,'neck':3e-5,'bbox':3e-5,'class_hidden':3e-5,'class_output':3e-4}
    for group in optimizer.param_groups:
        group['lr']=(1e-3*min(1,(updates+1)/50) if group['role']=='class_output' else 0.) if epoch<=3 else base[group['role']]*factor
    return {g['role']:g['lr'] for g in optimizer.param_groups}

@torch.no_grad()
def update_ema(ema,model,updates,warm):
    decay=.9998*(1-math.exp(-updates/2000));params=dict(model.named_parameters());source=model.state_dict()
    for name,value in ema.state_dict().items():
        if name in params and params[name].requires_grad and value.is_floating_point():
            value.mul_(decay).add_(source[name].detach(),alpha=1-decay)
            if warm and name in FINAL_KEYS:value[:10].copy_(source[name][:10])
        else:value.copy_(source[name])

def checkpoint(path,model,epoch,metrics,updates,optimizer=None):
    saved=deepcopy(model).cpu().float().eval()
    for p in saved.parameters():p.requires_grad_(False)
    payload={'model':saved,'optimizer':optimizer.state_dict() if optimizer is not None else None,'epoch':epoch-1,
       'train_args':{'task':'detect','imgsz':640,'epochs':50,'batch':16,'seed':SEED,'model':'yolov8n','data':str(ASSET_ROOT/'configs/manifest.json')},
       'train_metrics':metrics,'version':ultralytics.__version__,'date':datetime.now().isoformat(),
       'actual_optimizer_updates':updates,'source_10class_sha256':INIT_SHA,'standard_detection_inference':True}
    tmp=path.with_suffix('.pt.tmp');torch.save(payload,tmp);tmp.replace(path)

def protection_state(model,warm):
    return {name:t.detach().cpu().clone() for name,t in model.state_dict().items()}

def check_warm_protection(model,before):
    current=model.state_dict();changed=[]
    for name,value in before.items():
        a=current[name].detach().cpu();a=a[:10] if name in FINAL_KEYS else a;b=value[:10] if name in FINAL_KEYS else value
        if not torch.equal(a,b):changed.append(name)
    if changed:raise RuntimeError(('Warmup old path changed',changed))
    return {'checked_tensors':len(before),'changed':changed,'old_rows_and_shared_paths_exact':True}
