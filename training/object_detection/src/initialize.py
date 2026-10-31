"""Expand only the three final class convolutions of the exact existing standard Detect object."""
import sys
sys.dont_write_bytecode=True
from common import *
from copy import deepcopy
from datetime import datetime
import math
import random
import platform
import torch
import numpy as np
from torch import nn
import cv2
import ultralytics
from ultralytics import YOLO
from ultralytics.nn.tasks import DetectionModel
from ultralytics.nn.modules.head import Detect

def seed():
    random.seed(SEED);np.random.seed(SEED);torch.manual_seed(SEED);torch.cuda.manual_seed_all(SEED)
    torch.backends.cudnn.benchmark=False;torch.backends.cuda.matmul.allow_tf32=False;torch.backends.cudnn.allow_tf32=False
    torch.set_num_threads(6)

def image_tensor(path):
    im=cv2.imread(str(path));assert im is not None
    h,w=im.shape[:2];r=640/max(h,w);nw,nh=round(w*r),round(h*r)
    small=cv2.resize(im,(nw,nh));canvas=np.full((640,640,3),114,np.uint8);x=(640-nw)//2;y=(640-nh)//2;canvas[y:y+nh,x:x+nw]=small
    return torch.from_numpy(np.ascontiguousarray(canvas[:,:,::-1].transpose(2,0,1))).float().div(255).unsqueeze(0)

def main():
    assert sha(INIT)==INIT_SHA and ultralytics.__version__=='8.4.155'
    seed();source=YOLO(str(INIT)).model.float().eval();assert type(source) is DetectionModel
    assert source.names==dict(enumerate(NAMES[:10])),source.names
    target=deepcopy(source);h=target.model[-1];assert type(h) is Detect and h.nc==10 and h.reg_max==16 and not h.end2end
    assert len(target.model)==23 and h.f==[15,18,21]
    for i in range(h.nl):
        old=h.cv3[i][-1];assert type(old) is nn.Conv2d and old.out_channels==10 and old.in_channels==64
        new=nn.Conv2d(old.in_channels,11,kernel_size=old.kernel_size,stride=old.stride,padding=old.padding,
                      dilation=old.dilation,groups=old.groups,bias=old.bias is not None,padding_mode=old.padding_mode)
        with torch.no_grad():
            new.weight[:10].copy_(old.weight);new.bias[:10].copy_(old.bias)
            nn.init.normal_(new.weight[10:],mean=0,std=.01)
            new.bias[10:].fill_(math.log(5/11/(640/float(h.stride[i]))**2))
        h.cv3[i][-1]=new
    h.nc=11;h.no=h.nc+h.reg_max*4;target.names=dict(enumerate(NAMES));target.yaml=deepcopy(source.yaml);target.yaml['nc']=11
    oldstate=source.state_dict();newstate=target.state_dict();trace=[]
    final={f'model.22.cv3.{i}.2.{part}' for i in range(3) for part in ['weight','bias']}
    assert set(oldstate)==set(newstate)
    for key,value in oldstate.items():
        actual=newstate[key][:10] if key in final else newstate[key]
        assert torch.equal(value,actual),key
        trace.append({'tensor':key,'old_shape':list(value.shape),'new_shape':list(newstate[key].shape),
             'old_values_exact':True,'policy':'copy_old_rows0_9; new_row10 seeded' if key in final else 'entire_tensor_inherited'})
    tests=[];manifest=read(OLD/'dataset_manifest.json')['images']
    paths=[OLD/'dataset/images'/r['split']/r['name'] for src in ['BDD100K','S2TLD'] for r in [next(x for x in manifest if x['source']==src and x['split']=='val')]]
    g5=read(CD/'g5_preparation_20261019/configs/final_manifest.json')['records']
    paths += [Path(next(x for x in g5 if x['partition']==p)['image']) for p in ['cdset_train','ceymo_train']]
    source=source.cuda();target=target.cuda()
    with torch.inference_mode():
        for path in paths:
            x=image_tensor(path).cuda();a,ar=source(x);b,br=target(x)
            err={'decoded_bbox':float((a[:,:4]-b[:,:4]).abs().max()),'old_decoded_scores':float((a[:,4:14]-b[:,4:14]).abs().max()),
                 'box_logits':float((ar['boxes']-br['boxes']).abs().max()),'old_class_logits':float((ar['scores']-br['scores'][:,:10]).abs().max()),
                 'features':[float((u-v).abs().max()) for u,v in zip(ar['feats'],br['feats'])]}
            assert err['decoded_bbox']<1e-5 and err['old_decoded_scores']<1e-6 and err['box_logits']<1e-6 and err['old_class_logits']<1e-5,err
            tests.append({'image':str(path),'input_shape':list(x.shape),'errors':err})
    target=target.cpu().float();target.eval()
    path=ROOT/'weights/eleven_class_untrained_init.pt';assert not path.exists()
    torch.save({'model':target,'optimizer':None,'epoch':-1,'train_args':dict(source.args) if isinstance(source.args,dict) else vars(source.args),
          'version':ultralytics.__version__,'date':datetime.now().isoformat(),'initialization_source_sha256':INIT_SHA,
          'class_names':NAMES,'zero_training':True},path)
    loaded=YOLO(str(path));assert loaded.names==dict(enumerate(NAMES)) and type(loaded.model) is DetectionModel
    report={'source_sha256':INIT_SHA,'source_architecture':'ultralytics.nn.tasks.DetectionModel','source_names':source.names,
         'target_names':target.names,'source_parameters':sum(p.numel() for p in source.parameters()),'target_parameters':sum(p.numel() for p in target.parameters()),
         'state_tensors':len(trace),'trace':trace,'zero_training_forward_tests':tests,'all_old_tensor_values_exact':True,
         'bbox_regression_and_DFL_inherited':True,'class_index_shift':False,'shared_paths_rebuilt':False,
         'new_class_bias_rule':'log(5/11/(640/stride)^2)','new_class_weight_rule':'normal(0,.01),seed20261020',
         'initialization_path':str(path),'initialization_sha256':sha(path),'standard_YOLO_reload':'PASS',
         'environment':{'python':platform.python_version(),'torch':torch.__version__,'ultralytics':ultralytics.__version__,
         'numpy':np.__version__,'cuda':torch.version.cuda,'gpu':torch.cuda.get_device_name(0),'runtime':sys.executable,
         'dependencies':'read-only train_china_road .venv site-packages via new local venv .pth; configs/temp/caches isolated'}}
    write(ROOT/'audit/initialization.json',report);print(json.dumps({k:v for k,v in report.items() if k not in ['trace','zero_training_forward_tests']},ensure_ascii=False,indent=2))

if __name__=='__main__':main()
