"""Paths and isolated runtime configuration for the authorized eleven-class Detect run."""
import sys
sys.dont_write_bytecode=True
import os
import yaml
import json
import hashlib
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
_data=yaml.safe_load((ROOT/'data_config.yaml').read_text(encoding='utf-8'))
ASSET_ROOT=Path(os.environ.get('YOLO_ASSET_ROOT',_data['asset_root'])).resolve()
OLD=Path(os.environ.get('YOLO_OLD_ROOT',_data['old_project_root'])).resolve()
SEG=Path(os.environ.get('YOLO_SEG_ROOT',_data['seg_project_root'])).resolve()
CD=Path(os.environ.get('YOLO_CD_ROOT',_data['cd_project_root'])).resolve()
NAMES=['person','rider','car','truck','bus','train','motor','bike','traffic light','traffic sign','crosswalk']
SEED=20261020
INIT=ASSET_ROOT/'weights/china_road_10class_INIT.pt'
INIT_SHA='90c0fd4b6a7d30dc9fea80e6f6a5b9e789a1da219dcd8b9acb3e5ba4bad4bf5e'
for key,folder in [('YOLO_CONFIG_DIR','.ultralytics'),('MPLCONFIGDIR','.mpl_config'),('TORCH_HOME','.torch_cache'),('TEMP','.tmp'),('TMP','.tmp')]:
    target=ROOT/'environment'/folder;target.mkdir(parents=True,exist_ok=True);os.environ[key]=str(target)
os.environ['MPLBACKEND']='Agg'
os.environ['YOLO_AUTOINSTALL']='False'
os.environ['PYTHONDONTWRITEBYTECODE']='1'

def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))

def write(p,obj):
    p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
    p.write_text(json.dumps(obj,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')

def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(8*1024*1024),b''):h.update(b)
    return h.hexdigest()
