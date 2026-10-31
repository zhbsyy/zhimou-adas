"""Pedestrian risk summaries for stationary demonstrations; no actuator writes."""
import math

LEVEL = {'clear':0,'warning':1,'slow':2,'emergency':3}
PERSON = {'person','pedestrian','行人'}


def validate_config(config):
    c = dict(config or {})
    if type(c.get('enabled',False)) is not bool or c.get('mode','video_demo') not in ('video_demo','stereo_distance'):
        raise ValueError('Invalid AEB enabled/mode')
    for key, default in [('min_confidence',.5),('release_hold_s',1.2),('ttl_s',.6),
                         ('warning_height_ratio',.15),('slow_height_ratio',.25),('brake_height_ratio',.40),
                         ('warning_m',3.),('slow_m',2.),('brake_m',1.)]:
        v=c.setdefault(key,default)
        if type(v) not in (int,float) or not math.isfinite(v) or v<=0:
            raise ValueError('Invalid AEB '+key)
    if not 0<c['min_confidence']<=1 or not 0<c['warning_height_ratio']<c['slow_height_ratio']<c['brake_height_ratio']<=1:
        raise ValueError('Invalid AEB image thresholds')
    if not .2<=c['brake_m']<c['slow_m']<c['warning_m']<=8:
        raise ValueError('Invalid AEB diagnostic distance thresholds')
    c.setdefault('confirm_samples',2)
    if type(c['confirm_samples']) is not int or not 1<=c['confirm_samples']<=8:
        raise ValueError('Invalid AEB confirm_samples')
    return c


class PedestrianMonitor:
    def __init__(self, config=None):
        self.config=validate_config(config)
        self.reset()

    def reset(self):
        self.previous=None
        self.confirmed_count=0
        self.brake_active=False
        self.clear_since=None
        self.last_stamp=None

    def update(self, detections, names, shape, stamp, ranges=None):
        c=self.config
        result={'mode':c.get('mode','video_demo'),'level':'clear','valid':True,
                'distance_m':None,'ttc_s':None,'targets':[],'reason':'no_corridor_pedestrian',
                'metric_distance_used':False,'simulation_only':True,
                'range':ranges,'exposure_synchronized':False}
        if not c.get('enabled',False):
            result.update(level='disabled',valid=False,reason='AEB_disabled');return result
        if self.last_stamp is not None and (stamp<=self.last_stamp or stamp-self.last_stamp>c['ttl_s']):
            self.previous=None;self.confirmed_count=0;self.clear_since=None
        self.last_stamp=stamp
        h,w=shape[:2]; best=None; needs_range=False
        for row in detections:
            if len(row)!=6 or not all(math.isfinite(float(v)) for v in row):continue
            x1,y1,x2,y2,conf,cls=map(float,row)
            if int(cls)!=cls or not 0<=int(cls)<len(names) or str(names[int(cls)]).lower() not in PERSON:continue
            if not c['min_confidence']<=conf<=1 or not 0<=x1<x2<=w or not 0<=y1<y2<=h:continue
            # A configurable demonstration corridor narrows towards the horizon.
            foot=((x1+x2)/2/w,y2/h); t=max(0.,min(1.,(foot[1]-.45)/.55))
            half=.10+.18*t
            if foot[1]<.45 or abs(foot[0]-.5)>half:continue
            ratio=(y2-y1)/h
            level='warning';distance=None
            if result['mode']=='video_demo':
                if ratio<c['warning_height_ratio']:continue
                level='emergency' if ratio>=c['brake_height_ratio'] else 'slow' if ratio>=c['slow_height_ratio'] else 'warning'
            else:
                match=next((r for r in (ranges or {}).get('targets',[]) if r.get('class_id')==int(cls) and
                            len(r.get('box',[]))==4 and all(abs(float(a)-b)<.01 for a,b in zip(r['box'],(x1,y1,x2,y2)))),None)
                if not match or not match.get('valid'):
                    needs_range=True;continue
                distance=match.get('distance_m');lateral=match.get('lateral_m')
                if not all(type(v) in (int,float) and math.isfinite(v) for v in (distance,lateral)):
                    needs_range=True;continue
                if abs(lateral)>.65 or not .2<=distance<=8:continue
                if distance>c['warning_m']:continue
                level='emergency' if distance<=c['brake_m'] else 'slow' if distance<=c['slow_m'] else 'warning'
            target={'box':[x1,y1,x2,y2],'confidence':conf,'height_ratio':round(ratio,4),
                    'footpoint_normalized':list(foot),'distance_m':distance,'level':level}
            result['targets'].append(target)
            if best is None or LEVEL[level]>LEVEL[best['level']]:best=target
        if best:
            from stereo_roi_range import iou
            if self.previous and iou(self.previous['box'],best['box'])>.3 and self.previous['level']==best['level']:
                self.confirmed_count+=1
            else:self.confirmed_count=1
            self.previous=best;self.clear_since=None
            result.update(level=best['level'] if self.confirmed_count>=c['confirm_samples'] else 'warning',
                          reason='pedestrian_in_front' if self.confirmed_count>=c['confirm_samples'] else 'confirming_pedestrian',
                          distance_m=best['distance_m'],metric_distance_used=best['distance_m'] is not None)
            if result['level']=='emergency':self.brake_active=True
        else:
            self.previous=None;self.confirmed_count=0
            if needs_range:
                result.update(level='unknown',valid=False,reason='pedestrian_range_unavailable')
                self.clear_since=None
            else:
                if self.clear_since is None:self.clear_since=stamp
                if stamp-self.clear_since>=c['release_hold_s']:self.brake_active=False
        if result['mode']=='stereo_distance' and (not ranges or not ranges.get('valid') or not ranges.get('calibration_confirmed')):
            result.update(level='unknown',valid=False,reason='stereo_pair_or_calibration_unavailable')
            self.clear_since=None
        if self.brake_active:
            result.update(level='emergency',reason='pedestrian_brake_hold',valid=True)
        return result
