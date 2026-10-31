"""Bounded ROI stereo matcher. No extra RKNN inference; no size-to-metre guess."""
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np
from stereo_source import validate_pair


def iou(a, b):
    ix = max(0, min(a[2], b[2])-max(a[0], b[0]))
    iy = max(0, min(a[3], b[3])-max(a[1], b[1]))
    inter = ix*iy
    return inter/max(1., (a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-inter)


class RoiStereoRanger:
    def __init__(self, calibration_path, confirmed=False, max_targets=6):
        raw = Path(calibration_path).read_bytes()
        d = json.loads(raw.decode('utf-8-sig'))
        if d.get('image_size') != [640, 360]:
            raise ValueError('Calibration must describe the original 640x360 eye geometry')
        for k, shape in [('K1',(3,3)),('K2',(3,3)),('R1',(3,3)),('R2',(3,3)),('P1',(3,4)),('P2',(3,4))]:
            v = np.asarray(d[k], np.float64)
            if v.shape != shape or not np.isfinite(v).all():
                raise ValueError('Invalid calibration '+k)
            setattr(self, k, v)
        self.f, self.cx = self.P1[0,0], self.P1[0,2]
        self.B = abs(float(self.P2[0,3]/self.P2[0,0]))/1000.
        if not (100 < self.f < 3000 and .005 < self.B < .3):
            raise ValueError('Invalid rectified focal length or baseline')
        self.maps = []
        for eye in (1,2):
            K, R, P = (getattr(self,k+str(eye)) for k in ('K','R','P'))
            D = np.asarray(d['D'+str(eye)],np.float64)
            if not np.isfinite(D).all():
                raise ValueError('Nonfinite distortion')
            self.maps.append(cv2.initUndistortRectifyMap(K,D,R,P[:,:3],(640,360),cv2.CV_16SC2))
        self.D1 = np.asarray(d['D1'],np.float64)
        self.confirmed, self.max_targets = bool(confirmed), max_targets
        self.sha256 = hashlib.sha256(raw).hexdigest()
        self.tracks = []

    def describe(self):
        return {'method':'rectified_roi_NCC_roundtrip', 'calibration_sha256':self.sha256,
                'calibration_confirmed':self.confirmed, 'eye_size':[640,360],
                'baseline_m':self.B, 'exposure_synchronized':False,
                'validity_scope':'static_or_quasistatic_only', 'extra_NPU_calls':0}

    def _patch(self, L, R, x, y):
        radius = 10
        x, y = int(round(x)), int(round(y))
        if not (radius+108 < x < 640-radius and radius+4 < y < 360-radius-4):
            return None
        tpl = L[y-radius:y+radius+1,x-radius:x+radius+1]
        if float(tpl.std()) < 8:
            return None
        # 0.2..8 m is the legacy calibrated module's diagnostic range.
        dmin, dmax = max(1,int(self.f*self.B/8)), min(108,int(self.f*self.B/.2)+1)
        sx0, sx1 = x-dmax-radius, x-dmin+radius+1
        sy0, sy1 = y-radius-3,y+radius+4
        scores = cv2.matchTemplate(R[sy0:sy1,sx0:sx1],tpl,cv2.TM_CCOEFF_NORMED)
        _, best, _, (mx,my) = cv2.minMaxLoc(scores)
        other = scores.copy()
        other[max(0,my-1):my+2,max(0,mx-3):mx+4] = -1
        if not np.isfinite(best) or best < .70 or best-float(other.max()) < .04:
            return None
        sub = 0.
        if 0 < mx < scores.shape[1]-1:
            a,b,c = map(float,scores[my,mx-1:mx+2]); den=a-2*b+c
            if den < -1e-6: sub=float(np.clip(.5*(a-c)/den,-.5,.5))
        xr, yr = sx0+mx+radius+sub, sy0+my+radius
        rx, ry = int(round(xr)),int(round(yr))
        back = R[ry-radius:ry+radius+1,rx-radius:rx+radius+1]
        if back.shape != tpl.shape: return None
        search = L[y-radius-3:y+radius+4,x-radius-4:x+radius+5]
        _, rscore, _, (bx,by) = cv2.minMaxLoc(cv2.matchTemplate(search,back,cv2.TM_CCOEFF_NORMED))
        rt = float(np.hypot(bx-4,by-3))
        disparity = x-xr
        if rscore < .70 or rt > 1.5 or abs(y-yr)>2 or disparity <= 0:
            return None
        z=self.f*self.B/disparity
        if not .2 <= z <= 8: return None
        return {'z_m':z, 'lateral_m':(x-self.cx)*z/self.f,'score':float(best),'roundtrip_px':rt}

    def measure(self, left, right, detections, names, stamp):
        ok, reason = validate_pair(left, right)
        if not ok:
            self.tracks=[]
            return {'valid':False,'reason':reason,'targets':[],**self.describe()}
        # Both rectifications happen once per detector result, not per captured frame.
        L,R=[cv2.remap(cv2.cvtColor(im,cv2.COLOR_BGR2GRAY),*maps,cv2.INTER_LINEAR)
             for im,maps in zip((left,right),self.maps)]
        selected=[]
        for row in detections:
            row=np.asarray(row,dtype=float)
            if row.shape != (6,) or not np.isfinite(row).all(): continue
            x1,y1,x2,y2,score,cls=row
            if not (int(cls)==cls and 0<=int(cls)<len(names) and .25<=score<=1 and x2>x1 and y2>y1):continue
            name=str(names[int(cls)])
            selected.append((0 if name.lower() in ('person','pedestrian','行人') else 1,-score,row,name))
        selected.sort(key=lambda x:x[:2])
        out,new_tracks=[],[]
        for _,_,row,name in selected[:self.max_targets]:
            box=row[:4].tolist(); x1,y1,x2,y2=box
            points=np.array([[(x1+x2)/2,y1+(y2-y1)*v] for v in (.40,.55,.70)],np.float64).reshape(-1,1,2)
            rect=cv2.undistortPoints(points,self.K1,self.D1,R=self.R1,P=self.P1[:,:3]).reshape(-1,2)
            patches=[r for x,y in rect if (r:=self._patch(L,R,x,y)) is not None]
            target={'box':box,'class_id':int(row[5]),'name':name,'confidence':float(row[4]),
                    'distance_m':None,'candidate_distance_m':None,'lateral_m':None,'valid':False}
            if len(patches)<2:
                target['reason']='insufficient_consistent_texture'
            else:
                zs=[r['z_m'] for r in patches]; z=float(np.median(zs))
                previous=max((t for t in self.tracks if t['class_id']==int(row[5]) and 0<stamp-t['at']<=.5),
                             key=lambda t:iou(t['box'],box),default=None)
                stable=bool(previous and iou(previous['box'],box)>.5 and
                            np.linalg.norm((np.array(box[:2])+box[2:])/2-(np.array(previous['box'][:2])+previous['box'][2:])/2)<=6 and
                            abs(previous['z']-z)<=max(.1,z*.10))
                consistent=max(zs)-min(zs)<=max(.08,z*.15)
                target.update(candidate_distance_m=round(z,3),lateral_m=round(float(np.median([p['lateral_m'] for p in patches])),3),
                              matching_score=min(p['score'] for p in patches),stable=stable,
                              reason='calibration_unverified' if not self.confirmed else 'moving_or_first_observation' if not stable else
                              'inconsistent_patches' if not consistent else 'static_match_accepted')
                target['valid']=bool(self.confirmed and stable and consistent)
                if target['valid']: target['distance_m']=round(z,3)
                new_tracks.append({'class_id':int(row[5]),'box':box,'z':z,'at':stamp})
            out.append(target)
        self.tracks=new_tracks
        return {'valid':True,'reason':'pair_available','targets':out,**self.describe()}
