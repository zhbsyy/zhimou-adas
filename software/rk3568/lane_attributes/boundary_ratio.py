"""Length-gated, per-boundary class ratio from R15 masks; no RGB color picking."""
import cv2
import numpy as np


def extract_boundaries_ratio(direct,lane,cfg):
    h,w=lane.shape
    count=cfg.get('rows',24)
    ys=np.unique(np.linspace(int(cfg.get('row_end',.94)*(h-1)),
                             int(cfg.get('row_start',.45)*(h-1)),count).astype(int))[::-1]
    spacing=max(1.,(int(ys[0])-int(ys[-1]))/max(1,len(ys)-1))
    threshold=cfg.get('dashed_ratio_threshold',.4)
    mode=cfg.get('anchor_mode','road_center')
    minimum=cfg.get('min_length_ratio',.06)*h
    min_span=cfg.get('min_vertical_span_ratio',.04)*h
    min_samples=cfg.get('min_samples',4)
    min_pixels=cfg.get('min_marked_pixels',24)
    component=None
    selection={'anchor_mode':mode,'measurement_size':[w,h],
               'class_1':'solid_red_overlay','class_2':'dashed_yellow_overlay',
               'road_pixels':0,'anchor_x':None}

    def unknown(reason,samples=0,**data):
        return {'kind':'unknown','valid':False,'reason':reason,'samples':samples,'points':[],
                'method':'length_ratio','dashed_ratio_threshold':threshold,
                'min_length_px':round(minimum,2),**data}

    def both_unknown(reason):
        return {'left':unknown(reason),'right':unknown(reason),'selection':selection}

    if mode=='road_center':
        road=(direct==1).astype(np.uint8)
        number,labels,stats,centroids=cv2.connectedComponentsWithStats(road,connectivity=8)
        if number<=1:
            return both_unknown('no_road_component')
        support=np.bincount(labels[int(ys[-1]):int(ys[0])+1].ravel(),minlength=number)
        support[0]=0
        order=np.argsort(support)[::-1]
        best=int(order[0]);area=int(support[best])
        selection['road_pixels']=area
        if area<max(24,cfg.get('min_road_area_ratio',.002)*h*w):
            return both_unknown('road_component_too_small')
        if len(order)>1 and support[order[1]]>=.75*area:
            return both_unknown('ambiguous_road_components')
        component=labels==best
    elif mode!='frame_center':
        raise ValueError('Unknown boundary anchor_mode')

    points={'left':[],'right':[]}
    anchors=[]
    max_width=cfg.get('max_boundary_width_ratio',.06)*w
    for y in ys:
        y0,y1=max(0,y-1),min(h,y+2)
        if component is not None:
            extent=np.flatnonzero(np.any(component[y0:y1],axis=0))
            if len(extent)<4:
                continue
            anchor=float((extent[0]+extent[-1])/2)
        else:
            anchor=cfg.get('anchor_x',.5)*w
            a,b=max(0,int(anchor-.08*w)),min(w,int(anchor+.08*w)+1)
            band=direct[y0:y1,a:b]
            if not band.size or np.mean(band==1)<.25:
                continue
        anchors.append(anchor/w)
        band=lane[y0:y1]
        marked=np.any((band==1)|(band==2),axis=0)
        edges=np.diff(np.pad(marked.astype(np.int8),(1,1)))
        runs=list(zip(np.flatnonzero(edges==1),np.flatnonzero(edges==-1)))
        for side in ('left','right'):
            candidates=[(a,b) for a,b in runs if b-a<=max_width and
                        (.015*w<=anchor-(a+b)/2<=.47*w if side=='left' else
                         .015*w<=(a+b)/2-anchor<=.47*w)]
            candidates.sort(key=lambda r:abs((r[0]+r[1])/2-anchor))
            if not candidates:
                continue
            a,b=candidates[0]
            x=float((a+b)/2)
            old=points[side]
            if old:
                last_x,last_y,*_=old[-1]
                predicted=last_x
                if len(old)>1:
                    prev_x,prev_y,*_=old[-2]
                    predicted+=(last_x-prev_x)*(y-last_y)/min(-1,last_y-prev_y)
                if abs(x-predicted)>.08*w+.6*w*abs(y-last_y)/h:
                    continue
            labels_here=band[:,a:b]
            solid=int(np.count_nonzero(labels_here==1))
            dashed=int(np.count_nonzero(labels_here==2))
            old.append((x,int(y),solid,dashed))
    if anchors:
        selection['anchor_x']=round(float(np.median(anchors)),4)
    result={'selection':selection}
    for side,found in points.items():
        if len(found)<min_samples:
            result[side]=unknown('insufficient_boundary_samples',len(found))
            continue
        # Integrate only observed sampling cells. Empty dash gaps are not
        # counted as solid paint or as a long connecting line.
        weights=[]
        for i,(x,y,solid,dashed) in enumerate(found):
            slopes=[]
            for j in (i-1,i+1):
                if not 0<=j<len(found):continue
                nx,ny,*_=found[j]
                dy=abs(ny-y)
                if 0<dy<=1.8*spacing:
                    slopes.append(abs(nx-x)/dy)
            # At most two neighboring slopes; their median equals their mean.
            slope=min(2.,sum(slopes)/len(slopes)) if slopes else 0.
            weights.append(spacing*(1+slope*slope)**.5)
        length=float(sum(weights))
        vertical_span=float(max(p[1] for p in found)-min(p[1] for p in found))
        marked_pixels=sum(p[2]+p[3] for p in found)
        yellow=sum(weight*p[3]/(p[2]+p[3]) for weight,p in zip(weights,found))
        ratio=yellow/length
        metrics={'estimated_length_px':round(length,2),'length_fraction':round(length/h,4),
                 'vertical_span_px':round(vertical_span,2),'marked_pixels':marked_pixels,
                 'dashed_fraction':round(ratio,6),'solid_fraction':round(1-ratio,6),
                 'yellow_length_px':round(yellow,2),'red_length_px':round(length-yellow,2)}
        if length<minimum or vertical_span<min_span or marked_pixels<min_pixels:
            result[side]=unknown('boundary_too_short',len(found),**metrics)
            continue
        reaches_threshold=ratio>=threshold or abs(ratio-threshold)<=1e-12
        result[side]={'kind':'dashed' if reaches_threshold else 'solid','valid':True,
                      'reason':'length_and_ratio_passed','method':'length_ratio',
                      'samples':len(found),'dashed_ratio_threshold':threshold,
                      'min_length_px':round(minimum,2),**metrics,
                      'points':[[round(x/w,4),round(y/h,4)] for x,y,*_ in found]}
    return result
