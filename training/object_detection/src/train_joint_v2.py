"""One authorized replacement formal run. No automatic retry or precision changes."""
from common import *
from copy import deepcopy
from datetime import datetime
import argparse
import math
import shutil
import time
import traceback
import torch
import numpy as np
from ultralytics import YOLO
from initialize import seed
from data_runtime import loader, cuda_batch
from data_runtime_v2 import epoch_loader
from partial_loss_v2 import PartialDetectionLoss, distillation
from numerical_runtime_v2 import *
from evaluate_v2 import evaluate, old_score, cw_score, retention
from train_joint import (FINAL_KEYS, configure, build_optimizer, set_lr, update_ema,
                         checkpoint, protection_state, check_warm_protection)

REV = ASSET_ROOT/'numerical_revision_20261020'
RUN = ROOT/'runs/formal_joint_v2_bf16_seed20261020'
PRECISION = 'bf16'


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--resume-state', help='Explicit recovery only; never automatically invoked')
    args=parser.parse_args()
    seed(); __import__('cv2').setNumThreads(0)
    assert read(REV/'NUMERICAL_GATE.json')['status']=='PASS'
    assert torch.cuda.is_bf16_supported()
    lock=read(ASSET_ROOT/'configs/MANIFEST_FROZEN.json')
    assert sha(ASSET_ROOT/'configs/manifest.json')==lock['manifest_sha256']
    assert sha(INIT)==INIT_SHA==sha(OLD/'exports/china_road_yolov8n_best.pt')
    init_path=ROOT/'audit/initialization.json'
    init=read(init_path if init_path.exists() else ASSET_ROOT/'audit/initialization.json')
    if not init_path.exists():init['initialization_path']=str(ASSET_ROOT/'weights/eleven_class_untrained_init.pt')
    assert sha(init['initialization_path'])==init['initialization_sha256']
    if not args.resume_state:
        RUN.mkdir(parents=True,exist_ok=False)
    else:
        assert Path(args.resume_state).resolve().parent==RUN.resolve()
        assert not (RUN/'COMPLETED.json').exists()
    protocol=read(ROOT/'train_config.json')
    protocol.update(precision='BF16 forward; all loss/TAL/box/DFL/KD FP32; model/master/optimizer FP32; no gradient scaling',
       normalization='all windows denominator32, including partial; sum(bs*micro_mean)/32; no samples dropped',
       validation_precision='FP32 both baseline and candidate, same NMS/coverage/matching as v1',
       revision='numerical_revision_20261020', previous_run='formal_joint_v1_verified:104 updates, not resumed',
       augmentation_rng='same HSV/flip distributions; deterministic keyed by seed/epoch/record_index',
       recovery='initial + first update + every100 successful updates + epoch boundaries; no automatic retry',
       source_files={p.name:sha(p) for p in (ROOT/'src').glob('*.py')})
    if not args.resume_state:
        write(RUN/'protocol.json',protocol)
        write(RUN/'ATTEMPT.json',{'started':datetime.now().isoformat(),'source_sha256':INIT_SHA,
          'fresh_initialization':True,'previous_run_updates_discarded':104,'automatic_retry':False})
    records=read(ASSET_ROOT/'configs/manifest.json')['records']
    train=[r for r in records if r['partition']=='train']
    warmrows=[r for r in train if r['coverage']==[10]]
    validation=[r for r in records if r['partition'] in ['val_old','val_ceymo','val_road']]
    assert len(train)==27864 and len(warmrows)==3333
    val_loader=loader(validation,batch=16,workers=0)
    model=YOLO(init['initialization_path']).model.cuda().float()
    criterion=PartialDetectionLoss(model)
    teacher=YOLO(str(INIT)).model.cuda().float().eval()
    for p in teacher.parameters(): p.requires_grad_(False)
    baseline_path=RUN/'original10_val_metrics.json'
    if not args.resume_state:
        print('FP32_BASELINE_VALIDATION_START',flush=True)
        baseline=evaluate(teacher,val_loader,prediction_path=RUN/'original10_val_predictions.jsonl')
        write(baseline_path,baseline)
        initial=evaluate(model,val_loader,projection='old10')
        write(RUN/'eleven_init_old10_projection_val_metrics.json',initial)
        assert abs(old_score(initial)-old_score(baseline))<1e-7
        print('FP32_BASELINE_VALIDATION_DONE',old_score(baseline),flush=True)
    else: baseline=read(baseline_path)
    ema=deepcopy(model).eval()
    for p in ema.parameters(): p.requires_grad_(False)
    mode={'warm':True}
    for name,p in model.named_parameters():
        if name in FINAL_KEYS:
            p.requires_grad_(True)
            def hook(grad):
                if mode['warm']:
                    grad=grad.clone(); grad[:10]=0
                return grad
            p.register_hook(hook)
    optimizer=build_optimizer(model)
    scaler=torch.amp.GradScaler('cuda',enabled=False)
    optimizer.zero_grad(set_to_none=True)
    counters={'actual_updates':0}
    optimizer.register_step_post_hook(lambda *unused:counters.__setitem__('actual_updates',counters['actual_updates']+1))
    best=-float('inf');best_any=-float('inf');best_epoch=None;eligible=False
    history=[];last_improvement=0;maxepochs=40;epoch=1;start_microbatch=0
    total_presentations=0;accumulated_elapsed=0.;partial_epoch=None
    if args.resume_state:
        saved=torch.load(args.resume_state,weights_only=False,map_location='cuda')
        assert saved['valid'] and saved['source_sha256']==INIT_SHA
        assert saved['manifest_sha256']==sha(ASSET_ROOT/'configs/manifest.json')
        model.load_state_dict(saved['model']);ema.load_state_dict(saved['ema'])
        optimizer.load_state_dict(saved['optimizer']);scaler.load_state_dict(saved['scaler'])
        pos=saved['position'];sel=saved['selection']
        epoch=pos['epoch'];start_microbatch=pos['next_microbatch']
        counters['actual_updates']=pos['actual_updates'];total_presentations=pos['presentations']
        accumulated_elapsed=pos['elapsed_s'];partial_epoch=pos.get('partial_epoch')
        best=sel['best'];best_any=sel['best_any'];best_epoch=sel['best_epoch'];eligible=sel['eligible']
        history=sel['history'];last_improvement=sel['last_improvement'];maxepochs=sel['maxepochs']
        restore_rng({k:(v.cpu() if k=='torch' else [x.cpu() for x in v] if k=='cuda' else v) for k,v in saved['rng'].items()})
        del saved
    start=time.perf_counter();torch.cuda.reset_peak_memory_stats()
    # Exact protected state is always the initialization, including on explicit recovery.
    original_model=YOLO(init['initialization_path']).model.float()
    warm_before=protection_state(original_model,True);del original_model
    elapsed=lambda:accumulated_elapsed+time.perf_counter()-start
    def selection():
        return dict(best=best,best_any=best_any,best_epoch=best_epoch,eligible=eligible,
          history=history,last_improvement=last_improvement,maxepochs=maxepochs)
    def position(ep,micro,partial=None):
        return dict(epoch=ep,next_microbatch=micro,actual_updates=counters['actual_updates'],
          presentations=total_presentations,elapsed_s=elapsed(),precision=PRECISION,partial_epoch=partial)
    if not args.resume_state:
        save_valid_state(RUN/'latest_valid_state.pt',model,ema,optimizer,scaler,position(1,0),selection())
    update_stream=(RUN/'optimizer_updates.jsonl').open('a',encoding='utf-8',buffering=1)
    micro_stream=(RUN/'microbatches.jsonl').open('a',encoding='utf-8',buffering=1)
    epoch_stream=(RUN/'epochs.jsonl').open('a',encoding='utf-8',buffering=1)
    stage='ready';cpu=None;diagnostic={};window_images=0
    try:
        while epoch<=maxepochs:
            begin=time.perf_counter();warm=configure(model,epoch);mode['warm']=warm
            rows=warmrows if warm else train
            current,order=epoch_loader(rows,epoch,start_microbatch=start_microbatch,workers=2)
            write(RUN/'orders'/f'epoch_{epoch:03d}.json',{'record_indices':order,'pool':'crosswalk_sources' if warm else 'full_train'})
            loss_sum=np.zeros(3);kd_sum=0.;presentations=0;batch_count=0;first_ids=[];steps_before=counters['actual_updates']
            if partial_epoch:
                loss_sum=np.array(partial_epoch['loss_sum']);kd_sum=partial_epoch['kd_sum'];presentations=partial_epoch['presentations']
                batch_count=partial_epoch['batch_count'];first_ids=partial_epoch['first_ids'];steps_before=partial_epoch['steps_before']
            def partial():
                return dict(loss_sum=loss_sum.tolist(),kd_sum=kd_sum,presentations=presentations,
                    batch_count=batch_count,first_ids=first_ids,steps_before=steps_before)
            for bi,cpu in enumerate(current,start=start_microbatch):
                batch=cuda_batch(cpu);bs=len(batch['img']);window_images+=bs
                diagnostic={'epoch':epoch,'microbatch':bi+1,'actual_bs':bs,'window_images_so_far':window_images,
                   'normalizer':32,'scale':scaler.get_scale(),'ids':batch['id']}
                if len(first_ids)<32:first_ids.extend(batch['id'][:32-len(first_ids)])
                lrvalues=set_lr(optimizer,epoch,counters['actual_updates'])
                stage='forward_bf16'
                with context(PRECISION):raw=model(batch['img'])
                finite_tensor('raw_boxes',raw['boxes']);finite_tensor('raw_scores',raw['scores'])
                stage='loss_fp32';terms,parts=criterion(raw,batch)
                diagnostic['loss_parts']={k:float(v) for k,v in parts.items()}
                for k,v in parts.items():finite_tensor('loss:'+k,v)
                supervised=terms.sum()/32
                if warm:kd=raw['scores'][...,:0].float().sum();kdinfo={'selected_cells':0,'cls':0.,'dfl':0.}
                else:
                    indices=torch.where((~batch['coverage'][:,:10]).any(1))[0]
                    stage='teacher_forward_bf16'
                    if len(indices):
                        with torch.no_grad(),context(PRECISION):teacher_raw=teacher(batch['img'][indices])[1]
                        finite_tensor('teacher_boxes',teacher_raw['boxes']);finite_tensor('teacher_scores',teacher_raw['scores'])
                    else:teacher_raw=None
                    stage='distillation_fp32'
                    kd,kdinfo=distillation(raw,teacher_raw,indices,batch['coverage'],criterion.last_fg)
                    del teacher_raw
                total=supervised+kd*(bs/32);finite_tensor('total_loss',total)
                diagnostic.update(total_loss=float(total.detach()),teacher=kdinfo)
                stage='backward';scaler.scale(total).backward()
                stage='accumulated_gradient_check';check_gradients(model,'accumulated_gradient')
                if torch.cuda.max_memory_allocated()>MEMORY_LIMIT:raise RuntimeError('Peak allocation exceeds frozen6.5GiB limit')
                values=np.array([float(parts[k]) for k in ['box_loss','cls_loss','dfl_loss']])
                loss_sum+=values*bs;kd_sum+=float(kd.detach())*bs
                presentations+=bs;total_presentations+=bs;batch_count+=1
                micro_stream.write(json.dumps({**diagnostic,'all_finite':True,'elapsed_s':elapsed()})+'\n')
                if (bi+1)%2==0 or bi+1==math.ceil(len(rows)/16):
                    stage='unscale';scaler.unscale_(optimizer);check_gradients(model,'unscaled_gradient')
                    stage='clip';norm=float(torch.nn.utils.clip_grad_norm_(model.parameters(),10.,error_if_nonfinite=True))
                    diagnostic['gradient_norm_before_clip']=norm
                    prior={n:p[:10].detach().clone() for n,p in model.named_parameters() if n in FINAL_KEYS}
                    stage='optimizer_step';before=counters['actual_updates'];scaler.step(optimizer);scaler.update()
                    assert counters['actual_updates']==before+1
                    with torch.no_grad():
                        for name,p in model.named_parameters():
                            if name in FINAL_KEYS:
                                if warm:
                                    p[:10].copy_(prior[name])
                                    for key in ['exp_avg','exp_avg_sq']:
                                        if key in optimizer.state[p]:optimizer.state[p][key][:10]=0
                                else:p[:10].copy_(prior[name]+.2*(p[:10]-prior[name]))
                    stage='parameter_optimizer_finite_check';check_parameters(model,optimizer)
                    optimizer.zero_grad(set_to_none=True);update_ema(ema,model,counters['actual_updates'],warm)
                    stage='ema_finite_check';check_parameters(ema)
                    update_stream.write(json.dumps({'update':counters['actual_updates'],'epoch':epoch,'microbatch':bi+1,
                       'images_in_window':window_images,'normalizer':32,'loss_terms_current':values.tolist(),
                       'teacher':kdinfo,'gradient_norm_before_clip':norm,'scaler_scale':scaler.get_scale(),
                       'lr_by_role':lrvalues,'peak_allocated_bytes':torch.cuda.max_memory_allocated(),'elapsed_s':elapsed()})+'\n')
                    window_images=0
                    if counters['actual_updates']==1:
                        write(RUN/'first_optimizer_step.json',{**check_warm_protection(model,warm_before),
                           'actual_update':1,'gradient_norm':norm,'all_finite':True,'precision':PRECISION})
                    if counters['actual_updates']==1 or counters['actual_updates']%100==0:
                        stage='save_valid_boundary'
                        save_valid_state(RUN/'latest_valid_state.pt',model,ema,optimizer,scaler,position(epoch,bi+1,partial()),selection())
                    if (bi+1)%100==0 or bi+1==math.ceil(len(rows)/16):
                        live={'phase':'training','run':RUN.name,'epoch':epoch,'max_epochs_current':maxepochs,
                          'batch':bi+1,'batches':math.ceil(len(rows)/16),'actual_updates':counters['actual_updates'],
                          'presentations':total_presentations,'loss_mean':(loss_sum/presentations).tolist(),
                          'elapsed_s':elapsed(),'epoch_elapsed_s':time.perf_counter()-begin,
                          'peak_allocated_bytes':torch.cuda.max_memory_allocated()}
                        write(ROOT/'training_live.json',live);print('PROGRESS',json.dumps(live),flush=True)
                del raw,terms,total,supervised,kd,batch
            if warm:
                write(RUN/f'warmup_epoch{epoch}_protection.json',check_warm_protection(model,warm_before))
                write(RUN/f'warmup_epoch{epoch}_EMA_protection.json',check_warm_protection(ema,warm_before))
            stage='validation_fp32'
            write(ROOT/'training_live.json',{'phase':'validation','run':RUN.name,'epoch':epoch,'actual_updates':counters['actual_updates']})
            val=evaluate(ema,val_loader,PartialDetectionLoss(ema));keep=retention(val,baseline)
            fitness=.5*old_score(val)+.5*float(cw_score(val))
            rec={'epoch':epoch,'stage':'new_row_only' if warm else 'joint_neck_head' if epoch<=10 else 'joint_upper_backbone',
              'unique_pool_images_this_epoch':len(rows),'actual_image_presentations':presentations,'omitted_images':0,
              'microbatches':batch_count,'optimizer_updates_this_epoch':counters['actual_updates']-steps_before,
              'actual_optimizer_updates_total':counters['actual_updates'],'train_loss_mean':(loss_sum/presentations).tolist(),
              'teacher_loss_mean':kd_sum/presentations,'validation':val,'retention':keep,'fitness':fitness,
              'cw_macro_AP95':float(cw_score(val)),'lr_by_role':lrvalues,'epoch_seconds':time.perf_counter()-begin,
              'elapsed_s':elapsed(),'peak_allocated_bytes':torch.cuda.max_memory_allocated(),
              'total_presentations':total_presentations,'first32_draw_ids':first_ids}
            stage='save_epoch_models'
            if fitness>best_any:
                best_any=fitness;checkpoint(RUN/'best_any.pt',ema,epoch,rec,counters['actual_updates'])
            if keep['pass'] and fitness>best+1e-4:
                best=fitness;best_epoch=epoch;last_improvement=epoch;eligible=True
                checkpoint(RUN/'best.pt',ema,epoch,rec,counters['actual_updates'])
            checkpoint(RUN/'last.pt',ema,epoch,rec,counters['actual_updates'])
            history.append(rec);write(RUN/'epoch_metrics'/f'epoch_{epoch:03d}.json',rec);epoch_stream.write(json.dumps(rec)+'\n')
            stop=epoch>=30 and epoch-last_improvement>=10
            if epoch==40 and not stop:
                previous=max(r['cw_macro_AP95'] for r in history if 31<=r['epoch']<=35)
                recent=max(r['cw_macro_AP95'] for r in history if 36<=r['epoch']<=40)
                decision={'previous_best_CW95':previous,'recent_best_CW95':recent,'improvement':recent-previous,
                    'old_retention_pass':keep['pass'],'extend_to50':recent-previous>=.005 and keep['pass']}
                write(RUN/'extension_at40.json',decision)
                if decision['extend_to50']:maxepochs=50
            stage='save_epoch_valid_state'
            save_valid_state(RUN/'last_epoch_state.pt',model,ema,optimizer,scaler,position(epoch+1,0),selection())
            shutil.copyfile(RUN/'last_epoch_state.pt',RUN/'latest_valid_state.pt')
            print('EPOCH_DONE',epoch,'CW95',round(rec['cw_macro_AP95'],5),'OLD95',round(old_score(val),5),
                'retention',keep['pass'],'seconds',round(rec['epoch_seconds'],1),flush=True)
            write(ROOT/'training_live.json',{'phase':'epoch_complete','run':RUN.name,'epoch':epoch,
                'actual_updates':counters['actual_updates'],'best_epoch':best_epoch,'elapsed_s':elapsed(),'retention':keep})
            if stop:print('PREDEFINED_EARLY_STOP',epoch,flush=True);break
            epoch+=1;start_microbatch=0;partial_epoch=None
        if not eligible:shutil.copyfile(RUN/'best_any.pt',RUN/'best.pt')
        summary={'status':'TRAINING_COMPLETED','epochs_completed':len(history),'actual_optimizer_updates':counters['actual_updates'],
          'total_image_presentations':total_presentations,'train_unique_pool':len(train),'best_eligible_epoch':best_epoch,
          'best_has_retention_eligibility':eligible,'best_fitness':best if eligible else best_any,
          'wall_seconds_training_and_validation':elapsed(),'peak_allocated_bytes':torch.cuda.max_memory_allocated(),
          'best_path':str(RUN/'best.pt'),'best_sha256':sha(RUN/'best.pt'),'last_path':str(RUN/'last.pt'),
          'last_sha256':sha(RUN/'last.pt'),'precision':PRECISION,'test_used_for_selection':False,
          'no_automatic_retry':True,'final_test_status':'PENDING'}
        write(RUN/'COMPLETED.json',summary);write(ROOT/'training_live.json',summary)
        print('TRAINING_COMPLETED',json.dumps(summary),flush=True)
    except BaseException as error:
        failure={'status':'ABORTED','exception':repr(error),'stage':stage,'diagnostic':diagnostic,
          'epoch':epoch,'actual_updates':counters['actual_updates'],'total_presentations':total_presentations,
          'elapsed_s':elapsed(),'traceback':traceback.format_exc(),'no_automatic_retry':True,
          'latest_valid_state':str(RUN/'latest_valid_state.pt'),'failed_parameters_not_saved_as_valid_model':True}
        if cpu is not None:torch.save(cpu,RUN/'FAILURE_INPUT_BATCH.pt')
        write(RUN/'ABORTED.json',failure);write(ROOT/'training_live.json',failure)
        print('ABORTED',json.dumps(failure),flush=True);raise
    finally:
        update_stream.close();micro_stream.close();epoch_stream.close()

if __name__=='__main__':main()
