"""Single-camera RK decisions with an isolated, stationary CARLA vehicle."""
from __future__ import annotations

from datetime import datetime
import json
import os
from pathlib import Path
import queue
import threading
import time
import traceback

from PyQt5 import QtCore, QtGui, QtWidgets

try:
    from .rk_decision_transport import DecisionTransport, MjpegInput, display_feedback
except ImportError:
    from rk_decision_transport import DecisionTransport, MjpegInput, display_feedback


class VehicleWorker:
    def __init__(self, output, host='127.0.0.1',port=2000):
        self.output,self.host,self.port=output,host,port
        self.lock=threading.Lock()
        self.stop_event=threading.Event()
        self.desired={'turn':'off','brake_lamp':True,'wheel':'center','body_locked':True}
        self.self_test_at=None
        self.status={}
        self.frames={}
        self.error=None
        self.connecting=True
        self.thread=threading.Thread(target=self._run,daemon=True)

    def start(self): self.thread.start()

    def set_feedback(self, value):
        with self.lock: self.desired=dict(value)

    def start_self_test(self):
        with self.lock: self.self_test_at=time.monotonic()

    def state(self):
        with self.lock: return dict(self.status),dict(self.frames),self.error,self.connecting

    def _run(self):
        session=None
        try:
            try:
                from .static_vehicle_session import DemoSession
            except ImportError:
                from static_vehicle_session import DemoSession
            session=DemoSession(self.host,self.port,self.output)
            session.start()
            applied=None
            last_status=0.0
            saved_phases=set()
            test_identity=None
            while not self.stop_event.is_set():
                with self.lock:
                    wanted=dict(self.desired)
                    test_at=self.self_test_at
                phase=None
                if wanted.get('aeb_emergency') or wanted.get('reaction') in ('warning','slow','emergency','alarm'):
                    with self.lock:self.self_test_at=None
                    test_at=None
                if test_at is not None:
                    elapsed=time.monotonic()-test_at
                    phase='left' if elapsed<2 else 'right' if elapsed<4 else 'hazard' if elapsed<6 else None
                    if phase:
                        wanted.update(turn=phase,brake_lamp=phase=='hazard',wheel=phase if phase!='hazard' else 'center')
                    else:
                        with self.lock: self.self_test_at=None
                    if test_identity!=test_at:
                        test_identity=test_at;saved_phases.clear()
                pair=(wanted['turn'],wanted['brake_lamp'])
                if pair!=applied:
                    session.set_feedback(*pair)
                    applied=pair
                session.apply_feedback()
                with session.lock: frames=dict(session.frames)
                state=None
                if time.monotonic()-last_status>.4:
                    state=session.status()
                    state["self_test_phase"]=phase
                    state['reaction']='self_test' if phase else wanted.get('reaction','normal')
                    last_status=time.monotonic()
                with self.lock:
                    self.frames=frames
                    if state is not None: self.status=state
                    self.connecting=False
                if phase and phase not in saved_phases:
                    if (time.monotonic()-test_at)%2>.65:
                        session.save_evidence('SELF_TEST_'+phase)
                        saved_phases.add(phase)
                self.stop_event.wait(.05)
        except Exception as exc:
            with self.lock:
                self.error=str(exc)
                self.connecting=False
            self.output.mkdir(parents=True,exist_ok=True)
            (self.output/'error.txt').write_text(traceback.format_exc(),encoding='utf-8')
        finally:
            if session is not None:
                try:
                    session.set_feedback('off',True)
                    session.close()
                except Exception:
                    pass

    def close(self):
        self.set_feedback({'turn':'off','brake_lamp':True,'wheel':'center','body_locked':True})
        self.stop_event.set()


class DecisionPanel(QtWidgets.QWidget):
    status_changed=QtCore.pyqtSignal(str)

    def __init__(self,parent=None):
        super().__init__(parent)
        self.active=False
        self.transport=None
        self.video=None
        self.vehicle=None
        self.output=None
        self.last_snapshot=None
        self.last_feedback=None
        self.seen={}
        self.last_save=0.0
        self.last_ack=None
        self.last_ack_at=0.0
        self.rk_url=os.environ.get('ADAS_RK_DECISION_URL','http://192.168.2.3:18088')
        self.stream_url=os.environ.get('ADAS_RK_DECISION_STREAM','http://192.168.2.3:18089/stream')
        self.setStyleSheet('''
            QWidget {font-size:15px;}
            QLabel#decisionTitle {font-size:21px;font-weight:700;color:#eaf4ff;}
            QLabel#decisionBanner {font-size:20px;font-weight:700;padding:9px;border-radius:6px;}
            QPushButton {padding:7px;font-size:16px;min-height:24px;background:#20344b;color:#eff7ff;border:1px solid #7393b5;border-radius:4px;}
            QPushButton:hover {background:#345779;border-color:#9ad2ff;}
            QPushButton:pressed {background:#125c83;}
            QPushButton:disabled {background:#172333;color:#6d8399;border-color:#35485f;}
            QComboBox {padding:6px;min-width:90px;font-size:15px;}
            QLabel#evidencePill {font-size:17px;padding:8px;background:#182b43;border:1px solid #6b8aa8;border-radius:5px;}
        ''')
        outer=QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(0,0,0,0)
        outer.setSpacing(7)
        heading=QtWidgets.QHBoxLayout()
        title=QtWidgets.QLabel('单镜头感知 · 静止车辆决策演示')
        title.setObjectName('decisionTitle')
        heading.addWidget(title)
        heading.addStretch()
        self.connection=QtWidgets.QLabel('正在连接RK…')
        heading.addWidget(self.connection)
        self.reconnect=QtWidgets.QPushButton('连接 / 重连车辆')
        self.reconnect.clicked.connect(self.connect_vehicle)
        heading.addWidget(self.reconnect)
        outer.addLayout(heading)
        hint=QtWidgets.QLabel('摄像头拍摄赛事视频，RK判断规则；车辆车身锁定，行驶按钮仅展示通行意图、灯光和车轮。')
        hint.setStyleSheet('color:#b8c9dc;')
        outer.addWidget(hint)
        controls=QtWidgets.QHBoxLayout()
        self.scenario=QtWidgets.QComboBox()
        self.scenario.addItem('路口信号灯','intersection')
        self.scenario.addItem('实虚线变道','lane_change')
        self.lane=QtWidgets.QComboBox()
        for label,value in [('左转车道','left'),('直行车道','straight'),('右转车道','right')]:
            self.lane.addItem(label,value)
        self.lane.setCurrentIndex(1)
        controls.addWidget(self.scenario)
        controls.addWidget(self.lane)
        self.buttons={}
        for action,label in [('set_context','应用场景 / 车道'),('move','行驶请求'),
                             ('left','← 左变道请求'),('right','右变道请求 →'),
                             ('stop','停止')]:
            b=QtWidgets.QPushButton(label)
            b.clicked.connect(lambda checked=False,a=action:self.command(a))
            controls.addWidget(b)
            self.buttons[action]=b
        self.auto_alarm=QtWidgets.QLabel('报警：规则自动触发 / 自动结束')
        controls.addWidget(self.auto_alarm)
        outer.addLayout(controls)
        self.self_test=QtWidgets.QPushButton('车辆灯光 / 前轮自检（6秒，非RK决策）')
        self.self_test.clicked.connect(self.start_vehicle_self_test)
        outer.addWidget(self.self_test)
        fields=QtWidgets.QHBoxLayout()
        self.fields={}
        for key,label in [('signal','灯色'),('left','左边界'),('right','右边界')]:
            field=QtWidgets.QLabel(label+'：等待RK')
            field.setObjectName('evidencePill')
            fields.addWidget(field,1)
            self.fields[key]=field
        self.context_label=QtWidgets.QLabel('RK当前场景：等待连接')
        fields.addWidget(self.context_label)
        outer.addLayout(fields)
        self.banner=QtWidgets.QLabel('■ 保持停止 · 等待RK结果')
        self.banner.setObjectName('decisionBanner')
        outer.addWidget(self.banner)
        split=QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.video_label=self._image_label('等待RK处理画面…')
        left=QtWidgets.QWidget()
        l=QtWidgets.QVBoxLayout(left);l.setContentsMargins(0,0,0,0)
        l.addWidget(QtWidgets.QLabel('单镜头 · RK模型处理结果'))
        l.addWidget(self.video_label,1)
        split.addWidget(left)
        right=QtWidgets.QWidget()
        r=QtWidgets.QVBoxLayout(right);r.setContentsMargins(0,0,0,0)
        self.views={}
        for name,caption in [('rear','CARLA后侧 · 转向灯 / 双闪 / 制动灯'),
                             ('front','CARLA前侧 · 前轮方向')]:
            r.addWidget(QtWidgets.QLabel(caption))
            view=self._image_label('正在等待静止车辆…')
            self.views[name]=view
            r.addWidget(view,1)
        split.addWidget(right)
        split.setSizes([650,650])
        split.setStretchFactor(0,1);split.setStretchFactor(1,1)
        outer.addWidget(split,1)
        self.ack_label=QtWidgets.QLabel('请求结果：尚未点击按钮')
        self.ack_label.setWordWrap(True)
        outer.addWidget(self.ack_label)
        self.vehicle_label=QtWidgets.QLabel('车辆：等待连接')
        self.vehicle_label.setWordWrap(True)
        outer.addWidget(self.vehicle_label)
        self.details=QtWidgets.QLabel()
        self.details.setStyleSheet('color:#9eafc5;font-size:11px;')
        self.details.setWordWrap(True)
        outer.addWidget(self.details)
        self.timer=QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)

    def _image_label(self,text):
        label=QtWidgets.QLabel(text)
        label.setAlignment(QtCore.Qt.AlignCenter)
        label.setMinimumSize(260,130)
        label.setSizePolicy(QtWidgets.QSizePolicy.Ignored,QtWidgets.QSizePolicy.Ignored)
        label.setStyleSheet('background:#05090e;border:1px solid #6b8aa8;')
        return label

    def activate(self):
        if self.active: return
        self.active=True
        self.output=Path(os.environ.get('ADAS_DECISION_RESULTS',str(Path(__file__).resolve().parent.parent/'decision_results')))/datetime.now().strftime('%Y%m%d_%H%M%S')
        self.output.mkdir(parents=True,exist_ok=True)
        self.last_snapshot=None;self.last_feedback=None;self.last_ack=None;self.seen={}
        self.transport=DecisionTransport(self.rk_url,os.environ.get('RK_DECISION_TOKEN'))
        self.video=MjpegInput(self.stream_url)
        self.transport.start();self.video.start()
        self.connect_vehicle()
        self.timer.start(50)
        self.refresh()

    def connect_vehicle(self):
        if not self.active: return
        if self.vehicle is not None and self.vehicle.thread.is_alive():
            self.vehicle_label.setText('车辆正在连接或已连接；如需重建，请先退出决策模式。')
            return
        self.vehicle=VehicleWorker(self.output/'vehicle')
        self.vehicle.start()

    def start_vehicle_self_test(self):
        if self.vehicle is None or self.vehicle.connecting or self.vehicle.error:
            self.ack_label.setText('请先连接静止车辆，再运行自检。')
            return
        self.vehicle.start_self_test()
        self._event({'type':'vehicle_self_test','scope':'manual actuator visual test; NOT RK perception/decision'})
        self.ack_label.setText('车辆自检：左灯+左前轮2秒 → 右灯+右前轮2秒 → 双闪+制动2秒；车身保持原地。')

    def command(self,action):
        if self.transport is None or self.last_snapshot is None: return
        try:
            command=self.transport.request(action,self.last_snapshot['source_epoch'],
                       self.scenario.currentData(),self.lane.currentData())
            self.ack_label.setText('请求已提交RK，等待判断…')
            self._event({'type':'sent','command':command})
        except Exception as exc:
            self.ack_label.setText('请求未提交：'+str(exc))

    def _event(self,data):
        if self.output:
            with (self.output/'events.jsonl').open('a',encoding='utf-8') as f:
                f.write(json.dumps({'time':datetime.now().astimezone().isoformat(),**data},ensure_ascii=False)+'\n')

    def _put_image(self,label,image):
        label.setPixmap(QtGui.QPixmap.fromImage(image).scaled(label.size(),QtCore.Qt.KeepAspectRatio,QtCore.Qt.SmoothTransformation))

    def refresh(self):
        if not self.active or self.transport is None: return
        now=time.monotonic()
        snapshot,received,error,pending=self.transport.state()
        age=now-received if received else 1e9
        self.last_snapshot=snapshot
        feedback,blocked=display_feedback(snapshot,age)
        online=error is None and age<=.8
        self.connection.setText('RK决策在线' if online else 'RK连接中断 / 等待连接')
        self.connection.setStyleSheet('color:#5be8b2;' if online else 'color:#ffc36b;')
        for button in self.buttons.values(): button.setEnabled(online and not pending)
        while True:
            try: event=self.transport.events.get_nowait()
            except queue.Empty: break
            self._event({'type':'reply',**event})
            self.last_ack=event
            self.last_ack_at=now
            if 'ack' in event:
                ack=event['ack']
                self.ack_label.setText(('允许：' if ack['accepted'] else '拒绝：')+ack['message']+'  ['+ack['reason_code']+']')
                self.ack_label.setStyleSheet('color:#5be8b2;' if ack['accepted'] else 'color:#ffbd70;')
            else:
                self.ack_label.setText('请求通信失败：'+event.get('error','未知错误'))
        perception=(snapshot or {}).get('perception',{})
        signal=perception.get('signal',{})
        boundaries=perception.get('boundaries',{})
        colors={'red':'红灯','green':'绿灯','yellow':'黄灯','unknown':'未知'}
        kinds={'solid':'实线','dashed':'虚线','mixed':'混合 / 待确认','unknown':'未知'}
        lamp=colors.get(signal.get('color'),'未知') if online and signal.get('fresh') and signal.get('valid') else '未知'
        self.fields['signal'].setText('灯色：'+lamp)
        for side,label in [('left','左边界'),('right','右边界')]:
            item=boundaries.get(side,{})
            kind=kinds.get(item.get('kind'),'未知') if online and boundaries.get('fresh') and item.get('valid') else '未知'
            self.fields[side].setText(label+'：'+kind)
        context=(snapshot or {}).get('context',{})
        self.context_label.setText('RK：'+{'intersection':'路口','lane_change':'变道'}.get(context.get('scenario'),'--')+' / '+{'left':'左转车道','straight':'直行车道','right':'右转车道'}.get(context.get('lane'),'--'))
        decision=(snapshot or {}).get('decision',{})
        aeb=decision.get('aeb',{})
        levels={'warning':'前方有行人，请注意','slow':'↓ 正在减速（模拟） · 前方有行人','emergency':'前方有行人！紧急制动'}
        sample_age=aeb.get('age_ms');ttl=(snapshot or {}).get('active_aeb',{}).get('ttl_s',.6)
        fresh_risk=bool(aeb.get('fresh') and aeb.get('valid') and type(sample_age) in (int,float) and
                        type(ttl) in (int,float) and 0<=sample_age+age*1000<=ttl*1000)
        risk=bool(online and not blocked and aeb.get('level') in levels and
                  (aeb.get('level')=='emergency' or fresh_risk))
        self.self_test.setEnabled(online and not risk and not decision.get('alarm'))
        alarm=decision.get('alarm')
        arrow={'left':'← 左转','right':'右转 →','straight':'↑ 直行','none':'■ 停止'}.get(decision.get('arrow'),'■ 停止')
        message=blocked or (alarm or {}).get('message') or decision.get('message','等待RK结果')
        banner_text=('警报：' if alarm and not blocked else '')+(('■ 保持停止' if blocked else arrow)+' · '+message)
        banner_style='background:#59252c;color:#ffb9bd;' if alarm or blocked else 'background:#153c38;color:#83efc5;'
        if not alarm and not blocked and self.last_ack and now-self.last_ack_at<3:
            ack=self.last_ack.get('ack')
            if ack and not ack['accepted']:
                banner_text='■ 请求被拒绝 · '+ack['message']+' ['+ack['reason_code']+']'
                banner_style='background:#503b19;color:#ffcf83;'
        # All risk feedback uses the existing single banner. No extra row is
        # inserted/removed when pedestrians appear, so the video layout stays fixed.
        if risk and not alarm:
            marker={'warning':'前方有行人','slow':'正在减速','emergency':'紧急制动'}[aeb['level']]
            if marker not in banner_text:
                banner_text+=' | '+levels[aeb['level']]
        self.auto_alarm.setText('报警：自动触发中' if alarm else '报警：规则自动触发 / 自动结束')
        if self.vehicle is not None: self.vehicle.set_feedback(feedback)
        image,seq,video_at,video_error=self.video.state()
        if image is not None and now-video_at<2 and self.seen.get('rk')!=seq:
            self.seen['rk']=seq;self._put_image(self.video_label,image)
        elif not video_at or now-video_at>=2:
            self.video_label.setText('RK视频等待 / 中断\n'+(video_error or ''))
        vehicle_status={}
        if self.vehicle is not None:
            vehicle_status,frames,vehicle_error,connecting=self.vehicle.state()
            for name,(rgb,frame,at) in frames.items():
                if now-at>1.5:
                    self.views[name].setText('CARLA画面中断')
                elif self.seen.get(name)!=frame:
                    self.seen[name]=frame
                    im=QtGui.QImage(rgb.data,rgb.shape[1],rgb.shape[0],rgb.strides[0],QtGui.QImage.Format_RGB888).copy()
                    self._put_image(self.views[name],im)
            if vehicle_error:
                self.vehicle_label.setText('CARLA车辆未连接：'+vehicle_error)
            elif connecting:
                self.vehicle_label.setText('正在创建独立静止车辆与实时相机…')
            else:
                self.vehicle_label.setText('车身锁定 | 实际车速 %.3f km/h | 最大位移 %.4f m | 转向 %s | 尾灯常亮 | 刹车灯 %s | 世界帧 %s' % (
                    vehicle_status.get('speed_kmh',0),vehicle_status.get('max_drift_m',0),
                    {'off':'回正','left':'左','right':'右','hazard':'双闪'}.get(vehicle_status.get('turn'),'--'),
                    '亮' if vehicle_status.get('brake_lamp') else '灭',vehicle_status.get('world_frame','--')))
        vision=(snapshot or {}).get('vision',{})
        self.details.setText('检测 %.1f Hz | 车道线 %.1f Hz | 结果接收年龄 %s ms | %s | 源帧 %s' % (
            vision.get('inf_fps',0),vision.get('lane_fps',0),round(age*1000) if received else '--',
            error or signal.get('reason',''),perception.get('frame_seq','--')))
        runtime=(snapshot or {}).get('runtime_config',{})
        if runtime:
            self.details.setText(self.details.text()+' | '+runtime.get('version','')+' | class10≥'+str(runtime.get('class10_min_conf','?'))+' | 线型长度比例判定')
        self.details.setText(self.details.text()+' | PC面板 stereo_console.v0.1.5')
        if vehicle_status.get('self_test_phase'):
            banner_text='车辆自检（非RK决策） · '+{'left':'左灯 / 前轮向左','right':'右灯 / 前轮向右','hazard':'故障双闪 / 制动灯'}.get(vehicle_status['self_test_phase'],'')
            banner_style='background:#3a285b;color:#e3ceff;'
        elif blocked or alarm or '紧急制动' in banner_text:
            banner_style='background:#59252c;color:#ffb9bd;'
        elif '正在减速' in banner_text or '减速提示' in banner_text:
            banner_style='background:#503b19;color:#ffcf83;'
        elif '前方有行人' in banner_text:
            banner_style='background:#153c38;color:#83efc5;'
        # Commit text and its semantic color once. Never flash the default green
        # in the middle of a refresh while the final message still says "slow".
        if self.banner.text()!=banner_text:self.banner.setText(banner_text)
        if self.banner.styleSheet()!=banner_style:self.banner.setStyleSheet(banner_style)
        if self.last_feedback!=feedback:
            self._event({'type':'display_feedback','feedback':feedback,'blocked':blocked})
            self.last_feedback=dict(feedback)
        if now-self.last_save>=1:
            self.last_save=now
            evidence={'active':True,'pc_panel_version':'stereo_console.v0.1.5','rk_online':online,'rk_local_age_ms':round(age*1000) if received else None,
                      'banner_text':banner_text,'banner_style':banner_style,
                      'rk':snapshot,'display_feedback':feedback,'display_block_reason':blocked,
                      'vehicle':vehicle_status,'last_request':self.last_ack,'video_frames_seen':self.seen,
                      'note':'Real RK interface evidence. No synthetic signal/line override. Vehicle remains stationary.'}
            (self.output/'status.json').write_text(json.dumps(evidence,ensure_ascii=False,indent=2),encoding='utf-8')

    def deactivate(self):
        self.active=False
        self.timer.stop()
        if self.vehicle is not None: self.vehicle.close()
        if self.transport is not None: self.transport.close()
        if self.video is not None: self.video.close()
        self._event({'type':'mode_closed'})

    def closeEvent(self,event):
        self.shutdown()
        event.accept()

    def shutdown(self):
        self.deactivate()
        # Closing the application must let the owner thread destroy its actors
        # before Python exits; ordinary mode switching remains nonblocking.
        if self.vehicle is not None and self.vehicle.thread.is_alive():
            self.vehicle.thread.join(timeout=4)
        if self.transport is not None and self.transport.thread.is_alive():
            self.transport.thread.join(timeout=1)
