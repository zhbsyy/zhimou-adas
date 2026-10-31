"""Add optional AEB without replacing any of the three original modes."""
import json
import copy
import math
from pathlib import Path
import threading
import time
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from urllib.request import build_opener,ProxyHandler
from PyQt5 import QtCore,QtWidgets
from reliable_streamer import ReliableStreamer
from control_guard import ProgressWatch,SCHEMA
from aeb_session import projected_box

ROOT=Path(__file__).resolve().parent


class AnnotatedSocket:
    def __init__(self,original,session):
        self.original,self.session=original,session
        self.lock,self.seq=threading.Lock(),0
    def __getattr__(self,name):
        return getattr(self.original,name)
    def sendto(self,data,address):
        payload=json.loads(data)
        sample,received=getattr(self.session,'scene_record',(self.session.scene,self.session.scene_received))
        scene=dict(sample)
        age=time.monotonic()-received
        scene['scene_age_ms']=round(age*1000,1)
        if age>.4:
            scene['valid']=False
        a=self.session.arbiter
        payload.update(carla_aeb_scene=scene,carla_aeb_session=a.protocol.session_id,
            carla_aeb_enabled=a.aeb_enabled,carla_aeb_epoch=a.epoch)
        with self.lock:
            self.seq+=1
            payload['carla_aeb_state_seq']=self.seq
        return self.original.sendto(json.dumps(payload,ensure_ascii=False).encode(),address)


class NavigationSession(QtCore.QObject):
    message=QtCore.pyqtSignal(str)
    finished=QtCore.pyqtSignal()
    def __init__(self,window):
        super().__init__(window)
        self.window,self.arbiter=window,window.arbiter
        self.streamer=None
        self.rk,self.rk_received={},0.
        self.scene,self.scene_received={'valid':False},0.
        self.scene_record=(self.scene,self.scene_received)
        self.scene_revision=None
        self.scene_error=''
        self.scene_max_sample_ms=0.
        self.last_actor,self.last_geometry=None,None
        self.progress,self.stopping=ProgressWatch(),threading.Event()
        self.job,self.last_error=None,''
        self.last_event='保留原三种模式；AEB默认关闭，座舱导航不依赖RK'
        self.arbiter.session=self
        self.arbiter._udp_client_sock=AnnotatedSocket(self.arbiter._udp_client_sock,self)
        for label in window.findChildren(QtWidgets.QLabel):
            if label.text().startswith('规格:'):
                label.setText('MJPEG 1280×720；四路各640×360；实际帧率见推流状态')
        self.records=ROOT/'Results'
        self.records.mkdir(exist_ok=True)
        dock=QtWidgets.QDockWidget('导航与可选AEB（距离来源：CARLA仿真真值）',window)
        dock.setFeatures(QtWidgets.QDockWidget.NoDockWidgetFeatures)
        pane=QtWidgets.QWidget()
        layout=QtWidgets.QHBoxLayout(pane)
        self.aeb_box=QtWidgets.QCheckBox('启用AEB：行人通过后恢复导航')
        self.aeb_box.toggled.connect(self.arbiter.set_aeb_enabled)
        layout.addWidget(self.aeb_box)
        self.buttons=[]
        for title,slot in [('连接CARLA与四相机',self.prepare),('生成前方行人',self.spawn_request),('刷新座舱串口',self.refresh_ports)]:
            button=QtWidgets.QPushButton(title)
            button.clicked.connect(slot)
            button.setStyleSheet('QPushButton {background:#234665;color:white;border:1px solid #6386a3;padding:7px;}')
            self.buttons.append(button)
            layout.addWidget(button)
        self.label=QtWidgets.QLabel(self.last_event)
        self.label.setWordWrap(True)
        self.message.connect(self.label.setText)
        layout.addWidget(self.label,1)
        dock.setWidget(pane)
        dock.setStyleSheet('QWidget {background:#111f31;color:#e5efff;}')
        window.addDockWidget(QtCore.Qt.TopDockWidgetArea,dock)
        window.btn_carla.clicked.disconnect()
        window.btn_carla.clicked.connect(self.toggle_carla)
        self.finished.connect(lambda:[button.setEnabled(True) for button in self.buttons])
        self.last_logged=0.
        self.timer=QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(100)
        self.poller=threading.Thread(target=self.poll_rk,daemon=True)
        self.scene_poller=threading.Thread(target=self.poll_scene,daemon=True)
        self.poller.start()
        self.scene_poller.start()
        self.server=ThreadingHTTPServer(('127.0.0.1',18090),self.handler())
        self.http=threading.Thread(target=lambda:self.server.serve_forever(poll_interval=.1),daemon=True)
        self.http.start()

    def notice(self,text):
        self.last_event=text
        self.arbiter.log('【可选AEB】'+text)
        if not self.stopping.is_set():
            self.message.emit(text)

    def start_job(self,function):
        if self.job and self.job.is_alive():
            return
        for button in self.buttons[:2]:
            button.setEnabled(False)
        def run():
            try:
                self.arbiter.intent.pause()
                with self.arbiter.operation_lock:
                    self.arbiter._park_for_operation()
                    function()
            except Exception as exc:
                self.last_error=repr(exc)
                self.notice('操作未完成：'+str(exc))
            finally:
                if not self.stopping.is_set():
                    self.finished.emit()
        self.job=threading.Thread(target=run,daemon=True)
        self.job.start()

    def prepare(self):
        self.start_job(self.prepare_parked)

    def prepare_parked(self):
        if not self.arbiter.carla.connected and not self.arbiter.connect_carla():
            raise RuntimeError(self.arbiter.carla.get_status())
        if self.streamer is None or not self.streamer.health()['ready']:
            if self.streamer:
                self.streamer.stop()
            self.streamer=ReliableStreamer(http_port=8081)
            if not self.streamer.attach_to_carla():
                raise RuntimeError(self.streamer.attach_error)
            self.streamer.start_http_server()
        if self.arbiter.state.system_mode=='perception_debug' and not self.arbiter.carla.trajectory_waypoints:
            self.arbiter.set_system_mode('perception_debug')
        self.notice('四相机已连接。原三种模式可用；座舱模式等待串口导航，AEB可独立开关')

    def toggle_carla(self):
        if not self.arbiter.carla.connected:
            self.prepare()
            return
        def disconnect():
            if self.streamer:
                self.streamer.stop()
                self.streamer=None
            self.arbiter.disconnect_carla()
            self.notice('CARLA已断开，车辆保持暂停')
        self.start_job(disconnect)

    def spawn_request(self):
        def spawn():
            backend=self.arbiter.carla
            path=backend.trajectory_waypoints
            if len(path)<10:
                raise RuntimeError('先通过座舱导航或感知单机模式建立路线')
            index=min(len(path)-2,backend.last_matched_idx+max(10,int(28/backend.trajectory_step)))
            if index<=backend.last_matched_idx+4 or not backend._spawn_dynamic_pedestrian(path[index],automatic=False):
                raise RuntimeError('行人未生成，可能已接近终点')
            self.arbiter.flow.reset()
            self.arbiter._new_epoch()
            self.notice('已在原路线前方约28m生成行人；通过D挡或继续仿真开始')
        self.start_job(spawn)

    def refresh_ports(self):
        self.window._refresh_ports()
        for index in range(self.window.cb_ports.count()):
            if self.window.cb_ports.itemData(index)=='COM12':
                self.window.cb_ports.setCurrentIndex(index)
                break

    def update_scene(self):
        backend=self.arbiter.carla
        if not backend.connected or not self.streamer or not self.streamer.health()['ready']:
            return {'valid':False,'range_source':'CARLA_actor_truth'}
        revision=getattr(backend,'_scene_revision',0)
        if revision!=self.scene_revision:
            self.last_actor,self.last_geometry=None,None
            self.scene_revision=revision
        snapshot=backend.world.get_snapshot()
        common={'valid':True,'range_source':'CARLA_actor_truth','carla_frame':snapshot.frame,
                'scene_revision':revision,'front_size':[640,360]}
        walker,vehicle=backend._scene_walker,backend.vehicle
        if walker is None:
            if self.last_actor is not None:
                completed=bool(backend._scene_crossing_done)
                result=dict(common,**(self.last_geometry or {}),actor_id=self.last_actor,bbox_front=None,
                            clear_state='completed' if completed else 'unknown',pedestrian_phase='已通过' if completed else '场景异常')
                result['valid']=completed
                return result
            return dict(common,actor_id=None,bbox_front=None,forward_gap_m=0.,lateral_m=0.,clear_state='no_target',pedestrian_phase='未布置')
        vehicle_state,walker_state=snapshot.find(vehicle.id),snapshot.find(walker.id)
        if vehicle_state is None or walker_state is None:
            return dict(common,valid=False,invalid_reason='actor_not_in_snapshot')
        pose,target=vehicle_state.get_transform(),walker_state.get_transform().location
        yaw=math.radians(pose.rotation.yaw)
        dx,dy=target.x-pose.location.x,target.y-pose.location.y
        forward=dx*math.cos(yaw)+dy*math.sin(yaw)-vehicle.bounding_box.extent.x
        lateral=dx*math.sin(yaw)-dy*math.cos(yaw)
        camera=self.streamer.carla_cameras[0]
        bbox=projected_box(walker.bounding_box.get_world_vertices(walker_state.get_transform()),camera.get_transform().get_inverse_matrix())
        self.last_actor=walker.id
        self.last_geometry={'forward_gap_m':round(forward,3),'lateral_m':round(lateral,3)}
        if revision!=getattr(backend,'_scene_revision',0) or walker is not backend._scene_walker:
            return dict(common,valid=False,invalid_reason='scene_changing')
        return dict(common,**self.last_geometry,actor_id=walker.id,bbox_front=bbox,
                    pedestrian_phase='错过入场窗口（本轮未执行）' if getattr(backend,'_scene_missed',False) else
                        '横穿受阻' if getattr(backend,'_scene_blocked',False) else
                        '正在横穿' if backend._scene_crossing_started else '等待车辆驶入横穿路段',
                    clear_state='outside' if abs(lateral)>=(getattr(backend,'_scene_site',None) or {}).get('clear_lateral',2.2) or forward<-2 else 'not_clear')

    def poll_scene(self):
        while not self.stopping.is_set():
            started=time.monotonic()
            try:
                # Cached CARLA snapshots/getters do not need the actuator lock.
                # Serial I/O or route planning must never starve this producer.
                value=self.update_scene()
                now=time.monotonic()
                previous,received=self.scene_record
                if not value.get('valid') or value.get('carla_frame')!=previous.get('carla_frame') or value.get('scene_revision')!=previous.get('scene_revision'):
                    self.scene_record=(value,now)
                    self.scene,self.scene_received=value,now
                self.scene_error=''
            except Exception as exc:
                value={'valid':False,'range_source':'CARLA_actor_truth','invalid_reason':'snapshot_error'}
                self.scene_record=(value,time.monotonic())
                self.scene,self.scene_received=self.scene_record
                self.scene_error=repr(exc)
            self.scene_max_sample_ms=max(self.scene_max_sample_ms,(time.monotonic()-started)*1000)
            self.stopping.wait(.05)

    def poll_rk(self):
        opener=build_opener(ProxyHandler({}))
        while not self.stopping.is_set():
            try:
                with opener.open('http://192.168.2.3:18084/aeb_status',timeout=.6) as response:
                    value=json.loads(response.read(300000))
                aeb=value.get('carla_aeb',{})
                if (aeb.get('schema')!=SCHEMA or aeb.get('session_id')!=self.arbiter.protocol.session_id or aeb.get('epoch')!=self.arbiter.epoch):
                    continue
                if not self.arbiter.aeb_enabled or not self.arbiter.last_result['drive']:
                    self.arbiter.protocol.select_run(aeb.get('run_id'))
                self.progress.observe(value,time.monotonic())
                self.rk,self.rk_received=value,time.monotonic()
            except Exception:
                pass
            self.stopping.wait(.4)

    def perception_reason(self,now):
        return NavigationSession.perception_health(self,now)[0]

    def perception_health(self,now):
        a=self.arbiter
        if a.state.perception_source!='carla':
            return 'AEB需要CARLA感知源',False
        if not self.streamer or not self.streamer.health()['ready']:
            return '四相机或视频发布中断',True
        if now-self.scene_received>.4 or not self.scene.get('valid'):
            return '行人场景未知/已过期',True
        p=self.rk.get('carla_aeb',{})
        if (now-self.rk_received>1.5 or self.rk.get('source')!='carla' or p.get('schema')!=SCHEMA or p.get('session_id')!=a.protocol.session_id
            or p.get('epoch')!=a.epoch or p.get('run_id')!=a.protocol.run_id):
            return '等待RK感知连接或运行已变化',True
        # AEB requires fresh detection, not a particular segmentation/JPEG rate.
        # Display switching still checks all three producers independently.
        reason=self.progress.stale_reason(now,required=('det',))
        if reason:
            return reason,True
        e=a.control_report.get('evidence',{})
        if now-a.report_received>.6 or not e.get('capture_fresh') or not e.get('valid_scene'):
            return 'RK控制/图像已过期',True
        if e.get('actor_id')!=self.scene.get('actor_id'):
            return '等待RK同步当前行人',True
        return '',False

    def tick(self):
        pending=getattr(self.window,'_navigation_view_pending',None)
        self.window._navigation_view_pending=None
        if pending is not None:
            self.window._render_state(pending)
        a=self.arbiter
        text=(('AEB开启：' if a.aeb_enabled else 'AEB关闭：')+a.last_result['reason']+
            '；行人：'+self.scene.get('pedestrian_phase','待场景更新')+
            ('；'+getattr(a,'speed_plan_reason','') if a.aeb_enabled else ''))
        if self.label.text()!=text:
            self.label.setText(text)
        now=time.monotonic()
        if now-self.last_logged>=1:
            self.last_logged=now
            with (self.records/'navigation_events.jsonl').open('a',encoding='utf-8') as out:
                out.write(json.dumps(self.snapshot(),ensure_ascii=False)+'\n')

    def snapshot(self):
        a=self.arbiter
        return {'schema':'navigation_modes.v1','mode':a.state.system_mode,'aeb_enabled':a.aeb_enabled,
            'gear':a.state.gear,'speed_kmh':a.state.actual_speed_kmh,'result':dict(a.last_result),
            'requested_gear':a.intent.gear,'requested_running':a.intent.running,'serial_connected':a.serial.is_connected(),
            'arrived':getattr(a,'arrived',False),
            'automatic_resumes':getattr(a,'automatic_resumes',0),
            'serial_port':a.serial.port,'serial_reader_running':a.serial.running,
            'speed_cap_kmh':getattr(a.state,'navigation_speed_cap_kmh',None),
            'speed_plan_reason':getattr(a,'speed_plan_reason',''),
            'effective_target_kmh':getattr(a.carla,'_navigation_effective_target',None),
            'crossing_entry':getattr(a.carla,'_scene_entry',{}),
            'crossing_site':{k:(getattr(a.carla,'_scene_site',None) or {}).get(k) for k in ('route_index','route_fraction','route_s')},
            'collisions':list(getattr(a.carla,'_navigation_collisions',[])),
            'navigation_active':a.nav_active,'navigation_age_s':time.monotonic()-a.nav_received if a.nav_received else None,
            'navigation_diagnostics':getattr(a,'nav_diagnostics',{}),
            'navigation_destination':[getattr(a,'_current_dest_x',None),getattr(a,'_current_dest_y',None)],
            'carla_connected':a.carla.connected,'route':a.state.route_name,'route_remaining_m':a.state.route_remaining_distance_m,
            'camera_health':self.streamer.health() if self.streamer else None,'scene':self.scene,
            'scene_age_ms':round((time.monotonic()-self.scene_received)*1000,1),
            'scene_sample_max_ms':self.scene_max_sample_ms,'scene_error':self.scene_error,
            'rk_control_age_ms':round((time.monotonic()-a.report_received)*1000,1),
            'version':'navigation_reliability.v1.5',
            'rk_connected':time.monotonic()-self.rk_received<1.5,'rk_report':a.control_report,
            'control_error':a.control_error,'last_event':self.last_event}

    def handler(self):
        session=self
        class Handler(BaseHTTPRequestHandler):
            def log_message(self,*args):
                pass
            def do_GET(self):
                body=json.dumps(session.snapshot(),ensure_ascii=False).encode()
                self.send_response(200)
                self.send_header('Content-Type','application/json')
                self.send_header('Content-Length',str(len(body)))
                self.end_headers()
                self.wfile.write(body)
        return Handler

    def close(self):
        self.arbiter.intent.pause()
        self.timer.stop()
        self.stopping.set()
        self.server.shutdown()
        self.server.server_close()
        self.poller.join(2)
        self.scene_poller.join(2)
        if self.streamer:
            self.streamer.stop()


def publish_view_state(window,state):
    # Backend callbacks run on the control worker. Only publish an owned
    # snapshot here; Qt widgets and styles are handled by the GUI timer.
    window._navigation_view_pending=copy.copy(state)


def install_console_view(window_class):
    if getattr(window_class,'_navigation_view_installed',False):
        return
    window_class._navigation_view_installed=True
    window_class._safe_emit_state=publish_view_state
    original=window_class._render_state
    def render(self,state):
        original(self,state)
        session=getattr(self,'aeb_session',None)
        if session:
            a=session.arbiter
            self.btn_carla.setText('断开CARLA 3D' if a.carla.connected else '连接CARLA 3D')
            self.lbl_maneuver_txt.setText('动作：'+a.last_result['reason'])
            self.lbl_aeb_pill.setText('AEB关闭（RK可选）' if not a.aeb_enabled else
                'AEB制动/等待行人通过' if a.last_result['brake'] else
                'AEB已就绪' if not session.perception_reason(time.monotonic()) else 'AEB等待感知')
    window_class._render_state=render
