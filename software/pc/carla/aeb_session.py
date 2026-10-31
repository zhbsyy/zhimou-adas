"""Demo controls and annotated CARLA telemetry; vehicle control stays in arbiter."""
import json
import math
from pathlib import Path
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.request import build_opener, ProxyHandler

import numpy as np
from PyQt5 import QtCore, QtWidgets
from reliable_streamer import ReliableStreamer
from control_guard import ProgressWatch,SCHEMA

ROOT = Path(__file__).resolve().parent


def install_console_view(window_class):
    original = window_class._render_state
    def render(self,state):
        original(self,state)
        session = getattr(self,'aeb_session',None)
        if session is None:
            return
        if not state.cockpit_drive_mode:
            self.lbl_maneuver_txt.setText('动作：'+('AEB制动保持' if session.arbiter.aeb_latched else
                state.current_maneuver if session.active else '驻停，等待开始AEB测试'))
        if not session.active and not session.arbiter.aeb_latched:
            reason = session.unready_reason(time.monotonic())
            self.lbl_aeb_pill.setText('AEB未就绪' if reason else 'AEB已就绪')
            self.lbl_aeb_pill.setStyleSheet('color:#fbbf24;' if reason else 'color:#34d399;')
    window_class._render_state = render


def projected_box(vertices, inverse, width=640, height=360, fov=60):
    points = np.asarray([[v.x, v.y, v.z, 1.] for v in vertices], dtype=float)
    coords = (np.asarray(inverse) @ points.T).T
    if np.any(coords[:, 0] <= .2):
        return None
    focal = width/(2*math.tan(math.radians(fov)/2))
    u = focal*coords[:, 1]/coords[:, 0]+width/2
    v = height/2-focal*coords[:, 2]/coords[:, 0]
    box = [max(0., float(u.min())), max(0., float(v.min())),
           min(width-1., float(u.max())), min(height-1., float(v.max()))]
    return [round(x, 2) for x in box] if box[2]-box[0] > 1 and box[3]-box[1] > 1 else None


class AnnotatedSocket:
    def __init__(self, original, session):
        self.original, self.session = original, session
        self.seq = 0
        self.lock = threading.Lock()

    def sendto(self, data, address):
        payload = json.loads(data)
        scene = dict(self.session.scene)
        age = time.monotonic()-self.session.scene_received
        scene['scene_age_ms'] = round(age*1000,1) if self.session.scene_received else None
        if age > .4:
            scene['valid'] = False
        payload['carla_aeb_scene'] = scene
        payload['carla_aeb_session'] = self.session.arbiter.protocol.session_id
        with self.lock:
            self.seq += 1
            payload['carla_aeb_state_seq'] = self.seq
        return self.original.sendto(json.dumps(payload, ensure_ascii=False).encode(), address)

    def __getattr__(self, name):
        return getattr(self.original, name)


class AebSession(QtCore.QObject):
    message_changed = QtCore.pyqtSignal(str)
    job_finished = QtCore.pyqtSignal()

    def __init__(self, window):
        super().__init__(window)
        self.window, self.arbiter = window, window.arbiter
        self.streamer, self.scene, self.rk = None, {'valid': False}, {}
        self.rk_received = 0.
        self.scene_received = 0.
        self.job = None
        self.progress = ProgressWatch()
        self.active, self.stopping = False, threading.Event()
        self.max_speed, self.brake_started = 0., None
        self.last_error, self.last_event = '', '先准备CARLA，再启动板端双模型，最后开始AEB测试'
        self.last_logged, self.last_report_seq, self.last_progress = 0., 0, 0.
        self.arbiter._udp_client_sock = AnnotatedSocket(self.arbiter._udp_client_sock, self)
        dock = QtWidgets.QDockWidget('CARLA移动行人AEB测试（距离来源：仿真真值）', window)
        dock.setFeatures(QtWidgets.QDockWidget.NoDockWidgetFeatures)
        pane, layout = QtWidgets.QWidget(), QtWidgets.QHBoxLayout()
        self.buttons = []
        for label, slot in [('1. 准备CARLA场景', self.prepare), ('2. 开始AEB测试', self.start_drive),
                            ('重新生成前方行人', self.respawn_request), ('暂停车辆', self.pause)]:
            button = QtWidgets.QPushButton(label)
            button.setStyleSheet('QPushButton {background:#234665;color:#ffffff;border:1px solid #6386a3;'
                                'padding:8px;min-width:105px;} '
                                'QPushButton:disabled {background:#1b2838;color:#7791a5;}')
            button.clicked.connect(slot)
            layout.addWidget(button)
            self.buttons.append(button)
        self.label = QtWidgets.QLabel(self.last_event)
        self.label.setWordWrap(True)
        self.message_changed.connect(self.label.setText)
        self.job_finished.connect(lambda: [button.setEnabled(True) for button in self.buttons])
        layout.addWidget(self.label, 1)
        pane.setLayout(layout)
        dock.setWidget(pane)
        dock.setStyleSheet('QDockWidget, QWidget {background:#111f31;color:#e5efff;} '
                          'QPushButton {background:#234665;color:white;border:1px solid #6386a3;'
                          'padding:8px;min-width:105px;} QPushButton:disabled {color:#7791a5;} '
                          'QLabel {padding:5px;}')
        window.addDockWidget(QtCore.Qt.TopDockWidgetArea, dock)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(100)
        self.poller = threading.Thread(target=self.poll_rk, daemon=True)
        self.poller.start()
        self.scene_poller = threading.Thread(target=self.poll_scene, daemon=True)
        self.scene_poller.start()
        self.server = ThreadingHTTPServer(('127.0.0.1', 18090), self.handler())
        self.http = threading.Thread(target=lambda: self.server.serve_forever(poll_interval=.2), daemon=True)
        self.http.start()
        self.events = ROOT/'Results'
        self.events.mkdir(exist_ok=True)
        self.arbiter.session = self
        # The frozen console remains available separately. This dedicated test
        # must not bypass the preparation/start gates via legacy manual controls.
        for name in ('btn_mode_cockpit','btn_mode_perception','btn_mode_full_ad','btn_carla','btn_toggle_sim','btn_reset_sim','btn_rk_decision','btn_serial','cb_ports'):
            widget = getattr(window,name,None)
            if widget is not None:
                widget.setEnabled(False)
        for gear,button in window.gear_buttons.items():
            if gear in ('D','R','N'):
                button.setEnabled(False)
        for button in window.findChildren(QtWidgets.QPushButton):
            if '切换感知源' in button.text():
                button.setEnabled(False)
        for label in window.findChildren(QtWidgets.QLabel):
            if label.text().startswith('规格:'):
                label.setText('相机源1280×720 / 采样20 Hz；RK推理帧率见画面横条')

    def notice(self, text):
        self.last_event = text
        self.arbiter.log('[CARLA AEB测试] '+text)
        self.message_changed.emit(text)

    def prepare(self):
        self.start_job(self.prepare_worker)

    def start_job(self, function):
        if self.job is not None and self.job.is_alive():
            return
        for button in self.buttons[:3]:
            button.setEnabled(False)
        def run():
            try:
                function()
            finally:
                self.job_finished.emit()
        self.job = threading.Thread(target=run,daemon=True)
        self.job.start()

    def wait_for_park(self):
        self.pause()
        if not self.arbiter.carla.connected:
            return
        requested = time.monotonic()
        deadline = requested+4
        while time.monotonic()<deadline:
            if self.arbiter.control_ok_at>requested and self.arbiter.state.actual_speed_kmh<.3:
                return
            if self.stopping.wait(.03):
                raise RuntimeError('窗口已关闭')
        raise RuntimeError('车辆尚未确认驻停，暂不修改场景')

    def prepare_worker(self):
        try:
            self.wait_for_park()
            with self.arbiter.operation_lock:
                self.prepare_parked()
        except Exception as exc:
            self.last_error = repr(exc)
            self.pause()
            self.notice('准备失败：'+str(exc))

    def prepare_parked(self):
        try:
            self.last_error = ''
            self.active = False
            if not self.arbiter.carla.connected and not self.arbiter.connect_carla():
                raise RuntimeError(self.arbiter.carla.get_status())
            self.arbiter.set_perception_source('carla')
            # The original mode shortcut also engages D immediately. Prepare
            # the same route here while staying in P until the explicit start.
            self.pause()
            self.arbiter.state.system_mode = 'perception_debug'
            self.arbiter.set_takeover_enabled(True)
            self.arbiter.set_cockpit_drive_mode(False)
            self.arbiter.set_destination(98.8,82.9,'东岸CBD巡航测试线',notify_cockpit=False)
            self.arbiter.state.speed_limit_kmh = 20
            self.arbiter.set_target_speed(20, notify_cockpit=False)
            if self.streamer is None or not self.streamer.health()['ready']:
                if self.streamer is not None:
                    self.streamer.stop()
                self.streamer = ReliableStreamer(http_port=8081)
                if not self.streamer.attach_to_carla():
                    reason = self.streamer.attach_error
                    self.streamer = None
                    raise RuntimeError('四相机挂载失败：'+reason)
                self.streamer.start_http_server()
            if not self.respawn():
                raise RuntimeError('前方行人未就绪')
            self.notice('场景与四相机8081已就绪，车辆驻停；启动RK后点“开始AEB测试”')
        except Exception as exc:
            self.last_error = repr(exc)
            self.pause()
            self.notice('准备失败：'+str(exc))

    def respawn_request(self):
        def parked_respawn():
            try:
                self.wait_for_park()
                with self.arbiter.operation_lock:
                    self.respawn()
            except Exception as exc:
                self.last_error = repr(exc)
                self.notice('生成行人失败：'+str(exc))
        self.start_job(parked_respawn)

    def respawn(self):
        try:
            self.pause()
            backend = self.arbiter.carla
            path = backend.trajectory_waypoints
            if not path:
                raise RuntimeError('请先准备CARLA并规划路线')
            index = min(len(path)-2, backend.last_matched_idx+max(10, int(28/backend.trajectory_step)))
            if index <= backend.last_matched_idx+4:
                raise RuntimeError('已接近路线终点，请重新准备场景')
            if not backend._spawn_dynamic_pedestrian(path[index]):
                raise RuntimeError('行人生成失败')
            self.brake_started, self.max_speed = None, 0.
            self.arbiter.aeb_latched = False
            self.arbiter.state.vision_aeb_active = False
            self.arbiter.state.vision_speed_override = None
            self.notice('已复用原场景模块，在前方路线约28m设置横穿行人；等待开始')
            return True
        except Exception as exc:
            self.last_error = repr(exc)
            self.notice('生成行人失败：'+str(exc))
            return False

    def start_drive(self):
        if self.active:
            return
        now = time.monotonic()
        reason = self.unready_reason(now)
        if reason:
            self.notice('尚未就绪：'+reason)
            return
        self.active, self.max_speed, self.brake_started = True, 0., None
        self.arbiter.set_gear('D', notify_cockpit=False)
        self.arbiter.set_simulation_running(True, notify_cockpit=False)
        self.notice('AEB闭环测试开始：道路循迹20km/h，等待RK识别横穿行人')

    def unready_reason(self,now,driving=False):
        arbiter = self.arbiter
        if not arbiter.carla.connected:
            return 'CARLA未连接'
        if arbiter.control_error or now-arbiter.control_ok_at>.6:
            return 'CARLA控制线程未正常更新'
        if self.streamer is None or not self.streamer.health()['ready']:
            return '四相机已断流或尚未就绪'
        if arbiter.state.perception_source != 'carla' or arbiter.state.cockpit_drive_mode:
            return '测试模式或感知源已改变'
        if now-self.rk_received>1.5:
            return '等待RK双模型连接'
        aeb = self.rk.get('carla_aeb',{})
        if (self.rk.get('source')!='carla' or aeb.get('schema')!=SCHEMA or not aeb.get('enabled')
            or aeb.get('session_id')!=arbiter.protocol.session_id or aeb.get('run_id')!=arbiter.protocol.run_id):
            return 'RK程序与本轮场景不匹配'
        reason = self.progress.stale_reason(now)
        if reason:
            return reason
        if now-arbiter.report_received>.6 or not arbiter.state.vision_udp_connected:
            return 'RK控制报文未持续更新'
        evidence = arbiter.control_report.get('evidence',{})
        if not arbiter.aeb_latched and (not evidence.get('valid_scene') or not evidence.get('capture_fresh')):
            return '输入图像或行人场景已过期'
        if not arbiter.aeb_latched and (now-self.scene_received>.4 or not self.scene.get('valid')
            or evidence.get('actor_id') != self.scene.get('actor_id')):
            return '等待RK同步本轮行人场景'
        if not driving and (arbiter.aeb_latched or arbiter.control_report.get('aeb_triggered')):
            return '制动已保持，请先重新生成行人'
        return ''

    def pause(self):
        self.active = False
        self.arbiter.set_simulation_running(False, notify_cockpit=False)
        self.arbiter.set_gear('P', notify_cockpit=False)

    def poll_rk(self):
        opener = build_opener(ProxyHandler({}))
        while not self.stopping.is_set():
            try:
                with opener.open('http://192.168.2.3:18084/aeb_status', timeout=.6) as response:
                    value = json.loads(response.read(300000))
                if not isinstance(value,dict):
                    continue
                aeb = value.get('carla_aeb',{})
                if not isinstance(aeb,dict) or aeb.get('schema')!=SCHEMA or aeb.get('session_id')!=self.arbiter.protocol.session_id:
                    continue
                if not self.active:
                    self.arbiter.protocol.select_run(aeb.get('run_id'))
                self.progress.observe(value,time.monotonic())
                self.rk, self.rk_received = value, time.monotonic()
            except Exception:
                pass
            self.stopping.wait(.4)

    def update_scene(self):
        backend = self.arbiter.carla
        walker, vehicle = backend._scene_walker, backend.vehicle
        if not backend.connected or walker is None or vehicle is None or self.streamer is None or not self.streamer.carla_cameras:
            return {'valid': False, 'range_source': 'CARLA_actor_truth'}
        camera = self.streamer.carla_cameras[0]
        pose, target = vehicle.get_transform(), walker.get_location()
        dx, dy = target.x-pose.location.x, target.y-pose.location.y
        yaw = math.radians(pose.rotation.yaw)
        forward = dx*math.cos(yaw)+dy*math.sin(yaw)-vehicle.bounding_box.extent.x
        lateral = dx*math.sin(yaw)-dy*math.cos(yaw)
        bbox = projected_box(walker.bounding_box.get_world_vertices(walker.get_transform()), camera.get_transform().get_inverse_matrix())
        return {'valid': bbox is not None and forward > 0, 'range_source': 'CARLA_actor_truth',
                'actor_id': walker.id, 'bbox_front': bbox, 'front_size': [640, 360],
                'forward_gap_m': round(forward, 3), 'lateral_m': round(lateral, 3),
                'carla_frame': backend.world.get_snapshot().frame,
                'note': 'simulation actor projection/range, requires matching RK visual detection; not stereo measured'}

    def poll_scene(self):
        while not self.stopping.is_set():
            try:
                if self.arbiter.operation_lock.acquire(blocking=False):
                    try:
                        self.scene = self.update_scene()
                        self.scene_received = time.monotonic()
                    finally:
                        self.arbiter.operation_lock.release()
            except Exception as exc:
                self.scene = {'valid':False,'range_source':'CARLA_actor_truth'}
                self.last_error = repr(exc)
            self.stopping.wait(.1)

    def tick(self):
        now = time.monotonic()
        state = self.arbiter.state
        report_seq = self.rk.get('carla_aeb', {}).get('det_report_seq', 0)
        if report_seq != self.last_report_seq:
            self.last_report_seq, self.last_progress = report_seq, now
        if self.active:
            self.max_speed = max(self.max_speed, state.actual_speed_kmh)
            reason = self.unready_reason(now,driving=True)
            if reason:
                self.pause()
                self.notice('测试暂停并驻车：'+reason)
            elif state.vision_aeb_active:
                self.brake_started = self.brake_started or now
                if self.max_speed >= 3 and state.actual_speed_kmh < .5 and now-self.brake_started > .8:
                    self.pause()
                    self.notice('已观察到RK AEB触发后车辆停住；本轮测试结束，可重新生成行人再测')
        busy = self.job is not None and self.job.is_alive()
        self.buttons[1].setEnabled(not busy and not self.active and not self.unready_reason(now))
        if now-self.last_logged >= 1:
            self.last_logged = now
            snapshot = self.snapshot()
            with (self.events/'events.jsonl').open('a', encoding='utf-8') as out:
                out.write(json.dumps(snapshot, ensure_ascii=False)+'\n')

    def snapshot(self):
        state = self.arbiter.state
        return {'event': self.last_event, 'active': self.active, 'error': self.last_error,
            'carla_connected': self.arbiter.carla.connected,
            'vehicle_id': self.arbiter.carla.vehicle.id if self.arbiter.carla.vehicle else None,
            'speed_kmh': state.actual_speed_kmh, 'max_speed_kmh': self.max_speed, 'gear': state.gear,
            'takeover': state.takeover_enabled, 'vision_connected': state.vision_udp_connected,
            'vision_aeb_active': state.vision_aeb_active, 'brake_lamp': state.brake_lamp,
            'scene': self.scene, 'rk_aeb': self.rk.get('carla_aeb'),
            'rk_stats_age_s': round(time.monotonic()-self.rk_received, 2) if self.rk_received else None,
            'streamer_cameras': len(self.streamer.carla_cameras) if self.streamer else 0,
            'camera_health':self.streamer.health() if self.streamer else None,
            'control_error':self.arbiter.control_error,'aeb_latched':self.arbiter.aeb_latched,
            'ready':not self.unready_reason(time.monotonic()),
            'not_ready_reason':self.unready_reason(time.monotonic()),
            'scene_age_ms':round((time.monotonic()-self.scene_received)*1000,1) if self.scene_received else None,
            'time_pc': time.time()}

    def handler(self):
        session = self
        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass
            def do_GET(self):
                body = json.dumps(session.snapshot(), ensure_ascii=False).encode()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
        return Handler

    def close(self):
        self.pause()
        self.timer.stop()
        self.stopping.set()
        self.server.shutdown()
        self.server.server_close()
        self.poller.join(2)
        self.scene_poller.join(2)
        if self.streamer is not None:
            self.streamer.stop()
