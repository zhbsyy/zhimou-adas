"""Dedicated AEB candidate arbiter; frozen navigation and submission are unchanged."""
import json
import math
import socket
import threading
import time
import uuid

import carla
from bridge.arbiter import SimulationArbiter
from control_guard import ProtocolGate


def full_emergency_brake(backend,state):
    """One immediate command: cut throttle and apply full service brake."""
    vehicle = backend.vehicle
    velocity = vehicle.get_velocity()
    state.actual_speed_kmh = 3.6*math.sqrt(velocity.x**2+velocity.y**2+velocity.z**2)
    pose = vehicle.get_transform()
    state.pos_x,state.pos_y = pose.location.x,pose.location.y
    state.heading_rad = math.radians(pose.rotation.yaw)
    state.brake_lamp = True
    state.current_maneuver = '视觉AEB紧急制动：'+state.vision_action
    backend._current_throttle,backend._current_brake = 0.,1.
    backend._target_v_smooth = 0.
    backend._longitudinal_mode = 'brake'
    vehicle.apply_control(carla.VehicleControl(throttle=0.,brake=1.,
        steer=max(-1.,min(1.,backend._current_steer)),hand_brake=False))
    lights = carla.VehicleLightState.Brake
    if state.turn_signal == 'left':
        lights |= carla.VehicleLightState.LeftBlinker
    elif state.turn_signal == 'right':
        lights |= carla.VehicleLightState.RightBlinker
    vehicle.set_light_state(carla.VehicleLightState(lights))
    backend._update_camera()


class AebArbiter(SimulationArbiter):
    def __init__(self,*args,**kwargs):
        self.protocol = ProtocolGate(uuid.uuid4().hex)
        self.session = None
        self.control_report,self.report_received = {},0.
        self.aeb_latched = False
        self.control_error,self.control_ok_at = '',0.
        self.operation_lock = threading.RLock()
        self._last_fault_logged = ''
        super().__init__(*args,**kwargs)

    # UI setters update desired state; actual CARLA commands belong to one worker.
    def set_gear(self,gear,notify_cockpit=True):
        gear = gear.upper()
        if gear not in ('D','P'):
            return
        if gear == 'D' and not (self.session and self.session.active):
            self.log('请使用顶部“开始AEB测试”；未就绪时禁止直接挂行车挡')
            return
        if gear=='P' and self.session:
            self.session.active = False
        self.kinematic.set_gear(gear)
        self._dispatch_state()

    def set_target_speed(self,speed_kmh,notify_cockpit=True):
        self.kinematic.set_target_speed(min(20.,max(0.,float(speed_kmh))))
        self._dispatch_state()

    def set_system_mode(self,mode):
        if self.session:
            self.session.pause()
            self.log('此窗口为CARLA移动AEB专用；请使用顶部场景/开始/暂停按钮')

    def _start_udp_server(self):
        self._udp_server_sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        if hasattr(socket,'SO_EXCLUSIVEADDRUSE'):
            self._udp_server_sock.setsockopt(socket.SOL_SOCKET,socket.SO_EXCLUSIVEADDRUSE,1)
        try:
            self._udp_server_sock.bind(('0.0.0.0',8888))
        except OSError:
            self._udp_server_sock.close()
            raise RuntimeError('UDP8888已被占用；先正常关闭其他总控台')
        self._udp_server_sock.settimeout(.2)
        self._udp_thread = threading.Thread(target=self._udp_listen_loop,daemon=True)
        self._udp_thread.start()

    def _udp_listen_loop(self):
        while self._running:
            try:
                data,peer = self._udp_server_sock.recvfrom(16384)
                payload = json.loads(data)
                accepted = self.protocol.accept(payload,peer[0])
                if accepted is None:
                    continue
                self.state.vision_udp_connected = True
                self.state.vision_udp_last_time = time.time()
                self.state.vision_udp_endpoint = peer[0]+':'+str(peer[1])
                self._last_adas_ip = peer[0]
                if accepted == 'heartbeat':
                    continue
                self.control_report,self.report_received = payload,time.monotonic()
                self.state.vision_action = str(payload.get('action',''))[:120]
                self.state.vision_veto = str(payload.get('veto',''))[:240]
                if self.session and self.session.active and payload['aeb_triggered']:
                    self.aeb_latched = True
                self.state.vision_aeb_active = self.aeb_latched
                self.state.vision_speed_override = 0. if self.aeb_latched else None
                self.state.vision_steer_bias = 0.
            except socket.timeout:
                if time.time()-self.state.vision_udp_last_time > 2.5:
                    self.state.vision_udp_connected = False
            except (ValueError,TypeError,KeyError,AttributeError):
                continue
            except OSError:
                break

    def _physics_loop(self):
        while self._running:
            try:
                # Scene preparation/destruction can reserve the backend briefly.
                if not self.operation_lock.acquire(blocking=False):
                    time.sleep(.02)
                    continue
                try:
                    session = self.session
                    if not session or not session.active:
                        self.state.gear,self.state.simulation_running = 'P',False
                    elif session.unready_reason(time.monotonic(),driving=True):
                        self.state.gear,self.state.simulation_running = 'P',False
                    self.state.vision_aeb_active = self.aeb_latched
                    if self.carla.connected:
                        if self.aeb_latched and self.state.gear != 'P':
                            full_emergency_brake(self.carla,self.state)
                        else:
                            self.carla.apply_control(self.state)
                    self.control_ok_at = time.monotonic()
                    self.control_error = ''
                finally:
                    self.operation_lock.release()
                self._dispatch_state()
            except Exception as exc:
                self.control_error = repr(exc)
                self.state.gear,self.state.simulation_running = 'P',False
                if self.control_error != self._last_fault_logged:
                    self.log('CARLA控制接口异常，停止本轮：'+self.control_error)
                    self._last_fault_logged = self.control_error
            time.sleep(.02)
