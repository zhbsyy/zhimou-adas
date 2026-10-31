"""Keep the original three modes; add optional AEB over navigation intent."""
from collections import deque
import json
import math
import socket
import threading
import time
import uuid

from bridge.arbiter import SimulationArbiter
from control_guard import ProtocolGate
from navigation_policy import DriveIntent,AebFlow
from safe_arbiter import full_emergency_brake
from navigation_motion import speed_plan


class NavigationArbiter(SimulationArbiter):
    def __init__(self,*args,**kwargs):
        self.protocol=ProtocolGate(uuid.uuid4().hex)
        self.session=None
        self.intent,self.flow=DriveIntent(),AebFlow()
        self.aeb_enabled=False
        self.arrived=False
        self.automatic_resumes=0
        self.epoch=0
        self.control_report,self.report_received={},0.
        self.control_error,self.control_ok_at='',0.
        self.operation_lock=threading.RLock()
        self.commands=deque(maxlen=64)
        self.serial_frames=deque(maxlen=64)
        self.nav_received,self.nav_active=0.,False
        self.nav_cancelled=False
        self.serial_last_seen=0.
        self.nav_diagnostics={'guidance_frames':0,'cruise_frames':0,'last_key_type':None}
        self.last_nav_sequence=None
        self.last_result={'drive':False,'brake':False,'phase':'paused','reason':'已暂停'}
        self._planning=False
        self._last_control_notice=''
        self._last_telemetry=0.
        super().__init__(*args,**kwargs)
        self._perception_source=self.state.perception_source='carla'

    def set_gear(self,gear,notify_cockpit=True):
        if str(gear).upper() in ('P','N','D','R'):
            self.commands.append(('gear',str(gear).upper(),notify_cockpit))

    def set_simulation_running(self,running,notify_cockpit=True):
        self.commands.append(('run',bool(running),notify_cockpit))

    def set_target_speed(self,value,notify_cockpit=True):
        try:
            value=float(value)
            if math.isfinite(value) and 0<=value<=120:
                self.commands.append(('speed',value,notify_cockpit))
        except (ValueError,TypeError):
            pass

    def set_system_mode(self,mode):
        if mode in ('cockpit_sim','perception_debug','full_ad'):
            self.commands.append(('mode',mode,False))

    def set_aeb_enabled(self,enabled):
        self.commands.append(('aeb',bool(enabled),False))

    def set_destination(self,x,y,name='',notify_cockpit=True):
        try:
            x,y=float(x),float(y)
            if not math.isfinite(x) or not math.isfinite(y):
                return
            self.commands.append(('destination',(x,y,str(name)),notify_cockpit))
        except (ValueError,TypeError):
            pass

    def reset_vehicle(self,notify_cockpit=True):
        self.commands.append(('reset',None,notify_cockpit))

    def _on_cockpit_frame(self,frame):
        if isinstance(frame,dict):
            self.serial_frames.append((dict(frame),time.monotonic()))

    def _on_serial_status(self,message,ok):
        super()._on_serial_status(message,ok)
        if not ok:
            self.serial_frames.append(({'action':'_LINK_RESET'},time.monotonic()))

    def _apply_serial_frame(self,frame,received):
        if time.monotonic()-received>2.:
            return
        action=frame.get('action')
        if action=='_LINK_RESET':
            self.last_nav_sequence=None
            self.nav_received,self.nav_active=0.,False
            self.nav_cancelled=False
            self.state.has_cockpit_nav=False
            if self.state.cockpit_drive_mode and self.flow.has_run:
                self.flow.fault='座舱串口已断开；请重新连接并继续'
            return
        if action not in ('AMAP_NAVIGATION','SIMULATOR_STATE'):
            return
        self.serial_last_seen=received
        if action=='AMAP_NAVIGATION':
            # 12205 is an independent cruise/standby message, not a navigation
            # cancel. It must not overwrite guidance, its timestamp or sequence.
            key=frame.get('amap_key_type',10001)
            diagnostics=getattr(self,'nav_diagnostics',{})
            diagnostics['last_key_type']=key
            if key==12205:
                diagnostics['cruise_frames']=diagnostics.get('cruise_frames',0)+1
                self.nav_diagnostics=diagnostics
                return
            diagnostics['guidance_frames']=diagnostics.get('guidance_frames',0)+1
            self.nav_diagnostics=diagnostics
            seq=frame.get('sequence')
            if type(seq) is not int:
                return
            if self.last_nav_sequence is not None and not 0<((seq-self.last_nav_sequence)&65535)<32768:
                return
            self.last_nav_sequence=seq
        # Reject malformed booleans/numbers before the unchanged legacy parser.
        if 'simulation_running' in frame and type(frame['simulation_running']) is not bool:
            return
        if 'navigation_active' in frame and type(frame['navigation_active']) is not bool:
            return
        for key in ('target_speed_kmh','speed_limit_kmh','navigation_speed_limit_kmh','destination_x','destination_y'):
            if key in frame and (type(frame[key]) not in (int,float) or not math.isfinite(frame[key])):
                return
        for key in ('target_speed_kmh','speed_limit_kmh','navigation_speed_limit_kmh'):
            if key in frame and not 0<=frame[key]<=120:
                return
        if 'gear' in frame and (not isinstance(frame['gear'],str) or frame['gear'].upper() not in ('P','N','D','R')):
            return
        new_destination=('destination_x' in frame and 'destination_y' in frame and
            (self._current_dest_x is None or abs(frame['destination_x']-self._current_dest_x)>1. or
             self._current_dest_y is None or abs(frame['destination_y']-self._current_dest_y)>1.))
        explicit_start=(frame.get('simulation_running') is True and self.intent.last_remote_running is False)
        if self.intent.remote(frame):
            self.flow.explicit_command()
        clean={key:value for key,value in frame.items() if key not in ('gear','simulation_running')}
        super()._on_cockpit_frame(clean)
        if action=='AMAP_NAVIGATION':
            self.nav_received=received
            self.nav_active=frame.get('navigation_active') is True
            self.nav_cancelled=not self.nav_active
        elif (not getattr(self,'nav_cancelled',False) or new_destination or explicit_start or
              frame.get('navigation_active') is True):
            self.nav_received=received
            self.nav_active=frame.get('navigation_active',True)
            self.nav_cancelled=not self.nav_active
        self.state.has_cockpit_nav=self.nav_active

    def _new_epoch(self):
        self.epoch+=1
        self.protocol.epoch=self.epoch
        self.protocol.last_seq=0
        self.control_report,self.report_received={},0.

    def _apply_command(self,kind,value,notify):
        if kind=='gear':
            self.intent.gear_request(value)
            self.flow.explicit_command()
            if value in ('D','R') and self.state.target_speed_kmh<=0:
                self.state.target_speed_kmh=float(self.state.speed_limit_kmh)
        elif kind=='run':
            self.intent.run_request(value)
            self.flow.explicit_command()
        elif kind=='speed':
            self.state.target_speed_kmh=value
        elif kind=='aeb':
            if self.aeb_enabled!=value:
                self.intent.pause()
                self.aeb_enabled=value
                self.flow.reset()
                self._new_epoch()
                self.state.takeover_enabled=value or self.state.system_mode!='cockpit_sim'
                self.log('AEB'+('开启' if value else '关闭')+'；已暂停，请明确继续导航')
        elif kind=='mode':
            # Original functions/labels: cockpit, standalone perception, full AD.
            self.intent.pause()
            self.flow.reset()
            self._new_epoch()
            super().set_system_mode(value)
            self.state.takeover_enabled=self.aeb_enabled or value!='cockpit_sim'
        elif kind=='destination':
            self.arrived=False
            self._park_for_operation()
            x,y,name=value
            if not self.carla.connected:
                self.kinematic.set_destination(x,y,name)
                self._current_dest_x,self._current_dest_y=x,y
                return
            if not self.carla.set_destination(x,y,name):
                self.carla.trajectory_waypoints=[]
                self.flow.fault='导航目标无法规划，请重新选点'
                return
            self._current_dest_x,self._current_dest_y=x,y
            self.state.route_id,self.state.route_name='dynamic_nav',name
            self.state.route_total_distance_m=self.carla.current_route_total_m
            self.state.route_remaining_distance_m=self.carla.current_route_total_m
            self._sync_route()
        elif kind=='reset':
            self.arrived=False
            self.intent.pause()
            self._park_for_operation()
            if self.carla.connected:
                self.carla.reset_to_default_spawn()
            self.kinematic.reset_to_default_spawn()
            self.flow.reset()
            self._new_epoch()
            self._current_dest_x,self._current_dest_y=None,None
        if notify and self.serial.is_connected():
            self.serial.send_route_and_state(self.state.route_id,self.intent.gear,
                self.state.target_speed_kmh,self.intent.running)

    def _sync_route(self):
        if self.serial.is_connected() and self.carla.planned_sync_waypoints:
            self.serial.send_route_waypoints(self.carla.planned_sync_waypoints,self.carla.current_route_total_m)

    def _park_for_operation(self):
        if not self.carla.connected:
            return
        self.state.gear,self.state.simulation_running='P',False
        self.carla.apply_control(self.state)
        deadline=time.monotonic()+4
        while self.state.actual_speed_kmh>.3 and time.monotonic()<deadline:
            time.sleep(.03)
            self.carla.apply_control(self.state)
        if self.state.actual_speed_kmh>.3:
            raise RuntimeError('车辆未确认驻停，取消场景修改')

    def navigation_reason(self,now):
        if not self.carla.connected and self.aeb_enabled:
            return 'CARLA未连接'
        if getattr(self.carla,'_navigation_collisions',[]):
            return 'CARLA检测到碰撞，已停止；请重置车辆后重新导航'
        if self.control_error:
            return 'CARLA控制异常'
        if self.state.cockpit_drive_mode:
            if not self.serial.is_connected():
                return '等待树莓派座舱串口'
            if not self.nav_active:
                return '导航已取消/等待有效导航指引'
            if now-self.nav_received>10.:
                return '导航指引超过10秒未更新'
        if self.carla.connected and not self.carla.trajectory_waypoints:
            return '等待有效导航目的地/路线'
        if not self.carla.connected and self.state.route_total_distance_m<=0:
            return '等待有效导航目的地/路线'
        return ''

    def driving_reason(self,now):
        return self.driving_health(now)[0]

    def driving_health(self,now):
        reason=self.navigation_reason(now)
        if reason:
            return reason,False
        if self.aeb_enabled:
            if self.intent.gear=='R':
                return '本AEB为前向功能，请先关闭AEB再倒车',False
            if self.session and hasattr(self.session,'perception_health'):
                return self.session.perception_health(now)
            return (self.session.perception_reason(now) if self.session else '感知会话未就绪'),False
        return '',False

    def _on_vehicle_arrived(self):
        self.arrived=True
        self.intent.pause()
        super()._on_vehicle_arrived()

    def _start_udp_server(self):
        self._udp_server_sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        if hasattr(socket,'SO_EXCLUSIVEADDRUSE'):
            self._udp_server_sock.setsockopt(socket.SOL_SOCKET,socket.SO_EXCLUSIVEADDRUSE,1)
        self._udp_server_sock.bind(('0.0.0.0',8888))
        self._udp_server_sock.settimeout(.2)
        self._udp_thread=threading.Thread(target=self._udp_listen_loop,daemon=True)
        self._udp_thread.start()

    def _udp_listen_loop(self):
        while self._running:
            try:
                data,peer=self._udp_server_sock.recvfrom(16384)
                payload=json.loads(data)
                kind=self.protocol.accept(payload,peer[0])
                if kind is None:
                    continue
                self.state.vision_udp_connected=True
                self.state.vision_udp_last_time=time.time()
                self.state.vision_udp_endpoint=peer[0]+':'+str(peer[1])
                self._last_adas_ip=peer[0]
                if kind=='control':
                    self.control_report,self.report_received=payload,time.monotonic()
                    self.state.vision_action=str(payload.get('action',''))[:120]
                    self.state.vision_veto=str(payload.get('veto',''))[:240]
            except socket.timeout:
                if time.time()-self.state.vision_udp_last_time>2.5:
                    self.state.vision_udp_connected=False
            except (ValueError,TypeError,KeyError,AttributeError):
                continue
            except OSError:
                break

    def _physics_loop(self):
        last_step=time.monotonic()
        while self._running:
            try:
                if not self.operation_lock.acquire(blocking=False):
                    time.sleep(.02)
                    continue
                try:
                    while self.serial_frames:
                        self._apply_serial_frame(*self.serial_frames.popleft())
                    # Preserve command order; route planning is done only here.
                    for _ in range(min(16,len(self.commands))):
                        self._apply_command(*self.commands.popleft())
                    now=time.monotonic()
                    reason,transient=self.driving_health(now)
                    result=self.flow.step(enabled=self.aeb_enabled,intent=self.intent,
                        reason=reason,transient=transient,report=self.control_report,
                        speed=self.state.actual_speed_kmh,now=now)
                    if self.arrived:
                        result={'drive':False,'brake':False,'phase':'arrived','reason':'已到达目的地，P挡驻车','resumed':False}
                    if result.get('resumed'):
                        self.automatic_resumes+=1
                    self.last_result=result
                    self.state.gear=self.intent.gear if result['drive'] or result['brake'] else 'P'
                    self.state.simulation_running=result['drive'] or result['brake']
                    self.state.vision_aeb_active=result['brake']
                    self.state.vision_speed_override=0. if result['brake'] else None
                    self.state.navigation_aeb_enabled=self.aeb_enabled
                    self.state.navigation_speed_cap_kmh,self.speed_plan_reason=speed_plan(self.carla,self.aeb_enabled) if self.carla.connected else (None,'')
                    if self.carla.connected:
                        if result['brake']:
                            self.state.vision_action=result['reason']
                            full_emergency_brake(self.carla,self.state)
                            self.carla._update_pedestrian_scene(self.state,0.,self.state.actual_speed_kmh)
                        else:
                            self.carla.apply_control(self.state)
                    else:
                        self.kinematic.step(min(.1,max(.001,now-last_step)))
                    last_step=now
                    self.control_ok_at=time.monotonic()
                    self.control_error=''
                    if result['reason']!=self._last_control_notice:
                        self.log('【导航/AEB】'+result['reason'])
                        self._last_control_notice=result['reason']
                    self._dispatch_state()
                    self._send_telemetry(now)
                finally:
                    self.operation_lock.release()
            except Exception as exc:
                self.control_error=repr(exc)
                self.intent.pause()
                self.flow.fault='CARLA控制异常；请重新连接并继续'
                self.state.gear,self.state.simulation_running='P',False
            time.sleep(.02)

    def _send_telemetry(self,now):
        if now-self._last_telemetry<.2 or not self.serial.is_connected():
            return
        self._last_telemetry=now
        s=self.state
        self.serial.send_telemetry(s.gear,s.actual_speed_kmh,s.target_speed_kmh,
            s.segment_remaining_distance_m,s.route_remaining_distance_m,
            route_id=s.route_id,route_name=s.route_name,simulation_running=s.simulation_running,
            pos_x=s.pos_x,pos_y=s.pos_y,heading_deg=math.degrees(s.heading_rad),
            pedestrian_detected=s.pedestrian_detected,pedestrian_distance_m=s.pedestrian_distance_m,
            aeb_active=s.vision_aeb_active)

    def close(self):
        self.intent.pause()
        with self.operation_lock:
            self.state.gear,self.state.simulation_running='P',False
            if self.carla.connected:
                try:
                    self.carla.apply_control(self.state)
                except Exception:
                    pass
            self._running=False
        for sock in (self._udp_server_sock,self._udp_client_sock):
            try:
                sock.close()
            except OSError:
                pass
        for thread in (self._loop_thread,self._broadcast_thread,self._udp_thread):
            if thread is not threading.current_thread():
                thread.join(2.)
        self.serial.disconnect()
        with self.operation_lock:
            self.carla.disconnect()
