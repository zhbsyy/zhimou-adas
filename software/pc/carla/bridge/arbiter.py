from __future__ import annotations

import json
import math
import socket
import threading
import time
from typing import Callable, Optional

from .carla_backend import CarlaBackend
from .kinematic_backend import KinematicSimulator, VehicleState
from .serial_client import SerialClient


class VisionStopDebouncer:
    """视觉"停车类"指令（红灯 / 让行）的时间防抖。

    单目灯态识别会逐帧翻转——把红色建筑、前车刹车灯误判成信号灯。一帧把目标车速
    打到 0、下一帧又放开，观感就是"车一窜一停、刹车灯一闪一闪"。这里要求
    "连续 confirm 帧确认才生效、连续 release 帧消失才解除"，带迟滞，
    挡掉单帧毛刺。纯逻辑，便于单测。
    """

    def __init__(self, confirm_frames: int, release_frames: int) -> None:
        self.confirm_frames = max(1, int(confirm_frames))
        self.release_frames = max(1, int(release_frames))
        self._run = 0
        self._clear = 0
        self.active = False

    def update(self, wants_stop: bool) -> bool:
        if wants_stop:
            self._run += 1
            self._clear = 0
        else:
            self._clear += 1
            self._run = 0
        if self._run >= self.confirm_frames:
            self.active = True
        elif self._clear >= self.release_frames:
            self.active = False
        return self.active


class SimulationArbiter:
    """自动驾驶仿真与双向座舱仲裁中心。"""

    # ICD §4.2：下行快照固定 20 Hz（每 50 ms 一帧）
    VISION_BROADCAST_HZ = 20.0

    # 视觉"停车类"指令（红灯 / 让行）的时间防抖：既要连续这么多帧确认才生效，
    # 也要连续这么多帧消失才解除。原因是单目的灯态识别会逐帧翻转（把红色建筑、
    # 前车刹车灯误判成信号灯），一帧把目标车速打到 0、下一帧又放开，观感就是
    # "车一窜一停、刹车灯一闪一闪"。带迟滞的连续帧确认可以挡掉这种毛刺。
    VISION_STOP_CONFIRM_FRAMES = 4
    VISION_STOP_RELEASE_FRAMES = 8

    def __init__(self,
                 on_state_change: Optional[Callable[[VehicleState], None]] = None,
                 on_log: Optional[Callable[[str], None]] = None) -> None:
        self.on_state_change = on_state_change
        self.on_log = on_log

        # 核心引擎组件
        self.kinematic = KinematicSimulator(
            on_update=self._on_kinematic_tick,
            on_arrival=self._on_vehicle_arrived
        )
        self.carla = CarlaBackend(on_arrival=self._on_vehicle_arrived, on_event=self.log)
        self.serial = SerialClient(on_frame=self._on_cockpit_frame, on_status=self._on_serial_status)

        # 默认严格开启座舱高德联动驱动模式 (无高德导航数据禁止潜行)
        self.state.cockpit_drive_mode = True
        self.state.wait_nav_reason = "等待座舱串口连接"

        self._last_telemetry_time = 0.0
        self._last_cockpit_log_time = 0.0
        self._running = True

        self._perception_source = "pcie"
        self.state.perception_source = "pcie"
        self.state.takeover_enabled = False

        # UDP 视觉智驾域双向链路 (8888 接收智驾控制请求，8892 广播车速与导航快照)
        self._udp_server_sock: Optional[socket.socket] = None
        self._udp_client_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._start_udp_server()

        self._loop_thread = threading.Thread(target=self._physics_loop, daemon=True)
        self._loop_thread.start()

        # ICD §4.2：8892 广播用独立线程跑固定 20 Hz。挂在物理循环上做门限做不到——
        # 物理循环实测约 20.5 ms/拍，"至少 50 ms" 的门限只能每 3 拍触发，实测仅 ~16 Hz。
        self._broadcast_thread = threading.Thread(
            target=self._broadcast_loop, name="oib-broadcast", daemon=True)
        self._broadcast_thread.start()


    @property
    def state(self) -> VehicleState:
        return self.kinematic.state

    def log(self, message: str) -> None:
        if self.on_log:
            self.on_log(f"[{time.strftime('%H:%M:%S')}] {message}")

    def _on_vehicle_arrived(self) -> None:
        """智驾到达终点后停止仿真、挂入 P 档并下发座舱。"""
        self.log("【智驾自动泊车】车辆已平稳到达路线终点，已自动制动减速至 0 km/h，系统自动挂入 P 挡并锁止手刹！")
        self.state.simulation_running = False
        self.set_gear("P", notify_cockpit=True)
        self.state.current_maneuver = "已到达终点 (P档已驻车)"
        self._dispatch_state()

    # ------------------------------------------------------------- 控制指令
    def set_gear(self, gear: str, notify_cockpit: bool = True) -> None:
        old_gear = self.state.gear
        self.kinematic.set_gear(gear)
        # 挂入行车挡 (D / R) 时自动解除暂停并确保有有效巡航速度
        if gear.upper() in ("D", "R"):
            self.state.simulation_running = True
            if self.state.target_speed_kmh <= 0.0:
                self.state.target_speed_kmh = float(self.state.speed_limit_kmh)
        if self.carla.connected:
            self.carla.apply_control(self.state)
        self.log(f"挡位切换: {old_gear} -> {self.state.gear}")
        if notify_cockpit and self.serial.is_connected():
            self.serial.send_gear_and_speed(
                self.state.gear, round(self.state.target_speed_kmh),
                route_id=self.state.route_id, route_name=self.state.route_name,
                simulation_running=self.state.simulation_running
            )
        self._dispatch_state()

    def set_target_speed(self, speed_kmh: float, notify_cockpit: bool = True) -> None:
        self.kinematic.set_target_speed(speed_kmh)
        if self.carla.connected:
            self.carla.apply_control(self.state)
        self.log(f"目标速度设置: {speed_kmh} km/h")
        if notify_cockpit and self.serial.is_connected():
            self.serial.send_gear_and_speed(
                self.state.gear, round(self.state.target_speed_kmh),
                route_id=self.state.route_id, route_name=self.state.route_name,
                simulation_running=self.state.simulation_running
            )
        self._dispatch_state()

    def set_perception_source(self, source: str) -> None:
        source = str(source).lower()
        if source in ("pcie", "carla"):
            self._perception_source = source
            self.state.perception_source = source
            self.log(f"【感知源切换】已切换视频感知源为: {source.upper()}")
            self._broadcast_vision_state()

    def set_takeover_enabled(self, enabled: bool) -> None:
        self.state.takeover_enabled = bool(enabled)
        status_txt = "开启" if self.state.takeover_enabled else "关闭"
        self.log(f"【智驾接管】底盘动力学接管控制已{status_txt}")
        self._dispatch_state()

    def set_system_mode(self, mode: str) -> None:
        """切换三态智驾运行模式：cockpit_sim (座舱联调) / perception_debug (感知单机) / full_ad (全域智驾)。"""
        if mode not in ("cockpit_sim", "perception_debug", "full_ad"):
            return
        self.state.system_mode = mode
        if mode == "cockpit_sim":
            self.state.takeover_enabled = False
            self.state.cockpit_drive_mode = True
            self.log("【模式切换】座舱仿真联调模式 (无RK环境：座舱全权导航，CARLA高精循迹，感知旁路)")
        elif mode == "perception_debug":
            self.state.takeover_enabled = True
            self.state.cockpit_drive_mode = False
            self.state.has_cockpit_nav = True
            # 自动规划默认巡航测试路线 (东岸CBD金融中心 98.8, 82.9)，生成道路轨迹与横穿行人场景
            self.set_destination(98.8, 82.9, "东岸CBD巡航测试线", notify_cockpit=False)
            self.set_gear("D", notify_cockpit=False)
            self.state.simulation_running = True
            self.log("【模式切换】感知算法单机模式 (自建巡航测试线+动态行人场景，验证推流/检测/AEB)")
        elif mode == "full_ad":
            self.state.takeover_enabled = True
            self.state.cockpit_drive_mode = True
            self.log("【模式切换】全域智驾闭环模式 (全要素联调：座舱导航 + RK视觉车道线与行人AEB)")
        self._dispatch_state()

    def set_destination(self, dest_x: float, dest_y: float, dest_name: str = "", notify_cockpit: bool = True) -> None:
        self.state.route_id = "dynamic_nav"
        self.state.route_name = dest_name or f"目标点 ({dest_x:.1f}, {dest_y:.1f})"
        self._current_dest_x = dest_x
        self._current_dest_y = dest_y
        if self.carla.connected:
            self.carla.set_destination(dest_x, dest_y, dest_name)
            self.state.route_total_distance_m = getattr(self.carla, "current_route_total_m", 150.0)
            self.state.route_remaining_distance_m = self.state.route_total_distance_m
        else:
            self.kinematic.set_destination(dest_x, dest_y, dest_name)
        if self.serial.is_connected():
            if notify_cockpit:
                self.serial.send_route_and_state(
                    self.state.route_id, self.state.gear, self.state.target_speed_kmh, self.state.simulation_running
                )
            sync_wps = getattr(self.carla, "planned_sync_waypoints", None)
            if sync_wps:
                self.serial.send_route_waypoints(sync_wps, self.state.route_total_distance_m)
        self._dispatch_state()

    def reset_to_default_spawn(self, notify_cockpit: bool = True) -> None:
        if self.carla.connected:
            self.carla.reset_to_default_spawn()
        self.kinematic.reset_to_default_spawn()
        self.set_gear("P", notify_cockpit=False)
        self.state.actual_speed_kmh = 0.0
        self.state.simulation_running = False
        self.log("【车辆重置】自车已重置回都会立交枢纽起点 (-4.0, 24.7)，已挂入 P 挡")
        if notify_cockpit and self.serial.is_connected():
            self.serial.send_route_and_state(
                self.state.route_id, self.state.gear, self.state.target_speed_kmh, self.state.simulation_running
            )
        self._dispatch_state()

    def reset_vehicle(self, notify_cockpit: bool = True) -> None:
        self.reset_to_default_spawn(notify_cockpit=notify_cockpit)

    def set_simulation_running(self, running: bool, notify_cockpit: bool = True) -> None:
        self.kinematic.state.simulation_running = bool(running)
        status = "运行中" if self.state.simulation_running else "已暂停"
        self.log(f"仿真状态切换: {status}")
        if notify_cockpit and self.serial.is_connected():
            self.serial.send_route_and_state(
                self.state.route_id, self.state.gear, self.state.target_speed_kmh, self.state.simulation_running
            )
        self._dispatch_state()

    def toggle_simulation(self, notify_cockpit: bool = True) -> None:
        self.set_simulation_running(not self.state.simulation_running, notify_cockpit=notify_cockpit)

    # ------------------------------------------------------------- 外部后端 (CARLA)
    def connect_carla(self) -> bool:
        ok = self.carla.connect()
        self.state.carla_connected = ok
        if ok:
            self.carla.reset_to_default_spawn()
            self.log("CARLA 3D 仿真器连接成功，已生成自车并置于都会立交枢纽起点")
        else:
            self.log(f"CARLA 连接未成功: {self.carla.get_status()}")
        self._dispatch_state()
        return ok

    def disconnect_carla(self) -> None:
        self.carla.disconnect()
        self.state.carla_connected = False
        self.log("CARLA 仿真器已断开，已回退至内置运动学仿真")
        self._dispatch_state()

    # ------------------------------------------------------------- 座舱通信
    def set_cockpit_drive_mode(self, enabled: bool) -> None:
        """切换是否强制要求座舱高德导航数据驱动。"""
        self.state.cockpit_drive_mode = bool(enabled)
        mode_str = "【座舱高德驱动模式】(需高德导航报文)" if enabled else "【离线单机自主调试模式】(无需座舱)"
        self.log(f"车辆驱动模式切换: {mode_str}")
        self._dispatch_state()

    def _on_cockpit_frame(self, frame: dict) -> None:
        now = time.time()
        action = frame.get("action", "")

        # 换挡指令即时响应 (座舱下发换挡)
        g = frame.get("gear")
        if g and g.upper() in ("P", "N", "D", "R") and g.upper() != self.state.gear:
            self.log(f"【座舱换挡】收到座舱端换挡请求 -> {g.upper()} 挡")
            self.set_gear(g.upper(), notify_cockpit=False)

        # 1. 真实高德导航二进制帧 (由 protocol_5aa5 从 5A A5 原始字节解析出的 AMAP_NAVIGATION)
        if action == "AMAP_NAVIGATION":
            self.state.has_cockpit_nav = True
            self.state.cockpit_nav_last_time = now
            self.state.wait_nav_reason = ""

            ktype = frame.get("amap_key_type", 10001)
            if ktype == 12205:
                self.state.current_maneuver = "高德巡航 (待命)"
            else:
                icon = int(frame.get("navigation_icon", 1))
                limit = int(frame.get("navigation_speed_limit_kmh", 0))
                self.state.navigation_icon = icon
                if limit > 0:
                    self.state.speed_limit_kmh = limit

                amap_actions = {
                    1: ("直行", "off"),
                    2: ("左转", "left"),
                    3: ("右转", "right"),
                    4: ("左前方行驶", "left"),
                    5: ("右前方行驶", "right"),
                    6: ("左后方行驶/调头", "left"),
                    7: ("右后方行驶", "right"),
                    8: ("调头", "left"),
                    9: ("靠左行驶", "left"),
                    10: ("靠右行驶", "right"),
                    11: ("进入环岛", "left"),
                    12: ("驶出环岛", "right"),
                }
                action_desc, turn_sig = amap_actions.get(icon, ("导航行驶", "off"))
                seg_m = self.state.segment_remaining_distance_m
                self.state.current_maneuver = f"高德: {action_desc} ({seg_m:.0f}m)"
                if seg_m <= 80.0:
                    self.state.turn_signal = turn_sig
                else:
                    self.state.turn_signal = "off"

        # 2. 模拟器状态帧 (下位机 SimulatorActivity 广播的 SIMULATOR_STATE)
        elif action == "SIMULATOR_STATE":
            self.state.has_cockpit_nav = True
            self.state.cockpit_nav_last_time = now
            self.state.wait_nav_reason = ""

            if "speed_limit_kmh" in frame:
                self.state.speed_limit_kmh = int(frame["speed_limit_kmh"])
            if "current_maneuver" in frame:
                self.state.current_maneuver = str(frame["current_maneuver"])
            if "navigation_icon" in frame:
                self.state.navigation_icon = int(frame["navigation_icon"])
            if "simulation_running" in frame:
                in_running = bool(frame["simulation_running"])
                if in_running != self.state.simulation_running:
                    # 过滤下位机误报的提前终点停止 (若上位机实际剩余距离仍大于 5 米，且下位机上报终点图标 12，忽略误报)
                    if not in_running and self.state.route_remaining_distance_m > 5.0 and frame.get("navigation_icon") == 12:
                        self.log(f"【座舱状态过滤】忽略座舱提前误报的终点暂停 (实际剩余里程仍有 {self.state.route_remaining_distance_m:.1f}m)")
                    else:
                        self.state.simulation_running = in_running
                        self.log(f"【座舱状态同步】仿真状态 -> {'运行中' if self.state.simulation_running else '已暂停'}")

            if "lane_change_request" in frame:
                lc_req = str(frame["lane_change_request"]).lower()
                if lc_req in ("left", "right", "none") and lc_req != self.state.lane_change_request:
                    self.state.lane_change_request = lc_req
                    if lc_req in ("left", "right"):
                        self.state.lane_change_status = "requested"
                        self.state.turn_signal = lc_req
                        self.log(f"【座舱变道指引】收到高德车道指引变道请求 -> 向{('左' if lc_req == 'left' else '右')}变道")

            # 动态目的地解析与响应 (座舱地图点击选点或 POI 选点)
            dest_x = frame.get("destination_x")
            dest_y = frame.get("destination_y")
            dest_name = frame.get("destination_name", "")
            if dest_x is not None and dest_y is not None:
                try:
                    dx = float(dest_x)
                    dy = float(dest_y)
                    prev_dx = getattr(self, "_current_dest_x", None)
                    prev_dy = getattr(self, "_current_dest_y", None)
                    if prev_dx is None or abs(dx - prev_dx) > 1.0 or abs(dy - prev_dy) > 1.0:
                        self._current_dest_x = dx
                        self._current_dest_y = dy
                        self.log(f"【座舱选点同步】收到座舱端动态目标 -> {dest_name} ({dx:.1f}, {dy:.1f})")
                        self.set_destination(dx, dy, dest_name, notify_cockpit=False)
                except (ValueError, TypeError):
                    pass

            # 目标车速即时响应
            in_tgt_spd = frame.get("target_speed_kmh")
            if in_tgt_spd is not None:
                try:
                    spd_val = float(in_tgt_spd)
                    if 0 <= spd_val <= 120 and abs(spd_val - self.state.target_speed_kmh) >= 1:
                        self.set_target_speed(spd_val, notify_cockpit=False)
                except (ValueError, TypeError):
                    pass

        # 定期 (每 10.0 秒) 或导航动作变更时打印一条座舱高德通信报文心跳，避免日志刷屏
        m_curr = f"{self.state.current_maneuver}_{self.state.navigation_icon}"
        m_prev = getattr(self, "_last_logged_maneuver", "")
        if (now - self._last_cockpit_log_time >= 10.0) or (m_curr != m_prev):
            self._last_cockpit_log_time = now
            self._last_logged_maneuver = m_curr
            seg_m = self.state.segment_remaining_distance_m
            route_m = self.state.route_remaining_distance_m
            spd = self.state.actual_speed_kmh
            gear = self.state.gear
            limit = self.state.speed_limit_kmh
            maneuver = self.state.current_maneuver
            self.log(f"【高德座舱通信】接收高德报文 · {maneuver} (限速: {limit} km/h · 距路口: {seg_m:.0f}m · 全程: {route_m:.0f}m) · 车速 {spd:.1f} km/h [{gear}挡]")

    def _on_serial_status(self, msg: str, ok: bool) -> None:
        self.log(f"串口状态: {msg}")

    # ------------------------------------------------------------- 视觉感知域 UDP 桥接
    def _start_udp_server(self) -> None:
        try:
            self._udp_server_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._udp_server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            # 绑定 0.0.0.0，不仅支持本机 127.0.0.1，还支持 RK3568 板卡等局域网跨机 UDP 通信
            self._udp_server_sock.bind(("0.0.0.0", 8888))
            self._udp_server_sock.settimeout(0.2)
            self._udp_thread = threading.Thread(target=self._udp_listen_loop, daemon=True)
            self._udp_thread.start()
            self.log("【视觉感知链路】UDP 执行器监听已就绪 (0.0.0.0:8888，支持本地与 RK3568 板卡)")
        except Exception as e:
            self.log(f"【视觉感知链路】端口 8888 启动异常: {e}")

    def _udp_listen_loop(self) -> None:
        last_aeb_logged = False
        last_light_logged = ""
        last_brake_request = 0.0
        last_full_aeb_request = 0.0
        last_brake_diag_time = 0.0
        last_logged_brake_reason = ""
        # 视觉停车指令的防抖器（见 VisionStopDebouncer 的说明）
        stop_deb = VisionStopDebouncer(self.VISION_STOP_CONFIRM_FRAMES,
                                       self.VISION_STOP_RELEASE_FRAMES)
        # ICD §6.2 给的缓冲是 0.8 s，但实测 0.8 s 太短：AEB 一松就立刻恢复油门，
        # 会和随后的制动请求交替出现，表现为车速前后窜动 + 刹车灯一闪一闪。
        # 这里保留原实现的经验值 1.2 s —— 它是"不小于 ICD 下限"的工程折中。
        brake_release_grace_s = 1.2
        while self._running:
            if not self._udp_server_sock:
                break
            try:
                data, addr = self._udp_server_sock.recvfrom(65536)
            except socket.timeout:
                # 检查超时 (超过 2.5 秒未收到报文判定为断开)
                if self.state.vision_udp_connected and (time.time() - self.state.vision_udp_last_time > 2.5):
                    self.state.vision_udp_connected = False
                    self.log(f"【视觉智驾链路】[告警] 智驾系统 UDP 连接已断开 ({self.state.vision_udp_endpoint})")
                    self._dispatch_state()
                continue
            except OSError:
                break

            now_recv = time.time()
            if not self.state.vision_udp_connected:
                self.state.vision_udp_connected = True
                self.log(f"【视觉智驾链路】[连接] 成功连接智驾系统 UDP 节点 -> {addr[0]}:{addr[1]}")
                self._dispatch_state()
            self.state.vision_udp_endpoint = f"{addr[0]}:{addr[1]}"
            self.state.vision_udp_last_time = now_recv
            self._last_adas_ip = addr[0]

            try:
                req = json.loads(data.decode("utf-8"))
            except Exception:
                continue

            # Heartbeats indicate that the ADAS process is alive, not a new
            # actuator command. In particular, never infer vehicle state from them.
            if req.get("message_type") == "heartbeat":
                continue

            action = str(req.get("action", ""))
            full_aeb_request = bool(req.get("aeb_triggered", False))
            try:
                brake_pressure = float(req.get("brake_pressure") or 0.0)
            except (TypeError, ValueError):
                brake_pressure = 0.0
            request_braking = full_aeb_request or brake_pressure > 0.0
            if request_braking:
                last_brake_request = now_recv
            if full_aeb_request:
                last_full_aeb_request = now_recv
            braking = request_braking or (
                last_brake_request > 0.0 and now_recv - last_brake_request < brake_release_grace_s)
            aeb = full_aeb_request or (
                last_full_aeb_request > 0.0 and
                now_recv - last_full_aeb_request < brake_release_grace_s)
            veto = str(req.get("veto", ""))
            v_speed = req.get("vehicle_speed")
            steer = req.get("steering_angle")
            lane_change_state = str(req.get("lane_change_state") or "").lower()

            self.state.vision_action = action
            self.state.vision_aeb_active = aeb

            # 1. 行人 / 障碍物紧急制动让行与驻停状态诊断
            cur_brake_reason = veto or action or "执行器制动介入"
            if braking:
                if not last_aeb_logged:
                    last_aeb_logged = True
                    last_logged_brake_reason = cur_brake_reason
                    last_brake_diag_time = now_recv
                    self.log(f"【视觉安全避险】[制动] 视觉制动介入！阻停原因: {cur_brake_reason}")
                elif cur_brake_reason != last_logged_brake_reason:
                    last_logged_brake_reason = cur_brake_reason
                    last_brake_diag_time = now_recv
                    self.log(f"【视觉安全避险】[注意] 阻停目标更新: {cur_brake_reason}")
                elif now_recv - last_brake_diag_time >= 5.0 and self.state.actual_speed_kmh <= 0.5:
                    last_brake_diag_time = now_recv
                    self.log(f"【驻停状态诊断】保持停车制动中 · 核心阻停原因: {cur_brake_reason}")
            else:
                if last_aeb_logged:
                    last_aeb_logged = False
                    last_logged_brake_reason = ""
                    self.log("【视觉安全避险】障碍排除/安全确认，解除视觉制动，恢复自主巡航。")

            # 2. 交通信号灯与停止线多源融合控制 (OIB 端中枢融合：高德导航路口距离 + 视觉停止线/红绿灯)
            is_red = ("红灯" in action)
            is_green = ("绿灯" in action)
            is_yield = ("让行" in action)

            # A. 右转路口豁免判定：若高德导航当前明确指示右转，右转车道不强制受红灯阻停
            nav_icon = int(getattr(self.state, "navigation_icon", 0) or 0)
            nav_maneuver = str(getattr(self.state, "current_maneuver", "") or "")
            is_right_turn = (nav_icon in (3, 7, 10) or "右转" in nav_maneuver)

            if is_red and is_right_turn:
                # 右转车道免受红灯拦截，由曲率和常规弯道限速管理
                is_red = False

            vision_stop = stop_deb.update(is_red or is_yield)

            if braking:
                self.state.vision_speed_override = 0.0
            elif vision_stop:
                # B. OIB 多源融合判定距停止线剩余物理距离 (stop_dist)
                vis_stop_line = float(req.get("stop_line_m") or 0.0)
                nav_dist = float(getattr(self.state, "segment_remaining_distance_m", 0.0) or 0.0)
                vis_light_dist = float(req.get("light_distance_m") or 0.0)

                stop_dist = None
                # 优先级 1: 视觉近场检测到的地面停止线物理距离 (毫米级对齐地面标线)
                if 0.5 <= vis_stop_line <= 25.0:
                    stop_dist = vis_stop_line
                # 优先级 2: 高德导航广播的距路口停止线距离 (中远距离宏观先验)
                elif 0.5 <= nav_dist <= 120.0:
                    stop_dist = nav_dist
                # 优先级 3: 视觉信号灯物理测距 (信号灯通常挂在十字路口远端对面，减去路口跨度约 16m)
                elif vis_light_dist > 18.0:
                    stop_dist = vis_light_dist - 16.0
                elif 0.0 < vis_light_dist <= 18.0:
                    stop_dist = vis_light_dist

                # C. 两阶段平滑控车：远距离舒适减速滑向路口，临近停止线 (<= 1.8m) 刹停
                stop_margin = 1.8  # 停止线前 1.8 米安全裕量
                if stop_dist is not None:
                    usable = stop_dist - stop_margin
                    if usable <= 0.0:
                        # 阶段 2: 已进入停止线裕量区，完全刹停并保持驻车制动
                        self.state.vision_speed_override = 0.0
                        label = "RED_STOP" if is_red else "YIELD"
                        if last_light_logged != label:
                            last_light_logged = label
                            self.log(f"【OIB多源融合】已抵达路口停止线前 (距线 {stop_dist:.1f}m)，保持驻车等待绿灯。"
                                     if is_red else
                                     "【OIB多源融合】识别到让行指令，停车让行。")
                    else:
                        # 阶段 1: 远距离平滑舒适减速滑向路口 (舒适减速度 a=1.8 m/s^2)
                        v_smooth = math.sqrt(2.0 * 1.8 * usable) * 3.6
                        max_limit = float(getattr(self.state, "speed_limit_kmh", 35.0) or 35.0)
                        self.state.vision_speed_override = max(6.0, min(max_limit, v_smooth))
                        if last_light_logged != "RED_SLOW":
                            last_light_logged = "RED_SLOW"
                            self.log(f"【OIB多源融合】监测到前方路口红灯 (距停止线约 {stop_dist:.1f}m)，平滑减速至 {self.state.vision_speed_override:.0f}km/h 滑行进站...")
                else:
                    # 若测距暂未就位，以安全平滑缓行车速 (15km/h) 接近路口
                    if v_speed is not None and float(v_speed) > 1.0:
                        self.state.vision_speed_override = float(v_speed)
                    else:
                        self.state.vision_speed_override = 15.0
                    if last_light_logged != "RED_SLOW":
                        last_light_logged = "RED_SLOW"
                        self.log(f"【OIB多源融合】识别到红灯，以安全车速 {self.state.vision_speed_override:.0f}km/h 缓行接近路口...")
            else:
                if last_light_logged in ("RED_STOP", "RED_SLOW", "YIELD"):
                    self.log("【OIB多源融合】识别到路口绿灯亮起！解除红灯制动，恢复自主巡航通行。"
                             if is_green else
                             "【OIB多源融合】前方红灯/让行解除，恢复自主巡航通行。")
                last_light_logged = "GREEN" if is_green else ""

                if v_speed is not None and float(v_speed) > 5.0 and ("限速" in action or "减速" in action):
                    self.state.vision_speed_override = float(v_speed)
                else:
                    self.state.vision_speed_override = None

            # 3. 变道状态同步与舵角偏置注入
            if lane_change_state in ("executing", "complete", "blocked"):
                self.state.lane_change_status = lane_change_state
                if lane_change_state == "complete":
                    self.state.lane_change_request = "none"
                    self.state.lane_change_status = "idle"
                    self.state.vision_steer_bias = 0.0
                    self.log("【变道执行完毕】车辆已平稳切入目标车道，恢复自主巡航。")
                elif lane_change_state == "blocked":
                    self.state.lane_change_request = "none"
                    self.state.lane_change_status = "idle"
                    self.state.vision_steer_bias = 0.0
                    self.log("【变道受阻拦截】视觉检测到实线或侧后方障碍物，本次变道已安全中止。")
            if (lane_change_state == "executing"
                    and self.state.lane_change_request in ("left", "right")
                    and steer is not None):
                # ICD §6.3：超出 ±12° 由仲裁端**钳位**，而不是把整帧丢掉
                self.state.vision_steer_bias = max(-12.0, min(12.0, float(steer)))
            else:
                self.state.vision_steer_bias = 0.0

    def _broadcast_loop(self) -> None:
        """ICD §4.2：以固定 20 Hz 独立广播车辆状态快照。

        单开线程的原因见 `__init__` 的注释：物理循环约 20.5 ms 的节拍会把
        "至少 50 ms" 的门限量化成 3 拍一次（~16 Hz），达不到 ICD 要求。
        这里用绝对时间点累加，避免 sleep 误差逐帧累积。
        """
        period = 1.0 / self.VISION_BROADCAST_HZ
        next_at = time.perf_counter()
        while self._running:
            next_at += period
            delay = next_at - time.perf_counter()
            if delay > 0:
                time.sleep(min(delay, period))
            if not self._running:
                break
            self._broadcast_vision_state()

    def _broadcast_vision_state(self) -> None:
        """向视觉智驾域 (UDP 端口 8892) 广播当前权威车辆状态与高德意图快照。"""
        now = time.time()
        payload = {
            "type": "OIB_DOMAIN_STATE",
            "stamp": round(now, 3),
            "vehicle": {
                "speed": round(self.state.actual_speed_kmh, 1),
                "gear": self.state.gear,
                "steer": round(self.state.steering_angle, 1),
                "aeb": self.state.vision_aeb_active,
                "pedestrian_detected": getattr(self.state, "pedestrian_detected", False),
                "pedestrian_distance_m": round(getattr(self.state, "pedestrian_distance_m", 0.0), 1),
                "pedestrian_lateral_m": round(getattr(self.state, "pedestrian_lateral_m", 0.0), 2),
            },
            "navigation": {
                "active": self.state.simulation_running,
                "icon": self.state.navigation_icon,
                "maneuver": self.state.current_maneuver,
                # ICD §4.2.1：这里是**高德/路线模型推荐的**转角，不是车辆当前转角
                "steer_deg": round(self.state.nav_steer_deg, 1),
                "limit_kmh": int(self.state.speed_limit_kmh),
                "segment_distance_m": round(self.state.segment_remaining_distance_m, 1),
                "route_distance_m": round(self.state.route_remaining_distance_m, 1),
                "lane_change_request": self.state.lane_change_request,
                "lane_change_status": self.state.lane_change_status,
            },
            "health": {
                # ICD §4.2.1：三项都必须是**真实链路状态**，不能硬编码 True
                "uart": self.serial.is_connected(),
                "adas": self.state.vision_udp_connected,
                "navigation": self.state.has_cockpit_nav,
            },
            "perception_source": getattr(self, "_perception_source", "pcie"),
            "takeover_enabled": getattr(self.state, "takeover_enabled", False),
        }
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        try:
            self._udp_client_sock.sendto(data, ("127.0.0.1", 8892))
            remote_ip = getattr(self, "_last_adas_ip", None) or "192.168.10.2"
            if remote_ip and remote_ip != "127.0.0.1":
                self._udp_client_sock.sendto(data, (remote_ip, 8892))
        except Exception:
            pass

    # ------------------------------------------------------------- 循环推算
    def _on_kinematic_tick(self, state: VehicleState) -> None:
        pass

    def _physics_loop(self) -> None:
        last = time.perf_counter()
        while self._running:
            now = time.perf_counter()
            dt = now - last
            last = now
            if self.carla.connected:
                self.carla.apply_control(self.state)
            else:
                self.kinematic.step(dt)
            self._dispatch_state()

            # 8892 快照广播已挪到独立线程（见 _broadcast_loop），固定 20 Hz

            # 以 5Hz 向树莓派/KICKPI座舱下发实车真实瞬时速度、里程、挡位、位姿与行人预警联动
            if now - self._last_telemetry_time >= 0.2:
                self._last_telemetry_time = now
                if self.serial.is_connected():
                    self.serial.send_telemetry(
                        self.state.gear,
                        self.state.actual_speed_kmh,
                        self.state.target_speed_kmh,
                        self.state.segment_remaining_distance_m,
                        self.state.route_remaining_distance_m,
                        route_id=self.state.route_id,
                        route_name=self.state.route_name,
                        simulation_running=self.state.simulation_running,
                        pos_x=self.state.pos_x,
                        pos_y=self.state.pos_y,
                        heading_deg=round(math.degrees(self.state.heading_rad), 1),
                        pedestrian_detected=getattr(self.state, "pedestrian_detected", False),
                        pedestrian_distance_m=getattr(self.state, "pedestrian_distance_m", 0.0),
                        aeb_active=bool(self.state.vision_aeb_active),
                    )

            time.sleep(0.02)  # ~50Hz

    def _dispatch_state(self) -> None:
        self.state.serial_connected = self.serial.is_connected()
        self.state.serial_port_name = self.serial.port or ""
        now = time.time()
        if not self.state.serial_connected:
            self.state.has_cockpit_nav = False
            self.state.wait_nav_reason = "等待座舱串口连接"
        elif not self.state.has_cockpit_nav:
            self.state.wait_nav_reason = "等待高德导航广播"
        elif now - self.state.cockpit_nav_last_time > 10.0:
            # 导航报文超过 10.0 秒未刷新即视为链路中断，立即触发安全停车
            self.state.has_cockpit_nav = False
            self.state.wait_nav_reason = "高德导航数据已中断"

        if self.on_state_change:
            try:
                self.on_state_change(self.state)
            except Exception:
                pass

    def close(self) -> None:
        self._running = False
        if self._udp_server_sock:
            try: self._udp_server_sock.close()
            except Exception: pass
            self._udp_server_sock = None
        try: self._udp_client_sock.close()
        except Exception: pass
        self.serial.disconnect()
        self.carla.disconnect()
