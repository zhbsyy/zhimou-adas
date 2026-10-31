"""高拟真阿克曼运动学仿真后端 (Kinematic Engine)。

基于阿克曼转向双轨/自行车模型 (Bicycle Model)，提供 60 FPS 物理微分积分：
1. 默认出生点：都会立交枢纽 (-4.0, 24.7)
2. 动态自主寻路模式：
   - 响应座舱地图选点，根据当前位置与目标点动态计算距离、航向与路口指引
3. 挡位与动力规则：
   - P 挡：实际速度强制为 0，手刹拉紧
   - N 挡：实际速度滑行归 0，动力切断
   - D 挡：正向行驶，向目标速度平滑加减速
   - R 挡：反向行驶，速度符号反转，倒车灯点亮
4. 视觉感知与底盘接管安全逻辑：
   - 默认未开启接管/未接入视觉时：不自动报警、不自主刹车，按巡航车速直接行驶通过
   - 开启底盘动力学接管且收到外部视觉 AEB 指令时：执行安全避险制动
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Callable, Optional

# 默认出生点：都会立交枢纽
DEFAULT_SPAWN = {
    "x": -4.0,
    "y": 24.7,
    "z": 0.6,
    "pitch": 0.0,
    "yaw": 0.2,
    "roll": 0.0,
}



@dataclass
class VehicleState:
    gear: str = "P"
    target_speed_kmh: float = 40.0
    actual_speed_kmh: float = 0.0
    speed_limit_kmh: int = 40
    steering_angle: float = 0.0
    nav_steer_deg: float = 0.0
    turn_signal: str = "off"  # off / left / right / hazard
    brake_lamp: bool = False
    reverse_lamp: bool = False
    route_id: str = "dynamic_nav"
    route_name: str = "都会立交 (待命)"
    route_shape: str = "dynamic"
    route_total_distance_m: float = 0.0
    segment_total_distance_m: float = 0.0
    segment_remaining_distance_m: float = 0.0
    route_remaining_distance_m: float = 0.0
    current_maneuver: str = "待命: 等待座舱选择目的地"
    navigation_icon: int = 1
    simulation_running: bool = True
    carla_connected: bool = False

    pos_x: float = -4.0
    pos_y: float = 24.7
    heading_rad: float = 0.0

    # 视觉感知域闭环介入标记
    vision_aeb_active: bool = False
    vision_speed_override: Optional[float] = None
    vision_steer_bias: float = 0.0
    vision_action: str = ""
    lane_change_request: str = "none"  # none / left / right
    lane_change_status: str = "idle"   # idle / requested / executing / complete / blocked

    # 行人场景真实遥测 (Ground Truth)
    pedestrian_detected: bool = False
    pedestrian_distance_m: float = 0.0
    pedestrian_lateral_m: float = 0.0

    # 外部硬件链路状态
    serial_connected: bool = False
    serial_port_name: str = ""
    vision_udp_connected: bool = False
    vision_udp_endpoint: str = ""
    vision_udp_last_time: float = 0.0

    # 智驾闭环运行模式与接管开关
    system_mode: str = "cockpit_sim"  # cockpit_sim / perception_debug / full_ad
    takeover_enabled: bool = False
    perception_source: str = "pcie"

    # 座舱高德联动驱动控制
    cockpit_drive_mode: bool = False
    has_cockpit_nav: bool = False
    cockpit_nav_last_time: float = 0.0
    wait_nav_reason: str = "等待座舱串口连接"

    def to_dict(self) -> dict:
        return {
            "gear": self.gear,
            "target_speed_kmh": round(self.target_speed_kmh, 1),
            "actual_speed_kmh": round(self.actual_speed_kmh, 1),
            "speed_limit_kmh": int(self.speed_limit_kmh),
            "steering_angle": round(self.steering_angle, 1),
            "turn_signal": self.turn_signal,
            "brake_lamp": self.brake_lamp,
            "reverse_lamp": self.reverse_lamp,
            "route_id": self.route_id,
            "route_name": self.route_name,
            "route_shape": self.route_shape,
            "route_total_distance_m": round(self.route_total_distance_m, 1),
            "segment_total_distance_m": round(self.segment_total_distance_m, 1),
            "segment_remaining_distance_m": round(self.segment_remaining_distance_m, 1),
            "route_remaining_distance_m": round(self.route_remaining_distance_m, 1),
            "current_maneuver": self.current_maneuver,
            "navigation_icon": self.navigation_icon,
            "simulation_running": self.simulation_running,
            "carla_connected": self.carla_connected,
            "lane_change_request": self.lane_change_request,
            "lane_change_status": self.lane_change_status,
            "pedestrian_detected": self.pedestrian_detected,
            "pedestrian_distance_m": round(self.pedestrian_distance_m, 1),
            "pedestrian_lateral_m": round(self.pedestrian_lateral_m, 2),
            "pos_x": round(self.pos_x, 2),
            "pos_y": round(self.pos_y, 2),
            "heading_deg": round(math.degrees(self.heading_rad), 1),
        }


class KinematicSimulator:
    """高拟真运动学仿真引擎。"""

    BRAKE_LAMP_HOLD_S = 0.4

    def __init__(self,
                 on_update: Optional[Callable[[VehicleState], None]] = None,
                 on_arrival: Optional[Callable[[], None]] = None) -> None:
        self.state = VehicleState()
        self.on_update = on_update
        self.on_arrival = on_arrival
        self._last_time = time.perf_counter()
        self._brake_lamp_until = 0.0
        self._distance_traveled = 0.0
        self.wheelbase = 2.85  # 轴距 2.85m
        self._has_auto_parked = False
        self._target_dest_x = 98.8
        self._target_dest_y = 82.9
        self.reset_to_default_spawn()

    def reset_to_default_spawn(self) -> None:
        """重置自车到默认起点（都会立交枢纽）。"""
        self.state.pos_x = DEFAULT_SPAWN["x"]
        self.state.pos_y = DEFAULT_SPAWN["y"]
        self.state.heading_rad = math.radians(DEFAULT_SPAWN["yaw"])
        self.state.gear = "P"
        self.state.actual_speed_kmh = 0.0
        self.state.target_speed_kmh = 40.0
        self.state.speed_limit_kmh = 40
        self.state.route_id = "dynamic_nav"
        self.state.route_name = "都会立交枢纽 (待命)"
        self.state.route_total_distance_m = 0.0
        self.state.segment_total_distance_m = 0.0
        self.state.segment_remaining_distance_m = 0.0
        self.state.route_remaining_distance_m = 0.0
        self._distance_traveled = 0.0
        self.state.current_maneuver = "待命: 请在座舱选择目的地"
        self.state.navigation_icon = 1
        self.state.turn_signal = "off"
        self.state.brake_lamp = True
        self._has_auto_parked = False
        self.state.lane_change_request = "none"
        self.state.lane_change_status = "idle"
        self.state.pedestrian_detected = False

    def set_destination(self, dest_x: float, dest_y: float, dest_name: str = "") -> None:
        """设置动态导航目标点并规划路径。"""
        self._target_dest_x = float(dest_x)
        self._target_dest_y = float(dest_y)
        dist = math.hypot(self._target_dest_x - self.state.pos_x, self._target_dest_y - self.state.pos_y)
        self.state.route_total_distance_m = max(dist, 10.0)
        self.state.route_remaining_distance_m = self.state.route_total_distance_m
        self.state.segment_total_distance_m = min(35.0, self.state.route_total_distance_m)
        self.state.segment_remaining_distance_m = self.state.segment_total_distance_m
        self._distance_traveled = 0.0
        self.state.route_id = "dynamic_nav"
        self.state.route_name = dest_name or f"目标点 ({dest_x:.1f}, {dest_y:.1f})"
        self.state.current_maneuver = f"直行前往 {self.state.route_name}"
        self.state.navigation_icon = 1
        self._has_auto_parked = False

    def reset(self) -> None:
        self.reset_to_default_spawn()

    def set_gear(self, gear: str) -> None:
        g = str(gear).upper()
        if g in ("P", "N", "D", "R"):
            self.state.gear = g
            self.state.reverse_lamp = (g == "R")
            if g in ("P", "N"):
                self.state.actual_speed_kmh = 0.0
            if g == "D":
                self._has_auto_parked = False

    def set_target_speed(self, speed_kmh: float) -> None:
        self.state.target_speed_kmh = max(0.0, min(80.0, float(speed_kmh)))

    def toggle_running(self) -> None:
        self.state.simulation_running = not self.state.simulation_running

    def step(self, dt: float) -> VehicleState:
        """单步物理推算 (典型 dt = 0.016s ~ 0.02s)。"""
        if dt <= 0 or dt > 0.5:
            dt = 0.016

        # 座舱高德联动门禁
        if self.state.cockpit_drive_mode:
            now = time.time()
            has_nav = (self.state.serial_connected and self.state.has_cockpit_nav and (now - self.state.cockpit_nav_last_time < 10.0))
            if not has_nav:
                self.state.actual_speed_kmh = 0.0
                self.state.brake_lamp = True
                self.state.steering_angle = 0.0
                self.state.wait_nav_reason = "等待座舱串口连接" if not self.state.serial_connected else "等待座舱导航广播"
                self.state.current_maneuver = "待命: 未连接座舱" if not self.state.serial_connected else "待命: 等待座舱导航"
                return self.state
            else:
                self.state.wait_nav_reason = ""

        gear = self.state.gear
        running = self.state.simulation_running
        cruise_v = self.state.target_speed_kmh if (running and gear in ("D", "R")) else 0.0
        full_route = self.state.route_total_distance_m
        current_route_remain = max(0.0, full_route - self._distance_traveled)

        effective_target_v = cruise_v
        target_steer = 0.0
        self.state.nav_steer_deg = 0.0

        if full_route <= 0.0:
            effective_target_v = 0.0
            self.state.current_maneuver = "待命: 请在座舱地图选择目的地"
            self.state.segment_remaining_distance_m = 0.0
            self.state.route_remaining_distance_m = 0.0
        elif current_route_remain <= 1.0:
            effective_target_v = 0.0
            self.state.segment_remaining_distance_m = 0.0
            self.state.route_remaining_distance_m = 0.0
            if not self._has_auto_parked and gear == "D":
                self._has_auto_parked = True
                self.set_gear("P")
                self.state.current_maneuver = f"已到达目的地 [{self.state.route_name}] (P档已驻车)"
                self.state.navigation_icon = 1
                self.state.turn_signal = "off"
                if self.on_arrival:
                    self.on_arrival()
        elif current_route_remain <= 30.0:
            effective_target_v = max(4.0, cruise_v * (current_route_remain / 30.0))
            self.state.current_maneuver = "即将到达目的地 (减速中)"
            self.state.navigation_icon = 1
            self.state.turn_signal = "off"
            self.state.segment_remaining_distance_m = max(0.0, current_route_remain)
        else:
            self.state.segment_remaining_distance_m = min(35.0, current_route_remain)
            effective_target_v = cruise_v
            self.state.turn_signal = "off"
            self.state.current_maneuver = f"直行前往 {self.state.route_name}"
            self.state.navigation_icon = 1

        # 视觉感知域闭环覆盖 (仅在底盘接管开启时生效)
        if getattr(self.state, "takeover_enabled", False):
            if self.state.vision_aeb_active:
                effective_target_v = 0.0
                self.state.brake_lamp = True
                self.state.current_maneuver = f"视觉AEB紧急制动: {self.state.vision_action or '前方避障'}"
            elif self.state.vision_speed_override is not None:
                effective_target_v = min(effective_target_v, self.state.vision_speed_override)
                if self.state.vision_speed_override == 0.0:
                    self.state.current_maneuver = f"视觉感知驻车: {self.state.vision_action}"
            if self.state.vision_steer_bias != 0.0:
                target_steer = max(-35.0, min(35.0, target_steer + self.state.vision_steer_bias))
        else:
            self.state.pedestrian_detected = False

        # 纵向加减速计算
        accel_rate = 14.0
        decel_rate = 32.0 if self.state.vision_aeb_active else 24.0
        cur_v = self.state.actual_speed_kmh

        if self.state.gear in ("P", "N"):
            cur_v = 0.0
            self.state.brake_lamp = False
        else:
            if cur_v < effective_target_v:
                cur_v = min(effective_target_v, cur_v + accel_rate * dt)
                braking_now = False
            elif cur_v > effective_target_v:
                cur_v = max(effective_target_v, cur_v - decel_rate * dt)
                braking_now = (cur_v > 0.5)
            else:
                braking_now = False

            if braking_now:
                self._brake_lamp_until = time.perf_counter() + self.BRAKE_LAMP_HOLD_S
                self.state.brake_lamp = True
            else:
                self.state.brake_lamp = time.perf_counter() < self._brake_lamp_until

        self.state.actual_speed_kmh = cur_v
        speed_ms = cur_v / 3.6
        gear_sign = 1.0 if self.state.gear == "D" else (-1.0 if self.state.gear == "R" else 0.0)

        # 里程与位姿更新
        ds = gear_sign * speed_ms * dt
        self._distance_traveled += ds
        self.state.route_remaining_distance_m = max(0.0, full_route - self._distance_traveled)

        # 朝向目标点的朝向微调
        dx = self._target_dest_x - self.state.pos_x
        dy = self._target_dest_y - self.state.pos_y
        if math.hypot(dx, dy) > 2.0:
            target_yaw = math.atan2(dy, dx)
            diff_yaw = (target_yaw - self.state.heading_rad + math.pi) % (2 * math.pi) - math.pi
            self.state.heading_rad += diff_yaw * min(1.0, dt * 2.5)

        self.state.pos_x += ds * math.cos(self.state.heading_rad)
        self.state.pos_y += ds * math.sin(self.state.heading_rad)

        if self.on_update:
            self.on_update(self.state)

        return self.state
