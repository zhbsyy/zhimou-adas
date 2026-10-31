"""CARLA 引擎适配与实时动态导航控制后端。

支持通过 CARLA Python-API (carla.Client) 连接 CARLA 仿真服务器：
1. 默认出生点：都会立交枢纽 (-4.0, 24.7, 0.6, yaw=0.2)
2. 动态自主寻路 (GlobalRoutePlanner)：
   - 响应座舱地图任意选点 / POI，以自车当前位置为起点动态规划全局路径
   - 毫秒级生成精确的车道路网参考轨迹
3. 动态随机行人场景：
   - 规划路线后，在路线上随机选点生成横穿行人
   - 未开启底盘接管 / 无视觉 AEB 时，车辆不主动报警、不私自制动，按目标车速直接开过
   - 开启底盘动力学接管且收到外部视觉 AEB 紧急制动指令时，执行毫秒级紧急制动
4. 动力与控制执行律：
   - Pure Pursuit 纯跟踪车道居中与转向控制
   - 弯道曲率自适应减速 (15 km/h 稳速通过)，出弯平滑恢复巡航
   - 到达终点平顺减速并自动挂入 P 挡锁止手刹
5. 第三人称视口平滑缓动跟随与车灯同步
"""

from __future__ import annotations

import math
import os
import random
import sys
import threading
import time
from typing import Callable, Optional

from .kinematic_backend import VehicleState

# 动态探测并加载 CARLA PythonAPI 路径
for _cand in [
    "F:/CARLA_0.9.16/PythonAPI/carla",
    "C:/CARLA_0.9.16/PythonAPI/carla",
    os.path.join(os.environ.get("CARLA_ROOT", ""), "PythonAPI", "carla"),
]:
    if os.path.isdir(_cand) and _cand not in sys.path:
        sys.path.append(_cand)

try:
    import carla
    CARLA_AVAILABLE = True
except ImportError:
    carla = None
    CARLA_AVAILABLE = False

try:
    from agents.navigation.global_route_planner import GlobalRoutePlanner
except ImportError:
    GlobalRoutePlanner = None


class _WaypointStub:
    """平滑变道虚拟路点包装，严格兼容 CARLA Waypoint 的 transform 属性接口。"""
    __slots__ = ("transform",)

    def __init__(self, transform) -> None:
        self.transform = transform


class CarlaBackend:
    """CARLA 仿真器高阶动态自主寻路与智驾控制后端。"""

    # 默认自车出生点：都会立交枢纽
    DEFAULT_SPAWN = {
        "x": -4.0,
        "y": 24.7,
        "z": 0.6,
        "pitch": 0.0,
        "yaw": 0.2,
        "roll": 0.0,
    }

    # 目标车速变化率限幅（km/h 每秒）
    TARGET_RATE_UP_KMH_S = 15.0
    TARGET_RATE_DOWN_KMH_S = 30.0

    @classmethod
    def rate_limit_target(cls, current_kmh: float, target_kmh: float, dt: float) -> float:
        dt = max(0.005, min(0.20, float(dt)))
        rate = cls.TARGET_RATE_UP_KMH_S if target_kmh > current_kmh else cls.TARGET_RATE_DOWN_KMH_S
        step = rate * dt
        delta = float(target_kmh) - float(current_kmh)
        return float(current_kmh) + max(-step, min(step, delta))

    def __init__(self, host: str = "127.0.0.1", port: int = 2000,
                 on_arrival: Optional[Callable[[], None]] = None,
                 on_event: Optional[Callable[[str], None]] = None) -> None:
        self.host = host
        self.port = int(port)
        self.on_arrival = on_arrival
        self.on_event = on_event
        self.client = None
        self.world = None
        self.vehicle = None
        self.grp = None
        self.connected = False
        self.running = False
        self._last_error = ""

        # 动态路线轨迹与跟踪状态
        self.trajectory_waypoints: list = []
        self.trajectory_options: list = []
        self.trajectory_step = 1.8  # 米
        self.last_matched_idx = 0
        self.current_route_total_m = 0.0
        self.current_dest_name = ""
        self._has_auto_parked = False
        self._lane_change_path_switched = False

        # 平滑纵向控制与相机缓动滤波状态
        self._current_throttle = 0.0
        self._current_brake = 0.0
        self._current_steer = 0.0
        self._longitudinal_mode = "coast"
        self._brake_lamp_active = False
        self._target_v_smooth = 0.0
        self._last_control_time = 0.0
        self._cam_loc = None
        self._cam_rot = None

        # 动态随机行人场景
        self._scene_walker = None
        self._scene_crossing_done = False
        self._scene_crossing_started = False
        self._scene_walk_dir = None
        self._scene_target = None
        self._scene_last_telemetry = 0.0
        self._scene_last_aeb = False

    def is_available(self) -> bool:
        return CARLA_AVAILABLE

    def get_status(self) -> str:
        if not CARLA_AVAILABLE:
            return "未安装 carla 库 (可运行独立运动学模式)"
        if self.connected and self.vehicle is not None:
            return f"已连接 CARLA [{self.host}:{self.port}] - 自车已就绪"
        if self._last_error:
            return f"CARLA 连接失败: {self._last_error}"
        return "CARLA 引擎未启动 (已自动切换至独立高拟真运动学模式)"

    def connect(self) -> bool:
        if not CARLA_AVAILABLE:
            self._last_error = "Python 环境缺少 carla 模块"
            return False
        try:
            self.client = carla.Client(self.host, self.port)
            self.client.set_timeout(4.0)
            self.world = self.client.get_world()
            self._init_grp()
            self._spawn_vehicle()
            self.connected = True
            self._last_error = ""
            return True
        except Exception as err:
            self.connected = False
            self.vehicle = None
            self._last_error = str(err)
            return False

    def _init_grp(self) -> None:
        if self.world is not None and GlobalRoutePlanner is not None:
            try:
                self.grp = GlobalRoutePlanner(self.world.get_map(), sampling_resolution=1.8)
                self._scene_log("【自主寻路】CARLA 全局路网拓扑规划器 (GlobalRoutePlanner) 初始化就绪")
            except Exception as e:
                self.grp = None
                self._scene_log(f"【自主寻路】GlobalRoutePlanner 初始化异常: {e}")

    def _spawn_vehicle(self) -> None:
        if self.world is None:
            return
        bp_lib = self.world.get_blueprint_library()
        blueprints = bp_lib.filter("vehicle.tesla.model3") or bp_lib.filter("vehicle.audi.etron") or bp_lib.filter("vehicle.*")
        bp = blueprints[0]
        bp.set_attribute("role_name", "ego_vehicle")

        # 默认起点：都会立交枢纽
        spawn_tf = carla.Transform(
            carla.Location(x=self.DEFAULT_SPAWN["x"], y=self.DEFAULT_SPAWN["y"], z=self.DEFAULT_SPAWN["z"]),
            carla.Rotation(pitch=self.DEFAULT_SPAWN["pitch"], yaw=self.DEFAULT_SPAWN["yaw"], roll=self.DEFAULT_SPAWN["roll"])
        )

        # 销毁残留旧车
        for actor in self.world.get_actors().filter("vehicle.*"):
            if actor.attributes.get("role_name") == "ego_vehicle":
                try:
                    if getattr(actor, "is_alive", True):
                        actor.destroy()
                except Exception:
                    pass

        self.vehicle = self.world.try_spawn_actor(bp, spawn_tf)
        if self.vehicle is None:
            # 容错：稍微抬高 0.2m 避开地面碰撞
            spawn_tf.location.z += 0.2
            self.vehicle = self.world.spawn_actor(bp, spawn_tf)

        self._update_camera()
        self._scene_log("【车辆初始化】自车已在默认起点 [都会立交枢纽] 生成就绪")

    def _update_camera(self) -> None:
        """保持 CARLA 视口第三人称追尾跟随自车 (低通平滑滤波)。"""
        if self.world is None or self.vehicle is None or carla is None:
            return
        try:
            spectator = self.world.get_spectator()
            tf = self.vehicle.get_transform()
            fwd = tf.get_forward_vector()
            target_loc = tf.location - fwd * 7.5 + carla.Location(z=3.0)
            target_rot = carla.Rotation(pitch=-15.0, yaw=tf.rotation.yaw, roll=0.0)

            if self._cam_loc is None:
                self._cam_loc = target_loc
                self._cam_rot = target_rot
            else:
                alpha_pos = 0.28
                alpha_rot = 0.25
                self._cam_loc = carla.Location(
                    x=self._cam_loc.x * (1 - alpha_pos) + target_loc.x * alpha_pos,
                    y=self._cam_loc.y * (1 - alpha_pos) + target_loc.y * alpha_pos,
                    z=self._cam_loc.z * (1 - alpha_pos) + target_loc.z * alpha_pos,
                )
                diff_yaw = (target_rot.yaw - self._cam_rot.yaw + 180) % 360 - 180
                new_yaw = self._cam_rot.yaw + diff_yaw * alpha_rot
                self._cam_rot = carla.Rotation(pitch=-15.0, yaw=new_yaw, roll=0.0)

            spectator.set_transform(carla.Transform(self._cam_loc, self._cam_rot))
        except Exception:
            pass

    def _scene_log(self, message: str) -> None:
        if self.on_event:
            try:
                self.on_event(message)
            except Exception:
                pass

    def _clear_pedestrian_scene(self) -> None:
        """清理已生成的行人 actor。"""
        walker = self._scene_walker
        self._scene_walker = None
        if walker is not None:
            try:
                if getattr(walker, "is_alive", True):
                    walker.destroy()
            except Exception:
                pass
        self._scene_crossing_done = False
        self._scene_crossing_started = False
        self._scene_walk_dir = None
        self._scene_target = None
        self._scene_last_telemetry = 0.0
        self._scene_last_aeb = False

    def _spawn_dynamic_pedestrian(self, waypoint) -> bool:
        """在规划的动态路线上随机选点生成横穿行人。"""
        if self.world is None or self.vehicle is None or carla is None or waypoint is None:
            return False
        try:
            self._clear_pedestrian_scene()
            center = waypoint.transform.location
            yaw = math.radians(waypoint.transform.rotation.yaw)
            # CARLA 坐标系：右向量 (-sin, cos)，左向量 (sin, -cos)
            rgt_x, rgt_y = -math.sin(yaw), math.cos(yaw)
            left_x, left_y = math.sin(yaw), -math.cos(yaw)

            blueprints = self.world.get_blueprint_library().filter("walker.pedestrian.*")
            if not blueprints:
                self._scene_log("【动态行人】CARLA 中没有 walker.pedestrian 蓝图")
                return False
            blueprint = blueprints[0]
            if blueprint.has_attribute("is_invincible"):
                blueprint.set_attribute("is_invincible", "false")
            if blueprint.has_attribute("role_name"):
                blueprint.set_attribute("role_name", "dynamic_route_pedestrian")

            offset = 3.0  # 右侧路沿偏移 3 米
            start_pos = carla.Location(
                x=center.x + rgt_x * offset,
                y=center.y + rgt_y * offset,
                z=center.z + 0.3
            )
            target_pos = carla.Location(
                x=center.x + left_x * (offset + 1.5),
                y=center.y + left_y * (offset + 1.5),
                z=center.z + 0.3
            )
            heading = math.degrees(math.atan2(target_pos.y - start_pos.y, target_pos.x - start_pos.x))

            walker = None
            for z_lift in (0.2, 0.4, 0.6, 0.8, 1.0):
                try_loc = carla.Location(x=start_pos.x, y=start_pos.y, z=start_pos.z + z_lift)
                walker = self.world.try_spawn_actor(blueprint, carla.Transform(try_loc, carla.Rotation(yaw=heading)))
                if walker is not None:
                    break

            if walker is None:
                # 容错：生成在路点中心略偏右
                alt_loc = carla.Location(x=center.x + rgt_x * 2.0, y=center.y + rgt_y * 2.0, z=center.z + 0.5)
                walker = self.world.try_spawn_actor(blueprint, carla.Transform(alt_loc, carla.Rotation(yaw=heading)))

            if walker is None:
                self._scene_log("【动态行人】行人生成失败")
                return False

            self._scene_walker = walker
            self._scene_walk_dir = carla.Vector3D(x=left_x, y=left_y, z=0.0)
            self._scene_target = target_pos
            self._scene_crossing_started = False
            self._scene_crossing_done = False
            self._scene_last_aeb = False
            self._scene_last_telemetry = 0.0

            self._scene_log(
                f"【动态行人】已在规划路线上随机设置横穿行人：位置 ({start_pos.x:.1f}, {start_pos.y:.1f})，静候自车接近"
            )
            return True
        except Exception as e:
            self._scene_log(f"【动态行人】生成异常: {e}")
            return False

    def _update_pedestrian_scene(self, state: VehicleState, dist_traveled: float, actual_v_kmh: float) -> None:
        walker = self._scene_walker
        if walker is None:
            state.pedestrian_detected = False
            return

        try:
            now = time.monotonic()
            veh_tf = self.vehicle.get_transform()
            ped_loc = walker.get_location()
            dx = ped_loc.x - veh_tf.location.x
            dy = ped_loc.y - veh_tf.location.y
            yaw_rad = math.radians(veh_tf.rotation.yaw)
            x_rel = dx * math.cos(yaw_rad) + dy * math.sin(yaw_rad)
            y_rel = dx * math.sin(yaw_rad) - dy * math.cos(yaw_rad)
            gap = math.hypot(dx, dy)

            # 自车接近 (前向距离 <= 22m 且未开始过街)：行人启动横穿
            if not getattr(self, "_scene_crossing_started", False) and gap <= 22.0 and x_rel > 1.0:
                self._scene_crossing_started = True
                walk_spd = 1.2
                walker.apply_control(carla.WalkerControl(direction=self._scene_walk_dir, speed=walk_spd))
                self._scene_log(f"【动态行人】自车接近 (距离 {gap:.1f}m)，行人开始匀速横穿马路 (速度 {walk_spd}m/s)")

            # 横穿完成判定：行人越过车道到达左侧路缘外 (y_rel >= 2.8m 或接近目标点)
            if getattr(self, "_scene_crossing_started", False) and not self._scene_crossing_done:
                if y_rel >= 2.8 or ped_loc.distance(self._scene_target) <= 1.2:
                    self._scene_crossing_done = True
                    self._scene_log("【动态行人】行人已穿过自车车道，清理行人")
                    try:
                        walker.destroy()
                    except Exception:
                        pass
                    self._scene_walker = None
                    state.pedestrian_detected = False
                    return

            # 空间真值遥测记录 (仅记录真值，绝对不触发自动避让刹车)
            state.pedestrian_distance_m = gap
            state.pedestrian_lateral_m = y_rel

            # 只有开启底盘接管时才上报给算法端；未开接管时不报警
            if getattr(state, "takeover_enabled", False):
                state.pedestrian_detected = (not self._scene_crossing_done and gap <= 20.0 and abs(y_rel) <= 2.2)
            else:
                state.pedestrian_detected = False

            # 视觉 AEB 介入日志
            aeb = bool(state.vision_aeb_active)
            if aeb and not self._scene_last_aeb:
                self._scene_log(
                    f"【动态行人】视觉 AEB 紧急制动介入：车速 {actual_v_kmh:.1f}km/h，"
                    f"自车距行人 {gap:.1f}m (前向 {x_rel:.1f}m, 横向 {y_rel:+.1f}m)"
                )
            self._scene_last_aeb = aeb

            # 周期性真实几何测距遥测打印
            if now - self._scene_last_telemetry >= 1.0 and gap <= 30.0:
                self._scene_last_telemetry = now
                in_path = (0.0 < x_rel <= 18.0 and abs(y_rel) <= 1.8)
                self._scene_log(
                    f"【行人真值遥测】空间距离 {gap:.1f}m | 纵向前向 {x_rel:.1f}m | 横向偏置 {y_rel:+.1f}m | "
                    f"在车道内={'是' if in_path else '否'} | 底盘接管={'开启' if state.takeover_enabled else '关闭'} | 视觉制动={'介入' if aeb else '未介入'}"
                )
        except Exception:
            pass

    def set_destination(self, dest_x: float, dest_y: float, dest_name: str = "") -> bool:
        """响应座舱选点，使用 GlobalRoutePlanner 规划从自车当前位置到目标点的动态路线。"""
        if not self.connected or self.vehicle is None or carla is None:
            return False
        try:
            if self.grp is None and self.world is not None:
                self._init_grp()
            if self.grp is None:
                self._scene_log("【自主寻路】GlobalRoutePlanner 未就绪，无法规划路线")
                return False

            ego_loc = self.vehicle.get_location()
            dest_loc = carla.Location(x=float(dest_x), y=float(dest_y), z=ego_loc.z)
            route = self.grp.trace_route(ego_loc, dest_loc)
            if not route:
                self._scene_log(f"【自主寻路】未找到从 ({ego_loc.x:.1f}, {ego_loc.y:.1f}) 到 ({dest_x:.1f}, {dest_y:.1f}) 的可行路线")
                return False

            self.trajectory_waypoints = [wp for wp, opt in route]
            self.trajectory_options = [opt for wp, opt in route]

            tot_m = 0.0
            for i in range(len(self.trajectory_waypoints) - 1):
                tot_m += self.trajectory_waypoints[i].transform.location.distance(
                    self.trajectory_waypoints[i+1].transform.location)
            self.current_route_total_m = max(tot_m, 1.0)
            self.last_matched_idx = 0
            self._has_auto_parked = False
            self._lane_change_path_switched = False
            self.current_dest_name = dest_name or f"({dest_x:.1f}, {dest_y:.1f})"

            self._scene_log(
                f"【自主寻路】成功规划路线至 {self.current_dest_name}："
                f"全长 {self.current_route_total_m:.1f}m，包含 {len(self.trajectory_waypoints)} 个路网路点"
            )

            # 提取真值离散路点序列供座舱界面同步 (下采样约 40 个关键点，毫秒级传输)
            step_stride = max(1, len(self.trajectory_waypoints) // 40)
            self.planned_sync_waypoints = [
                (wp.transform.location.x, wp.transform.location.y)
                for wp in self.trajectory_waypoints[::step_stride]
            ]
            last_loc = self.trajectory_waypoints[-1].transform.location
            self.planned_sync_waypoints.append((last_loc.x, last_loc.y))

            # 动态随机设置横穿行人 (在路线约 50% ~ 70% 处随机选点)
            if len(self.trajectory_waypoints) >= 15:
                min_idx = int(len(self.trajectory_waypoints) * 0.45)
                max_idx = int(len(self.trajectory_waypoints) * 0.70)
                ped_idx = random.randint(min_idx, max(min_idx, max_idx))
                self._spawn_dynamic_pedestrian(self.trajectory_waypoints[ped_idx])
            else:
                self._clear_pedestrian_scene()

            return True
        except Exception as e:
            self._scene_log(f"【自主寻路】规划异常: {e}")
            return False

    def reset_to_default_spawn(self) -> None:
        """将自车重置到默认起点（都会立交枢纽）并清空旧路线与行人。"""
        if not self.connected or self.vehicle is None or carla is None:
            return
        try:
            self._clear_pedestrian_scene()
            spawn_tf = carla.Transform(
                carla.Location(x=self.DEFAULT_SPAWN["x"], y=self.DEFAULT_SPAWN["y"], z=self.DEFAULT_SPAWN["z"]),
                carla.Rotation(pitch=self.DEFAULT_SPAWN["pitch"], yaw=self.DEFAULT_SPAWN["yaw"], roll=self.DEFAULT_SPAWN["roll"])
            )
            self.vehicle.set_transform(spawn_tf)
            self.vehicle.set_target_velocity(carla.Vector3D(0.0, 0.0, 0.0))
            self.vehicle.set_target_angular_velocity(carla.Vector3D(0.0, 0.0, 0.0))
            ctrl = carla.VehicleControl(hand_brake=True, throttle=0.0, brake=1.0)
            self.vehicle.apply_control(ctrl)

            self._current_throttle = 0.0
            self._current_brake = 1.0
            self._current_steer = 0.0
            self._target_v_smooth = 0.0
            self.trajectory_waypoints = []
            self.trajectory_options = []
            self.current_route_total_m = 0.0
            self.last_matched_idx = 0
            self._has_auto_parked = False
            self._cam_loc = None
            self._cam_rot = None
            self._update_camera()
            self._scene_log("【车辆重置】已传送回都会立交枢纽起点 (-4.0, 24.7)")
        except Exception as e:
            self._scene_log(f"【车辆重置】重置失败: {e}")

    def teleport_to_spawn(self, spawn_cfg: dict, route_cfg: Optional[dict] = None) -> None:
        """兼容接口：重置回默认起点。"""
        self.reset_to_default_spawn()

    def apply_control(self, state: VehicleState) -> None:
        """执行 ADAS 智驾综合规划 (纯跟踪 + 曲率自适应减速 + 到站自动挂P + 无抖动执行)。"""
        if not self.connected or self.vehicle is None or carla is None:
            return

        # 0. 座舱高德联动门禁：若开启高德模式，未收到导航广播时驻停
        if state.cockpit_drive_mode:
            now = time.time()
            has_nav = (state.serial_connected and state.has_cockpit_nav and (now - state.cockpit_nav_last_time < 10.0))
            if not has_nav:
                ctrl = carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0, hand_brake=True)
                self.vehicle.apply_control(ctrl)
                state.actual_speed_kmh = 0.0
                state.brake_lamp = True
                state.steering_angle = 0.0
                state.wait_nav_reason = "等待座舱串口连接" if not state.serial_connected else "等待座舱导航广播"
                state.current_maneuver = "待命: 未连接座舱" if not state.serial_connected else "待命: 等待座舱导航"
                self._update_camera()
                return
            else:
                state.wait_nav_reason = ""

        gear = state.gear
        running = state.simulation_running
        speed_limit = float(state.speed_limit_kmh) if state.speed_limit_kmh > 0 else 40.0
        cruise_v = min(speed_limit,max(0.,float(state.target_speed_kmh))) if (running and gear in ("D", "R")) else 0.0

        tf = self.vehicle.get_transform()
        v = self.vehicle.get_velocity()
        actual_v = 3.6 * math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)
        state.actual_speed_kmh = actual_v
        state.pos_x = float(tf.location.x)
        state.pos_y = float(tf.location.y)
        state.heading_rad = math.radians(tf.rotation.yaw)

        path = self.trajectory_waypoints
        total_dist = self.current_route_total_m

        # ---------------- 1. 轨迹匹配与里程推算 ----------------
        if path:
            min_d = 999.0
            best_i = self.last_matched_idx
            search_start = max(0, self.last_matched_idx - 2)
            search_end = min(len(path), self.last_matched_idx + 30)
            for i in range(search_start, search_end):
                wp_loc = path[i].transform.location
                d = math.hypot(wp_loc.x - tf.location.x, wp_loc.y - tf.location.y)
                if d < min_d:
                    min_d = d
                    best_i = i
            self.last_matched_idx = best_i
            dist_traveled = self.last_matched_idx * self.trajectory_step
            d_remain = max(0.0, total_dist - dist_traveled)
            state.route_remaining_distance_m = d_remain
            state.route_total_distance_m = total_dist
        else:
            dist_traveled = 0.0
            d_remain = state.route_remaining_distance_m

        # 更新动态行人逻辑 (无外部视觉指令时绝不自动刹车)
        self._update_pedestrian_scene(state, dist_traveled, actual_v)

        # ---------------- 2. 道路曲率扫描与前瞻转向判定 ----------------
        lookahead_turn = None
        dist_to_turn = 999.0

        if path and len(path) > self.last_matched_idx + 4:
            search_max = min(len(path) - 1, self.last_matched_idx + int(90.0 / self.trajectory_step))
            for i in range(self.last_matched_idx + 2, search_max - 4):
                w_end = min(len(path) - 1, i + int(12.0 / self.trajectory_step))
                y1 = path[i].transform.rotation.yaw
                y2 = path[w_end].transform.rotation.yaw
                dy = (y2 - y1 + 180) % 360 - 180
                if abs(dy) >= 28.0:
                    lookahead_turn = "right" if dy > 0 else "left"
                    dist_to_turn = max(0.0, (i - self.last_matched_idx) * self.trajectory_step)
                    break

        in_turn_now = False
        current_turn_dir = None
        if path and self.last_matched_idx + 6 < len(path):
            idx_ahead = min(len(path) - 1, self.last_matched_idx + 6)
            local_dy = (path[idx_ahead].transform.rotation.yaw - path[self.last_matched_idx].transform.rotation.yaw + 180) % 360 - 180
            if abs(local_dy) >= 18.0:
                in_turn_now = True
                current_turn_dir = "right" if local_dy > 0 else "left"

        target_v = cruise_v
        if gear != "D" or not running or not path:
            target_v = 0.0
            state.turn_signal = "off"
            if not path:
                state.current_maneuver = "待命: 请在座舱地图选择目的地"
        else:
            # A. 终点判定与到站自动停车挂 P 档
            if d_remain <= 1.5 or (d_remain <= 3.5 and actual_v < 1.0 and dist_traveled > total_dist * 0.85):
                target_v = 0.0
                state.segment_remaining_distance_m = 0.0
                if not self._has_auto_parked:
                    self._has_auto_parked = True
                    state.gear = "P"
                    state.current_maneuver = f"已到达目的地 [{self.current_dest_name}] (P档已驻车)"
                    state.navigation_icon = 1
                    state.turn_signal = "off"
                    if self.on_arrival:
                        self.on_arrival()
            # B. 终点前 30m 提前平滑减速
            elif d_remain <= 30.0:
                target_v = max(4.0, cruise_v * (d_remain / 30.0))
                state.segment_remaining_distance_m = max(0.0, d_remain)
                state.current_maneuver = "即将到达目的地 (减速中)"
                state.navigation_icon = 1
                state.turn_signal = "off"
            # C. 弯道减速规控 (稳速 15km/h 过弯)
            elif in_turn_now:
                target_v = 15.0
                state.segment_remaining_distance_m = max(1.0, dist_to_turn) if dist_to_turn > 0 else 0.0
                state.turn_signal = current_turn_dir
                state.navigation_icon = 3 if current_turn_dir == "right" else 2
                state.current_maneuver = f"弯道{'右转' if current_turn_dir == 'right' else '左转'}中 (稳速15km/h)"
            elif lookahead_turn is not None and dist_to_turn <= 35.0:
                p = dist_to_turn / 35.0
                target_v = 15.0 + (cruise_v - 15.0) * p
                state.segment_remaining_distance_m = dist_to_turn
                state.turn_signal = lookahead_turn
                state.navigation_icon = 3 if lookahead_turn == "right" else 2
                state.current_maneuver = f"前方{'右转' if lookahead_turn == 'right' else '左转'} (提前减速 {dist_to_turn:.0f}m)"
            else:
                target_v = cruise_v
                state.segment_remaining_distance_m = dist_to_turn if (lookahead_turn is not None and dist_to_turn < 1000.0) else max(1.0, d_remain)
                state.turn_signal = "off"
                state.navigation_icon = 1
                state.current_maneuver = f"直行前往 {self.current_dest_name}"

        speed_cap = getattr(state, "navigation_speed_cap_kmh", None)
        if speed_cap is not None:
            target_v = min(target_v, max(0., float(speed_cap)))
        self._navigation_effective_target = target_v

        # ---------------- 视觉感知域闭环覆盖 (仅在底盘接管开启时生效) ----------------
        # 用户特别要求：未开接管时不报警、不刹车，直接开过去；开了接管且视觉下发 AEB 才制动
        if getattr(state, "takeover_enabled", False):
            if getattr(state, "vision_aeb_active", False):
                target_v = 0.0
                state.brake_lamp = True
                state.current_maneuver = f"视觉AEB紧急制动: {getattr(state, 'vision_action', '前方障碍物')}"
            elif getattr(state, "vision_speed_override", None) is not None:
                v_ov = float(state.vision_speed_override)
                target_v = min(target_v, v_ov)
                if v_ov == 0.0:
                    state.current_maneuver = f"视觉驻车: {getattr(state, 'vision_action', '')}"

        # ---------------- 3. Pure Pursuit 横向转向规划 ----------------
        steer_cmd = 0.0
        if path:
            lookahead = max(3.5, min(8.0, 3.0 + 0.35 * (actual_v / 3.6)))
            target_idx = min(len(path) - 1, self.last_matched_idx + int(lookahead / self.trajectory_step))
            tgt_wp = path[target_idx].transform.location

            dx = tgt_wp.x - tf.location.x
            dy = tgt_wp.y - tf.location.y
            yaw_rad = math.radians(tf.rotation.yaw)
            local_x = dx * math.cos(-yaw_rad) - dy * math.sin(-yaw_rad)
            local_y = dx * math.sin(-yaw_rad) + dy * math.cos(-yaw_rad)

            alpha = math.atan2(local_y, local_x)
            delta = math.atan2(2 * 2.85 * math.sin(alpha), lookahead)
            raw_steer = delta / math.radians(35.0)

            max_steer_limit = 0.45 if (in_turn_now or (lookahead_turn is not None and dist_to_turn <= 15.0)) else 0.35
            steer_cmd = max(-max_steer_limit, min(max_steer_limit, raw_steer))
            state.steering_angle = math.degrees(delta)
        else:
            state.steering_angle = 0.0

        if getattr(state, "takeover_enabled", False) and getattr(state, "vision_steer_bias", 0.0) != 0.0:
            bias_cmd = state.vision_steer_bias / 35.0
            steer_cmd = max(-0.55, min(0.55, steer_cmd + bias_cmd))
            state.steering_angle += state.vision_steer_bias

        # 转向变化率平滑滤波
        max_d_steer = 0.035
        d_steer = steer_cmd - self._current_steer
        if abs(d_steer) > max_d_steer:
            self._current_steer += math.copysign(max_d_steer, d_steer)
        else:
            self._current_steer = steer_cmd
        smoothed_steer = max(-1.0, min(1.0, self._current_steer))
        state.steering_angle = smoothed_steer * 35.0

        # ---------------- 4. 平滑纵向控制律 ----------------
        ctrl = carla.VehicleControl()
        if state.gear == "P" or self._has_auto_parked:
            ctrl.hand_brake = True
            ctrl.brake = 1.0
            ctrl.throttle = 0.0
            ctrl.steer = 0.0
            ctrl.reverse = False
            self._current_throttle = 0.0
            self._current_brake = 1.0
            self._longitudinal_mode = "brake"
            state.brake_lamp = True
        elif getattr(state, "vision_aeb_active", False) and getattr(state, "takeover_enabled", False):
            ctrl.hand_brake = False
            ctrl.steer = smoothed_steer
            ctrl.reverse = False
            self._current_throttle = max(0.0, self._current_throttle - 0.10)
            self._current_brake = min(1.0, self._current_brake + 0.15)
            ctrl.throttle = self._current_throttle
            ctrl.brake = self._current_brake
            self._longitudinal_mode = "brake"
            state.brake_lamp = True
        elif state.gear == "N":
            ctrl.hand_brake = False
            ctrl.brake = 0.2
            ctrl.throttle = 0.0
            ctrl.steer = smoothed_steer
            ctrl.reverse = False
            self._current_throttle = 0.0
            self._current_brake = 0.2
            self._longitudinal_mode = "brake"
            state.brake_lamp = True
        elif state.gear in ("D", "R"):
            ctrl.hand_brake = False
            ctrl.reverse = (state.gear == "R")
            ctrl.steer = smoothed_steer

            t_now = time.perf_counter()
            ctrl_dt = (t_now - self._last_control_time) if self._last_control_time else 0.02
            self._last_control_time = t_now
            ctrl_dt = max(0.005, min(0.20, ctrl_dt))
            if getattr(state, 'navigation_aeb_enabled', False):
                # The speed cap is an upper bound even if the previous target
                # was high. Acceleration after a stop is deliberately gradual.
                self._target_v_smooth = min(target_v, self._target_v_smooth + 6.*ctrl_dt)
                from navigation_motion import longitudinal_command
                self._current_throttle, self._current_brake = longitudinal_command(
                    actual_v, self._target_v_smooth, ctrl_dt,
                    self._current_throttle, self._current_brake)
            else:
                self._target_v_smooth = self.rate_limit_target(self._target_v_smooth, target_v, ctrl_dt)
                eff_target_v = self._target_v_smooth
                speed_err = eff_target_v - actual_v

                if eff_target_v <= 0.5:
                    target_throttle = 0.0
                    target_brake = min(1.0, 0.40 + actual_v * 0.04)
                elif speed_err > 1.0:
                    ff = actual_v / 200.0
                    target_throttle = min(0.70, 0.18 + ff + speed_err * 0.028)
                    target_brake = 0.0
                elif speed_err >= -2.0:
                    ff = actual_v / 200.0
                    target_throttle = (0.15 + ff) if speed_err > 0.5 else 0.0
                    target_brake = 0.0
                else:
                    target_throttle = 0.0
                    excess = abs(speed_err) - 2.0
                    target_brake = min(0.55, 0.08 + excess * 0.030)

                max_d_throttle = 0.04
                max_d_brake = 0.08

                if target_brake > 0.01 and self._current_throttle > 0.01:
                    target_brake = 0.0
                    target_throttle = 0.0
                elif target_throttle > 0.01 and self._current_brake > 0.01:
                    target_throttle = 0.0
                    target_brake = 0.0

                d_t = target_throttle - self._current_throttle
                self._current_throttle += max(-max_d_throttle, min(max_d_throttle, d_t))
                self._current_throttle = max(0.0, min(1.0, self._current_throttle))

                d_b = target_brake - self._current_brake
                self._current_brake += max(-max_d_brake, min(max_d_brake, d_b))
                self._current_brake = max(0.0, min(1.0, self._current_brake))

            ctrl.throttle = self._current_throttle
            ctrl.brake = self._current_brake
            self._brake_lamp_active = (state.gear == "P" or self._has_auto_parked or ctrl.brake >= 0.05)
            state.brake_lamp = self._brake_lamp_active

        self.vehicle.apply_control(ctrl)
        self._update_camera()

        # ---------------- 5. 整车灯光同步 ----------------
        light_state = carla.VehicleLightState.NONE
        if state.turn_signal == "left":
            light_state |= carla.VehicleLightState.LeftBlinker
        elif state.turn_signal == "right":
            light_state |= carla.VehicleLightState.RightBlinker
        if state.brake_lamp:
            light_state |= carla.VehicleLightState.Brake
        if state.reverse_lamp or state.gear == "R":
            light_state |= carla.VehicleLightState.Reverse
        self.vehicle.set_light_state(carla.VehicleLightState(light_state))

    def disconnect(self) -> None:
        self.running = False
        self._clear_pedestrian_scene()
        if self.vehicle is not None:
            try:
                if getattr(self.vehicle, "is_alive", True):
                    self.vehicle.destroy()
            except Exception:
                pass
            self.vehicle = None
        self.connected = False
