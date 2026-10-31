#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
oib_link.py —— RK3568 与 PC HIL (OIB 控制中枢) 通信与自主决策层
==============================================================
严格遵循 ICD §4.1.1 规范与 IMPLEMENTATION_PROMPT 契约：
1. 下行监听 UDP 8892：接收 HIL 广播的底盘权威车速 (vehicle.speed) 与视频源选择 (perception_source)
2. 上行发送 UDP 8888：2Hz 心跳保活 + 决策报文上报 (AEB / TTC / 红绿灯 / 障碍物)
3. 启动顺序保障：先启动心跳发送向 PC 宣告板端 IP，使 PC 能定向将 8892 车速广播回板端
"""

import json
import math
import socket
import threading
import time
from typing import Any, Dict, List, Optional, Tuple


class OibLink:
    """RK3568 ADAS 域与 PC HIL (OIB 中枢) 交互链路及安全决策机。"""

    def __init__(self, pc_ip: str = "192.168.10.1", oib_port: int = 8888,
                 listen_port: int = 8892, x_sign: float = 1.0) -> None:
        self.pc_ip = pc_ip
        self.oib_port = oib_port
        self.listen_port = listen_port
        self.x_sign = x_sign  # 空间横向坐标符号因子 (ICD 规定自车左侧为正)

        self._running = False
        self._lock = threading.Lock()

        # 权威底盘状态 (由 PC 8892 下发)
        self.vehicle_speed_kmh: float = 0.0
        self.perception_source: str = "pcie"  # "pcie" | "carla"
        self.last_recv_8892_time: float = 0.0

        # 套接字初始化
        self._tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # 线程控制
        self._heartbeat_thread: Optional[threading.Thread] = None
        self._rx_thread: Optional[threading.Thread] = None

    def start(self) -> None:
        """按规定顺序启动通信：心跳线程先行，随后启动监听线程。"""
        if self._running:
            return
        self._running = True

        try:
            self._rx_sock.bind(("0.0.0.0", self.listen_port))
            self._rx_sock.settimeout(0.5)
        except Exception as e:
            print(f"[OibLink] 绑定 UDP {self.listen_port} 端口失败: {e}")

        # 1. 优先启动心跳保活线程 (向 PC 宣告源 IP)
        self._heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self._heartbeat_thread.start()

        # 2. 启动 8892 下行接收线程
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        print(f"[OibLink] 通信链路已启动: 上行 -> {self.pc_ip}:{self.oib_port}, 下行 <- UDP {self.listen_port}")

    def _heartbeat_loop(self) -> None:
        """2Hz (500ms) 周期性发送心跳保活包，刷新 HIL 看门狗。"""
        while self._running:
            hb = {
                "message_type": "heartbeat",
                "domain": "ADAS",
                "stamp": round(time.time(), 3),
            }
            self.send_raw_json(hb)
            time.sleep(0.5)

    def _rx_loop(self) -> None:
        """下行监听 UDP 8892，安全解析权威车速与视频源指令。"""
        while self._running:
            try:
                data, _ = self._rx_sock.recvfrom(4096)
                if not data:
                    continue
                payload = json.loads(data.decode("utf-8"))
                now = time.time()
                with self._lock:
                    self.last_recv_8892_time = now
                    veh = payload.get("vehicle", {})
                    if "speed" in veh and veh["speed"] is not None:
                        self.vehicle_speed_kmh = float(veh["speed"])

                    # 提取视频源切换指示
                    if "perception_source" in payload:
                        self.perception_source = str(payload["perception_source"]).lower()
            except socket.timeout:
                continue
            except Exception:
                continue

    def send_raw_json(self, payload: Dict[str, Any]) -> None:
        """底层单包 UDP 发送。"""
        try:
            data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            self._tx_sock.sendto(data, (self.pc_ip, self.oib_port))
        except Exception as e:
            pass

    def evaluate_and_report(self, dets: Any, stereo_pairs: Optional[List[Dict[str, Any]]] = None,
                            tl_state: Optional[str] = None) -> Dict[str, Any]:
        """
        基于检测结果、测距与权威车速进行自主安全决策并上报 UDP 8888。
        
        参数:
            dets: 目标检测结果
            stereo_pairs: 双目测距结果配对列表 [{"Z": 毫米, "X": 毫米, "class": 类别}, ...]
            tl_state: 交通灯颜色 ("RED", "GREEN", "YELLOW" 等)
        """
        with self._lock:
            cur_speed_kmh = self.vehicle_speed_kmh

        speed_mps = cur_speed_kmh / 3.6
        aeb_triggered = False
        brake_pressure = 0.0
        action = ""
        veto = ""
        min_x_m: Optional[float] = None
        min_ttc: float = float("inf")
        light_dist_m: Optional[float] = None

        # 1. 障碍物测距与 TTC 拍板 (若有双目测距数据)
        if stereo_pairs:
            for item in stereo_pairs:
                z_mm = item.get("Z")
                x_mm = item.get("X", 0.0)
                if z_mm is None or z_mm <= 0:
                    continue

                # 坐标变换：Z 对应前方纵向深度 x_m，X 对应横向偏差 y_m
                x_m = float(z_mm) / 1000.0
                y_m = (float(x_mm) / 1000.0) * self.x_sign

                # 仅关切本车道安全包络内的目标 (|y_m| <= 1.2m)
                if abs(y_m) <= 1.2 and 0.5 <= x_m <= 30.0:
                    if min_x_m is None or x_m < min_x_m:
                        min_x_m = x_m

                    ttc = (x_m / speed_mps) if speed_mps > 0.5 else float("inf")
                    if ttc < min_ttc:
                        min_ttc = ttc

                    cls_name = item.get("class", "前方障碍物")
                    # AEB 判定：TTC < 2.0s 或 纵向距离 < 3.5m 且非静止
                    if (ttc < 2.0 or x_m < 3.5) and not aeb_triggered:
                        aeb_triggered = True
                        brake_pressure = 1.0
                        action = f"紧急制动-{cls_name}"
                        veto = f"前方 {x_m:.1f}m 探测到{cls_name} (TTC={ttc:.1f}s)"

        # 2. 交通信号灯逻辑
        if tl_state:
            tl_upper = str(tl_state).upper()
            if "RED" in tl_upper or "红" in tl_state:
                if not action:
                    action = "红灯"
                else:
                    action += "|红灯"
                if min_x_m is not None:
                    light_dist_m = min_x_m
            elif "GREEN" in tl_upper or "绿" in tl_state:
                if not action:
                    action = "绿灯"
            elif "YELLOW" in tl_upper or "黄" in tl_state:
                if not action:
                    action = "减速"

        # 3. 组装标准 UDP 8888 报文 (严格遵循 ICD §4.1.1 字段定义)
        report = {
            "aeb_triggered": bool(aeb_triggered),
            "brake_pressure": float(brake_pressure),
            "action": str(action),
            "stop_line_m": None,         # 首版发 null，走高德路口优雅降级
            "light_distance_m": light_dist_m,
            "vehicle_speed": None,       # 巡航红线：必须发 null，绝不发自车车速
            "steering_angle": 0.0,
            "lane_change_state": "idle",
            "veto": str(veto),
            "domain": "ADAS",
            "stamp": round(time.time(), 3),
        }

        self.send_raw_json(report)
        return report

    def stop(self) -> None:
        """关闭通信。"""
        self._running = False
        try:
            self._rx_sock.close()
            self._tx_sock.close()
        except Exception:
            pass


if __name__ == "__main__":
    link = OibLink(pc_ip="127.0.0.1")
    link.start()
    print("[OibLink] 演示发送决策报文...")
    try:
        for i in range(5):
            time.sleep(1.0)
            rep = link.evaluate_and_report(dets=[], stereo_pairs=[{"Z": 3200, "X": 100, "class": "pedestrian"}])
            print("上报决策:", rep)
    finally:
        link.stop()
