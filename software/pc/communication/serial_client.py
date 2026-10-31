"""座舱域串口通信线程。

负责自动探测并连接树莓派 USB 串口 (115200 波特率)，
双向流转 5AA5 二进制高德导航帧与 JSON 联动帧。
"""

from __future__ import annotations

import select
import socket
import threading
import time
from typing import Callable, Optional

import serial
import serial.tools.list_ports

from .protocol_5aa5 import CockpitStreamSplitter, encode_gear_linkage


class VirtualSocketServerLink:
    """纯软件模拟器使用的 TCP 虚拟串口适配层 (兼具类似 serial.Serial 的接口)。
    监听 0.0.0.0:58800，接受 Android 模拟器 (通过 10.0.2.2:58800) 的 TCP 连接。
    支持多客户端多路复用与断线干净回收，防止多模拟器相互踢下线。
    """
    def __init__(self, host: str = "0.0.0.0", port: int = 58800) -> None:
        self.host = host
        self.port = port
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.bind((self.host, self.port))
        self.server_sock.listen(5)
        self.server_sock.settimeout(0.5)

        self.clients: list[socket.socket] = []
        self.is_open = True
        self._lock = threading.Lock()
        self._accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
        self._accept_thread.start()

    def _accept_loop(self) -> None:
        while self.is_open:
            try:
                conn, _ = self.server_sock.accept()
                conn.setblocking(False)
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                with self._lock:
                    self.clients.append(conn)
            except (socket.timeout, OSError):
                continue

    @property
    def in_waiting(self) -> int:
        return 0

    def read(self, size: int = 1) -> bytes:
        if not self.is_open:
            return b""
        with self._lock:
            active_clients = list(self.clients)
        if not active_clients:
            time.sleep(0.02)
            return b""

        try:
            r_ready, _, _ = select.select(active_clients, [], [], 0.05)
        except (select.error, OSError, ValueError):
            return b""

        chunks = bytearray()
        for sock in r_ready:
            try:
                data = sock.recv(1024)
                if not data:
                    self._remove_client(sock)
                else:
                    chunks.extend(data)
            except (socket.error, OSError):
                self._remove_client(sock)

        return bytes(chunks)

    def _remove_client(self, sock: socket.socket) -> None:
        with self._lock:
            if sock in self.clients:
                self.clients.remove(sock)
        try:
            sock.close()
        except Exception:
            pass

    def write(self, data: bytes) -> int:
        if not self.is_open or not data:
            return len(data)
        with self._lock:
            active_clients = list(self.clients)
        if not active_clients:
            return len(data)
        for sock in active_clients:
            try:
                sock.sendall(data)
            except (socket.error, OSError):
                self._remove_client(sock)
        return len(data)

    def flush(self) -> None:
        pass

    def close(self) -> None:
        self.is_open = False
        with self._lock:
            for sock in self.clients:
                try:
                    sock.close()
                except Exception:
                    pass
            self.clients.clear()
        try:
            self.server_sock.close()
        except Exception:
            pass


class SerialClient:
    """与树莓派座舱网关的双向串口通信管理器。"""

    def __init__(self,
                 on_frame: Optional[Callable[[dict], None]] = None,
                 on_status: Optional[Callable[[str, bool], None]] = None) -> None:
        self.on_frame = on_frame
        self.on_status = on_status
        self.port: Optional[str] = None
        self.baudrate = 115200
        self.link = None
        self.running = False
        self._thread: Optional[threading.Thread] = None

    VIRTUAL_EMU_PORT = "VIRTUAL_EMU"

    @classmethod
    def list_ports(cls) -> list[tuple[str, str]]:
        ports = [(cls.VIRTUAL_EMU_PORT, "💻 纯软件虚拟座舱模拟器 (TCP 58800)")]
        for p in serial.tools.list_ports.comports():
            desc = p.description or p.device
            ports.append((p.device, f"{p.device} ({desc})"))
        return ports

    @classmethod
    def find_preferred_port(cls) -> Optional[str]:
        for p in serial.tools.list_ports.comports():
            label = (p.description or "").upper() + " " + p.device.upper()
            if any(k in label for k in ("CH340", "CP210", "USB", "FTDI")):
                return p.device
        # 若未找到明确 USB 标签，尝试 COM6 (常见树莓派调试端口)
        for p in serial.tools.list_ports.comports():
            if p.device.upper() == "COM6":
                return p.device
        # 若未检测到有效物理串口硬件，自动回退到纯软件虚拟模拟器链路
        return cls.VIRTUAL_EMU_PORT

    def connect(self, port: Optional[str] = None) -> bool:
        if port is None:
            port = self.find_preferred_port()
        if not port:
            if self.on_status:
                self.on_status("未发现可用座舱串口", False)
            return False

        self.disconnect()
        self.port = port
        try:
            if self.port == self.VIRTUAL_EMU_PORT:
                self.link = VirtualSocketServerLink("0.0.0.0", 58800)
            elif self.port.startswith("socket://"):
                self.link = serial.serial_for_url(self.port, timeout=0.2)
            else:
                self.link = serial.Serial(self.port, self.baudrate, timeout=0.2)

            self.running = True
            self._thread = threading.Thread(target=self._read_loop, daemon=True)
            self._thread.start()
            if self.on_status:
                display_name = "纯软件虚拟座舱模拟器 (TCP 58800 监听中)" if self.port == self.VIRTUAL_EMU_PORT else self.port
                self.on_status(f"座舱链路已启动：{display_name}", True)
            return True
        except (serial.SerialException, OSError) as err:
            self.link = None
            if self.on_status:
                self.on_status(f"连接串口 {self.port} 失败: {err}", False)
            return False

    def disconnect(self) -> None:
        self.running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=0.8)
        if self.link and self.link.is_open:
            try:
                self.link.close()
            except Exception:
                pass
        self.link = None
        if self.on_status:
            self.on_status("座舱串口已断开", False)

    def is_connected(self) -> bool:
        return bool(self.link and self.link.is_open)

    def send_gear_and_speed(self, gear: str, speed_kmh: int, route_id: str = "", route_name: str = "", simulation_running: bool = True) -> bool:
        """向下位机座舱下发挡位与目标车速联动帧。"""
        if not self.is_connected() or self.link is None:
            return False
        try:
            data = encode_gear_linkage(gear, speed_kmh, route_id, route_name, simulation_running)
            self.link.write(data)
            self.link.flush()
            return True
        except (serial.SerialException, OSError):
            return False

    def send_route_and_state(self, route_id: str, gear: str, target_speed_kmh: float, simulation_running: bool = True) -> bool:
        """向下位机座舱下发路线切换与仿真状态联动帧。"""
        if not self.is_connected() or self.link is None:
            return False
        try:
            data = encode_gear_linkage(gear, int(round(target_speed_kmh)), route_id=route_id, simulation_running=simulation_running)
            self.link.write(data)
            self.link.flush()
            return True
        except (serial.SerialException, OSError):
            return False

    def send_telemetry(self, gear: str, actual_speed: float, target_speed: float,
                       seg_remain_m: float, route_remain_m: float,
                       route_id: str = "", route_name: str = "", simulation_running: bool = True,
                       pos_x: float = 0.0, pos_y: float = 0.0, heading_deg: float = 0.0,
                       pedestrian_detected: bool = False, pedestrian_distance_m: float = 0.0,
                       aeb_active: bool = False) -> bool:
        """向下位机座舱定时下发实车真实瞬时速度、里程、路线与仿真状态、位姿坐标与行人预警。"""
        if not self.is_connected() or self.link is None:
            return False
        try:
            from .protocol_5aa5 import encode_telemetry_linkage
            data = encode_telemetry_linkage(gear, actual_speed, target_speed, seg_remain_m, route_remain_m,
                                            route_id=route_id, route_name=route_name, simulation_running=simulation_running,
                                            pos_x=pos_x, pos_y=pos_y, heading_deg=heading_deg,
                                            pedestrian_detected=pedestrian_detected,
                                            pedestrian_distance_m=pedestrian_distance_m,
                                            aeb_active=aeb_active)
            self.link.write(data)
            self.link.flush()
            return True
        except (serial.SerialException, OSError):
            return False

    def send_route_waypoints(self, waypoints: list, total_m: float) -> bool:
        """向下位机座舱下发 CARLA 原生最优路径的高精度路点坐标序列。"""
        if not self.is_connected() or self.link is None:
            return False
        try:
            import json
            payload = {
                "action": "ROUTE_SYNC",
                "total_m": round(total_m, 1),
                "waypoints": [[round(x, 1), round(y, 1)] for x, y in waypoints]
            }
            line = json.dumps(payload, separators=(',', ':')) + "\n"
            self.link.write(line.encode("utf-8"))
            self.link.flush()
            return True
        except (serial.SerialException, OSError):
            return False

    def _read_loop(self) -> None:
        splitter = CockpitStreamSplitter()
        while self.running and self.link and self.link.is_open:
            try:
                n = self.link.in_waiting
                chunk = self.link.read(n if n > 0 else 1)
                if not chunk:
                    continue
                for frame in splitter.feed(chunk):
                    if self.on_frame:
                        self.on_frame(frame)
            except (serial.SerialException, OSError):
                break
        self.running = False
