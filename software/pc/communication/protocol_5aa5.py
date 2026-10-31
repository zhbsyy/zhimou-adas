"""座舱域串口协议 5A A5 / JSON 编解码模块。

严格兼容树莓派 Android 高德网关 (amap_uart_gateway) 与 ADAS 上位机契约。
- 接收：
  1. 高德导航帧：5A A5 头，0x01 类型，包含路段剩余米数、全程剩余米数、导航动作图标、当前车速
  2. 模拟器状态帧：5A A5 或 JSON 行，包含挡位、目标速度、实际速度、路线编号、动作
- 发送：
  1. 下发挡位/速度联动：向座舱广播 GEAR_LINKAGE JSON 帧，触发倒车影像与界面同步
"""

from __future__ import annotations

import json
import struct
from dataclasses import dataclass

MARK_HEAD = b"\x5a\xa5"
MARK_TAIL = b"\xbb\xbb"
WIRE_VERSION = 1
MSG_NAVIGATION = 1
PAYLOAD_BYTES = 22
HEADER_BYTES = 14
FRAME_BYTES = HEADER_BYTES + PAYLOAD_BYTES
KEY_TYPE_NAVIGATION = 10001
MAX_PAYLOAD = 1024

_NAV_STRUCT = struct.Struct("<HHIIIHHBB")
_ENVELOPE_STRUCT = struct.Struct("<HHHH")


def crc16_ccitt(chunk: bytes) -> int:
    """CCITT-FALSE 校验：多项式 0x1021，初值 0xFFFF，不反转。"""
    crc = 0xFFFF
    for byte in chunk:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


@dataclass
class NavFrame:
    """已通过全量校验的高德导航二进制帧。"""
    icon: int
    segment_m: int
    route_m: int
    remain_s: int
    speed_kmh: int
    limit_kmh: int
    active: bool
    sequence: int
    key_type: int = KEY_TYPE_NAVIGATION

    def as_dict(self) -> dict:
        return {
            "action": "AMAP_NAVIGATION",
            "navigation_icon": self.icon,
            "navigation_active": self.active,
            "segment_distance_m": self.segment_m,
            "route_distance_m": self.route_m,
            "route_remaining_s": self.remain_s,
            "navigation_current_speed_kmh": self.speed_kmh,
            "navigation_speed_limit_kmh": self.limit_kmh,
            "amap_key_type": self.key_type,
            "sequence": self.sequence,
        }


def decode_nav_frame(frame: bytes) -> NavFrame | None:
    if len(frame) != FRAME_BYTES:
        return None
    if frame[:2] != MARK_HEAD or frame[-2:] != MARK_TAIL:
        return None
    body = frame[2:-4]
    if crc16_ccitt(body) != int.from_bytes(frame[-4:-2], "little"):
        return None
    version, msg_type, sequence, payload_size = _ENVELOPE_STRUCT.unpack_from(body)
    if version != WIRE_VERSION or msg_type != MSG_NAVIGATION or payload_size != PAYLOAD_BYTES:
        return None
    (key_type, icon, segment_m, route_m, remain_s,
     speed_kmh, limit_kmh, flags, _reserved) = _NAV_STRUCT.unpack_from(body, 8)
    if key_type not in (KEY_TYPE_NAVIGATION, 12205):
        return None
    return NavFrame(
        icon=int(icon), segment_m=int(segment_m), route_m=int(route_m),
        remain_s=int(remain_s), speed_kmh=int(speed_kmh), limit_kmh=int(limit_kmh),
        active=bool(flags & 0x01), sequence=int(sequence), key_type=int(key_type),
    )


def decode_text_line(line: str) -> dict | None:
    line = line.strip()
    if not line:
        return None
    if line.startswith("{"):
        try:
            val = json.loads(line)
            return val if isinstance(val, dict) else None
        except json.JSONDecodeError:
            return None
    return None


class CockpitStreamSplitter:
    """增量式字节流切帧器：支持 5AA5 二进制帧与换行分隔 JSON 文本混传。"""

    def __init__(self, ceiling: int = MAX_PAYLOAD) -> None:
        self.buffer = bytearray()
        self.ceiling = int(ceiling)

    def reset(self) -> None:
        self.buffer.clear()

    def feed(self, chunk: bytes) -> list[dict]:
        self.buffer.extend(chunk)
        out: list[dict] = []
        while self.buffer:
            head_at = self.buffer.find(MARK_HEAD)
            line_at = self.buffer.find(b"\n")

            # 优先处理以 '{' 开头的完整 JSON 行
            if self.buffer.startswith(b"{") and line_at >= 0:
                raw = bytes(self.buffer[:line_at]).decode("utf-8", errors="ignore")
                del self.buffer[:line_at + 1]
                msg = decode_text_line(raw)
                if msg:
                    out.append(msg)
                continue

            if line_at >= 0 and (head_at < 0 or line_at < head_at):
                raw = bytes(self.buffer[:line_at]).decode("utf-8", errors="ignore")
                del self.buffer[:line_at + 1]
                msg = decode_text_line(raw)
                if msg:
                    out.append(msg)
                continue

            if head_at < 0:
                if self.buffer.startswith((b"{", b"$")):
                    if len(self.buffer) > self.ceiling:
                        del self.buffer[:-self.ceiling]
                    break
                keep = 1 if self.buffer.endswith(MARK_HEAD[:1]) else 0
                if len(self.buffer) > keep:
                    if keep:
                        del self.buffer[:-1]
                    else:
                        self.buffer.clear()
                break

            if head_at:
                if not self.buffer.startswith((b"{", b"$")):
                    del self.buffer[:head_at]
                else:
                    break
            if len(self.buffer) < 10:
                break

            payload_size = int.from_bytes(self.buffer[8:10], "little")
            if payload_size > self.ceiling:
                del self.buffer[0]
                continue
            frame_size = HEADER_BYTES + payload_size
            if len(self.buffer) < frame_size:
                break

            nav = decode_nav_frame(bytes(self.buffer[:frame_size]))
            if nav is None:
                del self.buffer[0]
                continue
            out.append(nav.as_dict())
            del self.buffer[:frame_size]
        return out


def encode_gear_linkage(gear: str, speed_kmh: int, route_id: str = "", route_name: str = "", simulation_running: bool = True) -> bytes:
    """编码下发给树莓派座舱的联动帧（触发倒车影像与 UI 挡位同步）。"""
    payload = {
        "action": "GEAR_LINKAGE",
        "gear": str(gear).upper(),
        "speed": int(speed_kmh),
        "actual_speed_kmh": int(speed_kmh),
        "target_speed_kmh": int(speed_kmh),
        "camera": "reverse" if str(gear).upper() == "R" else "none",
        "route_id": route_id,
        "route_name": route_name,
        "simulation_running": simulation_running,
    }
    return json.dumps(payload, ensure_ascii=False).encode("utf-8") + b"\n"


def encode_telemetry_linkage(gear: str, actual_speed_kmh: float, target_speed_kmh: float,
                             seg_remaining_m: float, route_remaining_m: float,
                             route_id: str = "", route_name: str = "",
                             simulation_running: bool = True,
                             pos_x: float = 0.0, pos_y: float = 0.0, heading_deg: float = 0.0,
                             pedestrian_detected: bool = False, pedestrian_distance_m: float = 0.0,
                             aeb_active: bool = False) -> bytes:
    """编码下发给树莓派座舱的实时瞬时遥测帧 (物理瞬时车速、剩余米数、挡位、路线与仿真状态、位姿坐标与行人预警)。"""
    payload = {
        "action": "GEAR_LINKAGE",
        "gear": str(gear).upper(),
        "speed": int(round(target_speed_kmh)),
        "actual_speed_kmh": int(round(actual_speed_kmh)),
        "target_speed_kmh": int(round(target_speed_kmh)),
        "segment_remaining_distance_m": int(round(seg_remaining_m)),
        "route_remaining_distance_m": int(round(route_remaining_m)),
        "camera": "reverse" if str(gear).upper() == "R" else "none",
        "route_id": route_id,
        "route_name": route_name,
        "simulation_running": simulation_running,
        "pos_x": round(float(pos_x), 2),
        "pos_y": round(float(pos_y), 2),
        "heading_deg": round(float(heading_deg), 1),
        "pedestrian_detected": bool(pedestrian_detected),
        "pedestrian_distance_m": round(float(pedestrian_distance_m), 1),
        "aeb_active": bool(aeb_active),
    }
    return json.dumps(payload, ensure_ascii=False).encode("utf-8") + b"\n"
