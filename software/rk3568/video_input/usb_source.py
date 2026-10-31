"""USB(UVC/V4L2) 最新帧采集源。

设计目标：接口与 `optimization_trial_20261015/fresh_source.py` 里的 `CarlaSource` 一致，
方便直接替换/混用；并且**永远只保留最新一帧**——4 路视频里最怕的就是某个源慢了以后
队列无限堆积，画面越看越旧。

统计里把 `grab_ms`（取一帧压缩数据）与 `retrieve_ms`（JPEG 解码）**分开计**，
因为 RK3568 上 720p MJPG 软解很可能是整条链路的真正瓶颈。

对外接口：
    src = UsbCameraSource("/dev/video0", label="front")
    src.start()
    frame = src.read_newer(last_seq, timeout=0.2)   # -> SourceFrame 或 None
    src.snapshot()                                   # -> dict 统计
    src.close()

自测：
    python3 usb_source.py --devices /dev/video0 --seconds 5
"""
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import threading
import time
from dataclasses import dataclass

import cv2
import numpy as np


def apply_v4l2_ctrls(device, ctrls) -> dict:
    """用 v4l2-ctl 按**控件名**下发设置（OpenCV 无法可靠设置这些具名控件）。

    典型用途：关闭"动态帧率"(exposure_dynamic_framerate=0)、改手动曝光，
    否则暗光下摄像头会自动把 30fps 降到 15/10fps。

    返回 {控件名: "ok" | 错误信息}。
    """
    if not ctrls:
        return {}
    if not shutil.which("v4l2-ctl"):
        return {k: "v4l2-ctl 不存在，跳过" for k in ctrls}
    dev = device if isinstance(device, str) else f"/dev/video{int(device)}"
    report = {}
    for name, value in ctrls.items():
        cmd = ["v4l2-ctl", "-d", dev, f"--set-ctrl={name}={value}"]
        try:
            p = subprocess.run(cmd, capture_output=True, text=True, timeout=3)
            report[name] = "ok" if p.returncode == 0 else (p.stderr or p.stdout or "失败").strip()
        except Exception as exc:
            report[name] = f"{type(exc).__name__}: {exc}"
    return report


@dataclass(frozen=True)
class SourceFrame:
    """与 fresh_source.SourceFrame 字段一致，便于两套源互换。"""
    bgr: np.ndarray
    sequence: int
    received_mono: float


class UsbCameraSource:
    """单个 UVC 摄像头的"最新帧"采集线程。

    参数
    ----
    device      : int 或 str。同型号摄像头无法用 by-id 区分时，用 /dev/videoN 或 udev 固定名。
    width/height: 期望分辨率。**注意**：如果下游每个画面只需要 640x360，
                  直接让摄像头出 640x360 最省——解码量约为 720p 的 1/4。
    fps         : 期望帧率。
    fourcc      : 必须是 "MJPG"。不要用 YUYV——720p 下 YUYV 只有 5 fps。
    warmup      : 打开后丢弃的前若干帧（自动曝光/白平衡收敛）。
    """

    def __init__(self, device, width: int = 1280, height: int = 720, fps: int = 30,
                 fourcc: str = "MJPG", warmup: int = 5, label: str = "",
                 reconnect_delay: float = 0.5, ctrls: dict | None = None,
                 required_ctrls: tuple[str, ...] = ()):
        self.device = device
        self.width, self.height, self.fps = int(width), int(height), int(fps)
        self.fourcc = fourcc
        self.warmup = int(warmup)
        self.label = label or str(device)
        self.reconnect_delay = float(reconnect_delay)
        # 关键：默认就关掉"动态帧率"，否则暗光会自动降帧（实测 30→15/10 fps）
        self.ctrls = dict(ctrls) if ctrls else {
            "exposure_dynamic_framerate": 0,
            "power_line_frequency": 1,          # 1 = 50Hz（中国），防闪烁
        }
        self.required_ctrls = tuple(required_ctrls)

        self._cond = threading.Condition()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._latest: SourceFrame | None = None
        self._opened = False
        self._actual: dict = {}
        self._ctrl_report: dict = {}
        self.stats = dict(
            label=self.label, device=str(device),
            decoded_count=0, read_failures=0, reconnects=0,
            grab_ms=0.0, retrieve_ms=0.0, total_ms=0.0,
            grab_ms_ema=0.0, retrieve_ms_ema=0.0, total_ms_ema=0.0,
            fps=0.0, connected=False, opened=False, actual={}, ctrls={}, last_error="",
        )

    # ---------------------------------------------------------------- 生命周期
    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("source 只能 start 一次；close 后请新建实例")
        self._thread = threading.Thread(target=self._worker, name=f"usb:{self.label}", daemon=True)
        self._thread.start()

    def close(self, timeout: float = 3.0) -> None:
        self._stop.set()
        with self._cond:
            self._cond.notify_all()
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout)

    # ---------------------------------------------------------------- 内部
    def _open(self) -> cv2.VideoCapture:
        dev = self.device
        cap = (cv2.VideoCapture(dev, cv2.CAP_V4L2) if isinstance(dev, str)
               else cv2.VideoCapture(int(dev), cv2.CAP_V4L2))
        if not cap.isOpened():
            raise RuntimeError("打不开设备（检查路径/权限/是否被占用）")
        # 顺序有讲究：先 FOURCC，再分辨率，再帧率，最后 buffersize
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)      # 只要最新帧
        except Exception:
            pass
        self._actual = {
            "width": int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            "height": int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            "fps": round(float(cap.get(cv2.CAP_PROP_FPS)), 2),
            "fourcc": self._fourcc_str(cap.get(cv2.CAP_PROP_FOURCC)),
        }
        # 关键：下发具名控件（关动态帧率/曝光/防闪），否则实际帧率会被自动曝光砍掉
        self._ctrl_report = apply_v4l2_ctrls(self.device, self.ctrls)
        failed = {k: self._ctrl_report.get(k, "not configured")
                  for k in self.required_ctrls if self._ctrl_report.get(k) != "ok"}
        if failed:
            cap.release()
            raise RuntimeError(f"required V4L2 controls failed on {self.device}: {failed}")
        return cap

    @staticmethod
    def _fourcc_str(v) -> str:
        try:
            v = int(v)
            return "".join(chr((v >> (8 * i)) & 0xFF) for i in range(4))
        except Exception:
            return "?"

    def _worker(self) -> None:
        cap = None
        t_last = None
        while not self._stop.is_set():
            try:
                cap = self._open()
                with self._cond:
                    self._opened = True
                    self.stats["connected"] = True
                    self.stats["opened"] = True
                    self.stats["actual"] = dict(self._actual)
                    self.stats["ctrls"] = dict(self._ctrl_report)
                    self.stats["last_error"] = ""
                for _ in range(self.warmup):          # 预热丢帧
                    if not cap.grab():
                        break
                t_last = time.perf_counter()
                while not self._stop.is_set():
                    t0 = time.perf_counter()
                    ok = cap.grab()                   # 只取压缩数据，不做解码
                    t1 = time.perf_counter()
                    if not ok:
                        with self._cond:
                            self.stats["read_failures"] += 1
                        raise EOFError("grab 失败（拔线/带宽不足/被抢占）")
                    ok, bgr = cap.retrieve()          # 这里才是 JPEG 解码
                    t2 = time.perf_counter()
                    if not ok or bgr is None:
                        with self._cond:
                            self.stats["read_failures"] += 1
                        continue
                    bgr.flags.writeable = False       # 多线程只读共享
                    now = time.perf_counter()
                    dt = now - (t_last or now)
                    t_last = now
                    g_ms, r_ms, tot_ms = (t1 - t0) * 1e3, (t2 - t1) * 1e3, (t2 - t0) * 1e3
                    with self._cond:
                        s = self.stats
                        s["decoded_count"] += 1
                        s["grab_ms"], s["retrieve_ms"], s["total_ms"] = \
                            round(g_ms, 3), round(r_ms, 3), round(tot_ms, 3)
                        s["grab_ms_ema"] = round(g_ms if s["grab_ms_ema"] == 0 else
                                                 0.9 * s["grab_ms_ema"] + 0.1 * g_ms, 3)
                        s["retrieve_ms_ema"] = round(r_ms if s["retrieve_ms_ema"] == 0 else
                                                     0.9 * s["retrieve_ms_ema"] + 0.1 * r_ms, 3)
                        s["total_ms_ema"] = round(tot_ms if s["total_ms_ema"] == 0 else
                                                  0.9 * s["total_ms_ema"] + 0.1 * tot_ms, 3)
                        if dt > 0:
                            inst = 1.0 / dt
                            s["fps"] = round(inst if s["fps"] == 0 else
                                             0.9 * s["fps"] + 0.1 * inst, 2)
                        self._latest = SourceFrame(bgr, s["decoded_count"], now)
                        self._cond.notify_all()
            except Exception as exc:                  # 断线重连
                with self._cond:
                    if not self._stop.is_set():
                        self.stats["last_error"] = f"{type(exc).__name__}: {exc}"
                        self.stats["reconnects"] += 1
                        self.stats["connected"] = False
            finally:
                if cap is not None:
                    cap.release()
                    cap = None
                with self._cond:
                    self.stats["connected"] = False
            if not self._stop.is_set():
                self._stop.wait(self.reconnect_delay)

    # ---------------------------------------------------------------- 读帧
    def read_newer(self, last_sequence: int, timeout: float = 0.2) -> SourceFrame | None:
        """返回比 last_sequence 更新的一帧；超时返回 None（不阻塞、不排队）。"""
        deadline = time.perf_counter() + timeout
        with self._cond:
            while not self._stop.is_set():
                if self._latest is not None and self._latest.sequence > last_sequence:
                    return self._latest
                wait = deadline - time.perf_counter()
                if wait <= 0:
                    return None
                self._cond.wait(wait)
        return None

    def snapshot(self) -> dict:
        with self._cond:
            out = dict(self.stats)
            out["last_frame_age_ms"] = (
                round((time.perf_counter() - self._latest.received_mono) * 1000, 2)
                if self._latest else None)
            return out

    @property
    def is_open(self) -> bool:
        with self._cond:
            return self._opened


# ---------------------------------------------------------------------- 自测
def _main() -> int:
    ap = argparse.ArgumentParser(description="USB 摄像头最新帧源自测")
    ap.add_argument("--devices", default="/dev/video0")
    ap.add_argument("--size", default="1280x720")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--fourcc", default="MJPG")
    args = ap.parse_args()

    w, h = (int(v) for v in args.size.lower().split("x"))
    src = UsbCameraSource(args.devices.split(",")[0], w, h, args.fps, args.fourcc, label="selftest")
    src.start()
    time.sleep(1.5)
    print("实际协商:", json.dumps(src.snapshot()["actual"], ensure_ascii=False))

    seq, n, t0, last = 0, 0, time.perf_counter(), None
    while time.perf_counter() - t0 < args.seconds:
        f = src.read_newer(seq, timeout=0.2)
        if f is None:
            continue
        seq, n, last = f.sequence, n + 1, f
    dt = time.perf_counter() - t0

    snap = src.snapshot()
    print(f"收到 {n} 帧 / {dt:.2f}s = {n / dt:.2f} fps")
    print(f"grab {snap['grab_ms_ema']} ms + retrieve(解码) {snap['retrieve_ms_ema']} ms "
          f"= {snap['total_ms_ema']} ms")
    if last is not None:
        cv2.imwrite("usb_selftest.png", last.bgr)
        print("已保存 usb_selftest.png", last.bgr.shape)
    src.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
