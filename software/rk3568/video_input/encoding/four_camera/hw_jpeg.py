#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
hw_jpeg.py —— RK3568 硬件 JPEG(MJPEG) 编码，替换 cv2.imencode
================================================================
v2 修正（2026-10-06，针对板端首次试跑暴露的三个问题）：
  * 修掉"读满 64KB 才返回"的致命 bug：改用 os.read(fileno, n)，来多少读多少
    （v1 一帧 JPEG 只有 ~55KB < 64KB，于是永远读不出来 → 网页全黑）
  * 修掉 gst 存在性误判：用 shutil.which，而不是 `gst-launch-1.0 --version`（它不支持这个选项）
  * mppjpegenc 没有 quality 属性 → 启动时自动从 gst-inspect 里找真正的质量属性名
  * 首帧给更长超时（管线要预热），稳态超时 0.8s；连续失败 2 次就永久停用硬编（不再每帧卡 2 秒）
  * 构造时可以不传尺寸，按第一帧自动配置

板端实测参考（2026-10-06）：
    cv2.imencode('.jpg',1280x720,q85) ~ 42.7 ms/帧（随机噪声=最坏情况）
    mpph264enc 300 帧 720p 用时 3.23s（含 CPU 转换）
    mppjpegenc 的 sink caps 支持 BGR / RGB / RGB16 / BGR16 → 不需要任何颜色转换

用法（最小改动）
    from hw_jpeg import JpegEncoder
    enc = JpegEncoder(verbose=True)          # 尺寸按第一帧自动配置
    jpg = enc.encode_bgr(vis)                # bytes 或 None（失败时已自动回退 cv2）
    jpg = enc.encode_rgb565(raw_bytes)       # 直接吃 FPGA 的 RGB565 原始帧
    enc.close()

命令行自测（**必须用带 numpy/cv2 的那个 python**，系统 python3 没有 numpy）
    ~/yolov8_deploy/.venv-offline/bin/python hw_jpeg.py --bench --height 755
    ~/yolov8_deploy/.venv-offline/bin/python hw_jpeg.py --bench --fmt BGR16
"""
import argparse
import os
import queue
import re
import shutil
import subprocess
import sys
import threading
import time

import numpy as np          # ← v2.4 修正：顶层必须有（I420 路径用 np.ascontiguousarray）

SOI = b"\xff\xd8"          # JPEG 起始
EOI = b"\xff\xd9"          # JPEG 结束（熵编码段内 0xFF 会转义成 0xFF00，不会误判）

FMT_RGB565 = "RGB16"       # 16bit 5-6-5，R 在高位
FMT_BGR565 = "BGR16"       # 16bit 5-6-5，B 在高位
FMT_BGR888 = "BGR"
FMT_I420 = "I420"          # MPP 最成熟的输入格式，BGR 直通不通时自动切过来


class JpegEncoder:
    """常驻一个 gst-launch 子进程做硬件 JPEG 编码；坏掉就回退 cv2。"""

    def __init__(self, width=None, height=None, quality=85, fps=30,
                 verbose=True, timeout=0.8, first_timeout=2.0, max_fail=2,
                 props=None, prefer=None, shape="parse", depth=3, drain_wait=0.004,
                 blocksize=65536):
        self.w = width
        self.h = height
        self.q = quality
        self.fps = fps
        self.verbose = verbose
        self.timeout = timeout
        self.first_timeout = first_timeout
        self.max_fail = max_fail
        self.shape = (shape or "parse").lower()   # parse=加 rawvideoparse；direct=整帧直喂
        self._has_parse = None
        # 额外要传给 mppjpegenc 的属性，例如 ["rc-mode=fixqp"]，用来做实验
        self.props = [p.strip() for p in (props or []) if p and p.strip()]
        # 输入格式偏好：默认 I420（MPP 最成熟）。实测 BGR 直通在本板 MPP 上
        # 管线能起但不出图（3 秒超时），所以默认不走 BGR，只有在 --fmt BGR 时才试。
        self.prefer = (prefer or FMT_I420).upper()
        self._err_logged = 0

        self._fmt = None
        self._proc = None
        self._out_q = queue.Queue(maxsize=4)
        self._lock = threading.Lock()
        self._reader = None
        self._fail = 0
        self._got_since_start = False
        self._qprop = None
        self._qprop_checked = False
        self._use_i420 = False          # 兼容旧字段
        self._in_q = None
        self._writer = None
        self.depth = max(1, int(depth))  # 输入侧最多预投喂几帧（流水线深度）
        self.blocksize = int(blocksize)  # fdsrc 每次读多少字节（性能关键，见 _pipeline）
        self.drain_wait = drain_wait     # collect() 最多等多久（秒）
        self._n_recv = 0
        self._last_jpg = None
        self._last_ok = 0.0
        self._t_start = 0.0
        self._t_first_jpg = 0.0
        self._t_last_jpg = 0.0
        self.stats = {"hw": 0, "sw": 0, "hw_ms": 0.0, "sw_ms": 0.0,
                      "submitted": 0, "cvt_ms": 0.0, "produced": 0, "collect_ms": 0.0}

        self._gst = shutil.which("gst-launch-1.0")
        if not self._gst:
            self._log("系统里没有 gst-launch-1.0 → 只能走 cv2 软编")

    # ---------------------------------------------------------------- 工具
    def _log(self, msg):
        if self.verbose:
            sys.stderr.write("[hw_jpeg] %s\n" % msg)
            sys.stderr.flush()

    def _detect_quality_prop(self):
        """mppjpegenc 的质量属性真名是 `q-factor`（1~99，默认 80）。
        不同版本可能叫别的名字，这里从 gst-inspect 里自动找。
        注意：必须先把属性名里的连字符/下划线去掉再匹配，否则 `q-factor` 匹配不到。"""
        if self._qprop_checked:
            return self._qprop
        self._qprop_checked = True
        # 注意：查属性必须用 gst-inspect-1.0，不能用 gst-launch-1.0
        # （用 gst-launch-1.0 mppjpegenc 会把 "mppjpegenc" 当管线名，结果是查不到的）
        insp = shutil.which("gst-inspect-1.0")
        if not insp:
            self._log("没有 gst-inspect-1.0，跳过质量属性探测（用默认 q-factor）")
            return None
        try:
            out = subprocess.run([insp, "mppjpegenc"], capture_output=True,
                                 timeout=8).stdout.decode(errors="replace")
        except Exception as e:
            self._log("gst-inspect 失败: %s" % e)
            return None
        cands = []
        for line in out.splitlines():
            m = re.match(r"^\s{2,}([A-Za-z0-9_\-]+)\s*:\s", line)
            if not m:
                continue
            name = m.group(1)
            norm = re.sub(r"[^a-z0-9]", "", name.lower())   # q-factor -> qfactor
            for pri, key in enumerate(("qfactor", "quality", "quant")):
                if key in norm:
                    cands.append((pri, name))
                    break
        if cands:
            self._qprop = sorted(cands)[0][1]
            self._log("找到质量属性：%s" % self._qprop)
            return self._qprop
        self._log("mppjpegenc 没有质量属性（画质用默认值 q-factor=80）")
        return None

    # ---------------------------------------------------------------- 管线
    def _have_rawvideoparse(self):
        if self._has_parse is None:
            insp = shutil.which("gst-inspect-1.0")
            ok = False
            if insp:
                try:
                    ok = subprocess.run([insp, "rawvideoparse"], capture_output=True,
                                        timeout=8).returncode == 0
                except Exception:
                    ok = False
            self._has_parse = ok
            if ok:
                self._log("检测到 rawvideoparse（用于把字节流重新打成帧）")
            else:
                self._log("没有 rawvideoparse，只能用整帧 blocksize 直喂")
        return self._has_parse

    def _pipeline(self, fmt, w, h):
        if fmt in (FMT_RGB565, FMT_BGR565):
            nbytes = w * h * 2
        elif fmt == FMT_I420:
            nbytes = (w * h * 3) // 2
        else:
            nbytes = w * h * 3

        # shape=parse：fdsrc 每次读 blocksize 字节，由 rawvideoparse 把字节流重新
        #              聚合成 1280x720 的帧 —— 对"管道短读"天然免疫
        # shape=direct：fdsrc 直接按整帧 blocksize 读（只在文件/不短读时可靠）
        #
        # blocksize 直接决定"每帧要过多少个缓冲"，是性能关键：
        #   4096  → 每帧 337 个缓冲，光分配+拷贝就吃掉 30+ ms（板端实测 58.8 ms/帧）
        #   65536 → 每帧 21 个缓冲（裸管线对照实测 23 ms/帧）
        # 有 rawvideoparse 在时缓冲大小不影响正确性，只影响性能，所以往大取。
        use_parse = (self.shape == "parse") and self._have_rawvideoparse()
        gfmt = {"I420": "i420", "BGR": "bgr", "RGB16": "rgb16", "BGR16": "bgr16"}.get(fmt, fmt.lower())

        bs = self.blocksize if use_parse else nbytes
        parts = [self._gst, "-q", "fdsrc", "fd=0", "blocksize=%d" % bs, "!"]
        if use_parse:
            parts += ["rawvideoparse", "format=%s" % gfmt, "width=%d" % w,
                      "height=%d" % h, "framerate=%d/1" % self.fps, "!"]
        else:
            parts += ["video/x-raw,format=%s,width=%d,height=%d,framerate=%d/1"
                      % (fmt, w, h, self.fps), "!"]
        parts.append("mppjpegenc")
        used = set()
        for p in self.props:                       # 用户显式给的优先
            used.add(p.split("=")[0].strip())
            parts.append(p)
        qp = self._detect_quality_prop()
        if qp and qp not in used:
            parts.append("%s=%d" % (qp, self.q))
        parts += ["!", "fdsink", "fd=1", "sync=false"]   # sync=false：别等时钟，立刻吐字节
        return parts

    def _write_loop(self, proc):
        """独立线程把输入帧写进 gst 的 stdin。

        ⚠️ v2.6 关键修正：Popen(bufsize=0) 时 `proc.stdin` 是原始 FileIO，
        `write()` **只做一次系统调用就返回**，而管道一次最多接收 64KB ——
        一帧 I420 有 1.38MB，会被**静默截断**（返回值被丢掉，表面看"写成功了"）。
        必须用 os.write + 循环写满。这就是"单测都通、合起来超时"的根因。
        """
        fd = proc.stdin.fileno()
        while True:
            try:
                item = self._in_q.get(timeout=0.5)
            except queue.Empty:
                if self._proc is not proc:
                    break
                continue
            if item is None:
                break
            total = 0
            try:
                view = memoryview(item)
                while len(view):
                    n = os.write(fd, view)
                    if n <= 0:
                        raise IOError("write 返回 %d" % n)
                    view = view[n:]
                    total += n
            except Exception as e:
                self._log("写管线失败: %s（本次已写入 %d/%d 字节）" % (e, total, len(item)))
                break

    def _start(self, fmt, w, h):
        if self._fail >= self.max_fail or not self._gst:
            return False
        self._stop()
        cmd = self._pipeline(fmt, w, h)
        try:
            proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                    stderr=subprocess.PIPE, bufsize=0)
        except Exception as e:
            self._log("启动 gst 失败: %s" % e)
            self._fail += 1
            return False

        t0 = time.time()
        while time.time() - t0 < 0.5:
            if proc.poll() is not None:
                err = proc.stderr.read().decode(errors="replace").strip()[-500:]
                self._log("gst 立刻退出：%s" % err)
                self._log("失败的命令：%s" % " ".join(cmd))
                proc.kill()
                self._fail += 1
                return False
            time.sleep(0.05)

        self._proc = proc
        self._fmt = (fmt, w, h)
        self._got_since_start = False
        self._in_q = queue.Queue(maxsize=self.depth)
        self._t_start = time.time()
        self._reader = threading.Thread(target=self._read_loop, args=(proc,), daemon=True)
        self._reader.start()
        self._writer = threading.Thread(target=self._write_loop, args=(proc,), daemon=True)
        self._writer.start()
        self._log("硬编管线已启动: %s" % " ".join(cmd))
        return True

    def _stop(self):
        p, self._proc = self._proc, None
        try:
            if self._in_q is not None:
                self._in_q.put_nowait(None)       # 通知写线程退出
        except Exception:
            pass
        if p is not None:
            for fn in (lambda: p.stdin.close(), lambda: p.terminate()):
                try:
                    fn()
                except Exception:
                    pass
            try:
                p.wait(timeout=1)
            except Exception:
                try:
                    p.kill()
                except Exception:
                    pass

    def _read_loop(self, proc):
        """从 gst 的 stdout 按 FFD8…FFD9 切出完整 JPEG。

        关键：必须用 os.read，一次系统调用"有多少读多少"。
        用 proc.stdout.read(N) 会一直等到凑满 N 字节 —— 一帧 JPEG 只有 ~55KB，
        小于 64KB 就永远读不出来（这是 v1 网页全黑的原因）。
        """
        fd = proc.stdout.fileno()
        buf = bytearray()
        while True:
            try:
                chunk = os.read(fd, 65536)
            except OSError:
                break
            if not chunk:
                break
            buf += chunk
            while True:
                s = buf.find(SOI)
                if s < 0:
                    if len(buf) > 1:
                        del buf[:-1]
                    break
                e = buf.find(EOI, s + 2)
                if e < 0:
                    if s > 0:
                        del buf[:s]
                    break
                jpg = bytes(buf[s:e + 2])
                del buf[:e + 2]
                self.stats["produced"] += 1     # 在源头计数：真实产量（不受队列丢弃影响）
                self._last_ok = time.time()
                if self.stats["produced"] == 1:
                    self._t_first_jpg = self._last_ok      # 首帧延迟（含进程启动/预热）
                self._t_last_jpg = self._last_ok           # 末帧时间（用来算持续吞吐）
                try:
                    self._out_q.put_nowait(jpg)
                except queue.Full:
                    try:
                        self._out_q.get_nowait()
                    except queue.Empty:
                        pass
                    try:
                        self._out_q.put_nowait(jpg)
                    except queue.Full:
                        pass

    # ---------------------------------------------------------------- 编码
    #  v2.7 改成"提交 + 取最新"的流水线模型。
    #  为什么：板端实测同一条 gst 管线，一次性喂 20 帧只要 23 ms/帧，
    #  但"投一帧→死等这一帧→再投下一帧"要 70 ms/帧 —— 因为同步等待把
    #  【往返延迟】变成了【每帧吞吐】。流水线化以后吞吐回到编码器真实水平。
    def _submit_hw(self, data, fmt, w, h):
        """非阻塞提交一帧（队列满则丢最旧）。返回 True 表示已交给写线程。"""
        with self._lock:
            if self._fmt != (fmt, w, h) or self._proc is None or self._proc.poll() is not None:
                if not self._start(fmt, w, h):
                    return False
            try:
                self._in_q.put_nowait(data)
            except queue.Full:
                try:
                    self._in_q.get_nowait()
                except queue.Empty:
                    pass
                try:
                    self._in_q.put_nowait(data)
                except queue.Full:
                    return False
            self.stats["submitted"] += 1
            return True

    def collect(self, wait=0.0):
        """取回【最新】的一张 JPEG（把队列里积压的都取掉，只留最新那张）。

        返回 None 只代表"本帧还没编出来"，**不代表失败** ——
        是否真的坏了请看 self.broken。
        """
        got, pending = None, 0
        t_c0 = time.perf_counter()
        t_end = time.time() + wait
        while True:
            try:
                got = self._out_q.get_nowait()
                pending += 1
                self.stats["hw"] += 1
                self._last_ok = time.time()
                self._fail = 0
            except queue.Empty:
                if (got is not None and pending > 0) or time.time() >= t_end:
                    break
                time.sleep(0.001)
        self.stats["collect_ms"] += (time.perf_counter() - t_c0) * 1e3
        if got is not None:
            self._last_jpg = got
            self._n_recv += 1
        return got

    @property
    def broken(self):
        """硬编是否已经不可用（调用方据此决定要不要退回 cv2）"""
        if self._fail >= self.max_fail:
            return True
        # 提交了若干帧却一张都没回来 → 认为起不来
        if (self.stats["submitted"] >= 4 and self._n_recv == 0
                and self._t_start and time.time() - self._t_start > self.first_timeout):
            return True
        return False

    # ---- 对外入口 ----
    #  submit_xxx()：只投递，不等待（流水线用）
    #  encode_xxx()：投递 + 取回一张最新 JPEG（普通调用用）
    def _config_size(self, w, h):
        if self.w is None or self.h is None:
            self.w, self.h = w, h
            self._log("按第一帧配置尺寸：%dx%d" % (w, h))
        elif (w, h) != (self.w, self.h):
            self._log("尺寸变了（%dx%d → %dx%d），重建管线" % (self.h, self.w, h, w))
            with self._lock:
                self._stop()
            self.w, self.h = w, h
            self._fail = 0

    def submit_bgr(self, arr):
        """投递一帧（BGR uint8 HxWx3），不等待结果。返回 True=已交给管线"""
        try:
            h, w = arr.shape[:2]
            self._config_size(w, h)
            if self.prefer == FMT_BGR888:
                data = arr if arr.flags["C_CONTIGUOUS"] else arr.copy()
                if self._submit_hw(memoryview(data).cast("B"), FMT_BGR888, w, h):
                    return True
                self.prefer = FMT_I420          # BGR 起不来 → 永久切 I420
                self._fail = 0
                self._log("BGR 直通不可用 → 自动切到 I420（cv2 转色 + 硬编）")
            import cv2
            if h % 2 or w % 2:                  # I420 需要偶数宽高
                arr = np.ascontiguousarray(arr[:h - (h % 2), :w - (w % 2)])
            t0 = time.perf_counter()
            yuv = cv2.cvtColor(arr, cv2.COLOR_BGR2YUV_I420)     # (h*3/2, w)
            self.stats["cvt_ms"] += (time.perf_counter() - t0) * 1e3
            hh, ww = arr.shape[:2]
            data = yuv.reshape(-1)
            return self._submit_hw(memoryview(data).cast("B"), FMT_I420, ww, hh)
        except Exception as e:
            self._err_logged += 1
            if self._err_logged <= 3:
                self._log("submit_bgr 异常: %s" % e)
            return False

    def submit_rgb565(self, raw, width=None, height=None):
        """投递一帧 RGB565 原始数据（w*h*2 字节），不等待。返回 True=已交给管线"""
        try:
            w = width or self.w
            h = height or self.h
            if not w or not h:
                self._log("submit_rgb565 需要先知道尺寸")
                return False
            need = w * h * 2
            if len(raw) < need:
                self._log("RGB565 数据不足：%d < %d" % (len(raw), need))
                return False
            self._config_size(w, h)
            return self._submit_hw(raw[:need], FMT_RGB565, w, h)
        except Exception as e:
            self._log("submit_rgb565 异常: %s" % e)
            return False

    def encode_bgr(self, arr):
        """投递一帧并取回最新 JPEG。

        返回 None 有两种含义：① 这一帧还没编出来（正常，继续投下一帧即可）；
        ② 硬编已判死（此时 self.broken 为 True，调用方应改走软件兜底）。
        """
        if not self.submit_bgr(arr):
            return self._sw_fallback(arr) if self.broken else None
        jpg = self.collect(self.drain_wait)
        if jpg is None and self.broken:
            return self._sw_fallback(arr)
        return jpg

    def encode_rgb565(self, raw, width=None, height=None):
        """投递 RGB565 并取回最新 JPEG（语义同 encode_bgr）"""
        if not self.submit_rgb565(raw, width, height):
            return None
        return self.collect(self.drain_wait)

    def _sw_fallback(self, bgr):
        try:
            import cv2
            t0 = time.perf_counter()
            ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, self.q])
            self.stats["sw"] += 1
            self.stats["sw_ms"] += (time.perf_counter() - t0) * 1e3
            return buf.tobytes() if ok else None
        except Exception as e:
            self._log("cv2 回退也失败: %s" % e)
            return None

    def close(self):
        self._stop()

    def report(self):
        s = self.stats
        hw = (s["hw_ms"] / s["hw"]) if s["hw"] else 0.0
        sw = (s["sw_ms"] / s["sw"]) if s["sw"] else 0.0
        cvt = (s["cvt_ms"] / s["submitted"]) if s["submitted"] else 0.0
        extra = ""
        if s["produced"] >= 2 and self._t_first_jpg and self._t_last_jpg:
            extra = (" | 首帧延迟 %.0f ms | **持续吞吐 %.1f ms/帧**"
                     % ((self._t_first_jpg - self._t_start) * 1e3,
                        (self._t_last_jpg - self._t_first_jpg) * 1e3 / (s["produced"] - 1)))
        return ("提交 %d 帧 | 硬编产出 %d 张 | 软编兜底 %d 张 | 转色 %.2f ms/帧%s%s"
                % (s["submitted"], s["produced"], s["sw"], cvt, extra,
                   ("  ⚠ 已判定不可用" if self.broken else "")))


# -------------------------------------------------------------------- 自测
def bench(fmt="I420", n=20, quality=85, save="/tmp", width=1280, height=720, props=None, shape="parse"):
    import numpy as np
    w, h = width, height
    y, x = np.mgrid[0:h, 0:w]
    r = (x // 6) % 256
    g = (y // 5) % 256
    b = ((x + y) // 9) % 256
    bgr = np.ascontiguousarray(np.dstack([b, g, r]).astype("uint8"))
    raw565 = ((((r >> 3).astype("<u2")) << 11) | (((g >> 2).astype("<u2")) << 5)
              | (b >> 3).astype("<u2")).tobytes()

    print("=" * 66)
    print(" 硬编 vs 软编   %dx%d  q=%d  输入=%s  props=%s" % (w, h, quality, fmt, props or "-"))
    print(" python: %s" % sys.executable)
    print("=" * 66)

    try:
        import cv2
        t0 = time.perf_counter()
        for _ in range(n):
            ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
        sw_ms = (time.perf_counter() - t0) * 1e3 / n
        cv2.imwrite(os.path.join(save, "bench_sw.jpg"), bgr)
        print("  cv2 软编      : %7.2f ms/帧   %d 字节  → %s/bench_sw.jpg"
              % (sw_ms, len(buf), save))
    except Exception as e:
        sw_ms = float("nan")
        print("  cv2 软编      : 不可用 (%s)" % e)

    enc = JpegEncoder(width=None, height=None, quality=quality, props=props,
                      prefer=("BGR" if fmt == "BGR" else "I420"), shape=shape,
                      depth=max(n, 4))          # 队列开大：先把 N 帧全投进去，量真实吞吐
    t0 = time.perf_counter()
    got, last = 0, None
    # 阶段1：连续投 N 帧（不等待）——这才是流水线该有的喂法
    for _ in range(n):
        if fmt in ("RGB16", "BGR16"):
            enc.submit_rgb565(raw565, w, h)
        else:
            enc.submit_bgr(bgr)
    # 阶段2：等它把 N 张编完（按"源头产量"计，不受输出队列丢弃影响）
    deadline = time.time() + max(20.0, n * 0.5)
    while enc.stats["produced"] < n and time.time() < deadline:
        jpg = enc.collect(0.05)
        if jpg:
            last = jpg
        got = enc.stats["produced"]
    wall = (time.perf_counter() - t0) * 1e3
    st = enc.stats
    sw_n = st["sw"]
    print("  投 %d 帧 → 硬编产出 %d 张" % (n, got))
    print("  ├ 硬编产出    : %d 张" % got)
    print("  └ 软编兜底    : %d 张" % sw_n)
    if got >= 2 and enc._t_first_jpg and enc._t_last_jpg:
        startup = (enc._t_first_jpg - enc._t_start) * 1e3
        sustained = (enc._t_last_jpg - enc._t_first_jpg) * 1e3 / (got - 1)
        print("  ★ 首帧延迟    : %7.1f ms（gst 进程启动 + 首帧聚合 + 编码器预热）" % startup)
        print("  ★ 持续吞吐    : %7.2f ms/帧  ← **这才是编码器真实能力**" % sustained)
        print("  （总平均 %.1f ms/帧 = 启动 + 持续；帧数少时启动占比极大，别用它下结论）"
              % (wall / got))
        print("  对比 cv2 软编 : %7.2f ms/帧（同步口径，不含启动）" % sw_ms)
    elif got:
        print("  硬编总平均    : %7.2f ms/帧（只有 %d 张，样本太少）" % (wall / got, got))
    print("  细分          : 转色 %.2f ms/帧 | collect %.2f ms/帧"
          % (st["cvt_ms"] / max(st["submitted"], 1),
             st["collect_ms"] / max(st["submitted"], 1)))
    if sw_n:
        print("  ⚠ 有 %d 帧走了 cv2 兜底 → 硬编这条路还没通" % sw_n)

    if last:
        p = os.path.join(save, "bench_hw.jpg")
        open(p, "wb").write(last)
        try:
            import cv2
            im = cv2.imread(p)
            print("  硬编出图 → %s   OpenCV 能读: %s" % (p, None if im is None else im.shape))
        except Exception:
            print("  硬编出图 → %s" % p)
        print("\n  对比 bench_hw.jpg 与 bench_sw.jpg：颜色一致=参数对；红蓝反了就把 --fmt %s 换成 %s"
              % (fmt, "BGR16" if fmt == "RGB16" else "RGB16"))
    if sw_ms == sw_ms and got:
        print("\n  对比：cv2 软编 %.1f ms/帧（同步口径）" % sw_ms)
    if got and sw_ms == sw_ms:
        print("  画质提示：MPP 的 q-factor 是 1~99（默认 80）。请对比两个 jpg 的字节数：")
        print("           硬编明显偏小 → q-factor 含义与 cv2 quality 相反，把 --quality 传 100-值 试试")
    print("  编码器统计：%s" % enc.report())
    enc.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bench", action="store_true")
    ap.add_argument("--fmt", default="I420", choices=["I420", "BGR", "RGB16", "BGR16"],
                    help="I420=默认(cv2转色+硬编，最稳)；BGR=BGR直通(本板实测不通，仅用于验证)；"
                         "RGB16/BGR16=直接吃 1280x720x2 的 RGB565")
    ap.add_argument("--frames", type=int, default=20)
    ap.add_argument("--quality", type=int, default=85)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720, help="draw() 加了标题栏后实际是 755")
    ap.add_argument("--out", default="/tmp")
    ap.add_argument("--shape", default="parse", choices=["parse", "direct"],
                    help="parse=加 rawvideoparse 重新打帧（默认，对付管道短读）；direct=整帧直喂")
    ap.add_argument("--props", default="",
                    help="额外传给 mppjpegenc 的属性，逗号分隔，例如 rc-mode=fixqp,q-factor=85")
    a = ap.parse_args()
    props = [p for p in a.props.split(",") if p.strip()] if a.props else None
    if a.bench:
        bench(a.fmt, a.frames, a.quality, a.out, a.width, a.height, props, a.shape)
    else:
        ap.print_help()


if __name__ == "__main__":
    main()



