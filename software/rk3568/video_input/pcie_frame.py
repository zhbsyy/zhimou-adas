#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pcie_frame.py —— 从 cap_reader.py 读帧，转成 numpy BGR 数组
============================================================
相对原版两处改动：

[FIX] 隔行黑修复（奇偶自适应）
    FPGA 每帧只有"每隔一行"有数据、另一行整行为 0，而且**有内容的是奇行还是
    偶行每次运行会翻**。这里按"哪一半非零"判断后，把有效行摊回 720 行。

[PERF] RGB565→BGR 提速 3.5 倍（板端实测原路径 72.7 ms/帧 → 预计 ~21 ms/帧）
    三个关键点：
      1) 原始字节本身就是小端 uint16，用 frombuffer 直接看成 (H,W) uint16
         —— 零拷贝，省掉原来 2 次 astype + 移位 + 或运算
      2) 65536×3 的查找表（196KB，只建一次），一次 np.take 出 BGR
         （实测三种查表写法里 np.take 最快：3.79 vs LUT[z] 7.73 ms）
      3) 隔行修复放在 565 域做，数据量只有 BGR 域的 2/3
    有 cv2 且带 COLOR_RGB5652BGR 时优先用 cv2 的 SIMD 专用路径（本板 OpenCV
    没暴露该常量，会自动跳过）。

用法：
    src = PcieFrameSource("/home/linaro/board_files/cap_reader.py")
    src.start()
    bgr = src.read_bgr()              # numpy uint8 (720,1280,3) BGR
    print(src.convert_ms, src.fixed_frames, src.mode)
    src.close()

    # 关掉隔行修复 / 强制走最慢的原始路径：
    #   PcieFrameSource(..., fix_interlace=False)
    #   PcieFrameSource(..., use_cv2=False, use_lut=False)
"""
import os
import subprocess
import sys
import threading
import time

import numpy as np

WIDTH, HEIGHT = 1280, 720

_LUT = None


def get_lut():
    """65536×3 的 RGB565→BGR 查找表（进程内只建一次，约 196KB）"""
    global _LUT
    if _LUT is None:
        idx = np.arange(65536, dtype=np.uint16)
        r = ((idx >> 11) & 0x1F).astype(np.uint16)
        g = ((idx >> 5) & 0x3F).astype(np.uint16)
        b = (idx & 0x1F).astype(np.uint16)
        lut = np.empty((65536, 3), np.uint8)
        lut[:, 2] = ((r << 3) | (r >> 2)).astype(np.uint8)     # R
        lut[:, 1] = ((g << 2) | (g >> 4)).astype(np.uint8)     # G
        lut[:, 0] = ((b << 3) | (b >> 2)).astype(np.uint8)     # B
        _LUT = lut
    return _LUT


class PcieFrameSource:
    """启动 cap_reader.py 子进程，把它 stdout 的 RGB565 流变成 numpy BGR 帧。"""

    def __init__(self, reader_path, width=WIDTH, height=HEIGHT,
                 fix_interlace=True, use_cv2=True, use_lut=True, check_every=30):
        self.reader_path = reader_path
        self.width = width
        self.height = height
        self.frame_bytes = width * height * 2
        self.fix_interlace = fix_interlace
        self.use_cv2 = use_cv2
        self.use_lut = use_lut
        self.check_every = max(1, int(check_every))
        self.fixed_frames = 0
        self.checked_frames = 0
        self.convert_ms = 0.0            # 最近一帧的转换耗时（含隔行修复）
        self._mode = None                 # 第一帧自动探测后填 "cv2:xxx" 或 "lut"
        self._cv2_code = None
        self._cv2_byteswap = False
        self._cv2_swap = False
        self.interlace_parity = -1       # 0=内容在偶行 1=内容在奇行 -1=正常
        self._parity_known = False       # 是否已经在"有内容的帧"上判定过
        self.proc = None
        self._lock = threading.Lock()

    @property
    def mode(self):
        return self._mode or "未定"

    # ---------- 生命周期 ----------
    def start(self):
        if self.proc is not None:
            return
        if not os.path.isfile(self.reader_path):
            raise FileNotFoundError(self.reader_path)
        cmd = ([sys.executable, self.reader_path] if self.reader_path.endswith(".py")
               else [self.reader_path])
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                     stderr=subprocess.PIPE,
                                     bufsize=self.frame_bytes * 2)

        def drain():
            for line in iter(self.proc.stderr.readline, b""):
                try:
                    sys.stderr.write(line.decode(errors="replace"))
                except Exception:
                    pass
        threading.Thread(target=drain, daemon=True).start()

    def close(self):
        if self.proc is not None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.proc.kill()
            self.proc = None

    # ---------- 取帧 ----------
    def read_raw(self):
        if self.proc is None:
            self.start()
        need = self.frame_bytes
        chunks = []
        with self._lock:
            while need > 0:
                chunk = self.proc.stdout.read(need)
                if not chunk:
                    raise EOFError("capture reader 已结束")
                chunks.append(chunk)
                need -= len(chunk)
        return b"".join(chunks)

    # ---------- 隔行判定 ----------
    def _check_interlace(self, z):
        """z: (H,W) uint16 视图。返回 0（内容在偶行）/1（在奇行）/-1（正常）

        ⚠️ 两条必须遵守的规则（都是 2026-10-06 在板子上踩出来的）：
        1. 【全零帧不参与判定】第 0 帧按设计就是空的（你们文档里的"坑 6"）。
           若让它占用判定机会，会得出"正常"的错误结论，而且接下来 30 帧
           都不再复查 → 隔行黑永远不会被修掉（实际就发生了这个 bug）。
        2. 【判定成功前每帧都查】真实启动序列是"先来一两帧全零，再来有内容的"，
           所以必须一直查到第一帧有内容的为止。
        判定确定后按 check_every 复查，因为奇偶在运行之间会翻。
        """
        need = (not self._parity_known) or (self.checked_frames >= self.check_every)
        if need and z.max() > 0:                 # 只在【有内容】的帧上判定
            ev, od = z[0::2], z[1::2]
            ev_ok, od_ok = ev.max() > 0, od.max() > 0
            if ev_ok and not od_ok:
                self.interlace_parity = 0        # 内容在偶数行
            elif od_ok and not ev_ok:
                self.interlace_parity = 1        # 内容在奇数行
            else:
                self.interlace_parity = -1       # 正常帧
            if not self._parity_known:
                self._parity_known = True
                sys.stderr.write("[pcie_frame] 隔行判定（首个有内容的帧）：%s\n"
                                 % {0: "内容在偶行 → 修复",
                                    1: "内容在奇行 → 修复",
                                    -1: "正常，无需修复"}[self.interlace_parity])
            self.checked_frames = 0
        self.checked_frames += 1
        return self.interlace_parity

    # ---------- 主入口 ----------
    def read_bgr(self):
        """读一帧并转成 numpy BGR（uint8, H×W×3）。"""
        raw = self.read_raw()
        t0 = time.perf_counter()

        # ★ 零拷贝：原始字节本身就是小端 uint16（FPGA 每像素 2 字节，低字节在前）
        z = np.frombuffer(raw, dtype="<u2").reshape(self.height, self.width)

        if self.fix_interlace:
            p = self._check_interlace(z)
            if p in (0, 1):
                z = np.repeat(z[p::2], 2, axis=0)   # 在 565 域复制，省内存带宽
                self.fixed_frames += 1

        bgr = self._convert(z)
        self.convert_ms = (time.perf_counter() - t0) * 1e3
        return bgr

    def _convert(self, z):
        if self._mode is None:
            return self._pick_method(z)
        if self._cv2_code is not None:
            return self._cv2_apply(z)
        return np.take(get_lut(), z, axis=0)

    def _cv2_apply(self, z):
        import cv2
        if self._cv2_byteswap:
            z = ((z & 0xFF) << 8) | (z >> 8)
        a = np.ascontiguousarray(z).view(np.uint8).reshape(z.shape[0], z.shape[1], 2)
        out = cv2.cvtColor(a, self._cv2_code)
        if self._cv2_swap:
            out = cv2.cvtColor(out, cv2.COLOR_BGR2RGB)
        return out

    def _pick_method(self, z):
        """在有内容的帧上把各条候选都试一遍，用查表结果当基准逐字节比对，谁对用谁。

        为什么这么写：
        ① 本板 OpenCV 只有 BGR565 系列、没有 RGB565 系列，而 565 的位序
           （R 在高位还是 B 在高位）光看名字容易猜错 —— 猜错就是红蓝反，
           而且【不会报错】，只会悄悄毁掉颜色。所以不猜：算出一定正确的
           查表结果，拿各候选去对，对上才采用。
        ② ⚠️ 必须只在【有内容的帧】上校验！全零帧上任何转换都输出全零，
           比对会平凡通过 → 假阳性（2026-10-06 实际踩到：14:27 报"核对通过"，
           一到真实画面两个候选就都不符）。
        """
        ref = np.take(get_lut(), z, axis=0)          # 基准：一定正确的慢路径
        if int(z.max()) == 0:
            return ref                               # 空帧不定案，等有内容的帧

        # 通道顺序要有色彩差异才判定得出来：灰度画面下"红蓝互换"的差异是 0，
        # 会得出错误结论。所以要求这一帧的 B 与 R 至少差 40。
        d_br = int(np.abs(ref[:, :, 0].astype(np.int16) -
                          ref[:, :, 2].astype(np.int16)).max())
        if d_br < 40:
            return ref

        try:
            import cv2
        except ImportError:
            self._mode = "lut"
            sys.stderr.write("[pcie_frame] 无 cv2 → 转换方式: 查表 np.take\n")
            return ref

        # 容差：565→888 有两种等价展开 —— "位复制"（本 LUT，31→255）与
        # "纯移位"（OpenCV，31→248），最大差 7~8。而位序猜错（红蓝反）差 100+，
        # 所以用 12 做阈值可以干净区分"展开方式不同"和"通道顺序错了"。
        TOL = 12

        def diff(a, b):
            return int(np.abs(a.astype(np.int16) - b.astype(np.int16)).max())

        variants = [("", z), ("+字节交换", ((z & 0xFF) << 8) | (z >> 8))]
        for vname, zz in variants:
            a = np.ascontiguousarray(zz).view(np.uint8).reshape(z.shape[0], z.shape[1], 2)
            for name in ("COLOR_BGR5652BGR", "COLOR_BGR5652RGB",
                         "COLOR_RGB5652BGR", "COLOR_RGB5652RGB"):
                code = getattr(cv2, name, None)
                if code is None:
                    continue
                try:
                    out = cv2.cvtColor(a, code)
                except Exception:
                    continue
                if out.shape != ref.shape:
                    continue
                d = diff(out, ref)
                if d <= TOL:
                    self._set_cv2(code, name + vname, False, bool(vname), d)
                    return out
                out2 = cv2.cvtColor(out, cv2.COLOR_BGR2RGB)   # 可能只是红蓝反了
                d2 = diff(out2, ref)
                if d2 <= TOL:
                    self._set_cv2(code, name + vname + "+BGR2RGB", True, bool(vname), d2)
                    return out2

        self._mode = "lut"
        sys.stderr.write("[pcie_frame] 所有 cv2 候选都与查表差异过大 → 用查表 np.take(LUT)\n")
        return ref

    def _set_cv2(self, code, label, swap, byteswap, d):
        self._cv2_code, self._cv2_swap, self._cv2_byteswap = code, swap, byteswap
        self._mode = "cv2:" + label
        sys.stderr.write("[pcie_frame] 转换方式: cv2 %s"
                         "（SIMD；与查表最大差 %d，属两种等价展开之差，通道顺序已验证正确）\n"
                         % (label, d))
def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    prefix = sys.argv[2] if len(sys.argv) > 2 else "shot"
    here = os.path.dirname(os.path.abspath(__file__))
    src = PcieFrameSource(os.path.join(here, "cap_reader.py"))
    src.start()
    try:
        for i in range(n):
            bgr = src.read_bgr()
            name = {0: "内容在偶行(已修复)", 1: "内容在奇行(已修复)", -1: "正常"}[src.interlace_parity]
            print("第 %d 帧: shape=%s 均值=%.1f 转换=%.1f ms 方式=%s 隔行=%s"
                  % (i, bgr.shape, bgr.mean(), src.convert_ms, src.mode, name))
            try:
                import cv2
                cv2.imwrite("%s%d.png" % (prefix, i), bgr)
                print("  已保存 %s%d.png" % (prefix, i))
            except ImportError:
                print("  (无 cv2，跳过存图)")
    finally:
        src.close()
    if src.fixed_frames:
        print("隔行黑修复生效：%d 帧" % src.fixed_frames)


if __name__ == "__main__":
    main()



