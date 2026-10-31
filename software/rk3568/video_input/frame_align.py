#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
frame_align.py —— 纠正 FPGA 输出帧的"垂直滚动"(roll)
=====================================================

【现象】
    抓到的帧整体上下滚动了若干行（每帧的滚动量都不同，因为 FPGA 自由运行、
    主机异步读取，相位持续漂移）：
      · 画面看起来错位（像 3 行而不是 2 行）
      · 检测框位置跟着偏

【主判据：接缝法（快且稳）】
    拼图的两个分界线（行 0 和行 360）处，行间差异会突然变大；
    而且因为上下两半重复，同一个接缝会在 +360 行处再出现一次。
    于是：
        g[y]     = 第 y 行与第 y-1 行的平均绝对差
        score[y] = g[y] + g[y+360]        ← 只有真正的接缝才会两处都大
        delta    = argmax(score)          ← 这就是滚动量
    纠正：frame[delta:] + frame[:delta]

【辅助判据：重复性（用于校验）】
    纠正后，frame[y] 与 frame[y+360] 的残差应显著下降。

用法：
    from frame_align import detect_roll, apply_roll
    roll = detect_roll(frame_bytes)          # 每帧调用（约 1ms）
    frame = apply_roll(frame_bytes, roll)
"""
import numpy as np

HALF = 360


def _rows(frame, w, h, sample):
    a = np.frombuffer(frame, dtype="<u2", count=w * h).reshape(h, w)
    return a[:, ::sample].astype(np.int32)


def detect_roll(frame, w=1280, h=720, sample=16, half=HALF, verify=True):
    """返回滚动行数 delta（0 表示不需要纠正）

    每帧调用一次（采样后只需 ~1ms）。
    """
    a = _rows(frame, w, h, sample)

    # ---- 接缝法 ----
    g = np.concatenate([[0.0], np.abs(a[1:] - a[:-1]).mean(axis=1)])  # g[y] 属于第 y 行
    pair = g[:h - half] + g[half:]
    pair[:2] = 0.0
    pair[-2:] = 0.0
    delta = int(np.argmax(pair))

    if not verify:
        return delta

    # ---- 校验：纠正前后残差对比（frame[y] vs frame[y+360]）----
    idx = np.arange(h)
    d = np.abs(a - a[(idx + half) % h]).mean(axis=1)

    def resid(rows):
        rows = list(rows)
        if not rows:
            return 1e18
        return float(d[rows].mean())

    # 纠正 delta 后，有效区间是 [delta, delta+half)（其余是"上一帧尾巴"）
    n = h - half - 20
    if n > 0:
        before = resid(range(delta, delta + n))
        # 不纠正（delta=0）时的残差
        zero = resid(range(0, n))
        # 如果纠正后反而更差，就放弃纠正
        if before > zero * 1.05:
            return 0
    return delta


def apply_roll(frame, roll, w=1280, h=720):
    """整体上移 roll 行（环形旋转），roll=0 时原样返回"""
    roll %= h
    if roll == 0:
        return frame
    off = roll * w * 2
    return frame[off:] + frame[:off]


def check_alignment(frame, w=1280, h=720, sample=16, half=HALF):
    """全局上下半平均差异（越小越对齐）"""
    a = _rows(frame, w, h, sample)
    idx = np.arange(h)
    return float(np.abs(a - a[(idx + half) % h]).mean())


if __name__ == "__main__":
    import sys
    path = sys.argv[1] if len(sys.argv) > 1 else "m.rgb565"
    data = open(path, "rb").read()
    print("文件: %s (%d 字节)" % (path, len(data)))

    a = _rows(data, 1280, 720, 16)
    g = np.concatenate([[0.0], np.abs(a[1:] - a[:-1]).mean(axis=1)])
    pair = g[:720 - 360] + g[360:]
    pair[:2] = 0.0
    pair[-2:] = 0.0
    order = np.argsort(-pair)[:6]
    print("\n接缝候选（g[y]+g[y+360] 最大的几个）:")
    for i in sorted(order):
        print("   y=%3d  score=%10.1f  (g=%.1f, g+360=%.1f)" % (i, pair[i], g[i], g[i + 360]))

    roll = detect_roll(data)
    print("\n最终滚动量: %d 行" % roll)
    print("纠正前全局残差: %.2f" % check_alignment(data))
    print("纠正后全局残差: %.2f" % check_alignment(apply_roll(data, roll)))

    # 逐帧稳定性演示（连续 5 帧的滚动量应该各自独立）
    print("\n提示：滚动量每帧都会变，抓帧程序里是【每帧都检测】的 ✓")
