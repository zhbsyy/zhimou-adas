#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
stereo_range.py —— 实时双目测距模块（供 pcie_stream.py 调用）  v2
====================================================================
v2 相对 v1 的三处改进（都来自 2026-10-07 实测日志）：
  ① 【双向一致性验证】正向匹配完，再从右图回搜左图，往返误差 >2px 的判为错匹配并剔除
     —— 实测日志里出现过 3.82m / 0.27m 这种离谱值（dx 相差 14 倍），就是错匹配漏网
  ② 【时间中值 + 运动门控】每个目标维护最近 7 帧的距离取中值；
     目标框位移过大时标 moving（运动时距离不可信 —— 两个相机曝光不同步，物理限制）
  ③ 【范围门控】Z 不在 0.2~8m 的直接剔除

依赖：stereo_8params.json（stereo_calibrate.py 生成，H 必须含 R_rect）
自测：python stereo_range.py --calib stereo_8params.json --selftest
"""
import json
import os
from collections import deque

import cv2
import numpy as np

TILE_W, TILE_H = 640, 360


def _iou(a, b):
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    iw, ih = max(0.0, x2 - x1), max(0.0, y2 - y1)
    inter = iw * ih
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 1e-6 else 0.0


class _Track:
    """一个目标的时间历史（用于中值滤波 + 运动判定）"""
    __slots__ = ("cls", "box", "zs", "last", "miss", "moving")

    def __init__(self, cls, box):
        self.cls = cls
        self.box = list(box)
        self.zs = deque(maxlen=7)
        self.last = 0
        self.miss = 0
        self.moving = False


class StereoRanger:
    def __init__(self, calib_path, min_score=0.55, max_dy=3.0,
                 patch=36, search=64, max_pairs=12,
                 z_min=200.0, z_max=8000.0, rt_tol=2.0,
                 move_px=6.0, smooth=True):
        d = json.load(open(calib_path, encoding="utf-8"))
        for k in ("H_L", "H_R", "f_px", "baseline_mm"):
            if k not in d:
                raise ValueError("标定文件缺少 %s" % k)
        self.HL = np.array(d["H_L"], float)
        self.HR = np.array(d["H_R"], float)
        self.f = float(d["f_px"])
        self.B = float(d["baseline_mm"])
        self.rms = float(d.get("rms_stereo_px", -1))
        self.dy_cal = float(d.get("rectify_dy_max_px", -1))
        self.min_score, self.max_dy = min_score, max_dy
        self.patch, self.search, self.max_pairs = patch, search, max_pairs
        self.z_min, self.z_max, self.rt_tol = z_min, z_max, rt_tol
        self.move_px, self.smooth = move_px, smooth
        self.tracks = {}
        self.frame_i = 0
        self.n_solved = self.n_rejected = self.n_badmatch = self.n_moving = 0

    # ---------------- 基础 ----------------
    def _rect(self, H, x, y):
        v = H @ np.array([x, y, 1.0])
        return v[0] / v[2], v[1] / v[2]

    def _z_from(self, xL, yL, xR, yR):
        dx = xL - xR
        if dx <= 1e-6:
            return None
        Z = self.f * self.B / dx
        return dict(X=xL * Z / self.f, Y=(yL + yR) / 2.0 * Z / self.f,
                    Z=Z, dx=dx, dy=abs(yL - yR))

    # ---------------- 模板匹配（含双向验证） ----------------
    def _match_once(self, tpl, roi):
        res = cv2.matchTemplate(roi, tpl, cv2.TM_CCOEFF_NORMED)
        _, maxv, _, (mx, my) = cv2.minMaxLoc(res)

        def sub(vals, i):
            if i <= 0 or i >= len(vals) - 1:
                return float(i)
            a, b, c = vals[i - 1], vals[i], vals[i + 1]
            den = a - 2 * b + c
            return float(i) + (0.5 * (a - c) / den if abs(den) > 1e-9 else 0.0)

        return sub(res[my, :], mx), sub(res[:, mx], my), float(maxv)

    def _refine(self, L, R, boxL, boxR):
        """左->右匹配，再右->左回搜做一致性验证。返回 (xL,yL,xR,yR,score,rt_err)"""
        h, w = L.shape[:2]
        cxL, cyL = (boxL[0] + boxL[2]) / 2.0, (boxL[1] + boxL[3]) / 2.0
        cxR = (boxR[0] + boxR[2]) / 2.0
        pw = int(min(self.patch, max(14, boxL[2] - boxL[0])))
        ph = int(min(self.patch, max(14, boxL[3] - boxL[1])))
        x0, y0 = int(round(cxL - pw / 2)), int(round(cyL - ph / 2))
        if x0 < 0 or y0 < 0 or x0 + pw > w or y0 + ph > h:
            return None
        tpl = L[y0:y0 + ph, x0:x0 + pw]

        # --- 正向：在右图找 ---
        sx0, sx1 = max(0, int(cxR) - self.search), min(w, int(cxR) + self.search)
        sy0, sy1 = max(0, int(round(cyL)) - ph), min(h, int(round(cyL)) + ph)
        if sx1 - sx0 < pw or sy1 - sy0 < ph:
            return None
        rx, ry, score = self._match_once(tpl, R[sy0:sy1, sx0:sx1])
        xR = sx0 + rx + pw / 2.0
        yR = sy0 + ry + ph / 2.0

        # --- 反向：拿右图该处图块，回左图搜（左-右一致性检查，能滤掉绝大多数错匹配）---
        rt_err = -1.0
        bx0, by0 = int(round(xR - pw / 2)), int(round(yR - ph / 2))
        if 0 <= bx0 and 0 <= by0 and bx0 + pw <= w and by0 + ph <= h:
            tpl2 = R[by0:by0 + ph, bx0:bx0 + pw]
            tx0, tx1 = max(0, int(cxL) - self.search), min(w, int(cxL) + self.search)
            ty0, ty1 = max(0, int(round(yR)) - ph), min(h, int(round(yR)) + ph)
            if tx1 - tx0 >= pw and ty1 - ty0 >= ph:
                bx, by, _ = self._match_once(tpl2, L[ty0:ty1, tx0:tx1])
                back_x = tx0 + bx + pw / 2.0
                back_y = ty0 + by + ph / 2.0
                rt_err = float(np.hypot(back_x - cxL, back_y - cyL))
        return cxL, cyL, xR, yR, score, rt_err

    # ---------------- 单对求解 ----------------
    def solve_pair(self, L, R, boxL, boxR):
        m = self._refine(L, R, boxL, boxR)
        if m is None:
            self.n_rejected += 1
            return None
        pxL, pyL, pxR, pyR, score, rt = m
        if score < self.min_score:
            self.n_rejected += 1
            return None
        if rt >= 0 and rt > self.rt_tol:          # 双向不一致 -> 错匹配
            self.n_badmatch += 1
            self.n_rejected += 1
            return None
        xL, yL = self._rect(self.HL, pxL, pyL)
        xR, yR = self._rect(self.HR, pxR, pyR)
        r = self._z_from(xL, yL, xR, yR)
        if r is None or not (self.z_min <= r["Z"] <= self.z_max):
            self.n_rejected += 1
            return None
        r.update(score=score, rt=rt, method="tm", Z_raw=r["Z"],
                 boxL=(pxL, pyL), boxR=(pxR, pyR))
        if r["dy"] > self.max_dy:
            r["suspect"] = True
            self.n_badmatch += 1
            self.n_rejected += 1
        else:
            r["suspect"] = False
        self.n_solved += 1
        return r

    # ---------------- 时间滤波 ----------------
    def _track(self, cls, box, Z, moving):
        best_iou, key = 0.30, None
        for k, t in self.tracks.items():
            if t.cls != cls or self.frame_i - t.last > 3:
                continue
            v = _iou(box, t.box)
            if v > best_iou:
                best_iou, key = v, k
        if key is None:
            key = max(self.tracks.keys(), default=0) + 1
            self.tracks[key] = _Track(cls, box)
        t = self.tracks[key]
        t.moving = moving
        t.box = list(box)
        t.last = self.frame_i
        if not moving or not self.smooth:
            t.zs.append(float(Z))
        elif len(t.zs) == 0:
            t.zs.append(float(Z))
        return key, t

    # ---------------- 自动配对 ----------------
    @staticmethod
    def _center(b):
        return (b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0

    def solve_pairs(self, frame, dets):
        """dets: (N,6) [x1,y1,x2,y2,conf,cls]（1280x720 拼图坐标）
        返回 [(左目索引, 结果dict)]；结果里 Z 是时间中值，Z_raw 是本帧原始值"""
        self.frame_i += 1
        for k in [k for k, t in self.tracks.items() if self.frame_i - t.last > 10]:
            del self.tracks[k]
        if dets is None or len(dets) == 0:
            return []
        L = frame[0:TILE_H, 0:TILE_W]
        R = frame[0:TILE_H, TILE_W:2 * TILE_W]
        left = [i for i, d in enumerate(dets) if (d[0] + d[2]) / 2.0 < TILE_W]
        right = [i for i, d in enumerate(dets) if (d[0] + d[2]) / 2.0 >= TILE_W]

        cand = []
        for i in left:
            cxl, cyl = self._center(dets[i])
            for j in right:
                cxr, cyr = self._center(dets[j])
                if int(dets[i][5]) != int(dets[j][5]):
                    continue
                dy = abs(cyl - cyr)
                dx = cxl - (cxr - TILE_W)
                if dy > 45 or dx < -20 or dx > 200:
                    continue
                cand.append((dy + (0.0 if dx > 0 else 50.0), i, j))
        cand.sort(key=lambda t: t[0])

        used_l, used_r, out = set(), set(), []
        for _, i, j in cand:
            if i in used_l or j in used_r or len(out) >= self.max_pairs:
                continue
            used_l.add(i)
            used_r.add(j)
            bl = [dets[i][0], dets[i][1], dets[i][2], dets[i][3]]
            br = [dets[j][0] - TILE_W, dets[j][1], dets[j][2] - TILE_W, dets[j][3]]
            r = self.solve_pair(L, R, bl, br)
            if r is None:
                continue
            # 运动判定：与本轨迹上一帧框位置的位移（换算成"每帧像素"）
            moving = True
            for t in self.tracks.values():
                if t.cls == int(dets[i][5]) and self.frame_i - t.last <= 3:
                    cx0, cy0 = self._center(t.box)
                    cx1, cy1 = self._center(bl)
                    if np.hypot(cx1 - cx0, cy1 - cy0) <= self.move_px:
                        moving = False
                        break
            key, t = self._track(int(dets[i][5]), bl, r["Z"], moving)
            if t.zs:
                r["Z_smooth"] = float(np.median(t.zs))
                span = max(t.zs) - min(t.zs)
            else:
                r["Z_smooth"], span = r["Z"], 0.0
            r["Z"] = r["Z_smooth"]
            r["moving"] = t.moving
            r["spread"] = span
            r["track"] = key
            if t.moving:
                self.n_moving += 1
            r["pair"] = j
            out.append((i, r))
        return out

    # ---------------- 画面标注 ----------------
    @staticmethod
    def annotate(vis, dets, pairs, good=(0, 255, 0), moving_col=(0, 200, 255),
                 bad=(0, 0, 255)):
        """运动中的目标用橙色、可疑用红色、静止可信用绿色"""
        for idx, r in pairs:
            d = dets[idx]
            z = r["Z"]
            txt = "%.2fm" % (z / 1000.0) if z >= 1000 else "%dmm" % z
            if r.get("suspect"):
                txt += "?"
                col = bad
            elif r.get("moving"):
                txt += "~"
                col = moving_col
            else:
                col = good
            x, y = int(d[0]), int(d[3]) + 16
            if y > vis.shape[0] - 4:
                y = int(d[1]) - 6
            x, y = max(2, x), max(14, y)
            cv2.putText(vis, txt, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3,
                        cv2.LINE_AA)
            cv2.putText(vis, txt, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 1,
                        cv2.LINE_AA)
        return vis

    def stat_line(self):
        return ("测距: 成功%d 剔除%d(错匹配%d) 运动%d"
                % (self.n_solved, self.n_rejected, self.n_badmatch, self.n_moving))


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--calib", required=True)
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    rg = StereoRanger(a.calib)
    print("标定载入成功")
    print("  f = %.2f px    B = %.2f mm" % (rg.f, rg.B))
    print("  重投影误差 %.3f px   矫正|dy|最大 %.2f px" % (rg.rms, rg.dy_cal))
    print("\n过滤参数: 匹配度>=%.2f  |dy|<=%.1f  双向往返<=%.1fpx  Z∈[%.0f,%.0f]mm"
          % (rg.min_score, rg.max_dy, rg.rt_tol, rg.z_min, rg.z_max))
    print("\n理论精度（模板匹配 ~0.2px）:")
    for Z in (500, 1000, 2000, 3000):
        print("  Z=%4dmm  视差 %5.2f px   0.2px -> ±%4.0f mm (%.1f%%)"
              % (Z, rg.f * rg.B / Z, Z * Z / (rg.f * rg.B) * 0.2,
                 100.0 * Z / (rg.f * rg.B) * 0.2))


if __name__ == "__main__":
    main()
