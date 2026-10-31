#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pcie_post.py —— 快速前处理 / 后处理（比原版快 10~20 倍）
=========================================================

原版（yolov8_deploy/postprocess.py）在 RK3568 上实测：
    预处理 58.9 ms   后处理 48.4 ms
本模块：
    预处理 ~3-5 ms   后处理 ~3-5 ms

为什么快这么多？
【预处理】原版路线：
    uint8 图 ─cv2─► uint8 ─/255─► float32(1,3,640,640) ─clip─► ─*255─► ─+0.5─► ─astype─► uint8
    （在 120 万个 float 上来回折腾 5 遍 → 59ms）
    本模块路线：
    uint8 图 ─cv2 resize/letterbox/cvtColor─► uint8(1,640,640,3)   ← 一步到位（~4ms）
    两者给 RKNN 的输入完全相同（已实拍验证）

【后处理】原版：对全部 80x80+40x40+20x20=8400 个候选都算 DFL（每个 64 通道 softmax）→ 48ms
    本模块：先用 score_sum 粗筛（1 个数/候选）→ 通常只剩几十个 → 只对这几十个算 DFL ✓

接口与 postprocess.py 兼容：preprocess_fast / decode_fast / nms_fast / to_original / draw
"""
import numpy as np

# OpenCV 默认可能只用单线程；RK3568 有 4 核，开多线程能明显加快 resize/颜色转换
try:
    import os as _os
    import cv2 as _cv2
    _cv2.setNumThreads(int(_os.environ.get("CV_THREADS", "4")))
except Exception:
    pass


# ------------------------------------------------------------------ 预处理
def preprocess_fast(bgr, size=640, pad_value=114):
    """BGR(uint8) -> (uint8 NHWC RGB, transform)

    几何变换与 yolov8_deploy/postprocess.py 的 preprocess() 完全一致
    （同样的 ratio / left / top 取整方式），保证坐标映射一致。
    """
    import cv2
    h, w = bgr.shape[:2]
    ratio = min(size / h, size / w)
    resized_w, resized_h = round(w * ratio), round(h * ratio)
    half_w, half_h = (size - resized_w) / 2, (size - resized_h) / 2
    left, top = round(half_w - 0.1), round(half_h - 0.1)
    right, bottom = round(half_w + 0.1), round(half_h + 0.1)
    resized = cv2.resize(bgr, (resized_w, resized_h), interpolation=cv2.INTER_LINEAR)
    padded = cv2.copyMakeBorder(resized, top, bottom, left, right,
                                cv2.BORDER_CONSTANT, value=(pad_value,) * 3)
    rgb = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB)
    return np.ascontiguousarray(rgb[None]), {
        "ratio": ratio, "left": left, "top": top, "shape": [h, w]}


# ------------------------------------------------------------------ 后处理
def _nms_np(dets, iou_threshold=0.45, max_det=300):
    """dets: (N,6) = x1,y1,x2,y2,score,cls  —— 纯 numpy，候选很少时很快"""
    if len(dets) == 0:
        return dets
    out = []
    for c in np.unique(dets[:, 5]):
        d = dets[dets[:, 5] == c]
        order = np.argsort(-d[:, 4])
        d = d[order]
        while len(d):
            best = d[0]
            out.append(best)
            if len(d) == 1:
                break
            rest = d[1:]
            x1 = np.maximum(best[0], rest[:, 0])
            y1 = np.maximum(best[1], rest[:, 1])
            x2 = np.minimum(best[2], rest[:, 2])
            y2 = np.minimum(best[3], rest[:, 3])
            inter = np.maximum(x2 - x1, 0) * np.maximum(y2 - y1, 0)
            a1 = max(best[2] - best[0], 0) * max(best[3] - best[1], 0)
            a2 = np.maximum(rest[:, 2] - rest[:, 0], 0) * np.maximum(rest[:, 3] - rest[:, 1], 0)
            iou = inter / np.maximum(a1 + a2 - inter, 1e-9)
            d = rest[iou <= iou_threshold]
    if not out:
        return np.zeros((0, 6), dtype=np.float32)
    res = np.array(out, dtype=np.float32)
    res = res[np.argsort(-res[:, 4])][:max_det]
    return res


def decode_fast(outputs, size=640, nc=10, reg_max=16, confidence=0.25, iou=0.45):
    """9 输出 DFL 解码（先粗筛再算 DFL）

    outputs 顺序（每个 stride 3 个）：
        [0] regression logits (1, 4*reg_max, gs, gs)
        [1] sigmoid class scores (1, nc, gs, gs)
        [2] clipped class-score sum (1, 1, gs, gs)
    """
    cand = []
    for branch, stride in enumerate((8, 16, 32)):
        logits, classes, score_sum = outputs[branch * 3:branch * 3 + 3]
        gs = size // stride
        cls = classes.reshape(nc, gs, gs)
        best = cls.max(axis=0)                       # (gs,gs) 每格最高类别分
        ss = score_sum.reshape(gs, gs)               # 每格分数和（≥ max，可安全粗筛）
        mask = (ss >= confidence) & (best >= confidence)
        if not mask.any():
            continue
        idx = np.flatnonzero(mask.ravel())
        cid = cls.reshape(nc, -1).argmax(axis=0)[idx]
        sc = best.ravel()[idx]

        # 只对候选算 DFL（候选通常只有几十个）
        lg = logits.reshape(4, reg_max, gs, gs)[:, :, mask]        # (4,reg_max,N)
        lg = lg - lg.max(axis=1, keepdims=True)
        np.exp(lg, out=lg)
        lg /= lg.sum(axis=1, keepdims=True)
        dist = (lg * np.arange(reg_max, dtype=np.float32)[None, :, None]).sum(axis=1)  # (4,N)

        ys, xs = np.divmod(idx, gs)
        cx = xs + 0.5
        cy = ys + 0.5
        cand.append(np.stack([
            (cx - dist[0]) * stride, (cy - dist[1]) * stride,
            (cx + dist[2]) * stride, (cy + dist[3]) * stride,
            sc, cid], axis=1))
    if not cand:
        return np.zeros((0, 6), dtype=np.float32)
    return _nms_np(np.concatenate(cand, axis=0).astype(np.float32), iou)


def guess_size_from_model(model_path, default=640):
    """从模型文件名猜输入尺寸，例如 ..._i8_416.rknn → 416

    这样换模型（416/640）时不用手动传 --size，避免"模型 416 但 letterbox 按 640 做"的坑。
    """
    import os
    import re
    common = (320, 352, 384, 416, 448, 480, 512, 544, 576, 608, 640)
    for n in re.findall(r"(\d{3})", os.path.basename(str(model_path))):
        v = int(n)
        if v in common:
            return v
    return default


def decode_std(outputs, size=640, nc=10, confidence=0.25, iou=0.45):
    """标准 Ultralytics YOLOv8 导出格式：单输出 (1, 4+nc, N)

    框是 xywh（letterbox 图像像素坐标），类别分已过 sigmoid。
    重新导出模型（尤其是换 416 输入尺寸）后通常是这种格式。
    """
    pred = np.asarray(outputs[0])
    while pred.ndim > 2:
        pred = pred[0]
    if pred.shape[0] < pred.shape[1]:
        pred = pred.T                       # (N, 4+nc)
    xywh = pred[:, :4]
    cls = pred[:, 4:]
    nc = cls.shape[1]

    # 【实测兼容】两种类别分支：
    #   ① 概率（已过 sigmoid，0~1）—— 旧 9 输出模型是这种
    #   ② logits（未过 sigmoid，可能 >1）—— 新导出的标准 ONNX 是这种
    #      注意：int8 量化会把"无目标"锚点的负 logits 压成 0（= 概率 0.5），
    #      所以不能用概率阈值 0.25 去过滤，必须按 logit > 0 过滤。
    vmax = float(cls.max()) if cls.size else 0.0
    if vmax > 1.0:                       # ---- logits 模式 ----
        thr = 0.0
        if 0.0 < confidence < 1.0:
            thr = float(np.log(confidence / (1.0 - confidence)))
        thr = max(thr, 0.0)              # 不低于 0，否则空锚点全部入选
        cid = cls.argmax(axis=1)
        raw = cls[np.arange(len(cid)), cid]
        keep = raw > thr
        sc = 1.0 / (1.0 + np.exp(-np.clip(raw, -50.0, 50.0)))   # 显示用概率
    else:                                # ---- 概率模式 ----
        cid = cls.argmax(axis=1)
        sc = cls[np.arange(len(cid)), cid]
        keep = sc >= confidence

    if not keep.any():
        return np.zeros((0, 6), dtype=np.float32)
    xywh, sc, cid = xywh[keep], sc[keep], cid[keep]
    cx, cy, w, h = xywh[:, 0], xywh[:, 1], xywh[:, 2], xywh[:, 3]
    dets = np.stack([cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2, sc, cid],
                    axis=1).astype(np.float32)
    return _nms_np(dets, iou)


def decode_auto(outputs, size=640, nc=10, confidence=0.25, iou=0.45):
    """自动识别模型输出格式：
        9 个张量  → 旧 DFL 格式（decode_fast）
        其它      → 标准 Ultralytics 单输出（decode_std）
    """
    if len(outputs) >= 9:
        return decode_fast(outputs, size=size, nc=nc,
                           confidence=confidence, iou=iou)
    return decode_std(outputs, size=size, nc=nc,
                      confidence=confidence, iou=iou)


def to_original(detections, transform):
    """把 letterbox 坐标映射回原图（与原版一致）"""
    if len(detections) == 0:
        return detections
    r = detections.copy()
    r[:, [0, 2]] = (r[:, [0, 2]] - transform["left"]) / transform["ratio"]
    r[:, [1, 3]] = (r[:, [1, 3]] - transform["top"]) / transform["ratio"]
    h, w = transform["shape"]
    r[:, [0, 2]] = np.clip(r[:, [0, 2]], 0, w)
    r[:, [1, 3]] = np.clip(r[:, [1, 3]], 0, h)
    return r


def draw(image, detections, names, title):
    """与原版 draw() 相同风格"""
    import cv2
    canvas = image.copy()
    for x1, y1, x2, y2, confidence, class_id in detections:
        class_id = int(class_id)
        color = ((37 * class_id + 40) % 220, (71 * class_id + 180) % 220,
                 (111 * class_id + 60) % 220)
        p1 = (round(float(x1)), round(float(y1)))
        p2 = (round(float(x2)), round(float(y2)))
        cv2.rectangle(canvas, p1, p2, color, 2)
        cv2.putText(canvas, "%s %.2f" % (names[class_id], confidence),
                    (p1[0], max(18, p1[1] - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
    canvas = cv2.copyMakeBorder(canvas, 35, 0, 0, 0, cv2.BORDER_CONSTANT, value=(25, 25, 25))
    cv2.putText(canvas, title, (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 1)
    return canvas


# --------------------------------------------------- 对照测试（证明结果一致）
if __name__ == "__main__":
    import os
    import sys
    import time

    YOLO = os.environ.get("YOLO_DIR", "/home/linaro/yolov8_deploy")
    sys.path.insert(0, YOLO)
    import cv2
    from postprocess import preprocess as pre_old, decode as dec_old, nms as nms_old

    from rknnlite.api import RKNNLite

    model = os.path.join(YOLO, "models", "bdd100k_yolov8n_rk3568_i8.rknn")
    img_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(YOLO, "images", "test_image.jpg")
    bgr = cv2.imread(img_path)
    print("图片:", img_path, bgr.shape)

    rk = RKNNLite()
    rk.load_rknn(model)
    rk.init_runtime()

    # ---- 新路线 ----
    t0 = time.time()
    for _ in range(10):
        inp, tr = preprocess_fast(bgr, size=640)
    t_pre_new = (time.time() - t0) / 10
    outs = rk.inference(inputs=[inp], data_format="nhwc")
    t0 = time.time()
    for _ in range(10):
        d_new = to_original(decode_fast(outs, size=640, nc=10), tr)
    t_post_new = (time.time() - t0) / 10

    # ---- 原路线 ----
    t0 = time.time()
    for _ in range(10):
        t_old, tr_old = pre_old(bgr, size=640)
        u8 = np.ascontiguousarray(
            (np.clip(t_old, 0, 1)[0].transpose(1, 2, 0) * 255.0 + 0.5).astype(np.uint8)[None])
    t_pre_old = (time.time() - t0) / 10
    outs2 = rk.inference(inputs=[u8], data_format="nhwc")
    t0 = time.time()
    for _ in range(10):
        b, s = dec_old(outs2, size=640, nc=10)
        d_old = to_original(nms_old(b, s, confidence=0.25), tr_old)
    t_post_old = (time.time() - t0) / 10

    print("\n预处理:  原版 %6.1f ms  →  新版 %5.1f ms   (快 %.1f 倍)"
          % (t_pre_old * 1000, t_pre_new * 1000, t_pre_old / max(t_pre_new, 1e-9)))
    print("后处理:  原版 %6.1f ms  →  新版 %5.1f ms   (快 %.1f 倍)"
          % (t_post_old * 1000, t_post_new * 1000, t_post_old / max(t_post_new, 1e-9)))
    print("\n原版检测 %d 个, 新版检测 %d 个" % (len(d_old), len(d_new)))
    print("原版:", [(int(x[5]), round(float(x[4]), 3)) for x in d_old[:8]])
    print("新版:", [(int(x[5]), round(float(x[4]), 3)) for x in d_new[:8]])
