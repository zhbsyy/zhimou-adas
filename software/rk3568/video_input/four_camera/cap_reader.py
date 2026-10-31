#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cap_reader.py —— 持续抓帧并写到 stdout（RGB565 原始帧流）
=========================================================

【v3.1 修正】命令长度字段 10 位，值 = 长度-1 → **最大 1023 个 DWORD = 4092 字节**
    （v3 写成 1024 个 DWORD 会溢出成 0 → 引擎不传输 → 画面全黑）
    每帧 1,843,200 字节 ≈ 451 × 4092，比按行模式的 720 条命令少 37%
"""
import ctypes
import mmap
import os
import signal
import struct
import sys
import time

DEV_SYSFS = "/sys/bus/pci/devices/0002:21:00.0"
WIDTH, HEIGHT = 1280, 720
FRAME_BYTES = WIDTH * HEIGHT * 2        # 1,843,200
ROW_BYTES = WIDTH * 2                   # 2560
PAGE = 4096
MAX_DWORDS = 1023                       # 10 位字段能表示的最大 DWORD 数

DELAY = float(os.environ.get("CAP_DELAY", "0.00001"))
# 【实测结论】分块模式(4092字节×451)失败——FPGA 数据源每次命令只能搬"一行 2560 字节"，
# 请求更长长度时引擎拿不到数据、地址被覆盖 → 画面全黑。所以默认用按行模式。
# 想复现实验可设 CAP_CHUNK=4092。
USE_CHUNK = os.environ.get("CAP_CHUNK", "0") != "0"

# 帧对齐（纠正垂直滚动）：**默认关闭**
#   实测：滚动量每帧都不同（FPGA 自由运行 + 主机异步读取），自动检测不稳定
#   → 每帧纠正会让画面乱跳，比不纠正更糟。等 RTL 侧做出"帧起始同步"再启用。
#   想实验可设 CAP_ROLL_FIX=1
ROLL_FIX = os.environ.get("CAP_ROLL_FIX", "0") == "1"
ROLL = 0

try:
    from frame_align import detect_roll, apply_roll, check_alignment
except Exception:                       # 没有 numpy 时退化为"不对齐"
    detect_roll = apply_roll = check_alignment = None
    ROLL_FIX = False

if USE_CHUNK:
    TRANS_BYTES = MAX_DWORDS * 4                        # 4092
    NTRANS = (FRAME_BYTES + TRANS_BYTES - 1) // TRANS_BYTES   # 451
    CMD = (MAX_DWORDS - 1) | (1 << 16) | (1 << 24)      # 长度字段=长度-1
else:
    TRANS_BYTES = ROW_BYTES                             # 每次搬一行 2560 字节
    NTRANS = HEIGHT                                     # 720
    CMD = (ROW_BYTES // 4 - 1) | (1 << 16) | (1 << 24)

_limit = int(os.environ.get("CAP_TRANS", "0"))
if _limit > 0:
    NTRANS = min(NTRANS, _limit)

STOP = False


def precise_sleep(sec):
    """精确忙等（time.sleep 在 Linux 上最小实际约 50µs，太粗糙）

    为什么要等：写一行命令后 FPGA 才开始搬数据；
    太快写下一段的地址寄存器会覆盖掉，导致数据落到错误缓冲区（画面撕裂）。
    """
    if sec <= 0:
        return
    end = time.perf_counter() + sec
    while time.perf_counter() < end:
        pass


def _sig(_s, _f):
    global STOP
    STOP = True


def log(msg):
    sys.stderr.write("[cap_reader] %s\n" % msg)
    sys.stderr.flush()


def main():
    global ROLL
    signal.signal(signal.SIGTERM, _sig)
    signal.signal(signal.SIGINT, _sig)

    try:
        line = open(os.path.join(DEV_SYSFS, "resource")).read().splitlines()[1].split()
        bar1 = int(line[0], 16)
    except Exception as e:
        log("读 BAR1 地址失败: %s" % e)
        return 2

    total = PAGE * NTRANS
    buf = mmap.mmap(-1, total, prot=mmap.PROT_READ | mmap.PROT_WRITE)
    buf.write(b"\x00" * total)
    base = ctypes.addressof(ctypes.c_char.from_buffer(buf))
    ctypes.CDLL("libc.so.6", use_errno=True).mlock(
        ctypes.c_void_p(base), ctypes.c_size_t(total))

    pm = open("/proc/self/pagemap", "rb")

    def phys_of(va):
        pm.seek((va // PAGE) * 8)
        e = struct.unpack("<Q", pm.read(8))[0]
        if not (e >> 63):
            raise RuntimeError("页未驻留")
        return (e & ((1 << 55) - 1)) * PAGE

    addrs = [phys_of(base + i * PAGE) for i in range(NTRANS)]

    mem_fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
    bar = mmap.mmap(mem_fd, PAGE, mmap.MAP_SHARED,
                    mmap.PROT_READ | mmap.PROT_WRITE, offset=bar1)

    def wr(off, val):
        struct.pack_into("<I", bar, off, val)

    log("BAR1=0x%x  模式=%s  搬运%d次x%d字节  delay=%.0fus(精确忙等)  ~%.1f fps 上限"
        % (bar1, ("分块%d" % TRANS_BYTES) if USE_CHUNK else ("按行%d" % TRANS_BYTES),
           NTRANS, TRANS_BYTES, DELAY * 1e6, 1.0 / max(NTRANS * DELAY, 1e-6)))

    out = sys.stdout.buffer
    n = 0
    t_last = time.time()
    try:
        while not STOP:
            for a in addrs:
                wr(0x100, CMD)
                wr(0x110, a & 0xFFFFFFFF)
                wr(0x120, a >> 32)
                if DELAY:
                    precise_sleep(DELAY)
            time.sleep(0.0005)
            frame = b"".join(bytes(buf[i * PAGE:i * PAGE + TRANS_BYTES])
                             for i in range(NTRANS))[:FRAME_BYTES]

            # ---- 纠正"垂直滚动"：每帧都检测（滚动量每帧不同）----
            if ROLL_FIX:
                try:
                    r = detect_roll(frame, WIDTH, HEIGHT)
                    frame = apply_roll(frame, r, WIDTH, HEIGHT)
                    if n % 50 == 0:
                        log("   第%d帧: 滚动 %d 行" % (n, r))
                except Exception as e:
                    log("滚动检测失败(%s)，本帧不纠正" % e)

            out.write(frame)
            out.flush()
            n += 1
            if n % 20 == 0:
                now = time.time()
                log("%d 帧, 平均 %.1f fps" % (n, 20.0 / max(now - t_last, 1e-6)))
                t_last = now
    except BrokenPipeError:
        pass
    finally:
        bar.close()
        os.close(mem_fd)
    log("结束，共 %d 帧" % n)
    return 0


if __name__ == "__main__":
    sys.exit(main())
