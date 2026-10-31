"""
OIB 智能车载控制中枢与全域硬件在环 (HIL) 仿真综合监控台 (PyQt5)
================================================================
双栏工控工作站架构：
1. 【左栏大屏视频】：左侧最大化展示 1280x720 智驾感知/CARLA 实时视频，内嵌 ADAS 决策状态；
2. 【右栏紧凑控制】：右侧集中收纳车辆动力学仪表、电子挡位、动力学接管开关、路线选择与诊断终端；
3. 【浅色醒目分割】：面板边框、模块卡片与中缝分割线采用高对比浅色设计，边界清晰利落；
4. 【纯净专业符号】：全面剔除 Emoji 符号，采用现代工业级文本与状态药丸指示；
5. 【防卡死抗崩溃】：样式更新采用差分缓存机制，彻底根除高频刷新导致的 C++ 段错误闪退。
"""

from __future__ import annotations

import math
import os
import sys
import threading
import time
import urllib.request
from typing import Optional

from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QImage, QPainter, QPixmap
from PyQt5.QtWidgets import (
    QApplication, QComboBox, QDialog, QFrame, QGridLayout, QGroupBox,
    QHBoxLayout, QLabel, QLineEdit, QMainWindow, QPlainTextEdit,
    QProgressBar, QPushButton, QScrollArea, QSizePolicy, QSlider,
    QSplitter, QVBoxLayout, QWidget,
)

from bridge.arbiter import SimulationArbiter
from bridge.kinematic_backend import VehicleState
from bridge.serial_client import SerialClient

BOARD_STREAM_URL = os.environ.get("ADAS_BOARD_STREAM_URL", "http://192.168.10.2:8080/stream")

# -----------------------------------------------------------------------------
# 浅色分割线 + 深色座舱 HUD 样式表 (零 Emoji)
# -----------------------------------------------------------------------------
DUAL_PANE_HUD_STYLE = """
QMainWindow, QDialog {
    background-color: #0b0f19;
}
QWidget {
    color: #e2e8f0;
    font-family: -apple-system, "Segoe UI", "PingFang SC", "Microsoft YaHei", sans-serif;
    font-size: 12px;
}

/* 顶部系统标题与状态栏 (浅色底部强分割线) */
QFrame#topNavFrame {
    background-color: #121927;
    border-bottom: 2px solid #cbd5e1;
    padding: 6px 12px;
}
QLabel#sysMainTitle {
    font-size: 15px;
    font-weight: 800;
    color: #f8fafc;
    letter-spacing: 0.5px;
}
QLabel#sysSubTitle {
    font-size: 11px;
    color: #94a3b8;
    font-weight: 500;
}

/* 浅色分割中缝 (QSplitter handle) */
QSplitter::handle:horizontal {
    background-color: #cbd5e1;
    width: 3px;
    margin: 4px 2px;
    border-radius: 1px;
}
QSplitter::handle:horizontal:hover {
    background-color: #38bdf8;
}

/* 浅色分割横线 */
QFrame.light-separator {
    background-color: #94a3b8;
    min-height: 1px;
    max-height: 1px;
    border: none;
}

/* 主面板容器 (浅色外边框) */
QFrame.hud-panel-left {
    background-color: #101623;
    border: 1.5px solid #cbd5e1;
    border-radius: 8px;
}
QFrame.hud-panel-right {
    background-color: #101623;
    border: 1.5px solid #cbd5e1;
    border-radius: 8px;
}

/* 内部功能卡片 (浅灰蓝清晰边界) */
QFrame.hud-card {
    background-color: #162032;
    border: 1px solid #64748b;
    border-radius: 6px;
}

QLabel.card-title {
    font-size: 11px;
    font-weight: 700;
    color: #38bdf8;
    text-transform: uppercase;
    letter-spacing: 0.5px;
}

/* 状态药丸徽章 (Pill Badges) */
QLabel.status-pill {
    padding: 2px 7px;
    border-radius: 4px;
    font-size: 11px;
    font-weight: 700;
    background-color: #1e293b;
    color: #cbd5e1;
    border: 1px solid #64748b;
}
QLabel.status-pill.online {
    background-color: #064e3b;
    color: #34d399;
    border: 1px solid #10b981;
}
QLabel.status-pill.active {
    background-color: #1e3a8a;
    color: #60a5fa;
    border: 1px solid #3b82f6;
}
QLabel.status-pill.warning {
    background-color: #78350f;
    color: #fbbf24;
    border: 1px solid #f59e0b;
}
QLabel.status-pill.alert {
    background-color: #7f1d1d;
    color: #f87171;
    border: 1px solid #ef4444;
}

/* 仪表读数大标签 */
QLabel.gauge-num {
    font-family: "Cascadia Code", "Consolas", monospace;
    font-size: 34px;
    font-weight: 900;
    color: #f8fafc;
}
QLabel.gauge-unit {
    font-size: 12px;
    color: #94a3b8;
    font-weight: 600;
}

/* 通用工控按钮 (浅色描边) */
QPushButton.hud-btn {
    background-color: #1e293b;
    border: 1px solid #94a3b8;
    border-radius: 4px;
    padding: 4px 10px;
    color: #f1f5f9;
    font-weight: 600;
    font-size: 11px;
    min-height: 24px;
}
QPushButton.hud-btn:hover {
    background-color: #334155;
    border-color: #cbd5e1;
    color: #ffffff;
}
QPushButton.hud-btn:pressed {
    background-color: #0f172a;
}
QPushButton.hud-btn.primary {
    background-color: #2563eb;
    border-color: #93c5fd;
    color: #ffffff;
}
QPushButton.hud-btn.primary:hover {
    background-color: #1d4ed8;
}
QPushButton.hud-btn.success {
    background-color: #059669;
    border-color: #6ee7b7;
    color: #ffffff;
}
QPushButton.hud-btn.success:hover {
    background-color: #047857;
}
QPushButton.hud-btn.danger {
    background-color: #dc2626;
    border-color: #fca5a5;
    color: #ffffff;
}
QPushButton.hud-btn.danger:hover {
    background-color: #b91c1c;
}

/* 电子挡位按钮 */
QPushButton.gear-btn {
    background-color: #1e293b;
    border: 1px solid #64748b;
    border-radius: 4px;
    font-size: 13px;
    font-weight: 800;
    color: #94a3b8;
    min-width: 38px;
    min-height: 30px;
}
QPushButton.gear-btn:hover {
    border-color: #cbd5e1;
    color: #ffffff;
}
QPushButton.gear-btn.active-p {
    background-color: #b45309;
    border-color: #fcd34d;
    color: #ffffff;
}
QPushButton.gear-btn.active-r {
    background-color: #b91c1c;
    border-color: #fca5a5;
    color: #ffffff;
}
QPushButton.gear-btn.active-n {
    background-color: #475569;
    border-color: #cbd5e1;
    color: #ffffff;
}
QPushButton.gear-btn.active-d {
    background-color: #047857;
    border-color: #6ee7b7;
    color: #ffffff;
}

/* 路线切换按钮 */
QPushButton.route-btn {
    background-color: #1e293b;
    border: 1px solid #64748b;
    border-radius: 4px;
    padding: 3px 6px;
    color: #cbd5e1;
    font-size: 11px;
    font-weight: 600;
    min-height: 22px;
}
QPushButton.route-btn:hover {
    border-color: #cbd5e1;
    color: #ffffff;
}
QPushButton.route-btn.active {
    background-color: #1e3a8a;
    border-color: #93c5fd;
    color: #ffffff;
    font-weight: 700;
}

/* 智驾运行模式分段按钮 */
QPushButton.mode-btn {
    background-color: #1e293b;
    border: 1px solid #475569;
    border-radius: 5px;
    padding: 5px 6px;
    color: #94a3b8;
    font-size: 11px;
    font-weight: 600;
    min-height: 26px;
}
QPushButton.mode-btn:hover {
    background-color: #334155;
    color: #ffffff;
}
QPushButton.mode-btn.active-cockpit {
    background-color: #0369a1;
    border-color: #38bdf8;
    color: #ffffff;
    font-weight: 700;
}
QPushButton.mode-btn.active-perception {
    background-color: #6b21a8;
    border-color: #c084fc;
    color: #ffffff;
    font-weight: 700;
}
QPushButton.mode-btn.active-ad {
    background-color: #047857;
    border-color: #34d399;
    color: #ffffff;
    font-weight: 700;
}

/* 车灯指示器芯片 */
QLabel.indicator-chip {
    padding: 2px 4px;
    border-radius: 3px;
    font-size: 10px;
    font-weight: 700;
    background-color: #1e293b;
    color: #64748b;
    border: 1px solid #475569;
    min-width: 44px;
}
QLabel.indicator-chip.amber-on {
    background-color: #78350f;
    color: #fbbf24;
    border-color: #f59e0b;
}
QLabel.indicator-chip.red-on {
    background-color: #7f1d1d;
    color: #f87171;
    border-color: #ef4444;
}
QLabel.indicator-chip.blue-on {
    background-color: #1e3a8a;
    color: #60a5fa;
    border-color: #3b82f6;
}

/* 进度条 */
QProgressBar {
    background-color: #1e293b;
    border: 1px solid #64748b;
    border-radius: 3px;
    text-align: center;
    color: #cbd5e1;
    font-size: 10px;
    font-weight: 600;
    min-height: 8px;
    max-height: 8px;
}
QProgressBar::chunk {
    background-color: #0284c7;
    border-radius: 2px;
}

/* 诊断终端 */
QPlainTextEdit {
    background-color: #0a0e17;
    border: 1px solid #64748b;
    border-radius: 5px;
    color: #cbd5e1;
    font-family: "Cascadia Code", "Consolas", monospace;
    font-size: 11px;
    padding: 5px;
}

/* 下拉框与输入框 */
QComboBox, QLineEdit {
    background-color: #1e293b;
    border: 1px solid #64748b;
    border-radius: 4px;
    padding: 3px 6px;
    color: #f8fafc;
    font-size: 11px;
    min-height: 22px;
}
QComboBox:focus, QLineEdit:focus {
    border-color: #cbd5e1;
}
QComboBox QAbstractItemView {
    background-color: #1e293b;
    border: 1px solid #94a3b8;
    selection-background-color: #2563eb;
    color: #f8fafc;
}

/* 滚动条 */
QScrollBar:vertical {
    background: #0f172a;
    width: 6px;
    margin: 0px;
}
QScrollBar::handle:vertical {
    background: #475569;
    min-height: 20px;
    border-radius: 3px;
}
QScrollBar::handle:vertical:hover {
    background: #94a3b8;
}
"""


def update_widget_class(widget: QWidget, new_class: str) -> None:
    """仅在样式类属性发生跳变时才触发 QStyle polish/unpolish，彻底杜绝 C++ 段错误与卡死闪退。"""
    if widget.property("class") != new_class:
        widget.setProperty("class", new_class)
        style = widget.style()
        if style:
            style.unpolish(widget)
            style.polish(widget)


class MjpegViewerWindow(QDialog):
    """独立弹出的高分辨率视频监视器窗口。"""

    sig_frame = pyqtSignal(QPixmap)
    sig_status = pyqtSignal(str)

    def __init__(self, parent=None, default_url: str = BOARD_STREAM_URL) -> None:
        super().__init__(parent)
        self.setWindowTitle("独立感知视频监视器 (RK3568 / CARLA)")
        self.resize(760, 500)
        self.setStyleSheet(DUAL_PANE_HUD_STYLE)

        self._stream_url = default_url
        self._running = False
        self._thread: Optional[threading.Thread] = None

        self._init_ui()
        self.sig_frame.connect(self._update_frame)
        self.sig_status.connect(self.lbl_status.setText)

    def _init_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(8)

        top = QHBoxLayout()
        top.setSpacing(6)

        lbl = QLabel("视频流地址:")
        lbl.setStyleSheet("font-weight: 700; color: #cbd5e1;")
        top.addWidget(lbl)

        self.edit_url = QLineEdit(self._stream_url)
        top.addWidget(self.edit_url, 1)

        self.btn_toggle = QPushButton("连接")
        self.btn_toggle.setProperty("class", "hud-btn primary")
        self.btn_toggle.clicked.connect(self._toggle_stream)
        top.addWidget(self.btn_toggle)

        layout.addLayout(top)

        self.lbl_video = QLabel("未连接视频流\n(请输入流地址并点击【连接】)")
        self.lbl_video.setAlignment(Qt.AlignCenter)
        self.lbl_video.setStyleSheet(
            "background-color: #070a10; color: #475569; font-size: 13px; "
            "border: 1.5px solid #cbd5e1; border-radius: 6px;"
        )
        self.lbl_video.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        layout.addWidget(self.lbl_video, 1)

        self.lbl_status = QLabel("状态: 空闲待命")
        self.lbl_status.setStyleSheet("color: #94a3b8; font-size: 11px;")
        layout.addWidget(self.lbl_status)

    def _toggle_stream(self) -> None:
        if self._running:
            self._stop_stream()
            self.btn_toggle.setText("连接")
            update_widget_class(self.btn_toggle, "hud-btn primary")
            self.sig_status.emit("状态: 视频流已断开")
        else:
            self._stream_url = self.edit_url.text().strip()
            self._start_stream()
            self.btn_toggle.setText("断开")
            update_widget_class(self.btn_toggle, "hud-btn success")

    def _start_stream(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def _stop_stream(self) -> None:
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=0.5)
        self._thread = None

    def _worker(self) -> None:
        url = self._stream_url
        self.sig_status.emit(f"状态: 正在连接 {url} ...")
        while self._running:
            resp = None
            try:
                req = urllib.request.Request(url, headers={"User-Agent": "HIL_Monitor/2.0"})
                resp = urllib.request.urlopen(req, timeout=3.0)
                self.sig_status.emit("状态: 正常拉流中 (MJPEG 25~30 FPS)")
                buf = b""
                while self._running:
                    chunk = resp.read(4096)
                    if not chunk:
                        break
                    buf += chunk
                    start = buf.find(b"\xff\xd8")
                    end = buf.find(b"\xff\xd9", start + 2) if start != -1 else -1
                    if start != -1 and end != -1:
                        jpg = buf[start:end + 2]
                        buf = buf[end + 2:]
                        pix = QPixmap()
                        if pix.loadFromData(jpg):
                            self.sig_frame.emit(pix)
            except Exception as e:
                self.sig_status.emit(f"状态: 连接重试中 ({str(e)[:40]})")
                time.sleep(1.0)
            finally:
                if resp is not None:
                    try:
                        resp.close()
                    except Exception:
                        pass

    def _update_frame(self, pix: QPixmap) -> None:
        scaled = pix.scaled(self.lbl_video.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.lbl_video.setPixmap(scaled)

    def closeEvent(self, event) -> None:
        self._stop_stream()
        event.accept()


class DashboardWindow(QMainWindow):
    """
    OIB 智能车载控制中枢与全域 HIL 仿真监控台 (全新双栏设计):
    - 左栏：大屏实时视频感知视口 + ADAS 安全决策矩阵 (占据主视区)
    - 右栏：精简工控区 (车辆动力学、挡位控制、动力学接管、总线诊断终端)
    - 浅色边框与浅色中缝强分割，无 Emoji 纯净工业风格。
    """

    sig_state_update = pyqtSignal(object)
    sig_log = pyqtSignal(str)
    sig_video_frame = pyqtSignal(QPixmap)
    sig_video_status = pyqtSignal(str)

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("OIB 智能车载控制中枢与全域 HIL 监控工作站")
        self.setStyleSheet(DUAL_PANE_HUD_STYLE)

        # 宽屏双栏舒适比例 (1320 x 820)
        self.resize(1320, 820)
        self.setMinimumSize(1120, 680)

        self._is_always_on_top = False
        self._decision_active = False
        self.decision_panel = None

        # 仲裁中枢核心
        self.arbiter = SimulationArbiter(
            on_state_change=self._safe_emit_state,
            on_log=self._safe_emit_log,
        )
        self.sig_state_update.connect(self._render_state)
        self.sig_log.connect(self._append_log)

        # 视频拉流引擎
        self.popup_window = MjpegViewerWindow(self)
        self._video_running = False
        self._video_thread: Optional[threading.Thread] = None
        self.sig_video_frame.connect(self._on_video_frame)
        self.sig_video_status.connect(self._on_video_status)

        self._init_ui()
        self._auto_connect_serial()
        self._start_video_stream()

    def _safe_emit_state(self, state: object) -> None:
        try:
            self.sig_state_update.emit(state)
        except Exception:
            pass

    def _safe_emit_log(self, text: str) -> None:
        try:
            self.sig_log.emit(text)
        except Exception:
            pass

    # =========================================================================
    # UI 布局初始化
    # =========================================================================
    def _init_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        root_layout = QVBoxLayout(central)
        root_layout.setContentsMargins(10, 8, 10, 10)
        root_layout.setSpacing(8)

        # ---------------- 1. 顶栏：系统标题与四域状态指示 (浅色底部分割线) ----------------
        top_frame = QFrame()
        top_frame.setObjectName("topNavFrame")
        top_layout = QHBoxLayout(top_frame)
        top_layout.setContentsMargins(8, 4, 8, 4)
        top_layout.setSpacing(12)

        title_box = QVBoxLayout()
        title_box.setSpacing(1)
        self.lbl_main_title = QLabel("OIB 智能车载中枢控制系统")
        self.lbl_main_title.setObjectName("sysMainTitle")
        self.lbl_sub_title = QLabel("全域硬件在环 (HIL) 综合监控台 · 瑞芯派座舱 / PC 控制中枢 / RK3568 智驾")
        self.lbl_sub_title.setObjectName("sysSubTitle")
        title_box.addWidget(self.lbl_main_title)
        title_box.addWidget(self.lbl_sub_title)
        top_layout.addLayout(title_box)

        top_layout.addStretch(1)

        # 四域连接指示药丸
        self.pill_cockpit = QLabel("座舱域: 未连")
        self.pill_cockpit.setProperty("class", "status-pill")
        self.pill_amap = QLabel("高德协议: 未接收")
        self.pill_amap.setProperty("class", "status-pill")
        self.pill_adas = QLabel("智驾域: 离线")
        self.pill_adas.setProperty("class", "status-pill")
        self.pill_sim = QLabel("底盘: 内置运动学")
        self.pill_sim.setProperty("class", "status-pill")

        for pill in (self.pill_cockpit, self.pill_amap, self.pill_adas, self.pill_sim):
            pill.setAlignment(Qt.AlignCenter)
            top_layout.addWidget(pill)

        top_layout.addSpacing(10)

        # 辅助功能按钮
        self.btn_rk_decision = QPushButton("单镜头决策演示")
        self.btn_rk_decision.setObjectName("btn_rk_decision")
        self.btn_rk_decision.setProperty("class", "hud-btn primary")
        self.btn_rk_decision.clicked.connect(self._toggle_rk_decision)
        top_layout.addWidget(self.btn_rk_decision)

        self.btn_pop_video = QPushButton("独立视频")
        self.btn_pop_video.setProperty("class", "hud-btn")
        self.btn_pop_video.setToolTip("弹出独立视频监视窗口")
        self.btn_pop_video.clicked.connect(self._toggle_popup_video)
        top_layout.addWidget(self.btn_pop_video)

        self.btn_topmost = QPushButton("置顶: 关")
        self.btn_topmost.setProperty("class", "hud-btn")
        self.btn_topmost.clicked.connect(self._toggle_always_on_top)
        top_layout.addWidget(self.btn_topmost)

        root_layout.addWidget(top_frame)

        # ---------------- 2. 主体区：双栏分割布局 (左栏大屏视频 / 右栏紧凑控制) ----------------
        splitter = QSplitter(Qt.Horizontal)
        splitter.setHandleWidth(4)

        # 左栏：大屏视频视口与 ADAS 决策
        left_widget = self._build_left_video_pane()
        splitter.addWidget(left_widget)

        # 右栏：紧凑工控台与总线遥测
        right_widget = self._build_right_control_pane()
        splitter.addWidget(right_widget)

        # 初始分配比例：左边 63% (视频最大化)，右边 37% (控制区)
        splitter.setStretchFactor(0, 63)
        splitter.setStretchFactor(1, 37)
        splitter.setSizes([830, 470])

        root_layout.addWidget(splitter, 1)
        self.normal_splitter = splitter
        self.mode_root_layout = root_layout

    # -------------------------------------------------------------------------
    # 左栏：大屏实时视频监视器与 ADAS 安全决策矩阵
    # -------------------------------------------------------------------------
    def _build_left_video_pane(self) -> QFrame:
        pane = QFrame()
        pane.setProperty("class", "hud-panel-left")
        layout = QVBoxLayout(pane)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(8)

        # 顶部视频工具条
        v_header = QHBoxLayout()
        v_header.setSpacing(8)

        lbl_vt = QLabel("智驾感知实时视频视口")
        lbl_vt.setProperty("class", "card-title")
        v_header.addWidget(lbl_vt)

        self.lbl_video_source_badge = QLabel("源: PCIe 物理双目")
        self.lbl_video_source_badge.setProperty("class", "status-pill active")
        v_header.addWidget(self.lbl_video_source_badge)

        v_header.addStretch(1)

        self.btn_source_toggle = QPushButton("切换感知源: PCIe / CARLA")
        self.btn_source_toggle.setProperty("class", "hud-btn primary")
        self.btn_source_toggle.setToolTip("动态下发指令切换 RK3568 推理输入源")
        self.btn_source_toggle.clicked.connect(self._toggle_perception_source)
        v_header.addWidget(self.btn_source_toggle)

        self.btn_reconnect_video = QPushButton("重连视频")
        self.btn_reconnect_video.setProperty("class", "hud-btn")
        self.btn_reconnect_video.clicked.connect(self._restart_video_stream)
        v_header.addWidget(self.btn_reconnect_video)

        layout.addLayout(v_header)

        # 浅色分割线
        sep1 = QFrame()
        sep1.setProperty("class", "light-separator")
        layout.addWidget(sep1)

        # 视频渲染画面 (占据最大空间，浅色描边)
        self.lbl_video_screen = QLabel("正在连接视频监控流 (HTTP 8080 / 8081) ...")
        self.lbl_video_screen.setAlignment(Qt.AlignCenter)
        self.lbl_video_screen.setStyleSheet(
            "background-color: #070a10; color: #475569; font-size: 13px; font-weight: 600; "
            "border: 1.5px solid #cbd5e1; border-radius: 6px;"
        )
        self.lbl_video_screen.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.lbl_video_screen.setMinimumHeight(440)
        layout.addWidget(self.lbl_video_screen, 1)

        # 视频底部信息条
        v_info_bar = QHBoxLayout()
        v_info_bar.setSpacing(8)

        self.lbl_video_status = QLabel("视频流: 正在探测...")
        self.lbl_video_status.setStyleSheet("color: #94a3b8; font-size: 11px;")
        v_info_bar.addWidget(self.lbl_video_status, 1)

        lbl_spec = QLabel("规格: 1280x720 @ 25~30 FPS MJPEG")
        lbl_spec.setStyleSheet("color: #64748b; font-size: 11px;")
        v_info_bar.addWidget(lbl_spec)

        layout.addLayout(v_info_bar)

        # 浅色分割线
        sep2 = QFrame()
        sep2.setProperty("class", "light-separator")
        layout.addWidget(sep2)

        # ADAS 安全决策矩阵卡片 (直挂视频下方，即时感知联动)
        decision_card = QFrame()
        decision_card.setProperty("class", "hud-card")
        dc_layout = QVBoxLayout(decision_card)
        dc_layout.setContentsMargins(10, 8, 10, 8)
        dc_layout.setSpacing(4)

        dc_top = QHBoxLayout()
        lbl_dc_title = QLabel("ADAS 安全决策矩阵 (UDP 8888 上报)")
        lbl_dc_title.setProperty("class", "card-title")
        self.lbl_aeb_pill = QLabel("AEB 待命中")
        self.lbl_aeb_pill.setProperty("class", "status-pill online")
        dc_top.addWidget(lbl_dc_title)
        dc_top.addStretch(1)
        dc_top.addWidget(self.lbl_aeb_pill)
        dc_layout.addLayout(dc_top)

        self.lbl_decision_action = QLabel("规划指令: 正常自主巡航")
        self.lbl_decision_action.setStyleSheet("font-size: 14px; font-weight: 800; color: #34d399;")
        dc_layout.addWidget(self.lbl_decision_action)

        self.lbl_decision_veto = QLabel("制动阻停原因: 无 | 目标距离: -- m | 交通信号: --")
        self.lbl_decision_veto.setStyleSheet("color: #cbd5e1; font-size: 11px;")
        dc_layout.addWidget(self.lbl_decision_veto)

        layout.addWidget(decision_card)
        return pane

    # -------------------------------------------------------------------------
    # 右栏：紧凑工控台与总线遥测区
    # -------------------------------------------------------------------------
    def _build_right_control_pane(self) -> QWidget:
        pane = QFrame()
        pane.setProperty("class", "hud-panel-right")
        main_vbox = QVBoxLayout(pane)
        main_vbox.setContentsMargins(10, 10, 10, 10)
        main_vbox.setSpacing(8)

        # 滚动区域 (保证在任何小屏下内容均不折叠被吃掉)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setStyleSheet("background: transparent;")

        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(8)

        # 1. 车辆动力学仪表卡片
        dyn_card = QFrame()
        dyn_card.setProperty("class", "hud-card")
        d_layout = QVBoxLayout(dyn_card)
        d_layout.setContentsMargins(10, 8, 10, 8)
        d_layout.setSpacing(5)

        # 车速行
        sp_top = QHBoxLayout()
        lbl_sp_t = QLabel("瞬时车速")
        lbl_sp_t.setProperty("class", "card-title")
        self.lbl_speed_limit_badge = QLabel("限速 40 km/h")
        self.lbl_speed_limit_badge.setProperty("class", "status-pill warning")
        sp_top.addWidget(lbl_sp_t)
        sp_top.addStretch(1)
        sp_top.addWidget(self.lbl_speed_limit_badge)
        d_layout.addLayout(sp_top)

        sp_num_box = QHBoxLayout()
        sp_num_box.setSpacing(6)
        self.lbl_speed_val = QLabel("0.0")
        self.lbl_speed_val.setProperty("class", "gauge-num")
        lbl_unit = QLabel("KM/H")
        lbl_unit.setProperty("class", "gauge-unit")
        sp_num_box.addWidget(self.lbl_speed_val)
        sp_num_box.addWidget(lbl_unit, 0, Qt.AlignBottom)
        sp_num_box.addStretch(1)

        # 转角与航向角小卡
        steer_box = QVBoxLayout()
        steer_box.setSpacing(1)
        self.lbl_steer_val = QLabel("前轮转角: 0.0°")
        self.lbl_steer_val.setStyleSheet("color: #f8fafc; font-weight: 700; font-size: 11px;")
        self.lbl_heading_val = QLabel("车身航向: 0.0°")
        self.lbl_heading_val.setStyleSheet("color: #94a3b8; font-size: 11px;")
        steer_box.addWidget(self.lbl_steer_val)
        steer_box.addWidget(self.lbl_heading_val)
        sp_num_box.addLayout(steer_box)

        d_layout.addLayout(sp_num_box)

        self.pbar_speed = QProgressBar()
        self.pbar_speed.setRange(0, 120)
        self.pbar_speed.setValue(0)
        d_layout.addWidget(self.pbar_speed)

        # 电子挡位 PRND
        gear_row = QHBoxLayout()
        gear_row.setSpacing(6)
        lbl_gear = QLabel("挡位:")
        lbl_gear.setStyleSheet("font-weight: 700; color: #94a3b8;")
        gear_row.addWidget(lbl_gear)

        self.gear_buttons = {}
        for g_code, g_text in [("P", "P 驻车"), ("R", "R 倒车"), ("N", "N 空挡"), ("D", "D 前进")]:
            btn = QPushButton(g_text)
            btn.setProperty("class", "gear-btn")
            btn.clicked.connect(lambda _c, gear=g_code: self.arbiter.set_gear(gear))
            self.gear_buttons[g_code] = btn
            gear_row.addWidget(btn)
        d_layout.addLayout(gear_row)

        # 灯光与动作指令
        self.lbl_maneuver_txt = QLabel("动作指令: 待命中 (等待高德导航)")
        self.lbl_maneuver_txt.setStyleSheet("font-weight: 700; color: #38bdf8;")
        d_layout.addWidget(self.lbl_maneuver_txt)

        lamp_row = QHBoxLayout()
        lamp_row.setSpacing(4)
        self.chip_turn_left = QLabel("左转向灯")
        self.chip_turn_right = QLabel("右转向灯")
        self.chip_brake = QLabel("制动灯")
        self.chip_reverse = QLabel("倒车灯")
        for chip in (self.chip_turn_left, self.chip_turn_right, self.chip_brake, self.chip_reverse):
            chip.setProperty("class", "indicator-chip")
            chip.setAlignment(Qt.AlignCenter)
            lamp_row.addWidget(chip)
        d_layout.addLayout(lamp_row)

        layout.addWidget(dyn_card)

        # 2. 智驾接管与座舱自主导航规划卡片
        ctrl_card = QFrame()
        ctrl_card.setProperty("class", "hud-card")
        c_layout = QVBoxLayout(ctrl_card)
        c_layout.setContentsMargins(10, 8, 10, 8)
        c_layout.setSpacing(6)

        # 智驾运行模式分段切换 (取代旧的单个接管按钮)
        m_top = QHBoxLayout()
        lbl_mode_t = QLabel("智驾运行模式")
        lbl_mode_t.setProperty("class", "card-title")
        self.lbl_mode_badge = QLabel("座舱仿真联调")
        self.lbl_mode_badge.setStyleSheet("color: #38bdf8; font-weight: 700; font-size: 11px;")
        m_top.addWidget(lbl_mode_t)
        m_top.addStretch(1)
        m_top.addWidget(self.lbl_mode_badge)
        c_layout.addLayout(m_top)

        mode_row = QHBoxLayout()
        mode_row.setSpacing(4)

        self.btn_mode_cockpit = QPushButton("座舱联调")
        self.btn_mode_cockpit.setProperty("class", "mode-btn active-cockpit")
        self.btn_mode_cockpit.setToolTip("无 RK 板卡环境：座舱全权导航，CARLA 地图高精循迹，感知旁路")
        self.btn_mode_cockpit.clicked.connect(lambda: self._set_system_mode("cockpit_sim"))
        mode_row.addWidget(self.btn_mode_cockpit, 1)

        self.btn_mode_perception = QPushButton("感知单机")
        self.btn_mode_perception.setProperty("class", "mode-btn")
        self.btn_mode_perception.setToolTip("无座舱环境：上位机自建虚拟巡航并向 RK 推流，验证 RK 目标检测与 AEB 闭环")
        self.btn_mode_perception.clicked.connect(lambda: self._set_system_mode("perception_debug"))
        mode_row.addWidget(self.btn_mode_perception, 1)

        self.btn_mode_full_ad = QPushButton("全域智驾")
        self.btn_mode_full_ad.setProperty("class", "mode-btn")
        self.btn_mode_full_ad.setToolTip("全要素真机闭环：座舱下发导航 + RK 车道线变道许可 + RK 行人 AEB 紧急制动")
        self.btn_mode_full_ad.clicked.connect(lambda: self._set_system_mode("full_ad"))
        mode_row.addWidget(self.btn_mode_full_ad, 1)

        c_layout.addLayout(mode_row)

        # 座舱导航状态 (纯动态接收，无固定路线)
        r_top = QHBoxLayout()
        lbl_rc = QLabel("座舱导航目标")
        lbl_rc.setProperty("class", "card-title")
        self.lbl_remain_countdown = QLabel("目标: 都会立交 (待命)")
        self.lbl_remain_countdown.setStyleSheet("color: #38bdf8; font-weight: 700;")
        r_top.addWidget(lbl_rc)
        r_top.addStretch(1)
        r_top.addWidget(self.lbl_remain_countdown)
        c_layout.addLayout(r_top)

        lbl_hint = QLabel("路线由车机座舱动态下发 · 上位机执行车辆动力学与自主巡航")
        lbl_hint.setStyleSheet("color: #64748b; font-size: 11px;")
        c_layout.addWidget(lbl_hint)

        self.pbar_route = QProgressBar()
        self.pbar_route.setRange(0, 100)
        self.pbar_route.setValue(100)
        c_layout.addWidget(self.pbar_route)

        layout.addWidget(ctrl_card)

        # 3. 硬件总线互联与仿真操作卡片
        bus_card = QFrame()
        bus_card.setProperty("class", "hud-card")
        b_layout = QVBoxLayout(bus_card)
        b_layout.setContentsMargins(10, 8, 10, 8)
        b_layout.setSpacing(6)

        # 座舱串口选择
        s_row = QHBoxLayout()
        s_row.setSpacing(6)
        lbl_serial = QLabel("座舱串口:")
        lbl_serial.setStyleSheet("font-weight: 700; color: #94a3b8;")
        s_row.addWidget(lbl_serial)

        self.cb_ports = QComboBox()
        self.cb_ports.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self._refresh_ports()
        s_row.addWidget(self.cb_ports, 1)

        self.btn_serial = QPushButton("连接")
        self.btn_serial.setProperty("class", "hud-btn primary")
        self.btn_serial.clicked.connect(self._toggle_serial)
        s_row.addWidget(self.btn_serial)
        b_layout.addLayout(s_row)

        # CARLA 与仿真按钮
        sim_row = QHBoxLayout()
        sim_row.setSpacing(6)
        self.btn_carla = QPushButton("连接 CARLA 3D")
        self.btn_carla.setProperty("class", "hud-btn")
        self.btn_carla.clicked.connect(self._toggle_carla)
        sim_row.addWidget(self.btn_carla, 1)

        self.btn_toggle_sim = QPushButton("暂停仿真")
        self.btn_toggle_sim.setProperty("class", "hud-btn")
        self.btn_toggle_sim.clicked.connect(self._toggle_sim)
        sim_row.addWidget(self.btn_toggle_sim, 1)

        self.btn_reset_sim = QPushButton("重置自车")
        self.btn_reset_sim.setProperty("class", "hud-btn")
        self.btn_reset_sim.clicked.connect(self.arbiter.reset_vehicle)
        sim_row.addWidget(self.btn_reset_sim, 1)
        b_layout.addLayout(sim_row)

        layout.addWidget(bus_card)

        # 4. 实时交互诊断终端卡片
        log_card = QFrame()
        log_card.setProperty("class", "hud-card")
        l_layout = QVBoxLayout(log_card)
        l_layout.setContentsMargins(10, 8, 10, 8)
        l_layout.setSpacing(4)

        log_top = QHBoxLayout()
        lbl_lt = QLabel("总线诊断日志终端")
        lbl_lt.setProperty("class", "card-title")
        log_top.addWidget(lbl_lt)
        log_top.addStretch(1)

        btn_clear = QPushButton("清空")
        btn_clear.setProperty("class", "hud-btn")
        btn_clear.clicked.connect(lambda: self.txt_log.clear())
        log_top.addWidget(btn_clear)
        l_layout.addLayout(log_top)

        self.txt_log = QPlainTextEdit()
        self.txt_log.setReadOnly(True)
        self.txt_log.setMinimumHeight(150)
        l_layout.addWidget(self.txt_log)

        layout.addWidget(log_card)
        layout.addStretch(1)

        scroll.setWidget(container)
        main_vbox.addWidget(scroll)
        return pane

    # =========================================================================
    # 视频拉流工作线程
    # =========================================================================
    def _start_video_stream(self) -> None:
        if self._video_running:
            return
        self._video_running = True
        self._video_thread = threading.Thread(target=self._video_stream_worker, daemon=True)
        self._video_thread.start()

    def _restart_video_stream(self) -> None:
        self._video_running = False
        if self._video_thread and self._video_thread.is_alive():
            self._video_thread.join(timeout=0.3)
        self._start_video_stream()

    def _video_stream_worker(self) -> None:
        candidate_urls = [
            BOARD_STREAM_URL,
            "http://127.0.0.1:8080/stream",
            "http://127.0.0.1:8081/stream",
        ]
        url_idx = 0
        while self._video_running:
            url = candidate_urls[url_idx % len(candidate_urls)]
            resp = None
            try:
                self.sig_video_status.emit(f"视频流连接中: {url}")
                req = urllib.request.Request(url, headers={"User-Agent": "HIL_DualPane_Monitor/2.0"})
                resp = urllib.request.urlopen(req, timeout=2.5)
                self.sig_video_status.emit(f"视频流在线: {url} (MJPEG)")
                buf = b""
                while self._video_running:
                    chunk = resp.read(8192)
                    if not chunk:
                        break
                    buf += chunk
                    start = buf.find(b"\xff\xd8")
                    end = buf.find(b"\xff\xd9", start + 2) if start != -1 else -1
                    if start != -1 and end != -1:
                        jpg = buf[start:end + 2]
                        buf = buf[end + 2:]
                        pix = QPixmap()
                        if pix.loadFromData(jpg):
                            self.sig_video_frame.emit(pix)
            except Exception:
                url_idx += 1
                time.sleep(1.0)
            finally:
                if resp is not None:
                    try:
                        resp.close()
                    except Exception:
                        pass

    def _on_video_frame(self, pix: QPixmap) -> None:
        scaled = pix.scaled(self.lbl_video_screen.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.lbl_video_screen.setPixmap(scaled)

    def _on_video_status(self, text: str) -> None:
        self.lbl_video_status.setText(text)

    # =========================================================================
    # 交互控制槽函数
    # =========================================================================
    def _refresh_ports(self) -> None:
        self.cb_ports.clear()
        ports = SerialClient.list_ports()
        for dev, label in ports:
            self.cb_ports.addItem(label, dev)

    def _auto_connect_serial(self) -> None:
        pref = SerialClient.find_preferred_port()
        if pref:
            for i in range(self.cb_ports.count()):
                if self.cb_ports.itemData(i) == pref:
                    self.cb_ports.setCurrentIndex(i)
                    break
            self.arbiter.serial.connect(pref)
            self.btn_serial.setText("断开")
            update_widget_class(self.btn_serial, "hud-btn danger")

    def _toggle_serial(self) -> None:
        if self.arbiter.serial.is_connected():
            self.arbiter.serial.disconnect()
            self.btn_serial.setText("连接")
            update_widget_class(self.btn_serial, "hud-btn primary")
        else:
            dev = self.cb_ports.currentData()
            if dev:
                ok = self.arbiter.serial.connect(dev)
                if ok:
                    self.btn_serial.setText("断开")
                    update_widget_class(self.btn_serial, "hud-btn danger")

    def _toggle_carla(self) -> None:
        if self.arbiter.carla.connected:
            self.arbiter.disconnect_carla()
            self.btn_carla.setText("连接 CARLA 3D")
            update_widget_class(self.btn_carla, "hud-btn")
        else:
            ok = self.arbiter.connect_carla()
            if ok:
                self.btn_carla.setText("断开 CARLA 3D")
                update_widget_class(self.btn_carla, "hud-btn success")

    def _toggle_sim(self) -> None:
        new_state = not self.arbiter.state.simulation_running
        self.arbiter.set_simulation_running(new_state)
        self.btn_toggle_sim.setText("暂停仿真" if new_state else "继续仿真")

    def _set_system_mode(self, mode: str) -> None:
        self.arbiter.set_system_mode(mode)

    def _toggle_perception_source(self) -> None:
        curr = getattr(self.arbiter, "_perception_source", "pcie")
        next_src = "carla" if curr == "pcie" else "pcie"
        self.arbiter.set_perception_source(next_src)

    def _toggle_rk_decision(self) -> None:
        if not self._decision_active:
            if self.decision_panel is None:
                from ui.rk_decision_panel import DecisionPanel
                self.decision_panel = DecisionPanel(self)
                self.mode_root_layout.addWidget(self.decision_panel, 1)
            self._decision_active = True
            self.arbiter.set_simulation_running(False, notify_cockpit=False)
            self.btn_toggle_sim.setText("继续仿真")
            self._video_running = False
            self.popup_window.close()
            self.normal_splitter.hide()
            self.btn_pop_video.hide()
            for pill in (self.pill_cockpit, self.pill_amap, self.pill_adas, self.pill_sim):
                pill.hide()
            self.decision_panel.show()
            self.decision_panel.activate()
            self.btn_rk_decision.setText("返回四路 / 导航")
            self.arbiter.log("【单镜头决策】已暂停原导航，RK判断按钮请求；独立演示车辆保持静止。")
        else:
            self._decision_active = False
            self.decision_panel.deactivate()
            self.decision_panel.hide()
            self.normal_splitter.show()
            self.btn_pop_video.show()
            for pill in (self.pill_cockpit, self.pill_amap, self.pill_adas, self.pill_sim):
                pill.show()
            self.btn_rk_decision.setText("单镜头决策演示")
            self._resume_normal_video()
            self.arbiter.log("【单镜头决策】已退出；原导航保持暂停，可手动继续。")

    def _resume_normal_video(self) -> None:
        if self._decision_active:
            return
        if self._video_thread and self._video_thread.is_alive():
            QTimer.singleShot(200, self._resume_normal_video)
            return
        self._start_video_stream()

    def _toggle_popup_video(self) -> None:
        if self.popup_window.isVisible():
            self.popup_window.hide()
        else:
            self.popup_window.show()

    def _toggle_always_on_top(self) -> None:
        self._is_always_on_top = not self._is_always_on_top
        self.setWindowFlag(Qt.WindowStaysOnTopHint, self._is_always_on_top)
        self.show()
        if self._is_always_on_top:
            self.btn_topmost.setText("置顶: 开")
            update_widget_class(self.btn_topmost, "hud-btn primary")
        else:
            self.btn_topmost.setText("置顶: 关")
            update_widget_class(self.btn_topmost, "hud-btn")

    # =========================================================================
    # 状态渲染刷新
    # =========================================================================
    def _render_state(self, state: VehicleState) -> None:
        if self._decision_active:
            return
        # 1. 速度与限速
        spd = state.actual_speed_kmh
        self.lbl_speed_val.setText(f"{spd:.1f}")
        self.pbar_speed.setValue(int(min(120, max(0, spd))))
        self.lbl_speed_limit_badge.setText(f"限速 {state.speed_limit_kmh} km/h")

        # 2. 转向角与航向角
        self.lbl_steer_val.setText(f"前轮转角: {state.steering_angle:+.1f}°")
        self.lbl_heading_val.setText(f"车身航向: {math.degrees(state.heading_rad):.1f}°")

        # 3. 挡位按钮高亮
        for g, btn in self.gear_buttons.items():
            cls_name = f"gear-btn active-{g.lower()}" if g == state.gear else "gear-btn"
            update_widget_class(btn, cls_name)

        # 4. 动作与灯光芯片
        if not state.has_cockpit_nav:
            self.lbl_maneuver_txt.setText(f"待命中: {state.wait_nav_reason or '等待高德导航'}")
            self.lbl_maneuver_txt.setStyleSheet("font-weight: 700; color: #fbbf24;")
        else:
            self.lbl_maneuver_txt.setText(f"动作: {state.current_maneuver or '直行巡航'}")
            self.lbl_maneuver_txt.setStyleSheet("font-weight: 700; color: #38bdf8;")

        update_widget_class(self.chip_turn_left, "indicator-chip amber-on" if state.turn_signal == "left" else "indicator-chip")
        update_widget_class(self.chip_turn_right, "indicator-chip amber-on" if state.turn_signal == "right" else "indicator-chip")
        update_widget_class(self.chip_brake, "indicator-chip red-on" if state.brake_lamp else "indicator-chip")
        update_widget_class(self.chip_reverse, "indicator-chip blue-on" if state.reverse_lamp else "indicator-chip")

        # 5. 路线倒计时与进度
        seg_m = state.segment_remaining_distance_m
        tot_m = state.route_total_distance_m or 1.0
        rem_m = state.route_remaining_distance_m
        self.lbl_remain_countdown.setText(f"{state.route_name} (剩余 {rem_m:.0f}m)")
        pct = int(min(100, max(0, (1.0 - rem_m / tot_m) * 100))) if tot_m > 0 else 0
        self.pbar_route.setValue(pct)
        # 6. 智驾运行模式分段按钮高亮与状态更新
        cur_mode = getattr(state, "system_mode", "cockpit_sim")
        update_widget_class(self.btn_mode_cockpit, "mode-btn active-cockpit" if cur_mode == "cockpit_sim" else "mode-btn")
        update_widget_class(self.btn_mode_perception, "mode-btn active-perception" if cur_mode == "perception_debug" else "mode-btn")
        update_widget_class(self.btn_mode_full_ad, "mode-btn active-ad" if cur_mode == "full_ad" else "mode-btn")

        if cur_mode == "cockpit_sim":
            self.lbl_mode_badge.setText("座舱仿真联调 (无RK)")
            self.lbl_mode_badge.setStyleSheet("color: #38bdf8; font-weight: 700; font-size: 11px;")
        elif cur_mode == "perception_debug":
            self.lbl_mode_badge.setText("感知算法单机 (无车机)")
            self.lbl_mode_badge.setStyleSheet("color: #c084fc; font-weight: 700; font-size: 11px;")
        elif cur_mode == "full_ad":
            if getattr(state, "vision_udp_connected", False):
                self.lbl_mode_badge.setText("全域智驾闭环 (双脑就绪)")
                self.lbl_mode_badge.setStyleSheet("color: #34d399; font-weight: 700; font-size: 11px;")
            else:
                self.lbl_mode_badge.setText("全域智驾 [RK离线]")
                self.lbl_mode_badge.setStyleSheet("color: #f87171; font-weight: 700; font-size: 11px;")

        # 7. 感知源徽章更新
        src = getattr(state, "perception_source", "pcie").upper()
        self.lbl_video_source_badge.setText(f"源: {src}")

        # 8. ADAS 安全决策矩阵
        act = getattr(state, "vision_action", "") or "正常巡航"
        veto_reason = getattr(state, "vision_veto", "") or "无"
        if getattr(state, "vision_aeb_active", False):
            self.lbl_aeb_pill.setText("AEB 紧急制动触发")
            update_widget_class(self.lbl_aeb_pill, "status-pill alert")
            self.lbl_decision_action.setText(f"紧急制动: {act}")
            self.lbl_decision_action.setStyleSheet("font-size: 14px; font-weight: 800; color: #f87171;")
        elif getattr(state, "vision_speed_override", None) is not None:
            self.lbl_aeb_pill.setText("主动控速避让")
            update_widget_class(self.lbl_aeb_pill, "status-pill active")
            self.lbl_decision_action.setText(f"控速巡航: 限速 {state.vision_speed_override:.0f} km/h ({act})")
            self.lbl_decision_action.setStyleSheet("font-size: 14px; font-weight: 800; color: #60a5fa;")
        else:
            self.lbl_aeb_pill.setText("AEB 待命中")
            update_widget_class(self.lbl_aeb_pill, "status-pill online")
            self.lbl_decision_action.setText(f"规划指令: {act}")
            self.lbl_decision_action.setStyleSheet("font-size: 14px; font-weight: 800; color: #34d399;")

        self.lbl_decision_veto.setText(f"制动阻停原因: {veto_reason} | 节点: {getattr(state, 'vision_udp_endpoint', '--')}")

        # 9. 顶栏四域连接药丸
        if state.serial_connected:
            p_name = state.serial_port_name or "UART"
            self.pill_cockpit.setText(f"座舱域: {p_name}")
            update_widget_class(self.pill_cockpit, "status-pill online")
            self.btn_serial.setText("断开")
            update_widget_class(self.btn_serial, "hud-btn danger")
        else:
            self.pill_cockpit.setText("座舱域: 未连")
            update_widget_class(self.pill_cockpit, "status-pill")
            self.btn_serial.setText("连接")
            update_widget_class(self.btn_serial, "hud-btn primary")

        if state.has_cockpit_nav:
            self.pill_amap.setText(f"高德协议: 引导中 ({state.segment_remaining_distance_m:.0f}m)")
            update_widget_class(self.pill_amap, "status-pill online")
        else:
            self.pill_amap.setText("高德协议: 未接收")
            update_widget_class(self.pill_amap, "status-pill")

        if state.vision_udp_connected:
            self.pill_adas.setText("智驾域: 已连通")
            update_widget_class(self.pill_adas, "status-pill online")
        else:
            self.pill_adas.setText("智驾域: 离线")
            update_widget_class(self.pill_adas, "status-pill")

        if state.carla_connected:
            self.pill_sim.setText("底盘: CARLA 3D")
            update_widget_class(self.pill_sim, "status-pill online")
        else:
            self.pill_sim.setText("底盘: 运动学")
            update_widget_class(self.pill_sim, "status-pill")

        self.btn_toggle_sim.setText("暂停仿真" if state.simulation_running else "继续仿真")

    def _append_log(self, line: str) -> None:
        self.txt_log.appendPlainText(line)
        if self.txt_log.blockCount() > 120:
            cursor = self.txt_log.textCursor()
            cursor.movePosition(cursor.Start)
            cursor.movePosition(cursor.Down, cursor.KeepAnchor, 40)
            cursor.removeSelectedText()

    def closeEvent(self, event) -> None:
        if self.decision_panel is not None:
            self.decision_panel.shutdown()
        self._video_running = False
        if hasattr(self, "popup_window") and self.popup_window:
            self.popup_window.close()
        self.arbiter.close()
        event.accept()


def main() -> None:
    app = QApplication(sys.argv)
    window = DashboardWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
