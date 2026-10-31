"""CARLA ADAS 自动驾驶仿真与座舱联动系统入口。"""

from __future__ import annotations

import os
import sys

# 确保包路径加入 sys.path
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
if BASE_DIR not in sys.path:
    sys.path.insert(0, BASE_DIR)

from PyQt5.QtWidgets import QApplication
from ui.dashboard import DashboardWindow


def main() -> None:
    try:
        app = QApplication(sys.argv)
        window = DashboardWindow()
        window.show()
        sys.exit(app.exec_())
    except Exception as e:
        import traceback
        traceback.print_exc()
        with open("crash.log", "w", encoding="utf-8") as f:
            traceback.print_exc(file=f)


if __name__ == "__main__":
    main()
