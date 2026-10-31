# 软件环境与依赖边界

## 已经收入本目录的项目文件

- 两个实际使用的 INT8 RKNN、11类顺序、双目参考标定和颜色配置。
- 当前 RK 推理/前后处理/规则/通信代码、MMIO32采集器、实际试验生成的ARM64 `.so`及其C源码。
- 2026-10-30新版总控台的 `bridge/ui` 代码、当前决策面板、CARLA静止车辆代码及 CARLA PythonAPI agents 源码。
- 当前双目位流、PDS项目、全部PDS引用文件、IP和引脚约束、时序报告。
- Linux6.1.99驱动二进制、现存驱动源码、原始44.97秒赛事道路录像（启动自检必需）。

`Verification/PROVENANCE.json` 给出每个拷贝文件的来源和哈希；`FROZEN_VERSION.json` 给出定档组合。历史绝对路径只作为来源证据，不是正式入口的运行依赖。

## RK3568 系统前提

- 已有 aarch64 Linux，当前驱动适配内核6.1.99。
- `/userdata` 正确挂载并有空间保存独立Python环境、录像和结果。
- `/usr/lib/librknnrt.so`：当前已验证 RKNN Runtime2.3.0，NPU驱动0.9.8。
- GStreamer、`gst-launch-1.0`、`rawvideoparse`、`mppjpegenc`、`fdsink`。
- `sudo`、`pciutils`（lspci/setpci）、系统基础Python与动态库、`/dev/mem`和pagemap权限。
- 原离线Python环境：已观测 NumPy1.26.4、OpenCV4.10.0、RKNNLite。先执行独立复制脚本，**不是从Windows的NumPy/OpenCV环境复制给板端**。

正式程序使用 `Python_Environment/venv/bin/python`，不会再用旧项目的解释器。复制脚本还会检查导入路径不指回原venv。系统Python标准库、动态链接器及板端GStreamer/NPU驱动仍来自板端系统，不可删除系统目录。本目录不含整张Linux系统镜像，备份镜像应另行保存。

首次复制后请把 `Board_RK3568/Python_Environment/venv`、`ENVIRONMENT_READY.json` 一并备份；它们在本次Windows整理时尚未从板端导出。恢复到不同内核/系统镜像需要重新核验二进制兼容性，不能直接强制安装旧驱动。

`System_Driver/Driver_Source` 是项目现存驱动源码，来源可追溯，但没有在本次重新编译证明与随包`.ko`的逐字节对应。正式恢复使用已归档二进制，不把重编译驱动列为普通启动步骤。

## Windows 系统前提

- 当前 Python3.12.4，已安装包版本记录在 `PC_Windows/PYTHON_PACKAGES_SNAPSHOT.json`。`requirements_current_environment.txt` 是当前环境的版本记录，当前整理没有联网下载安装。
- CARLA引擎0.9.16，当前位于 `D:/CARLA_download`。本目录带CARLA调用与agents源码，**不复制整个游戏引擎、地图资源或GPU驱动**。
- `settings.json` 可修改已安装Python/CARLA位置及RK地址；源代码目录无固定盘符要求。
- PDS2022.2-SP6.4与PANGO下载线驱动用于下载/重建位流；普通PC总控台不需要PDS运行。

不应将“删除旧项目源码”理解为删除上述已安装软件。旧桌面ADAS源码、旧候选包目录等可在新入口验收和备份后清理，当前交付不执行删除。
