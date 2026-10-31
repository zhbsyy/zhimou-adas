# 已安装的运行环境

交付模块包含业务源码、两份 RKNN 模型、模型类别表、解码器源码和预检视频。Python、CARLA 引擎、RK 系统驱动和 Android 已安装应用属于运行环境，不能用复制业务文件夹代替。

## Windows PC

当前入口使用 `D:python3.12.4python.exe` / `pythonw.exe`，CARLA `D:CARLA_download`。两份 CMD 和 `launch_console.py` 保留已验证的本机路径；若换电脑，需同时调整这些路径，使用与引擎匹配的 CARLA Python 包，并保留 `PythonAPI/carla/agents`。版本快照见 `PC_CURRENT_ENVIRONMENT.json`、`requirements_PC_snapshot.txt`，后者用于复现记录，不要求在当前电脑重复安装或升级。

## RK3568

保留现有 `/home/linaro/yolov8_deploy/.venv-offline` 及板端系统 Python、RKNN runtime/driver、GStreamer、MPP、RGA。本模块不需要原 `03_RK3568_Perception`、`board_files` 或其他候选工程的业务脚本。**不能因此把 `.venv-offline` 当作历史试验目录删除。** 若需要迁移该环境，应在同版 RK 系统上做离线备份和恢复验证；本次没有远程打包系统 Python 或设备驱动。

历史板端已验证环境为 Python3.11、RKNN runtime2.3.0/driver0.9.8、OpenCV4.10.0、NumPy1.26.4；这是既有记录，不是本次重新读取的系统清单。原生接口要求 aarch64 Linux 及 `librknnrt.so`。

`PREPARE_DECODER.sh` 在板端编译本模块私有 `Mpp_Decoder/mpp_native_stage`，需要 g++、pkg-config、GStreamer app/video/allocators 开发文件、RGA 头文件和库。视频运行需要现有 MPP/GStreamer 插件（含 JPEG 编码）及硬件访问权限。源码已独立保存，aarch64 可执行文件由本机编译产生；不要拿 Windows 可执行程序替代。首次部署编译一次即可。

## 树莓派 Android

继续使用当前已验证的座舱应用和串口设置。维护源码在 `Cockpit_Android_RaspberryPi`；本次没有核对树莓派已安装 APK 的二进制版本，也没有重新安装应用。
