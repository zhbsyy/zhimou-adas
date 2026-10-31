# 运行入口

命令从仓库根目录执行，Windows入口位于`scripts/windows`，RK入口位于`scripts/rk3568`。

|功能|Windows入口|RK入口|
|---|---|---|
|静止车辆决策|`START_DECISION_CONSOLE.cmd`|`INITIALIZE_PCIE.sh decision`、`START_DECISION.sh`|
|物理四路观察|`START_FOUR_CAMERA_CONSOLE.cmd`|`INITIALIZE_PCIE.sh four_camera`、`START_FOUR_CAMERA.sh`|
|导航与AEB仿真|`START_CARLA_NAVIGATION_CONSOLE.cmd`|`PREPARE_CARLA_DECODER.sh`、`START_CARLA_INFERENCE.sh`|
|离线视频|使用结果检查工具|`START_OFFLINE.sh --video "$VIDEO_FILE"`|
|运行环境准备|配置PC Python与CARLA|`PREPARE_ENVIRONMENT.sh --source "$RK_VENV_SOURCE"`|
|FPGA维护|`PREPARE_FPGA_PROJECTS.cmd`|使用PDS与对应位流|

CARLA相关模式先运行`START_CARLA_ENGINE.cmd`。同一板端只运行一个推理模式，同一PC只保留一个控制会话。

|数据服务|默认端口|
|---|---:|
|物理四路视频|8083|
|Decision视频 / 规则摘要|18089 / 18088|
|CARLA输入 / RK处理视频|8081 / 18084|
|PC导航状态|18090|

目标IP以`configs/pc.json`与模块配置为准。各端点路径见对应模块文档。
