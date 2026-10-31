# 功能演示与使用

## 环境与设备

Windows PC运行总控台与CARLA；RK3568运行视觉模型；PG2L50H承担本地双目及PCIe链路，PGL50H用于物理四路模式的SFP发送。Android座舱通过串口提供导航消息。

PC环境、CARLA路径及网络地址在`configs/pc.json`配置。启动器可通过`FPGA_ADAS_PYTHON`指定Python解释器。RK需具备匹配的NPU Runtime、PCIe驱动及MPP/RGA环境，版本见[环境说明](../environment/README_zh.md)。

以下命令从仓库根目录执行。Linux示例部署位置为`/userdata/zhimou-adas`，也可使用`/userdata`下其他项目目录。

```bash
cd /userdata/zhimou-adas
bash scripts/rk3568/PREPARE_ENVIRONMENT.sh --source "$RK_VENV_SOURCE"
```

`RK_VENV_SOURCE`指向目标板上可用的离线RKNN Python环境。Windows环境不复制到ARM设备。源码仓库不附视频；引用固定自检视频的运行布局需要另行提供对应素材，位置为`assets/videos/video_20260929_112507.mp4`。

## 静止车辆决策

该模式展示道路状态到车辆灯光、前轮和报警的反馈，车身保持静止。

1. PG2L50H加载`hardware/PG2L50H/bitstreams/local_stereo_pg2l50h_20261025.sbit`。
2. RK启动单板采集与决策程序：

```bash
bash scripts/rk3568/INITIALIZE_PCIE.sh decision
bash scripts/rk3568/START_DECISION.sh
```

3. PC运行`scripts/windows/START_CARLA_ENGINE.cmd`和`START_DECISION_CONSOLE.cmd`。
4. 在决策页选择信号灯或线型场景，观察感知状态、行驶请求和车辆反馈。

拍摄屏幕作为输入时，双目测量对应屏幕平面；视频目标的演示距离与实际道路测距区分使用。

## 物理四路视频

PG2L50H加载`cascade_fov_pg2l50h_20261029.sbit`，PGL50H加载`sfp_tx_camera_frame_fix_20261014.sbit`，两者均位于各自`bitstreams`目录。SFP连接发送端SFP0与接收端SFP1；USB相机角色按相机配置绑定。

```bash
bash scripts/rk3568/INITIALIZE_PCIE.sh four_camera
bash scripts/rk3568/START_FOUR_CAMERA.sh
```

PC运行`scripts/windows/START_FOUR_CAMERA_CONSOLE.cmd`，显示四路目标检测和前视分割结果。该模式不需要CARLA或座舱导航。

## 座舱导航与辅助制动仿真

PC运行`START_CARLA_ENGINE.cmd`和`START_CARLA_NAVIGATION_CONSOLE.cmd`，在总控台建立CARLA车辆和四相机。座舱串口使用115200、8N1，端口根据操作系统实际枚举选择。

使用RK感知时，在板端执行：

```bash
bash scripts/rk3568/PREPARE_CARLA_DECODER.sh
bash scripts/rk3568/START_CARLA_INFERENCE.sh
```

在Android座舱选择目的地并开始导航。AEB关闭时展示导航控制；开启时，行人横穿场景结合RK视觉检测与仿真场景关联，执行制动、通行确认和续行，到达目的地后驻车。人工暂停及故障停车需要明确恢复指令。CARLA距离来自仿真场景信息，不作为双目实测距离。

## 离线视频

```bash
bash scripts/rk3568/START_OFFLINE.sh --video "$VIDEO_FILE"
```

`VIDEO_FILE`指向目标板上的道路视频。系统逐帧执行检测与分割，生成`result.mp4`、`result.summary.json`和运行日志。摘要中`complete=true`、`stop_reason=source_eof`表示完整读至源文件结束；片段处理和中断结果按摘要记录。

运行输出集中在`results/rk3568/<profile>`及`results/pc`。板端推理使用Ctrl+C结束；切换单板与级联采集时，加载对应位流并重新初始化PCIe。
