# 智眸领航智能驾驶系统 — FPGA ADAS

本目录是从原 `Final submission` 整理出的独立项目。原目录不修改。硬件、RK软件、PC软件、树莓派维护源码、模型、配置、素材和启动脚本按职责保存；相同文件合并，不再复制四套模型、录像和总控台。日期：2026-10-31。

**第一次使用请按 [新手完整操作手册](docs/USER_GUIDE_zh.md) 操作。** 需要快速查命令看 [启动速查表](docs/START_COMMANDS_zh.md)，需要看代码位置看 [项目结构说明](docs/ARCHITECTURE_zh.md)。

```text
智眸领航智能驾驶系统/
├── hardware/             # 两种黑板工程、红板工程、IP与可直接下载的位流
├── software/
│   ├── rk3568/           # 视频输入、检测、车道线、灯色、融合决策与通信
│   ├── pc/               # 唯一新版总控台、CARLA、串口/网络通信
│   └── raspberry_pi/     # Android座舱与UART维护源码、影像联动说明
├── models/               # 两份部署INT8 RKNN、11类表及已归档PT/ONNX
├── configs/              # 四模式参数记录、相机校色、部署文件映射、PC配置
├── assets/videos/        # 运行布局预期的自检视频位置，当前源码目录未附
├── training/             # YOLOv8n 11类正式训练代码/配置；R15训练与转换材料
├── environment/          # 共用RK环境准备脚本与现有环境快照
├── scripts/              # Windows双击入口、板端启动、校验/运行目录准备
├── docs/                 # 操作手册、结构、切换、故障及原模块参考文档
├── tests/                # 新结构检查和保留的原始回归测试
├── verification/         # 来源、去重清单、原目录快照与验证结果
└── results/              # 运行生成的日志/离线视频，不属于源码
```

## 四种使用方式

| 方式 | PC双击入口（scripts/windows） | RK脚本（scripts/rk3568） | class10 |
|---|---|---|---|
| 静止车辆决策演示 | START_DECISION_CONSOLE.cmd | START_DECISION.sh | 0.50 |
| 物理四路摄像头 | START_FOUR_CAMERA_CONSOLE.cmd | START_FOUR_CAMERA.sh | 0.40 |
| CARLA座舱导航＋可选AEB | START_CARLA_NAVIGATION_CONSOLE.cmd | START_CARLA_INFERENCE.sh（需要RK时） | 0.30 |
| 离线视频处理 | 不开总控台，下载结果播放 | START_OFFLINE.sh | 默认0.60 |

两模型始终相同：检测 `A_rk3568_int8.rknn`、车道线 `R15_256x448_patched_native_int8.rknn`。四路/CARLA只有Front做车道线，其他三路只做检测；Decision处理左目、后台保留双目；离线把单路视频当作前视。不同模式的阈值、采集和规则不强行合成一份算法。

**推流器在哪里开：** CARLA导航总控台顶部“连接CARLA与四相机”按钮会创建自车、挂载四相机并启动8081推流；不再另开旧推流脚本。Decision右侧静止车辆由决策页自动创建，不用点击导航区的四相机按钮。物理四路不需要CARLA引擎。

本项目的 `.runtime` 是启动脚本按 `configs/layouts` 自动生成的工作目录，用来兼容已验证程序的相对导入/文件路径；它不是第五套源码，不需要手工维护。源码只保存在上面的分类目录。首次启动复制当前选中的文件；已有文件保持原样，发现被手改则停止并提示，绝不静默覆盖。RK环境只复制一次到 `environment/rk3568/venv`，由四模式共享。

PC当前环境为 `D:\python3.12.4`、CARLA `D:\CARLA_download`；RK地址192.168.2.3、PC地址192.168.2.10、树莓派COM12。整个项目上传到 `/userdata/Final submission 2` 后按手册部署；不能直接向无写权限的 `/userdata/` 根目录拖文件。旧总控台正常关闭后再打开新入口，板端同一时间只运行一个推理程序。

本次重组没有重新训练、转换模型、改位流、改变阈值或控制规则，也没有重启当前演示。新入口经过无硬件检查；完整实板场景验收仍应按手册各做一次。原有真实验收证据在 `verification/preserved_evidence`，不冒充新目录的实板重测。

2026-10-31 已补齐三个独立PC工具、R15训练材料和YOLOv8n 11类正式训练入口/配置。来源与提交范围见 [补充归档说明](docs/SUPPLEMENT_20261031_zh.md)，训练步骤见 [训练材料入口](training/README.md)，YOLO训练环境见 [环境说明](environment/yolo_requirements.md)。现有运行直接使用两份已归档RKNN，无需重训或转换。

## 源码提交范围

- 提交FPGA源码/位流、RK3568感知与通信、PC总控台、Android座舱网关、模型，以及YOLO/R15的既有训练和转换代码。
- 独立倒车影像App `cn.embedded.dualcamera` 的源码及APK不在本仓库提交范围。网关保留原挡位联动接口；要使用该外部App，需由使用者自行提供。RK/PC的物理四路环视仍在本仓库中。
- 不附完整训练数据、训练虚拟环境、操作系统或CARLA引擎。训练配置保留原资产路径，迁移时应恢复数据与初始化输入后调整路径，不把仓库称为无需外部数据的完整重训包。
- 仓库显示名称已更新；板端文档仍沿用既定部署目录 `/userdata/Final submission 2`，这与仓库名称可以不同，无需全局替换运行路径。

本次仅完善说明，未运行测试或重新计算校验值。`MANIFEST.sha256`及历史来源清单保留整理前快照；新增训练文件与本次更新的文档不应被声称已通过旧整包清单验证。`VERIFY_PROJECT.cmd`使用旧整包清单，会报告已更新文档的差异；这不代表模型或运行源码被改动。各模式的运行源码布局及模型配置未修改。

当前源码目录未附 `assets/videos/video_20260929_112507.mp4`，本轮未删除或补回视频。部分运行布局仍引用这份自检原片；使用这些模式前须恢复原文件到该路径，否则布局检查会因缺文件拒绝启动。本机原片位于 `F:/current9-11/End/officialVideo/video_20260929_112507.mp4`。源码大小与包含自检视频的运行包大小不同，不把GitHub限制或其他赛道的压缩包限制当作本赛道已确认上限。
