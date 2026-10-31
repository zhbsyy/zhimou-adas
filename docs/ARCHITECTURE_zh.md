# 项目结构、去重和运行方式

## 代码在哪里

| 目录 | 实际内容 |
|---|---|
| hardware/PG2L50H/source/decision | 黑板本地双目决策RTL、约束及PDS项目 |
| hardware/PG2L50H/source/four_camera | 黑板SFP接收、四路视野修复工程 |
| hardware/PGL50H/source/rear_camera | 红板后向双目发送工程 |
| hardware/common/ip | 两板/两种工程重复的IP文件，按内容合并 |
| hardware/*/bitstreams | 当前可直接下载的三份.sbit |
| software/rk3568/video_input | PCIe/MMIO32、USB、单/双目、网络视频、MPP/RGA、编码 |
| software/rk3568/object_detection | YOLOv8解码、阈值策略、RKNN原生接口 |
| software/rk3568/lane_detection | TwinLiteNet+ Small前后处理、掩膜与绘制 |
| software/rk3568/lane_attributes | 实线/虚线长度比例判断 |
| software/rk3568/traffic_light | 参考HSV/红绿像素算法、关联和汇总 |
| software/rk3568/fusion | 各模式主流水线、摘要、规则、行人/AEB、双目ROI测距 |
| software/rk3568/communication | RK接口、OIB/AEB报文、PCIe资源检查和内核驱动 |
| software/rk3568/runtime_support | 原程序内部启动/兼容依赖，外部使用scripts入口 |
| software/pc/oib | 统一总控台布局、视频页和决策页 |
| software/pc/carla | 车辆、路线、行人横穿、静止反馈、四相机推流 |
| software/pc/communication | 串口协议、RK请求、视频接收 |
| software/pc/diagnostics、tools | CARLA相机/制动独立检查、离线MP4检查 |
| software/pc/vendor | 已收入交付的CARLA agents源码 |
| software/raspberry_pi/cockpit_android | 已有Android导航、串口、挡位影像联动维护源码 |
| models/object_detection、lane_detection | 两模型和11类表，各保留一份 |
| training/lane_detection | R15实际训练/导出源码、配置、清单、初始化权重与量化图片 |
| training/object_detection/reference_conversion | 检测模型比较导出源码及依赖；正式训练来源尚未证实 |
| configs/rk3568 | 实际相机校色、标定和决策参数，保留不同模式变体 |

其他目录含义：assets是素材；environment是已安装环境的准备/快照；scripts是唯一用户入口；docs是操作说明；tests是检查；verification是来源与证据；results是生成物。

## 为什么还要生成.runtime

已验证程序用同目录import、相对模型路径、PDS的相对IP引用。把文件直接拖到新分类目录会破坏这些关系。此次不改推理/控制算法，而用 `configs/layouts/*.json` 记录“运行位置→分类源码→SHA256”，启动时仅为选中方式生成兼容布局。

源码、模型和录像在分类目录各存一份；`.runtime`只是自动生成的可运行工作目录。若选多个方式，工作目录中可能出现额外文件副本，这是运行布局，不是交付源码重复。一个共用Linux venv由运行目录链接使用，不复制四份大环境。不要单独编辑.runtime内的代码；有差异时启动会停止，防止覆盖用户修改。保留结果并正常停止后，可换一个全新运行目录重新生成。

`scripts/project.py prepare rk3568_decision` 等命令只准备文件，不开NPU/摄像头/车辆。可通过环境变量FPGA_ADAS_RUNTIME_DIR选择新的工作根目录；普通使用无需设置。

PDS修改工程时先双击 `scripts/windows/PREPARE_FPGA_PROJECTS.cmd`，它会打印并生成 `.runtime/hardware/decision`、`four_camera`、`rear_camera`，在PDS打开里面的project.pds即可。源码/IP已经闭合保存，不引用原Final submission。常规演示直接下载hardware/*/bitstreams中的.sbit，不需要重编译。PDS自动生成的网表、布局数据库和编译中间文件不再当源码保存，原目录仍完整保留它们。

## 保留差异

相同文件名与相同SHA256才合并；同名不同内容放入decision/four_camera/carla_navigation/offline子目录。不同采集器、版本的R15适配或阈值不能仅凭名字覆盖。来源到新位置完整表在verification/source_mapping.json，各模式布局在configs/layouts。

class10：Decision0.50、物理四路0.40、CARLA0.30、离线默认0.60；其他类别0.25。configs/profiles是定档记录，不能只改该JSON就宣称算法阈值已改；当前固定策略仍在相应Python文件/离线命令中。

物理四路检测沿用“一张四宫格缩放到640×640后做一次检测”的原实现，不是四个独立640×640检测任务；Front另外做256×448分割。此次结构整理不改变这项算力与小目标分辨率取舍。

## 数据流

- 物理四路：两板双目→SFP/PCIe＋两USB→RK拼四宫格→检测全部四路、Front分割→RK绘制/压缩→PC解码显示。
- Decision：黑板两目→RK左目检测分割＋后台双目/摘要/规则→视频18089＋规则18088→PC静止车灯/车轮反馈。
- CARLA：PC引擎/四相机→8081→RK双模型→18084视频＋UDP感知→PC导航/AEB仲裁→CARLA控制；树莓派串口输入导航目的地/报文。
- 离线：本地文件→每帧检测后分割→RK绘制→MP4和JSON，既不依赖HTTP预览也不控制车辆。

原四模块文档保存在docs/reference，可能仍写旧目录；新部署以本项目README和USER_GUIDE为准。历史验收记录注明其原版本/路径，不应当作本目录已重新实测。

本机补齐的训练权重、ONNX与比较PT位于models的对应目录，生产运行布局只引用原有两份RKNN。树莓派UART源码保存在单一Android工程；独立倒车应用尚缺，统一说明在software/raspberry_pi/README_zh.md，不再用三个空功能目录占位。补齐来源及边界见SUPPLEMENT_20261031_zh.md。
