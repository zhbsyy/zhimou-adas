# 补充归档与完整性边界

本次收尾面向“智眸领航智能驾驶系统”仓库，原研发工程不修改。板端部署示例目录仍沿用 `/userdata/Final submission 2`；程序、位流、控制规则和四模式class10阈值均未修改。

## 已补入的实际文件

| 内容 | 新位置 | 用途 |
|---|---|---|
| CARLA 四相机独立检查 | `software/pc/diagnostics/probe_cameras.py` | 排查相机挂载/无画面 |
| CARLA 制动执行器检查 | `software/pc/diagnostics/check_actuator.py` | 检查车辆减速、停止和刹车灯 |
| 离线结果检查 | `software/pc/tools/inspect_result.py` | 读取下载后 MP4 帧数、帧率、时长和同名摘要 |
| R15 训练材料 | `training/lane_detection` | 训练快照、配置、模型定义、初始化权重、数据清单、历史指标、导出/转换代码和校准图片 |
| R15 PyTorch/ONNX | `models/lane_detection` | 历史训练权重及两份256×448导出图；现有RKNN保持原样 |
| YOLO正式训练与转换材料 | `training/object_detection`、`models/object_detection/reference` | 正式训练入口、UNKNOWN/旧类保护、配置、原A PT、保留的转换代码及历史导出资料 |
| 原模块文档及验收记录 | `verification/original_reference` | 原先被归类为历史资料的文件逐字节保留；旧入口/验证脚本仅作溯源 |

三个工具此前被错误归类成“旧PC入口”，现已恢复。完整文件来源及SHA256见 `verification/SUPPLEMENT_PROVENANCE.json`，原交付映射仍在 `verification/source_mapping.json`。

## 三个工具如何使用

离线处理结束后，把 `result.mp4` 拖到 `scripts/windows/INSPECT_OFFLINE_RESULT.cmd` 上。若同目录有 `result.summary.json`，会同时输出。此工具只检查文件，不重新推理。

CARLA引擎启动后，可手动双击 `CHECK_CARLA_CAMERAS.cmd` 或 `CHECK_CARLA_BRAKES.cmd`。前者创建自己的临时车辆和四相机；后者让自己的临时车辆加速后制动。结束时清理本次创建的对象。它们是独立诊断，不是日常总控台启动步骤，也不能代替RK模型/AEB整链路验收。相机检查的固定出生点若被占用会报错，不会移动当前自车。

诊断结果归集到 `results/pc/diagnostics`；原工具还在 `.runtime/pc/Results` 生成一份结果。只检查依赖、不连接CARLA：

```powershell
& 'D:\python3.12.4\python.exe' -B '.\scripts\project.py' pc-tool cameras --check
& 'D:\python3.12.4\python.exe' -B '.\scripts\project.py' pc-tool brakes --check
```

## 提交范围与外部依赖

YOLO正式训练代码和配置已由本机 `FPGA_ADAS_Submission_Staging/training/yolov8n_11class` 补入，详细来源和使用方法见 [目标检测训练说明](../training/object_detection/README_zh.md)，环境见 [YOLO环境说明](../environment/yolo_requirements.md)。这部分代码与配置约55 KiB，不另加重复权重、数据集或日志。

Android网关中的 `GearLinkage.java` 保留调用 `cn.embedded.dualcamera` 的接口。用户已确认独立倒车影像App源码及APK不提交；它属于外部应用，不再列为本仓库待补项。原 `uart`、`reverse_camera`、`surround_camera` 说明已合并在 `software/raspberry_pi/README_zh.md`；物理四路环视实际实现仍位于RK和PC代码。

完整训练图片/掩膜、YOLO大型manifest和初始化输入不随仓库提供，需在重训机器恢复；模型来源及恢复说明与运行入口分开记录。原A的原始Ubuntu转换记录仍未归档，R15材料仍是历史训练资料，不将复制资料称为重新训练、量化或实板验收。

本次只完善文档和提交范围，没有运行测试或重新计算校验值。现有 `MANIFEST.sha256`、来源映射和旧验收资料保留为修改前快照；新增训练文件及本次文档收尾未被重新纳入全量校验。运行源码、模型和布局文件未改，不伪造新的校验通过记录。
