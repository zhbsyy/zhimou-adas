# YOLOv8n 11类正式训练与模型来源

## 已提交的文件

|文件|用途|
|---|---|
|`src/train_joint_v2.py`|原A正式BF16训练入口|
|`train_config.json`|实际正式run的分阶段协议副本|
|`data_config.yaml`|原数据资产根目录、类别顺序及coverage约定|
|`src/common.py`、`src/initialize.py`|路径、种子及原10→11类初始化|
|`src/data_runtime.py`、`src/data_runtime_v2.py`|原标签/coverage读取、增广与固定采样|
|`src/partial_loss.py`、`src/partial_loss_v2.py`|UNKNOWN屏蔽、正常TAL/CIoU/DFL与原0–9类教师保护|
|`src/train_joint.py`、`src/numerical_runtime_v2.py`、`src/evaluate_v2.py`|正式入口实际使用的优化器/阶段/EMA/保存及验证选模依赖|
|`reference_conversion/`|原有九输出导出、转换、后处理、历史记录和当时vendor，保留原路径|

正式源码及配置来自 `F:/current9-11/End/yolov8n_11class_detect_20261019`，经过 `FPGA_ADAS_Submission_Staging` 精简归档后补入。本目录不加入v1/v2/v3/P2-B/P3-B等后续探索训练配方；`train_joint.py`只保留正式入口实际复用的原辅助函数。

## 原A权重与部署关系

当前部署模型是项目根 `models/object_detection/A_rk3568_int8.rknn`；11类顺序以 `models/object_detection/classes.txt` 为准，class10=crosswalk。

现有 `models/object_detection/reference/A_original.pt` 根据既有模型来源记录对应原A正式 `runs/formal_joint_v2_bf16_seed20261020/best.pt`。正式run共30轮、23,832次更新，采用第14轮EMA best。保留现有归档名称和目录，不复制另一份权重、不替换RKNN。

历史记录中的PT标识为 `7a26be2cf2d245df134679e999fa1c7bfadd1a3e766650a8d8254696554b06ff`，板端RKNN标识为 `65111dd43d15d289d6001657832b77bb4e68a1afa277684564517eef8c4d9543`。这些引用已有记录，本次未重新计算。已有PT→九输出ONNX导出与实际板端模型身份资料；原Ubuntu的 `conversion_result.json` 仍未归档，因此不宣称原编译/校准过程已完整自包含或可逐字节重建。

PT是PyTorch训练模型；INT8量化发生在RKNN构建阶段，不能因为文件名含int8就将PT视作INT8 RKNN。

## 训练入口

先按 [YOLO环境说明](../../environment/yolo_requirements.md) 准备独立训练环境并恢复外部输入。从项目根执行：

```powershell
python -B training/object_detection/src/train_joint_v2.py
```

原训练采用seed20261020、640输入、microbatch16/累积2、AdamW、分阶段冻结与解冻、冻结BN统计、BF16前向/FP32损失、旧类保护及原验证选模；实际参数以代码和 `train_config.json` 为准。不能改用普通 `YOLO.train(data=...)` 替代coverage-aware训练器。

`data_config.yaml`中的本机目录是来源记录及默认输入位置，不是GitHub使用者自动具备的目录。异机需恢复图片、完整manifest及lock、10类/11类初始化权重和记录、原数值门记录，调整根目录及外部清单中的图片路径。数据与这些初始化输入不随本仓库提供；已有模型推理不依赖它们。新训练输出在本目录的 `runs/` 下，不写入最终 `models/`。

## 保留的参考导出方式

仅维护转换材料时，在独立转换环境安装 `reference_conversion/vendor`，从项目根运行：

```powershell
python -m pip install -e training/object_detection/reference_conversion/vendor
python -B training/object_detection/reference_conversion/export_reference.py --image "C:/path/road.jpg" --output results/model_export/detector_reference_new
```

该vendor版本为8.3.163，正式训练环境为Ultralytics8.4.155；不要用前者覆盖训练环境。原A检测校准图未并入此归档，不可用R15的256×448图代替640×640检测校准图。当前演示不需要执行导出或量化；新候选不得直接覆盖已部署模型。
