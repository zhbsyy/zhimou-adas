# YOLOv8n 11类道路目标检测

## 任务与监督

模型保留person、rider、car、truck、bus、train、motor、bike、traffic light、traffic sign十类，并新增crosswalk。斑马线按同一物理横道的整组可见区域标注，不同物理组分开。

训练混合BDD100K、S²TLD、CDSet与CeyMo。不同来源提供的类别不同，通过逐图coverage声明有效监督：未提供标签的类别为UNKNOWN，分类负监督被屏蔽；已知正GT参与原TAL分配、CIoU与DFL回归。原十类教师仅在训练期提供保持约束。

## 训练配置

原A训练池27,864张，正式训练30轮、23,832次更新，采用第14轮EMA best。输入640×640，microbatch16、梯度累积2，AdamW。前3轮学习新增分类行，随后开放neck/Detect，第11轮起开放backbone6–9；BN运行统计冻结。BF16前向与FP32损失用于数值稳定。

|文件|作用|
|---|---|
|`src/train_joint_v2.py`|正式训练入口|
|`train_config.json`、`data_config.yaml`|阶段协议与外部资产配置|
|`src/partial_loss*.py`|coverage、UNKNOWN与旧类保持损失|
|`src/data_runtime*.py`|样本读取、增广和固定采样|
|`src/initialize.py`|继承十类模型并扩展新增分类行|
|`src/train_joint.py`、`src/numerical_runtime_v2.py`、`src/evaluate_v2.py`|优化、EMA、状态保存与选模|

## 复现入口

恢复图片、标签、完整manifest、初始化权重与其记录后，在仓库根目录运行：

```powershell
python -B training/object_detection/src/train_joint_v2.py
```

外部根目录由`data_config.yaml`或`YOLO_ASSET_ROOT`等变量设置；输出位于本模块`runs`目录。依赖版本见[训练环境](../../environment/yolo_requirements.md)。

## 模型与转换

原A PT位于`models/object_detection/reference/A_original.pt`，部署模型位于`models/object_detection/A_rk3568_int8.rknn`。九输出包含三尺度DFL64、11类概率和辅助class sum1；sum不作为objectness相乘。

`reference_conversion`保留对应导出工具、后处理和历史vendor。其Ultralytics8.3.163环境与正式训练8.4.155分开。检测校准输入为640×640道路图，不能使用R15的256×448输入替代。原Ubuntu完整转换记录未收录，重新量化结果与既有部署资产分别管理。
