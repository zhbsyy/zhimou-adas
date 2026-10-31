# YOLOv8n训练环境

## 依赖

|组件|正式训练版本|
|---|---|
|Python|3.12.14|
|PyTorch / torchvision|2.6.0+cu124 / 0.21.0+cu124|
|Ultralytics|8.4.155|
|NumPy|1.26.4|
|OpenCV|4.11.0.86|
|PyYAML|6.0.3|

原训练平台为RTX4060 Laptop，CUDA12.4。模型使用BF16前向，TAL、分类、CIoU、DFL及教师约束采用FP32计算。参数与优化器状态保持FP32。

## 训练输入

`training/object_detection/data_config.yaml`配置外部数据资产。恢复图像、检测标签、完整manifest及其锁定记录、10类教师/初始化权重、11类未训练初始化及记录后，从项目根执行：

```powershell
python -B training/object_detection/src/train_joint_v2.py
```

根目录也可由`YOLO_ASSET_ROOT`、`YOLO_OLD_ROOT`、`YOLO_SEG_ROOT`和`YOLO_CD_ROOT`指定。原始数据与初始化输入不随源码提供；配置中的开发路径表示资产来源，部署时按数据所在位置设置。

训练器包含类别coverage、UNKNOWN屏蔽、旧类蒸馏与分阶段冻结，不能用普通单数据集训练调用替代。输出位于`training/object_detection/runs`，与最终部署模型分离。

历史参考转换vendor为Ultralytics8.3.163，仅用于其对应导出材料；正式训练使用上述8.4.155环境。板端运行环境见[运行环境](README_zh.md)。
