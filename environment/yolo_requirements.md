# YOLO模块环境与运行

## 训练环境

原A环境记录：Windows，Python **3.12.14**；PyTorch **2.6.0+cu124**；torchvision **0.21.0+cu124**；Ultralytics **8.4.155**；NumPy **1.26.4**；opencv-python **4.11.0.86**；PyYAML **6.0.3**。RTX 4060 Laptop，CUDA 12.4。

这些版本来自原工程 `environment/runtime_lock.json` 和 `source_requirements_locked.txt`，未复制虚拟环境。主要直接依赖为上述包；未打包安装包或下载新依赖。

## 训练入口和外部数据

唯一入口：`training/object_detection/src/train_joint_v2.py`。`train_joint.py`仅提供原训练实际复用的辅助函数，旧v1训练main已从提交副本移除。

`train_config.json`原样保留正式run协议。实际算法仍在原代码中：seed20261020；640输入；microbatch16、累积2；AdamW；epoch1–3只学习新增分类行，epoch4–10解冻neck/Detect，epoch11起开放backbone6–9；BN统计冻结；BF16前向与FP32损失；旧类教师保护、coverage/UNKNOWN屏蔽与原选模规则均保留。正式run完成30轮，采用第14轮best。

在本机工程数据仍位于原目录的前提下，命令为：

```powershell
cd "F:\智眸领航智能驾驶系统"
& F:\current9-11\End\yolov8n_11class_detect_20261019\.venv\Scripts\python.exe -B training/object_detection/src/train_joint_v2.py
```

**本轮未执行训练命令。** 新输出写入提交副本的 `training/object_detection/runs/`，不会写入原正式run；已有输出目录时原脚本拒绝覆盖。

训练图片、标签和大型manifest不随包提供。`data_config.yaml`记录原资产目录；异机复现须恢复：

- 原图片与标注，以及 `configs/manifest.json`、`configs/MANIFEST_FROZEN.json`。
- 原10类训练初始化 `weights/china_road_10class_INIT.pt`、11类未训练初始化 `weights/eleven_class_untrained_init.pt`、`audit/initialization.json`。
- 原 `numerical_revision_20261020/NUMERICAL_GATE.json` 及原十类工程的初始化来源文件。

这些外部输入目前在原研发工程中；按本次仅收录最终模型的要求，不额外复制初始化模型。原10→11类初始化代码 `src/initialize.py` 已保留。异机迁移需调整 `data_config.yaml` 的根目录和外部manifest中的图片路径；也可用 `YOLO_ASSET_ROOT / YOLO_OLD_ROOT / YOLO_SEG_ROOT / YOLO_CD_ROOT`覆盖根目录。初始化校验引用的BDD/S²TLD/CDSet/CeyMo原工程数据也须按实际路径恢复。

必须使用自定义coverage-aware训练入口，不能改用普通 `YOLO.train(data=...)`，否则会丢失UNKNOWN监督语义。未恢复上述外部资产前，不是可以直接在任意机器完整重训的自包含包。

## 当前工程的RK3568运行

RK3568推理环境以 [当前工程环境说明](README_zh.md) 为准，板端版本已有归档，不用Windows训练版本代填。检测模型保持640×640 letterbox、padding114、BGR转RGB一次、NHWC uint8输入，Python端不重复 `/255`。

九输出为stride8/16/32，每尺度DFL64、class11概率和辅助class sum1；辅助sum不是objectness。DFL reg_max16。各模式的阈值与NMS沿用原运行代码和配置，不因训练资料补入而改变。

本工程有四种实际运行入口，见 [启动速查表](../docs/START_COMMANDS_zh.md)。不要使用旧暂存模块的 `yolo_inference.py` 命令代替本工程入口：此工程未收录那个独立文件输入适配器。

`reference_conversion/vendor`为历史转换用Ultralytics8.3.163，正式训练需要8.4.155，应分别建立环境。原A转换记录 `conversion_result.json` 未随包取得；已有部署RKNN可直接按工程入口使用，本次不重训或重新量化。
