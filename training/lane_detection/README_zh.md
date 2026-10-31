# R15 TwinLiteNet+ Small 训练与转换

## 已保存的材料

本目录保留原项目相对布局，`scripts/training/model_adapter.py`、`dataset.py`、`third_party/TwinLiteNetPlus/model` 可被历史训练脚本直接导入。`runs/R15_external_data_training/train_r15_lanefocus.py` 是最后一轮训练保存的源码快照，SHA256与 `records/final_run/config.json` 一致。

| 材料 | 位置 |
|---|---|
| 最后阶段训练配置、35轮指标与加载记录 | `records/final_run` |
| 训练/验证CSV快照 | `runs/R15_external_data_training/data_v1` |
| 上游Small初始化权重 | `checkpoints/upstream_small.pth` |
| 最后阶段初始化权重 | `runs/R15_external_data_training/candidate_checkpoints/targetmix_e15.pt` |
| 选中的第32轮权重 | 项目根 `models/lane_detection/R15_twinlitenet_small_direct2_lane3_research.pt` |
| 两份256×448 ONNX | 项目根 `models/lane_detection/onnx` |
| 原始ONNX等价性报告 | `records/export_report.json` |
| 导出、GCN等价改写、量化脚本 | `conversion` |
| 160张256×448量化输入与清单 | `conversion/calibration_v1` |

训练输入是384×640；部署导出缩到256×448。模型输出direct两类、lane三类（背景/实线/虚线），不能把训练尺寸和部署尺寸混为一谈。历史 `ORIGINAL_MODEL_STATUS.md` 是当时“研究候选”的原记录，后续项目使用它做了256×448部署试验；本次没有重写历史结论。

关键哈希：PT `664b642c8476fc1bdcfb39dec04db05c8173651b398a8cf711a92918338a77c0`；修补ONNX `34c21d6d1cc2d2a595ab2b68c2f4289cec6bea43b88dd4599a35ccc00f51298b`；当前部署RKNN `65a233fef461d89f1413bddc9d0316e24aaa310512caac686d88bab388c8c289`。PT→ONNX有历史数值对照；仅凭这些文件不能保证重新量化得到逐字节相同的RKNN。

## 重训最后阶段

这不是日常演示步骤。在训练电脑准备PyTorch、NumPy、OpenCV、Matplotlib；依赖清单见 `requirements_training.txt`。上游原requirements也保留供溯源，其中旧torch版本并非本次训练环境锁文件，不要直接拿它覆盖现有环境。

先恢复CSV里所有 `image_path`、`direct_mask_path`、`lane_mask_path` 指向的原始图片/掩膜。原始数据集未归档，未恢复前不能完整重训。在有数据的训练电脑，从本项目根进入 `training/lane_detection`，复用最后阶段参数：

```powershell
python -B runs/R15_external_data_training/train_r15_lanefocus.py --run reproduced_dashed_v4 --init-from runs/R15_external_data_training/candidate_checkpoints/targetmix_e15.pt --epochs 35 --batch 12 --lr 0.00004 --workers 0 --patience 15 --samples-per-epoch 2400 --synthetic-fraction 0.2 --human-fraction 0.1 --rlmd-fraction 0.3 --a2d2-fraction 0.25 --freeze-bn --lane-only --background-weight 0.5 --solid-weight 1.8 --dashed-weight 2.2
```

新结果写到 `runs/R15_external_data_training/reproduced_dashed_v4`，不覆盖历史记录。它复现最后一段微调，不表示从零重建全部初始化链。数据路径迁移后CSV哈希会变化，应保存新的配置/清单，勿宣称与历史文件哈希相同。

## 重新导出ONNX（可选）

在本项目根，用训练电脑Python执行。输出用全新目录，脚本拒绝覆盖既有导出文件：

```powershell
python -B training/lane_detection/conversion/export_small_trial.py --project training/lane_detection --checkpoint models/lane_detection/R15_twinlitenet_small_direct2_lane3_research.pt --output-dir results/model_export/r15_new
```

它调用已归档的 `runs/R13_competition_video/r13_video.py` 中 `load_model`，只使用该文件的模型加载函数，不运行旧的双模型视频比较流程。导出后检查 `export_report.json`，再做量化。

## Linux RKNN-Toolkit2 INT8转换（可选）

历史环境是原生Ubuntu、`/home/zhbsyy/venvs/rknn230`、RKNN-Toolkit2 2.3.0，目标rk3568。把本项目复制到Ubuntu；在本项目根执行：

```bash
source /home/zhbsyy/venvs/rknn230/bin/activate
python -B training/lane_detection/conversion/convert_small_linux.py --onnx models/lane_detection/onnx/R15_256x448_gcn_matmul.onnx --out results/model_conversion/R15_rebuilt_int8.rknn --precision int8 --calibration-dir training/lane_detection/conversion/calibration_v1/input_256x448
```

RKNN输入是RGB uint8 NHWC，转换配置在模型内除以255；运行时不能再重复归一化。新RKNN需要单独验证数值、效果和实板耗时，不能直接覆盖 `models/lane_detection/R15_256x448_patched_native_int8.rknn`。现有运行入口不会读取 `results/model_conversion` 的候选文件。

## 补充的前两阶段及评估材料（2026-10-31）

本次从 `F:/R15_Lane_Model_Training_20261031` 增量补入下列文件，保持原相对目录，不复制整个独立训练包。现有 `train_r15_lanefocus.py` 与独立包的最终阶段 `train_stage3_snapshot.py` 内容一致，因此保留现有入口，没有覆盖或再放一份最终阶段脚本。

|阶段/用途|本目录内位置|历史实际轮数与初始化|
|---|---|---|
|第一主阶段|`runs/R15_external_data_training/train_stage1_snapshot.py`；`long_laneonly_a2d2_v2/config.json`|实际25轮；配置上限70；从R12初始化|
|第二主阶段|`runs/R15_external_data_training/train_stage2_snapshot.py`；`long_target_refine_v3/config.json`|实际16轮；配置上限45；从第一阶段第25轮初始化|
|最终主阶段（原已存在）|`runs/R15_external_data_training/train_r15_lanefocus.py`；`records/final_run/config.json`|实际35轮；选第32轮；从第二阶段第15轮初始化|
|固定验证评估|`runs/R15_external_data_training/evaluate_r15.py`|依赖同层新增的原始 `train_r15.py` 提供数据读取和模型构建函数|
|阶段和模型来源|`records/MODEL_LINEAGE.json`|保留独立包的历史阶段、尺寸和来源记录，未重算摘要|

表中的短目录名均位于 `runs/R15_external_data_training/` 下。新增配置目录只收录配置，不表示同时包含其历史日志和检查点。第一、二阶段的25/16轮是历史实际完成轮数，不能把70/45轮配置上限当成实际训练成果，也不保证简单重跑上限即可复现当时人工停止位置。

最终阶段初始化 `candidate_checkpoints/targetmix_e15.pt` 原已在本目录。若要复现更早阶段，还需另行提供 `runs/R12_china_carla_autotrain/selected_model/R12_twinlitenet_small_direct2_lane3.pt` 和 `runs/R15_external_data_training/candidate_checkpoints/laneonly_best_e25.pt`；它们保存在独立训练包中，本次为精简提交不复制。所有阶段仍需恢复训练/验证CSV指向的图片和掩膜。本次新增材料是源码与来源补充，不是从零重建全部训练输入的自包含数据包。

恢复验证数据后，从 `training/lane_detection` 运行既有模型评估：

```powershell
python -B runs/R15_external_data_training/evaluate_r15.py --checkpoint ../../models/lane_detection/R15_twinlitenet_small_direct2_lane3_research.pt --out runs/R15_external_data_training/eval_reproduced --lane-threshold 0.6
```

这里0.6是车道分割评估阈值，与YOLO class10阈值无关。输出目录需为新目录；不要在未恢复数据时执行。此次没有训练、运行评估、计算哈希或修改任何部署模型；新增文件也不冒称已被旧整包校验清单覆盖。
