# R15道路结构分割

## 网络与标签

R15采用TwinLiteNet+ Small，输出direct两类与lane三类：背景、实线、虚线。标签255表示忽略。训练输入为384×640，部署导出输入为256×448；尺寸变换由图像与标签同步处理，掩膜采用最近邻缩放。

训练清单包含9,465条记录，验证清单1,190条，来源包括BDD、RLMD、CARLA和A2D2。A2D2提供实虚线监督，其direct标签全部忽略。A2D2训练/验证按采集日期划分；旧来源的场景独立性仍按内部验证范围解释。

## 三阶段训练

|阶段|入口|实际轮数|初始化|
|---|---|---:|---|
|外部道路车道学习|`train_stage1_snapshot.py`|25|R12|
|目标域混合纠偏|`train_stage2_snapshot.py`|16|阶段1第25轮|
|虚线响应调整|`train_r15_lanefocus.py`|35，选第32轮|阶段2第15轮|

入口均位于`runs/R15_external_data_training`。阶段1/2配置在其`long_laneonly_a2d2_v2`、`long_target_refine_v3`子目录，最终阶段配置在`records/final_run`。70/45/35为各配置的轮数上限，前两阶段实际结束于25/16轮。

模型定义和适配位于`scripts/training`及`third_party/TwinLiteNetPlus/model`，训练/验证CSV位于`data_v1`。最终阶段冻结BN、共享特征和direct分支，仅优化车道分支；35轮微调使用2400次采样/轮、batch12、lr4e-5。重复采样呈现不等于新增独立图片。

## 训练与评估

恢复CSV对应的图片、掩膜和初始化输入后，进入本模块目录：

```powershell
python -B runs/R15_external_data_training/train_r15_lanefocus.py --run reproduced_dashed_v4 --init-from runs/R15_external_data_training/candidate_checkpoints/targetmix_e15.pt --epochs 35 --batch 12 --lr 0.00004 --workers 0 --patience 15 --samples-per-epoch 2400 --synthetic-fraction 0.2 --human-fraction 0.1 --rlmd-fraction 0.3 --a2d2-fraction 0.25 --freeze-bn --lane-only --background-weight 0.5 --solid-weight 1.8 --dashed-weight 2.2
python -B runs/R15_external_data_training/evaluate_r15.py --checkpoint ../../models/lane_detection/R15_twinlitenet_small_direct2_lane3_research.pt --out runs/R15_external_data_training/eval_reproduced --lane-threshold 0.6
```

`evaluate_r15.py`使用同层`train_r15.py`的数据与模型函数。0.6是分割评估阈值。原始数据不随仓库提供；前两阶段还需R12及`laneonly_best_e25.pt`初始化文件，最终阶段初始化`targetmix_e15.pt`已归档。

## 部署转换

模型PT及两份ONNX位于`models/lane_detection`，转换工具位于`conversion`，160张校准输入位于`conversion/calibration_v1`。RKNN-Toolkit2 2.3.0在Linux x86_64构建rk3568模型；RK输入为RGB uint8 NHWC，归一化由模型配置完成。

阶段关系见`records/MODEL_LINEAGE.json`，历史指标见`records/final_run`。外部道路与内部审核集的收益不同，历史训练指标不作为中国道路整体泛化或板端实时性保证。
