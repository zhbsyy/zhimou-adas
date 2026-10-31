# R15 模型状态

- 文件：`R15_twinlitenet_small_direct2_lane3_research.pt`
- SHA256：`664b642c8476fc1bdcfb39dec04db05c8173651b398a8cf711a92918338a77c0`
- 来源：`long_dashed_recall_v4/best.pt`，实际训练 35 轮，按训练配置的综合内部验证分数选择第 32 轮。
- 初始化链：R12 正式研究模型 → `long_laneonly_a2d2_v2` 第 25 轮 → `long_target_refine_v3` 第 15 轮 → `long_dashed_recall_v4` 第 32 轮。
- direct 和共享张量与 R12 一致；lane 分支使用新 A2D2 官方数据和原有经筛选数据训练。
- 状态：**研究候选，不替换 R12 正式研究基线，也未获准部署。** 外部/合成验证收益与内部人工审核集虚线召回下降同时存在。
- 评估输入：384×640、原始掩膜、lane 阈值 0.6；细节见上级目录 `R15_training_report.md`。
