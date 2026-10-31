# 感知模型

|文件|任务与接口|
|---|---|
|`object_detection/A_rk3568_int8.rknn`|YOLOv8n 11类检测，640×640输入，九输出头|
|`lane_detection/R15_256x448_patched_native_int8.rknn`|R15 TwinLiteNet+ Small，道路结构分割，256×448输入|
|`object_detection/reference/A_original.pt`|原A正式训练权重|
|`lane_detection/R15_twinlitenet_small_direct2_lane3_research.pt`|R15选中训练权重|
|`lane_detection/onnx`|R15原始与等价改写ONNX|

检测类别依次为person、rider、car、truck、bus、train、motor、bike、traffic light、traffic sign、crosswalk；类别表保存在`object_detection/classes.txt`。斑马线采用物理横道整组框定义。

两份部署RKNN由四种模式共用。阈值由各模式策略设置，不属于模型权重。R15输出direct两类和lane三类；背景、实线、虚线的类别定义与训练标签一致。

模型训练与转换说明见[训练文档](../training/README.md)。原A的原始Ubuntu转换记录未收录，现有RKNN作为部署资产保留，重新编译的模型需另行评价。

斑马线与箭头、相似路面纹理仍可能混淆，检测结果用于辅助观察；模型表现需结合目标域场景评价。
