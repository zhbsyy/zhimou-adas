# 部署模型与已归档训练权重

|文件|用途|
|---|---|
|`object_detection/A_rk3568_int8.rknn`|原A 11类目标检测，class10=crosswalk；顺序见同目录 `classes.txt`|
|`lane_detection/R15_256x448_patched_native_int8.rknn`|R15车道线，256×448、RGB uint8 NHWC输入|
|`object_detection/reference/A_original.pt`|按已有来源记录归档的原A正式训练best，保留现有文件名|
|`lane_detection/R15_twinlitenet_small_direct2_lane3_research.pt`|R15历史训练权重|
|`lane_detection/onnx/`|R15已归档256×448 ONNX|

四种运行方式共用两份部署RKNN；不同class10阈值来自各模式配置，不是更换模型。阈值、输出语义与模型文件本次均未改动。

训练代码、配置、权重来源与外部数据要求见 [training](../training/README.md)。原始数据集未随包提供，原A的原始Ubuntu转换记录仍未归档；不能仅凭已有PT和RKNN承诺完全相同的重编译结果。
