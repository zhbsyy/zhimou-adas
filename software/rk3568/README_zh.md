# RK3568边缘感知

RK程序完成视频输入、双模型推理、灯色与道路状态分析、图像叠加及通信。

|目录|实现|
|---|---|
|`video_input`|PCIe、USB、网络视频与媒体编码|
|`object_detection`|目标检测前后处理、策略和NPU接口|
|`lane_detection`、`lane_attributes`|R15掩膜、绘制和线型分析|
|`traffic_light`|候选区域颜色分析|
|`fusion`|四种模式的主流水线与状态融合|
|`communication`|PC/OIB/AEB消息、状态服务与驱动|

运行入口统一位于`scripts/rk3568`。布局配置恢复各模块相对导入关系；相机映射、输入尺寸和阈值按模式使用。CPU负责预后处理与规则，检测/分割共享NPU访问锁。技术细节见[系统架构](../../docs/ARCHITECTURE_zh.md)。
