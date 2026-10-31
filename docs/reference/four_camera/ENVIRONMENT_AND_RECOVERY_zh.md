# 四路模式部署接口

硬件输入需要前后双目、双板SFP链路和两路USB。RK需匹配的PCIe驱动、NPU Runtime、V4L2、MPP与GStreamer环境；PC接收RK发布的四宫格。

启动前根据`configs/rk3568/camera_profiles.json`确认相机角色，根据[硬件设计](../../../hardware/README_zh.md)加载级联位流。改变硬件顺序时同步检查颜色配置，不用软件颜色增益掩盖通道或角色错误。

输出状态分别报告输入帧、模型处理和视频发布。具体入口见[运行索引](../../START_COMMANDS_zh.md)。


[文档导航](../../README.md) · [系统使用](../../USER_GUIDE_zh.md)
