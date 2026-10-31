# 导航与仿真代码职责

|位置|职责|
|---|---|
|`software/pc/launch_console.py`|总控台模式入口|
|`software/pc/oib/ui`|视频、导航和决策界面|
|`software/pc/carla`|路线、自车、四相机、行人横穿和会话状态|
|`software/pc/communication`|串口事件、RK报告和视频接收|
|`software/rk3568/fusion/run_carla_aeb.py`|RK感知与AEB关联|
|`software/rk3568/communication`|控制消息、状态与时效检查|
|`software/rk3568/video_input/encoding`|媒体解码和输出|

`configs/layouts/rk3568_carla_navigation.json`组织所需文件，`configs/pc.json`配置PC路径和网络。用户入口位于`scripts`，无需分别从内部模块启动多个相机会话。


[文档导航](../../README.md) · [系统使用](../../USER_GUIDE_zh.md)
