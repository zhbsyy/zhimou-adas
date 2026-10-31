# 座舱导航与CARLA交互

该模式连接Android导航、PC控制会话和RK视觉感知。座舱提供目的地与导航事件，PC负责路线与车辆执行，RK检测参与行人关联和辅助制动判断。

系统支持座舱联调、感知单机和全域智驾三种界面状态，并提供独立AEB开关。AEB关闭时展示导航；开启时以有效RK视觉关联为前提，在路线中段的横穿场景中执行制动、通过确认和续行，最终到达驻车。

PC入口为`START_CARLA_NAVIGATION_CONSOLE.cmd`；RK入口为`START_CARLA_INFERENCE.sh`。四相机由总控台的CARLA连接会话建立，原始画面发往RK，处理后视频与感知消息回到PC。


[文档导航](../../README.md) · [系统使用](../../USER_GUIDE_zh.md)
