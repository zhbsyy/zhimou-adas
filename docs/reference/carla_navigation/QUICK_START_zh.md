# 导航仿真演示流程

1. PC启动CARLA引擎和导航总控台，建立自车与四相机。
2. 连接Android座舱串口，选择目的地并发送开始导航事件。
3. 使用RK视觉时启动`PREPARE_CARLA_DECODER.sh`和`START_CARLA_INFERENCE.sh`。
4. AEB关闭时观察路线执行；开启时观察横穿场景、视觉关联、制动、通过确认与到达驻车。

引擎、总控台和RK入口均由`scripts`组织。源视频服务默认8081，RK结果视频默认18084。操作顺序及环境要求见[系统使用](../../USER_GUIDE_zh.md)。


[文档导航](../../README.md) · [系统使用](../../USER_GUIDE_zh.md)
