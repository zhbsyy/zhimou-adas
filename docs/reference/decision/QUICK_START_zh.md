# 决策演示流程

1. PG2L50H加载单板双目位流。
2. RK执行`scripts/rk3568/INITIALIZE_PCIE.sh decision`与`START_DECISION.sh`。
3. PC启动CARLA和`START_DECISION_CONSOLE.cmd`。
4. 在决策页选择信号灯或线型场景，发出行驶、变道或停止请求，观察感知状态与车辆反馈。

视频服务默认18089，规则摘要默认18088。CARLA在此模式中表现灯光和前轮，车身不移动。

演示输出包括RK图像、道路状态、PC接收状态和车辆实际执行状态。完整步骤见[系统使用](../../USER_GUIDE_zh.md)。


[文档导航](../../README.md) · [系统使用](../../USER_GUIDE_zh.md)
