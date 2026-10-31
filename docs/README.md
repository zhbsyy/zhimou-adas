# 作品技术文档

本项目以FPGA视频链路、RK3568边缘感知和多设备交互为主线组织技术材料。

|阅读目的|文档|
|---|---|
|了解系统分工和代码实现|[系统架构](ARCHITECTURE_zh.md)|
|了解各功能的输入、输出和演示流程|[使用说明](USER_GUIDE_zh.md)|
|部署与启动|[部署方案](DEPLOYMENT_AND_ARCHIVE_NOTES_zh.md)、[命令索引](START_COMMANDS_zh.md)|
|查询模块组成和外部依赖|[工程组成](SUPPLEMENT_20261031_zh.md)、[运行条件](TROUBLESHOOTING_zh.md)|
|了解物理视频、决策和仿真实现|[四路视频](reference/four_camera/README.md)、[决策](reference/decision/README.md)、[导航仿真](reference/carla_navigation/README.md)|
|了解模型来源与训练方法|[训练说明](../training/README.md)|
|查看功能验证依据|[验证资料](../verification/README.md)|

路径均以仓库根目录为基准。代码中的默认地址、端口和设备映射由配置与具体运行模式确定；部署时按目标设备配置，不依赖开发电脑的磁盘目录。
