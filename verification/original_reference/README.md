# Final submission

项目交付总目录，四个功能模块各自保存源码、模型、配置和启动说明，不以压缩包交付。

| 独立模块 | 功能 | 使用说明 |
|---|---|---|
| `Decision` | 双目后台测距、单目显示、交通灯/实虚线/行人决策及静止车辆反馈 | [README](Decision/README.md)、[快速启动](Decision/Documentation/QUICK_START_zh.md) |
| `Cascaded_Four_Camera` | 两板级联、四路物理输入、颜色与视野固化、Front双模型/其他三路检测、总控台PCIe视频源 | [README](Cascaded_Four_Camera/README.md)、[快速启动](Cascaded_Four_Camera/Documentation/QUICK_START_zh.md) |
| `CARLA_Navigation` | 座舱导航报文驱动CARLA、四相机双模型视频、路线中段横穿行人、AEB制动后自动续行到达 | [README](CARLA_Navigation/README.md)、[快速启动](CARLA_Navigation/Documentation/QUICK_START_zh.md) |
| `Offline_Video_Processing` | RK3568逐帧处理本地道路录像，保存双模型绘制结果、日志和摘要，无需CARLA/摄像头 | [README](Offline_Video_Processing/README.md)、[快速启动](Offline_Video_Processing/Documentation/QUICK_START_zh.md) |

CARLA模块定档 navigation_reliability.v1.5，保留座舱联调、感知单机、全域智驾三模式。AEB关闭时RK可选，AEB开启时需要真实RK感知；已有真实COM12＋RK的一次制动/续行/到达证据，详见该模块Verification。2026-10-30已将Decision的PC内容覆盖为新版总控台独立副本，默认进入定档决策面板；板端算法和位流未修改。**CARLA模块class10=0.30；物理四路=0.40；Decision=0.50；离线默认=0.60，不要跨模块覆盖。**

Python、CARLA引擎、板端系统/驱动/GStreamer、Android已安装座舱和FPGA下载工具属于运行环境，需按各模块环境说明保留。业务文件已各自保存，但不能因此删除现有RK离线Python环境。CARLA模块另附树莓派座舱维护源码，未核对板上已安装APK哈希。

Decision与物理四路共用PCIe采集设备且使用不同位流，不能同时采集。CARLA通过网络输入，无需切换FPGA位流；首次上板编译模块私有MPP/RGA解码器。三个实时模块的PC总控台不能同时打开，板端也不要并行运行多份RKNN程序。离线模块不启动PC总控台或PCIe采集，默认完整处理到EOF，结果写入userdata子目录；处理前正常停止其他板端推理。

先核对对应模块MANIFEST、启动入口和最小验收，再整理项目外的历史目录。模块内Results/日志/缓存为运行生成物，关键验收记录已单独纳入各自Verification。
