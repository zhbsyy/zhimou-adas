# 四路视频数据流与实现

前后FPGA图像通过SFP与PCIe进入RK，左右USB通过V4L2采集。软件按Front、Rear、Left、Right角色组织画面，形成1280×720四宫格。

检测、前视分割、合成显示与JPEG发布使用独立处理阶段，NPU调用共享锁。画面更新与模型前向分别计数，因此显示频率可以高于某一路物理输入或单模型处理频率。

|代码位置|职责|
|---|---|
|`video_input/four_camera`|模式专用采集与帧处理|
|`video_input/camera_ports.py`|USB设备角色映射|
|`video_input/color_profiles.py`|颜色配置与绑定|
|`object_detection/detection_policy.py`|候选处理与类别阈值|
|`lane_detection/four_camera`|前视分割与缓存显示|
|`fusion/quad_infer.py`|流水线组织与状态发布|

上述路径位于`software/rk3568`。共享NPU和四宫格检测控制计算成本，同时也限制单路小目标的输入分辨率。


[文档导航](../../README.md) · [系统使用](../../USER_GUIDE_zh.md)
