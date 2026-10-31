# RK3568

先看模块上一级 `Documentation/QUICK_START_zh.md`。主入口 `START_INFERENCE.sh`，纯预览对照 `START_PREVIEW_ONLY.sh`；正常Ctrl+C退出。

`Camera_Input`为摄像头／PCIe／USB／校色，`Inference_Four_Views`为双模型和视频HTTP流水线，`Models`为精确INT8权重，`Configuration`为已验收持久颜色，`System_Driver`为内核驱动，`Python_Environment`为离线环境复制工具。日志运行后写入本目录`Results`，存放于`/userdata`。

本模块对四路画面检测，只有前视分割，class10固定0.40；不运行Decision规则或AEB。不要将Decision的位流、颜色文件或0.50阈值策略覆盖到这里。
