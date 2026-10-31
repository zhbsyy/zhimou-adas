# 四路视频演示流程

1. 接收板加载级联位流，发送板加载SFP发送位流。
2. 建立SFP与双目连接，并按配置连接左右USB相机。
3. RK执行`INITIALIZE_PCIE.sh four_camera`和`START_FOUR_CAMERA.sh`。
4. PC运行`START_FOUR_CAMERA_CONSOLE.cmd`，查看四路画面及前视分割。

脚本位于`scripts/rk3568`和`scripts/windows`。默认视频端口8083；该模式不需要CARLA或座舱串口。完整部署步骤见[系统使用](../../USER_GUIDE_zh.md)。


[文档导航](../../README.md) · [系统使用](../../USER_GUIDE_zh.md)
