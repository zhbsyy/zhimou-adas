# RK四路视频运行接口

四路模式合成前、后、左、右画面，统一目标检测并对Front执行R15分割。默认视频端口8083，class10阈值0.40；不执行Decision规则或导航控制。

运行入口为`scripts/rk3568/START_FOUR_CAMERA.sh`，内部布局由`configs/layouts/rk3568_four_camera.json`组织。相机颜色与角色说明见[四路视频设计](../../four_camera/README.md)。


[文档导航](../../../README.md) · [系统使用](../../../USER_GUIDE_zh.md)
