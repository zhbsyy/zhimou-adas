# RK单板感知运行接口

Decision模式使用PCIe双目输入，左目执行检测和分割，后台两目提供参考匹配。规则摘要与标注视频经网络送往PC。

正式入口为`scripts/rk3568/START_DECISION.sh`，采集初始化由`INITIALIZE_PCIE.sh decision`执行。内部运行布局由`configs/layouts/rk3568_decision.json`生成；代码职责见[RK模块](../../../software/rk3568/README_zh.md)。


[文档导航](../../README.md) · [系统使用](../../USER_GUIDE_zh.md)
