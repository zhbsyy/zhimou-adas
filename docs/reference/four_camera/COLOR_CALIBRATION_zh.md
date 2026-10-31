# 相机颜色与角色绑定

前后双目及左右USB采用独立角色绑定，颜色参数与相机顺序、通道约定和位流图像布局配套。

|画面|处理|
|---|---|
|Front|RGB增益0.928571、1.114286、0.975|
|Rear|红蓝交换后应用RGB增益1.0625、0.944444、1.0|
|Right USB|绑定xhci-hcd.1.auto对应video-index0|
|Left USB|绑定xhci-hcd.0.auto对应video-index0|

参数存放于`configs/rk3568/camera_profiles.json`及颜色配置文件。更换相机、端口或位流输出顺序时应重新确认绑定。颜色补偿用于画面一致性，不替代几何标定。


[文档导航](../../README.md) · [系统使用](../../USER_GUIDE_zh.md)
