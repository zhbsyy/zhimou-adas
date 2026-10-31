# Android座舱导航与通信

座舱工程位于`cockpit_android`，通过目的地选择、导航状态与挡位消息连接PC控制会话。

|模块|实现文件|
|---|---|
|导航界面与地图|`CockpitNavActivity.java`、`CockpitMapView.java`、`TownMapData.java`|
|消息与通信|`GatewayService.java`、`NavFrame.java`、`TcpBridgeClient.java`|
|挡位影像联动|`GearLinkage.java`|
|启动与生命周期|`MainActivity.java`、`BootReceiver.java`|

串口配置为115200、8N1，设备节点与PC端口按实际连接配置。目的地和导航事件进入PC的路线控制，挡位接口负责外部影像应用的调用。

独立倒车影像App `cn.embedded.dualcamera` 不在源码提交范围；需要影像联动时由设备提供对应应用。RK/PC实现的物理四路观察属于另一条视频链路。
