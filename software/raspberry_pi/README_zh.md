# 树莓派Android座舱与影像联动

可维护的实际工程是 `cockpit_android`，其中完整保存 `app/src/main/java/cn/embedded/amapgateway` 的导航、UART、报文和挡位联动源码。它们属于同一Android工程，按原工程编译，不拆成多个不可编译的子工程。

| 功能 | 实际位置/状态 |
|---|---|
| 串口与导航报文 | `cockpit_android/.../GatewayService.java`、`NavFrame.java`、`TcpBridgeClient.java` |
| R挡拉起倒车应用、离开R挡退出 | `cockpit_android/.../GearLinkage.java` |
| 独立倒车影像应用 | 调用外部包名 `cn.embedded.dualcamera`；其源码和APK按提交范围不收入本仓库 |
| 本项目物理四路环视 | `software/rk3568/video_input`、`fusion/quad_infer.py` 和PC总控台PCIe源；不是独立树莓派应用 |

当前串口链路是树莓派Android座舱→CH340→PC COM12，115200/8N1。源码优先 `/dev/ttyAMA0` 并有回退，实际设备节点以安装配置为准。PC重连后只收到巡航待命时，应在安卓端重新选点开始导航。

已检查 `C:\Users\lenovo\Desktop\ADAS\ADAS_Fusion` 和 `C:\Users\lenovo\Desktop\ADAS_Fusion`。外部网关的旧变体没有覆盖本目录冻结源码。本仓库提交座舱导航、通信及挡位联动源码；独立倒车影像App不在本次提交范围，也不作为待补项。要使用原挡位拉起功能，设备需自行提供该外部App。四路环视的RK/PC实现不受此范围说明影响。
