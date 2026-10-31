# 树莓派 Android 座舱：串口与维护源码

当前演示继续使用树莓派上已经验证的 Android 座舱，通过 CH340 USB 串口连接 PC **COM12，115200、8N1**。PC 启动只自动选择 COM12 且名称含 CH340 的设备，不把 FPGA 的 FTDI/JTAG 串口当作座舱。

`Reference_Source/` 是项目中 `Android13_Cockpit_UART` 的本地源码副本，包含应用源码、Town10 地图资源、Manifest、Gradle 配置和离线构建脚本。不依赖原项目目录。树莓派源码优先 `/dev/ttyAMA0`，并有设备节点回退；实际节点和权限仍以现有 Android 配置为准。

本次未读取树莓派上已安装 APK 的哈希，因此**源码副本不能作为已安装 APK 完全同版的证明**。旧目录中的历史模拟器 APK 未作为当前安装包交付，也不需要重新安装座舱。维护时可在 Android Studio 中打开 `Reference_Source`，或检查 SDK/JDK 路径后运行其中 `build_debug_apk.ps1`。该脚本使用 API33、build-tools36.0.0、现有 Android Studio JBR；输出文件名仍带 Emulator，这是历史命名，不能用文件名判断部署对象。构建没有作为本次 PC/RK 打包验证的一部分。

连接后在安卓选择目的地并开始导航；如果 PC 重连后只显示“巡航（待命）”，重新选点并开始导航，重新发送目的地。报文定义以本模块 `PC_Windows/Console_Application/bridge/protocol_5aa5.py` 和 `Documentation/ARCHITECTURE_AND_PROTOCOL_zh.md` 为准。
