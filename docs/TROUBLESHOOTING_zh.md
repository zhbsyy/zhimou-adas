# 常见问题

| 现象 | 处理 |
|---|---|
| SFTP向/userdata上传被拒绝 | 先按USER_GUIDE创建属于linaro的项目子目录，刷新SFTP后进入该目录上传，不改整个分区权限 |
| cd: No such file | 检查是否多套一层目录；路径含空格时完整加引号；不要复制终端提示符或代码框反引号 |
| BAR1=0、PCIE_NOT_READY | 正常停推理，确认黑板位流正确；执行INITIALIZE_PCIE.sh decision或four_camera，看Mem+ BusMaster+和非零Region1再启动 |
| 前后颜色突然互反 | 检查两个物理双目是否接反；固定校色绑定设备位置，不要先改增益 |
| USB找不到xhci端口 | 放回已验证物理端口，右xhci.1、左xhci.0，检查/dev/v4l/by-path/*video-index0；不要让左右都插同一个Hub |
| 只有右侧CARLA黑屏 | 先开引擎等地图加载，再在Decision点连接/重连车辆；本次采用后台渲染，不要求独立游戏窗口 |
| CARLA四相机挂载失败 | 用CARLA导航入口，先点顶部连接CARLA与四相机；关闭重复推流/旧总控台，避免重复自车或8081占用 |
| 总控台等RK、没处理框 | 对应RK程序是否运行、IP是否192.168.2.3；检查8083/18084/18089是不是选对了，三者不可混用 |
| 导航只显示巡航待命 | 确认COM12/CH340已连接，在安卓重新选点并开始，重发目的地；串口接通不等于导航已开始 |
| 8888或18090被占用 | 正常关闭原总控台窗口再启动新入口，不同时开不同模块总控台 |
| 环境复制空间不足 | 保留原venv，检查userdata空间；四种方式只复制一次，勿删系统Python/库/驱动 |
| Runtime file was modified | 不覆盖；保留修改和结果，停止程序后用新的FPGA_ADAS_RUNTIME_DIR生成新布局，或核对分类源码后再维护 |
| 离线视频短于原视频 | 先看result.summary.json的complete和stop_reason，low_disk_space/interrupted不是完整结果；/tmp可能与根分区同盘 |
| CARLA报DXGI_ERROR_DEVICE_REMOVED | 这是引擎/显卡错误，正常退出损坏引擎后重新打开；先用Low/后台渲染，恢复后再连接车辆，不把它当RK模型报错 |

查文件/依赖可以运行 `python scripts/project.py rk --profile decision --action check` 或 `python scripts/project.py pc --profile decision --check`。这两种check不启动NPU、相机或车辆。真实网络状态只在对应服务已运行时检查，浏览器打不开未启动的端口不等于代码损坏。

新目录只做结构与启动适配，没有承诺消除所有硬件条纹、长时间重影或达到双模型8Hz/视频20FPS。出现光学/采集异常时先保留原始帧、位流和启动日志，避免用FPS计数代替图像正确性。
