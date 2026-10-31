# 源码职责索引

## PC_Windows：只在电脑运行

| 文件/子目录 | 作用 |
|---|---|
| START_CARLA.cmd / START_CONSOLE.cmd / launch_console.py | 引擎、总控台入口；本地导入、COM12选择、单实例与日志 |
| Console_Application/ui/dashboard.py | 总控台界面、原三模式与原有操作入口 |
| Console_Application/bridge/carla_backend.py | CARLA自车、导航路线执行、车辆执行器 |
| Console_Application/bridge/carla_streamer.py / reliable_streamer.py | 四相机挂载、帧健康、发布原始视频 |
| navigation_arbiter.py / navigation_policy.py | 导航消息、驾驶意图、AEB优先级、停车确认与自动续行 |
| navigation_motion.py / pedestrian_crossing.py | 控速、碰撞记录、中段横穿选点和入场时机 |
| navigation_session.py | 总控台整合、20Hz场景采样、最新状态邮箱与10Hz界面刷新 |
| control_guard.py / video_bridge.py | RK会话/时效检查、处理视频选择与重连 |
| Console_Application/bridge/protocol_5aa5.py / serial_client.py | Android串口协议、CRC、分帧和车辆遥测回传 |
| aeb_session.py / safe_arbiter.py | 共享几何投影/紧急制动函数；虽然名称来自旧入口，当前代码仍导入，必须保留 |
| Console_Application/ui/static_vehicle_session.py / rk_decision_*.py | 保留的静止决策界面组件；CARLA导航演示不要求同时启动Decision |
| probe_cameras.py / check_actuator.py | 诊断工具，不是日常启动入口，会操作CARLA，演示时不另开 |
| Results | 运行生成的日志，不是源码依赖；关键记录已复制到Verification |

## Board_RK3568：整个目录上板

| 文件/子目录 | 作用 |
|---|---|
| START_CARLA_AEB.sh / check_start.py | 包/模型校验、并发进程与四相机就绪检查、连续运行 |
| run_carla_aeb.py | 当前CARLA入口：class10=0.30、接入本模块AEB通信 |
| pcie_stream_r15_async.py / pcie_stream_r15_profile.py | 保留文件名的异步视频/检测/分割流水线；正式脚本选择的是网络CARLA源 |
| pcie_post.py / r15_front_fast.py / postprocess_candidate.py | 检测与车道线预处理、解码、后处理 |
| runtime_backend.py / native_api.py | RKNN原生接口、模型SHA校验、复用输出缓冲 |
| cached_display.py / overlay_roi_candidate.py / publication_candidate.py | 显示缓存、掩膜绘制、JPEG发布 |
| hw_jpeg.py / Mpp_Decoder | RK硬件JPEG编码、网络MJPEG的MPP/RGA解码源码与适配器 |
| carla_aeb_link.py / aeb_status.py | RK视觉行人关联、轻量AEB规则、UDP和HTTP状态 |
| Models | 两个INT8 RKNN模型及11类类别顺序 |
| Selfcheck_Video | 原生输出/模型预检素材，必须随包保留 |
| pcie_frame.py / frame_align.py / stereo_range.py / carla_source.py / video_source.py | 共享基座导入所需适配器；本模块不启用PCIe/双目距离，不应按文件名删除 |
| trial_metrics.py / PACKAGE.sha256 | 阶段计数与文件校验 |

Environment保存安装环境信息，Cockpit_Android_RaspberryPi保存座舱维护源码，Maintenance不是普通启动入口。tests中的实际CARLA集成脚本会接管自车/场景，不能与正式演示同时运行；普通 `test_*.py` 为离线回归。
