# 数据流、推理流水线和通信

Android树莓派座舱 → COM12导航/目的地 → PC导航控制器 → CARLA车辆；CARLA四相机 → PC MJPEG → RK双模型 → RK结果视频回PC。PC把仿真行人投影、前向间距和车辆状态发给RK；RK只有在真实person检测与该行人关联后才可请求AEB。PC负责执行制动、确认通过、恢复原路线和到达驻车。

## 视频与模型

四宫格1280×720，Front/Right/Rear/Left各640×360。RK检测对整个四宫格letterbox到640×640，单路有效区域约320×180；不是每一路单独640输入。R15取Front后resize到256×448。检测与分割各有工作线程，共用RK3568的NPU锁，CPU预/后处理可交叠；采集和显示/JPEG发布独立。q_depth=1以最新帧为主，旧帧主动替换，跳帧不等同于数据错误。模型前向次数与视频发布FPS分别计数。

MPP/RGA用于视频解码/转换；裁剪Front是数组视图/软件取区域，模型resize和后处理仍有CPU工作，不是FPGA硬件裁剪。`native_buffered`复用原生输出缓冲但仍有输入/输出转换与必要复制，不是端到端零复制。结果进入共享状态前已变成框与uint8掩膜；绘制在私有画面副本，避免污染检测输入。总控台解码显示RK已经绘制的JPEG，本模块没有把所有框迁到PC绘制。原始PC视频仅在未接RK等允许场景下显示，有状态区分。

## 网络

| 方向/地址 | 内容 |
|---|---|
| PC 192.168.2.10:2000 | CARLA引擎RPC，使用匹配的Python SDK |
| PC 192.168.2.10:8081/stream、/status | 原始四相机MJPEG、健康和发布计数 |
| RK 192.168.2.3:18084/stream、/stat、/aeb_status | 处理视频、流水线统计、AEB报告 |
| RK → PC UDP8888 | carla_nav_aeb.control.v3报告/心跳 |
| PC → RK UDP8892 | OIB_DOMAIN_STATE，仿真场景与驾驶状态 |
| PC 127.0.0.1:18090/status | 只读诊断：版本、模式、串口、场景、制动/续行、碰撞 |

报告带source、run_id、session_id、epoch和递增det_report_seq。PC不拿心跳冒充新检测；重连/新场景重建会话，旧结果不能跨会话沿用。RK使用本机单调时间计算年龄，PC也检查接收年龄；板端日历时间与PC不一致时，不能直接相减算端到端延迟。

## 座舱串口

115200/8N1。二进制帧36字节：`5A A5`＋小端envelope（version/type/sequence/payload_size，4个uint16）＋22字节payload（`<HHIIIHHBB`）＋CRC16-CCITT-FALSE小端＋`BB BB`。CRC覆盖envelope与payload。接收器也支持换行JSON混传。

keyType10001是导航引导，12205是巡航待命；待命不取消现有路线，也不刷新有效引导的时间戳。真正导航取消、串口断开、有效引导超过10秒未更新才按相应规则停车。JSON提供目的地、挡位、运行意图等；PC回传GEAR_LINKAGE包含真实车速、目标车速、剩余里程、位姿及AEB状态。权威实现保存在 `protocol_5aa5.py`、`navigation_arbiter.py` 和座舱GatewayService源码，修改协议须同步两端。
