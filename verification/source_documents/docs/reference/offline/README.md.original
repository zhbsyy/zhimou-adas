# Offline_Video_Processing — 离线视频双模型处理

在RK3568上读取本地单路道路录像，用当前 **A INT8检测模型 + R15 256×448 INT8车道线模型**逐帧处理，保存带目标框、可行驶区域、实虚线的MP4文件。代码、两个模型、11类表、前后处理、原始44.97秒示例录像和启动说明均已收入本模块；不引用其他交付模块的业务文件。

**先读 [快速使用](Documentation/QUICK_START_zh.md)。** 板端唯一入口是 `Board_RK3568/START_OFFLINE.sh`。不需要打开CARLA、总控台、推流器，也不读取FPGA摄像头或控制车辆。正式处理前正常停止其他RK推理程序。

| 文件夹 | 内容及用途 |
|---|---|
| `Board_RK3568/Inference_and_Postprocess` | 实际离线处理器、检测前后处理、R15前后处理及绘制 |
| `Board_RK3568/Models` | 两个已用INT8 RKNN和类别顺序 |
| `Board_RK3568/Configuration` | 默认参数，class10=0.60、其他类别=0.25 |
| `Board_RK3568/Input_Videos` | 原始赛事道路示例视频；另可用 `/userdata/videos/1.mp4` 或 `2.mp4` |
| `Board_RK3568/Python_Environment` | 在RK上独立备份已安装的离线Python环境，首次执行一次 |
| `Board_RK3568/Results` | 运行时自动创建，每次单独保存视频、日志、摘要、命令 |
| `PC_Windows` | SFTP下载/播放说明与结果元数据检查脚本；不在PC上运行RKNN |
| `Documentation` | 使用、处理顺序、阈值、环境和异常处理说明 |
| `Verification` | 来源/模型哈希、完整性脚本及无NPU本地验证 |

默认每帧都推理（stride=1），处理到视频结束（seconds=0）；源宽度超过1280时等比例缩小。使用 `--max_width 0` 保留原视频分辨率，速度和输出空间相应增加。输出时长按源帧率写入，与处理耗时无关。原脚本的静音输出方式保留，不复制音轨。

**本模块沿用最后一次离线测试的class10=0.60；Decision=0.50，四路物理输入=0.40，CARLA导航=0.30，各自独立。** 可通过入口 `--class10_conf` 选其他值，无需再用字符串替换生成临时脚本；其他类别仍是0.25。不修改模型权重。

本次只整理代码、独立依赖并验证数据流/时长/阈值/失败报告，没有在正在运行Decision的板子上并行重跑整段视频。RK运行环境仍需现有Linux、RKNN Runtime/NPU驱动、OpenCV和解码/编码支持；不把Windows Python包拷到RK。
