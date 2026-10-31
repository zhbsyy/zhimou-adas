# 处理顺序、模型和环境

每个选中的源帧依次经过：OpenCV读文件 → 可选等比例缩小 → 检测模型640×640 letterbox/RGB uint8 NHWC → YOLOv8解码/NMS/还原坐标 → class10阈值过滤 → 同一张未绘制的图送R15 256×448 letterbox/RGB uint8 NHWC → 去补边/还原掩膜 → 绘制可行驶区域与红色实线、黄色虚线 → 绘制目标框和顶部标题 → MP4写文件。

两模型都读取原始图，不把已画框/已着色的结果再次当输入；检测在先不改变同帧分割的输入。离线逐帧顺序处理，无实时输入队列和主动丢旧帧；耗时较长只会延长等待，不会自行把45秒源视频压成19秒。源文件读完、请求片段结束、解码提前失败或磁盘不足都在摘要中区分。文件无音轨，原实现也未保留音轨。

`pcie_post.py`和`r15_front_fast.py`来自实际使用的离线工具依赖副本，按字节保留；`offline_pipeline.py`只导出这些前后处理接口，代替原来会顺带导入PCIe/网页/通信模块的直播profile。离线处理器增加可配置class10过滤、读到EOF、结果摘要和不完整退出码；没有重新训练、转换或修改权重。

两个RKNN SHA256：

```text
A_rk3568_int8.rknn
65111dd43d15d289d6001657832b77bb4e68a1afa277684564517eef8c4d9543
R15_256x448_patched_native_int8.rknn
65a233fef461d89f1413bddc9d0316e24aaa310512caac686d88bab388c8c289
```

类别共11个，class10=crosswalk。检测其他类别最低0.25、class10默认0.60；车道线置信度0.60。R15输出为可行驶区域和背景/实线/虚线类别，不把叠加的红黄颜色用于再次识别。离线不包含灯色决策、行人测距、AEB、颜色校准或车辆控制。

板端环境沿用项目已用的aarch64 Linux、Python3.11、RKNN Runtime2.3.0/驱动0.9.8、OpenCV4.10.0、NumPy1.26.4；这是历史环境记录，不是本次重新读取板端版本。保留现有系统Python、动态库、NPU驱动。业务模块不依赖旧候选工程；首次独立复制venv后，还应备份新环境及ENVIRONMENT_READY.json。Windows目录内只提供环境复制/检查脚本，不假装已经包含全部Linux软件包。

视频解码使用现有OpenCV后端，原始HEVC1920×1080/30FPS录像此前已在板端成功读入。输出MP4使用mp4v编码；不宣称硬件解码/编码或实时速度。本工具的 `processed_per_wall_s` 是离线处理吞吐，不是源帧率、播放器FPS或CARLA控制频率。帧率写30FPS不能证明每秒推理30帧。

本机检查可执行 `D:\python3.12.4\python.exe -B Verification\verify_submission.py`，以及 `-m unittest discover -s Verification\tests -v`。测试用合成视频和替身RKNN验证文件数据流、阈值、帧数和异常报告，不运行真实NPU，不连接CARLA或摄像头。真实模型效果以板端生成文件为准。
