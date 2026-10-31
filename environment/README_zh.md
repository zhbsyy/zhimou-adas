# 软件与工具环境

|用途|环境基线|
|---|---|
|PC总控台|Windows、Python3.12.4、CARLA0.9.16及相匹配的Python API|
|FPGA工程|PDS2022.2-SP6.4|
|RK3568运行|aarch64 Linux、Python3.11、RKNN Runtime2.3.0、NPU驱动0.9.8|
|RK图像处理|OpenCV4.10.0、NumPy1.26.4、MPP/RGA/GStreamer|
|PCIe驱动|归档二进制对应Linux内核6.1.99|
|YOLO正式训练|Python3.12.14、PyTorch2.6.0+cu124、Ultralytics8.4.155|
|RKNN模型构建|Linux x86_64、RKNN-Toolkit2 2.3.0|

PC路径与网络参数在`configs/pc.json`设置，入口解释器也可由`FPGA_ADAS_PYTHON`指定。RK的媒体解码器在目标板编译，需要g++、pkg-config、GStreamer开发文件及RGA头文件和库。

项目代码不包含操作系统、CARLA引擎或已安装Python环境。`environment/snapshots`记录运行环境版本，依赖快照用于恢复对应环境。YOLO训练与参考转换vendor分别使用独立环境。

详细训练依赖见[YOLO环境](yolo_requirements.md)与[R15训练](../training/lane_detection/README_zh.md)。
