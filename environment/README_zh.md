# 运行环境

Windows沿用Python3.12.4与已安装包、CARLA0.9.16引擎和PDS2022.2-SP6.4。快照在snapshots，requirements_pc_snapshot.txt供记录，不要求当前电脑联网重装。新电脑需安装相同依赖，CARLA Python包与引擎版本匹配。修改configs/pc.json的安装路径，双击CMD的引导Python默认在scripts/windows/RUN_PC.cmd；也可设置FPGA_ADAS_PYTHON环境变量。没有把CARLA整套引擎、GPU驱动或SDK安装器塞进源码。

RK沿用aarch64 Linux、系统Python3.11、RKNN Runtime2.3.0/驱动0.9.8、OpenCV4.10.0/NumPy1.26.4和已有MPP/RGA/GStreamer。PCIe预编译驱动匹配内核6.1.99。首次仅复制一次离线venv到environment/rk3568/venv，不从Windows拷Python包；系统原生库与NPU驱动仍属于已安装系统，不能删除。

CARLA输入首次编译私有MPP/RGA解码器，需要g++、pkg-config、RGA和GStreamer开发文件；复制后输出在所选.runtime布局的Mpp_Decoder。其他三个模式不需要为了它编译解码器。系统依赖说明以原交付快照为准，本次未联网升级或重新读取板端完整包清单。

当前局域网PC192.168.2.10、RK192.168.2.3，COM12/CH340、115200。configs/pc.json用于启动器与PC显示地址，部分保留的算法/预检模块也固定了这些网络地址；换网段需要一并核查，不能只改一处就认为所有链路已迁移。

## YOLO训练环境

YOLOv8n原A训练环境与上面的PC总控台环境分开，详见 [yolo_requirements.md](yolo_requirements.md)。正式训练使用Python3.12.14、PyTorch2.6.0+cu124及Ultralytics8.4.155；原参考转换vendor为8.3.163，不要覆盖训练环境。训练代码可以随源码提交，完整数据集、环境目录与初始化输入不随包提供。
