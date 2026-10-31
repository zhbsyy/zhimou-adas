# 环境版本记录

本目录记录PC软件环境与Python依赖快照，供部署恢复和版本追溯。运行程序的功能入口与安装路径由项目配置确定。

|材料|内容|
|---|---|
|`PC_CURRENT_ENVIRONMENT.json`|PC解释器、CARLA及环境信息|
|`PYTHON_PACKAGES_SNAPSHOT.json`|归档时的Python包版本|
|`requirements_PC_snapshot.txt`|PC依赖版本清单|

RK3568基线为Python3.11、RKNN Runtime2.3.0、NPU驱动0.9.8、OpenCV4.10.0与NumPy1.26.4。系统媒体库和驱动属于设备环境，Python依赖快照不能替代它们。完整环境分工见[软件与工具环境](../README_zh.md)。
