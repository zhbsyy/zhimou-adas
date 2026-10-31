# 环境、校验和常见问题

## 环境

板端：aarch64 RK3568、当前Linux内核6.1.99、RKNN Runtime2.3.0与驱动0.9.8、NumPy/OpenCV/RKNNLite离线venv、`/usr/lib/librknnrt.so`、GStreamer `gst-launch-1.0`／`rawvideoparse`／`mppjpegenc`、`v4l2-ctl`、`lspci`／`setpci`、sudo权限、原PCIe DMA内存驱动。

`System_Driver/Driver_Source`包含驱动源码与Makefile；`Prebuilt_Kernel_6.1.99`为现用ko。不同内核不能直接当作同一个驱动环境使用。

PC：当前Windows Python3.12环境，具体包版本见 `PC_Windows/PYTHON_PACKAGES_SNAPSHOT.json` 和 `requirements_current_environment.txt`。这些是依赖记录，不会自动联网安装。物理四路显示不需要启动CARLA引擎；完整总控台保留的CARLA功能需要已安装CARLA与相应Python包。其agents源码已独立收录到 `Runtime_Support/CARLA_PythonAPI`。

PDS：2022.2-SP6.4，黑板Logos2 PG2L50H、红板Logos PGL50H。工程使用厂商IP／器件库，需本机安装工具；复制源码不包含编译器本体。

## 文件校验

电脑端，在模块目录运行：

```powershell
& 'D:/python3.12.4/python.exe' -B './Verification/verify_submission.py'
```

板端上传整个模块后，可先用原可用离线环境只读核对：

```bash
cd /userdata/Cascaded_Four_Camera &&
/home/linaro/yolov8_deploy/.venv-offline/bin/python -B Verification/verify_submission.py --section board
```

运行程序时也会自动检查板端文件清单。清单不覆盖新生成的`Results`、环境副本和`ENVIRONMENT_READY.json`。定档配置也受清单保护；如果在预览里重新校色，文件哈希将变化，需作为新的配置版本保存和验收，不能悄悄覆盖这次定档。

## PCIe错误／画面黑

`PCIE_NOT_READY`或BAR1=0，表示PCIe资源未就绪，不表示模型损坏。先正常停止采集，再在本模块Board_RK3568目录执行`sudo bash ./INITIALIZE_PCIE.sh`，确认Mem+、BusMaster+和非零Region1后启动。不要在采集仍运行时重新枚举或重载驱动。

前视条纹且后视正常，先检查此前已出现过的相机排线／插头问题；确认黑板实际下载本模块的全幅视野位流。下载成功只证明配置完成，不能替代光学画面检查。先用`START_PREVIEW_ONLY.sh`对照，不随意改DMA延时、通道顺序或反复叠加颜色增益。

## USB找不到／左右颠倒

运行`ls -l /dev/v4l/by-path/*video-index0`。默认RIGHT匹配xhci-hcd.1.auto，LEFT匹配xhci-hcd.0.auto。不要依赖/dev/video0、video2编号永久不变；移过插口后遮挡镜头确认方向。左路部分曝光控件属于必须配置，失败会报告错误，不静默套到另一只相机。

## 总控台无图

先看浏览器`http://192.168.2.3:8083/`。浏览器也无图时先检查板端程序、IP和`/stat`；浏览器有图而总控台无图时，确认用本模块`START_CONSOLE.cmd`打开，检查settings.json并点击重连视频。启动器不会结束别的总控台；有旧窗口时应正常关闭后再开复制版。

总控台“智驾域离线”来自原OIB决策通信，不是这条JPEG视频的在线指标；本模块没有新增决策上报。不要把“PCIe”源标签或CARLA本地兜底视频当成已连到当前板端的证明，检查显示的在线URL和板端`/stat`。

## 性能与留影

`/stat`使用真实完成计数／五秒窗口时间，不以NPU前向耗时倒数当整体FPS。输入是最新帧流水线，过载会跳过旧工作，`*_input_skipped`本身不表示摄像头坏帧。JPEG帧率、模型Hz、主机抓帧率和传感器新帧率是不同量。

画面底图出现留影／灰化时，保留当前运行状态，用`/camera/raw.png?camera=front`和`/snapshot.jpg`对照采集与叠加后画面；单纯调class10阈值不会混合视频底图。本次保留当前实现，不将曾修复Decision读取器的方案未经四路验证替换进来。

## 清理旧目录

本模块启动不导入Decision或旧试验代码，也不从旧模型目录读取RKNN。唯一首次需要原目录的是板端venv备份源；备份成功且迁移短测通过前保留原环境。系统库、CARLA/PDS等已安装软件不属于可删试验目录。本次不执行删除操作。
