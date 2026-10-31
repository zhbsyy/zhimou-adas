# 数据流、职责与文件定位

```text
红板双目 → DDR → SFP0 ═光链路═ 黑板SFP1
黑板本地双目 ────────────────→ DDR四路拼接 → PCIe RGB565 1280×720
  RK读取：前视=左上本地左目；后视=左下远端左目
  RK两路USB：右侧与左侧，以物理端口识别
       ↓ 校色／裁成显示格／四路合成1280×720
       ├→ A INT8：四宫格整体缩放补边640×640 → 检测 → class10≥0.40
       ├→ R15 INT8：仅前视640×360 → 256×448 → 可行驶区域＋实虚线
       └→ 私有绘图副本 → MPP JPEG → HTTP 8083 → PC总控台解码显示
```

这不是四次逐格检测推理：A 每轮处理一幅四宫格，因此每格在模型中的物体像素尺寸会比单镜头推理小。前视处理检测和分割；其他三个显示格只有检测。FPGA 原始双目的另外两只眼并未作为四路显示中的左右 USB 使用，也未用于本模块测距。新全幅几何后不能直接沿用旧双目标定作为距离精度证明。

## 板端代码

| 路径 | 职责 |
|---|---|
| `Board_RK3568/launch_cameras.py` | 本模块相对路径、启动前文件／PCIe／占用检查、权限、日志；不更改推理数学或图像处理 |
| `Camera_Input/cap_reader.py` | 原已验证按行 PCIe MMIO/DMA 抓取，行间延时40μs；本次未换成其他试验读取器 |
| `Camera_Input/pcie_frame.py`、`frame_align.py` | RGB565 解码与既有隔行判定，几何／色彩解释保持原实现 |
| `Camera_Input/camera_ports.py`、`usb_source.py` | USB 物理端口识别、V4L2 参数、最新帧采集、重连 |
| `Camera_Input/color_profiles.py` | 每路固定通道顺序和 LUT 校色；前后各自配置，校正后的同一数据供模型和预览 |
| `Configuration/camera_profiles.json` | 用户验收的持久配置，revision 2；已复制真实有效字段，无需每次传参 |
| `Inference_Four_Views/quad_infer.py` | 采集、合成、A／R15、绘图、编码、HTTP 各工作线程；`/stat`、`/results` 等接口 |
| `Inference_Four_Views/pipeline.py` | 最新一帧容器、图像所有权、四路布局、时效检查 |
| `Inference_Four_Views/detection_policy.py` | class10 固定0.40、其他类0.25；分格裁框和跨接缝框过滤 |
| `Inference_Four_Views/pcie_post.py` | A 模型输入预处理、DFL/NMS 解码、坐标映射 |
| `Inference_Four_Views/r15_front_fast.py`、`postprocess_candidate.py` | R15 letterbox／RGB输入、掩膜解码、稀疏置信度筛选、平面 argmax |
| `Inference_Four_Views/native_api.py`、`runtime_backend.py` | 已验证 RKNN C API 与可复用输出缓冲；非零拷贝，不改模型算子 |
| `Inference_Four_Views/cached_display.py` | 缓存掩膜几何，始终叠加在最新底图上，不缓存旧道路像素 |
| `Inference_Four_Views/hw_jpeg.py`、`jpeg_lifecycle.py` | MPP 常驻 JPEG、回调发布、退出等待线程；模型与显示解耦 |
| `Models/` | 两份精确 SHA256 的 INT8 RKNN 和11类顺序表 |
| `Preview_Only/` | 纯四路预览及其原编码器副本，作为无推理对照，不启动新算法 |
| `System_Driver/` | 驱动源码和适配6.1.99的预编译 ko |
| `Python_Environment/` | 一次性备份现有 Linux venv 的工具；运行后得到本模块独立环境 |

`postprocess_candidate.py` 等名字保留以避免无意义重命名，与旧试验目录无导入关系。来源与逐文件校验见 `Verification/PROVENANCE.json` 和根清单。

## PC 视频接入

`PC_Windows` 已覆盖为新版 navigation_reliability.v1.5 的独立副本，保留原三种模式；`launch_console.py`从自身settings.json读取地址，默认将状态和视频源设为PCIe，并加载本目录配套模块，不依赖相邻CARLA_Navigation或旧Desktop/ADAS目录。

主要位置：

- `video_bridge.py`：按源选择8083物理视频或CARLA地址；单个可重连拉流线程、最新JPEG邮箱，GUI线程解码并显示，避免旧画面排队。
- `navigation_session.py`：新版状态邮箱和GUI定时更新；导航/AEB控制面板随新版保留，但物理四路显示不要求启动这些链路。
- `Console_Application/ui/dashboard.py`：布局、独立视频窗口、原三模式及源选择按钮；主视频方法由新版视频适配器接管。
- `navigation_arbiter.py`及本地navigation/control支持模块：新版三模式仲裁与COM12/CH340选择，入口默认PCIe、AEB关闭、CARLA未连接。
- `Console_Application/bridge`：本地CARLA后端、串口及5AA5协议实现。切换PC视频源不会改变RK固定物理摄像头采集。

**绘框和分割叠加发生在 RK，不在 PC。** PC收到的是压缩后的1280×720四宫格，每格640×360；USB实际协商320×240 MJPG，再居中裁成16:9和缩放显示，不能把放大后的格子称为USB原生高清。模型预处理的缩放补边由RK软件OpenCV执行；黑板本地1280×720→640×360交替列／行抽取由FPGA完成。

## 缓冲与时效

采集、合成、模型、绘图、编码各阶段只保留最新对象，编码待写队列深度1。NPU调用共享锁，前后处理可在CPU重叠；输出数组只由各模型自己的线程使用，完成解码后才复用。合成底图不允许绘图线程原地修改。

来源超过750ms不再作为有效输入；检测和掩膜超过各自源帧500ms不再叠加。显示使用最新画面与最近有效结果，快速物体会存在位置时间差，不能称逐帧同步。`*_input_skipped` 是处理端跳过的旧版本，不等同于物理传感器掉帧。抓帧计数也不能证明光学新帧率。

## FPGA 源码

黑板 `Build_Project/source/user/image_reshape.v` 是本轮视野修复文件，按1280列／720行取交替列和行，保持每目640×360输出和原DDR帧长。传感器寄存器、SFP、PCIe、约束与其他RTL不因本轮视野修复而变化。红板继续使用原已验证发送工程。

两块板的 PDS、约束、RTL、IP及工程保存时列出的任务输出均已复制；黑板226项、红板69项PDS引用核对存在。JTAG下载以各自 `PROGRAM_THIS_BITSTREAM` 为唯一正常入口；`Build_Project` 里的工具生成副本供工程打开与追溯。

`Documentation/Hardware_Manuals`保存两块板的硬件使用手册原件，SHA256和来源已写入PROVENANCE；实际本项目引脚配置以各板当前PDS工程的约束文件为准。
