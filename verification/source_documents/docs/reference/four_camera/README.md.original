# Cascaded Four Camera — 当前交付版

更新日期：2026-10-30。新内容直接覆盖本目录原文件，不另建运行版本；与 `Decision`、`CARLA_Navigation` 独立。板端代码、模型、校色配置、驱动、位流及新版PC总控台均有自己的副本，不使用压缩包作为交付。

## 当前固定行为

- **class10/crosswalk＝0.40**，其他检测类别＝0.25；过滤、视频底部文字、网页与HTTP结果使用同一常量。
- 前视使用A INT8检测＋R15 INT8分割；后视和两路USB只检测。检测模型对完整四宫格推理一次，R15仅处理Front；R15线条阈值仍为0.60。
- **颜色保持revision2原配置**，无需启动命令附RGB参数。两套双目模组必须对应原校色通道：FRONT接黑板本地输入，REAR接红板经SFP送回。接反曾造成前绿后粉，接回后用户确认正常；这次没有重新校色。
- **PC总控台为navigation_reliability.v1.5的本地独立副本**，保留原三模式，默认选择PCIe、读取RK的8083视频；不自动启动CARLA或开启AEB。

## 文件位置

| 目录 | 内容与入口 |
|---|---|
| `Board_RK3568` | 四路采集、固定校色、双模型推理、绘图、硬件JPEG；运行 `START_INFERENCE.sh` |
| `Board_RK3568/Preview_Only` | 不加载模型的四路预览对照；运行 `START_PREVIEW_ONLY.sh` |
| `PC_Windows` | 新版总控台和视频接入；双击 `START_CONSOLE.cmd` |
| `FPGA_Black_PG2L50H_RX` | 黑色复合板全幅视野接收/PCIe工程与位流 |
| `FPGA_Red_PGL50H_TX` | 红色PGL50H双目/SFP发送工程与位流 |
| `Documentation` | 连接、上板、使用、环境与维护说明 |
| `Verification` | 当前冻结信息、软件检查和分日期的实测证据 |
| `MANIFEST.sha256` | 静态交付文件清单；不包含运行日志、缓存和现场复制的Linux venv |

首先阅读 [快速启动](Documentation/QUICK_START_zh.md)。板端工作目录为 `/userdata/Cascaded_Four_Camera/Board_RK3568`，处理视频为 `http://192.168.2.3:8083/stream`。框和分割由RK绘制，PC解码显示，不重复推理。

正常新部署直接使用当前代码，不需要执行升级补丁。已部署旧0.60版本的用户才使用 `APPLY_CLASS10_040.py`；[更新方法](Documentation/CLASS10_040_UPDATE_zh.md)只替换两个相关文件并同步原清单，不覆盖颜色或模型。`CALIBRATE_COLORS.py`是以后确有校色需要时使用的维护工具，本轮接线恢复后不需要运行。

现有板端独立Python环境已经复制成功，迁移入口、PCIe和新版总控台四格显示均有记录。当前0.40代码的9项流水线回归及3项路径检查通过；运行端是否已加载新阈值，以 `/stat` 的 `class10_min_confidence=0.4` 为准，不能用历史0.60记录代替。最新同步结果见 `Verification/RELEASE_SYNC_20261030.json`。

模型、颜色与位流保持已验收内容。Python、RK系统驱动/GStreamer、Windows环境和PDS是已安装环境前提；详细版本与回退见Documentation。不要同时打开多份总控台或并行运行多份RKNN采集程序。
