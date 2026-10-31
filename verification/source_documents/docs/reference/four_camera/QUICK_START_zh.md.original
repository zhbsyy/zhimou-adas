# 操作顺序

## 1. 连接和位流

红板 PGL50H **SFP0** 通过现有光链路接黑板 PG2L50H **SFP1**。各板保持原双目接线；两只 USB 接 RK3568 原端口。显示顺序：左上前视 FPGA、右上右侧 USB、左下后视 FPGA、右下左侧 USB。

**两套双目摄像头也要保持身份对应**：原已校色的前视相机接黑板本地摄像头输入；原已校色的后视相机接红板，经SFP送回黑板。不要只按朝向把两套相机互换。2026-10-30已实测确认接反会造成前绿后粉，接回原对应关系后恢复；此时不需要重校色。建议给两套模组分别标记FRONT/REAR。

| 下载对象 | 文件（模块内相对路径） |
|---|---|
| 黑色 RK3568+PG2L50H 复合板的 FPGA | `FPGA_Black_PG2L50H_RX/PROGRAM_THIS_BITSTREAM/cascade_fov_pg2l50h_20261029.sbit` |
| 红色 PGL50H 板的 FPGA | `FPGA_Red_PGL50H_TX/PROGRAM_THIS_BITSTREAM/sfp_tx_camera_frame_fix_20261014.sbit` |

用 PDS 2022.2-SP6.4 的 JTAG Boundary Scan 扫描确认器件，再在线 Program。两个文件下载对象不同。当前若已加载这两份位流，无需再次下载。复制 sbit 到 Linux 不等于配置 FPGA；本项目目前记录的是在线加载，不宣称 Flash 上电自启动已完成。

这组级联位流与 `Decision` 的单板双目位流不同。切换模块时先正常停止采集，按模块重新加载相应位流，再初始化 PCIe。两模块共享物理设备，不能同时占用 PCIe。

## 2. 第一次把整理目录放上板

建议上传整个 `Cascaded_Four_Camera` 文件夹到 `/userdata/Cascaded_Four_Camera/`，包括 `MANIFEST.sha256` 和 `Verification`，不要只复制启动脚本。

先在板端创建可上传目录：

```bash
sudo install -d -o linaro -g linaro /userdata/Cascaded_Four_Camera
```

上传完成后，只需首次执行一次离线 Python 环境备份：

```bash
cd /userdata/Cascaded_Four_Camera/Board_RK3568 &&
bash ./PREPARE_PYTHON_ENVIRONMENT.sh
```

脚本把现有 `/home/linaro/yolov8_deploy/.venv-offline` 复制到本模块的 `Python_Environment/venv`，不删除原环境，不联网安装。它检查空间和复制后的导入路径；成功生成 `ENVIRONMENT_READY.json` 后，新入口才会使用该独立副本。如果原环境路径不同，可作为参数传入。空间不足或自检失败时停止，不要删除原环境。

## 3. 正常启动板端

在旧预览或推理终端按 Ctrl+C，确认已退出。随后执行：

```bash
cd /userdata/Cascaded_Four_Camera/Board_RK3568 &&
sudo bash ./INITIALIZE_PCIE.sh &&
bash ./START_INFERENCE.sh
```

初始化应显示 Mem+、BusMaster+、非零 Region 1。初始化脚本使用随包驱动，要求当前内核为 6.1.99；内核不同会拒绝操作。程序持续运行，Ctrl+C 正常停止。

本目录当前已直接包含0.40策略；新部署不用先跑升级补丁。启动自动使用目录内两份 RKNN、类别表和已验收的 revision 2 颜色配置，无需命令行 RGB 增益，也不引用旧试验目录。class10 固定 0.40；不要替换成 Decision 的 0.50 策略文件。

正常校色配置与两套物理相机身份绑定；接线正确且颜色正常时，不运行校色工具、不替换配置。

日志及本次命令记录自动保存到 `Board_RK3568/Results/`，也就是 `/userdata` 分区。

## 4. PC 总控台

电脑端打开 `PC_Windows/settings.json`，当前配置为板子 `192.168.2.3`、视频端口 `8083`、Python `D:/python3.12.4/python.exe`。机器或 IP 改变时只改此配置。

若已有总控台，先正常关闭旧窗口，再双击 `PC_Windows/START_CONSOLE.cmd`。新入口运行本模块内独立复制的新版 navigation_reliability.v1.5 总控台，保留原三模式与新视频/界面更新机制，默认选择 PCIe；默认视频地址为 `http://192.168.2.3:8083/stream`，正常显示 PCIe 四路处理后画面。此入口不自动进入静止决策页面，不要求启动 CARLA。

总控台已打开时，板端重启后可点击“重连视频”。“独立视频”窗口仍保留。总控台源码的源选择按钮发送 OIB 源选择消息；本四路入口固定采集物理摄像头，未添加 CARLA 动态源切换或决策上报监听，源标签变化不能证明视频来源改变。四路显示以 8083 视频地址为准；CARLA／Decision 使用对应的独立模块入口。

## 5. 最小验收

1. 前后视野与当前一致；颜色无新增前粉／后绿，四路均有动态画面。
2. 各格物体可出现检测框，只有前视出现 R15 道路与实虚线叠加。
3. 访问 `http://192.168.2.3:8083/stat`：模型路径应已指向本模块 `Models`，`class10_min_confidence=0.4`、颜色 revision=2、错误不增长。
4. `/results` 中 class10 框分数均 ≥0.40。场景没有斑马线时没有 class10 框是正常现象。
5. 预热后看两条 `[四路双模型 5s]`，记录实际 det、lane、jpeg 数值。当前采样不是三个指标全部达标的承诺，详见冻结记录。

不需要为这次目录整理做长时间重复测试。独立环境复制和迁移已完成；以后重装时仍应做上述最小验收，再决定清理外部历史目录。

## 6. 保持位流不变，只退回纯预览

Ctrl+C 正常停止推理，再运行：

```bash
cd /userdata/Cascaded_Four_Camera/Board_RK3568 &&
bash ./START_PREVIEW_ONLY.sh
```

仍使用同一校色、视野、USB 映射和 8083 视频地址，不调用模型。纯预览与双模型入口不能同时启动。
