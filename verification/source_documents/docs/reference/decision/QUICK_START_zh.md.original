# 使用方法与最小验收

## 一、电脑端文件

保留整个 `Final submission` 目录，移动到其他位置也可以。电脑端项目依赖使用目录内的文件；已安装的 Python/CARLA 路径位于 `PC_Windows/settings.json`，当前值为：

```text
python_executable = D:/python3.12.4/python.exe
carla_root        = D:/CARLA_download
rk_ip             = 192.168.2.3
rk_decision_port  = 18088
rk_video_port     = 18089
```

先关闭其他模块的总控台，避免控制端口8888和诊断端口18090冲突，再双击 `PC_Windows/START_CARLA.cmd`。等待 CARLA 地图加载完，再双击 `PC_Windows/START_CONSOLE.cmd`。启动脚本检测到已有对应进程时不会再启动第二份；若需重启，先正常关闭旧窗口。当前面板自动进入静止车辆决策模式，车身位置锁定，右侧两路 CARLA 摄像机展示灯光和前轮反馈。

总控台不需要通过树莓派 COM12 参与这次静止决策演示。本目录使用新版总控台的独立副本，保留原三个模式，默认进入Decision；不要在本次测试时另开其他写同一演示车辆动作的程序。

## 二、复制到板端

正常停止旧 RK 推理终端。通过 MobaXterm/SFTP 将完整 **Decision 文件夹**上传到：

```text
/userdata/Final submission/Decision/
```

板端需要 `Board_RK3568` 和同级 `Verification`/`MANIFEST.sha256`；上传整个 Decision 最省事。目录名有空格，命令须保留引号。不要上传到小容量根分区 `/home/linaro`，不要再解压历史补丁。

若 SFTP 没有权限，在板端执行一次：

```bash
mountpoint -q /userdata && sudo install -d -o linaro -g linaro -m 755 "/userdata/Final submission"
```

然后刷新 SFTP 再上传。该命令不会更改 `/userdata` 其他目录的所有权。

## 三、首次独立备份 Python 环境

不要先删除旧 `/home/linaro/yolov8_deploy/.venv-offline`。在板端执行：

```bash
cd "/userdata/Final submission/Decision/Board_RK3568" &&
bash ./PREPARE_PYTHON_ENVIRONMENT.sh
```

此操作不联网安装、不修改旧环境，将现有环境复制到本目录的 `Python_Environment/venv`，解除其中的符号链接依赖，并用隔离模式检查 NumPy、OpenCV、RKNNLite、系统 RKNN Runtime。复制可能需要几分钟及额外磁盘空间；空间不足会停止。

看到 `ENVIRONMENT_READY .../Python_Environment/ENVIRONMENT_READY.json` 后，后续启动只使用这里的 Python。**只有在此步骤成功、新入口上板确认且有备份后，才可删除旧工程目录中的源代码和旧 venv。** 板端新 venv 本身也要随最终资料备份；当前 Windows 交付的是其复制/检查脚本，尚不是从板端导出的完整 Linux 环境。

## 四、确认位流与初始化 PCIe

如果板上已经是当前双目位流且画面正确，不必重新下载。需要恢复时，只给黑色复合板 PG2L50H 在线下载：

```text
FPGA_PG2L50H/PROGRAM_THIS_BITSTREAM/local_stereo_pg2l50h_20261025.sbit
```

不要给红色 PGL50H 下载这份位流。本次决策采集直接使用复合板的两目，不依赖红板 SFP 输入。

在确认所有旧采集已停止后执行：

```bash
cd "/userdata/Final submission/Decision/Board_RK3568" &&
sudo bash ./INITIALIZE_PCIE.sh
```

它使用提交目录中的 Linux 6.1.99 驱动重枚举/加载驱动并检查 BAR1。须看到 `Mem+ BusMaster+`、非零 Region 1 和 `PCIE_PREFLIGHT_OK`。如果内核不匹配会停止；不要强制装载错误内核的 `.ko`，也不要跳过 BAR1 检查。

## 五、持续运行决策

```bash
cd "/userdata/Final submission/Decision/Board_RK3568" &&
bash ./START_DECISION.sh
```

一直运行到 Ctrl+C。启动前仍执行原有的两模型输出一致性及 R15 掩膜自检，读取的是目录内原始录像；**这段录像仅用于启动自检，实时输入仍是 PCIe 双目摄像头。** 双目后台处理，左目进入模型和显示。

保持摄像头对准正在播放的赛事视频，并尽量让道路视频充满视野。电脑总控台读取：

```text
视频： http://192.168.2.3:18089/stream
预览： http://192.168.2.3:18089/
决策： http://192.168.2.3:18088/snapshot
```

`18088` 独立网页也可查看规则与提交按钮请求，但正式演示使用总控台的车辆灯光/车轮反馈。

## 六、输出与故障处理

- RK 日志：`Board_RK3568/Results/run_日期时间_PID/live.log`，绝对位置在 `/userdata` 下。
- RK 启动记录：同目录 `invocation.json`，包含模型、配置、采集路径及环境变量。
- PC 日志：`PC_Windows/Results/日期时间.log`。
- PC 状态与事件：`PC_Windows/Results/Decision_Events`。
- `PCIE_NOT_READY/BAR1=0`：先停止采集，执行本目录 `INITIALIZE_PCIE.sh`，再启动；不是再安装补丁。
- PC 没有画面：先在浏览器打开 18089，检查 IP、防火墙和 RK 是否仍运行。
- CARLA 恢复后无车辆反馈：先关闭旧决策窗口，再启动正式总控台；不要保留重复的静止演示进程。
- 灯色 `unknown`：无交通灯检测框仍允许 unknown；有合格框才进行红/绿像素统计，参阅冻结说明。
- 首次环境复制失败：保留旧环境，查看 `Python_Environment/environment_check.log`；不要删除已复制内容后盲目重试。

## 七、最小验收（一次即可）

1. 确认当前模型/采集入口正常，动态道路视频没有之前明显的条纹、新旧画面混合或残留。
2. 看到有效红灯时，正常直行被阻止；红灯下请求行驶自动出现报警。无当前红灯/紧急制动等阻停原因时，默认模拟行驶；手动停止仍有效。
3. 左右边界长度足够时，分别展示实线拒绝变道、虚线允许变道；线太短继续 unknown，不凭空补判。
4. 屏幕行人演示依次检查绿色警告、黄色减速、红色紧急制动；主状态栏不新增反复弹出的独立 AEB 栏。
5. CARLA 车身始终不移动；允许左右请求时前轮和单侧转向灯响应；警告/减速/制动时刹车灯亮，正常不自动双闪。

记录一次五秒统计即可；本次定档没有宣称达到双模型各 8Hz、视频 20FPS 的稳定性能目标。

## 2026-10-30总控台更新

使用navigation_reliability.v1.5主界面、视频最新帧线程和GUI状态邮箱，默认进入stereo_console.v0.1.5决策面板。RK视频18089、规则18088；返回主界面后的PCIe四路仍是8083。静止决策时不连接顶部导航区的四相机，不运行其他车辆控制程序。红板不参与。
