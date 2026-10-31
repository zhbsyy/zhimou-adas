# Decision — 决策功能最终定档

板端定档：2026-10-27；总控台更新：2026-10-30。**RK 决策 v0.1.3 + 原生 MMIO32 连续采集 + 新版总控台 navigation_reliability.v1.5 + 决策面板 stereo_console.v0.1.5**。本目录保存最新一次用户确认未出现条纹、重影的方案；不混入 720p 试验、旧补丁安装器或新的性能优化。

## 文件夹怎么用

| 文件夹 | 内容与用途 | 在哪里使用 |
|---|---|---|
| `Board_RK3568` | 双模型、感知摘要、规则、双目后台测距、采集器、驱动、配置及连续启动入口 | RK3568 Linux |
| `PC_Windows` | 当前总控台、静止 CARLA 车辆反馈、通信和电脑启动入口 | Windows PC |
| `FPGA_PG2L50H/PROGRAM_THIS_BITSTREAM` | 目前使用的唯一决策位流 | 黑色复合板 PG2L50H |
| `FPGA_PG2L50H/Build_Project` | 对应 RTL、引脚约束、IP 和 PDS 工程，供维护重建 | Windows + PDS |
| `Documentation` | 上板、使用、系统依赖、接口、验收和冻结行为说明 | 阅读 |
| `Verification` | 完整性校验、来源、模型/版本哈希和验证记录 | PC 或板端 |

**先读 [`Documentation/QUICK_START_zh.md`](Documentation/QUICK_START_zh.md)。**

正式入口只有：

- 板端：`Board_RK3568/START_DECISION.sh`，一直运行到 Ctrl+C。
- 电脑启动 CARLA：`PC_Windows/START_CARLA.cmd`。
- 电脑启动总控台：`PC_Windows/START_CONSOLE.cmd`。

不要直接运行 `Console_Application/main.py`：它是总控台基础程序，正式入口还会装入当前决策面板、关闭串口自动选择并设置 RK 地址。

## 和当前效果一致的边界

当前运行的模型、RK 推理/规则代码、PC 决策面板/车辆代码按字节复制；原生采集器和 ARM64 `.so` 与成功试验的哈希一致。只增加可迁移的启动/检查/记录入口，把模型、配置、前处理依赖、启动自检录像放到本目录内部。颜色配置用已确认的同一组 RGB 增益 `[0.916667, 1.137931, 0.970588]`，由历史板端校准输出和成功运行日志复原，来源记录在 `Verification/PROVENANCE.json`。

帧率统计、CPU 测距开销及现有阈值均保持本轮版本，未借本次归档修改算法。2026-10-30已从提交目录启动板端，并在新版总控台看到RK视频与CARLA车辆反馈，停止请求与刹车灯联动通过；红绿灯、实虚线与行人三档的完整素材验收仍按下方文档执行。

## 清理旧目录前必须做的事

项目源码、模型、位流、前后处理与 PC 基础程序已收入本目录。**板端 Python 软件包实际安装在旧 `.venv-offline` 内，因此首次上板须执行独立备份脚本，确认 `ENVIRONMENT_READY.json`，并运行新入口。** 在完成前不要删除旧 Python 环境。之后如还要清理整台电脑/板端，保留系统依赖；本目录不包含 CARLA 引擎和 Linux 系统镜像。

不自动删除任何原项目、修改当前运行程序、重刷位流或改变系统调频。建议把整个 `Final submission` 另存一份外部备份。

本次覆盖的是PC总控台、启动入口、说明及验证记录；Board_RK3568代码/模型/颜色配置、class10=0.50和决策位流保持原定档。当前运行窗口无需重启，后续统一使用本目录入口。
