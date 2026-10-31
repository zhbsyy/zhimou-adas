# CARLA_Navigation：座舱导航、双模型视频与自动行人 AEB

本模块定档 **navigation_reliability.v1.5**，与旁边的 `Decision`、`Cascaded_Four_Camera` 独立。当前 PC/RK 业务代码、两个 INT8 模型、MPP/RGA 解码源码及预检视频均在本目录，不引用历史候选目录。正常使用无需压缩包。

**首先阅读 [快速启动](Documentation/QUICK_START_zh.md)。** PC 双击 `PC_Windows/START_CARLA.cmd`，再双击 `PC_Windows/START_CONSOLE.cmd`；RK 当前入口为 `/userdata/carla_navigation_20261030/START_CARLA_AEB.sh`。

保留原「座舱联调」「感知单机」「全域智驾」三模式和独立 AEB 开关：

- 纯座舱导航：AEB 关闭，RK 可接可不接；车辆依座舱目的地和有效导航报文行驶。
- 座舱导航＋AEB：全域智驾、AEB 开启；路线中段自动布置横穿行人，RK 检出并关联行人后制动，确认通过后恢复原导航，到达后 P 挡驻车。

检测模型处理四路画面，**class10/crosswalk 固定 0.30**，其余类别 0.25；行人 AEB 关联阈值 0.40。TwinLiteNet+ Small 仅处理 Front，输入256×448。权重未修改。本次整理没有改动控制、画质、模型或推理参数，也没有重启正在运行的演示。

| 文件夹 | 用途 |
|---|---|
| `PC_Windows` | 总控台、CARLA 后端、四相机推流、导航与 AEB 状态机 |
| `Board_RK3568` | 上板文件：模型、推理/后处理、通信、硬件解码与 JPEG 编码 |
| `Cockpit_Android_RaspberryPi` | 树莓派座舱串口说明与维护源码副本 |
| `Documentation` | 启动、文件索引、数据流、规则参数和维护方法 |
| `Environment` | 当前电脑版本快照及板端环境要求 |
| `Maintenance` | 旧板端 class10=0.60 升级到0.30的可选工具；新部署不用 |
| `tests` | 回归与集成测试；正式演示时不运行会接管 CARLA 的集成脚本 |
| `Verification` | 校验脚本、验证结果和真实 COM12＋RK 全流程关键证据 |

已记录真实 COM12＋RK：路线约50%处横穿、制动停稳、自动续行、到达、未记录碰撞，见 `Verification/PHYSICAL_COM12_RK_ROUTE.json`。规则回归与目录独立性验证见 `Verification/VALIDATION.json`。成功记录不等同于所有路线都无条件通过。

当前距离来自 **CARLA_actor_truth 仿真真值**，由 RK 的真实视觉检出作为制动前提，不是双目实测。本模块使用网络视频，不读取 FPGA PCIe，不需要切换位流。Python/CARLA/板端驱动等已安装环境需保留，详见 `Environment/README_zh.md`。树莓派维护源码已备份，但未核对已安装 APK 哈希。

交付校验：在 Windows 运行 `D:python3.12.4python.exe -B Verificationerify_submission.py`。`PC_Windows/Results` 是持续生成的日志目录，缓存、日志、编译生成物不计入静态清单；关键验收证据已另存并纳入清单。不再需要项目外的 v1.4 备份作为本版本运行条件。
