# FPGA_PG2L50H — 当前决策双目位流

普通恢复仅使用 `PROGRAM_THIS_BITSTREAM/local_stereo_pg2l50h_20261025.sbit`，目标为**黑色复合板 PG2L50H-6 FBG484**。不要将它下载到红色PGL50H。

SHA256：`645225cd9f0d28d5b36ef7e63ac960e96a7102a2e9ac7bf6052bb021988d2935`。

双目都采集；PCIe载荷上左为CMOS1、上右为CMOS2，下半幅重复上面的两目。软件只显示/推理左目，后台可做两目匹配。没有SFP接收依赖；两目曝光同步未作保证。

`Build_Project` 收录RTL、IP、约束和PDS工程，PDS2022.2-SP6.4可打开 `project.pds`。`BUILD_BITSTREAM.ps1` 仅在明确维护位流时使用；重新生成的产物不会自动覆盖定档位流。项目继承的跨时钟时序告警保存在 `Timing_Record`，不要将位流生成成功等同完整时序签核。

在线下载成功/系统重启后，在RK端停止原采集，再执行提交目录的 `INITIALIZE_PCIE.sh`。BAR1恢复后启动推理。
