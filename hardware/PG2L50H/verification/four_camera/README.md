# 黑色复合板 PG2L50H：级联接收＋本地全幅视野＋PCIe

正常下载 `PROGRAM_THIS_BITSTREAM/cascade_fov_pg2l50h_20261029.sbit`，SHA256=`d63eee870d652f49cce62ffd62c460c4fdf3d70414320e35e3bec8271ce8c6a7`。只下载到黑色板PG2L50H；SFP1接红色板SFP0。不要与Decision的单板双目位流混用。

PDS2022.2-SP6.4打开 `Build_Project/project.pds`，约束为`project.fdc`，顶层`image_pcie_capture`。RTL、IP、初始化表以及PDS列出的226个文件引用均已复制。`source/user/image_reshape.v`是本轮全幅视野修复：本地1280×720按交替列／行抽取至640×360，不调整光学镜头。

`Timing_Records`保留原时序报告，`Build_Records`记录构建与更改，`Functional_Simulation`保留两帧功能仿真。跨时钟时序仍有已知告警，见模块冻结说明。PDS任务输出中的位流副本只用于工程恢复，正常JTAG操作认准`PROGRAM_THIS_BITSTREAM`。
