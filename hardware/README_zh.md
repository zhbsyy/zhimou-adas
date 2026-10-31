# 板卡和位流

| 使用方式 | 黑板PG2L50H | 红板PGL50H |
|---|---|---|
| Decision | PG2L50H/bitstreams/local_stereo_pg2l50h_20261025.sbit | 不使用 |
| 物理四路 | PG2L50H/bitstreams/cascade_fov_pg2l50h_20261029.sbit | PGL50H/bitstreams/sfp_tx_camera_frame_fix_20261014.sbit |
| CARLA导航/离线 | 不读取PCIe，不需为此换位流 | 不使用 |

普通演示只下载上表.sbit。PDS Fabric Configuration选择正确黑/红设备，加载对应文件，Program后确认成功和done bit=1。此项目没有自动烧Flash或操作JTAG；在线下载后重启可能恢复原Flash版本，按实际供电/配置状态核对。

源码和IP分开存储以去重，维护重建先运行scripts/windows/PREPARE_FPGA_PROJECTS.cmd，再打开打印的.runtime/hardware/<方式>/project.pds。不要直接在已分类拆开的source目录里编译一个未还原的工程。PDS输入源码、约束和IP引用由布局还原。工程文件还保留历史编译输出位置；generate_netlist/place_route/synthesize等中间产物未复制，PDS若提示旧结果不存在，应从综合开始重新生成。本次没有重跑PDS编译，常规演示使用已验证的归档位流；原交付目录未删除。
