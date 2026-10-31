# FPGA视频采集与传输

## 设计分工

PGL50H作为后向双目SFP发送端；PG2L50H作为本地双目与SFP图像接收、重组和PCIe传输端。单板决策设计只使用PG2L50H本地双目，级联四路设计同时使用两块FPGA。

|模式|PG2L50H位流|PGL50H位流|
|---|---|---|
|单板双目决策|`PG2L50H/bitstreams/local_stereo_pg2l50h_20261025.sbit`|不使用|
|双板级联四路|`PG2L50H/bitstreams/cascade_fov_pg2l50h_20261029.sbit`|`PGL50H/bitstreams/sfp_tx_camera_frame_fix_20261014.sbit`|

## 工程实现

源码包含摄像头接口、初始化表、DDR帧缓存、SFP收发与PCIe逻辑。共用IP位于`common/ip`，各模式的RTL和约束位于对应`source`目录。PDS2022.2-SP6.4工程通过`scripts/windows/PREPARE_FPGA_PROJECTS.cmd`生成可编译布局；常规演示使用对应归档位流。

级联连接采用PGL50H的SFP0与PG2L50H的SFP1。接收端以交替行列抽取重组本地图像，形成供RK处理的图像布局。USB侧向视频由RK软件链路接入。

构建、功能仿真及原时序资料保存在各`verification`目录。位流生成、功能仿真与时序签核分别评价；跨时钟路径仍需结合工程约束复核。当前使用JTAG在线加载，不包含Flash自启动配置。
