# 双目SFP发送设计

PGL50H采集双OV5640、经DDR帧缓存后从SFP0发送到PG2L50H的SFP1。对应位流为`sfp_tx_camera_frame_fix_20261014.sbit`，顶层为`stereo_sfp_tx_top`。

工程包含摄像头、DDR、SFP逻辑与PDS约束及原时序资料。发送端与接收端使用各自板卡工程，运行位流按硬件角色选取。

硬件工程索引见[FPGA视频采集与传输](../../../README_zh.md)。
