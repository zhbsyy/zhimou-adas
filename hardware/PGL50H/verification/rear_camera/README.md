# 红色 PGL50H：双目摄像头SFP发送端

正常下载 `PROGRAM_THIS_BITSTREAM/sfp_tx_camera_frame_fix_20261014.sbit`，SHA256=`104da43e186af66ab4f44ec6566a757e966a949011587c46ca868272e3d7e6d0`。下载到红色PGL50H，SFP0接黑色PG2L50H的SFP1。

PDS2022.2-SP6.4打开 `Build_Project/stereo_sfp_tx.pds`，约束`stereo_sfp_tx_top.fdc`，顶层`stereo_sfp_tx_top`。双OV5640采集、DDR帧缓存、SFP发送RTL/IP和69个PDS引用均已复制；本轮保持此发送设计不变。`Timing_Records`为原构建时序记录。
