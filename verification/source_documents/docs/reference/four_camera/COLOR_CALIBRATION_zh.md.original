# 校色保存与版本同步

校色命令修改 `Board_RK3568/Configuration/camera_profiles.json` 的RGB增益、参考采样和revision；不改Python推理源码、不改模型、不改FPGA位流。运行中的程序同时更新LUT，之后启动读取保存配置，无需每次附RGB命令行参数。

2026-10-30核查：板端旧预览、旧双模型、新交付目录以及电脑副本中的revision2参数一致；采集转换和校色模块源码也一致。排查期间曾观察到前绿后粉；随后用户确认两套双目摄像头接反，接回后已正常。配置绑定前后输入通道，不会自动识别两套相机是否互换，所以错误接线会将一套相机的增益用于另一套。此次未重新校色，保留revision2及全部原增益。记录见Verification/CAMERA_ASSIGNMENT_RESOLVED_20261030.json。

**本次接回后已经正常，不需要运行以下校色命令。** 客户端为交付目录新增维护工具；当前板端上传的37文件版本没有它，未来有实际校色需要时才上传该文件，不必重传配置。

需要重新校色时保持本模块推理运行，把无反光的白纸或中性灰纸放到所选画面中央，覆盖宽40%、高40%的区域，稳定两秒。仅在纸张已就绪后，在RK第二个终端执行其中一条：

```bash
cd /userdata/Cascaded_Four_Camera/Board_RK3568 && ./Python_Environment/venv/bin/python -B CALIBRATE_COLORS.py front --confirm-neutral
```

换到后视，纸张稳定后：

```bash
cd /userdata/Cascaded_Four_Camera/Board_RK3568 && ./Python_Environment/venv/bin/python -B CALIBRATE_COLORS.py rear --confirm-neutral
```

新客户端确认8083正在使用本模块配置，调用原有本地校色接口，然后只更新MANIFEST中该配置的哈希，保留其他文件校验。旧配置/清单及结果备份在 `Board_RK3568/Results/Color_Calibration`，服务器另在Configuration/profile_history留存旧配置。避免使用旧目录的客户端校色后没有同步新清单，导致下次启动校验失败。

移开白纸后检查白/灰、肤色和红绿物体；不把“脚本成功”当作所有颜色都准确。一次校色不保证更换传感器、位流或照明条件后仍然有效。正常使用同一环境时直接启动START_INFERENCE.sh。

校色稳定后，需把板端实际 `Configuration/camera_profiles.json` 回收至本交付目录并同步MANIFEST及冻结记录。上传维护工具或源码时不要重新覆盖板端已经保存的有效配置。回退颜色应成对恢复同一次备份的配置与清单，正常重启后生效；不回退其他业务代码。
