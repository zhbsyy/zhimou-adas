# 启动速查表

Windows：进入 `scripts/windows` 双击对应CMD。需要CARLA的模式先开START_CARLA_ENGINE.cmd；只开物理四路不需要引擎。先关闭原总控台。

| 想做什么 | PC入口 | RK终端（先cd到/userdata/Final submission 2） |
|---|---|---|
| 首次准备共用环境 | 无 | `bash scripts/rk3568/PREPARE_ENVIRONMENT.sh` |
| 决策 | START_DECISION_CONSOLE.cmd | `bash scripts/rk3568/INITIALIZE_PCIE.sh decision && bash scripts/rk3568/START_DECISION.sh` |
| 四路摄像头 | START_FOUR_CAMERA_CONSOLE.cmd | `bash scripts/rk3568/INITIALIZE_PCIE.sh four_camera && bash scripts/rk3568/START_FOUR_CAMERA.sh` |
| CARLA导航＋AEB | START_CARLA_NAVIGATION_CONSOLE.cmd；点击顶部“连接CARLA与四相机”开推流 | 首次 `bash scripts/rk3568/PREPARE_CARLA_DECODER.sh`，随后 `bash scripts/rk3568/START_CARLA_INFERENCE.sh` |
| 离线1.mp4 | 不用总控台，下载结果播放 | `bash scripts/rk3568/START_OFFLINE.sh --video /userdata/videos/1.mp4` |
| 原归档快照校验（见下方说明） | VERIFY_PROJECT.cmd | `/usr/bin/python3 -B scripts/project.py verify` |

每次新终端先执行：

```bash
cd "/userdata/Final submission 2"
```

| 服务 | 地址 |
|---|---|
| 物理四路处理后视频 | http://192.168.2.3:8083/stream |
| Decision视频 | http://192.168.2.3:18089/stream |
| Decision规则摘要 | http://192.168.2.3:18088/snapshot |
| PC原始CARLA四路推流 | http://192.168.2.10:8081/stream |
| PC相机状态 | http://192.168.2.10:8081/status |
| RK处理后的CARLA | http://192.168.2.3:18084/stream |
| PC导航诊断（本机） | http://127.0.0.1:18090/status |

浏览器地址用于单独核对，不替代总控台按钮与车辆反馈。没有运行对应服务时连接失败是正常现象。

原整包清单保留为文档收尾前的归档快照，本次未重算校验值。上述全量校验会提示已更新文档的差异，不可将其声称为本次新增训练文件和文档的校验通过记录；不影响按原模式配置运行。
