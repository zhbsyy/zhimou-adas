# 上传、运行与下载结果

## 1. 在板端创建可写目录，再上传

```bash
mountpoint -q /userdata && sudo install -d -o linaro -g linaro "/userdata/Final submission/Offline_Video_Processing"
```

通过MobaXterm/SFTP，将电脑本模块中的 `Board_RK3568`、`Verification` 和 `MANIFEST.sha256` 上传到上述目录。不要把内容直接拖进没有写权限的 `/userdata/` 根目录，也不要多套一层模块名。

最终入口必须是：

```text
/userdata/Final submission/Offline_Video_Processing/Board_RK3568/START_OFFLINE.sh
```

## 2. 第一次准备独立Python环境

保留原 `/home/linaro/yolov8_deploy/.venv-offline`，执行：

```bash
cd "/userdata/Final submission/Offline_Video_Processing/Board_RK3568" && bash ./PREPARE_PYTHON_ENVIRONMENT.sh
```

不联网安装、不改旧环境；复制到本模块 `Python_Environment/venv` 并检查RKNNLite、OpenCV、NumPy、系统librknnrt。若原目录以后已迁移，可把已验证的其他RK venv路径作为参数传入。只有看到 `ENVIRONMENT_READY` 且新入口验证成功后，才能考虑清理旧环境；此次不会替你删除任何内容。

## 3. 正常停止其他板端推理，然后处理完整视频

在当前Decision、四路或CARLA推理终端按Ctrl+C。PC总控台可以保持打开，但此时不会收到新的RK实时结果；不要同时运行另一个NPU任务。

完整处理1.mp4，class10=0.60，其余类别0.25：

```bash
cd "/userdata/Final submission/Offline_Video_Processing/Board_RK3568" && bash ./START_OFFLINE.sh --video /userdata/videos/1.mp4
```

处理2.mp4：

```bash
cd "/userdata/Final submission/Offline_Video_Processing/Board_RK3568" && bash ./START_OFFLINE.sh --video /userdata/videos/2.mp4
```

直接复现已收入模块的原始道路录像：

```bash
cd "/userdata/Final submission/Offline_Video_Processing/Board_RK3568" && bash ./START_OFFLINE.sh
```

若需要对比class10=0.40，只改参数，不修改源代码：

```bash
cd "/userdata/Final submission/Offline_Video_Processing/Board_RK3568" && bash ./START_OFFLINE.sh --video /userdata/videos/1.mp4 --class10_conf 0.40
```

可选：`--seconds 10`只处理前10秒；`--start 5 --seconds 10`处理5～15秒；`--stride 3`每3帧处理1帧、输出帧率降为原来的1/3，播放时长基本保持；`--max_width 0`不缩小原视频。默认没有45秒或10秒上限。

## 4. 结果与完整性

终端打印 `OFFLINE_RESULTS=/userdata/.../Results/run_...`。每次建新目录，不覆盖旧结果：

- `result.mp4`：模型绘制后的静音视频。
- `result.summary.json`：输入、输出、阈值、源/输出帧率、帧数、时长、逐阶段耗时、停止原因及 `complete`。
- `run.log`：完整处理日志和 `OFFLINE_SUMMARY`。
- `invocation.json`：本次实际命令及校验情况。

程序使用sudo调用RKNN，退出时会把本次结果交回原登录用户，SFTP可直接下载。下载整个run目录更方便核对。播放MP4，不需要打开18085或其他网页。

开始前要求输出分区空闲至少1GiB；长视频可能需要更多。`complete=true`且 `stop_reason=source_eof` 表示正常读完；指定时长时 `duration` 表示达到请求的片段长度。`low_disk_space`、`interrupted`、`decode_ended_early`、`error`均为部分结果，返回非零退出码，不冒充完整成功。查看 `processed/output_fps` 可核对输出时长。

不要改用 `/tmp` 绕过空间不足：它以前与 `/home` 同在根分区。空间不足时保留原文件，换有空间的userdata子目录或先自行清理已备份的旧输出。允许用 `--output_dir /userdata/另一个可写文件夹` 指定结果父目录。

## 5. 恢复实时Decision

离线程序结束后，回到Decision模块启动；离线处理未改动摄像头位流或PCIe驱动。若PCIe状态仍正常无需重刷位流；看到BAR1=0时先正常停止采集，再执行Decision自己的INITIALIZE_PCIE.sh。

```bash
cd "/userdata/Final submission/Decision/Board_RK3568" && bash ./START_DECISION.sh
```
