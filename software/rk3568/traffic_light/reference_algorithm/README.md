# YOLO 交通灯颜色：视频文件测试工具

2026-10-25：新增固定候选强制判色模式，支持红绿二选一或红黄绿三选一，详见 [强制判色使用说明](README_forced.md)。默认保守配置继续保留，启用新版请传 `--config hsv_config_forced_ryg.json`。原代码已备份到 `revisions/before_forced_20261025/`。

用于现有 YOLO 检出的 traffic light 框，增加 HSV 颜色判断及每目标时序稳定。
视频文件直接送 YOLO，不需要经过摄像头、FPGA 或 PCIe；单摄像头画面按普通整帧处理。
本工具独立于原工程，现有相机程序、模型和 FPGA 文件均未修改。

2026-10-07 v2：已针对用户提供的 20.2 秒道路视频改进黄色外壳误判与重复框。
默认只画有效交通灯的一条颜色标签，详细状态保存在日志中。

## 1. 文件

- `traffic_light_color.py`：可接到现有 YOLO 后的颜色及目标状态模块。
- `video_test.py`：读取视频，YOLO 推理，颜色识别，保存结果和耗时。
- `hsv_config.json`：HSV、确认次数、未知状态保持时间、跟踪参数。
- `classes_bdd100k.txt`：当前本地 ONNX 模型的 10 类顺序。板上优先传实际 classes.txt。
- `test_color.py`：人工构造灯珠、多个目标和 YOLO 解码行为测试。
- `make_test_video.py`：生成合成视频，仅供管线检查。

算法基于 OpenCV 的 HSV/inRange 颜色分割：
https://docs.opencv.org/4.x/da/d97/tutorial_threshold_inRange.html
连通域评分、目标关联、状态管理及本项目接口代码为本次编写。

## 2. 推荐：在 RK3568 上用当前模型测视频

使用 MobaXterm 的文件窗口把解压后的整个 `traffic_light_video_demo` 文件夹拖到：

`/home/linaro/yolov8_deploy/`

另外创建 `videos` 文件夹，把道路视频放到：

`/home/linaro/yolov8_deploy/videos/road.mp4`

在板上终端运行：

```bash
cd /home/linaro/yolov8_deploy/traffic_light_video_demo
../.venv-offline/bin/python video_test.py \
  --video ../videos/road.mp4 \
  --model ../models/bdd100k_yolov8n_rk3568_i8.rknn \
  --classes ../models/classes.txt \
  --yolo-dir .. \
  --out ./road_test_01 \
  --debug-every 30
```

路径依据现有 yolo_pcie.py 的默认值；如果你们实际目录或模型文件名不同，替换对应路径。
模型输入默认 640。如果现有模型是 416，必须加 `--size 416`。

RKNN 模式直接复用 `--yolo-dir` 下的 `postprocess.py`，包含你们当前的
preprocess / decode / nms / to_original 接口。输入转换沿用原程序的 uint8 RGB NHWC。
运行只需要当前已有的 numpy、OpenCV、rknn_toolkit_lite2；不需要 ONNX Runtime。
若原环境已经能运行 yolo_pcie.py，优先使用同一个 Python，不要随意替换运行库。
离线视频不访问 PCIe 设备，通常不需要 sudo。

可以先加 `--max-frames 300` 做短测试。输出目录须为空；重跑时改成 road_test_02 等。

## 3. 电脑上使用已有 ONNX 模型

电脑环境需要 numpy、OpenCV、ONNX Runtime：

```powershell
python -m pip install -r requirements_pc.txt
```

在本目录运行（视频和输出路径自行替换）：

```powershell
python video_test.py --video "F:\videos\road.mp4" --model "F:\current9-11\End\MES2L50H_DualCam_PCIe（2）\models_source\bdd100k_yolov8n_best_640.onnx" --out road_test_pc_01 --debug-every 30
```

上述 ONNX 是工作区已有的 BDD100K 模型，本次未重新训练或修改。
支持静态 float32 RGB NCHW 输入，以及标准 YOLOv8 单输出、Rockchip 6/9 路 DFL+sigmoid 输出。
检测类别顺序必须与模型一致；有 names 元数据时会检查是否匹配。
ONNX 和 INT8 RKNN 可能存在量化差异；电脑测试不能代替最终上板测试。

本次验证用的依赖仅放在 `.test_deps` 中。若使用 Codex 的内置 Python，需把该目录加入
PYTHONPATH；你们自己的电脑 Python 可以直接使用上述 requirements_pc.txt。

## 4. 输出怎么看

| 文件 | 内容 |
|---|---|
| annotated.avi | 默认仅显示交通灯框和 RED/GREEN/YELLOW 简短标签 |
| lights.csv | 每个交通灯每个采样帧的判断及框坐标 |
| frames.jsonl | 每帧目标数量、交通灯结果、各阶段耗时；无灯帧也记录 |
| summary.json | 实际处理速度、颜色计数、平均耗时、参数和是否中断 |
| crops/ | 指定间隔保存的原始灯框裁剪图，便于调整 HSV |

默认标签例子：`GREEN`。临时保持的状态后面显示问号，表示这一帧没有清晰新证据。
`--display all` 同时画车辆、行人、交通标志等现有类别。
`--display debug` 显示目标 ID、原始颜色、状态与顶部耗时栏。
`--show-unknown` 在 clean/all 模式下也画不确定目标；默认隐藏不确定框。
调试标签例子：`TL #1 GREEN raw:green confirmed`。
采用英文标签以兼容板上 OpenCV 字体，RED=红、YELLOW=黄、GREEN=绿、UNKNOWN=未知。

- confirmed：已有一致证据。
- pending：新目标或颜色切换等待确认，显示 UNKNOWN。
- held：这一帧不清楚，短暂保留最近已确认颜色，并明确标注 held。
- unknown：没有可靠当前颜色，或保持时间已过。

一旦发现相反颜色，会取消旧颜色；不能在下一帧不清楚时又恢复旧灯态。
跟踪只用框位置、重叠和大小，颜色变化不会直接导致换 ID。
目标消失后不会把旧框继续画在新视频上；重新出现超时会创建新 ID。
轻量几何跟踪在相邻灯框重叠、运动很快或镜头切换时仍可能关联错误。

## 5. 速度和时间

离线模式逐帧处理，处理慢也不丢掉未处理帧。按视频源时间做时序判断，
不会因为电脑处理慢而把一个灯态保留很久。
当前采用固定帧率时间 `frame_index / source_fps`；可变帧率素材应先转换为固定帧率。
若素材 FPS 元数据错误，用 `--fps 25` 等指定正确值。

annotated.avi 按素材帧率播放，播放流畅并不代表推理实时。
真实处理速度看 summary.json 的 processing_fps；各阶段耗时看 average_ms。
本工具计时包含解码、YOLO、颜色/跟踪、绘制与视频保存，模型加载不计入处理速度。
它不会自动加速原有相机推理或浏览器显示链路。

默认每帧检测。若用 `--stride 2`，每两帧检测一次，结果视频帧率相应减半，保持大致时长。
稀疏采样可能漏过短暂黄灯，也会影响跟踪，首次验证建议 stride=1。
`--no-video` 可只生成日志，比较视频编码所占耗时。
`--show` 仅用于具备桌面的本地环境；默认无显示窗口，适合远程终端。

## 6. 参数和调试

默认两次一致观测确认，至少相隔 0.05 秒；一次 unknown 最多保持旧状态 0.30 秒。
候选观测最大间隔及目标失联时间均为 0.75 秒。这些是初值，需要用实际素材调节。
当前阈值不会强制按灯珠上下顺序判色，适用于竖排和横排的圆形灯珠。
v2 默认增加明亮程度与近圆形灯珠筛选，主要用于圆形机动车信号灯；
箭头灯、暗灯或极小灯珠可能被拒绝，不能把此配置视为覆盖所有灯型。
算法只输出颜色，不判断箭头方向、所控制车道或是否应该通行。

v2 用 min_mean_value=175 筛除本素材中平均亮度约 130～153 的黄色外壳候选。
同时检查 min_circularity 与 min_component_aspect；这些都是可调初值。
roi_padding 会扩展被 YOLO 裁断的灯框，但 max_center_outside 限制灯珠中心
相对原检测框的位置，避免把旁边的灯偷用作当前目标颜色。
重复框按实际灯珠位置合并，不采用固定画面区域，也不强制只保留两个灯。

先看原始裁剪图及 raw_color：

- 若 YOLO 没框出灯，颜色模块无从判断；先检查检测框、置信度和类别文件。
- 框内有清晰灯珠但 raw_color 一直 unknown，检查 HSV 参数与候选面积门槛。
- 框太大或背景干扰多，优先检查框位置和真实灯珠区域。
- 原始灯珠过曝成白色、太小或模糊，降低阈值不一定能恢复信息。
- raw_color 正确但稳定状态切换慢，再调整 confirm_samples / confirm_seconds。

低频实时检测接入时，max_sample_gap、track_ttl 应与实际检测间隔相匹配。
离线逐帧模式即使处理速度很慢，其采样间隔仍由视频时间决定。
RGB565 实拍与普通压缩视频的颜色可能不同；素材测试通过后仍需实际相机验证。

## 7. 之后接到摄像头程序

只需在坐标还原后、绘制之前调用模块。单摄像头普通画面可直接用整帧：

```python
import time
from traffic_light_color import TrafficLightColorEngine, draw_results

# 初始化一次，不能放在每帧循环里。
color_engine = TrafficLightColorEngine()

# 每次 YOLO 输出之后。dets 为原图坐标 [x1,y1,x2,y2,conf,class_id]。
# 即使没有检出目标，也调用 update，以便清理过期目标。
states = color_engine.update(bgr, dets, names, time.monotonic())
vis = draw_results(bgr, dets, names, states, title, display="all")
```

以上是接入示例，当前未修改摄像头入口。
视频测试时使用素材时间；实时相机接入时应使用采集时间或 monotonic 时间。

## 8. 验证记录与边界

本次已完成 23 项行为测试：三种颜色、红色色相环绕、暗灯、过曝白灯、冲突颜色、
无效及小框、两个目标独立、检测顺序变化、切灯确认、未知超时、低采样率、
失联、重复时间戳、输入图像不被修改以及 YOLO 单输出/DFL 解码等。
v2 另覆盖黄色表面、裁断灯珠的恢复及相邻灯排除、重复框合并、框尺寸变化。

已完成 120 帧/20 FPS 合成视频检查：两个灯独立、红→绿→黄→未知、切换确认、
输出视频帧数/帧率和日志一致。合成素材不是真实道路，也没有经过 YOLO 定位。

使用工作区现有 ONNX 模型完成实际推理入口测试，但合成图未检出交通灯，
该测试仅证明模型可以运行，不能证明真实交通灯颜色识别准确率。
另外在同一图像上以 0.01 诊断阈值对比标准单输出与已有 head9 模型，
4 个检测结果的框、分数与类别在指定容差内一致；实际使用仍默认 0.25。
用户已在板上运行过 v1 RKNN 版本并提供原视频及标注视频；
v2 在电脑上使用已有 ONNX 模型复测，尚待板上 RKNN 确认。

真实道路视频已提供并用于本次改进，是单一素材验证，不能据此报告通用准确率。
输出帧数、颜色计数及剩余不确定情况详见 validation_report.json。
summary.json 的颜色计数是观测统计，并不是准确率。

使用同一批 485 帧及相同 YOLO 检测框对比 v1/v2，v1 有 555 次原始黄色判定，
v2 为 0 次。本素材中该黄色区域为灯的侧面外壳，抽帧检查支持这一判断。
新版最大显示 2 个有效灯框；450 帧显示两个，27 帧显示一个，8 帧无有效显示。
这反映仍有不确定/漏检情况，不能把删除黄色误判等同于全部帧都检测正确。
提供的旧视频是板上 RKNN 输出，新视频是电脑 ONNX 输出；上述同检测框对比
用于单独确认后处理改动的作用，最终仍需 RKNN 复测。

```bash
python -m unittest -v test_color
python make_test_video.py
python video_test.py --video validation/synthetic_input.avi --backend roi \
  --roi 125 90 175 270 --roi 405 90 455 270 --out validation/color_check
```

ROI 模式仅用于测试颜色/时序管线，固定框不会跟随运动，不能作为 YOLO 检测结果。
