# HSV 固定候选强制判色版（2026-10-25）

目标：YOLO 已检出的有效交通灯框，即使偏暗、很小、模糊或颜色接近，也在固定候选中选一个结果。一个 YOLO/RKNN 不变，新增逻辑只依赖已有 NumPy、OpenCV，在 CPU 运行。

## 文件与模式

- `traffic_light_color.py`：修改后的算法及同目标时序平滑。
- `video_test.py`：已有视频入口，增加强制模式参数和颜色权重日志。
- `hsv_config_forced_ryg.json`：红/黄/绿三选一，建议先用这一份。
- `hsv_config_forced_rg.json`：红/绿二选一，只用于明确没有黄灯的两状态素材。
- `hsv_config.json`：原保守配置，仍保持原字节内容。
- `classes_bdd100k.txt`：原十类，traffic light=8，traffic sign=9。

默认不传新配置时仍走原保守逻辑。请显式传 `--config ./hsv_config_forced_ryg.json` 启用新版。

## 算法具体变化

1. 取 YOLO 原图灯框，BGR→HSV。强制模式不扩框，避免把框外邻灯或建筑拉入投票。
2. H 使用环状距离：落在某色原有色相区间内时距离为0，区间外按 `exp(-0.5*(距离/8)^2)` 逐渐降低。红色同时处理0和179两侧。
3. 像素权重为 `(S/255)^1.5 * (V/255)^1.5`。每色取最高5%像素响应的平均值作为 score，不再用硬性亮度、最小面积、圆形度、对比度阈值拒绝。
4. `p(color)=score(color)/sum(score)`，从候选中argmax。这里p是启发式归一化相对权重，未经真实有标签数据校准，不能解释为“正确率90%”。
5. 同一个目标对p做按时间的EMA：`alpha=1-exp(-dt/0.15)`。第一帧立即给出候选，不等待两帧确认；颜色过渡可能有约若干采样帧的平滑延迟。要立即跟随当前帧，可将 `force_smoothing_seconds` 改为0。

全黑/全灰/全白且没有色彩证据时，原始p为均匀分布。已有目标参考自己的历史倾向并向均匀分布衰减；新目标按配置候选顺序打破平局，默认先选red。日志明确记录无颜色证据。此时显示标签是强制选择，不是恢复了丢失的颜色。

有效非空灯框即输出候选；无检测框、无效坐标、非交通灯类别不会产生新灯色。新模式保留上游YOLO框，不凭强制颜色虚构“同一灯珠”的合并依据，因此同一真实灯存在多框时仍可能显示重复框。

## 板端文件目录

把压缩包内 `traffic_light_video_demo` 放到 `/home/linaro/yolov8_deploy/` 下。需要同步上传两份修改后的py文件，不能只上传配置。也可解压到新的独立目录，将 `--yolo-dir` 指向原程序目录。

```text
/home/linaro/yolov8_deploy/
  .venv-offline/                  # 原板端Python环境
  postprocess.py                 # 原YOLO后处理，复用其九输出解码等接口
  models/china_road_yolov8n_rk3568_i8.rknn
  videos/road.mp4
  traffic_light_video_demo/
    traffic_light_color.py
    video_test.py
    hsv_config.json
    hsv_config_forced_ryg.json
    hsv_config_forced_rg.json
    classes_bdd100k.txt
```

### 红黄绿三选一

```bash
cd ~/yolov8_deploy/traffic_light_video_demo

../.venv-offline/bin/python video_test.py \
  --video ../videos/road.mp4 \
  --model ../models/china_road_yolov8n_rk3568_i8.rknn \
  --classes ./classes_bdd100k.txt \
  --yolo-dir .. \
  --config ./hsv_config_forced_ryg.json \
  --size 640 \
  --conf 0.25 \
  --display all \
  --max-frames 300 \
  --out ./forced_ryg_01
```

红绿二选一仅把config换成 `./hsv_config_forced_rg.json`，out改成 `./forced_rg_01`。完整视频去掉max-frames；重复测试用新的out目录，保留此前结果。

也支持命令行直接指定：

```bash
# 在原有完整运行命令中增加以下参数（二选一）：
--color-mode forced --color-candidates red yellow green
--color-mode forced --color-candidates red green
```

输出 `annotated.avi`、`lights.csv`、`frames.jsonl`、`summary.json`。不要加 `--no-video`，否则没有标注视频。`--display all`保留车辆、行人、交通标志显示；`clean`仅灯；`debug`显示目标ID、raw颜色和forced状态。

画面 `RED* / YELLOW* / GREEN*` 的星号表示强制模式。CSV新增p_red/p_yellow/p_green、evidence_strength、candidate_margin、forced_choice；二分类排除的p_yellow为空。JSON还包含raw_probabilities与平滑后probabilities。score是单帧获胜权重，实际显示结果来自平滑后的probabilities。两个最高权重差越小，倾向越不明确。

## 接实时PCIe摄像头入口

初始化一次：

```python
from traffic_light_color import TrafficLightColorEngine, load_config, draw_results

color_cfg, state_cfg = load_config("hsv_config_forced_ryg.json")
engine = TrafficLightColorEngine(color_cfg, state_cfg)
```

YOLO还原坐标之后、在原始图上画框之前：

```python
# 同一帧BGR和原图坐标Nx6框：x1,y1,x2,y2,confidence,class_id
states = engine.update(bgr, dets, names, source_timestamp_seconds)
vis = draw_results(bgr, dets, names, states, display="all")
```

四路分别使用独立engine；source_timestamp_seconds使用各路单调递增采集时间，不能把整幅拼图的框直接配到局部前视图。若现有上位机只接收status=confirmed，需要明确支持新status=forced/forced_no_evidence；不要把它们自动当作原保守模式的confirmed。本轮修改算法和视频入口，没有自动改写当前PCIe采集主程序，也没有上板执行。

## 本机验证与实际发现

原23项行为测试仍通过，新15项覆盖弱饱和/暗灯、1像素ROI、模糊/JPEG、白/黑平局、两/三候选、时序与目标隔离，共38项。

两个120帧合成视频分别通过三候选配置、两候选命令行的完整输入/输出检查，各240条灯框观测，无UNKNOWN。合成检查不是检测准确率。

使用既有真实道路原视频485帧和保留的同一批YOLO框做复放，没有重新运行YOLO。另模拟缩小到1/5尺寸、JPEG质量25、放回原尺寸；这是低画质模拟，不是实际PCIe退化重现。

|输入|保守模式 UNKNOWN条数|强制三色 UNKNOWN条数|强制两色 UNKNOWN条数|
|---|---:|---:|---:|
|原图|1088/2015|0/2919|0/2919|
|模拟低画质|1868/2455|0/2919|0/2919|

保守模式会合并有验证灯珠依据的重复框；强制模式保留YOLO框，两者观察条数不同，不可据该表计算准确率提升。复放确实恢复了不少模糊红/绿标签，也重新把黄色外壳框选为yellow；红绿模式则将该外壳选为red。**少了UNKNOWN并不自动意味着颜色更准确。** 这份素材无完整人工灯色GT，尚不能报告精度提升；没有本轮板端FPS数据。

本机对照图、前120帧对照视频及日志在 `validation/forced_20261025/road_replay/`，未放入上传包。日志包括原始和模拟低画质两种输入、三种模式的逐帧结果。

旧代码备份在本机 `revisions/before_forced_20261025/`，包含原始SHA256；旧配置和既有模型不改动。验证：`python -m unittest -v test_color test_forced_color`。
