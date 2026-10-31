---
license: agpl-3.0

pipeline_tag: object-detection

library_name: ultralytics

datasets:
  - dronefreak/BDD100K

tags:
  - object-detection
  - detectionbench
  - ultralytics
  - pytorch
  - computer-vision
  - autonomous-driving
  - self-driving-cars
  - driving-scenes
  - traffic-sign-detection
  - traffic-light-detection
metrics:
  - map50
  - map50-95
  - precision
  - recall
  - f1

base_model: "Ultralytics/YOLOv8"

model-index:
  - name: YOLOv8n Finetuned on BDD100K
    results:
      - task:
          type: object-detection
          name: Object Detection
        dataset:
          name: BDD100K
          type: bdd100k
        metrics:
          - type: mAP50
            value: 51.67
            name: mAP@50 (test split)
          - type: mAP50-95
            value: 29.09
            name: mAP@50-95 (test split)
          - type: precision
            value: 70.95
            name: Precision (test split)
          - type: recall
            value: 46.59
            name: Recall (test split)
        source:
          url: https://github.com/dronefreak/DetectionBench
          name: DetectionBench
---


# YOLOv8n Finetuned on BDD100K

Fine-tuned YOLOv8n object detector on the **BDD100K** benchmark dataset, trained and evaluated as part of [DetectionBench](https://github.com/dronefreak/DetectionBench) -- a framework for reproducibly benchmarking modern object detectors with identical training recipes and evaluation metrics across multiple real-world datasets.

<!--
  Demo banner: side-by-side video of this checkpoint's detections on two
  BDD100K test clips. Media lives under assets/ in this repo. The <video>
  renders on the Hugging Face model page (absolute resolve/ URL); on GitHub
  the nested <img> poster is shown instead.
-->
<p align="center"><video controls autoplay loop muted playsinline width="900" poster="https://huggingface.co/dronefreak/bdd100k-yolov8n/resolve/main/assets/demo_banner_poster.jpg" src="https://huggingface.co/dronefreak/bdd100k-yolov8n/resolve/main/assets/demo_banner.mp4"><img src="https://huggingface.co/dronefreak/bdd100k-yolov8n/resolve/main/assets/demo_banner_poster.jpg" alt="YOLOv8n detections on two BDD100K test clips" width="900"></video></p>

<br>

<!-- ROW 1: Identity & Tech Stack -->
<div style="display: flex; justify-content: center; align-items: center; gap: 8px; margin-bottom: 8px; flex-wrap: wrap;">
  <img src="https://img.shields.io/badge/Task-Object_Detection-blue?style=flat-square" alt="Task">
  <img src="https://img.shields.io/badge/Framework-Ultralytics_YOLO-0aa1a7?style=flat-square" alt="Framework">
  <img src="https://img.shields.io/badge/Base_Model-YOLOv8n-purple?style=flat-square" alt="Base Model">
</div>

<!-- ROW 2: Performance Metrics -->
<div style="display: flex; justify-content: center; align-items: center; gap: 8px; margin-bottom: 8px; flex-wrap: wrap;">
  <img src="https://img.shields.io/badge/mAP@50-51.67%25-success?style=flat-square" alt="mAP@50">
  <img src="https://img.shields.io/badge/mAP@50:95-29.09%25-orange?style=flat-square" alt="mAP@50:95">
  <img src="https://img.shields.io/badge/Params-3.2M-lightgrey?style=flat-square" alt="Params">
</div>

<!-- ROW 3: Metadata -->
<div style="display: flex; justify-content: center; align-items: center; gap: 8px; margin-bottom: 24px; flex-wrap: wrap;">
  <img src="https://img.shields.io/badge/License-AGPL--3.0-lightgrey?style=flat-square" alt="License">
  <a href="https://github.com/dronefreak/DetectionBench"><img src="https://img.shields.io/badge/Source-DetectionBench-black?style=flat-square" alt="Source"></a>
</div>

---

## Usage

### Install Dependencies

```bash
pip install ultralytics huggingface_hub
```

### Load Model from Hugging Face

```python
from huggingface_hub import hf_hub_download
from ultralytics import YOLO

weights = hf_hub_download(
    repo_id="dronefreak/bdd100k-yolov8n",
    filename="best.pt"
)

model = YOLO(weights)
```

### Run Inference

```python
results = model.predict(
    source="image.jpg",
    conf=0.25
)

results[0].show()
```
---

## Performance

Evaluated on the BDD100K **test** split, using DetectionBench's standard evaluation pipeline (`detectionbench-evaluate`).

| Metric     | Score (%)       |
| ---------- | --------------- |
| mAP@50     | 51.67     |
| mAP@50-95  | 29.09   |
| Precision  | 70.95 |
| Recall     | 46.59    |
| F1 Score   | 56.25        |
| Parameters | 3.2M    |
| FLOPs      | 8.7B (at 640 px)     |

---

## BDD100K Model Zoo

Every model DetectionBench has trained and evaluated on BDD100K so far, for full transparency -- see [DetectionBench](https://github.com/dronefreak/DetectionBench) for the smaller, curated comparison set used on the project README.

| Model                | mAP@50        | mAP@50-95       | Precision         | Recall         |
| --------------------- | ------------- | --------------- | ----------------- | -------------- |
| YOLOv8m | 60.89 | 35.47 | 66.52 | 55.42 |
| YOLO26s | 58.76 | 33.86 | 75.49 | 53.07 |
| YOLOv8s | 57.93 | 33.25 | 75.38 | 51.63 |
| YOLOv10s | 57.64 | 33.34 | 75.02 | 52.12 |
| YOLO11s | 57.63 | 33.1 | 74.31 | 52.42 |
| RF-DETR Nano | 56.9 | 31.58 | 80.68 | 64.78 |
| YOLO26n | 52.25 | 29.23 | 72.56 | 46.87 |
| YOLOv9t | 52.04 | 29.46 | 71.34 | 46.72 |
| YOLOv10n | 51.95 | 29.31 | 71.58 | 46.62 |
| **YOLOv8n** | **51.67** | **29.09** | **70.95** | **46.59** |
| YOLO11n | 51.63 | 29.06 | 71.68 | 46.34 |
---

## Per-Class Performance

| Class                      | mAP@50          | mAP@50-95         |
| -------------------------- | --------------- | ----------------- |
| person       | 61.65 | 30.82 |
| rider       | 41.33 | 20.69 |
| car       | 79.67 | 49.26 |
| truck       | 61.86 | 44.77 |
| bus       | 60.43 | 46.63 |
| train       | 0.0 | 0.0 |
| motor       | 37.35 | 17.59 |
| bike       | 42.38 | 20.71 |
| traffic light       | 64.45 | 24.58 |
| traffic sign       | 67.55 | 35.87 |

![Normalized Confusion Matrix](confusion_matrix_normalized.png)

---

## Dataset

This model was trained on **BDD100K**. For the full dataset description, provenance, license, and citation, see the dataset card:

https://huggingface.co/datasets/dronefreak/BDD100K

### Classes

* person
* rider
* car
* truck
* bus
* train
* motor
* bike
* traffic light
* traffic sign
---

## Training Configuration

| Setting          | Value                           |
| ---------------- | -------------------------------- |
| Dataset          | BDD100K       |
| Framework        | Ultralytics YOLO                  |
| Training Toolkit | DetectionBench                   |
| Epochs (configured max) | 50 |
| Epochs (actually trained) | 50 |
| Early Stopping Patience | 10 |
| Batch Size | auto (Ultralytics AutoBatch) |
| Image Size | 960 |
| Optimizer | SGD |
| Initial Learning Rate | 0.01 |
| Seed | 0 |
---

## Repository Contents

```text
best.pt
results.csv
args.yaml
BoxPR_curve.png
BoxF1_curve.png
BoxP_curve.png
BoxR_curve.png
confusion_matrix.png
confusion_matrix_normalized.png
val_batch0_pred.jpg
bdd100k_yolov8n_showcase.jpg
assets/demo_banner.mp4
assets/demo_banner_poster.jpg
README.md
```

---

## Related Resources

* [BDD100K dataset card](https://huggingface.co/datasets/dronefreak/BDD100K) on Hugging Face
* [DetectionBench](https://github.com/dronefreak/DetectionBench) -- reproducible benchmarks for modern object detectors on real-world datasets
* [BDD100K project website (official data source)](https://www.bdd100k.com/)

---

## Training Framework

This model was trained using [DetectionBench](https://github.com/dronefreak/DetectionBench), an open-source framework for benchmarking object detectors across multiple real-world datasets with a common pipeline.

Features include:

* A dataset-adapter registry for converting real-world datasets into a canonical format
* Identical training/evaluation recipes across model families (Ultralytics YOLO/RT-DETR, RF-DETR)
* Hardware profiling (latency, FPS, VRAM, parameters, FLOPs)
* One-command reproducibility via versioned Hydra configs

If you find this model useful, please consider starring the repository.

---

## Known Limitations

* Not comparable to the official BDD100K test-server leaderboard: the official test split has no released labels, so the `test` split here is BDD100K's official validation set (10,000 images) and a seeded 15% slice of the official train set is held out for validation.
* Severe class imbalance: `car` (55.4%), `traffic sign` (18.6%) and `traffic light` (14.5%) dominate the boxes, while `rider` (0.4%), `motor` (0.2%) and especially `train` (about 150 boxes in the whole dataset) are rare -- per-class accuracy on those classes is measured on very few examples and is close to noise for `train`.
* Small objects: the median box covers only 0.09% of the 1280x720 frame, and traffic lights and signs are the smallest and hardest classes (medians of roughly 16 px and 21 px at native resolution), so scores on them depend heavily on input resolution.
* Detection labels only: BDD100K's lane-marking and drivable-area annotations are dropped, so these models cover the 2D object detection task only.
* Conditions are not broken down: the images span weather, time-of-day and scene conditions, but scores here are aggregated over all of them, and generalization outside the US road scenes BDD100K covers is untested.
* Non-commercial redistribution only: BDD100K's own data license (reproduced in full on the dataset card) permits redistribution for educational, research and not-for-profit purposes with notice -- commercial use needs separate permission from UC Berkeley's Office of Technology Licensing, not a blanket no-redistribution grant.
* Label vintage is 2018, not the paper's 2020 publication date: the detection labels mirrored here are BDD100K's original 2018 annotation release (matching the data license's own copyright year), carried through a two-hop Kaggle mirror -- see the dataset card for why a separate, later-dated label release couldn't be located or verified.
---

## Citation

If you use this model in your research, please consider citing the dataset and the model architecture:

```
@inproceedings{yu2020bdd100k,
  title={BDD100K: A Diverse Driving Dataset for Heterogeneous Multitask Learning},
  author={Yu, Fisher and Chen, Haofeng and Wang, Xin and Xian, Wenqi and Chen, Yingying and Liu, Fangchen and Madhavan, Vashisht and Darrell, Trevor},
  booktitle={Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition},
  pages={2636--2645},
  year={2020}
}
```

```bibtex
No official YOLOv8 research paper has been published by Ultralytics; this is their own recommended software citation instead:

@software{jocher2023yolov8,
  author = {Glenn Jocher and Ayush Chaurasia and Jing Qiu},
  title = {Ultralytics YOLOv8},
  version = {8.0.0},
  year = {2023},
  url = {https://github.com/ultralytics/ultralytics},
  license = {AGPL-3.0}
}
```