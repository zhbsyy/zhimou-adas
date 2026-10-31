# 斑马线候选输出策略

物理四路模式对YOLO class10=crosswalk采用最低0.40分数，其他普通类别保持0.25；R15分割阈值为0.60。过滤后的检测进入绘图和输出，不能把R15阈值与目标检测阈值混用。

策略位于`software/rk3568/object_detection/detection_policy.py`等模式实现中，配置记录位于`configs/profiles/four_camera.json`。配置文件表达定档参数，具体执行以实际调用代码为准。

该策略控制候选输出，不改变模型本身的类别判别能力。箭头、路面纹理等斑马线混淆仍需结合目标域场景评价。


[文档导航](../../README.md) · [系统使用](../../USER_GUIDE_zh.md)
