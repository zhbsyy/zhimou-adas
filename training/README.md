# 训练与转换材料

日常演示直接使用 `models` 中的两份INT8 RKNN，不需要训练环境。训练源码随GitHub工程提交，用于说明模型来源、监督机制和必要复现；不提交完整数据集、虚拟环境、训练日志或中间恢复状态。

- [YOLOv8n 11类正式训练](object_detection/README_zh.md)：`src/train_joint_v2.py`、真实训练协议、数据配置，以及原UNKNOWN屏蔽、TAL/回归、旧类教师保护、数据读取、初始化和验证依赖。已归档的原A PT保留现有路径 `models/object_detection/reference/A_original.pt`。
- [R15训练与转换](lane_detection/README_zh.md)：已有最终阶段训练、配置、初始化权重、清单、指标、PT/ONNX和转换材料；补充前两主阶段的准确源码与配置、固定验证评估脚本及模型来源记录。早期阶段初始化权重和原始训练数据仍为外部输入。

YOLO训练环境见 [yolo_requirements.md](../environment/yolo_requirements.md)，与PC总控台环境及旧参考转换vendor分开使用。数据清单、初始化输入和原始图片/掩膜需按各模块说明恢复；放入训练代码不等于已经附带完整训练数据。

现有原A部署链有历史PT/ONNX导出与板端模型身份记录，但原Ubuntu的 `conversion_result.json` 未随包归档。不能承诺重新量化得到字节相同的RKNN。当前演示模型和控制逻辑不因本次文档收尾而改变。
