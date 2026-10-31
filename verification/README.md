# 系统验证依据

验证资料按硬件构建、软件接口、实板链路和仿真反馈分类保存。

|资料位置|内容与解释范围|
|---|---|
|`preserved_evidence/decision`|RK实时视频、停止请求、车辆刹车灯及静止车反馈记录|
|`preserved_evidence/four_camera`|四路软件契约、工程布局与模型接口记录|
|`preserved_evidence/carla_navigation`|座舱串口、导航路线及RK关联的仿真交互记录|
|`preserved_evidence/offline`|离线处理与输出范围记录|
|`original_reference`|原始模块资料和历史验收记录|
|`source_documents`|保留原字节的来源文档，用于运行布局兼容与来源映射|

Decision已有实板视频、规则状态与车辆反馈的联合记录；四路资料同时包含CPU接口检查与历史短时板端采样，两者的范围分别解释。CARLA记录用于仿真链路评价，不能替代真实道路自动驾驶性能。模型数据集指标、候选区域诊断与整机感知效果也分别评价。

`source_mapping.json`记录原模块文件与仓库来源位置，来源文档的阅读版位于`docs`等模块目录，原文以`.original`后缀保存。已有摘要与整包清单对应其历史归档范围，作品说明按Git版本维护，不将历史摘要表当成新版全部文档的校验结果。
