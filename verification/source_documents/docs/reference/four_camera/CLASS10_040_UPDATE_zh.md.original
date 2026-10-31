# 四路class10阈值更新到0.40

当前交付目录的class10/crosswalk阈值已从0.60改为0.40，并统一用于四路检测结果、HTTP输出及绘图。其他检测类仍为0.25，R15线条阈值0.60不变；模型权重、颜色配置、位流和采集参数不变。

已部署的37文件旧版不用重传整个模块，避免覆盖已验收颜色配置。将本模块Board_RK3568/APPLY_CLASS10_040.py上传到板端 `/userdata/Cascaded_Four_Camera/Board_RK3568/`，先在推理终端Ctrl+C正常退出，再执行：

```bash
cd /userdata/Cascaded_Four_Camera/Board_RK3568 &&
./Python_Environment/venv/bin/python -B APPLY_CLASS10_040.py &&
bash ./START_INFERENCE.sh
```

工具仅接受已知0.60/0.40文件哈希，替换detection_policy.py及quad_infer.py，更新板端原MANIFEST中的这两个条目，其他文件不动。重复执行不会叠加修改。新画面和网页标题直接读取统一阈值，避免“过滤已改、标签未改”。

备份保存在 `/userdata/Cascaded_Four_Camera/Board_RK3568/Results/Class10_040_Backups/<本次目录>/`，包含原文件与原清单；需要回退时正常退出推理，把同一次备份的两个源码文件和MANIFEST.before.sha256恢复到原位置，再启动。不要恢复其他颜色文件或模型。

验证：`http://192.168.2.3:8083/stat`中的class10_min_confidence应为0.4、other_min_confidence仍为0.25；画面底部class10应为0.40。必须重启程序，已经加载到内存中的0.60不会因磁盘文件改变而自动生效。

旧Migration/Preserved_Evidence记录保留原0.60，属于当时实测证据；不把它们改写成新阈值已经上板的证明。
