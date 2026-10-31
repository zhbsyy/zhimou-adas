# 离线分析入口

在板端仓库根目录运行：

```bash
bash scripts/rk3568/START_OFFLINE.sh --video "$VIDEO_FILE"
```

默认class10阈值0.60。`--class10_conf`指定检测对照阈值，`--seconds`限制处理片段，`--max_width`控制输出尺寸。采用片段上限的结果按摘要标记，不与全片处理混用。

输出`result.mp4`、`result.summary.json`及日志。`complete=true`与`source_eof`表示完整读到源结尾。PC可通过`INSPECT_OFFLINE_RESULT.cmd`读取下载结果的帧数和摘要。


[文档导航](../../README.md) · [系统使用](../../USER_GUIDE_zh.md)
