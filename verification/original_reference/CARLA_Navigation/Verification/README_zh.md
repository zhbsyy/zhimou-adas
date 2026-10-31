# 验证记录的层次

`VALIDATION.json` 是本次整理的总索引；`FINAL_RELEASE.json` 固定版本、模型和范围；`MANIFEST.sha256`校验静态交付文件。

| 记录 | 证明范围 |
|---|---|
| PHYSICAL_COM12_RK_ROUTE.json | 实际Android/COM12＋CARLA＋真实RK，同一会话、scene96；中段制动、停稳、通过续行、到达；从既有运行日志摘录，不是本次打包另跑 |
| RELEASE_LIVE_STATE_20261030.json | 打包前只读快照；PC已到达、四相机在线；该次RK HTTP取样超时，如实保留 |
| MOTION_CHECK.json | v1.5 PC指定目的地＋真实RK；不是Android串口全链路；为此前阈值调整前的独立运动检查 |
| PC_INTEGRATION_CHECK.json | 使用测试UART/UDP夹具＋实际CARLA；不能冒充真实Android/RK |
| UNIT_TESTS.txt | 规则、时效、串口、控速、横穿、GUI邮箱等离线单元回归 |
| PACKAGING_CHECK.json | 移动到隔离目录后导入及清单检查；不启动车辆、不调用NPU |
| SOURCE_BASE.json / RELIABILITY_FIX.json | 历史来源/修复过程，时间和范围按各记录，不替代上述最新总索引 |

端到端证据中的板端日历时钟不正确，分组使用会话、scene_revision、actor_id、CARLA frame。class10阈值不是每条PC事件的字段，不能据此声称0.30比0.60更准确。当前交付代码阈值为0.30，模型SHA另行固定。

检查静态文件可运行 `python -B Verification/verify_submission.py`；离线回归可运行 `python -B -m unittest discover -s tests -p "test_*.py"`。`pc_integration_check.py`、`check_motion_real_rk.py`、`check_crossing_live.py`会操作CARLA场景，正式总控台运行时不要执行。本次整理只重跑离线检查，不重复控制车辆。
