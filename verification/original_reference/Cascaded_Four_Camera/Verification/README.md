# 当前验证索引

`FROZEN_VERSION.json`为当前参数，`RELEASE_SYNC_20261030.json`为本轮覆盖同步与状态范围，`MANIFEST.sha256`覆盖静态交付文件；正常运行入口直接使用本目录当前源码，不需要恢复任何历史版本。

- `LOCAL_VALIDATION.json`：当前0.40代码的9项流水线回归、3项路径检查及板端/PC入口导入检查。HTTP用例是本机合成数据检查，不是RK硬件性能测量。
- `CLASS10_040_UPDATE_20261030.json`：class10更新、显示文字统一、补丁防错与验证范围。
- `CONSOLE_REFRESH_20261030.json`：新版v1.5总控台同步及默认PCIe选择。
- `CAMERA_ASSIGNMENT_RESOLVED_20261030.json`：用户确认两套双目接反、接回后恢复，原revision2颜色参数未修改。
- `MIGRATION_CHECK_20261030.json`：目录迁移后实际四路运行与PC显示，采样时class10仍为0.60，保留历史原值。
- `Preserved_Evidence`与`PROVENANCE.json`：原始来源、构建、校色、实测记录，按各自时间理解；其0.60不能作为当前0.40已上板的证明。

本轮API未开放时不会把待核对运行状态写为通过。`verify_submission.py`只核文件完整性，不操作JTAG、采集、NPU或车辆；`run_local_checks.py`只做本机软件检查。运行生成的Results、缓存和现场复制的venv不纳入静态清单。Decision与CARLA_Navigation内容未随本轮覆盖修改。
