# 检查范围

```powershell
& "D:\python3.12.4\python.exe" -B -m unittest discover -s ".\tests" -p "test_project.py" -v
& "D:\python3.12.4\python.exe" -B ".\tests\run_preserved_PC_tests.py"
```

第一组检查新目录独立导入、原始文件哈希、四套RK布局、两个模型/录像去重、三个PC启动模式、三份PDS引用闭合和源目录不变。第二组把原PC测试恢复到其相对位置，只运行test_*.py，不运行check_crossing_live.py、check_motion_real_rk.py、pc_integration_check.py等会接管CARLA的脚本。

检查会生成工作目录，但不初始化PCIe、不执行RKNN、不编程FPGA、不创建CARLA车辆、不重启当前窗口。运行环境需现有PC Python包。legacy_*中保留原始测试用于来源追溯，不应从分类目录直接运行一个依赖旧相对位置的测试脚本。
