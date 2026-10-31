# 完整性和本地检查

在PC打开Decision目录的PowerShell，执行：

```powershell
& "D:\python3.12.4\python.exe" -B ".\Verification\verify_submission.py"
```

会校验归档源代码、模型、录像、位流等文件的SHA256。程序退出码0且 `passed: true` 表示匹配定档文件。模型、代码参数或PC设置经有意修改后，原始校验必然显示差异；请另存新版本，不要修改本次冻结记录来掩盖变化。

首次板端备份的 `Python_Environment/venv` 和运行产生的 `Results` 不属于原始Windows清单，按环境说明另行备份。未收入清单的新增文件不会导致原有文件校验失败。

本地规则/面板测试：

```powershell
& "D:\python3.12.4\python.exe" -B -m unittest discover -s ".\Board_RK3568\Inference_and_Rules\tests" -v
& "D:\python3.12.4\python.exe" -B -m unittest discover -s ".\PC_Windows\Tests" -v
& "D:\python3.12.4\python.exe" -B -m unittest discover -s ".\Verification" -p "test_submission_layout.py" -v
```

前两组验证冻结代码的既有规则/显示逻辑；第三组验证来源哈希、模型和ARM64动态库、连续启动参数、目录内导入、PC跨目录迁移、PDS引用闭合及无压缩包。它们不运行真实RKNN、不抓PCIe、不创建CARLA车辆，因此不能代替 `QUICK_START_zh.md` 的一次上板验收。HTTP回归测试仅访问本机回环地址，运行它们时不要同时用同一个测试端口。
