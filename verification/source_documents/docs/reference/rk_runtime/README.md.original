# Board_RK3568 — 板端运行部分

正式入口 `START_DECISION.sh`；PCIe恢复 `INITIALIZE_PCIE.sh`；首次环境备份 `PREPARE_PYTHON_ENVIRONMENT.sh`。

| 子目录 | 用途 |
|---|---|
| `Inference_and_Rules` | 原样保存当前推理、摘要、规则、HTTP服务及回归测试；不直接运行旧候选启动器 |
| `Camera_PCIe` | 当前MMIO32读帧器、原解码器、对齐工具、实板`.so`与C源码 |
| `Models` | 实际运行的两模型及classes_11顺序 |
| `Configuration` | 固定输入RGB增益和双目参考标定 |
| `Dependencies` | 从旧工程收拢的检测前后处理及兼容模块 |
| `System_Driver` | 当前内核驱动二进制、现存源码，普通启动不重新编译 |
| `Startup_Selfcheck_Video` | 启动一致性自检读取的原始视频；实时输入仍为摄像头 |
| `Python_Environment` | 首次备份脚本及备份成功后独立运行的Linux venv |
| `Results` | 新入口的运行日志与参数记录，首次运行时创建 |

请先阅读同级上层 `Documentation/QUICK_START_zh.md`。整个Decision上传到 `/userdata/Final submission/Decision/` 后使用带引号的路径。当前采集不需要每次再生成试验目录或重新编译 `.so`。
