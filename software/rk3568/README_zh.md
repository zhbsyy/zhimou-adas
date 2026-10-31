# RK代码导航

日常运行只用 `scripts/rk3568`。本目录按职责放源文件，不在这些分类文件夹里直接执行一个依赖同目录import的主程序。

主流水线：`fusion/run_vision.py`为Decision，`fusion/quad_infer.py`为物理四路，`fusion/run_carla_aeb.py`及`pcie_stream_r15_async.py`为CARLA，`fusion/process_video_r15_offline.py`为离线处理。`configs/layouts/rk3568_*.json`说明每一种方式实际选择的文件版本。

通用文件只放一份；同名但内容不同的文件放在对应profile子目录。比如检测后处理、R15适配和采集器可能因已验证版本不同而保留多个变体，不能只凭名字用一个覆盖其他版本。所有来源与哈希可在verification/source_mapping.json检索。

runtime_support里保留原启动、校验、私有依赖等兼容文件，是生成工作布局的组成部分；不是要求用户再运行历史升级补丁。统一环境只在项目environment/rk3568内准备一次。
