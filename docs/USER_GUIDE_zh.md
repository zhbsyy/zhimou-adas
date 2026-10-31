# 从零使用本项目

## 先分清四台设备和四种程序

- Windows电脑：运行CARLA引擎和总控台，显示视频、模拟车辆、连接树莓派串口。
- 黑色RK3568＋PG2L50H复合板：FPGA采集，RK运行两个INT8模型与决策规则。
- 红色PGL50H板：物理四路模式的后向双目采集，经SFP级联；Decision不使用红板。
- 树莓派Android座舱：选择导航目的地，通过串口COM12发报文；静止Decision和物理四路显示不需要它。

CARLA“引擎”负责世界运行；“总控台”负责按钮、导航、相机和画面；“RK推理终端”负责模型；“推流器”由CARLA导航总控台里的按钮启动。不是所有模式都需要四者一起开。

## 源码与自检视频

当前源码目录未附 `assets/videos/video_20260929_112507.mp4`。保留的部分布局/预检仍要求这份文件，使用对应模式前先恢复原片。本机可从 `F:/current9-11/End/officialVideo/video_20260929_112507.mp4` 复制到上述项目路径；异机需另行取得对应原片。未放回时缺文件报错不代表模型损坏。此次文档收尾没有补入视频，也没有改掉原启动布局或其校验要求。

## 一、电脑端第一次准备

1. 保留已有Python、CARLA、PDS、显卡/下载线驱动。此次整理没有安装或升级软件。
2. 正常关闭其他总控台。原目录和新目录的总控台不能同时运行，都会使用8888和18090。
3. `VERIFY_PROJECT.cmd`按原整包清单检查。此次训练资料补入和文档收尾未重算校验值，原清单是整理前快照，已更新文档会报差异；不将该旧清单的全量检查作为本次启动的必做步骤。运行源码及模型未因文档收尾改变。
4. 当前Python为 `D:\python3.12.4\python.exe`，CARLA为 `D:\CARLA_download`。安装位置变化时按environment说明调整PC配置和RUN_PC.cmd；当前电脑不用改。

## 二、板端第一次上传

在MobaXterm右侧RK终端执行下面一条命令，只复制代码框里的命令，不复制 `linaro@...$` 提示符，不加反引号：

```bash
mountpoint -q /userdata && sudo install -d -o linaro -g linaro "/userdata/Final submission 2"
```

刷新SFTP，进入 `/userdata/Final submission 2`，将电脑本项目**里面的内容**上传进去。首次可上传整个项目内容（以实际目录大小为准），不要再套一层 `Final submission 2`。`.runtime`、已生成results和venv不用从Windows上传。

正确路径是 `/userdata/Final submission 2/scripts/rk3568/START_DECISION.sh`。不是 `/userdata/Final submission 2/Final submission 2/...`。

正常停止板端旧推理（在原终端Ctrl+C），然后准备唯一共用环境：

```bash
cd "/userdata/Final submission 2" && bash scripts/rk3568/PREPARE_ENVIRONMENT.sh
```

它从现有 `/home/linaro/yolov8_deploy/.venv-offline` 独立复制，不联网、不改旧环境。出现ENVIRONMENT_READY即成功。空间不足就停止，不自动删除。今后四种方式都不需要重复复制。要使用已迁移的RK环境，可增加 `--source /某个已验证的venv`。Linux系统库、RKNN Runtime和驱动仍需保留。

## 三、静止车辆决策演示

**用途：摄像头拍赛事视频，RK判断灯色/线型/行人，PC车辆只做灯光和前轮反馈，车身不移动。**

1. 停止其他板端推理。PDS给黑板PG2L50H在线下载 `hardware/PG2L50H/bitstreams/local_stereo_pg2l50h_20261025.sbit`；确认Program成功、done bit=1。红板不用连接或操作。位流没有改变时不用反复下载。
2. RK终端执行：

```bash
cd "/userdata/Final submission 2" && bash scripts/rk3568/INITIALIZE_PCIE.sh decision && bash scripts/rk3568/START_DECISION.sh
```

看到Mem+、BusMaster+和非零Region1后开始抓帧。终端保持运行，Ctrl+C才结束。
3. PC双击 `START_CARLA_ENGINE.cmd`。它是后台渲染，不一定出现游戏窗口；等待地图加载。
4. PC双击 `START_DECISION_CONSOLE.cmd`，自动进入决策页。左侧是RK处理视频，右侧前后两路静止车辆。若引擎后开，点决策页右上“连接/重连车辆”。
5. 摄像头对准正在播放的道路视频，尽量让视频充满画面。选择“路口信号灯”或“实虚线变道”、所属车道，再点击“应用场景/车道”。之后使用行驶、左/右变道、停止按钮。
6. 红灯阻停、红灯行驶请求报警；实线拒绝越线、虚线允许；行人绿色警告、黄色减速、红色紧急制动。报警由规则自动触发/结束，没有手动“确认报警”。对应刹车灯/单侧转向灯/前轮视觉转向响应，车身始终0速。正常状态不自动双闪。6秒车辆自检是独立按钮，不代表RK作出决策。

此模式不需要COM12、不需要导航报文，也不要点击顶部导航区的“连接CARLA与四相机”创建另一辆导航车。拍屏幕时双目只能实测屏幕平面距离，视频行人AEB是演示距离，不声称实测视频内人物距离。

## 四、两板级联物理四路视频

**用途：前/后FPGA、左/右USB进入RK，Front做双模型，其他三路只做目标检测。**

1. 黑板下载 `hardware/PG2L50H/bitstreams/cascade_fov_pg2l50h_20261029.sbit`；红板下载 `hardware/PGL50H/bitstreams/sfp_tx_camera_frame_fix_20261014.sbit`。两者不要下反，SFP和双目接口插牢。
2. 两个双目保持此前校色对应的位置，不能互换；否则校色会套错。右USB接xhci-hcd.1.auto，左USB接xhci-hcd.0.auto，别改插到同一个USB Hub。
3. RK终端执行：

```bash
cd "/userdata/Final submission 2" && bash scripts/rk3568/INITIALIZE_PCIE.sh four_camera && bash scripts/rk3568/START_FOUR_CAMERA.sh
```

4. PC双击 `START_FOUR_CAMERA_CONSOLE.cmd`，默认PCIe视频源，显示8083处理后的四宫格。若显示CARLA，点“切换视频源：PCIe/CARLA”回到PCIe，必要时点“重连视频”。
5. 本演示不需要打开CARLA或接树莓派。右侧等待座舱状态不代表左侧视频有故障。校色参数已经保存，正常启动不再输入RGB增益；固定class10=0.40。

## 五、CARLA座舱导航＋可选AEB

**用途：车辆依据Android目的地与导航报文移动，路线上段正常行驶、中段行人横穿，RK检出后制动、通过后续行、到达驻车。**

1. 此模式不用PCIe摄像头，不需更换位流。正常关闭其他PC总控台和RK推理。
2. PC双击 `START_CARLA_ENGINE.cmd`，再双击 `START_CARLA_NAVIGATION_CONSOLE.cmd`。
3. 点击总控台最上方“连接CARLA与四相机”。它会创建/连接自车、挂四路相机并启动推流器。看到视频后再继续。**不用另开独立推流脚本**，否则可能重复相机或占用8081。
4. 若要RK画框、分割或AEB，首次在RK编译一次私有MPP/RGA解码器：

```bash
cd "/userdata/Final submission 2" && bash scripts/rk3568/PREPARE_CARLA_DECODER.sh
```

5. 同一个RK终端启动并保持：

```bash
cd "/userdata/Final submission 2" && bash scripts/rk3568/START_CARLA_INFERENCE.sh
```

6. 树莓派串口接电脑后，在总控台右侧“虚舱串口”下拉框选择COM12/USB-SERIAL CH340，点“连接”，确认顶栏座舱链显示COM12。未出现时先点顶部“刷新座舱串口”；不要把FPGA下载线或虚拟TCP串口当树莓派。参数115200、8N1。随后在安卓座舱选择目的地并点“开始导航”。PC保留“座舱联调、感知单机、全域智驾”三个原模式。
7. **纯导航：** 选择“全域智驾”，取消顶部AEB勾选，按D挡/继续仿真。RK可接可不接，没接时显示PC原始CARLA视频。
8. **导航＋AEB：** 选择“全域智驾”，勾选AEB，确保RK已在线，再在安卓选点开始，按D挡/继续仿真。程序在路线中段自动找横穿点，不要手动点“生成前方行人”。观察等待进入横穿段→行人横穿→RK检出/制动→行人通过→自动续行→目的地P挡。
9. 人工暂停/P挡、模式或AEB切换后需要明确继续；碰撞/持续故障不能自动放行。若只收到“巡航待命”，安卓重新选点并开始导航，重发目的地事件。

CARLA中距离来自仿真行人真值，并以RK真实视觉检出为制动前提，不是双目测距。RK检测class10=0.30。新目录没有提高或重新标定推理FPS的承诺。

## 六、离线视频处理

不开CARLA、推流器或总控台也能处理。先Ctrl+C停止板端实时推理，再执行：

```bash
cd "/userdata/Final submission 2" && bash scripts/rk3568/START_OFFLINE.sh --video /userdata/videos/1.mp4
```

把1.mp4换成2.mp4即可处理另一段。省略 `--video` 就处理项目自带的44.97秒示例。默认逐帧处理到EOF，class10=0.60；可加 `--class10_conf 0.40`做对照，`--seconds 10`只看前10秒，`--max_width 0`保留原分辨率。两模型都看未绘制的同一帧，先检测再分割，结果为静音视频。

终端会打印OFFLINE_RESULTS。在SFTP里进入打印的目录下载整个run文件夹，打开result.mp4播放；保留result.summary.json和run.log。`complete=true`且stop_reason=source_eof才表示读完整段，空间不足/中断是部分结果，不能当完整成功。输出均在本项目results/rk3568/offline下，不写根分区/tmp。开始至少需1GiB空间，长视频可能更多。

## 七、停止、切换与再次打开

RK：原终端Ctrl+C，等待退出提示后再开另一方式。PC：点窗口右上角X正常关闭，再双击另一入口。CARLA引擎同一时间只保留一份。

Decision和物理四路切换必须换黑板位流，并重新初始化PCIe。红板只有物理四路需要。CARLA导航与离线不使用PCIe，不改位流。任何方式都不要同时运行两个RK模型程序，也不要同时开两个总控台。

当前老 `Final submission` 的运行不因归档而自动迁移；验证本项目时按上述新入口启动。先各做一次最小验收再考虑清理旧目录。原 `Final submission` 应作为本次整理前的保留版本，不由本项目自动删除。

## 八、补充工具与训练源码

下载离线结果后，可把result.mp4拖到 `scripts/windows/INSPECT_OFFLINE_RESULT.cmd` 检查帧数、时长和摘要。CARLA独立四相机/刹车检查入口、补齐的训练源码和仍缺材料见 [补充归档说明](SUPPLEMENT_20261031_zh.md)。日常演示不需要重训或重新转换模型。
