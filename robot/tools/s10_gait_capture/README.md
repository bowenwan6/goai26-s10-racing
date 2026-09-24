# S10 手机步态采集程序

与现有建图相同的使用方式：手机连接机器人热点，网页启动/停止采集，Orin 后台保存，结束后下载整轮 ZIP。此程序只订阅，不发布运动指令。运动仍由原厂控制器与遥控器负责。

当前真机已换为 **48 号、本队 golai**，见 [48 号安装记录] `git show docs-archive-20260920:docs/S10_48_SETUP_ZH.md` 与 [SLAM 使用说明] `git show docs-archive-20260920:docs/S10_48_SLAM_ZH.md`。`xwy` 是 50 号其他队伍账号，48 号不存在；下文 xwy、050 热点、旧建图配置和中转服务均为历史部署，不是当前入口。

2026-09-10 已按用户要求移除 50 号 AGX 独立采集站、自启服务和 **<CAPTURE_WIFI_SSID>** 热点，原 `http://10.42.50.1:8091/` 已下线。103 的临时 8091 中转目录也已清理；完整执行结果见 [DEPLOYMENT_050.md](DEPLOYMENT_050.md)。本地演示和源码继续保留。

2026-09-06 原 xwy 实例使用 `http://10.21.41.1:8090`，当时完成了实际录制、自动停止、热点下载、SHA-256 校验与离线解码。下文原实例部署部分保留为历史说明。后续完成历史数据审查、短时 MuJoCo 跟踪和地图预览，见 [研究交接] `git show docs-archive-20260920:docs/S10_DATA_RESEARCH_ZH.md`；没有完成原厂步态复刻或 RL 训练。

当前已收到 IMU、16 关节反馈、运动状态/步态、原厂轴指令与双雷达。实测 RL 控制状态下仍未收到 `/JOINTS_CMD`，因此尚不具备完整低层状态—动作克隆数据。先保存观测时需勾选“关键话题缺失时，仅录制诊断数据”；后续确认低层动作输出后再进行完整示范采集。不要把关节反馈当作教师动作。

## 队友接手：当前版本

- 手机连接 `<CAPTURE_WIFI_SSID>`，访问 `http://10.42.50.1:8091/`；热点和采集网页已配置开机自启，不依赖电脑，不自动录制。口令通过队内单独交接，Git 不提供口令。
- 当前网页采用浅色 B 版，支持前后雷达点云抽样预览、叠加及俯视/侧视。点云来自 AIRY 激光雷达；**深度相机的深度图、RGB 和 CameraInfo 尚未加入采集白名单**。
- 9 月 9 日新增 17 段录制（15.15 GB、约 20 分 52 秒）已校验转存至采集工作站 `D:/S10Data/050/2026-09-09/`；只删除了 AGX 独立目录中对应的这批数据。它们不在 Git，也不同于 9 月 6 日的 xwy 历史录制。领取时带上 `transfer-inventory.json` 和 `verified.json`。
- 已知问题：ROS 底层消息解析曾拖住网页，服务显示 active 仍可能无法访问。重启本采集服务后恢复，但底层原因未修复；操作前确认没有录制，详见 [部署与排障](DEPLOYMENT_050.md)。

新机器部署需要匹配的 ROS Jazzy、厂商消息包及网页依赖。`start-agx-session.sh` 和 `s10-capture-session.service` 固定了 50 号的用户和路径；不要原样复制到 48 号，也不要启动第二个相机/采集实例。当前未部署 48 号采集站；仅查看界面可使用下面的本地演示。

修改代码后的本地检查（不连接机器人）：

```sh
python tools/s10_gait_capture/test_snapshot.py
python tools/s10_gait_capture/test_point_preview.py
node tools/s10_gait_capture/test_point_overlay.cjs
```

## 本地体验

在项目根目录执行 `python tools/s10_gait_capture/run-demo-local.py`（需要 Flask），打开 http://127.0.0.1:8090 。访问口令位于 `artifacts/s10-capture-demo/.access-token`。程序生成高强度随机口令，口令不放在网页地址中；该目录不提交 Git。

本地默认明确启用 `--demo`；合成记录只验证交互与保存流程，全部 `training_ready=false`。Windows 没有 ROS 时，真实模式拒绝启动录制，不会退回合成数据。

手动真实模式入口（已具备 ROS 2 的环境）：

```bash
python server.py --host 127.0.0.1 --port 8090 --output ~/s10_gait_data
```

## 现场操作

1. 选择基础、台阶、高台或碎石，填写地形尺寸、教师/固件版本、速度档位和其他条件。
2. 核对 IMU、关节反馈、动作输出的接收状态；关键数据缺失时只允许显式选择诊断采集。
3. 点击开始；用原有遥控器完成运动。可以标记进入/离开障碍、打滑、接管和失败。
4. 选择本轮成功、失败、接管或待复核，点击停止并保存。
5. 下载 ZIP，后续在工作站进行数据审查、模仿学习与强化学习。

手机关闭页面不结束采集。默认录制 120 秒，最多 1800 秒；超时自动保存。低于 512 MB 禁止开始，录制时低于 256 MB 结束并记录原因。停止录制不是机器人急停。后台异常重启会把未完成会话标为 interrupted；不会自动继续录制。异常断电后的 rosbag 可能需要 `ros2 bag reindex`，不能把残缺包作为完整轨迹。

## 原始数据与限制

### 自动读取当前步态

《S10软件开发指南202607》50–51 页的 2.2.3 节明确规定：订阅 `/MOTION_INFO`（`drdds/msg/MotionInfo`，标称 20 Hz）读取当前运动状态与步态。ROS 表中的 gait 编号为 `0x1001` 基础（标准）、`0x3002` 平地（敏捷）、`0x3003` 楼梯（敏捷）。程序直接解码反馈，无需从 IMU 猜测步态。`/GAIT` 在 49 页定义为切换命令，不能代替实际反馈。

部署实测中收到 `state=17, gait=4099 (0x1003)`，结合指南第 11 页标准楼梯模式定义，已补充显示“楼梯（标准运动模式）”。实际厂商字段路径为 `data.motion_state.state` 和 `data.gait_state.gait`。

页面同时显示运动状态与当前步态。超过 0.5 秒没有收到反馈时，后端返回未知；页面每 2 秒刷新，不能作为实时控制指示灯。录制起点反馈和每次反馈步态/运动状态变化自动写入 manifest，全部原始运动反馈仍在 rosbag 中。人工选择的地形保持独立：同一基础步态也可能走在碎石上。

指南 20 页状态协议表另列 `0x1002` 高台，但 ROS 51 页没有列出；程序保留原码并提示待实机确认，不擅自沿用另一接口的完整枚举。指南没有给出碎石独立步态编号。未知编号原样保留。指南消息结构示意中多次复用 `data` 字段名，程序从已安装的真实消息类型中查找唯一 `gait`/`state` 数值叶字段并记录实际路径；缺失、歧义或非法值显示未知。本地库缺少厂商 MotionInfo 定义，部署时仍需实际 ROS 类型包。

这些反馈给出了原厂报告的步态模式，不暴露策略权重或关节动作；步态标签不能替代复刻所需的状态—动作数据。

实际模式保存 rosbag2 SQLite3/CDR 原始消息，不把手机刷新频率作为采样频率；时间戳取 ROS 接收时钟，源时间戳保留在消息内。网页每两秒读取健康信息。IMU、关节、动作、运动信息、步态、速度指令、SLAM 里程计、定位状态、双雷达、TF 均在白名单内，按实际发现的类型订阅。原始类型需要匹配机器人固件。

ZIP 内有 `manifest.json`、`bag/`、`SHA256SUMS.json`。演示包使用 `synthetic.jsonl`，没有机器人数据。manifest 包含地形类别/尺寸、教师版本、结果、事件、接收计数、缺少源时间戳数量、源时间戳非递增数量、最大接收间隔。

在白名单中不等于已录到数据。9 月 9 日这 17 段均有 IMU、关节高/低频反馈、运动状态和前后雷达；16 段有 STEER/HANDLE_STEER，9 段有 GAIT。均没有 JOINTS_CMD、里程计、定位状态、TF 或相机数据。完整性校验通过仅证明文件传输和数据库可读，时间同步、丢帧、动作语义和训练适用性仍需逐段分析。

事件时间为后台收到手机请求的时间，不代表准确触地时刻。频率来自接收窗口；网络丢失、QoS、磁盘写入速度均需机器人实测。源时间戳统计不等于跨板同步验收。高带宽数据写入可能降低接收率，部署后须测吞吐与最大间隔。

`/JOINTS_CMD` 名称与在线状态不能证明其为当前原厂教师输出；必须核实发布者、16 个关节顺序、位置/速度/力矩控制语义、增益与单位。`/cmd_vel` 也不能未经核实当作遥控器输入。实际关节位置不能冒充期望关节动作。缺失这些信息时，数据只适合诊断或轨迹模仿研究，不能声称直接复刻了教师策略。

所有原始包 `training_ready=false`。训练前需单独产出审查记录，验证有效定位、动作来源、时间同步、观测/动作转换、机器人几何和控制栈版本。开发中的训练导出器尚未实现；此版本不提供虚假的“一键训练”按钮。

## 原 xwy 实例部署与维护（历史）

新 50 号独立采集站使用 [DEPLOYMENT_050.md](DEPLOYMENT_050.md)，不要用以下旧路径覆盖新实例。

采用独立端口复用建图方法，保留建图 8080：

```text
手机 http://10.21.41.1:8090
  → 103 运动板 SSH 转发
  → Orin xwy 的 127.0.0.1:8090
  → 本机 ROS 2 只读订阅
  → /home/xwy/s10_gait_data/<会话>/
```

将本目录上传至 Orin `/home/xwy/s10_gait_capture`，以 xwy 执行：

```bash
cd ~/s10_gait_capture
python3 -m venv --system-site-packages .venv
.venv/bin/pip install -r requirements.txt
mkdir -p ~/.config/systemd/user ~/.config/s10-gait-capture
cp s10-gait-capture.service ~/.config/systemd/user/
```

系统需要 `rclpy`、`rosbag2_py`、SQLite3 rosbag 存储插件及匹配厂商固件的 drdds 类型。本次通过已验证的 SSH 从 106 定位板读取 19 份实际厂商消息定义，在 Orin `~/s10_gait_capture/vendor_ws` 独立编译；来源和文件哈希记录于 `vendor_ws/VENDOR_SOURCE.json`。`prepare_vendor_overlay.py` 可复现这一步。没有覆盖系统或建图的消息包。

`~/.config/s10-gait-capture/environment` 已配置 `S10_GAIT_ROS_OVERLAY=/home/xwy/s10_gait_capture/vendor_ws/install/setup.bash`、ROS_DOMAIN_ID=0、RMW_IMPLEMENTATION=rmw_fastrtps_cpp。采集虚拟环境复用本机已有网页依赖，不需要机器人接外网。`start-robot.sh` 读取已有建图配置中的密码哈希；不保存额外明文密码。

```bash
systemctl --user daemon-reload
systemctl --user enable --now s10-gait-capture.service
systemctl --user status s10-gait-capture.service
```

热点中转已使用附带的 `s10-gait-hotspot.service`。103 上创建了独立 `s10_gait_relay` 密钥，对应公钥仅允许 `127.0.0.1:8090` 转发、限制来源 `10.21.33.103`、禁止 shell。现有建图密钥与服务保持原配置。103 中转服务和 Orin 后台均已启用，后台约 0.43 秒正常完成停止/重启；修复了 rclpy 默认信号处理妨碍网页进程退出的问题。

最后一轮 12 秒验收：IMU 2403 条、关节反馈 2404 条、运动反馈 240 条、两路原厂轴指令各 120 条、双雷达各 119 帧。ZIP 约 194 MB，三个文件的 SHA-256 全部通过。雷达接收曾出现约 0.90/1.00 秒间隔；原始源时间戳保留，后续同步与数据质量审查不能只看平均频率。此处证明录制与下载成功，不代表全部传感器实时性或训练质量已通过。采集节点发布者仅有 ROS 日志和参数事件，没有运动命令发布者。

部署验收：手机登录 → 实际话题健康 → 短时诊断录制 → 关页重连 → 标注与停止 → 下载校验 → `ros2 bag info` 与消息回放核对 → 检查命令话题发布者数没有因采集器增加。最后才进行真实运动示范采集。

## 复刻与训练路线

已有工程有冻结的 57 维观测基础 ONNX 和 STAIR2/MANTLE/Recovery 管线。先区分“本地可查询 ONNX”与“只能在实机运行的原厂黑盒”。前者可在仿真查询教师动作并蒸馏；后者需先取得同步的状态、指令、教师动作。实机记录不能自动恢复原始权重。

建议先训基础学生，再按台阶、高台、碎石建立专项课程并分别评估；高台允许独立技能，基础/碎石也可共享条件化策略。不要仅凭数据标签就固定训练四个互不关联的模型。保留成功、失败和接管数据，但失败动作不能无差别作为行为克隆的正样本。

流程：原始数据审查与整轮划分 → 行为克隆/教师蒸馏 → 仿真闭环验证 → 分地形 PPO 或有界残差 RL → 未见地形评估 → 实机逐级验证。PPO 需要在仿真中持续产生 rollout，不能把录制文件直接当作 PPO 环境。若教师可查询，可用 DAgger 补充学生偏离分布时的教师标签。可移动碎石的接触动力学不能仅靠静态高程图代表。

参考：[DAgger 算法说明](https://imitation.readthedocs.io/en/latest/algorithms/dagger.html)、[NVIDIA Isaac Lab](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/isaac_lab_tutorials/index.html)。
