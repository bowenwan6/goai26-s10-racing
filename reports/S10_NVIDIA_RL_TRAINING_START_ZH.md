# S10-048：NVIDIA GPU 走路／转弯 RL 训练启动与交接指南

更新：2026-09-17。面向当前这台 Mac、本队 048 号 S10 Pro，以及准备接入的 NVIDIA GPU 训练服务器。

本文区分三种状态：“已核实”是本地文件或实机只读检查支持的事实；“建议”是下一步工程方案；“待实现／待确认”不能当成已经能运行的功能。本文不包含密码、私钥或运动启动指令。本次只整理文档、读取文件和检查接口，没有启动训练、切换机器人模式或让机器人运动。

## 1. 先读结论：我们现在能开始什么

我们已有机器人 URDF／MJCF／mesh、官方 57D SDK actor、此前 speedturn 57D actor、MuJoCo PPO／残差／蒸馏实现参考，以及实机部署和数据采集基础。**可以开始搭建 NVIDIA GPU 环境和实现 S10 平地速度跟踪任务，但目前没有一条已经验证可直接启动的“S10 GPU 快速行走／转弯训练”命令。**

推荐继续采用同一条路线：

> 冻结官方 57D actor → PPO 训练带特权信息的残差 teacher → 蒸馏成只用实机可观测信息的 GRU residual student → 将官方 actor＋student 合并导出为一个 ONNX。

遥控器驱动的原生控制器可以提供实机专家示范，作为辅助数据；**不需要拿到其权重，也不需要等示范数据齐全才开始仿真训练。** 现有遥控数据大多只有状态，不能直接冒充完整的 state-action BC 数据。

第一阶段只做平地的前进、停止、倒退、原地转向和行进转弯，不混入 Gate16 高台任务。S10 是轮足机器人，“走路／跑步”在这里允许轮驱＋关节姿态协同，不应直接照搬纯四足的跑步步态奖励。

当前最重要的缺口：

| 项目 | 状态 | 下一步 |
|---|---|---|
| GPU 服务器地址、账号、GPU 型号 | 未提供；未连接服务器 | 提供 SSH 别名／地址、账号和型号，不需要密码或私钥 |
| 可并行运行的 Isaac Lab S10 locomotion task | 待实现 | 导入机器人，写 mixed leg-position／wheel-velocity actuator 和 57D observation |
| 官方 actor 在 Isaac 中的物理匹配 | 未验收 | 先冻结 actor 做动态回归，再开始 PPO |
| 原生遥控策略的实际关节命令示范 | 话题可见，但本次未采到动态示范 | 现场人工安全遥控，同时只读录制 |
| speedturn 原训练仓库／model_2000.pt | 此 Mac 未找到 | 向原训练工作站取回；不能把元数据路径当成已有文件 |
| 新 GRU ONNX 对应的 C++ runner | 待实现 | 增加 hidden 输入／输出、状态复位和自动测试 |

最快开始顺序：确认 RTX 服务器 → 第 7 节环境安装 → 第 8 节材料复制 → 第 9 节实现任务／冻结基线验证 → PPO → student／导出。不要先跑现有 Gate16 trainer 然后把它称作快速行走训练。

## 2. 哪台机器做什么

```text
Mac（当前本机）
  ├─ SSH → GPU Linux 服务器：Isaac Lab + PhysX GPU + PyTorch / PPO / 蒸馏
  │                            训练期间不需要连接真机 DDS
  └─ SSH → 048 AGX 10.21.33.102：文件、ROS 观测、数据录制、最终 SDK 部署
                                ├─ 103 机身控制器：原生官方运控
                                └─ 106 定位板：SLAM / ODOM / 地图

Android 遥控器 → 模式／摇杆指令 → 103 原生控制器 → 电机
                                       ↓
                              状态／可见关节命令 → 专家示范数据
```

不要在 Mac 的 Apple GPU 上尝试运行这套 NVIDIA Isaac Sim 训练栈；Mac 用于编辑、SSH 和离线分析。AGX 有 NVIDIA GPU，但当前是 ARM 实机计算平台，不是本文 x86_64 RTX 训练环境；不能直接照抄下面的安装命令到机器人上。

### 2.1 当前机器人信息

| 项目 | 已核实信息／边界 |
|---|---|
| 机器人 | 本队 048 号，山猫 S10 Pro；原生运控日志型号 CD1 |
| AGX 地址／账号 | 10.21.33.102，golai；ARM64／aarch64 |
| AGX 软件 | Ubuntu 24.04.4、ROS 2 Jazzy、Fast DDS；SDK 随附 ARM ONNX Runtime 1.22.0 |
| 机身运控 | 10.21.33.103；原生 motion_master 加载 CD1_policy |
| 定位板 | 10.21.33.106；已有厂商 SLAM／规划栈 |
| 当前地图 | 0914_fr_v3-20260914-142008；有地图不等于当前全局定位有效 |
| 结构 | 4 条腿，每腿 hipx、hipy、knee、wheel，共 16 个执行器 |
| SDK 控制类型 | 12 个腿关节位置目标＋4 个轮关节速度目标，不是 16 个位置目标 |
| 传感器／状态 | IMU、关节状态、ODOM；已部署双 Airy 雷达、深度相机 |
| 当前 base policy 输入 | 57D 本体感知，没有点云／高度图、真实线速度或绝对航向 |
| SDK policy 频率 | agent_timestep = 0.02，即 50 Hz；代码 SetDecimation(4)，底层循环时序需实测核对 |

机器人型号与配置参考：[048 安装记录](<workspace>/s10-real-readiness/docs/S10_48_SETUP_ZH.md)、[048 SLAM 记录](<workspace>/s10-real-readiness/docs/S10_48_SLAM_ZH.md)、[S10 产品手册](<<workspace>/s10-real-readiness/docs/reference/山猫S10 Pro产品手册 V1.0.1.pdf>)。

### 2.2 物理／控制参数：先作为建模基准，不当成安全认证

| 参数 | 当前来源与值 |
|---|---|
| 轮半径 | 现有 MuJoCo 代码 WHEEL_RADIUS = 0.081 m；需与实轮尺寸／滚动标定核对 |
| 腿关节 effort／velocity limit | 官方 URDF：50 Nm／25.76 rad/s |
| 轮关节 effort／velocity limit | 官方 URDF：14 Nm／65.50 rad/s |
| hipx 位置范围 | URDF：约 ±0.6109 rad |
| hipy 位置范围 | URDF：约 ±2.5307 rad |
| knee 位置范围 | URDF：约 ±2.7227 rad |
| 腿 SDK 增益 | kp = 80，kd = 2 |
| 轮 SDK 增益 | kp = 0，kd = 0.6 |
| SDK 前馈力矩 | 当前 runner 为 0 |
| 默认关节角 | 前两腿 [0, -0.3, 0.6]，后两腿 [0, 0.3, -0.6]；单位 rad |
| 产品速度边界 | 产品手册标称最高运行速度 5 m/s；不是新策略可安全持续达到的证明 |

轮速上限乘轮半径得到约 5.31 m/s，只是无滑移运动学估算；真实最高安全速度还受力矩－转速曲线、供电、温升、附着、载荷和转弯侧向加速度限制。不要把 URDF effort 当成全转速下可持续输出的额定力矩。

实机质量、附加载荷、质心偏移、关节零偏、轮胎摩擦和 motor delay 尚需测量。URDF 惯量用于初始建模，不能代替当前装载机器人的辨识。

## 3. 我们已经有哪些资料可以利用

下面是当前 Mac 上的实际位置。复制到 GPU 后，应在配置中改用 GPU 路径；这些 Mac 绝对路径不会在服务器上自动存在。

### 3.1 优先使用的代码仓库

| 资料 | 当前本地地址 | 用途与限制 |
|---|---|---|
| 完整比赛仿真／部署工程 | [goai26-s10-racing](<workspace-legacy>/goai26-s10-racing) | 机器人资产、SDK、ONNX、ROS 部署、回归测试；其 training 目录不是完整 PPO trainer |
| 完整旧训练工程的干净副本 | [goai-s10-gate16-policy-v1-5](<workspace-legacy>/goai-s10-gate16-policy-v1-5) | 有 MuJoCo PPO、teacher warmstart、残差训练、DAgger；是高台任务参考，需改造成新平地 GPU task |
| 论文／技术报告源码证据包 | [source_bundle](<workspace>/academic_assets/source_bundle) | 固定发布版本的代码选集、模型谱系、评测证据；不是完整可执行仓库 |
| 真机准备／数据调查工程 | [s10-real-readiness](<workspace>/s10-real-readiness) | SSH、部署监控、安全试验、SLAM／数据文档；注意旧 050／051 号记录不等于当前 048 |

Git 来源与本次版本：

- 仿真／部署：[bowenwan6/goai26-s10-racing](https://github.com/bowenwan6/goai26-s10-racing)，HEAD 9566ab9bf7eaf39af3d1ffd904965d597f23b20a。工作区 docs/TECHNICAL_DESIGN.md 有用户未提交修改，应保留。
- 旧训练：[belsun/goai-s10-gate16-policy](https://github.com/belsun/goai-s10-gate16-policy)，上述 v1-5 本地副本 HEAD 0832f7bcd93eec6e51976c7c7e07913e92479f8c，核查时干净。
- 官方资产来源：[DeepRoboticsLab/goai_embodied_future_material](https://github.com/DeepRoboticsLab/goai_embodied_future_material)，已有完整本地 upstream，优先复制本地版本，不靠“latest”重下载替换。
- source_bundle 的训练选集来自 5ef14fa，部署选集来自 3660b81；保留了原 training/ 前缀，所以部分路径连续出现两次 training。

### 3.2 建环境必需的文件

官方 SDK 根目录：

```text
<workspace-legacy>/goai26-s10-racing/
  upstream/goai_embodied_future_material/src/S10_sdk_deploy/
```

| 文件 | 本机地址 | 为什么要用 |
|---|---|---|
| S10 URDF | [S10.urdf](<workspace-legacy>/goai26-s10-racing/upstream/goai_embodied_future_material/src/S10_sdk_deploy/S10_description/s10_mjcf/urdf/S10.urdf) | 转为 Isaac USD；关节、轴、惯量、限位 |
| S10 MuJoCo 模型 | [S10.xml](<workspace-legacy>/goai26-s10-racing/upstream/goai_embodied_future_material/src/S10_sdk_deploy/S10_description/s10_mjcf/mjcf/S10.xml) | 跨仿真器动态验证，参考 actuator／接触 |
| 全部机器人资源 | [S10_description](<workspace-legacy>/goai26-s10-racing/upstream/goai_embodied_future_material/src/S10_sdk_deploy/S10_description) | URDF/XML 的 mesh 依赖；必须整目录保留相对结构 |
| 真实 observation／action 规则 | [s10_policy_runner.hpp](<workspace-legacy>/goai26-s10-racing/upstream/goai_embodied_future_material/src/S10_sdk_deploy/run_policy/s10_policy_runner.hpp) | 网络名称、维数、关节顺序、缩放、PD 目标 |
| Python 规则 | [observation.py](<workspace-legacy>/goai26-s10-racing/training/s10_rl/observation.py) | BASELINE 57D；PERCEPTIVE 174D，不可混用 |
| DDS 接口 | [dds_interface.hpp](<workspace-legacy>/goai26-s10-racing/upstream/goai_embodied_future_material/src/S10_sdk_deploy/interface/robot/hardware/dds_interface.hpp) | 实机状态到 SDK 的适配 |
| SDK 消息定义 | [drdds/msg](<workspace-legacy>/goai26-s10-racing/upstream/goai_embodied_future_material/src/drdds/msg) | 离线解析参考；原生接口以机上实际安装定义为准 |

旧训练工程中可复用的实现：

- [train.py](<workspace-legacy>/goai-s10-gate16-policy-v1-5/training/mujoco_s10/train.py)：PPO、ONNX teacher 数值核对、anchor／BC loss。
- [network.py](<workspace-legacy>/goai-s10-gate16-policy-v1-5/training/mujoco_s10/network.py)：actor／critic 结构参考。
- [env.py](<workspace-legacy>/goai-s10-gate16-policy-v1-5/training/mujoco_s10/env.py)：混合位置／速度控制、力矩裁剪和轮接触参考。
- [official_policy_env.py](<workspace-legacy>/goai-s10-gate16-policy-v1-5/training/mujoco_s10/official_policy_env.py)、[official_residual_env.py](<workspace-legacy>/goai-s10-gate16-policy-v1-5/training/mujoco_s10/official_residual_env.py)：冻结 base＋residual 的实现思路；原来是 174D／障碍任务，不是新任务即插即用组件。
- [distill_residual_dagger.py](<workspace-legacy>/goai-s10-gate16-policy-v1-5/training/mujoco_s10/distill_residual_dagger.py)：student rollout → teacher 标签 → dataset aggregation 参考。
- [training/s10_rl](<workspace-legacy>/goai-s10-gate16-policy-v1-5/training/s10_rl)：checkpoint／导出／输入规范工具。

特别注意：[warmstart.py](<workspace-legacy>/goai-s10-gate16-policy-v1-5/training/s10_rl/warmstart.py) 明确要求 RSL-RL 5 的 actor_state_dict／mlp checkpoint 格式；本文固定 Isaac Lab v2.3.2 使用 RSL-RL 3.1.2。必须写 checkpoint adapter 或从 ONNX 恢复冻结 actor，不能直接互换，也不要为了旧脚本把新环境强行升级到 RSL-RL 5。

### 3.3 当前模型：三种“官方／旧模型”不是一回事

| 模型 | 位置与可用性 | 用途 |
|---|---|---|
| 本地公开 SDK 57D actor | [policy.onnx](<workspace-legacy>/goai26-s10-racing/upstream/goai_embodied_future_material/src/S10_sdk_deploy/policy/policy.onnx) | 现在就能离线验证；固定基线／冻结 base 候选 |
| AGX 公开 SDK 57D actor | 机器人 /home/golai/goai_embodied_future_material/src/S10_sdk_deploy/policy/policy.onnx；2026-09-17 SSH 核实 | 更贴近当前机上 SDK 文件，但是否对应最佳实机表现仍要测试 |
| 之前训练的 speedturn 57D actor | 机器人 /home/golai/provisioning/speedturn2000-20260912/s10_general_speedturn_57d_model2000.onnx；SSH 核实 | 强对照基线，先做回归；不是“已经比原生遥控更好”的证据 |
| 103 原生遥控运控 | 日志加载 /opt/robot/share/motion_master/run_policy/lib/arm/CD1/libCD1RunPolicy.so | 黑盒实机专家；未取得可用于训练的独立权重 |
| Gate16 感知策略／残差 | [policy/gate16](<workspace-legacy>/goai26-s10-racing/policy/gate16) | 障碍专项，不能当作 57D locomotion base |
| Gate16 已有 checkpoint | [speed_core_best_checkpoint.pt](<workspace-legacy>/gate16_front_tuck_competition_v4_20260818/checkpoint/speed_core_best_checkpoint.pt) | 174D 模型谱系／训练工具参考，不是 speedturn model_2000.pt |

已核对 SHA-256：

```text
本地 SDK policy.onnx
0ac99f3093d4a984d7587b88d57300cbf7ec2f788401dfa1570d1e4800568f6b

AGX SDK policy.onnx
92db62c118c4ebad3da8bfedb89691f8caae8803966b472b4ce541e1a32f2d0d

AGX speedturn 57D model2000
f9a789b5362ebe383da35c6669d5ad1680ceb0997de91f8491b30d9957e271b0

Gate16 policy.onnx
5c1b388f951b282693b4497cd4fd2fd1b53fe1bcd989758c0af42f140a20fb16

Gate16 climb_residual.onnx
de61441facb21f0301787b26f168f8447fdc0d83c337eeea884537bbf2468889
```

本地 SDK 与 AGX SDK 哈希不同：保存为两个不同文件，不覆盖；分别用各自 observation／command contract 验证。先比较两者和 speedturn 的冻结动态表现，再锁定唯一 base。默认优先评估当前 AGX SDK 版本，而不是仅凭文件名选择。

speedturn 的元数据在机上：

```text
/home/golai/provisioning/speedturn2000-20260912/model-metadata.json
```

它指向历史训练 checkpoint：

```text
/workspace/rl_training/logs/rsl_rl/deeprobotics_s10_general_57d/
2026-08-19_09-12-07_speedturn_from1400_v1/model_2000.pt
```

这是**原训练机器上的历史地址，不是当前 Mac／AGX 上已经找到的 checkpoint**。原 rl_training 源码与该 checkpoint 若能取回，能帮助恢复原 reward、随机化和 command 范围；仅有 ONNX 也能冻结推理或恢复可识别的 MLP 权重，但不能恢复 optimizer／critic／训练配置。

### 3.4 数据／地形：哪些是真正的训练 input

| 资料 | 当前地址／状态 | 可以做什么 | 不能做什么 |
|---|---|---|---|
| 今日 048 LiDAR＋IMU＋ODOM | [slam_test_20260917_170443](<workspace>/recordings/slam_test_20260917_170443)；全量本地存在 | 时钟／IMU／定位接口分析；速度／路径指标辅助验证 | 没有 JOINTS_DATA／JOINTS_CMD／STEER，不能生成可靠 BC action labels |
| 今日数据压缩包 | [S10_048_LiDAR_IMU_20260917_170443.zip](<workspace>/deliverables/S10_048_LiDAR_IMU_20260917_170443.zip) | 交接给 SLAM 开发者 | 首次平地 RL 不必上传大体积点云 |
| v3 地图／MuJoCo 包 | [S10_v3_Map_MuJoCo_20260916](<workspace>/deliverables/S10_v3_Map_MuJoCo_20260916) | 后续跨仿真器／小地形留出测试 | 不是全场碰撞 mesh，也不带可直接行走的 controller |
| 历史 9 段状态包／17 段遥控包 | [数据调查文档](<workspace>/s10-real-readiness/docs/S10_DATA_RESEARCH_ZH.md) 中有清单；原始大包未在此 Mac 完整找到 | 取回后分析关节轨迹、侧倾、轮速、状态分布 | 缺动作与可靠定位时，不直接作 state-action BC；050 历史结果不能替代 048 |
| 原生运控调查证据 | [native-navigation-audit-20260917](<workspace>/s10-real-readiness/artifacts/native-navigation-audit-20260917) | 控制器位置／模式／接口／授权读取边界 | 发现接口不代表实际收到动态 action，更不代表已完成专家采集 |

今日 bag 时长 177.824 s，共 39,121 条：点云 1,778 条约 10 Hz，IMU 35,566 条约 200 Hz，ODOM 1,777 条约 10 Hz。raw_bag/metadata.yaml 与 8 个 MCAP 分卷属于同一录制，必须一起保留。没有 TF，IMU frame_id 为空；用 ODOM 做监督前仍需核对 frame、外参和定位有效性。[完整说明](<workspace>/recordings/slam_test_20260917_170443/README_先读我.md)。

地图包有全场 3,346,032 点的 full_cloud.pcd，但可运行碰撞场景只覆盖 Start＋B＋B 后约 4.88 m：

- [full_cloud.pcd](<workspace>/deliverables/S10_v3_Map_MuJoCo_20260916/maps/v3/full_cloud.pcd)
- [terrain/scene_contact_v1.xml](<workspace>/deliverables/S10_v3_Map_MuJoCo_20260916/mujoco/terrain/scene_contact_v1.xml)
- [robot_scene/scene.xml](<workspace>/deliverables/S10_v3_Map_MuJoCo_20260916/mujoco/robot_scene/scene.xml)

trajectory.csv 是优化关键帧，不是导航路点。优先在简单 plane 上训练；这些复杂场景放到后续鲁棒性测试。

## 4. 训练／部署究竟输入什么、输出什么

### 4.1 三个层面的 I/O

| 层面 | Input | Output |
|---|---|---|
| 训练任务 | 机器人资产、冻结 base、command 采样、物理随机化、reward；可选专家数据 | teacher checkpoint、rollout、评测／训练日志 |
| 最终 student 网络每步 | 当前 57D obs＋GRU hidden_in | 16D 最终组合 action＋hidden_out |
| 真机电机接口 | action 经顺序变换／缩放，配 kp／kd／tau_ff | 12 个腿 q_des＋4 个轮 dq_des；不是直接输出机身速度或导航路线 |

用户给 policy 的高层指令为机身坐标系 command = [vx, vy, yaw_rate]，单位建议统一为 [m/s, m/s, rad/s]。它是“希望怎样移动”，不是“已经移动多快”。实际速度用于仿真 reward／teacher／评测。

### 4.2 官方 base 的 57D observation 合同

切片采用 Python 左闭右开，float32：

| 切片 | 维数 | 具体内容 |
|---|---:|---|
| [0:3] | 3 | body frame 三轴角速度 × 0.25 |
| [3:6] | 3 | projected_gravity = R_world_from_body 的逆旋转 × [0,0,-1]；是单位重力方向，不乘 9.81 |
| [6:9] | 3 | command：vx、vy、yaw_rate；缩放必须先与所选 runner 统一 |
| [9:25] | 16 | policy order 关节位置减默认角；4 个轮位置先置 0，不累计轮转角 |
| [25:41] | 16 | policy order 关节速度 × 0.05 |
| [41:57] | 16 | 上一帧**组合网络 action**，不是上一帧 residual，不是电机 q_des |

这个合同没有 base linear velocity、LiDAR、heightmap、绝对 yaw 或定位坐标。不能把 174D Gate16 输入截掉后就假设其网络仍可用。

IMU quaternion 顺序、角速度单位和 joint sign 必须通过实机适配和测试确认，不能只按 ROS 字段名猜。Isaac quaternion 表示也需显式转换。

**已发现的 command 不一致：**

- 当前 Mac 的 runner 直接构造 [uc.forward_vel_scale, uc.side_vel_scale, uc.turnning_vel_scale]。
- 本次 SSH 检查的 AGX runner 构造 [forward × 1.5, side × 0.5, turn × 0.6]。
- 原生 /STEER 可能是归一化摇杆；原生 /NAV_CMD 在导航模式下则是物理速度接口。它们不能直接混在同一字段。

建议新训练／部署 contract 明确规定 [6:9] 为物理 command，并在进入网络前只有一个统一转换层。给旧 actor 供值时，先确认它原来训练／部署的 command 语义；必要时为冻结 actor 保留独立 adapter。复制 ONNX 不会自动修复 command 语义。

**启动／历史状态的另一个不一致：** AGX 旧 runner 启动前 150 帧有 action／增益 ramp，last_action 在 ramp 之前保存；Mac runner 有 blending 后更新 history 的实现。新环境必须明确保存的是组合网络 action 还是经过额外 ramp 的动作，并与选定 C++ runner 完全一致。不要训练一种 history、部署另一种。

### 4.3 两套关节顺序

```text
ROBOT / DDS 顺序（16）：
fl_hipx, fl_hipy, fl_knee, fl_wheel,
fr_hipx, fr_hipy, fr_knee, fr_wheel,
hl_hipx, hl_hipy, hl_knee, hl_wheel,
hr_hipx, hr_hipy, hr_knee, hr_wheel

POLICY 顺序（16）：
fl_hipx, fl_hipy, fl_knee,
fr_hipx, fr_hipy, fr_knee,
hl_hipx, hl_hipy, hl_knee,
hr_hipx, hr_hipy, hr_knee,
fl_wheel, fr_wheel, hl_wheel, hr_wheel

实际 joint name 以官方资产的 *_joint 后缀为准。
Isaac 导入后的内部关节顺序可能不同，要按名字构造映射，不能凭 index 假设。
```

网络 action = a，按 POLICY 顺序：

```text
每条腿：
q_des_hipx = q_default_hipx + 0.125 * a_hipx
q_des_hipy = q_default_hipy + 0.250 * a_hipy
q_des_knee = q_default_knee + 0.250 * a_knee

每个轮：
dq_des_wheel = 5.0 * a_wheel

参考 actuator：
tau_leg   = 80 * (q_des - q) + 2 * (0 - dq)
tau_wheel = 0.6 * (dq_des - dq)
再应用实际电机约束／力矩限位。
```

这里的 a 不是天然限制在 [-1,1] 的物理量。若给所有输出套 tanh，轮速度目标最大只剩 5 rad/s，按 0.081 m 约 0.405 m/s，会直接限制提速。残差范围、最终动作限幅、轮速度上限必须分别设计，不盲目复用纯四足网络的 action clip。

### 4.4 teacher 与 student 输入不同，但最终部署仍明确可测

- 冻结 base 始终只接收同一个 57D prefix。
- 仿真 residual teacher actor 可额外用真实 body linear velocity、接触、摩擦／载荷／delay 等 privileged information；critic 也可使用。这只是训练期信息。
- student 使用 57D＋短期记忆，不能在部署时暗中依赖仿真真值、摩擦系数或未来 command。
- 初始 GRU 建议 1 层、hidden size 128；这是设计起点，不是现有模型配置。
- 训练／蒸馏处理 episode mask；起身、进入 RL、策略切换、趴下、急停／失联后按明确规则复位 hidden 与 action history。

最终建议 ONNX 合同：

```text
obs        float32 [batch, 57]
hidden_in  float32 [1, batch, 128]
  ↓ 冻结 base + GRU residual + 已定义的动作组合／限幅
actions    float32 [batch, 16]
hidden_out float32 [1, batch, 128]
```

当前 SDK 只有 obs [1,57] → actions [1,16]，**不会自动接受这个新合同**。需要修改 runner 的 input_names／output_names、tensor shape、hidden 更新／复位和错误处理。第一个 teacher smoke test 可先不接 GRU，最终发布再完成 recurrent student。

## 5. 从当前 Mac SSH 到机器人

### 5.1 首选：已经配置并实测可用的远程别名

在 Mac Terminal 或本机终端：

```bash
ssh s10-48-remote
```

2026-09-17 本次只读登录成功。进入后是在机器人 AGX 的 golai shell；输入 exit 回到 Mac。

配置位置：

- Mac SSH 主配置：[config](~/.ssh/config)
- 已包含的机器人配置：[s10-48-remote.conf](~/.ssh/s10-48-remote.conf)
- 此别名使用当前本机配置的 userspace Tailscale ProxyCommand；不需要把机器人端口开放到公网。

别名与后台连接服务只保证当前 Mac 的配置，不会自动同步到新电脑。本文不给出任何私钥内容，也不要把 .ssh 整目录上传 GPU 或公开仓库。Windows 历史资料中的 s10-48-golai 并不是当前 Mac 已配置的同名别名。

只执行一个远程只读命令，无需先进入 shell：

```bash
ssh -o BatchMode=yes -o ConnectTimeout=8 s10-48-remote 'hostname; uname -m; date'
```

### 5.2 现场直连机器人网络

当前 Mac 配置有 golai-server → golai@10.21.33.102。Mac 必须连到机器人热点或可路由到该地址的网络；只连普通办公 Wi-Fi 时通常不可达。本次直连超时，但远程别名可用。

```bash
ssh -o StrictHostKeyChecking=yes \
    -o HostKeyAlias=10.21.33.102 \
    -o HostKeyAlgorithms=ssh-ed25519 \
    golai-server
```

检查 Mac 到机器人地址的路由：

```bash
route -n get 10.21.33.102
```

连接超时先查网络／机器人开机／Tailscale 状态，不先改机器人配置。Permission denied 查账号和已授权 key；host key changed 先核对机器编号／现场指纹，不能用 StrictHostKeyChecking=no 或删除 known_hosts 当作默认修复。

### 5.3 只读检查 ROS 输入

先 ssh s10-48-remote，在 AGX 运行：

```bash
source /opt/ros/jazzy/setup.bash
source /home/golai/goai_embodied_future_material/install/setup.bash
source /home/golai/s10_control_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_DEFAULT_PROFILES_FILE=/home/golai/.ros/fastdds_ethernet.xml

ros2 topic list
ros2 topic type /JOINTS_DATA
ros2 topic info /JOINTS_CMD -v
ros2 interface show drdds/msg/JointsDataCmd
timeout 5 ros2 topic echo /MOTION_INFO --once
```

非交互 ssh 命令也要显式 source 这些环境，不能依赖登录 shell 自动加载。若某个 setup 不存在或类型不可见，应查当前安装环境，不能直接用旧 drdds package 覆盖机上的定义。

本次机上确认：

| Topic | 实际 ROS 类型 | 用途 |
|---|---|---|
| /JOINTS_DATA | drdds/msg/JointsData | q／dq 等电机状态；具体字段用 interface show 核对 |
| /JOINTS_CMD | drdds/msg/JointsDataCmd | 可见关节控制命令候选；需验证动态内容和控制源 |
| /IMU | sensor_msgs/msg/Imu | 姿态／角速度／加速度 |
| /STEER | drdds/msg/Steer | 原生遥控指令候选，需确认归一化／单位语义 |
| /MOTION_INFO | drdds/msg/MotionInfo | 当前控制器／模式／运动状态 |
| /ODOM | nav_msgs/msg/Odometry | 速度／位姿参考；需检查来源和质量 |

话题可见、存在 publisher，都不等于收到有效消息。本次 /JOINTS_CMD 发现原生 bare-DDS publisher，Reliable QoS；仍要在人工遥控时验证实际样本、时间、控制源和字段。

只做 RL 的第一阶段不需要直接登录 103／106。它们有不同的授权／运行环境，不能假设 AGX 的 golai 账号也适用。当前已授权的 AGX 入口足以检查可见 ROS 接口。

**不要运行** ros2 topic pub、模式切换、rl_deploy、自动起身或 ros2 bag play 来“测试 SSH”。bag 回放可能发布到真实控制话题，必须在隔离环境／ROS 域中进行。

## 6. 原生遥控策略在哪里，怎么利用

最新只读日志证明：103 机身 motion_master 加载 libCD1RunPolicy.so，策略 CD1_policy，版本 CD1_Policy_v1.0.3、ABI 1、Build 2026-08-12。这不是 Mac 的公开 policy.onnx，也不是之前训练的 speedturn ONNX。

Android 遥控器目前有证据支持的是“选模式、发指令”；没有证据表明真正的低层 locomotion 网络在 Android APK 内。当前 103 SSH 容器视图看不到日志所示库，有限读取没有找到独立权重。不能因此断言权重已加密；已确认的是正常控制通信使用 TLS／DTLS。[调查证据](<workspace>/s10-real-readiness/artifacts/native-navigation-audit-20260917/README.md)。

因此先用黑盒 teacher：人工通过正常遥控器让原生策略实际运动，只读收集 command、state、可见 motor target 和结果。不要绕过保护、root Android 或提取受限制文件作为训练前置条件；如果厂商明确授权提供可导出模型，再单独核对许可、接口和安全测试。

### 6.1 需要重新采哪些数据

建议一个示范 session 至少包含：

```text
必须优先：
  /JOINTS_DATA       关节状态
  /JOINTS_CMD        若可收到有效原生命令
  /IMU              姿态与角速度
  /STEER            实际遥控输入
  /MOTION_INFO      模式／控制源／切换状态
  /ODOM             经质量验证的速度／位姿

按实际接口追加：
  /GAIT、/MOTION_STATUS、/NAV_CMD（仅适用于实际使用的控制链）
  /tf、/tf_static（如果有效且可见）
  session 元数据：机器人编号、载荷、地面、模式、起止、操作者、安全干预
```

开始前先对上述 topic 做 type／info／消息定义快照，配置兼容 QoS。保存源 timestamp 与 bag 接收 timestamp，不把两者混成一个时间。当前 AGX 与 103／106 的时钟体系不能未经验证就当作同一精准时钟。

录制程序只订阅、不发布运动；会写入新的 bag 文件并占用资源，需先检查磁盘和 DDS／CPU 负载。现场采集应另行安排人工操作者、急停和安全场地；本文不自动触发采集或运动。

建议覆盖低速直行、加减速、左右缓弯、停止、倒退、原地转向、模式切换；分清普通／快走模式，左右转都采。先低风险覆盖稳定区，再逐步扩大范围，不为了“老师更快”直接满摇杆。

### 6.2 从 bag 到训练标签

预处理以 50 Hz policy tick 对齐，但不能把 10 Hz ODOM 插值当作 50 Hz 无误差真值。处理流程：

1. 反序列化，按 joint name／编号校验顺序、单位、符号、源 timestamp。
2. 排除失联、起身／趴下过渡、急停、外部 SDK 控制、无效定位和未知模式片段；保留排除原因。
3. 确认每个 motor command 的 publisher／控制源，以及 kp、kd、tau_ff、q_des、dq_des 语义。
4. 若原生 actuator 合同确实等同当前 SDK，才将目标反解为 POLICY action：

```text
leg action   = (q_des - q_default) / [0.125, 0.25, 0.25]
wheel action = dq_des / 5.0
然后从 ROBOT 顺序转换为 POLICY 顺序。
```

5. 原生若有不同默认角、动态增益、非零前馈力矩或另一种控制形式，**不能**用上述反解后直接 BC。保留原始 motor target，做 actuator retarget／输出级模仿；无法可靠 retarget 时只用状态轨迹作辅助先验／评价。
6. 对齐 observation 到“该动作计算时可用的状态”，而不是动作后的状态；辨识感知／网络／电机 delay。上一帧 action 也由同一序列构造。
7. 保存 obs57、action、command、valid_mask、episode_id、模式和 schema hash；建议 Parquet／NPZ，原始 bag 不改动。
8. 按完整 session／轨迹划分 train／validation／test，不能随机拆相邻帧以免数据泄漏。

原生通常是闭环、有隐藏状态的 controller。一张瞬时 obs57 不一定唯一决定其 action；所以使用序列 student，并允许示范只是辅助，而不是要求精确克隆每个 motor command。

只有 JOINTS_DATA／IMU 时，可以研究状态模仿或辨识后的逆动力学，但不是直接 action BC。现阶段不为缺失标签强加 AMP 等另一整套训练方法；先把统一残差 PPO 路线跑通。

## 7. NVIDIA GPU 环境怎么搭

### 7.1 先选对服务器，不先买“任意大显存 CUDA 卡”

推荐 Linux x86_64、Ubuntu 22.04／24.04、带 RT cores 的 RTX GPU。第一版建议 24 GB 级 RTX、64 GB RAM、至少约 200 GB 可用 SSD；更大环境／日志需要更多空间，这是本项目容量建议。

Isaac Sim 5.1 官方要求列出 32 GB RAM、16 GB VRAM 的最低配置，并明确 A100／H100 等无 RT cores GPU 不支持。headless 不表示绕过这一限制。B200 等其他计算卡也不能仅凭 CUDA／显存推定兼容，先运行 compatibility checker／官方 smoke test。[NVIDIA 系统要求](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html)。

如果现有服务器只有 A100／H100：它可做 PyTorch 离线 BC／蒸馏，但不要按本文声称能运行 Isaac Sim 主训练环境；需要另配兼容 RTX 仿真机器或另选经过验证的仿真后端，那是环境方案变更。

在服务器执行只读检查：

```bash
uname -m
cat /etc/os-release
ldd --version
nvidia-smi
df -h
```

确认 x86_64、驱动、GPU 型号／显存、GLIBC 和存储。shared server 的驱动升级应由管理员处理，不自行替换宿主机驱动。

### 7.2 锁定版本基线

为避免旧资料与新 API 混用，本文给一个可复现起点：

| 软件 | 起始版本 |
|---|---|
| Python | 3.11 |
| Isaac Sim | 5.1.0 |
| Isaac Lab | Git tag v2.3.2 |
| PyTorch／torchvision | 2.7.0／0.22.0，cu128 wheel |
| RSL-RL | rsl-rl-lib 3.1.2，由该 Lab tag 安装 |
| NumPy | < 2，按 Lab 依赖解析 |

这不是“2026 年所有机器的最新版本推荐”，而是有明确 tag／安装组合的工程基线。若新服务器要求其他受支持版本，先单独评估并重建锁文件和测试，不能把本文与 latest／3.x 的命令拼接。

Python 版本依据：[Isaac Lab v2.3.2 环境说明](https://github.com/isaac-sim/IsaacLab/blob/v2.3.2/docs/source/setup/installation/include/pip_python_virtual_env.rst)；Sim／Torch／GLIBC 依据：[该 tag 安装说明](https://github.com/isaac-sim/IsaacLab/blob/v2.3.2/docs/source/setup/installation/pip_installation.rst)；RSL-RL／NumPy 依据：[isaaclab_rl 依赖声明](https://github.com/isaac-sim/IsaacLab/blob/v2.3.2/source/isaaclab_rl/setup.py)。

### 7.3 安装命令：在 GPU 服务器执行，不是在 Mac／机器人

以下假设 Miniconda 已安装、目标目录尚未存在。先把 GPU_USER 换成实际 Linux 账号；目录只是建议，不代表现存。

```bash
conda create -n s10_rl python=3.11 -y
conda activate s10_rl
python -m pip install --upgrade pip

# 先安装 Isaac Sim，再安装 Isaac Lab。
python -m pip install "isaacsim[all,extscache]==5.1.0" \
  --extra-index-url https://pypi.nvidia.com

python -m pip install torch==2.7.0 torchvision==0.22.0 \
  --index-url https://download.pytorch.org/whl/cu128

mkdir -p /home/GPU_USER/work/s10-rl
cd /home/GPU_USER/work/s10-rl
git clone --branch v2.3.2 --depth 1 \
  https://github.com/isaac-sim/IsaacLab.git IsaacLab
cd IsaacLab
./isaaclab.sh --install rsl_rl

# 本项目的分析／导出／测试工具；Lab 自动装的包不重复手工换版本。
python -m pip install "numpy<2" onnx onnxruntime \
  scipy pyarrow rosbags pytest ruff
python -m pip check
python -c "import torch; print(torch.__version__); print(torch.cuda.is_available()); print(torch.cuda.get_device_name(0))"
```

首次启动可能要确认 NVIDIA 使用许可、联网获取扩展／资产；读清许可后由使用者确认。需要下载较大依赖，安装过程不能视作训练已经开始。

补充包用途：

| 包 | 用途 |
|---|---|
| torch／RSL-RL | GPU 网络与 PPO |
| onnx | 权重检查、导出、模型合法性检查 |
| onnxruntime | CPU 数值回归；冻结 base 转成 Torch 后在 GPU 批量推理 |
| scipy／numpy | 时间对齐、统计与离线检查 |
| rosbags | 离线读取 ROS bag；自定义 drdds 仍需注册正确的 msg 定义 |
| pyarrow | 专家／rollout 表格数据存储 |
| tensorboard | 训练日志；由 Lab 依赖安装 |
| pytest／ruff | contract、导出、runner 回归测试与代码检查 |

不要同时安装 onnxruntime 和 onnxruntime-gpu 到同一环境来“加速”。本方案训练 loop 的冻结 actor 建议使用 Torch CUDA，不在每个并行环境逐次调用 batch=1 CPU ONNX。ONNX 权重转换必须通过数值等价测试；识别不了图结构时改用验证过的转换或示范蒸馏，并记录误差。

环境验收通过后保存完整 pip freeze、Git SHA、nvidia-smi、资产和模型 SHA 到一次 run 的 manifest；上面补充包尚未形成最终锁文件，不能宣称已在目标 GPU 服务器安装测试。

### 7.4 先跑官方 smoke test

在 /home/GPU_USER/work/s10-rl/IsaacLab：

```bash
conda activate s10_rl
./isaaclab.sh -p scripts/tutorials/00_sim/create_empty.py --headless

./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
  --task Isaac-Velocity-Flat-Anymal-C-v0 \
  --num_envs 64 --headless --max_iterations 2
```

这是安装／GPU 验证，不是 S10 训练。该 ANYmal task 在 [v2.3.2 注册文件](https://github.com/isaac-sim/IsaacLab/blob/v2.3.2/source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/config/anymal_c/__init__.py) 中存在。通过后才导入 S10；显存／GPU 崩溃时先减少环境数并查驱动／兼容性，不先调 reward。

旧比赛 MuJoCo 环境与新地图包环境应分开：地图包要求 MuJoCo 3.13.0／NumPy 2.5.3，与 Lab 的 numpy<2 冲突，不能直接把其 requirements.txt 装进 s10_rl。

## 8. 从 Mac 把材料送到 GPU

### 8.1 配置 GPU SSH

GPU 服务器信息尚未给出，以下均是占位模板。不要原样执行 GPU_HOST／GPU_USER。

在本机 SSH config 增加一个经授权的目标，例如：

```sshconfig
Host gpu-s10
    HostName GPU_HOST
    User GPU_USER
    Port 22
    IdentityFile ~/.ssh/id_ed25519
    IdentitiesOnly yes
```

IdentityFile 仅当该公钥已被服务器授权时才适用；使用指定的服务器 key，不复制私钥到服务器。首次连接核对 host fingerprint。然后：

```bash
# Mac 上执行。
ssh gpu-s10
# GPU 上查看 nvidia-smi；exit 返回 Mac。
```

### 8.2 先另存机上的两个 57D 模型

下面是在 Mac 新建材料目录并下载，不会覆盖 SDK 原文件：

```bash
mkdir -p <workspace>/training_materials/s10_048/models

scp s10-48-remote:/home/golai/goai_embodied_future_material/src/S10_sdk_deploy/policy/policy.onnx \
  <workspace>/training_materials/s10_048/models/official_sdk_agx_92db62c.onnx

scp s10-48-remote:/home/golai/provisioning/speedturn2000-20260912/s10_general_speedturn_57d_model2000.onnx \
  <workspace>/training_materials/s10_048/models/speedturn_57d_model2000.onnx

scp s10-48-remote:/home/golai/provisioning/speedturn2000-20260912/model-metadata.json \
  <workspace>/training_materials/s10_048/models/speedturn_model-metadata.json

shasum -a 256 <workspace>/training_materials/s10_048/models/*.onnx
```

按第 3.3 节逐个比对 SHA。此 training_materials 目录是**建议创建的新目录，本文编写时未执行这些复制命令**。

scp 的方向：“远程别名:/远程路径 本机路径”是下载；反过来是上传。机器人访问与 GPU 访问分别由 Mac 发起，不要假设 GPU 能通过机器人的现场 IP 访问它。

### 8.3 最小上传集合

替换 GPU_USER 后，在 Mac 执行；不使用 --delete：

```bash
ssh gpu-s10 'mkdir -p /home/GPU_USER/work/s10-rl/materials /home/GPU_USER/work/s10-rl/reference /home/GPU_USER/work/s10-rl/docs'

rsync -av \
  <workspace-legacy>/goai26-s10-racing/upstream/goai_embodied_future_material/src/S10_sdk_deploy/S10_description/ \
  gpu-s10:/home/GPU_USER/work/s10-rl/materials/S10_description/

rsync -av --exclude .git --exclude .venv --exclude __pycache__ \
  <workspace-legacy>/goai-s10-gate16-policy-v1-5/ \
  gpu-s10:/home/GPU_USER/work/s10-rl/reference/gate16-training/

rsync -av \
  <workspace>/training_materials/s10_048/models/ \
  gpu-s10:/home/GPU_USER/work/s10-rl/materials/models/

scp <workspace-legacy>/goai26-s10-racing/upstream/goai_embodied_future_material/src/S10_sdk_deploy/run_policy/s10_policy_runner.hpp \
  gpu-s10:/home/GPU_USER/work/s10-rl/reference/s10_policy_runner_mac.hpp

scp s10-48-remote:/home/golai/goai_embodied_future_material/src/S10_sdk_deploy/run_policy/s10_policy_runner.hpp \
  <workspace>/training_materials/s10_048/s10_policy_runner_agx.hpp

scp <workspace>/training_materials/s10_048/s10_policy_runner_agx.hpp \
  gpu-s10:/home/GPU_USER/work/s10-rl/reference/s10_policy_runner_agx.hpp

scp <workspace>/S10_NVIDIA_RL_TRAINING_START_ZH.md \
  gpu-s10:/home/GPU_USER/work/s10-rl/docs/
```

还需复制 Python observation.py、授权说明和模型 manifest；第一版不必复制全部点云、历史 bag、Docker 镜像或 SSH 配置。上面的 reference 仅用于阅读和迁移，不意味着直接运行旧 trainer。

如果 GPU 不能访问 GitHub，可在 Mac 下载并核对相同 tag 后传输完整目录；不要用来源不明的模型镜像替代。

## 9. 需要新增什么代码，怎么真正开始 RL

### 9.1 推荐新建独立训练项目，别把旧比赛工作区改乱

建议服务器目录结构如下；**这些是待创建的文件，不是当前已经存在的地址**：

```text
/home/GPU_USER/work/s10-rl/
  IsaacLab/                        固定 v2.3.2
  materials/
    S10_description/               完整官方机器人资源
    models/                        各基线另存、SHA 固定
    s10.usd                        导入后生成，待验证
  s10_fast_turn/                    新的可安装 Isaac Lab 扩展／项目
    pyproject.toml
    s10_fast_turn/
      __init__.py                  注册 task
      assets/s10_cfg.py            机器人、关节映射、PD／电机模型
      tasks/fast_turn_env.py       GPU 向量环境与 reset／termination
      mdp/observations.py          obs57／privileged obs
      mdp/actions.py               base＋residual、混合 actuator
      mdp/rewards.py               速度／转向／滑移／稳定性
      mdp/commands.py              合理速度－转弯联合采样与 ramp
      mdp/randomization.py         物理／延迟／噪声随机化
      agents/ppo_cfg.py            teacher PPO 配置
      networks/base_actor.py       ONNX → 冻结 Torch，等价验证
      networks/residual_student.py GRU
    scripts/train_teacher.py       导入并注册自定义 task 后启动
    scripts/distill_student.py     sequence DAgger
    scripts/export_onnx.py          组合模型导出
    scripts/evaluate.py             三基线＋teacher＋student 同条件测试
    scripts/preprocess_native.py    bag → 经验证的示范，非必需前置
    tests/                         合同、reset、Torch／ONNX、映射测试
  runs/<run_id>/                   checkpoint／配置／manifest／指标
```

自定义项目必须安装并在训练脚本中 import 注册模块；仅给官方 train.py 一个新 task 字符串不会自动发现我们的代码。

### 9.2 第一步：导入 S10，验收物理模型

使用 Isaac 的 URDF importer／converter 生成 USD；完整 mesh 保持可寻址。需确保：

- floating base、不锁住机身；重力方向、单位 m／kg／s 正确。
- 16 个可控关节、4 个连续轮轴；关节轴／左右轮正转符号正确。
- 按名字映射关节；验证腿位置控制与轮速度控制，避免 URDF importer 给轮加默认位置 spring。
- 质量／惯量／质心与原资产一致；轮碰撞半径／接触不会因为简化 mesh 变形。
- 四轮接触、无异常初始穿透；碰撞 group／self-collision 合理。
- actuator 采用明确的 PD＋effort／velocity limit，检查 PhysX drive 与自己算力矩是否重复施加。

建议 physics_dt = 0.005、decimation = 4，让 policy_dt = 0.02。现有 MuJoCo 代码通常更小步长计算 PD；因此还要做 0.005 对更小 physics_dt 的收敛对照，不能将建议设置称为已验证的真实电机更新率。

必要的最小单元测试：零 action → 默认姿态；单个正 hipx action → 正确对应关节；四个轮 action → 正确轮速与行进符号；joint order round-trip；obs57 数值／维数一致；reset 后 action/history/hidden 无跨 episode 污染。

### 9.3 第二步：先跑冻结基线，不更新网络

对本地 SDK、AGX SDK、speedturn 分别测试相同 command：

```text
零速度站立／停止
低速直行 → 加速 → 减速
倒退
低速左右行进转弯
原地左右转向
直行期间短扰动
```

先 1 个环境看关节／姿态／接触，再 64 个环境统计。对齐与现有 MuJoCo 的动态表现；如果 base 在无随机化平地都频繁倒、轮空转或速度符号错误，先查物理／合同，不能期待 PPO 自动修复建模错误。

若将 ONNX MLP 还原为 Torch，使用保存的有效 obs 样本＋随机压力样本对比逐层／输出；可先设 max_abs_error < 1e-5 的 float32 数值门槛，实际容差按设备验证记录。旧 load_onnx_teacher_exact 只适用于它识别的参数名和拓扑；对所有 ONNX 并非通用转换器。

选定 base 后锁文件与 hash，不在一轮实验中偷偷换 base。冻结参数，但允许 residual 与 base 共同闭环；base 的 last_action 要看到组合 action。

### 9.4 第三步：训练 privileged residual teacher

核心：

```text
a_base  = frozen_base(obs57)
delta   = residual_actor(obs57, privileged_state)
a_final = compose_and_guard(a_base, delta)
q_des / dq_des = sdk_action_decode(a_final)
physics step → reward / termination
下一帧 obs57 的 last_action 按统一 runner 合同记录 a_final
```

先给残差较小权限，避免把已有平地能力破坏。例如起始限制在约 0.02–0.05 rad 的腿角修正、约 2–5 rad/s 的轮速修正；这是物理尺度建议，需要转换到各关节 normalized action 空间，不是直接用同一个数字套 16 维。通过低速回归后再扩大，最终仍受关节／轮速／力矩约束。

PPO 记录采样 residual 的 log probability，动作限幅／组合属于环境执行层；不要把裁剪后的动作误当成未裁剪 Gaussian 样本计算 log_prob。Gaussian mean 初始化为零残差，先小探索，不把冻结 base 的输出再随机化成大动作。

teacher actor 使用 privileged state 是为了学习真实速度误差／滑移／动态；最终发布的是蒸馏后的 student。也可先只让 critic privileged 来降低第一版复杂度，但必须记录采用哪一配置，不能混称。

建议 PPO 初始参数，均待调优：

| 参数 | 起点 |
|---|---|
| num_envs | smoke 64；稳定后 512／1024，再按显存吞吐扩大 |
| steps_per_env | 24–32 个 policy step |
| episode_length | 15–20 s |
| learning rate | 3e-4，结合 KL 自适应／早停 |
| clip ratio | 0.2 |
| gamma／GAE lambda | 0.99／0.95 |
| epochs／minibatches | 5／4 |
| seed | 开发先固定；正式至少 3 个独立 seed |

不要直接从第一轮启动 4096 环境、多 GPU 或最大速度。先确认显存、物理稳定性、reward 分项和 PPO 更新正常。

### 9.5 Command curriculum：速度与转弯一起学，但不组合出不合理动作

| 阶段 | command 范围建议 | 放行条件 |
|---|---|---|
| A | vx 0–0.8 m/s；停止／低速缓弯 | 基线能力未明显退化、动作平滑 |
| B | vx -0.5–1.5；低速 yaw_rate 约 ±0.4 | 倒退、左右转、停止均可跟踪 |
| C | vx 0–2.5；原地转向单独采样；逐步加 yaw | 转弯滑移／侧倾可控，无只学直行现象 |
| D | 在证据支持下扩到约 3–4 m/s | 速度提高同时控制误差和硬件约束合格 |
| E | 小坡／粗糙面／载荷／delay 随机化 | 留出条件下 student 仍优于基线 |

范围不是实机下发许可，也不是我们已经能达到的速度。高速弯道的 yaw_rate 要随 vx 收紧，例如以 |vx × yaw_rate| 的侧向加速度预算筛选；原地转向单独处理，避免除零。初始预算可从约 1–1.5 m/s² 的仿真实验起步，再依据摩擦和实测调整。

command 要有加速度／角加速度限制，包含持令、缓变、制动和合理阶跃。若只训练“固定速度一直往前”，最终停止和转弯不会自然变好。

vy 初期限制很小或为 0；S10 普通轮的横向能力并不等同全向底盘，不能均匀采大侧向速度。先检查原策略侧移能力与轮／腿协同，后续再按可达性扩展。

curriculum 应由独立评测的速度／yaw tracking、fall、slip 和约束触发率共同决定；不能只看 total reward 变高就升级。

### 9.6 Reward：提速不等于“奖励最大 vx”

主项：

- 机身坐标真实 vx／vy 跟踪 command；不能奖励无条件向前跑。
- 真实 yaw_rate 跟踪 command，训练直行时的 yaw 漂移、缓弯和原地转向。
- 制动后残余速度、过冲、command response delay。
- 轮接触点相对地面的滑移、离地／跳动、四轮载荷异常。
- 腿关节／轮速／力矩限位，机械功率、动作变化率与 jerk。
- residual 大小惩罚／低速 anchor，保留基线能力；高速区逐步减弱，不把新策略永远限制成老师。
- 翻倒、躯干触地、严重关节违规作为 termination 和负奖励。

可采用 exp(-velocity_error² / sigma²) 类跟踪项，但要分项记录，并按实际单位／dt 归一化。upright 奖励不能和合理转弯侧倾冲突：可对参考倾角误差惩罚，侧倾参考约由 atan(vx × yaw_rate / g) 提供初始线索，符号／质心／足轮布局需验证，并限制最大倾角。

利用关节提速的方向是：主动悬挂／降低或调整质心、左右载荷转移、转弯内倾、保持轮接触与改善牵引；不预先硬编码纯四足飞行相位。让同一个 residual 学轮速与腿姿态协同，不另外引入一套会争夺控制权的 MPC。

有可用专家标签时，添加小权重、按模式／质量筛选的 BC loss；原生专家只覆盖稳定区，不能把其动作硬约束到所有更高速状态。没有标签时照常做 residual PPO。

### 9.7 Domain randomization 与 sim-to-real

先名义模型跑通，再逐项加入，而不是一开始大范围全随机：

- 地面纵／横向摩擦、滚动阻力；普通轮的侧向滑移模型尤其重要。
- 质量／载荷／质心偏移；先以测量不确定区间为范围。
- motor strength、PD 参数、joint friction／零偏、轮有效半径。
- 感知 delay、actuation delay／jitter、IMU 噪声／bias、少量缺帧。
- 小坡、轻微不平、外力扰动；部署目标不含的极端环境先别主导训练。

模型应考虑轮／腿速度依赖的可用力矩；至少用保守界并记录简化。验证 URDF 限位、摩擦、载荷、时钟／delay 后才扩随机范围；随机化不是错误 URDF／错 action mapping 的补救。

### 9.8 第四步：GRU student 蒸馏与 DAgger

1. 用 teacher 在多种 command／随机化下 rollout，保存 obs57 序列、episode mask、teacher 最终 action 或 residual 标签。
2. 用 GRU student 做序列监督；若学 residual，base 仍由同一 obs57 算出。建议从约 1–2 s 序列起步，处理 hidden burn-in／截断，不打乱为独立帧。
3. 改用 student 闭环 rollout，在它到达的状态查询仿真 teacher，聚合数据，再训练；不能只在 teacher 的状态分布上评估。
4. student 在同一留出集达到 teacher 接近的速度／转向／稳定性后，才作为候选发布。
5. 导出一个组合 ONNX，验证 Torch、CPU ONNX 和机器人 ARM ORT 数值、序列状态与 50 Hz 延迟。

DAgger 查询的仿真 teacher 可以自动获得；原生实机 teacher 不能在 GPU 虚拟状态上随意查询。原生示范是附加固定数据，不是在线远程调用机器人来给每个训练 step 打标签。

### 9.9 将来的训练命令接口

下面只是建议 CLI，**scripts／task 尚未实现，不是现在可以粘贴运行的命令**：

```bash
# 在新的 s10_fast_turn 项目和 task 注册完成之后。
python scripts/train_teacher.py \
  --task S10-Fast-Turn-Teacher-v0 \
  --base-onnx /home/GPU_USER/work/s10-rl/materials/models/official_sdk_agx_92db62c.onnx \
  --num-envs 64 --seed 42 --headless

python scripts/distill_student.py \
  --teacher-checkpoint /home/GPU_USER/work/s10-rl/runs/RUN_ID/teacher.pt \
  --hidden-size 128

python scripts/export_onnx.py \
  --checkpoint /home/GPU_USER/work/s10-rl/runs/RUN_ID/student.pt
```

正式实现必须提供 resume、配置快照、阶段／seed、基线 hash、数据 schema、评测频率、checkpoint 保留和中断恢复；旧 ONNX 无 optimizer，不能用 --resume 假装恢复 PPO。

## 10. 要产出哪些文件，怎样证明更好

每次 run 至少交付：

```text
manifest.json              Git / 环境 / 硬件 / 资产 / base / 数据 SHA
config.yaml                obs / action / actuator / command / reward / randomization
teacher.pt                 网络 + optimizer + critic + 迭代 + RNG 状态
student.pt                 GRU + base 标识 + 蒸馏配置
policy.onnx                最终组合模型
policy-contract.json        输入输出、关节顺序、缩放、hidden/reset 规则
evaluation.json/csv         独立评测及 confidence / episode 数
tensorboard/                reward 分项、KL、损失、curriculum、约束
videos/                    留出代表案例及失败，不只选最佳一次
```

“比官方快”至少拆成两个对照：

1. 与公开 SDK／speedturn 在相同仿真模型和 command 下比较，控制实验可重复。
2. 与当前原生遥控控制器在相同真实场地／载荷／测量条件下比较；没有原生权重不能把它假装放进 Isaac 做公平基线。

建议的平地评测矩阵：

| 测试 | 必看指标 |
|---|---|
| 直行多速度 | 稳态 vx 误差、yaw drift、slip、轮速／功率／限位、最高可持续合格速度 |
| 左右定曲率弯 | yaw_rate 误差、实际转弯半径、横向漂移、roll、slip |
| 原地转向 | yaw_rate、转心平移／累计漂移 |
| 加减速／停走 | response delay、过冲、停止距离 |
| 扰动／随机化 | 失败率、恢复时间、约束触发率，按条件分桶 |
| 长时间连续运行 | 热／功率／延迟、hidden 漂移、错误恢复；实机逐步延长 |

初始成功标准应先测基线后写入配置，例如“最高合格持续速度提升 10–20%，同时 yaw／横向误差不退化”；这只是目标，不能在无实测时写成成果。只增大 command 但实际轮空转，不算提速。

“精准控制方向”也有两层：

- base policy 负责 body velocity／yaw_rate 跟踪。
- 绝对朝向、路径横向误差和回到防线／路线，需要外层 heading／path controller＋可靠定位。57D 不含绝对 yaw，仅训练 base 无法保证世界坐标方向或路径不漂。

若“防线”指特定区域／球场边线，需另外定义地图、目标朝向、允许误差和上层控制，不能靠 base reward 含糊代替。/ODOM frame_id=map 不是定位正确的充分证据；今日定位状态调查也未确认有效全局定位。

评测用至少 3 个训练 seed、独立随机化种子、完整 episode 数，并报告置信区间。少量试验零翻倒不能宣称绝对安全／保证不摔。

## 11. 真机部署前的门槛

先离线／跨仿真器，再真机：

1. Torch↔ONNX 序列数值、hidden reset、joint mapping、命令单位全部通过。
2. Isaac 留出任务与独立 MuJoCo 平地回归通过；复杂地图另作附加测试，不取代平地验收。
3. 复制新模型到隔离 SDK 副本，不覆盖官方文件；模型、runner、contract 一起版本化。
4. AGX 上纯 ORT 合成输入测试，不创建控制发布者；评估 p50／p95／p99 延迟，policy step 20 ms 内有充分余量。
5. 只读 shadow 输入测试，记录拟输出，不接管电机。
6. 有现场操作者、急停、支撑／安全边界后，从站立、低速、停止、左右缓弯逐项放行；高速最后。
7. 明确遥控原生运控与外部 SDK 的控制权互斥、失联／超时动作、hidden reset、退出／回退流程。

当前 SDK 与原生 /NAV_CMD 是两条不同控制链。不要让厂商规划器、我们的 /cmd_vel、遥控器和另一个 trial runner 同时竞争控制权。检查残留 publisher／supervisor 的真实状态，不能仅看“没有 rl_deploy 名字”就认定无人发命令。

可复用的本地基础：

- [prepare_s10_sdk_copy.py](<workspace>/s10-real-readiness/scripts/prepare_s10_sdk_copy.py)：隔离副本准备。
- [run_s10_sdk_trial.py](<workspace>/s10-real-readiness/scripts/run_s10_sdk_trial.py)、[s10_sdk_trial.py](<workspace>/s10-real-readiness/scripts/s10_sdk_trial.py)：试验／监控基础，使用前读清模式与参数，不直接启动运动。
- [真实机器人 quickstart](<workspace>/s10-real-readiness/docs/S10_REAL_ROBOT_QUICKSTART_ZH.md)：包含很多 050 历史内容，以 048 文档和当前实机为准。

本指南故意没有给出一键启动 SDK／起身／模式切换命令。训练环境搭建并不授权机器人运动。

## 12. 第一天可以实际完成的清单

- [ ] 选定 GPU_HOST／GPU_USER／GPU 型号，验证 RT／driver／GLIBC／磁盘兼容。
- [ ] Mac 能 ssh gpu-s10；机器人入口仍是 ssh s10-48-remote，两个目标不混淆。
- [ ] GPU 新建独立 s10_rl 环境，官方 empty／ANYmal smoke test 通过。
- [ ] 将完整机器人资源与两个机上 ONNX 另存复制，核对 SHA；保留本地 SDK 对照。
- [ ] 整理 command／action／history／ramp 合同并测试；不能带着已知缩放不一致开始实机部署。
- [ ] 创建 S10 USD，核对 16 个关节、轮轴、惯量、接触和两种 actuator。
- [ ] 完成 57D observation／action mapping 单元测试。
- [ ] 冻结三种 57D actor 做平地回归，选择 base。
- [ ] 仅在上述通过后跑新 task 的 64-env residual PPO smoke；验证 checkpoint／resume。
- [ ] 再扩大环境数和 curriculum；遥控专家采集可并行准备，但不是首轮 GPU 训练的前置。

如果交给另一位工程师，其第一个任务应是“实现并验收 S10 平地 Isaac task”，而不是“在现有目录找一个 train.py 直接跑”。

## 13. Q&A

### Q1：遥控器里的 policy 怎么被我们用？需要遥控器真实输出吗？

用正常遥控器驱动**机器人原生策略真实运动**，同步录 command／状态／可见关节命令。只记录 Android 按钮名称、界面截图或摇杆值，没有机器人闭环响应，不足以训练低层动作模仿。若录不到 motor target，状态轨迹仍可分析，但不能伪造 BC action。

### Q2：遥控器是 Android，能不能拿 APK／policy 出来测试？

有厂商授权时可检查正常可导出的应用信息／文件。现有证据证明 103 机身侧加载了实际运控库，但没有证明 Android 内含同一个可导出模型；也不据此断言 APK 里绝无模型。APK 可能只有 UI／协议调用，拿到也不等于拿到 locomotion 模型。优先黑盒采集，不 root／解密／绕过访问保护；模型提取不是本训练路线的必要步骤。

### Q3：我们有没有现成 base 可以直接用？

有三个已知 57D ONNX 候选：Mac SDK、AGX SDK、speedturn model2000。它们可以用于冻结推理；还要分别对齐 contract、验证动态表现。原生 .so 是另一个控制器，没有可用独立权重。

### Q4：今日雷达／IMU 包可以直接训练走路吗？

不能直接做低层动作 BC，因为缺关节状态、关节命令和 command。它适合传感器／时钟／ODOM 分析，且 ODOM 不是天然绝对真值。首轮平地 PPO 不需要点云。

### Q5：只给 GPU 服务器这些文件，就能开始训练吗？

能开始环境和任务实现；不能直接开始已验证的新 S10 PPO。还缺 S10 Isaac task、混合 actuator、contract 测试、base 动态匹配与 residual trainer 接线。第 9 节的计划目录不是已经完成的代码。

### Q6：训练时要一直 SSH 连着机器人吗？

不用。仿真 teacher／PPO／student 在 GPU 离线运行；机器人只用于模型／配置取回、专家采集和最后验证。不要通过 SSH 网络来闭环控制每个 50 Hz 电机 step。

### Q7：要装 ROS 2 到 GPU 上吗？

第一版 PPO 不需要。离线 bag 可用 rosbags＋正确 msg 定义；需要原生 ROS 重放时另设隔离环境。最终机器人继续用已装的 Jazzy。

### Q8：能不能直接 resume speedturn 的 ONNX？

不能恢复 PPO optimizer／critic。可冻结其 actor、数值等价转 Torch 或做 actor warmstart，但这是新训练起点。真正 resume 需要原 checkpoint、配置、环境版本和 RNG／optimizer 状态。

### Q9：为什么不直接把轮速度调高？

加大 command／输出不保证实际速度，可能造成打滑、侧倾、温升或刹不住。训练要同时优化牵引、腿姿态、载荷转移、yaw 跟踪和停止，且守住电机约束。

### Q10：为什么最终用 GRU？能否保持旧单输入 ONNX？

GRU 能从本体时间序列估计滑移／动态，帮助 student 补偿看不到真实线速度的问题，但带来 hidden 管理和 runner 修改。第一阶段冻结基线／teacher 可以先沿用旧单输入接口；若最后改用无记忆 student，需要单独证明精度／鲁棒性，不假设效果一样。

### Q11：只训练 yaw_rate 能保证直行方向准确吗？

不能保证全局方向。局部 yaw_rate 跟踪要配外层朝向／路径闭环、可靠定位和命令限幅；base 与上层各有明确误差指标。

### Q12：有没有保证比官方快且绝对不会摔的方法？

没有。可以设定公平基线、速度提升目标、稳定性／控制误差门槛和逐级实机验证，报告证据和置信区间；不能从仿真 reward 或少量成功视频推导绝对保证。

## 14. 继续工作需要补齐的信息

1. NVIDIA GPU 服务器 SSH 别名或地址、账号、GPU 型号／显存、是否可安装独立环境。
2. 原 speedturn 训练源码／checkpoint／config 的可访问位置；元数据指向原 Windows／GPU 工作站，不代表此 Mac 已有。
3. 实机测试目标：载荷、地面、期望持续速度、yaw／横向误差、停止距离及允许的测试范围。
4. 遥控普通／快走模式下能否实际订阅到 /JOINTS_CMD；命令字段／模式／QoS／timestamp 的动态验证。
5. 厂商模型与数据使用许可范围；取得文件不自动意味着可以再分发或移植。

本文据 RL Robotics、Sim-to-Real Transfer 和 NVIDIA Isaac Sim 技能组织了训练、物理匹配、环境隔离与逐级验收；具体机器人事实以本地源码、模型哈希和 2026-09-17 实机只读证据为准。
