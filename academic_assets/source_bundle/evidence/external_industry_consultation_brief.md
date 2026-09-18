# S10 多地形轮足运动与导航项目：工业界技术交流材料

版本：2026-09-03  
交流对象：四足/轮足机器人运动控制、强化学习、感知与导航团队  
用途：说明当前方案、实验事实和瓶颈，并获得可执行的技术路线建议

> 结论摘要：项目已经打通 S10 的仿真训练、策略导出、MuJoCo 部署接口和分地形评估链路，也训练出了可用的平地/坡道/湿滑通用基线与连续上楼专用策略。但目前**尚未训练出满足验收条件的跨地形 privileged LLC Teacher**：现有 Teacher 能稳定完成 PPO 更新、能够利用新增特权输入，也能保留部分基础运动能力，但仍未同时形成前后、横移、转向和复杂地形行为。短期工程交付采用“通用基线 + 楼梯专用策略 + 确定性切换”更有依据；“单一 Generalist LLC → GRU Student → Learned HLC”仍是一条合理的研究路线，但继续重复当前 Teacher 配方的成功概率较低，需要先请专家判断问题拆解、训练目标和系统边界是否正确。

文中将结论分为四类：**已验证**指有固定 checkpoint、脚本或 JSON 产物支持；**阶段性结论**指样本量有限但已足以停止当前实验；**计划**指尚未完成；**待确认**指材料不足或存在路线冲突。内部证据主要来自[决赛开发进度](./development_progress.md)、[开发经验](./development_experience.md)、[S10 接口契约](./s10_interface_contract.md)和[连续楼梯专用策略记录](../archive/preliminary_20260820/s10_stairs_57d_training.md)。

## 1. 项目背景

项目对象为 S10 轮足四足机器人。机器人具有 12 个腿部关节和 4 个驱动轮，目标场景包含平地、坡道、随机粗糙地形、湿滑地面、连续楼梯、单台阶/路缘和窄平台等。项目最初从比赛路线中的连续楼梯通过需求出发，后续希望建立可泛化的多地形运动与导航系统，并参考论文 [Learning Robust Autonomous Navigation and Locomotion for Wheeled-Legged Robots](https://arxiv.org/abs/2405.01792) 的 Teacher–Student LLC 与分层导航结构。

当前工作已经明确分成两条线：

1. **比赛交付线**：冻结已完成验收的 `model_796`，对连续楼梯保留 `Gate16/WP18` 专用策略回退；全局任务调度、路径跟随、策略切换和安全仲裁采用确定性逻辑。
2. **论文复现/研究线**：训练一个跨地形 privileged MLP Teacher，再以 DAgger 蒸馏为可部署 GRU Student；只有 LLC 成熟且确定性局部规划被证明不足时，才考虑训练 mobility-aware HLC。

这两条线的目标和验收标准不同。现有专用楼梯策略证明了 S10 具备通过目标楼梯的机械与控制能力，但不能证明单一跨地形策略可行；Teacher 训练失败也不等于比赛交付线不可用。

## 2. 项目目标

### 2.1 工程目标

建立一套可在真实 S10 上部署的多地形运动与导航系统，使机器人能够：

- 在平地、坡道、粗糙和湿滑地面上稳定跟踪 `[vx, vy, wz]`；
- 通过连续楼梯、单台阶/路缘和窄平台，且不以明显牺牲普通路段能力为代价；
- 接收真实 LiDAR/状态估计产生的局部地形信息，并对噪声、延迟和缺失保持鲁棒；
- 与全局规划、局部避障、安全仲裁和策略回退机制形成清晰接口；
- 在 Isaac、MuJoCo 和真实机器人之间保持一致的 observation、action 和控制语义。

### 2.2 当前阶段的可量化目标

- 通用 LLC 在 Isaac held-out 中按地形分类报告成功率、跌倒率、超时、非轮接触、姿态峰值和完成时间，而不是只看平均 reward。
- 15 cm 路缘在正确高度图下成功率至少 80%，且换成平地图后下降至少 40 个百分点，以证明策略确实利用地形感知。
- 平地/转向保持 `12/12`，不能出现“越障改善但基础运动退化”。
- 候选通过后再进入 MuJoCo 分段、多 seed、连续路段和实机低速测试。

### 2.3 研究目标

验证论文式“privileged Teacher → recurrent Student → optional HLC”能否适配 S10 的轮足形态、16D 混合动作空间和现有传感器条件。当前尚未完成 Teacher，因此不能启动 Student，更不能据此评价完整论文架构在 S10 上的最终可行性。

## 3. 当前技术架构

### 3.1 系统分层

```text
LiDAR / IMU / Joint State / State Estimation
                    │
                    ├── Global Map / Global Planner（确定性，尚未在本项目完整实现）
                    │
                    └── Local Elevation / Traversability Representation
                                      │
                          Local Planner / Safety Arbiter
                                      │  [vx, vy, wz] / policy mode
                                      ▼
                     RL Locomotion Controller, 50 Hz
                                      │  12 joint positions + 4 wheel velocities
                                      ▼
                         PD / Wheel Velocity Control
                                      │
                                      ▼
                                    S10
```

当前真正实现和重点验证的是 LLC 及其仿真/部署接口。全局规划、局部避障和真实地形图链路仍主要是架构规划，不能描述为已完成系统。

### 3.2 已冻结的可部署 LLC 接口

| 项目 | 当前合同 |
|---|---|
| Actor 输入 | `174D float32`，无 normalization |
| 本体感觉 | 机身角速度 3D、projected gravity 3D、命令 3D、相对关节位置 16D、关节速度 16D、上一帧 action 16D，共 57D |
| 地形感知 | 117D 高度图，`13×9`，分辨率 0.15 m，覆盖机器人前后约 `x=-0.6…1.2 m`、横向 `y=-0.6…0.6 m` |
| Actor 输出 | 16D：12 个腿部关节位置目标 + 4 个轮速目标 |
| 控制频率 | Physics 200 Hz，policy/control 50 Hz |
| 腿部执行 | position target，经 `Kp=80, Kd=2` 的 PD 控制 |
| 轮部执行 | velocity target，经 `Kd=0.6` 的速度控制 |
| 策略网络 | 当前通用 MLP 为 `174→512→256→128→16` |

PyTorch、ONNX、MuJoCo runner 的 observation slice、网络输出和 action decoder 已对齐；ONNX/runner raw action 最大误差为 `1.07e-6`。尚未完成的是 Isaac 与 MuJoCo 在同初态下的加速、刹停、转向和低台阶动力学响应对齐。

### 3.3 通用策略训练环境

- 仿真与训练：Isaac Lab manager-based environment + RSL-RL PPO。
- 混合地形：20 列，按 `4/4/3/3/3/2/1` 分配给平地/缓坡、随机粗糙、连续坡道、连续楼梯、单台阶/路缘、湿滑和窄平台。
- Curriculum：每类地形独立统计；成功率高于 80% 升级，低于 20% 降级。
- 奖励：以线速度、角速度跟踪为主，辅以姿态、垂直速度、关节力矩、关节/轮加速度、action rate/smoothness、关节限位和非期望接触约束。
- 初始化：通用 174D actor 从官方 57D actor 扩展，新增 117D 高度输入权重初始化为零，再进行 Stage 0/1 训练。
- 随机化：已有部分摩擦、质量、质心、执行器和 delay 设计；完整分阶段 DR 范围、真实测量依据及评估记录尚未闭环。

### 3.4 论文式 Teacher–Student 路线

当前 privileged Teacher 合同为 `187D/187D/16D`：在 174D 基础观测上增加机身线速度 3D、4 个轮接触状态、4 个轮接触力幅值和 2D 实际静/动摩擦。Teacher 是三层 MLP，以 PPO 学习；不输入 terrain ID，不使用 policy router、CPG 或显式 gait target。

计划中的 Student 是只接收可部署、有噪声和时序延迟观测的 GRU，通过 DAgger 模仿冻结 Teacher。该阶段尚未开始，因为 Teacher 尚未通过多轴运动、复杂地形及 held-out 验收。

## 4. 已完成工作

| 工作项 | 状态 | 可复用成果 |
|---|---|---|
| 本机训练环境 | 已完成 | RTX 4060 Laptop 8 GB 上 Docker、CUDA、Isaac 和 64→512 env 训练链路可用；paper-scale batch 可运行 |
| S10 资产与控制接口 | 基本完成 | 57D/174D observation、16D action、joint order、action scaling、PD 参数和 50 Hz 控制合同已冻结 |
| PyTorch/ONNX/MuJoCo runner 对齐 | 数值接口完成 | observation slice 和 decoder 零误差，网络输出误差在 `1.07e-6` 内 |
| 通用感知式任务 | 已完成 | `General-Perceptive-Deeprobotics-S10-v0` 可训练、可评估、可导出 |
| 七类混合地形与分类 curriculum | 已完成 | 每类独立升降级，基础测试通过 |
| 第一版通用策略 | 已完成 | `model_796`、TensorBoard、参数、ONNX 和三 seed held-out JSON 已产出 |
| 连续上楼专用策略 | 已完成阶段性版本 | 57D blind actor `model_499` 已导出，并完成规则楼梯及一次 WP18 MuJoCo 回归 |
| 连续下楼专用策略 | 完成能力版 | `model_1000` 在训练指标上成功率 100%，但冲击和左右稳定性未验收 |
| collision-reflex / centerline 消融 | 仅完成实验协议 | 已定义 6 个单 seed 分支和 held-out 指标，但现有材料没有可引用的正式结果，不能判断有效性 |
| 论文式 direct MLP 诊断 | 已完成，结论为失败 | `model_895` 对高度图有行为响应，但严格越障失败并破坏基础运动 |
| privileged Teacher 合同与 smoke | 已完成 | 187D 输入、16D 输出、特权信号、checkpoint 和 PPO 数值链路成立 |
| Teacher 多轮受控实验 | 已完成多组，未形成合格 Teacher | 已排查训练量、探索、命令采样、yaw 语义、对称性、输入依赖、部分 reward/curriculum 信号和执行分布 |

## 5. 当前实验结果

### 5.1 已取得的正向结果

1. **专用连续上楼策略已经证明任务可做。** `model_499` 在当前 132 个规则楼梯组合评估中为 `132/132`；训练末轮 curriculum success 为 96.1%。在一次固定条件的比赛 MuJoCo WP18→19 回放中到顶，用时 10.4 s，横向偏移 RMS/peak 为 `0.268/0.443 m`，roll RMS/peak 为 `3.24°/11.35°`，无 NaN/Inf。该结果仅是一条确定性回归，尚不等价于多初态、多摩擦的鲁棒成功率。
2. **`model_796` 已形成部分通用能力。** 三个 held-out seed 的 20 场景总成功率分别为 55%、50%、60%。合并分地形统计：平地/缓坡 `12/12`、随机粗糙 `6/12`、连续坡道 `8/9`、湿滑 `6/6`、连续楼梯 `1/9`、单台阶/路缘 `0/9`、窄平台 `0/3`。这是当前 Isaac suite 的结果，不能直接外推为实机鲁棒性。
3. **训练和部署接口大体可信。** 174D/16D 的排列、缩放、ONNX 推理和 MuJoCo runner decoder 已排除明显错位；官方 57D actor 扩展到 174D/187D 的零影响 warm-start 也通过 CPU、CUDA 和 Isaac 同轨迹检查。
4. **PPO 优化过程没有整体失效。** paper-scale 一次 update 为 `256×1950=499,200` transitions，本机可稳定执行；KL 大部分落在目标范围，reward/advantage 对正确运动方向有正向差值。问题更像是任务信号、探索覆盖和多目标竞争不足，而不是优化器完全不工作。
5. **Teacher 确实会读取新增特权输入。** warm-start Teacher 的新增列权重发生更新，friction/contact/force 消融会改变 action；但 action dependence 尚未转化为平台或湿滑任务收益。

### 5.2 主要方案与结果矩阵

| 输入/起点 | 方法或单变量 | 实验规模 | 结果 | 工程结论 |
|---|---|---:|---|---|
| 57D 官方 actor | 连续楼梯专用策略 + stability fine-tune | 首轮至 iteration 2916；从 `model_1800` 再微调 500 updates | 规则楼梯和固定 WP18 回归成功 | **有效**；适合作为短期 specialist，但泛化证据有限 |
| 57D `model_499` | 窄走廊硬边界续训 | 400 updates | 规则矩阵从 `132/132` 退化到 `46/132`；后期训练 success 约 0% | **无效**；blind actor 看不到横向边界，硬终止只诱导减速/失败 |
| 57D 官方 actor / `model_499` | open/closed collision reflex、centerline command | 已形成 6 分支单 seed 协议 | 当前仓库文档没有正式结果表 | **证据缺失**；不得写成已验证方案 |
| 174D `model_796` | 七类混合地形 Stage 0/1 | 512 env，至 iteration 796 | 普通地形较好，楼梯/路缘/窄边缘失败 | **部分有效**；可作基础 locomotion baseline，不是完整 generalist |
| 174D `model_796` 后续 | Stage 2 感知式 MLP `model_895` | 100 iterations | 正确高度图有局部进展，但严格任务失败，平地跟踪退化 | **无效**；不能靠继续延长同一路线修复 |
| 187D scratch Teacher | 原始 privileged PPO | 122.88 万 transitions | general `0/60`，平地/转向 `0/12`，terrain level 全 0 | **无效**；从零训练没有形成基础控制 |
| 187D scratch Teacher | 完整命令、论文式线速度 reward、提高初始 std | 61.44 万至累计 184.32 万 transitions | 前后运动逐步形成；yaw 近零、lateral 缺失 | **仅早期有效**；探索改善不是完整控制 |
| 187D scratch Teacher | paper-scale PPO | 累计 46 batches / 2296.32 万 transitions | command/stop 和部分方向形成，但无单一 checkpoint 同时通过前后与双向 yaw；后期出现正向能力回退 | **无效**；单纯增加训练量没有闭环 |
| 187D scratch Teacher | pure-yaw 覆盖 10%→30% | 50 iterations | yaw 仍未形成 | **无效**；不是单纯 yaw 样本不足 |
| 187D Teacher | direct yaw-rate 合同审计 | 零训练 | 既有配置本来就是 yaw-rate，不是 heading target | 已排除接口语义假设 |
| S10 资产 | 左右镜像增强审计 | 零训练 | mapping 数学正确，但官方资产 COM、惯量和后轮位置存在真实非对称 | 不应强行使用无损左右对称增强；需确认硬件一致性 |
| `model_796` actor | 187D Teacher warm-start | 1 paper batch | 保留基础控制，新增输入被更新 | **训练合同有效**，但不代表任务能力形成 |
| warm-start Teacher | 低探索续至累计 10 paper batches | 499.2 万 transitions | flat 保留；平台和湿滑严格 task benefit 均失败 | **无效**；当前低探索 lineage 冻结 |
| warm-start Teacher | 初始 std `0.2069→1.0` | 3 batches / 149.76 万 transitions | action dependence 明显增强，平台仍 `0/10`，湿滑改善不足 | **无效**；更强探索未转化为能力 |
| warm-start Teacher | terrain/reward rollout audit | 49.92 万 transitions，零 update | 障碍 exposure 足够；前进有正信号，但抬轮 transition 的 reward/return/advantage 相对更差；curriculum success 与 strict traverse 不一致 | 发现明确的 credit/语义风险 |
| warm-start Teacher | dense axle-clearance reward | 1 batch | 抬轮信号由负转正，但 flat retention 失败，也未形成上台 | **无效**；能改 credit，不等于能力形成 |
| warm-start Teacher | 论文式 MC terrain filtering 诊断 | 零 update | 当前 MC 可行带为空；横移、纵向和低速 tracking 同时失败 | 尚无单一 curriculum 变量可直接训练 |
| warm-start Teacher | deterministic mean 对照 | 零 update | 分布标准化后仍不优于 stochastic；sample noise 不是共同根因 | 已排除“只因探索噪声太大” |
| warm-start Teacher | command-normalized XY reward | 1 paper batch | 平均相对误差仅改善 2.513%，左右横移方向错误且 `|vy|<0.05`，flat retention 同时失败 | **无效**；不追加第二 batch |

### 5.3 对“LLC Teacher 训练不出来”的准确表述

当前不是代码无法运行，也不是 PPO 完全没有学习，而是：

- 从零 Teacher 可以提升 reward、形成 command conditioning 或某些方向的运动，但不能在同一 checkpoint 上形成完整多轴控制；
- 从 `model_796` warm-start 可以保留基础控制，并让策略依赖新增 privileged observation，但没有形成路缘/平台通过或显著湿滑收益；
- 已排除单纯训练量不足、yaw 命令语义错误、纯 yaw 样本不足、探索噪声是共同根因、网络完全不读取特权输入等解释；
- 现有证据最支持的是：**基础多轴速度控制、障碍动作 credit、curriculum success 语义和任务难度同时耦合，当前 Teacher 训练问题不是一个可直接通过单参数修复的故障。**

## 6. 当前遇到的问题

### 6.1 主要技术瓶颈

| 优先级 | 瓶颈 | 当前证据 | 风险 |
|---|---|---|---|
| P0 | Teacher 未形成完整多轴基础控制 | lateral 几乎为零或方向错误；linear/yaw 在不同 checkpoint 间互相回退 | 复杂地形训练之前，底层 command controllability 就不充分 |
| P0 | 越障 credit 与成功语义不一致 | curb 前进有正信号，但 lift transition 的 reward/return/advantage 更差；curriculum success 与 strict traverse 混淆 | 策略可能优化“移动了/结束了”，而非真正通过障碍 |
| P0 | 工程目标与研究目标尚未最终取舍 | specialist 已有效，generalist Teacher 仍失败 | 容易继续投入高成本训练，却没有明确的产品验收边界 |
| P1 | 理想仿真高度图与真实 LiDAR 链路未闭环 | 真实地图的外参、时延、invalid、dropout、age 未冻结 | 即使仿真成功，也可能无法实机复现 |
| P1 | 双仿真动力学尚未对齐 | 数值接口已对齐，但 Isaac/MuJoCo 同初态响应未完成 | 训练成功后仍可能出现控制响应差异 |
| P1 | 执行器和接触模型缺少实机辨识 | 当前采用固定 PD、轮速和仿真摩擦/延迟范围 | Sim-to-Real 风险无法量化 |
| P1 | 评估统计量不足 | 多个结论来自单 seed 或单次 MuJoCo 回放 | 无法估计置信区间和长尾失败概率 |
| P2 | 完整导航层尚未实现 | 当前主要完成 LLC，HLC/局部规划仍在设计阶段 | 不能证明端到端多地形导航能力 |

### 6.2 初步可行性判断

- **整体分层架构是合理的。** 感知/地图、规划、安全层和 50 Hz LLC 分离，符合可调试、可回退和可验收的工程需求。
- **短期采用通用基线 + specialist 的证据强于单一 Generalist。** 但目前只证明了部分能力，完整比赛交付的可行性仍需通过切换安全、分段多 seed、连续路线和双仿真/实机回归确认。
- **单一 Generalist LLC 研究路线可行性尚不能确认。** 论文结构本身合理，但当前 S10 训练合同、动作空间、传感器表示和 reward/curriculum 适配尚未证明；不建议原样追加训练量。
- **立即启动 GRU Student 或 Learned HLC 不可行。** Teacher 未达到可模仿的能力下界，蒸馏只会复制不完整行为，HLC 也无法补偿底层缺失的运动 primitive。
- **最应优先解决的是任务分解，而不是再选一个 PPO 超参数。** 需要先决定：Teacher 是否必须同时解决完整三轴控制和复杂地形；或者应先用已验证的基础控制器，单独学习 terrain-conditioned residual、mode/skill 或落脚/机身高度指令。

### 6.3 需要进一步确认的问题

1. **产品目标**：最终要求是比赛路线稳定完成、可泛化的工业原型，还是论文方法复现？三者允许的 specialist 数量、切换逻辑和训练成本不同。
2. **Generalist 的定义**：必须由一个网络直接输出全部 16D action，还是允许共享 backbone + 多 head、mixture-of-experts、residual policy 或少量显式 mode？
3. **论文复现程度**：当前 `13×9` 矩形局部高度图、187D 特权输入、命令范围和 reward 都是 S10 适配，并非论文完全同构；目标是方法复现还是数值复现？
4. **Student 观测差异**：当前 174D MLP 使用 projected gravity、关节状态、上一动作和单帧高度图；论文式 Student 强调原始 IMU 与时序记忆。后续应保留现有 174D 合同，还是按论文重新定义 recurrent observation？
5. **Teacher 特权信息差异**：当前 Teacher 有 base linear velocity、轮接触、接触力和摩擦，但没有论文使用的逐轮地形法向；这是可忽略的形态适配，还是关键缺失？
6. **Curriculum 参数来源**：现用 MC 阈值 `t_l=0.2/t_h=0.8` 是 S10 自行适配，不是论文公开数值；是否需要重新标定？
7. **传感器可用性**：实机 LiDAR 型号、安装位置、FOV、更新率、延迟、盲区、时间同步和定位误差尚未形成测量报告。
8. **硬件真实性**：官方 URDF 中 `base COM y=+57.317 mm`、惯性非对称和后轮 9 mm 偏置是否真实对应实机，而非资产误差？
9. **执行器模型**：是否有电机电流、力矩、轮胎滑移、传动间隙、控制延迟和饱和数据可用于 system identification？
10. **地形要求**：目标台阶高度、楼梯尺寸、坡度、窄道宽度、湿滑系数、动态障碍速度和允许通过时间尚未统一冻结。
11. **安全约束**：允许的最大倾角、冲击、非轮接触、关节温度/电流和策略切换瞬态尚未量化。
12. **评估规模**：目前若干关键结果是单 seed 或单次回放；需要确认工业交付要求的 seed 数、初态覆盖和失败率上限。
13. **导航输入输出**：高层是否只输出 `[vx, vy, wz]`，还是还需输出 body height、roll/pitch、gait/mode、落脚点或局部子目标？

## 7. 我们计划采用的下一步方案

以下是仓库中原定路线，需在专家评审后决定是否调整：

1. **先完成 U-002 动力学合同**：在相同初态和相同 action 下，对比 Isaac/MuJoCo 的平地加速、刹停、双向转向、15/20 cm 低台阶响应，避免训练问题与运行时问题混淆。
2. **重新定义 R-002 Teacher 的最小能力门槛**：先解决单一 checkpoint 的 stop、前进、后退、双向横移和双向 yaw，再进入障碍 curriculum；或者经专家确认后改为基于现有 `model_796` 的 residual/skill-conditioned 方案。
3. **修正任务语义**：使 curriculum success、严格 traverse 和部署任务成功使用一致事件定义；在任何新 reward 实验前，先确认 lift、跨越、离台和恢复的 credit 链。
4. **仅在 Teacher 通过后启动 R-003**：用 DAgger 训练可部署 GRU Student，并验证 hidden-state reset、teacher/student action error、缺图、延迟、滑移和跨运行时导出。
5. **补齐逐级 Domain Randomization**：范围由实机辨识数据给出，覆盖摩擦、质量/质心、motor strength、PD、轮半径、0–2 帧延迟、IMU、高度图噪声/丢失/延迟和外力。
6. **完成真实 LiDAR 高度图链路**：`PointCloud + IMU/里程计 → elevation/variance/validity/age → 13×9 LLC grid`，先离线回放，再低速实机。
7. **进行分层验收和部署**：Isaac held-out → MuJoCo 分段多 seed → 连续路段 → 实机低速；每层都保留官方策略和楼梯 specialist 回退。
8. **R-004 HLC 条件触发**：只有 LLC 已冻结、真实局部地图稳定、且确定性局部规划被实验证明不足时，才训练低频 HLC。

需要专家重点判断：第 2 步是否还应坚持“一个 187D MLP Teacher 同时学会所有运动和地形”，还是应该先改变问题分解。

## 8. 希望专家帮助判断的问题

### 8.1 技术路线可行性

1. 基于 `model_796` 在普通地形较好、在楼梯/路缘/窄边缘失败，而 57D 楼梯 specialist 可成功的证据，继续追求单一 16D Generalist LLC 是否值得？还缺什么决定性实验？
2. 目前把完整三轴速度跟踪和越障行为放在同一个 PPO Teacher 中联合学习，是否属于不合理的任务耦合？成熟团队通常如何安排预训练、skill curriculum 和 fine-tuning 顺序？
3. S10 的混合动作空间“12 个关节位置 + 4 个轮速”是否适合直接由同一 policy 输出？是否应改为腿部 torque/position residual、轮部速度/力矩，或增加低层 actuator model？
4. 当前 50 Hz policy + 固定 PD 的控制层是否足以支持目标楼梯和单台阶，还是需要更高频率、WBIC/MPC、足端/轮端 impedance 或 learned actuator network？

### 8.2 架构与风险

1. 对当前“确定性规划/安全层 + RL LLC + specialist fallback”的分层是否认可？工业项目中哪些模块必须保留可解释的 deterministic fallback？
2. 最大技术风险应如何排序：Teacher 目标设计、真实地形感知、动力学辨识、策略切换、局部规划，还是评估覆盖？
3. 当前哪些工作可以删除或推迟？例如，在 Teacher 失败阶段，是否应继续暂停 Student、HLC 和大规模 Sim-to-Real 随机化？
4. 如果由成熟团队重做，会先建立哪些 classical/control baseline、hardware-in-the-loop 测试和数据集，再开始大规模 RL？

### 8.3 优先级

1. 下一项最高信息增益实验应该是什么？希望对方给出具体的输入、唯一变量、训练预算、通过门槛和停止条件。
2. 如果只有两到四周，应优先把 specialist 路线做成可靠系统，还是继续解决 Generalist Teacher？判断依据是什么？
3. 哪个最小实机实验能够最快区分“仿真任务定义错误”和“Sim-to-Real 模型错误”？

## 9. 四足机器人强化学习控制相关问题

### 9.1 Locomotion Policy 输入与输出

1. 成熟轮足机器人 LLC 的 policy 输入通常保留哪些量：IMU 原始量、重力方向、机身线/角速度估计、关节状态、接触估计、上一动作、局部高度、轮滑移、历史状态？哪些量只给 critic/Teacher？
2. 工业部署中会直接使用 LiDAR elevation patch 作为 LLC 输入，还是先经过 traversability encoder、latent estimator 或局部规划器？
3. 输出层通常选择 joint position、joint torque、foot/wheel target、whole-body reference 还是 residual action？对于 S10 的 12 腿关节 + 4 轮，应如何拆分最稳健？
4. 是否会给 policy 显式 body height、roll/pitch、gait/mode、contact schedule 或 wheel/step preference？如果不使用 CPG，如何避免不同技能之间互相干扰？

### 9.2 Teacher–Student 与时序估计

1. Teacher 的 privileged observation 应包含什么，才能真正帮助 Student，而不只是让 Teacher 依赖不可部署信息？
2. Teacher 是否应从强基础 locomotion policy warm-start，还是从零训练？现有结果显示从零 Teacher 连基础多轴控制都不稳定，这在工业实践中是否常见？
3. Student 一般使用 GRU/LSTM、history encoder、state estimator + feed-forward policy，还是显式在线系统辨识？选择依据是什么？
4. DAgger 中 teacher/student action mixing、数据刷新频率、序列长度、hidden reset 和终止状态如何设置？如何避免 Student 在 Teacher 未访问状态上失稳？
5. 如何判断 privileged input 是“必要信息”还是“shortcut”？应做哪些 observation ablation 和 causal test？

### 9.3 Sim-to-Real 与训练流程

1. 成熟团队会先做哪些系统辨识：质量/质心、惯量、关节摩擦、motor strength、延迟、PD、轮半径、轮胎接触和地面摩擦？
2. Domain Randomization 的范围是经验设定、实测置信区间，还是 online adaptation 反推？通常分几个阶段加入？
3. 对高度图应模拟哪些故障：外参误差、时间同步、动态物体、ray miss、空洞、地图更新延迟、局部遮挡和机器人自身点云？
4. 从 simulation-only 到真实机器人，常见的安全升级顺序、速度限制、保护姿态、fallback 和自动停止条件是什么？
5. 评价 locomotion policy 时，除成功率外，工业界最看重哪些指标：tracking、energy/COT、冲击、轮滑、姿态、执行器峰值、thermal margin、恢复率还是失败可控性？

## 10. 多地形运动策略相关问题

当前事实形成了一个明确矛盾：specialist 楼梯策略已经成功，而通用策略在楼梯、路缘和窄边缘上失败。因此希望专家不要只从论文偏好回答，而是结合部署可靠性比较以下架构。

| 方案 | 希望重点判断的内容 |
|---|---|
| A. 多个 specialist + policy selection | 需要多少技能才不会产生维护爆炸；terrain classifier/router 如何训练；切换如何做滞回、动作混合、状态继承和失败恢复 |
| B. 单一 Generalist LLC | 需要怎样的 observation、recurrent state、网络容量和 curriculum；如何防止普通地形能力被复杂地形训练破坏 |
| C. Hierarchical RL | 高层应输出 skill、gait、body pose、footstep、速度还是局部子目标；层级频率、信用分配和联合/分阶段训练如何安排 |
| D. Hybrid | classical planner/MPC/WBC 负责几何与约束，RL 负责 tracking/residual/recovery；这种组合在轮足机器人上通常如何落地 |

希望得到以下具体经验：

1. 平地、坡道、碎石、湿滑、楼梯和高台阶是否真的需要不同 locomotion policy，还是只需要少量“rolling / stepping / recovery”技能？
2. 如果采用 specialist，切换条件应来自地形类别、可通行性、预测失败概率还是 policy uncertainty？
3. 切换时除了 0.3–0.5 s action blending 和共享 `last_action`，还必须同步哪些内部状态、command filter、积分量或 GRU hidden state？
4. 如果采用 Generalist，是否建议共享 encoder + 多 head、mixture-of-experts 或 skill latent，而非单一无结构 MLP？其工业可靠性如何？
5. 对“前轮需要抬升越障、后轮需要继续推进”的轮足任务，成熟方案会显式建模 contact/phase/axle clearance，还是让 policy 自行涌现？
6. 如何设置回退：检测到卡住、横向漂移、倾角过大或高度图失效后，是切 specialist、降速、后退重试，还是交给 recovery policy？

## 11. 规划与导航架构相关问题

我们倾向于采用以下职责划分，请专家判断是否符合成熟工程实践：

| 层级 | 候选职责 | 需要确认 |
|---|---|---|
| Perception / State Estimation | LiDAR、IMU、关节、里程计融合；生成局部 elevation/occupancy/semantic 信息 | 更新率、延迟和失效模式如何与控制对齐 |
| Terrain Understanding | slope、roughness、step height、traversability、support risk | 是否需要显式 terrain class，还是只输出连续 cost |
| Global Planning | 在静态地图上给出全局路径/航点 | 何时使用 graph/Dijkstra/A*，何时需要 learned planner |
| Local Planning / Avoidance | 动态障碍、局部可通行性、速度和姿态约束 | 与 LLC 地形适应职责如何分界 |
| Footstep / Wheel Contact Planning | 必要时规划接触点、轮迹或 body path | 轮足平台在多高/多离散障碍上才需要显式 planner |
| High-level Command | 输出 `[vx, vy, wz]`、body pose、skill 或短期子目标 | 最小充分接口和更新频率 |
| RL LLC | 执行指令、吸收局部模型误差、稳定通过地形 | 是否应承担避障和路线选择 |
| Safety / Fallback | 限幅、倾覆预测、停车、回退与人工接管 | 哪些规则必须独立于 learned policy |

重点请教：

1. 工业系统中，避障通常由 local planner 完成，还是 LLC 也会根据近场高度图直接改变路径？两者如何避免同时修正导致振荡？
2. 对楼梯、路缘、沟槽、踏脚石和窄边缘，是否必须维护显式 elevation/traversability map？使用 2.5D map 的适用边界是什么？
3. 轮足机器人何时需要 footstep/contact planner？若只给 LLC `[vx, vy, wz]`，是否足以稳定处理离散高差？
4. 高层 Planner 最常见的接口是速度、短期 SE(2)/SE(3) target、body trajectory、foothold，还是离散 skill？典型更新频率和 horizon 是多少？
5. 如何把 locomotion capability 纳入规划代价，例如坡度、台阶高度、轮滑风险、policy uncertainty 和恢复空间？
6. 动态障碍处理是否应完全在 local planner，还是要训练 LLC 的碰撞反射/紧急避让？
7. 局部地图短时失效时，系统应继续使用 recurrent belief、降低速度、切 blind baseline，还是立即停车？
8. 如果确定性规划已经能给出可靠速度指令，训练 learned HLC 的实际收益通常来自哪里？应以什么 A/B 指标证明值得增加复杂度？

## 12. 希望推荐的论文 / Repository / Framework

希望专家优先推荐**能够复现完整训练与部署链路**的材料，而不只是给出算法名称。每项推荐最好说明适用机器人、动作接口、仿真器、是否开源训练代码、是否有真实机器人结果，以及与 S10 的主要差异。

| 类别 | 希望获得的推荐 | 筛选条件 |
|---|---|---|
| Locomotion RL | 四足/轮足跨地形 LLC 论文与代码 | 支持 12+4 执行器、地形感知、速度命令、公开 reward/curriculum |
| Teacher–Student | privileged learning、DAgger、recurrent adaptation 项目 | Teacher/Student observation、序列训练和部署代码完整 |
| Generalist / Skill-based | 单策略、多专家、mixture-of-experts、HRL 对比实现 | 有跨技能干扰、切换安全和消融结果 |
| Sim-to-Real | actuator network、system identification、domain randomization 工程 | 有实测参数来源和真实机器人验证，而非只给随机范围 |
| Mapping / Traversability | LiDAR elevation map、traversability、terrain cost | 支持实时延迟、invalid/uncertainty 和机器人 footprint |
| Planning | mobility-aware local/global planning、footstep/contact planning | 能与 RL LLC 通过明确接口集成 |
| Simulation | Isaac Lab、MuJoCo 或其他并行训练/验证框架 | 支持轮足接触、批量地形、可重复 benchmark 和跨仿真验证 |
| Benchmark | 多地形 locomotion/navigation 数据集与测试协议 | 包含成功率之外的稳定、冲击、能耗、滑移和长尾失败指标 |

对每个 Repository，希望进一步询问：

- 是否包含训练配置、环境生成、reward、curriculum、评估脚本和部署 runner；
- 是否能在单张 8–24 GB GPU 上复现；
- 依赖版本、许可证和维护状态；
- 是否已有相近轮足平台的迁移经验；
- 最值得先复现的最小实验是什么，预期训练预算和验收指标是什么。

## 13. 最终技术交流问题清单

### 13.1 必须问的问题

1. **路线选择**：已有 `model_796` 普通地形基线和成功的楼梯 specialist，但 Generalist Teacher 经 2296 万级 scratch transitions 及多组 warm-start 实验仍未闭环。您会选择 specialist + router、单一 Generalist、HRL，还是 classical/RL hybrid？为什么？
2. **问题拆解**：是否应要求一个 Teacher 同时学习前后、横移、转向和所有复杂地形？如果不应，建议怎样分阶段，阶段间如何迁移且避免遗忘？
3. **下一实验**：请给出一个最高信息增益实验，包括起始 checkpoint、唯一变量、地形、命令分布、训练预算、通过门槛和停止条件。
4. **动作接口**：S10 当前输出 12 个关节位置目标 + 4 个轮速目标、50 Hz、固定 PD。该接口是否适合工业级多地形控制？您会改哪里？
5. **越障 credit**：当前 curb rollout 中前进有正奖励，但 lift transition 的 reward/return/advantage 更差；加入 dense lift reward又破坏 flat retention。成熟团队如何设计或规避这种 credit assignment？
6. **Curriculum 语义**：当前 distance success、strict traverse 和 MC tracking 是不同事件。工业项目通常如何定义统一、不可投机的能力门槛？
7. **Teacher–Student**：Teacher 应有哪些 privileged inputs、Student 应有哪些 deployable inputs和多长历史？Teacher 达到什么门槛才值得开始 DAgger？
8. **多策略切换**：若保留楼梯 specialist，terrain detection、切入/切出、action blending、hidden state、失败回退和安全保护应如何设计？
9. **感知/规划边界**：局部 elevation/traversability、local planner 和 RL LLC 各自应负责什么？Planner 给 LLC 的最小充分指令是什么？
10. **Sim-to-Real**：在继续训练前，最必须补做哪些实机 system identification 和 LiDAR 测量？哪些随机化参数必须来自实测？
11. **可行性结论**：以当前证据继续论文式单 Generalist 路线，您估计最大的失败原因和成功前提分别是什么？什么证据会让您建议停止该路线？

### 13.2 如果时间允许应该问的问题

1. 现有 `13×9`、0.15 m 分辨率、高度范围约 `x=-0.6…1.2 m` 的局部高度图，对 S10 楼梯/路缘是否足够？更建议何种坐标系、范围和表示？
2. 当前 Teacher 新增 base linear velocity、接触、接触力和摩擦后，策略会读取输入但没有任务收益。如何判断输入不足、网络利用方式错误，还是任务本身不需要这些输入？
3. 官方资产存在 COM/惯量/轮位非对称。工业界会保留这种真实非对称、校正资产，还是通过有限度 augmentation 处理？
4. 对轮足机器人，轮滑移估计和轮地接触状态通常如何获取并输入控制器？
5. Generalist 若采用共享 backbone + multi-head/MoE，expert 数量、gate 输入和训练稳定性如何控制？
6. 是否应使用 residual RL：由现有稳定 policy、MPC 或 WBC 给 nominal action，RL 只负责 terrain-conditioned residual？
7. 对楼梯和高台阶，是否需要显式 body-height/attitude trajectory 或 contact/footstep plan，而不能只依赖 `[vx, vy, wz]`？
8. 实机从低速测试到自主导航的验收阶梯应如何设置？每一级建议多少次无干预通过？
9. 如何构造跨 Isaac、MuJoCo、实机统一的 episode log，使失败能归因到 perception、planning、policy、actuator 或 contact model？
10. 请推荐一个与我们最相近、训练和部署代码都完整的 Repository，并指出最值得直接移植和最不应照搬的部分。

### 13.3 可以进一步深入讨论的问题

1. Learned HLC 相比 deterministic local planner 的明确收益、训练成本和安全代价是什么？
2. 是否需要把 LLC hidden state、uncertainty 或 predicted traversability 暴露给 Planner？怎样定义稳定接口？
3. 对动态障碍，训练 collision reflex 是否有价值，还是应该完全由 local planner 和 emergency controller 处理？
4. 如何在策略训练中加入 actuator thermal/current、轮胎磨损和能耗约束，而不破坏可学习性？
5. 是否值得采用 offline RL、behavior cloning 或真实遥操作/传统控制器数据，帮助初始化稀疏越障技能？
6. 如何设计 rolling、stepping、recovery 等 skill 的数据覆盖和切换验证，避免 policy collapse 或灾难性遗忘？
7. 地形能力能否建模为 planner 可查询的 capability envelope，例如 `P(success | terrain, speed, payload)`？如何在线标定？
8. 对真实高度图的 uncertainty、staleness 和 map discontinuity，建议怎样进入 policy、planner 和 safety layer？
9. 多地形 benchmark 应如何覆盖坡度、高差、摩擦、载荷、初始姿态、传感器失效和动态障碍，并给出可信置信区间？
10. 如果不追求论文架构同构，针对 S10 最简单且最可靠的工业实现会是什么？其模块、接口和开发里程碑如何定义？

---

交流时建议优先展示三组证据：`model_796` 的分地形 held-out 结果、`model_499` 的楼梯成功与 blind corridor 退化对照、Teacher 从 scratch 到 warm-start 的失败归因矩阵。这样可以让对方直接讨论架构与训练机制，而不是把时间花在确认环境是否能运行或接口是否接错。
