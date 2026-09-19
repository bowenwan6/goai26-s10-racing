# Lynx S10 地形感知导航与强化学习控制

**GOAI LAI／狗來 · 技术研究报告 · 2026-09-02**

**摘要。** 本文报告一套面向 GOAI 2026 Track 4 Challenge 2 的轮足机器人自主控制系统。系统以几何感知和顺序航点跟踪生成速度命令，以规则状态机选择运动策略，并通过 SDK 关节控制权仲裁执行策略交接。普通赛段使用官方 57D actor；WP15→WP16 的高台阶由 174D 基础 actor 与高度图门控残差 actor 执行。残差策略采用冻结基础模型的 PPO 微调。固定 45 个入口状态的局部评测取得 38 次成功、2 次跌倒及 5 次超时；归档整圈运行按序完成 33 个航点，仿真计时为 392.257 s。本文区分已归档的比赛部署与补充材料中的多地形 RL 研究；后者的 privileged Teacher 尚未通过完整多轴运动与复杂地形验收。

## 1. 任务定义与系统实现

### 1.1 任务与版本

机器人包含 12 个腿关节和 4 个轮驱动执行器。任务是在给定航点序列 \(W=(w_0,\ldots,w_{32})\) 上依次到点。官方计分半径为水平距离 0.20 m，follower 与 router 的内部到点半径为 0.18 m。航点折线水平长度为 224.21 m，累计上升为 6.70 m。定位输入为仿真真值 odometry，任务路线、台沿几何及困难路段参数由赛前配置提供。[E1–E3]

归档比赛系统固定部署版本 `3660b81`、官方资源版本 `13dd084`、Gate 16 训练源码版本 `5ef14fa`；完整 commit 与文件哈希见附件。§1–6 的部署实现及 §7.1–7.2 的实验对应这些版本。§4.3、§7.3 和 §8 引入补充研究记录 [E7]；其中的新模型不归入该次比赛成绩。

### 1.2 软件分层与数据流

| 层次／包 | 实现与输入输出 | 团队实现范围 |
|---|---|---|
| 感知：`s10_perception` | MuJoCo 几何与状态 → LiDAR、高度图、odometry、轮子状态 | Ray casting、ROS 发布和可视化 |
| 导航：`s10_auto_nav` | 航点与观测 → `/strategy/nav_cmd_vel` | 跟踪、局部避障、地形分类和恢复 |
| 路由：同一导航包 | 状态与导航命令 → `/cmd_vel`、owner 请求 | 入口约束、策略状态机及交接确认 |
| SDK 集成：`integration/` | 本体状态、速度条件和高度图 → 关节命令 | ONNX runner、动作解码与单一 owner 仲裁 |
| 启动：`s10_bringup` | Launch、导航和策略 YAML 配置 | 节点编排、参数装配和运行入口 |

导航节点的速度输出经 launch 重映射后进入 router。Router 统一发布 `/cmd_vel`；SDK 将速度条件送入当前运动策略。两类 actor 共用关节命令出口，SDK arbiter 是 `/JOINTS_CMD` 的唯一输出边界；`/joints/owner` 返回实际控制权。Router 为确定性有限状态机，策略选择不经过神经网络。[E3]

三个 ROS 包版本均为 `0.1.0`：感知与导航使用 `ament_python`，bringup 使用 `ament_cmake`。SDK 上游提供机器人模型、官方 actor 与基础通信接口。源码附件保留以下入口文件：

```text
src/s10_perception/package.xml
src/s10_auto_nav/package.xml
src/s10_bringup/package.xml
integration/gate16_policy_runner.hpp
integration/joint_command_owner.hpp
```

归档部署使用 Ubuntu 24.04、ROS 2 Jazzy、Python 3.12.3；NumPy 1.26.4、MuJoCo 3.11.0、ONNX Runtime 1.28.0 由 `docker/requirements.lock` 固定。源码与证据选集 `PROJECT_REPORT_SOURCES.zip` 包含配置、训练代码、模型审计和运行脚本；完整运行依赖原始仓库、模型及官方资源。

## 2. 感知与导航控制

### 2.1 几何观测与输入约定

| 观测 | 采样与表示 | 消费模块 |
|---|---|---|
| LiDAR range image | 8×64；方位 360°；仰角 −30° 至 +15°；量程 0.05–12 m | 导航、记录器 |
| 近水平 scan | 取仰角绝对值最小的一行，即 +2.142857° | 局部航向搜索 |
| Height map | 13×9；前后范围 [−0.6,1.2] m；左右范围 [−0.6,0.6] m；间距 0.15 m | 地形分类、Gate 16 actor |
| Odometry | 世界系位置、姿态与速度；仿真真值 | 航点跟踪、router |
| Wheel state | 四轮世界坐标与 terrain-contact flags | 攀爬完成确认 |

LiDAR 使用 `mj_multiRay` 求首次几何相交，包含几何遮挡。高度图由独立向下射线生成，随 yaw 对齐，不随 roll/pitch 倾斜；展平顺序为 x 外循环、y 内循环。系统不从 LiDAR 重建该高度图。导航高度值及 actor 高度值分别为：

\[
h^{\mathrm{raw}}_{ij}=\operatorname{clip}(z^{\mathrm{terrain}}_{ij}-z^{\mathrm{base}},-1,1),\quad
h^{\mathrm{policy}}_{ij}=\operatorname{clip}(-h^{\mathrm{raw}}_{ij}-0.5,-1,1).
\]

Raw≤−1 的空洞／无效编码在转换后保留为 −1；该编码同时覆盖被裁剪的深地面。SDK 高度图缓存的有效期为 200 ms。观测生成不加入测量噪声、材质回波、扫描畸变或传输延迟模型。[E3]

### 2.2 顺序跟踪与局部规划

令机身 yaw 为 \(\psi\)，目标相对位置为 \(\Delta=p^*-p\)，航向误差为 \(e_\psi=\operatorname{wrap}(\operatorname{atan2}(\Delta_y,\Delta_x)-\psi)\)。前视距离与目标速度命令为：

\[
\begin{aligned}
L_t&=1.5+0.7|v_{x,t-1}|,\\
\omega_z^*&=\operatorname{clip}(1.8e_\psi,-0.7,0.7),\\
v_y^*&=\operatorname{clip}\{0.9[-\sin\psi\Delta_x+\cos\psi\Delta_y],-0.4,0.4\}.
\end{aligned}
\]

当 \(|e_\psi|>30^\circ\) 时，目标平移速度置零；其余状态按航向误差衰减前向速度，并叠加到点制动。实际输出受前向 5.0 m/s²、横向 2.0 m/s²、yaw 6.0 rad/s² 的变化率限制。该控制器采用前视目标的航向与横向误差反馈。

局部规划在目标方位 ±90° 内采样 31 个航向。对每个航向，取半宽 0.45 m 走廊内最近正向回波距离 \(C(\theta)\)，上限 4 m，按下式选取最大得分；选中净空小于 0.8 m 时判为阻塞：

\[
J(\theta)=1.6\,C(\theta)/4-|\operatorname{wrap}(\theta-\theta_g)|/(\pi/2).
\]

### 2.3 地形约束与恢复

高度图走廊先拟合平面，再计算正负残差、坡度及起伏覆盖宽度。分类器输出八类地形状态，通过 dwell 与迟滞抑制抖动。`RAMP/STAIRS` 授权 step commitment；高障碍触发重规划，落差和无效观测限制前进。WP2、10、14、22 在距离、姿态及走廊条件满足时使用 2.1 m/s 上限；其余常规路段为 1.2 m/s，并叠加赛段限速。窄平台使用倒退转向和预设引导点；WP28→29 使用同层走廊配置。

请求前进时，速度低于 0.08 m/s 持续 2.5 s，或最佳航点距离连续 12 s 无有效改善，触发 follower 恢复。Gate 16 的策略失败重试预算独立设置为 0。[E3]

## 3. 残差运动策略与执行接口

### 3.1 网络结构与观测

| 模型 | 输入—隐藏层—输出 | 用途 |
|---|---|---|
| 官方 locomotion actor | 57D 输入，官方 runner | 普通赛段 |
| Gate 16 base | 174–512–256–128–16，ELU | 冻结基础动作 |
| Gate 16 residual | 174–512–512–256–128–16，ELU | PPO 学习的动作增量 |
| Residual critic | 174–256–128–64–1，ELU | 训练价值估计 |

部署网络为前馈 MLP。Actor 与 critic 使用同一 174D 观测，critic 不增加特权状态。Actor 的输入排列为：

\[
o_t=[0.25\omega_t^b,\ g_t^b,\ c_t,\ \tilde q_t-q_0,\ 0.05\dot q_t,\ a_{t-1},\ h_t].
\]

| 切片 | 维数 | 语义 |
|---|---:|---|
| 0:3／3:6 | 3／3 | 机身角速度／投影重力 \(R^{-1}[0,0,-1]\) |
| 6:9 | 3 | runner 处理后的前向、横向及 yaw 速度条件 |
| 9:25／25:41 | 16／16 | 默认姿态相对位置／关节速度；四轮角度先置零 |
| 41:57 | 16 | 上一步经过 guard 的实际最终 raw action |
| 57:174 | 117 | 按训练约定转换的高度图 |

Policy order 为 FL、FR、HL、HR 四组腿关节，随后为四个轮子。Actor 不接收实际机身线速度、轮接触、绝对航点或 router 状态；这些变量由上层路由和仿真评测使用。模型结构与训练身份由 checkpoint tensor shapes 和 ONNX 图共同确认。[E4]

### 3.2 门控残差与动作解码

系统采用残差策略分解 [L2]：冻结基础 actor \(\pi_b\)，学习动作增量 \(\mu_\theta\)。最终动作为：

\[
a_t=\operatorname{clip}\{\pi_b(o_t)+m_t s\odot\operatorname{clip}[\mu_\theta(o_t),-4,4],-a_{\max},a_{\max}\},
\]

其中 \(s=[1]^{12}\oplus[6]^4\)，\(a_{\max}=[8]^{12}\oplus[25]^4\)。门控 \(m_t\) 同时要求 router arming、高度 skill gate 激活和 runner engagement。高度差连续 2 帧≥0.04 m 时进入；保持至少 100 帧，连续 15 帧≤0.02 m 后退出，最长 600 帧。检测使用中央列 2–6 与边界行 4–8，保持阶段扩展到行 0–8。

下面为 `gate16_policy_runner.hpp` 中门控分支内部的原码节选；最终动作另经逐通道 guard：

```cpp
const VecXf residual =
    InferInPolicyFrame(residual_session_, current_observation_);
for (int i = 0; i < action_dim; ++i) {
  const float bounded_residual = std::clamp(residual(i), -4.0f, 4.0f);
  current_action_eigen(i) += bounded_residual * correction_scale_[i];
}
```

Hip-x、hip-y/knee 和轮速分别按 0.125 rad、0.25 rad 和 5 rad/s 每 raw unit 解码。腿 PD 为 \(K_p=80,K_d=2\)，轮 PD 为 \(K_p=0,K_d=0.6\)。训练环境每策略步执行 20 个 1 ms 物理步，逐步将腿／轮力矩裁剪至 ±50／±14 Nm。比赛配置启用 `gate16_climb_fallback`，网络前向条件上限为 0.15 m/s；stairs57、fast adapter、mirroring 与 front-tuck profile 均关闭。[E3–E4]

## 4. 强化学习训练

### 4.1 环境、初始化与训练数据

最终发布 residual 为 `speed_core` 第 120 次迭代。Base 来源为 `s10_29cm_stable.pt`；residual 从 `gate16_front_retention_20260817T121106/selected/best_checkpoint.pt` 初始化。前期 base 训练与 residual 课程的完整继承链为 **[待补 P1：训练日志、数据集及逐阶段 selected checkpoint]**。最终速度阶段采用以下发布训练配置。[E4–E5]

| 变量 | 训练设置 |
|---|---|
| 环境 | `OfficialClosedLoopResidualEnv`，官方 `S10_track.xml` |
| 台阶几何 | 台沿 x=12.646 m；floor=0.10181425 m；deck=0.47872480 m；高差 0.37691055 m |
| 初始距离／横向位置 | 距台沿 0.55–0.65 m；y=33.365 m 加均匀 ±0.02 m 偏置 |
| 距离重采样 | 85% 从 {0.55,0.60,0.65} m 抽取；15% 均匀采样 |
| 前向速度／yaw | 命令与初始实际速度均为 0.18–0.40 m/s；yaw 均匀 ±0.1396263 rad |
| 其余初始化 | 机身高度扰动、侧向速度、yaw rate 为 0；单 episode 上限 850 步／17 s |

每步 yaw 条件为 \(\operatorname{clip}(-1.5\psi,-0.8,0.8)\)。该阶段随机化入口状态，使用固定地形与动力学参数。通用 checkpoint 字段 `depth_range=[0.3,0.4]` 不参与 `official_track` 分支的地形生成。奖励和终止条件使用仿真轮心与机身状态。

### 4.2 PPO 优化与采样

Residual actor 定义对角高斯 \(\delta_t\sim\mathcal N(\mu_\theta(o_t),\operatorname{diag}\sigma^2)\)。Rollout 采样随机动作；评测与 ONNX 部署使用均值。PPO [L1] 的 likelihood ratio 对 clipping/gating 前的采样动作计算。令 \(\rho_t=\pi_\theta(\delta_t|o_t)/\pi_{\theta_{\mathrm{old}}}(\delta_t|o_t)\)，联合目标为：

\[
\begin{aligned}
\mathcal L={}&-\mathbb E\min[\rho_t\hat A_t,\operatorname{clip}(\rho_t,1-\epsilon,1+\epsilon)\hat A_t]\\
&+0.5\mathbb E(V(o_t)-\hat R_t)^2
+\alpha\mathbb E\left[\frac1{16}\sum_j(\mu_{\theta,j}(o_t)-\mu_{\mathrm{ref},j}(o_t))^2\right].
\end{aligned}
\]

\(\hat R_t\) 为 GAE return target；actor 使用 batch 标准化后的 \(\hat A_t\)。参考 actor 在阶段初始化后冻结。`zero_anchor_coef` 对应参考动作 MSE，warm-start 时其目标为初始 residual 输出。Critic 重新初始化，前 40 次迭代仅更新 critic。GAE 使用 \(\gamma=0.99,\lambda=0.95\)。成功、跌倒、发散和时间上限均返回 `done`，终止 bootstrap；未终止 episode 的 rollout batch 截断保留下一状态价值。

| 参数 | 发布配置 | 参数 | 发布配置 |
|---|---|---|---|
| 环境数×rollout | 12×32=384 transitions／迭代 | 物理执行 | 1 个 env-step worker，串行 MuJoCo |
| Actor／critic LR | \(2\times10^{-8}\)／\(10^{-5}\)，Adam | 网络更新 | CUDA；1 epoch，4 minibatches |
| PPO clip／anchor | 0.05／2.0 | 熵系数／梯度范数 | 0／1.0 |
| 初始 log-std／边界 | −3.5／[−5,0] | Seed | 377511 |
| 评测／保存周期 | 40 次迭代 | 最大迭代／early stop | 200／连续 3 次评估不改善 |

按 384 transitions／迭代计算，所选第 120 次迭代对应本阶段 46,080 次训练交互。完整训练成本为 **[待补 P2：speed_core 原始日志、GPU/CPU 型号、软件版本与墙钟时长]**；前期训练与评估交互单独计量。

### 4.3 多地形 LLC 与 privileged Teacher 研究

补充研究记录 [E7] 使用 Isaac Lab＋RSL-RL PPO，覆盖七类地形；174D 通用 MLP 从官方 57D actor 扩展，新增高度输入权重以零初始化。各地形独立 curriculum，以成功率 80%／20% 为升级／降级阈值。该分支 physics 为 200 Hz、policy 为 50 Hz，与上述 MuJoCo 残差训练的 1 ms 物理步长分属不同环境。

Teacher 的 actor／critic／action 合同为 187D／187D／16D：在 174D 上增加线速度 3D、轮接触 4D、接触力幅值 4D、静／动摩擦 2D。当前 Teacher 未通过验收；GRU Student 的 DAgger 蒸馏尚未开始。具体研究 commit、checkpoint 与原始评测产物为 **[待补 R1]**，不从这些配置推定完整训练已完成。


## 5. 奖励设计、模型选择与分布差异

### 5.1 速度目标与 episode 终止

令 \(\Delta x\) 为单步机身前进量，裁剪至 [−0.05,0.05] m；\(\Delta F,\Delta R,\Delta C\) 为前轮、后轮和机身进展的历史最大值增量；\(\Delta N\) 为历史最多越沿轮数增量。激活时的稠密奖励为：

\[
\begin{aligned}
r_t^{\mathrm{dense}}={}&35\Delta x+20\Delta F+70\Delta R+35\Delta C+6\Delta N-0.05\\
&-0.0005\operatorname{mean}(\bar\delta_t^2)
-0.0005\operatorname{mean}[(\bar\delta_t-\bar\delta_{t-1})^2].
\end{aligned}
\]

\(\bar\delta\) 为裁剪／门控后的 residual、尚未乘通道 scale。令 \([u]_0^1=\operatorname{clip}(u,0,1)\)，\(H=z_{\mathrm{deck}}-z_{\mathrm{floor}}\)。前／后轮瞬时进展为轮心水平进展与抬升高度的乘积：

\[
\begin{aligned}
F^{\mathrm{inst}}&=\tfrac12\sum_{i\in\mathrm{front}}[(x_i-x_{\mathrm{lip}}+0.45)/0.35]_0^1[(z_i-z_{\mathrm{floor}}-r_w)/H]_0^1,\\
R^{\mathrm{inst}}&=\tfrac12\sum_{i\in\mathrm{rear}}[(x_i-x_{\mathrm{lip}}+0.35)/0.35]_0^1[(z_i-z_{\mathrm{floor}}-r_w)/H]_0^1.
\end{aligned}
\]

机身项为 \(C^{\mathrm{inst}}=[(x_{\mathrm{base}}-x_{\mathrm{lip}}+0.35)/0.70]_0^1\)。各项更新历史最大值后计算增量；`best_com_progress` 指机身参考点，非质量加权质心。完整奖励为：

\[
\begin{aligned}
r_t={}&\mathbf1_{\mathrm{active}}r_t^{\mathrm{dense}}+\mathbf1_{\mathrm{success}}[300+0.5\max(0,850-n_t)]\\
&-250\mathbf1_{\mathrm{fallen}}-300\mathbf1_{\mathrm{diverged}}.
\end{aligned}
\]

局部成功要求四轮心均满足 \(x_i\ge x_{\mathrm{lip}}+0.02\) m 与 \(z_i\ge z_{\mathrm{deck}}+0.70r_w\)，其中 \(r_w=0.081\) m。跌倒定义为投影重力 z 分量>−0.20，或机身高度<floor+0.12 m。事件奖励独立累加，成功与跌倒 flags 独立计算。历史进度增量非负；前轮掉落不会回收既得奖励。`speed` 目标不加入 front-retention 或 phase reward。[E5]

### 5.2 候选选择与正式局部评测

正式矩阵为距离 {0.55,0.60,0.65} m × yaw {−12°,0°,12°} × 速度 {0.10,0.15,0.20,0.25,0.30} m/s，共 45 个固定状态。速度筛选另用 27 个状态：同一距离集合 × yaw {−8°,0°,8°} × 速度 {0.20,0.30,0.40} m/s。候选先满足两矩阵成功率不低于 baseline，再按失败惩罚完成步数选择；失败统一计为 850 步。发布选择继续约束跌倒和 drop-free 指标。最终保留 `speed_core`；后续 phase、DAgger 与融合候选未被部署。[E4–E6]

BC／失败状态聚合在开发代码中实现，失败状态使用成功轨迹近邻提供 pseudo-target。前期阶段与最终权重的完整继承关系列于 P1。归档的 `corrected_front_tuck_train.csv` 对应后续 80 次迭代实验，不能作为 `speed_core` 第 120 次迭代的训练曲线。

### 5.3 训练与执行条件对照

| 条件 | 最终训练配置 | 45-case 评测 | 部署 fallback |
|---|---|---|---|
| 距台沿 | 0.55–0.65 m | 0.55／0.60／0.65 m | 0.62–0.70 m |
| 速度条件 | 0.18–0.40 m/s | 0.10–0.30 m/s | 网络输入上限 0.15 m/s |
| 横向中心 | y=33.365 m | y=32.49969 m | 台沿中心 y=32.49969 m |
| Yaw 命令 | 反馈矫正 \(\operatorname{clip}(-1.5\psi,-0.8,0.8)\) | 固定为 0 | Router 在线生成 |
| 完成判据 | 四轮几何条件 | 四轮几何条件 | 几何＋接触＋姿态＋持续确认 |

三种条件明确不同。固定矩阵参与候选选择，属于 validation set；整圈运行评价完整控制系统。本文分别报告两类结果。[E3、E5–E6]

## 6. 策略路由、控制交接与时序

### 6.1 入口包络与状态机

Router 在 WP15→WP16 且距台沿<3 m 时进入接近阶段。官方 actor 执行移动接近，Gate 16 模型执行 shadow inference。交接要求入口包络连续满足 0.10 s；shadow 动作不写入实际动作历史。以下为部署 YAML 原码节选：[E3]

```yaml
gate16_fast_adapter_enabled: false
gate16_fallback_enabled: true
gate16_fallback_ready_distance_min: 0.62
gate16_fallback_ready_distance_max: 0.70
gate16_fallback_ready_dwell: 0.10
gate16_fallback_min_entry_speed: 0.08
gate16_fallback_max_entry_speed: 0.20
gate16_fallback_target_entry_speed: 0.18
gate16_fallback_max_lateral_error: 0.25
gate16_fallback_max_heading_error_deg: 6.0
gate16_fallback_max_yaw_rate: 0.10
```

入口另要求组合倾角≤12°。正常状态序列为 `NAVIGATE → APPROACH → ALIGN → CLIMB_READY → CLIMB → VERIFY_CLEAR → HANDOFF → NAVIGATE`。接管边沿使用实测关节位置和轮速逆解上一动作。SDK 仅在同一 tick 已获得有效 Gate 16 command matrix 时执行移动接管；未准备好时保留官方输出最多 0.10 s，随后 hold。

### 6.2 完成确认与单一控制权

部署四轮确认使用预设台沿法向：所有轮心越沿≥0.02 m，轮心高度≥0.5354248 m，terrain-contact 数≥3，倾角≤24°，进入确认时平面速度≤1.08 m/s。条件保持 0.10 s，允许 0.30 s 短暂不满足，确认超时为 5 s。

确认后依次执行：撤销 residual arming、释放 Gate 16 owner、保持实测关节姿态 0.25 s、重置官方 actor 历史、等待 `/joints/owner=official`、重置 follower，随后以 0.50 m/s 恢复 0.80 s。Owner 请求与实际 owner acknowledgment 分离；控制互斥在 SDK 关节命令出口执行。[E3]

### 6.3 失败处理与时序实现

| 触发条件 | 处理 |
|---|---|
| 启动观测不齐／运行时 age>0.5 s | 零速度；攀爬中撤销策略并恢复 |
| 观测持续 stale 3 s／倾角≥60° | Abort |
| 对齐 40 s／攀爬 20 s／12 s 无进展 | 超时或恢复；Gate 16 重试预算为 0 |
| 非有限／尺寸错误的动作 | 拒绝命令并 hold |
| Gate 16／stairs 的 CLIMB／VERIFY_CLEAR 中，任一实测关节力矩绝对值>50 Nm 持续 1 s | Abort；普通导航清空累计状态 |

观测 freshness 以 ROS callback 接收时刻记录，wheel state 无独立 age 字段。Router 使用 ROS node clock，并以固定控制 dt 累加部分 dwell；C++ runner 使用 monotonic clock；路线成绩使用 MuJoCo `sim_time`。局部 RL 控制严格为 20×1 ms 积分／策略步；完整 ROS/SDK 各模块配置为 50 Hz。[E3、E5]

端到端推理延迟、控制周期 jitter 及逐执行器力矩持续时间记录为 **[待补 P5：目标硬件上的时序与力矩测量]**。这里报告已实现的时钟和保护逻辑，不将配置频率等同于测得的实时性能。

## 7. 实验结果与研究现状

### 7.1 归档比赛系统的连续路线实验

实验使用部署版本 `3660b81`、Mac ARM64、镜像 `s10-racing:submission-3660b81-clean` 和 seed 8。Segment harness 仅设置 WP0 初态，随后连续执行生产控制栈。该实验为团队自测。[E1–E2]

| 指标 | 观测结果 | 指标 | 观测结果 |
|---|---|---|---|
| 顺序航点 | 33/33，WP0–WP32 | 路线仿真 elapsed | **392.257 s** |
| 起／终点 sim_time | 0.001／392.258 s | Recorder 墙钟时间 | 674.01 s |
| 行驶距离 | 249.94 m | 最终航点距离 | 0.177 m |
| 最大倾角 | 44.6° | 漏点／跌倒／超时／abort | 0／0／0／0 |

记录器终止时，实际 owner 为 `official`，router mode 为 `navigate`；0.18 m 终点条件触发后 launch 结束，未等待 `DONE`。完成依据为独立 timer 及按序 33 个航点事件。日志包含 Gate 16 接管、四轮确认、官方 owner acknowledgment 和导航恢复。

### 7.2 Gate 16 固定选模矩阵

下表从 `formal_summary.json` 的 45 条原始 case 重算；控制步为 20 ms。[E6]

| 指标 | 结果 | 定义 |
|---|---|---|
| 成功／跌倒／超时 | 38／2／5 | 成功率 38/45=84.44% |
| Drop-free success | 20/45=44.44% | 成功且前轮几何支撑未丢失 |
| 成功完成时间 | 6.134±3.098 s | 38 个成功 case；均值±总体标准差 |
| 失败惩罚完成时间 | 7.824 s | 失败统一按 17 s 计入 |

Drop-free 使用轮心几何条件：双前轮首次达标后至四轮完成前，不发生达标前轮数从 2 降为<2。该指标不以接触力定义。固定矩阵参与选模，属于 validation set；392.257 s 为单次整圈结果。统一协议的重复／独立测试及受控消融仍为 **[待补 P3–P4]**。历史测试与失败批次保留在附件。

### 7.3 补充多地形研究结果

以下结果按补充材料 [E7] 引用，原始 checkpoint、JSON 与运行配置尚待 R1 补齐。其评测协议与上面的 Gate 16 和整圈协议分别登记。

| 研究对象 | 补充材料记录的结果 | 当前能力边界 |
|---|---|---|
| 通用 `model_796` | 三个 seed、每个 20 场景：55%／50%／60%；平地／缓坡 12/12、坡道 8/9、湿滑 6/6 | 楼梯 1/9、路缘 0/9、窄平台 0/3；粗糙地形 6/12 |
| 楼梯专用 `model_499` | 规则楼梯矩阵 132/132；一次固定 WP18→19 回归 10.4 s | 窄走廊硬边界续训后矩阵降至 46/132；单次回归不足以估计鲁棒性 |
| 187D privileged Teacher | PPO 更新及新增输入依赖已建立；未获得满足验收的单一 checkpoint | 基础多轴运动、越障与普通地形能力未同时保持；Student 未启动 |

上述记录显示通用策略与 specialist 的能力分布不同。当前瓶颈集中在多轴控制保持、越障任务信号和跨环境部署验证，具体技术支持需求列于 §8。

## 8. 当前技术瓶颈、所需支持与改进计划

### 8.1 关键困难与技术支持需求

本节依据已归档系统及补充研究记录 [E7]，集中保留三项直接影响下一阶段的困难。困难的观测表现与根因判断分开：现有实验说明能力缺口，尚未证明某一训练因素是唯一原因。

| 关键困难 | 现有证据与影响 | 希望获得的具体帮助 |
|---|---|---|
| 多轴控制与地形技能干扰 | Teacher 的横移、双向 yaw 与前后运动未在同一 checkpoint 上通过；部分越障续训破坏平地能力 | 请运动控制／RL 专家评审任务分解：通用单网络、specialist 或残差方案；给出最小能力门槛与对照实验 |
| 越障奖励与任务成功语义不一致 | 研究记录显示抬轮 transition 的 reward／return／advantage 较差；curriculum success 与 strict traverse 事件不同 | 联合核查抬轮、越沿、离台及恢复的事件标签；设计只改变一个因素的 reward／curriculum 实验与停止条件 |
| 仿真到真实控制链尚未闭环 | 数值接口对齐不等于动力学响应一致；真实 LiDAR 高度图及执行器参数缺少完整验证 | 获取 Isaac／MuJoCo 同初态对照、实机动力学辨识及 LiDAR 同步数据，确定噪声／延迟模型与部署验收条件 |

### 8.2 已记录的改进路线

补充材料将以下工作列为原定路线，实施优先级和资源安排由团队确认：

1. **先统一任务和控制合同。** 对齐两种仿真的加速、刹停、双向转向和低台阶响应；统一 curriculum、严格越障与部署成功事件。
2. **再建立可保留基础能力的 LLC。** 先检查 stop、前后、双向横移和 yaw，再评估复杂地形；以同一 checkpoint 同时满足门槛作为验收单位。Generalist、specialist 与 residual 的最终取舍留待 R2。
3. **按条件推进部署研究。** Teacher 通过后再做 GRU／DAgger；真实地图和动力学测量形成后再标定随机化范围。按 Isaac held-out → MuJoCo 分段／连续路线 → 实机低速逐级验收。Learned HLC 的启动条件是 LLC 已冻结、地图稳定且确定性规划的不足已有实验依据。

### 8.3 待团队填写的决策与资源

::: {.fill-slot}
**[待补 R2｜目标与路线]** 最优先解决的问题、目标场景／验收指标，以及选择 generalist、specialist 或 residual 的决定。
:::

::: {.fill-slot}
**[待补 R3｜支持与执行计划]** 希望合作方提供的数据、设备或专家支持；负责人、预算、时间节点及阶段通过／停止条件。
:::

研究原始产物为 R1（§4.3、§7.3）；P1–P6 的训练、评估、测量与署名材料沿用附件待补清单。未填写内容保留占位，不预设团队决定。

**文献与证据。** [L1] Schulman et al., *PPO*, 2017，[arXiv:1707.06347](https://arxiv.org/abs/1707.06347)；[L2] Johannink et al., *Residual RL for Robot Control*, 2018，[arXiv:1812.03201](https://arxiv.org/abs/1812.03201)。[E1–E2] 归档设计／验证与原始整圈记录；[E3] `3660b81` 部署源码；[E4] checkpoint／ONNX 审计与发布选择；[E5] `5ef14fa` 训练源码；[E6] 45-case 原始数据；[E7] 用户提供的《工业界技术交流材料》。逐项来源见附件 `FACT_CHECK_ZH.md` 和 `evidence/source_manifest.json`。
