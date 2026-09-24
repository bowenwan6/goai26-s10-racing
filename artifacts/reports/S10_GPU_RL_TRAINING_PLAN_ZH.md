# S10 单 GPU Isaac Lab 训练计划

更新：2026-09-17。对象：048 号 S10 Pro（12 个腿关节位置目标 + 4 个轮速目标，双 RoboSense AIRY 雷达，可选 D435i 深度相机）。

依据：本地 [GPU 启动指南](S10_NVIDIA_RL_TRAINING_START_ZH.md)；朋友仓库 [Jack15678/rl_training](https://github.com/Jack15678/rl_training) 的 `codex/s10-stairs-57d` 分支（5637f70，2026-09-14）；第 10 节列出的论文。标“估计”的数字要在服务器上实测后再用。

---

## 0. 先看结论

1. **时间线先确认。** 本地参赛手册（7 月版，注明日程可能调整）写的总决赛是 **9 月 22–23 日**，出行表里的现场调试时段是 **9.4–9.18**。如果日期没变，决赛前只剩约 4 天 GPU 时间，而且真机测试窗口马上结束。**没在真机上逐级测过的新策略，不应该在决赛上用。** 所以计划分两条线：第 4 节是决赛前冲刺（以验证为主），第 5 节是完整训练计划（决赛后开始，约 6 周）。
2. **不要从零搭环境。** 朋友分支已经有：S10 USD、57D 合同、官方 actor ONNX 热启动、平地、转向停车、楼梯、高台阶和 174D 感知任务、严格评估脚本、ONNX 导出，以及 Docker 配置（Isaac Lab 2.3.2 + Isaac Sim 5.1 + **RSL-RL 5.0.1**）。直接用它作为基础。speedturn2000 的元数据路径 `/workspace/rl_training/logs/...` 正好是他们 Docker 的挂载路径，所以原始 `model_2000.pt` 很可能还在 Jack 的服务器上，可以找他要。
3. **先修物理，再训练。** S10 资产里轮子的 armature 是 0（M20 用的是 0.00243）。在 5 ms 物理步长下，轮子的显式 PD 在数值上处于振荡区间（见 2.1）。朋友的诊断里，5 ms 时 83.6% 的物理步轮力矩打满，2.5 ms 时为 0。在这个问题解决之前做的平地结果都不可信。
4. **感知分阶段上。** 平地提速和转向用 57D 本体感知就够，不用改部署 runner。楼梯和高台阶再做“特权高度图 teacher → 带噪声高度图的 GRU student”，高度图由双雷达的高程建图生成。深度相机放到最后，只有确认雷达近场有盲区时才加。

---

## 1. 现有资产盘点（均已核实）

| 类别 | 内容 | 状态／结论 |
|---|---|---|
| 训练框架 | 朋友分支：Isaac Lab 2.3.2、Isaac Sim 5.1、RSL-RL 5.0.1，Docker 基于 `nvcr.io/nvidia/isaac-lab:2.3.2` | 在 RTX 4090 24 GB 上跑通了 4096 个环境 |
| S10 模型 | 预转换 USD、URDF 限位、腿 PD 80/2（50 Nm）、轮 PD 0/0.6（14 Nm） | 轮子 armature 为 0，需要修正（2.1） |
| 官方 actor | `pretrained/s10/policy.onnx`，57→512→256→128→16 ELU，SHA `0ac99f…` | 热启动测试通过；AGX 上的 SDK 版本 hash 不同（`92db62c…`），要分开评估 |
| speedturn2000 | 57D，由 `General-SpeedTurn-57D` 任务热启动微调（vx 0.8–1.5，wz ±1.5） | 真机测过：前后指令到 ±2.0 m/s 的约 1 s 脉冲，+2.1 m/s 时轮速达到 31.9 rad/s，触发 30 rad/s 监控线 |
| 平地（朋友） | M20 Native 1000 次迭代：核心跟踪 50/160，停车 1/170；低速侧移、低速转向、停车都没过 | 没有候选通过严格验收，官方模型保留 |
| 楼梯（朋友） | 无参考任务奖励 + 课程，1500 次迭代；第 750 次候选九组各 32/32，约 0.36 m/s | 仅仿真结果，未上真机；同期 M20 Rough 配方为 0/352 |
| 高台阶（朋友） | 专家参考 + BC 共四轮，上台 23/27，但停稳只有 1/27 | 已关闭，未达标 |
| 我方 | MuJoCo S10 环境与 runner trace、Gate16 174D 残差、048 真机只读接口清单 | 可用于 sim2sim 和部署合同检查 |

朋友文档里测得的吞吐与显存：平地 4096 环境（5 ms×4）约 **5.05 万 transitions/s**，1000 次迭代约 33 分钟，显存采样峰值 5.8 GB；楼梯任务 4096 环境显存峰值约 14 GB。

---

## 2. 朋友约两周实验里最值得吸取的教训

### 2.1 5 ms 下的轮子数值振荡：一个具体的物理原因

显式阻尼项下，轮速每个物理步大约乘以 `1 − Kd·dt / I`。S10 轮子绕轴惯量（URDF iyy）为 0.002003 kg·m²，Kd = 0.6：

| 设置 | Kd·dt/I | 含义 |
|---|---:|---|
| dt = 5 ms，armature = 0 | **1.50** | 大于 1：每一步速度误差都会变号，也就是逐步抖动，接触后很容易打满 14 Nm |
| dt = 2.5 ms，armature = 0 | 0.75 | 单调衰减 |
| dt = 5 ms，armature = 0.00243（M20 值） | 0.68 | 单调衰减，而且能保留 5 ms 的吞吐 |

这和朋友的实测吻合：5 ms 下 83.6% 的步数力矩饱和、轮速 RMS 29 rad/s；2.5 ms 下两项都接近 0。官方模型在 5 ms Native 评估里 0/160 以及“零指令 5 秒漂移 3.1 m”，很可能主要来自这个物理问题，而不是策略本身差。Isaac Lab 文档也明确建议用 armature 抑制显式 actuator 的数值不稳定（[Actuators 文档](https://isaac-sim.github.io/IsaacLab/main/source/overview/core-concepts/actuators.html)）。

**决定：** 在拿到真实电机转子惯量 × 减速比² 之前，统一使用 **2.5 ms×8**（朋友已经验证过，策略频率仍是 50 Hz）。armature 辨识出来后，再考虑回到 5 ms×4，训练速度大约翻倍。

### 2.2 热启动好 actor 的方式不对，会把它训坏

Native 那一轮从官方 actor 热启动，但用了 `init_noise_std = 1.0`、lr = 1e-3、全新 critic。结果第 356 次迭代平均 KL 达到 11.4，单个 minibatch 最高 32.7。原因是：探索噪声相对动作尺度太大（腿 1 个单位 = 0.125–0.25 rad，轮 1 个单位 = 5 rad/s），新 critic 给出的 advantage 又很吵。后来固定小学习率（3e-5）的那轮，std 收缩后 KL 仍然超过 0.2。高斯分布的 KL 大约按 (Δμ/σ)² 放大，σ 很小时一点点均值变化就会触发 KL 门槛。

**做法：** 探索 std 起点 0.2–0.3，并设下限；先只训 critic 50–100 次迭代再放开 actor；lr 起点 1e-4，用 adaptive KL（0.01）；单次 KL 尖峰只记录，不作为终止条件。或者用 2.3 的零初始化残差结构。

### 2.3 预算太小、一次改动太多、验收门槛和实际需求脱节

- 每轮只训 400–1000 次迭代就下结论，而上游 M20 Flat 默认 5000 次、Rough 20000 次。
- 各轮之间同时改了 dt、奖励、命令分布和初始化，还遇到过命令时钟 reset 重复扣减的 bug，因果很难拆开。
- “5 秒 / 3 cm 停车”这种门槛，官方模型在同一仿真里也过不了。在没有真机标定前，它测不出比赛需要的东西。
- 纯侧移指令只占约 2% 的样本，低速侧移自然学不好。巡逻导航其实主要只需要 vx 和 wz。

**做法：** 一种配方至少训 2000–3000 次迭代再评判，每 250 次存一个 checkpoint；每轮只改一个变量；验收用“同一物理条件下相对官方模型的提升”加上贴近比赛的指标（3.3）；vy 限制在小范围、低占比。

### 2.4 任务设计比套用通用配方更重要

楼梯方向：通用 M20 Rough 配方 0/352；专门设计入口、进度奖励、按级计数、课程和真实前缀之后，达到 32/32。高台阶方向：参考轨迹 + BC 能让机器人上台，但停不稳。所以复杂地形优先做“任务奖励 + 课程 + 特权 teacher”，不要继续在专家轨迹 BC 上投入。

---

## 3. 服务器 Day 0：连接、环境与验收

### 3.1 连接（需要你自己操作一次）

在聊天里发出来的 root 密码应视为已泄露。我不会用密码登录。请在 Mac 终端执行下面这条命令，按提示输入一次密码，把现有公钥 `~/.ssh/id_ed25519.pub` 装到服务器上：

```bash
ssh-copy-id -i ~/.ssh/id_ed25519.pub -p <GPU_PORT> root@<GPU_HOST>
```

然后在 `~/.ssh/config` 里加一个别名，并到云平台控制台修改 root 密码：

```sshconfig
Host gpu-s10
    HostName <GPU_HOST>
    Port 30567
    User root
    IdentityFile ~/.ssh/id_ed25519
    IdentitiesOnly yes
```

`ssh gpu-s10` 能免密登录之后，我就可以做只读检查和环境搭建。

### 3.2 只读体检（决定走 Docker 还是 pip 安装）

```bash
ssh gpu-s10 'nvidia-smi; cat /etc/os-release | head -3; ldd --version | head -1; nproc; free -g; df -h / /root; docker info 2>&1 | head -3; ls /dev/dri 2>&1; cat /proc/1/cgroup | head -3'
```

| 检查项 | 通过条件 | 不通过怎么办 |
|---|---|---|
| GPU | 带 RT 核的 RTX 卡（4090、L40S、A6000、RTX 6000 Ada 等），显存 ≥ 16 GB | A100/H100 跑不了 Isaac Sim，只能做 PyTorch 蒸馏或导出 |
| 驱动 | 满足 Isaac Sim 5.1 要求（朋友服务器用 580.173.02 跑通） | 找平台换镜像或驱动，不在容器里自己装 |
| 容器 | 端口 <GPU_PORT> + root 看起来像容器云，通常没有 Docker | 走 pip 路线：conda + `isaacsim[all,extscache]==5.1.0` + Isaac Lab v2.3.2，再把 `rsl-rl-lib==5.0.1` 装进去 |
| Vulkan | `create_empty.py --headless` 能正常退出 | 容器缺 `libnvidia-gl` 或 ICD 时 Isaac Sim 会报设备丢失，需要平台支持 |
| 磁盘 | ≥ 150 GB | Isaac 缓存加日志很占空间，定期清理 `/tmp/IsaacLab/usd_*` |

### 3.3 环境与冒烟测试

1. 按启动指南 7.3 安装，但 **RSL-RL 固定为 5.0.1**。启动指南 7.2 写的 3.1.2 与朋友的训练脚本和 checkpoint 格式不兼容，以本计划为准。
2. `git clone -b codex/s10-stairs-57d https://github.com/Jack15678/rl_training.git`，然后 `pip install -e source/rl_training`，再用 `scripts/tools/list_envs.py` 确认 S10 任务都注册上了。
3. 跑 `test_onnx_actor_warmstart.py`：ONNX 与 PyTorch actor 数值一致，57→174 的列扩展正确。
4. **物理验收（新增，必须在任何 PPO 之前通过）：** 官方 actor 零指令站立 10 s，64 个环境，分别在 2.5 ms×8 和 5 ms×4 下跑。要求：轮力矩饱和比例 < 1%，轮速 RMS < 0.1 rad/s，机身漂移 < 5 cm。再做前进 1.0 m/s 和原地转向 1.0 rad/s 各 10 s，记录跟踪误差。
5. 吞吐基准：`Flat-TurnStop-57D` 用 4096 环境跑 20 次迭代，记录 transitions/s 和显存。第 6 节的时间估计以这次实测为准。

---

## 4. 决赛冲刺线（9/17–9/21，前提是决赛日期没变）

目标不是训练新模型，而是**在决赛前把“用哪个 57D 模型”这件事搞清楚、做对**。

| 天 | 工作 | 产出 |
|---|---|---|
| D0（今天） | 3.1–3.3：连接、安装、物理验收 | 可用的 2.5 ms S10 环境和吞吐数据 |
| D1 | 回归评估同一批候选：本地 SDK、AGX SDK、speedturn2000、Native1000、TurnStop 最新（找 Jack 要）、楼梯 750。统一 2.5 ms 物理，关闭随机化；同时在 MuJoCo 跑 sim2sim | 按比赛相关指标排序的表 |
| D1–D2 | 可选：只有确认还有真机测试时间，才跑一次 SpeedTurn v2 微调（配置见 5.2，从 speedturn2000 actor 热启动，2000 次迭代，估计约 2 小时） | 候选模型，默认不上决赛 |
| D2–D3 | 导出 ONNX，在 AGX 上用合成输入测试 ORT 延迟，再做 shadow 模式（只算不发） | 部署就绪检查 |
| 决赛 | 只使用真机上逐级测试通过的模型 | — |

D1 用的比赛相关指标：
- 持续直行的最高合格速度（跟踪误差 < 10%，无打滑、无翻倒），**以及此时的峰值轮速**。真机固件和监控线是 30 rad/s，按 0.081 m 轮半径约等于 2.4 m/s。瞬态有过冲，速度指令建议先封顶 1.8 m/s。
- 转弯：vx = 1.0/1.5 m/s 时 wz = ±0.5/±1.0 的实际转弯半径误差和侧倾。
- 停车：从 1.5 m/s 到停下的距离和时间。
- **巡逻代理赛道：** 用简单 pure-pursuit 外环沿一串带 90° 弯的航点跑，记录完成时间和横向误差。比赛按用时计分，这项最接近真实得分。

---

## 5. 完整训练计划（决赛后，约 6 周）

### 5.0 总路线

```text
P1 物理辨识 + 评估框架（第 1 周）
 └─ P2 平地提速/转向/停车 base v2（57D MLP，部署不用改 runner）（第 1–2 周）
     ├─ P3 本体感知粗糙地形：鹅卵石/坡道，特权 teacher → 本体历史 student（第 2–3 周）
     └─ P4 感知专项：楼梯/高台，特权高度图 teacher → 带噪声高度图 GRU student（第 3–5 周）
          └─ P5 集成、导出、runner 改造、真机逐级验证、策略切换（第 5–6 周）
```

### 5.1 P1：物理辨识与评估框架（先做，不跑 PPO）

1. **轮子 armature：** 先问厂商要轮电机转子惯量和减速比（armature = J_rotor·N²）。拿不到的话，用 048 已有的 200 Hz 轮速阶跃记录（speedturn 10%–100% 那批测试）做轨迹匹配：在仿真里扫 armature ∈ [0, 0.005]、关节摩擦、轮子有效半径，找与真实 `dq` 曲线误差最小的一组。
2. **腿关节：** 挂起机器人，做小幅位置阶跃（只动一条腿，保持现场安全规程），匹配 kp/kd 的实际响应和延迟。
3. **整机称重：** 带 AGX、双雷达、电池的实际质量和质心偏移。URDF 是 18.99 kg，实车加装了设备，一定会更重。
4. **固定评估套件：** 在朋友的 `s10_flat_eval.py` 基础上加入 4 节的比赛指标和巡逻代理赛道。每次评估都把**官方模型在同一物理条件下一起跑**，结果以相对提升报告。
5. **sim2sim：** 每个候选在 Isaac 和 MuJoCo 两边跑同一组指令，对比轨迹，差异大的先查模型。

**通过条件：** 3.3 第 4 步的物理验收通过；辨识出的参数写进 S10 资产配置，并记录来源。

### 5.2 P2：平地 base v2（提速、转弯、停车）

**结构（二选一，推荐 A）：**
- A. **零初始化残差：** 冻结官方（或 speedturn2000）actor，外加一个 57D→16D 残差 MLP（256-128，最后一层初始化为 0）。导出时和 base 合并成一个 57→16 的 ONNX，**现有 SDK runner 不用改**。
- B. 全网络微调：沿用朋友的热启动工具，但必须加 critic 预热和小 std（2.2）。朋友仓库里已有 `behavior_retention_ppo.py`，可以在低速状态上约束与冻结 base 的输出差。

**观测：** actor 用 57D 合同（command 语义要先和 AGX runner 的 ×1.5/×0.5/×0.6 缩放统一，见 8）；critic 用 57D + 真实机身线速度 3 + 四轮接触力 4 + 摩擦 1 + 附加质量/质心 4。

**命令采样（起点）：**

| 比例 | 类型 | 范围 |
|---:|---|---|
| 20% | 停车 | 全零，包括运动中突然归零 |
| 15% | 原地转向 | wz ∈ ±[0.3, 1.5]，vx = 0 |
| 50% | 前进带转弯 | vx ∈ [0.3, 1.8]，wz 满足 \|vx·wz\| ≤ 1.5 m/s² 的侧向加速度预算 |
| 10% | 倒车 | vx ∈ [−1.0, −0.2]，wz 小 |
| 5% | 小幅侧移 | vy ∈ ±0.3（巡逻导航不用侧移时可以设为 0） |

每 3–8 s 随机重采样，混合阶跃变化和限加速度的斜坡变化。速度上限按课程放开：1.0 → 1.5 → 1.8 m/s，只有当跟踪误差、打滑和轮速峰值都达标时才升一级。

**奖励（权重是调参起点）：**

| 项 | 起点 | 说明 |
|---|---:|---|
| 线速度跟踪 exp | +5 | 容差随速度放宽，比如 σ = 0.25 + 0.15·\|v_cmd\| |
| yaw 速度跟踪 exp | +3 | — |
| 停车：机身水平速度² / yaw² / 实际轮速² | −5 / −2 / −0.02 | 只在停车指令下生效（沿用 TurnStop） |
| 轮速软上限 relu(\|ω\| − 24)² | −0.1 | 给真机 30 rad/s 保护线留余量 |
| 轮子侧向打滑（接触点垂直于轮面的速度²） | −0.5 | 防止转弯时靠横滑 |
| 竖直速度² / roll-pitch 角速度² | −2 / −0.05 | — |
| 姿态偏离参考倾角 | −5 | 参考侧倾约 atan(vx·wz/g)，允许转弯内倾 |
| 动作变化率 / 平滑度 / 腿力矩² | −0.01 / −0.025 / −2.5e-5 | 沿用 M20 |
| 非轮部位接触 | −1，机身触地直接终止 | — |
| 残差幅值²（仅方案 A） | −0.01 | 高速档逐步减弱 |

**随机化（名义配置先跑通，再逐项加）：** 地面摩擦 0.4–1.2；机身附加质量 −1 到 +3 kg（以实测为中心）；质心 ±3 cm；kp/kd ×0.9–1.1；电机强度 ×0.9–1.1；armature ×0.7–1.3（以辨识值为中心）；控制延迟 0–10 ms；每 10–15 s 推一次，±0.5 m/s；观测噪声沿用 M20。

**PPO：** 4096 环境，每次迭代 24 步，5 个 epoch × 4 个 minibatch，γ = 0.99，λ = 0.95，lr 1e-4（adaptive KL 0.01），std 起点 0.25（设下限），先做 100 次 critic 预热，**3000 次迭代**，每 250 次保存。先用固定 seed 调通配方，定稿时跑 3 个 seed。

**验收：** 同一物理条件、同一套评估下，巡逻代理赛道用时比官方和 speedturn2000 更短，横向误差、停车距离、翻倒率不变差，峰值轮速 < 26 rad/s；MuJoCo 里结果方向一致。之后才进入真机逐级测试（启动指南第 11 节）。

### 5.3 P3：鹅卵石和坡道（本体感知 + 历史）

- 地形：平地 20%，鹅卵石 heightfield（起伏 2–8 cm）40%，坡道 0–20°（上坡、下坡、横坡都要有）40%，按地形等级做课程。
- teacher：57D + 特权信息（线速度、接触、摩擦、机身下方局部高度 117D），PPO，从 P2 的策略热启动。
- student：57D + GRU(128)，用 RSL-RL 5 自带的 Distillation runner 做 DAgger（支持 RNN student，见 [Isaac Lab RL 文档](https://isaac-sim.github.io/IsaacLab/main/source/api/lab_rl/isaaclab_rl.html)）。做法参考 RMA、DreamWaQ 和 FLORES（第 10 节）。
- 部署：这一步开始 ONNX 多了 hidden_in/out，**runner 需要改**（启动指南 4.4）。如果只追求简单，可以先试“57D × 5 帧历史”的 MLP student，但同样要改 runner 的输入维度。
- 验收要分组看：平路、上坡、下坡、横坡分别统计成功率、跟踪误差和打滑，不能用一个总成功率盖过去。

### 5.4 P4：楼梯和高台（雷达高度图）

**仿真侧**
1. **teacher（特权）：** 57D + 理想高度扫描。先沿用朋友 174D 合同里的 13×9、间距 15 cm、x ∈ −0.6..1.2 m、yaw 对齐，这样导出和部署 trace 工具都能直接用。critic 再加任务进度和接触信息。楼梯从朋友那套**无参考任务**（32/32 的那版）出发，只加高度图输入：扩展列初始化为 0，保证第 0 次迭代的行为和原策略一样。
2. **课程：** 楼梯台阶 8 → 18 cm，踏面 25–40 cm，含偏角入口，同时保留 20% 平地。高台 10 → 30 cm，只奖励有效进展，检测来回进退刷分。高台优先级排在楼梯之后。
3. **student（真实可得的输入）：** 57D + 带噪声的高度图 + GRU(128)，用 DAgger 蒸馏。噪声模型参考 Miki 2022：
   - 每个 episode 一个整体高度偏移 ±5 cm，每格独立噪声 2–3 cm；
   - 水平漂移 5–10 cm，地图延迟 50–200 ms（雷达 10 Hz）；
   - 10–30% 的格子随机缺失，台阶边缘后方的遮挡区做掩码；
   - 偶尔整张图中断 0.2–1 s，用来检验本体历史能不能兜底。
   - 注意：Isaac Lab 的 RayCaster 只和静态地形 mesh 求交，**模拟不出机身和腿对雷达的遮挡**。这部分只能用掩码近似，再用真机录的地图来标定。
4. 缺测格子需要给策略一个标记。之后可以把合同扩展为“高度 + 有效位”（247 格 10 cm 网格时是 551D），但这是新接口，导出、runner 和 trace 都要一起改、一起测。

**真机侧（和训练并行推进）**
1. 双 AIRY → 时间同步 → 运动去畸变 → 高程建图（参考 [elevation_mapping_cupy](https://github.com/leggedrobotics/elevation_mapping_cupy)，要先确认 ROS 2 Jazzy 和 AGX 上能跑）→ 50 Hz 采样成和仿真**完全相同**的网格（原点、yaw 对齐、行列顺序 `yx`、裁剪到 [−1, 1]）。
2. 在真实楼梯和台阶前，分别在静止、慢速接近、前轮上台时录地图，和仿真理想扫描对比，用来标定第 3 步的噪声参数。
3. 测端到端延迟：雷达时间戳 → 地图 → 策略输入。

**深度相机（D435i）：** 先用录到的点云确认雷达在前轮正前方是否有近场盲区。只有盲区确实存在，才考虑把深度相机的点并入同一张高程图。**不建议在单 GPU 上做深度图像端到端训练**（比如 StairMaster 那种）：Isaac 里并行渲染相机会让吞吐掉一个数量级。

### 5.5 P5：集成与真机

- 导出合并后的 ONNX 和 `policy-contract.json`，用 trace 工具做逐元素对比（朋友已有 `verify_s10_runner_trace.py`）。
- runner 增加 hidden 状态的管理和复位规则，以及高度图输入与有效位。
- 策略切换：平地 base 和楼梯/高台专项之间，进入和退出条件要明确，并做切换测试。
- 真机顺序：AGX 上合成输入 → shadow 模式 → 架空 → 低速 → 停车和转弯 → 提速，最后才上地形（启动指南第 11 节）。

---

## 6. GPU 时间预算（单卡，估计值）

以朋友 4090 实测的 5 ms×4 平地约 5 万 transitions/s 为基准。2.5 ms×8 物理计算翻倍，估计总吞吐会降到原来的 0.5–0.6，以 Day 0 实测为准。

| 阶段 | 规模 | 估计 GPU 时间 |
|---|---|---|
| Day 0 安装与验收 | — | 半天（大部分在下载和编译着色器缓存） |
| 回归评估（6 个候选 × 全套） | 64–256 环境 | 2–3 小时 |
| P2 调配方（约 4 次 × 3000 迭代） | 4096 | 约 12–20 小时 |
| P2 定稿（3 个 seed） | 4096 | 约 9–15 小时 |
| P3 teacher + student | 4096 | 约 1.5–2 天 |
| P4 楼梯 teacher + student + 噪声调试 | 4096（显存约 14 GB） | 约 3–4 天 |
| P4 高台 | 4096 | 约 2 天 |

单卡上训练任务**串行排队**：同时跑两个训练只是分摊算力，总吞吐不会增加。短评估可以插在训练的 checkpoint 间隙。24 GB 显存足够跑 4096 环境；16 GB 显存跑楼梯时可能要降到 2048。

---

## 7. 实验管理规则

1. 每个 run 固定四样东西：代码 commit、资产和模型的 SHA、完整的 env/agent yaml、评估套件版本。朋友的 `train.py` 已经会自动保存 params 和 git diff。
2. 每个 run 只改一个变量。对照组必须用相同的迭代数和 seed。
3. PPO 之前必须先过物理验收（3.3 第 4 步）。改了资产或 dt 之后要重新验收。
4. 至少训到 2000 次迭代才评判一个配方。每 500 次跑一次固定评估，看曲线，不看单点。
5. 官方模型在每次评估里都作为同条件参照。“比官方好”只在同一物理条件下成立。
6. KL、std、LR、value loss、每个奖励分项、轮力矩饱和比例、峰值轮速都要进 TensorBoard。
7. 和 Jack 共享结果，避免重复跑同一个实验；他的 TurnStop 续训结果（9/14 开始）要先拿到。

---

## 8. 风险与待确认

| 项 | 为什么重要 | 找谁 / 怎么查 |
|---|---|---|
| 决赛日期和真机测试窗口 | 决定第 4 节是否执行，以及新模型能不能上场 | 组委会最新通知 |
| GPU 型号是否有 RT 核、容器里能否用 Vulkan | 没有就跑不了 Isaac Sim | 3.2 体检 |
| 轮电机转子惯量、减速比 | 决定用 5 ms 还是 2.5 ms，以及轮子动力学准不准 | 厂商，或 5.1 的辨识 |
| command 缩放合同 | AGX runner 对指令乘 ×1.5/×0.5/×0.6，Mac 上的 runner 不乘；训练和部署必须一致 | 启动指南 4.2，先定下一个统一的规则 |
| 30 rad/s 轮速保护的实际生效方式 | 直接限制最高速度 | 厂商，以及 048 的故障规则文件 |
| speedturn `model_2000.pt` 和训练配置 | 有了它就能真正续训（带 critic 和 optimizer） | Jack 服务器上的 `logs/rsl_rl/deeprobotics_s10_general_57d/2026-08-19_09-12-07_speedturn_from1400_v1/` |
| 厂商模型的使用许可 | 能否基于它微调再部署 | 厂商 / 组委会 |

启动指南里有两处需要按本计划更正：7.2 的 RSL-RL 版本（3.1.2 → 5.0.1），以及 9.2 建议的 `physics_dt = 0.005`（armature 没修之前应改为 2.5 ms×8）。

---

## 9. 今天就能做的清单

- [ ] 你：`ssh-copy-id` 装公钥，加 `gpu-s10` 别名，改 root 密码
- [ ] 确认决赛日期，以及还有没有真机测试时间
- [ ] 找 Jack 要：TurnStop 最新结果、speedturn `model_2000.pt` 及其配置、楼梯 750 的 checkpoint
- [ ] 服务器体检（3.2）→ 安装（3.3）→ 物理验收
- [ ] 6 个候选模型在 2.5 ms 物理下的回归评估表

---

## 10. 参考论文与资料

**大规模并行训练与 sim2real**
- Rudin et al., *Learning to Walk in Minutes Using Massively Parallel Deep RL*, CoRL 2021. [arXiv:2109.11978](https://arxiv.org/abs/2109.11978)：几千个并行环境加课程的基本范式。
- NVIDIA et al., *Isaac Lab: A GPU-Accelerated Simulation Framework for Multi-Modal Robot Learning*, 2025. [arXiv:2511.04831](https://arxiv.org/abs/2511.04831)
- Kumar et al., *RMA: Rapid Motor Adaptation for Legged Robots*, RSS 2021. [arXiv:2107.04034](https://arxiv.org/abs/2107.04034)：特权 teacher 加本体历史估计环境参数。
- Nahrendra et al., *DreamWaQ*, ICRA 2023. [arXiv:2301.10602](https://arxiv.org/abs/2301.10602)：用本体历史隐式估计地形，不用蒸馏。

**轮腿机器人**
- Lee, Bjelonic et al., *Learning robust autonomous navigation and locomotion for wheeled-legged robots*, Science Robotics 2024. [DOI](https://www.science.org/doi/10.1126/scirobotics.adi9641)：轮式行驶与行走之间平滑切换，特权学习。
- Chamorro et al., *Reinforcement Learning for Blind Stair Climbing with Legged and Wheeled-Legged Robots*, 2024. [arXiv:2402.06143](https://arxiv.org/abs/2402.06143)：纯本体感知爬楼梯的任务设计。
- *A Reconfigured Wheel-Legged Robot for Enhanced Steering and Adaptability*（FLORES）, 2025. [arXiv:2507.22345](https://arxiv.org/abs/2507.22345)，[代码](https://github.com/ZhichengSong6/FLORES)：12 个腿位置加 4 个轮速的动作空间，和 S10 一致，带历史估计。
- *CTBC: Contact-Triggered Blind Climbing for Wheeled Bipedal Robots*, 2025. [arXiv:2509.02986](https://arxiv.org/abs/2509.02986)：接触触发的抬腿引导，训练中逐步撤掉。
- Zhao et al., *AWARE: Unleashing the Agility of Wheeled-Legged Robots for High-Dynamic Reflexive Obstacle Evasion*, 2026. [arXiv:2604.23761](https://arxiv.org/abs/2604.23761)：在 **M20** 上用 Isaac Lab 训练并部署到真机，和 S10 最接近。
- Matsuzawa et al., *Long-Distance Real-World Navigation of the Legged-Wheeled Robot Go2-W*, 2026. [arXiv:2606.21387](https://arxiv.org/abs/2606.21387)：轮式行驶时负载集中在髋关节导致过热，用奖励分散负载。长时间巡逻也要关注这一点。

**感知运动控制（高度图）**
- Miki et al., *Learning robust perceptive locomotion for quadrupedal robots in the wild*, Science Robotics 2022. [arXiv:2201.08117](https://arxiv.org/abs/2201.08117)：特权 teacher 加带噪声高度图的 belief encoder student，噪声模型是 5.4 的主要参考。
- He et al., *Attention-based map encoding for learning generalized legged locomotion*, Science Robotics 2025. [DOI](https://www.science.org/doi/10.1126/scirobotics.adv3604)
- Zhang et al., *AME-2*, 2026. [arXiv:2601.08485](https://arxiv.org/abs/2601.08485)：训练时在并行仿真里在线建图，处理噪声和遮挡。
- Tang et al., *StairMaster*, 2026. [arXiv:2606.25765](https://arxiv.org/abs/2606.25765)：深度图像爬镂空楼梯。作为深度相机路线参考，单 GPU 上成本高。

**工具**
- [Isaac Lab Actuators](https://isaac-sim.github.io/IsaacLab/main/source/overview/core-concepts/actuators.html)：显式 actuator 与 armature 的数值稳定性说明。
- [elevation_mapping_cupy](https://github.com/leggedrobotics/elevation_mapping_cupy)：GPU 高程建图。
