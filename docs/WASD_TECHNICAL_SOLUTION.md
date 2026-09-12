# S10 轮足机器人 WASD 手动控制：技术方案

> 历史版本说明：本文适用于 [cc93d39](https://github.com/bowenwan6/goai26-s10-racing/tree/cc93d39f8a939c88627f73534ca42c30d7705746)。当前分支已采用 main 的比赛实现，旧 WASD、键盘切换策略及 replay_waypoint.sh 入口已停用；当前操作见 [项目 README](../README.md)。

[项目介绍](WASD_PROJECT_INTRODUCTION.md) · [Demo 说明](WASD_DEMO_GUIDE.md)

## 总体架构

手动模式由 WSL 中的控制/物理进程和 Windows 原生 MuJoCo viewer 共同组成：

```text
Windows 原生 MuJoCo viewer
  ├─ 前台窗口键盘 Hook
  │    └─ UDP 18778：单字节项目按键
  └─ UDP 18777：接收 23 维 qpos，60 Hz 显示
             ▲
             │
             ▼
WSL / ROS 2
  sim_node ── /keyboard/key ──> KeyboardInterface
     │                               │
     │ MuJoCo physics @ 1 kHz        │ UserCommand
     │                               ▼
     └<── /JOINTS_CMD ── RL policy / 状态机 @ 50 Hz
                 │
                 └─ /ground_truth/odom + /JOINTS_DATA
                                      │
                                      ▼
                                  viewer_node
```

核心设计是“控制和物理解耦于显示”：

- `sim_node` 在 WSL 内无窗口运行，负责真实的 MuJoCo 物理计算；
- `rl_deploy` 运行状态机和 ONNX locomotion policy；
- `viewer_node` 从 ROS 2 获取机身位姿与 16 个关节位置，组合成 23 维 `qpos`；
- `viewer_node` 以默认 60 Hz 通过 UDP 将最新 `qpos` 发给 Windows viewer；
- Windows viewer 只做正向运动学和渲染，不参与控制和物理积分。

UDP 在这里采用“只关心最新状态”的方式：偶尔丢失一帧不会阻塞物理仿真，下一帧会直接覆盖旧画面。

## 键盘输入链路

Windows viewer 使用低级键盘 Hook，仅在该 viewer 是前台窗口时处理项目按键。对于未按下 `Ctrl` 或 `Alt` 的项目快捷键，Hook 会：

1. 将按键编码为带 `S10K` 标识的 UDP 数据包；
2. 发往 WSL 的控制端口 `18778`；
3. 返回“已处理”，阻止该按键继续传给 MuJoCo。

因此，按 `W` 不会再切换 MuJoCo 透视，按 `C` 也不会再打开 MuJoCo 的接触力可视化。程序启动时不会自动输入 `C`；进入 RL 模式必须由用户主动按下 `C`。

`sim_node` 收到合法按键后发布 `/keyboard/key`，`KeyboardInterface` 再统一处理来自 Windows viewer 和启动终端的输入。这样两种输入方式最终走同一套状态机，不需要维护两套控制逻辑。

Windows 端只发送按下事件。持续按键依靠操作系统的键盘重复事件刷新；Linux 端若连续 500 ms 没再收到某个速度键，会自动将其释放，防止网络中断后机器人持续运动。

## WASD 控制语义

WASD 控制的是机体坐标系下的期望速度：

```text
v_cmd = [v_forward, v_lateral, yaw_rate]
```

当前手动控制上限如下：

| 按键 | 指令 | 当前值 |
|---|---|---:|
| `W` | 向前 | `+1.0 m/s` |
| `S` | 向后 | `-1.0 m/s` |
| `A` | 向左横移 | `+0.6 m/s` |
| `D` | 向右横移 | `-0.6 m/s` |
| `Q` | 逆时针转向 | `+1.0 rad/s` |
| `E` | 顺时针转向 | `-1.0 rad/s` |

这些数值是 policy 的期望速度，不等于机器人一定能达到的实测速度，也不是四个轮子的角速度。地形、轮地摩擦、姿态限制和 policy 能力都会影响最终速度。

普通 RL 模式下，四轮实际目标速度由 policy 根据期望速度和机器人状态共同输出。因此不能在普通模式下把 `W` 简单理解为“给四轮写固定转速”。这种设计的优点是 policy 可以同步调整髋、膝和轮子，在加速和转向时维持平衡。

不带 `--manual` 的自动导航使用另一组上限：当前最大期望前进速度为 `1.6 m/s`，并会根据朝向误差、目标距离和加速度限制动态降低。它与手动模式 `W` 的 `1.0 m/s` 上限相互独立。

## 状态机与模式控制

机器人必须进入正确状态后才会响应移动指令：

```text
等待/趴下/阻尼 ── Z ──> 站起 ── C ──> RL 控制
                                      │
                                      ├─ H：平地高速模式
                                      ├─ P/L：切换台阶 policy
                                      └─ V：高台阶实验模式
```

| 按键 | 生效状态 | 功能 |
|---|---|---|
| `Z` | 等待、趴下或阻尼 | 开始站起 |
| `C` | 任意可切换状态 | 请求进入 RL 控制，并关闭高速覆盖 |
| `X` | 站立或 RL | 趴下 |
| `R` | 任意 | 进入关节阻尼 |
| `0` | 任意 | 重置仿真并启动自动恢复流程 |
| `H` | RL | 开关平地低重心高速模式 |
| `P` | 已加载对应模型 | 默认 policy / 上台阶 policy 切换 |
| `L` | 已加载对应模型 | 默认 policy / 下台阶 policy 切换 |
| `V` | RL | 进入实验性高台阶接近模式 |
| `M` | 高台阶模式 | 切换前膝展开/折叠，并改变后膝与轮速目标 |

旧的 `B/N` 分阶段控制当前已禁用，不会从 Windows viewer 转发，也不应再用于 Demo。

## 平地高速模式

在 RL 状态按 `H` 后，系统不会瞬间把轮速拉满，而是分两段平滑执行：

1. 约 1 秒内降低机身重心，髋/膝目标逐渐过渡到低姿态；
2. 姿态到位后，再约 1 秒把四个轮子的目标速度渐变到 `20 rad/s`。

默认目标为：

- 前腿髋/膝：`-0.75 / +1.50 rad`；
- 后腿髋/膝：`+0.75 / -1.50 rad`；
- 四轮目标速度：`-20 rad/s`，负号是当前模型中“向前滚动”的关节方向；
- 轮速阻尼增益：`2.0`。

再次按 `H` 时顺序相反：先降低轮速，再恢复普通 policy 姿态。切换到台阶 policy 时会自动关闭高速模式。

可在启动前通过以下环境变量标定：

| 环境变量 | 默认值 | 含义 |
|---|---:|---|
| `S10_HIGH_SPEED_HIP` | `0.75` | 高速姿态髋角绝对值 |
| `S10_HIGH_SPEED_KNEE` | `1.50` | 高速姿态膝角绝对值 |
| `S10_HIGH_SPEED_WHEEL` | `20.0` | 高速轮速绝对值，rad/s |
| `S10_HIGH_SPEED_WHEEL_KD` | `2.0` | 高速轮速阻尼增益 |
| `S10_HIGH_SPEED_RAMP` | `1.0` | 每段渐变时间，s |

示例：

```bash
S10_HIGH_SPEED_WHEEL=16 S10_HIGH_SPEED_RAMP=1.5 scripts/run_race.sh --manual
```

## 高台阶实验模式

`V` 和 `M` 是针对约 0.37 m 台阶的实验性硬编码控制，不属于普通 WASD 行驶流程。

- 按 `V`：先把腿调整到支撑姿态，再让前后轮以默认 `4 rad/s` 向立面接近；
- 按 `M`：前膝逐渐展开到 `1.90 rad`，后膝逐渐调整到 `0.35 rad`，四轮渐变到 `8 rad/s`；
- `7/8`：每次将障碍模式轮速减小/增加 `1 rad/s`；
- `1/2`、`3/4`、`5/6`、`[/]`：分别调整阶段时间、折叠髋角、折叠膝角和前腿下压量。

这些参数在进入高台阶状态时读取，因此应在按 `V` 前调整；已经进入该状态时修改，需要退出后重新进入才会生效。该模式直接生成关节命令，并带有机身横滚、俯仰和持续过载保护。完整爬台阶动作仍处于实验阶段，Demo 中应先在低速和可复位环境下使用。

## 接触与 waypoint 可视化

Windows viewer 会在每次收到新姿态后运行一次 `mj_forward` 并检查 MuJoCo contact：

- 绿色轮子：该轮所在 body 当前至少有一个接触；
- 红色轮子：该轮当前没有接触。

这表示“是否存在接触”，不代表接触力足够大，也不代表接触方向一定能产生有效牵引力。轮子显示绿色但转动无位移，仍可能由法向力不足、摩擦饱和、碰撞卡边、轮速方向错误或腿部关节饱和引起。

MuJoCo 原生的接触点标记默认关闭，避免大尺寸调试图形遮挡机器人。赛道 waypoint 的有效判定采用水平面 0.2 m 半径，并在场景地面显示对应范围。

## 安全与容错

- 速度键 500 ms 无刷新自动归零；
- Windows viewer 30 秒收不到首帧会退出，连接后连续 2 秒无帧也会退出；
- viewer 数据包必须长度正确且全部为有限数值，否则丢弃；
- 手动关节 UDP 模式带 0.5 秒心跳超时，超时进入阻尼；
- 高台阶模式检测姿态和关节持续过载，超过阈值进入阻尼；
- 物理进程与 viewer 解耦，关闭或卡住 viewer 不会直接卡死 1 kHz 物理循环。

## 关键实现文件

| 文件 | 作用 |
|---|---|
| `scripts/run_race.sh` | 统一启动和回收手动模式相关进程 |
| `scripts/windows_viewer.py` | Windows 原生 viewer、键盘 Hook、按键 UDP 和接触着色 |
| `src/s10_perception/s10_perception/viewer_node.py` | 汇总 ROS 状态并以 60 Hz 发送 23 维 `qpos` |
| `src/s10_perception/s10_perception/sim_node.py` | WSL MuJoCo 物理、按键 UDP 接收和 `/keyboard/key` 发布 |
| `scripts/patch_upstream.py` | 将键盘、状态机、高速模式和 policy 切换补丁应用到上游 SDK |
| `upstream/.../keyboard_interface.hpp` | WASD 速度生成、按键超时和模式命令处理 |
| `upstream/.../rl_control_state.hpp` | RL policy 执行与 `H` 高速覆盖 |
| `upstream/.../obstacle_state.hpp` | `V/M` 高台阶硬编码关节控制 |

## 当前边界

- WASD 仍依赖 policy 的能力边界，不能保证在训练分布外稳定跟踪目标速度；
- Windows 与 WSL 间使用本机 UDP，没有确认和重传，适合交互调试而不是安全关键遥控；
- 高速模式是固定低姿态和固定轮速覆盖，应只在平整、开阔地面使用；
- `V/M` 高台阶动作仍需针对具体台阶高度、摩擦和关节行程继续标定；
- `7/8` 调参逻辑在未设置环境变量时从 `2 rad/s` 开始，而高台阶状态本身默认是 `4 rad/s`；需要可复现实验时，应在启动前显式设置 `S10_OBSTACLE_WHEEL_SPEED`；
- 轮子红/绿只显示接触布尔值，不显示法向力、切向力或摩擦裕量。
