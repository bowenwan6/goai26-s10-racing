# S10 轮足机器人 WASD 手动控制：Demo 说明

[项目介绍](WASD_PROJECT_INTRODUCTION.md) · [技术方案](WASD_TECHNICAL_SOLUTION.md)

## 环境准备

推荐环境：

- Windows 11 + WSL 2；
- WSL 内 Ubuntu 24.04、ROS 2 Jazzy；
- 仓库已执行 `scripts/setup_upstream.sh` 和 `scripts/build.sh`；
- Windows 本地 `.venv-win` 已安装 NumPy 和 MuJoCo。

首次部署和依赖安装见仓库根目录 `README.md`。以下命令均在 WSL 的仓库根目录执行。

## 启动

```bash
cd /mnt/d/Desktop/Code/goai26-s10-racing
scripts/run_race.sh --manual
```

启动脚本会依次运行：

1. WSL 中的无窗口 `sim_node`；
2. Windows 原生 MuJoCo viewer；
3. WSL 到 Windows 的 60 Hz 姿态流；
4. 前台 `rl_deploy` 控制进程。

看到以下日志后，说明 MuJoCo 窗口键盘捕获已经启用：

```text
Focused-window robot keyboard capture active
Viewer stream connected
```

如只需无窗口测试：

```bash
scripts/run_race.sh --manual --headless
```

## 推荐演示流程

1. 点击 MuJoCo 画面，使其成为前台窗口。
2. 按 `Z`，等待机器人完成站起。
3. 按 `C`，等待终端出现进入 `rl_control` 的状态日志。
4. 按住 `W` 前进，松开后机器人应在最多约 0.5 秒内收到零速指令。
5. 分别演示 `S` 后退、`A/D` 横移、`Q/E` 转向。
6. 同时按 `W+Q` 或 `W+E`，演示速度指令叠加和转弯。
7. 在开阔平地按 `H`，展示先降低重心、后提高轮速的高速模式；再次按 `H` 退出。
8. 按 `P` 或 `L` 展示 policy 切换，终端会打印当前激活的模型。
9. 如需演示高台阶，在机器人正对立面且低速稳定时按 `V`，接触建立后再按 `M`。
10. 需要重新开始时按 `0`，等待重置和恢复流程完成。

建议把基础 WASD、高速模式和高台阶模式分成三个独立 Demo，发生失稳时直接复位，不连续叠加多个模式。

## 如何结束程序

完整结束仿真时，回到启动 `scripts/run_race.sh --manual` 的 WSL 终端并按：

```text
Ctrl+C
```

启动脚本会统一结束 `rl_deploy`、`sim_node` 和 `viewer_node`。Windows MuJoCo 窗口可按 `Ctrl+Q` 或点击关闭按钮退出，但这只关闭 viewer，不等于结束 WSL 中的控制与物理进程。

如果启动终端已经丢失，最后手段是在 Windows PowerShell 执行：

```powershell
wsl --shutdown
```

该命令会结束全部 WSL 发行版和其中的其他任务，使用前应先保存其他 WSL 工作。

## 预期现象

| 操作 | 正常现象 |
|---|---|
| 聚焦 MuJoCo 后按 `C` | 终端打印 RL Control，画面不出现 MuJoCo 的黄色调试圆盘 |
| 按住 `W` | 期望前进速度为 `1.0 m/s`，policy 自动生成腿部和轮子命令 |
| 松开移动键 | 最迟约 500 ms 后对应期望速度归零 |
| 轮子接触地面/墙面 | 轮子变绿；无接触时变红 |
| 按 `H` | 先趴低，后加速；再次按下则先减速再恢复姿态 |
| 关闭 MuJoCo 窗口 | Windows viewer 退出，WSL 仿真仍继续运行 |

## 常见问题

### 按 WASD 没有反应

先确认机器人已经完成 `Z -> C`，终端状态已进入 `rl_control`。站起过程中或处于阻尼、趴下、高台阶状态时，普通移动速度不会按 RL 方式执行。

再确认 MuJoCo 窗口确实是当前前台窗口，并且 viewer 启动日志包含：

```text
Focused-window robot keyboard capture active
```

### 按 `C` 后出现 MuJoCo 黄色圆盘

当前版本会在 Windows viewer 前台时拦截 `C`。如果仍出现 MuJoCo 原生调试图形，通常说明运行的是旧 viewer、键盘 Hook 没有安装成功，或者聚焦的不是本项目启动的 viewer。应结束旧进程后重新运行 `scripts/run_race.sh --manual`。

### 期望速度是 1.0 m/s，为什么实际没有这么快

`1.0 m/s` 是输入给 policy 的目标，不是强制轮速。Policy 会结合姿态和关节状态决定动作；轮地打滑、台阶碰撞、扭矩限制和训练分布都会造成跟踪误差。需要固定轮速进行平地高速测试时使用 `H` 模式，而不是提高 WASD 数值后假设 policy 一定能跟上。

### 轮子是绿色但转动没有位移

绿色只证明 contact 列表中存在该轮接触。继续检查接触法向力、摩擦利用率、轮速方向、机身/腿部是否卡在台阶碰撞体，以及髋膝关节是否接近限位或力矩饱和。

### MuJoCo 窗口自动退出

Windows viewer 在启动后 30 秒没有收到任何姿态帧，或已连接后连续 2 秒断流，会主动退出。检查 WSL 中的 `sim_node`、`viewer_node` 和 ROS 2 domain 是否仍在运行。
