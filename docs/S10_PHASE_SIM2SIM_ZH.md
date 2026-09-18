# 59 维 phase 策略：MuJoCo / Windows viewer

2026-09-17：接入既有 `run_race.sh --manual` 和 Windows viewer。本机已有两份完成 500 次 PPO 的模型：

- `policies/phase_20260917/A500.onnx`：M20 原生奖励 + phase 输入。
- `policies/phase_20260917/B500.onnx`：同接口，增加 phase 轨迹、腿动作变化与腿软限位奖励。

模型来自相邻 `rl_training/artifacts/s10_flat_phase_20260917/train_A` / `train_B` 的 `model_499.pt`。模型文件保存在本地，不随 Git 分发。

## 启动

Windows 双击 `scripts/start_s10_phase.cmd` 默认运行 A。PowerShell 中选择组：

```powershell
.\scripts\start_s10_phase.cmd A
.\scripts\start_s10_phase.cmd B
```

每次只运行一组，关闭前一次后再启动。窗口打开后，聚焦 viewer，按 **Z 站立 → C 策略控制**；W/S 前后，A/D 侧移，Q/E 转向，持续运动需持续按住。0 重置并自动恢复站立及策略控制。Ctrl+C 结束启动终端中的仿真。

WSL 等价入口：

```bash
bash scripts/run_phase.sh A
bash scripts/run_phase.sh B --headless
```

仍使用现有 MuJoCo 场景（默认赛道起点平地）和 Windows viewer；可通过已有 `S10_MUJOCO_XML` 选场景。策略 50 Hz；MuJoCo 保留当前 1 ms 物理步长，训练使用 2.5 ms × 8，两者不能混写。

启动器使用本机 WSL、`ROS_LOCALHOST_ONLY=1`，默认 `ROS_DOMAIN_ID=59`。P/L/K 仍会切到现有上楼/下楼/转向备用策略；再次按同键回到所选 A 或 B。观察本次 flat 时不需要按这些切换键。

## 接口

### A/B/C 续训至累计900次

新增模型为 `policies/phase_abc_20260917/A900.onnx`、`B900.onnx`、`C900.onnx`。A/B 分别从各自500次继续400次；C从B500开始，仅将 `feet_air_time_ang_z_M20` 的gait系数旁路，再训练400次。三者都是相同59维接口。

在本项目的WSL目录选择900次模型，例如C：

```bash
ROS_DOMAIN_ID=59 ROS_LOCALHOST_ONLY=1 S10_POLICY_PATH="$PWD/policies/phase_abc_20260917/C900.onnx" bash scripts/run_race.sh --manual
```

选择A/B时替换模型文件名即可。上面的 `start_s10_phase.cmd A|B` 对应上一轮500次模型；900次模型使用本段路径。

| 0 起始索引 | 输入 |
|---|---|
| 0–56 | 原有 57 维本体观测，顺序与缩放保持一致 |
| 57 | `sin(2π × policy_step × 0.02 / 0.6)` |
| 58 | `cos(2π × policy_step × 0.02 / 0.6)` |

输入 `obs: float32[1,59]`，输出 `actions: float32[1,16]`。相位在每次进入 RL、重置后重新进入 RL、切换模型时从 `[0,1]` 开始；每次策略推理推进一帧，零速度命令也推进，不由键盘或 Windows 渲染时钟决定。周期读取 ONNX 元数据 `phase_cycle_time`，缺省为当前约定 0.6 秒；无效周期拒绝加载。phase 是内部时钟，不是新增传感器。

共享 C++ runner 接受 57 / 59 / 174 维，各模型槽按各自输入宽度切换，并清空上一帧动作；174 维仍走高度图分支。改动保存在 `scripts/patch_upstream.py`，更新 upstream 后执行补丁并重新编译：

```bash
python3 scripts/patch_upstream.py
bash scripts/build.sh --packages-select s10_sdk_deploy --executor sequential
```

## 验证范围

- 本机 ROS C++ 编译通过，观测布局测试 12 项通过，补丁重复应用与 `--check` 通过。
- 两模型各用 380 条真实 Isaac 观测核对 Torch 与 ONNX，最大动作绝对误差分别 `2.5034e-6` / `2.5332e-6`。
- 短时 headless MuJoCo 实际推理检查覆盖 A59 → 原57 → A59 → B59 → A59、完整重置；逐帧核对相位、动作历史、关节顺序/缩放、ONNX 输出与解码。原始 trace、日志及数值结果位于相邻 `rl_training/artifacts/s10_flat_phase_20260917/sim2sim/`。
- 本次未做 GUI 操作验收或完整 MuJoCo 速度达标评估；原 Isaac 评估中两组持续转向均未通过。接口可用不代表已达到 80%–120% 速度目标。
- 限时停止时原 ROS 程序抛出 context shutdown 异常；测试进程已结束，推理记录完整保留。

复查已有 trace（在 `rl_training` 根目录；本次检查临时将槽 3 指向 B）：

```powershell
python scripts/tools/check_s10_phase_trace.py artifacts/s10_flat_phase_20260917/sim2sim/mujoco_trace.jsonl --policies ../goai26-s10-racing/policies/phase_20260917/A500.onnx ../goai26-s10-racing/policies/s10_stairs_stable_up_57d_model499.onnx ../goai26-s10-racing/policies/stable_down_compare/model300.onnx ../goai26-s10-racing/policies/phase_20260917/B500.onnx
```
