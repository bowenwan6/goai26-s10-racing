# S10 HIM ONNX 接入

S10 是四足轮腿机器人。共同接口见 [S10 HIM ONNX 契约 v1](S10_HIM_ONNX_CONTRACT_V1.md)。

本次从 `061c` 单独提取 HIM 推理、站姿/接管对齐、实机遥控与验证代码。
`main` 的导航、Gate16 控制、`patch_upstream.py` 和 `run_race.sh` 保持原实现。
HIM 使用明确选择的入口；实机默认模型为 checkpoint **1500**。

## 已验证的接口

- float32 输入 `obs [1,342]`，输出 `actions [1,16]`，旁边必须有同 stem JSON。
- 每帧 57 维、六帧历史，最新帧在前；策略周期 20 ms（SDK 5 ms × decimation 4）。
- 进入策略、切换槽位、收到 `/sim/reset_done` 后清零历史和上一原始动作。
- 按 sidecar 校验张量、关节顺序、缩放、PD、裁剪及历史约定；57/174 维旧接口也可预加载。
- HIM 起身终点对齐 `default_dof_pos`，进入 RL 后用 0.5 秒从实测关节位置平滑接管。
  插值在动作解码之后，网络历史仍记录原始裁剪动作。
- `s10_policy_runner.hpp`、`rl_control_state.hpp`、`standup_state.hpp` 为仓库维护的实机/HIM 源文件。
  不包含 Jackdev 的高速姿态或 obstacle 状态扩展。

## 实机遥控器

已部署地址为 `s10-48-golai:/home/golai/s10-sdk-isolated-vbyhquwi`。
使用 Windows Python（含 Tk）与已配置的 OpenSSH 别名：

```powershell
.\scripts\start_s10_him1500_handset.cmd
# 仅打开窗口、不连接：
python scripts/s10_him_gui.py
```

连接后等待遥控器操作，不自动起身。保持 SDK 模式，摇杆回中后按 C（G20 的 L1）起身；
起身完成后自动进入模型，再按 C 趴下，D（G20 的 R2）阻尼退出。
窗口“趴下并断开”结束会话。默认摇杆上限 20%，HIM 指令单位为 m/s、m/s、rad/s。

`S10_SSH_HOST`、`S10_SDK_TRIAL_ROOT`、`S10_MODEL_NAME` 可覆盖连接配置。
`s10_him_gui.py` → `run_s10_him_trial.py` → `s10_him_trial.py` 是本次实测启动链；
已有 `s10_sdk_trial.py` 和 `run_s10_sdk_trial.py` 保留原用途。

监控保留 SDK 模式、心跳、传感器新鲜度、急停、温度、姿态及关节反馈检查。
诊断阈值为腿/轮速度 `25.76/30 rad/s`、腿/轮力矩 `45/12 N·m`；
C++ 发送前另检查预计 PD 力矩和腿目标单步跳变 `0.1 rad`。这些不是硬件额定值。
触发保护后回零、请求阻尼并结束部署进程。

记录保存至 `results/s10-sdk/<host>/<run>/`。策略 trace 记录前 300 帧，
监控采样保留最近约一分钟，故障快照独立保存。长会话末尾故障不一定有网络输入 trace。
`deployment entered damping unexpectedly` 的 C++ 原因见部署日志中的 `[POLICY]`。

### 重建独立实机 SDK

将本仓库的 `scripts/`、`integration/` 及 ONNX/JSON 放到 AGX，使用已校验电机标定的 DDS SDK：

```bash
python3 scripts/prepare_s10_sdk_copy.py \
  --source /home/golai/s10-sdk-isolated-vbyhquwi/src/S10_sdk_deploy \
  --imu-angle-unit rad --him-model /absolute/path/to/policy.onnx
# 使用上一步打印的新目录：
cd /printed/s10-sdk-isolated-XXXX
source /opt/ros/jazzy/setup.bash
source /home/golai/s10_control_ws/install/setup.bash
colcon build --packages-select s10_sdk_deploy --cmake-args -DBUILD_PLATFORM=arm
```

准备脚本只复制与安装文件，不启动电机。它保留 DDS 入口、电机方向与零位，
修正 ROS 参数接收、弧度 IMU 和 SensorDataQoS，并安装 HIM 头文件、站姿字段和模型。
`third_party` 链接回源 SDK，因此源目录须保留。省略 `--him-model` 时保留原复制行为。

## HIM 仿真入口

`patch_him_upstream.py` 安装到实际构建目录
`upstream/goai_embodied_future_material/src/S10_sdk_deploy`。
该 SDK 变体使用现有 ROS 命令桥；默认比赛的 Gate16 SDK 仍由 `patch_upstream.py` 安装。
选择任一变体后必须重新构建。实机请使用上一节的 DDS 准备流程。
构建指纹记录 SDK 控制器类型；HIM 二进制不能通过默认比赛入口启动，反之亦然。

```bash
python3 scripts/patch_him_upstream.py
bash scripts/build.sh --packages-up-to s10_sdk_deploy s10_perception s10_auto_nav s10_bringup
export S10_POLICY_PATH=/absolute/path/to/policy.onnx
S10_USE_VIEWER=0 bash scripts/run_him.sh
```

HIM 启动默认只加载显式指定的主模型；可选的 `S10_SECOND_POLICY_PATH`、
`S10_DOWN_POLICY_PATH`、`S10_SPEEDTURN_POLICY_PATH` 默认为空。
需要混装时显式设置对应路径，允许 57/174/342 维；`S10_POLICY_SLOT` 选择预加载槽位，
切换时重建观测缓冲区与解码参数并清零历史。实机本轮只验证主槽的 HIM 1500。
HIM 仿真入口关闭 Gate16 路由，直接使用现有 waypoint follower 的物理速度指令。

MuJoCo 启动时核对每个 HIM sidecar 的力矩限幅、执行器顺序和齿比。
`/sim/reset` 复位机器人后先发布反馈，再发布 `/sim/reset_done` 清零策略历史。
原有模拟时钟、导航、感知和回放功能保持原实现。

## 检查

以下检查不会连接机器人：

```powershell
python scripts/prepare_s10_sdk_copy.py --check
python scripts/s10_him_trial.py --check
python scripts/run_s10_him_trial.py --check
python scripts/s10_him_gui.py --ui-check
# 使用 numpy / onnxruntime 复核已有的一秒零指令接管记录：
python scripts/check_s10_him_takeover.py <recorded-run-directory> --model <policy.onnx>
```

Linux / WSL 的 C++ 和仿真检查：

```bash
source /opt/ros/jazzy/setup.bash
python scripts/check_him_contract.py --model "$S10_POLICY_PATH" \
  --pristine-upstream /path/to/goai_embodied_future_material
S10_SECOND_POLICY_PATH= S10_DOWN_POLICY_PATH= S10_SPEEDTURN_POLICY_PATH= \
  results/him-contract/build/him_handover "$S10_POLICY_PATH"
source install/setup.bash
python -m pytest src/s10_perception/test/test_him_sim.py \
  src/s10_auto_nav/test/test_runtime_contract.py -q
python scripts/check_him_sim.py --model "$S10_POLICY_PATH"
```

接口检查覆盖错误模型拒绝、六帧历史、非对称关节排列、缩放/裁剪、57↔342 切换、
174 高度图、进入与复位清零、干净 SDK 重建及补丁幂等性。
`him_handover` 使用内存中的机器人接口，没有电机发布者。

本次在最新 `main` 上重新通过 C++ 接口与接管检查、7 项 Python 回归、5 个 ROS 包构建，
以及使用 1500 模型的闭环仿真接入检查（181 帧策略输出、一次复位并清空历史）。
构建指纹检查也确认默认比赛入口会拒绝 HIM 构建。此检查不评价爬阶性能。

## 已有实机结果（2026-09-16）

记录 `1789533335127877200` 完整完成起身、1 秒零指令 RL、趴下，退出码 0。
50 帧、50.009 Hz，ARM 与本地 ONNX 最大绝对误差 `4.768×10⁻⁷`；
历史、上一原始动作、关节顺序、解码和 PD 检查通过。腿/轮峰值力矩 `3.746/0.520 N·m`，
现场确认原先接管弹跳消失。

随后遥控前进接触第一阶时，右前轮 `31.109 rad/s`、左前轮 `-30.656 rad/s`
分别触发 30 rad/s 诊断保护，阻尼退出。接入已有上述验证，爬阶能力未通过；
本次提取保留保护参数，后续策略表现由 RL 训练迭代。
原始记录在训练仓库 `artifacts/s10-him1500-real-20260916/`，合入复核产物在 `results/him-real/`。
