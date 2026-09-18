<div align="center">

# goai26-s10-racing

**Perception-driven autonomous racing for a wheel-legged quadruped**

GOAI 2026 · Track 4 *Embodied Future* · Challenge 2 — S10 Perception Racing Contest

[![License](https://img.shields.io/badge/license-BSD--3--Clause-blue.svg)](LICENSE)
[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E.svg?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-E95420.svg?logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/24.04/)
[![MuJoCo](https://img.shields.io/badge/MuJoCo-simulation-000000.svg)](https://mujoco.org/)
[![Python](https://img.shields.io/badge/Python-3.12-3776AB.svg?logo=python&logoColor=white)](https://www.python.org/)

</div>

---

## Overview

- [59 维 phase 策略的 MuJoCo / Windows viewer 启动入口](docs/S10_PHASE_SIM2SIM_ZH.md)：`scripts/start_s10_phase.cmd A` 或 `B`，对应各 500 次 PPO 的 flat 模型。

### 当前真机：48 号（2026-09-11）

- [48 号环境、账号与传感器](docs/S10_48_SETUP_ZH.md)：使用本队 `golai`，Windows 登录 `ssh s10-48-golai`。
- [48 号官方 SLAM：当前状态、使用方法与算法](docs/S10_48_SLAM_ZH.md)：已核实 106 的官方 SLAM；手机连接 48 号 Wi-Fi，打开 `http://10.21.41.1:8080/`，103 转发至 AGX 网页，遥控器控制行走。
- 9月11日下午，重启后IMU时间和ODOM已恢复，106已部署PTP启动检查、手机测量时间检查；下一次正常开机及短距离闭环仍待验收，旧跳时地图不用于导航。
- 当前不再使用 50 号。`xwy` 是 50 号上其他队伍的账号，48 号没有该账号；下面的 50 号录制、部署及算法调查均是历史资料。51 号交接文档是未采用的计划。

### 实机采集与研究交接（2026-09-09）

- [历史数据、运动验证、地图与 RL 参考结论](docs/S10_DATA_RESEARCH_ZH.md)：队友先读这份，包含已完成事项、限制、地图预览及本地研究包复现入口。
- [楼梯匹配使用方法与经验](docs/S10_STAIRS_MATCHING_GUIDE_ZH.md)：第一梯段及剩余楼梯的参考包、回放/USD、逐阶段掩码、软参考训练用法和复现命令；尚未验证动力学跟踪或 RL 效果。
- [基础步态平面匹配与软参考用法](docs/S10_BASIC_FLAT_MATCHING_ZH.md)：5 段、34 秒 Isaac Sim 自由动力学实测、对照视频、参考 NPZ、软奖励示例与复现步骤；最终 3 段未摔倒，转向/侧向 2 段侧翻，尚未训练 RL。
- [50 号 106 定位板与 SLAM 算法调查（历史）](docs/S10_SLAM_106_RESEARCH_ZH.md)：厂商版本、LIO/回环/图优化结构及开源算法来源线索，不能代替 48 号版本验收。
- [50 号机器人采集站部署与清理](tools/s10_gait_capture/DEPLOYMENT_050.md)与[采集数据格式](tools/s10_gait_capture/LOCAL_COPY.md)：102 独立采集站和 103 临时中转均已于 2026-09-10 清理，本地录制保留。
- [采集工具交接与本地演示](tools/s10_gait_capture/README.md)：独立手机热点、自启、点云预览和已知故障。9 月 9 日新增 17 段数据单独存于采集工作站 `D:/S10Data/050/2026-09-09/`，不随 Git 分发；本批未录深度相机，也没有官方关节动作目标。
- [下一步验证事项](docs/TODO.md)。原始 bag、模型副本、完整地图及分析输出在本地 `artifacts/`，不随 Git 提交；仅 clone 仓库不会获得这些材料。本轮完成离线审查与短时仿真，尚未完成专家策略复刻或 RL 训练。

The DEEP Robotics **Lynx S10** is a 16-DOF wheel-legged quadruped. The contest asks it to
race a **33-waypoint, 224 m course with 6.7 m of cumulative climb** through an industrial
park, as fast as possible, using perception.

The policy shipped with the contest SDK is **blind** — its 57-dimensional observation is
entirely proprioceptive, with no terrain input of any kind. It walks wherever it is told
and stalls on the elevated sections. Closing that gap is the whole problem, and this
repository is our answer to it:

| Layer | What it does |
|---|---|
| **Perception** | Ray-cast lidar and a body-frame height map, synthesised in MuJoCo — neither exists in the stock scene |
| **Navigation** | Pure-pursuit waypoint following with heading-scheduled speed and a stall watchdog |
| **Locomotion** | An RL policy whose observation is the baseline extended with the height map |

Everything runs on **Ubuntu 24.04 + ROS 2 Jazzy**, the mandated contest environment.

---

## Architecture

```
                    ┌────────────────────────────┐
                    │      s10_auto_nav          │
                    │   waypoint_follower        │
                    │  pure pursuit @ 50 Hz      │
                    └──────────┬─────────────────┘
                               │ /cmd_vel
                               ▼
   ┌───────────────────────────────────────────────┐
   │  rl_deploy  (contest SDK + our ROS bridge)    │
   │  ONNX locomotion policy @ 50 Hz               │
   └──────────┬─────────────────────────▲──────────┘
              │ /JOINTS_CMD             │ /IMU_DATA, /JOINTS_DATA
              ▼                         │
   ┌───────────────────────────────────────────────┐
   │      s10_perception · sim_node                │
   │  MuJoCo physics @ 1 kHz  +  synthetic sensors │
   └───────────────────────────────────────────────┘
              │
              └──► /ground_truth/odom · /scan · /perception/heightmap
```

The upstream simulator is **subclassed, not forked**: physics, joint handling, waypoint
timing and scoring stay upstream's, and we add only the sensors the contest expects us to
build ourselves. Our single change to the SDK is one header that lets the policy take its
velocity command from a ROS topic instead of a keyboard.

---

## Repository layout

```
├── src/
│   ├── s10_perception/      MuJoCo lidar, height map, ground-truth odometry
│   ├── s10_auto_nav/        pure-pursuit waypoint follower
│   └── s10_bringup/         launch files, course and tuning configuration
├── training/
│   └── s10_rl/              observation spec (single source of truth), ONNX export
├── integration/
│   └── ros_cmd_interface.hpp   the one file we add to the contest SDK
├── scripts/                 setup, build, run, course extraction
└── upstream/                contest SDK — cloned, never committed
```

---

## Quick start

**Prerequisites** — Ubuntu 24.04, ROS 2 Jazzy, and access to the contest material
repository (issued to registered teams).

```bash
# 1. Clone
git clone <this-repo> && cd goai26-s10-racing

# 2. Python dependencies (keep ROS packages visible inside the venv)
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install "numpy<2.0" mujoco scipy pyyaml pytest ruff

# 3. Fetch and patch the contest SDK
scripts/setup_upstream.sh

# 4. Build
scripts/build.sh

# 5. Race
scripts/run_race.sh

# Or drive manually (keep this terminal focused)
scripts/run_race.sh --manual
# Z: stand, C: RL control, W/A/S/D: move, Q/E: turn
# Manual mode keeps odometry but skips unused lidar and height-map ray casts.
```

Manual mode runs physics headless and opens a native Windows MuJoCo viewer. WSL streams
only the latest robot pose over local UDP, so Windows window latency cannot stall physics
or RL control; `--manual --headless` skips the display stream too. Set
`S10_VIEWER_BACKEND=wsl` only when the older WSLg viewer is needed.

The Windows viewer uses a project-local environment. It is already installed on the
current workstation; to recreate it on D: from Windows PowerShell:

```powershell
$env:UV_PYTHON_INSTALL_DIR = 'D:\DevTools\uv-python'
$env:UV_CACHE_DIR = 'D:\DevTools\uv-cache'
uv python install 3.12
uv venv --python 3.12 .venv-win
uv pip install --python .venv-win\Scripts\python.exe "numpy<2" mujoco
```

The lap time is printed to the terminal by the simulator when the final waypoint is
reached.

<details>
<summary><b>Running the three processes by hand</b></summary>

`run_race.sh` is a convenience wrapper. The underlying sequence is:

```bash
export ROS_DOMAIN_ID=1
source install/setup.bash

# Terminal 1 — locomotion policy
ros2 run s10_sdk_deploy rl_deploy

# Terminal 2 — simulator with perception
ros2 run s10_perception sim_node

# Terminal 3 — autonomous navigation
ros2 launch s10_bringup race.launch.py launch_sim:=false
```

</details>

<details>
<summary><b>Common invocations</b></summary>

```bash
scripts/run_race.sh --headless                      # batch evaluation, no viewer
scripts/run_race.sh --manual                        # WASD/QE; P policy; V/M obstacle; H high speed
S10_START_WAYPOINT=16 S10_START_YAW_DEG=0 scripts/run_race.sh --manual  # face the 37 cm ledge normal
S10_VIEWER_BACKEND=wsl scripts/run_race.sh --manual # fallback WSLg viewer
scripts/build.sh --packages-select s10_auto_nav     # rebuild one package
BUILD_PLATFORM=arm scripts/build.sh                 # cross-build for the robot
scripts/extract_waypoints.py --check                # confirm the course is current
scripts/patch_upstream.py --revert                  # restore a pristine SDK checkout

S10_MUJOCO_XML=/abs/path/model.xml scripts/run_race.sh    # custom scene

# Path waypoints are 1-based here: 16 means course.yaml entry index 15.
S10_POLICY_PATH=/abs/path/policy.onnx \
S10_RECORD_QPOS=/abs/path/wp16.npy \
scripts/replay_waypoint.sh 16 0.60 30

python scripts/render_qpos_gif.py /abs/path/wp16.npy /abs/path/wp16.gif \
  --xml-path upstream/goai_embodied_future_material/src/S10_sdk_deploy/S10_description/s10_mjcf/mjcf/S10_track.xml \
  --width 640 --height 360 --fps 10
```

The native MuJoCo window starts with both parameter sidebars hidden. When it has focus,
the robot keys (`WASD/QE`, `Z/C/V/M/H/P/L/0` and the tuning keys) are captured before MuJoCo's
own visualization shortcuts, so the command terminal does not need focus. `Ctrl+Q` or the
window close button exits the viewer. Contact-point visualization starts disabled and uses
small metric-scale markers if enabled for diagnostics.

The 37 cm test ledge has its approach normal at world yaw `0 deg`; the waypoint 16→17
tangent (`-18.4 deg`) is not square to the face. `V` enters the approach pose and drives
the wheels toward the face; `M` extends the front knees, sets the rear knees to `0.35 rad`,
and ramps all four wheels to `8 rad/s`. The old `B`/`N` phase controls are disabled.
Keys `1/2`, `3/4`, `5/6`, `7/8`, and `[/]` decrease/increase phase time, fold hip,
fold knee, wheel speed, and `S10_OBSTACLE_FRONT_PRESS`.
Set `S10_OBSTACLE_WHEEL_KD` before launch to calibrate loaded-wheel torque.
Set `S10_OBSTACLE_REAR_KP` to calibrate rear-leg stiffness (default `20`), and
`S10_OBSTACLE_LIFT_HIP`/`S10_OBSTACLE_LIFT_KNEE` to calibrate the lift pose. Each phase
holds until its key; `C` cancels directly to RL. IMU and sustained torque limits can send
the robot to damping, but phase triggering remains manual.
Safety limits are configurable with `S10_OBSTACLE_ROLL_LIMIT` (`0.75`) and
`S10_OBSTACLE_PITCH_LIMIT` (`1.75`, applied to nose-down pitch magnitude).
The full climb remains experimental.

In RL control, `H` toggles flat-ground high-speed mode. It first crouches for one second
to `(front hip/knee=-0.75/+1.50, rear=+0.75/-1.50 rad)`, then ramps all four wheels to
`20 rad/s`; pressing `H` again slows down before standing back up. Calibrate with
`S10_HIGH_SPEED_HIP`, `S10_HIGH_SPEED_KNEE`, `S10_HIGH_SPEED_WHEEL`,
`S10_HIGH_SPEED_WHEEL_KD`, and `S10_HIGH_SPEED_RAMP` before launch.

In RL control, `P` switches instantly between the SDK default policy and
`policies/s10_stairs_stable_up_57d_model499.onnx`. Both sessions are preloaded, and action
history is cleared at the switch. Entering the stairs policy also disables the `H`
high-speed override. Set `S10_SECOND_POLICY_PATH` before launch to use another compatible
57- or 174-observation ONNX model.

`L` switches between the default policy and the preloaded stairs-down policy at
`policies/stable_down_compare/model300.onnx`. Pressing `P` while stairs-down is active
switches directly to stairs-up, and pressing `L` while stairs-up is active switches
directly to stairs-down.

`K` switches between the default policy and the preloaded speed-turn policy at
`policies/speedturn_sweep/model_2000.onnx`. Pressing `K` while either stairs policy is
active switches directly to speed-turn. Override it with `S10_SPEEDTURN_POLICY_PATH`.

</details>

---

## Interfaces

Published by `s10_perception` in addition to the SDK's proprioception:

| Topic | Type | Rate | Contents |
|---|---|---|---|
| `/ground_truth/odom` | `nav_msgs/Odometry` | 50 Hz | base pose and twist |
| `/scan` | `sensor_msgs/LaserScan` | 50 Hz | horizontal lidar ring |
| `/perception/lidar` | `std_msgs/Float32MultiArray` | 50 Hz | full range image, elevation × azimuth |
| `/perception/heightmap` | `std_msgs/Float32MultiArray` | 50 Hz | terrain height relative to the base |

Published and consumed by the autonomy layer:

| Topic | Type | Direction | Contents |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | follower → policy | forward, lateral, yaw rate |
| `/robot_mode` | `std_msgs/UInt8` | → policy | manual state override |
| `/robot_state` | `std_msgs/UInt8` | policy → | current state machine state |
| `/nav/progress` | `std_msgs/Float32` | follower → | fraction of waypoints reached |
| `/nav/finished` | `std_msgs/Bool` | follower → | course complete |

Reading ground truth is explicitly permitted: the contest does not require SLAM in
simulation. On hardware the same topic is fed by odometry, so nothing downstream changes.

---

## Key parameters

Tuning lives in [`src/s10_bringup/config/nav.yaml`](src/s10_bringup/config/nav.yaml).

| Parameter | Default | Why it matters |
|---|---|---|
| `max_forward` | `1.6` m/s | The course is ~224 m, so the speed ceiling dominates lap time. Must stay inside the velocity range the policy was trained on — raise it together with the training command range, never alone. |
| `lookahead` | `1.4` m | Larger is smoother and faster but cuts corners harder, which can miss a waypoint's 0.2 m scoring radius. |
| `advance_radius` | `0.35` m | Deliberately wider than the scorer's 0.2 m so the follower commits to the next leg instead of braking at every gate. |
| `stall_timeout` | `2.5` s | The dominant failure on elevated sections is wedging against a ledge while the gait keeps cycling — no fall detector catches that. |

Sensor geometry is set in `LidarConfig` and `HeightmapConfig`
([`lidar.py`](src/s10_perception/s10_perception/lidar.py),
[`heightmap.py`](src/s10_perception/s10_perception/heightmap.py)).

---

## The observation contract

Three components must agree on the exact layout of the policy input: the training
environment, the ONNX export, and the C++ runner. A disagreement raises no error — it
produces a policy that walks subtly wrong. So the layout is defined once, in
[`training/s10_rl/observation.py`](training/s10_rl/observation.py), and everything derives
from it.

```console
$ python -m s10_rl.observation
baseline: 57 dimensions

  [  0:  3]  base_angular_velocity  body frame, scaled by 0.25
  [  3:  6]  projected_gravity      gravity direction in the body frame
  [  6:  9]  velocity_command       forward, lateral, yaw rate
  [  9: 25]  joint_position         policy order, minus default, wheels zeroed
  [ 25: 41]  joint_velocity         policy order, scaled by 0.05
  [ 41: 57]  last_action            previous network output

perceptive: 174 dimensions
  ... plus
  [ 57:174]  heightmap              13x9 grid, terrain height relative to the base
```

Perception is **appended** rather than inserted, so the proprioceptive prefix keeps the
baseline's offsets and a perceptive policy can warm-start from baseline weights instead of
training from scratch.

Exporting a trained checkpoint:

```bash
python -m s10_rl.export_onnx checkpoint.pt -o policy.onnx --verify
```

The export binds the tensor names `obs` and `actions` that the runner looks up, and
`--verify` re-loads the graph the way the deployed runner does. It then prints the
`observation_dim` constant that must be updated in the SDK's policy runner.

---

## Scoring

```
score = elapsed_simulation_seconds ÷ mode_coefficient       (lower is better)
```

| Mode | Preliminary | Final |
|---|---|---|
| Remote operation | ÷ 1.0 | ÷ 1.0 |
| Autonomous following | — | ÷ 1.3 |
| Autonomous navigation | ÷ 1.2 | ÷ 1.4 |

All waypoints must be reached, in order, within a 0.2 m horizontal radius. A continuous
torque excursion beyond the model maximum lasting **more than 0.5 s is disqualifying**.

This repository runs in autonomous navigation mode.

---

## Development

### S10 录制复核与参考运动重建

团队交接见 [录制复核与重建经验](docs/S10_RECORDING_REVIEW_AND_RECONSTRUCTION_ZH.md)：包含两批录制的区别、关节/楼梯试验结果、3D 点云与动作复核区用法、人工标注协作、队友本地启动步骤和 Git/大数据交付边界。当前复核区支持保存人工判断；新批次地图、实机轨迹重建和 RL 训练尚未完成。

```bash
scripts/build.sh                       # build everything
colcon test --packages-select s10_auto_nav
ruff check .                           # lint
scripts/extract_waypoints.py --check   # course still matches the scene?
scripts/patch_upstream.py --check      # SDK still patched?
```

The course in `config/course.yaml` is **generated**, not hand-written — regenerate it with
`scripts/extract_waypoints.py` so what we follow can never drift from what we are scored
on.

See [CONTRIBUTING.md](CONTRIBUTING.md) for workflow and conventions.

---

## Acknowledgements

Built on the contest material published by [DEEP Robotics](https://github.com/DeepRoboticsLab)
for GOAI 2026 Track 4. The simulator, robot model, track scene and deployment SDK are
theirs; the perception, navigation and training layers here are ours.

Licensed under [BSD-3-Clause](LICENSE), matching upstream.
