<div align="center">

# goai26-s10-racing

**Perception-driven autonomous navigation for the DEEP Robotics Lynx S10**

GOAI 2026 · Track 4 *Embodied Future* · Challenge 2 — S10 Perception Racing Contest

[![License](https://img.shields.io/badge/license-BSD--3--Clause-blue.svg)](LICENSE)
[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E.svg?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-E95420.svg?logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/24.04/)
[![MuJoCo](https://img.shields.io/badge/MuJoCo-simulation-000000.svg)](https://mujoco.org/)
[![Python](https://img.shields.io/badge/Python-3.12-3776AB.svg?logo=python&logoColor=white)](https://www.python.org/)
[![Version](https://img.shields.io/badge/version-1.0-6f42c1.svg)](#version-status)

</div>

---

## 从这里开始（2026-09-19）

| 想做什么 | 看这里 |
|---|---|
| 一页看懂我们有哪些 policy 和 app | [docs/POLICIES_AND_APPS_ZH.md](docs/POLICIES_AND_APPS_ZH.md) |
| 仓库目录、分支规则、大文件 | [docs/REPO_GUIDE_ZH.md](docs/REPO_GUIDE_ZH.md) |
| 新狗 + 新 SLAM（x_nav）接入计划 | [docs/NEW_SLAM_XNAV_INTEGRATION_ZH.md](docs/NEW_SLAM_XNAV_INTEGRATION_ZH.md) |
| ROS 1 网关、106 点云 tap、ROS 1 运控 SDK | [ros1_gateway/README_ZH.md](ros1_gateway/README_ZH.md) |
| 手机采集助手 `/teach`（建图、标 WP、示教路径） | [tools/s10_mapping_web/TEACH_GUIDE_ZH.md](tools/s10_mapping_web/TEACH_GUIDE_ZH.md) |
| 9 月导航（`rl_nav`：J3100 步行 + 1150 台阶） | [docs/RL_ROUTE_ROBUST_PLAN_ZH.md](docs/RL_ROUTE_ROBUST_PLAN_ZH.md) · [docs/ROUTE_V2_PLANNER_ZH.md](docs/ROUTE_V2_PLANNER_ZH.md) |
| 原生步态导航 | [native_transfer/README_ZH.md](native_transfer/README_ZH.md) · [docs/NATIVE_START_B_ACCEPTANCE.md](docs/NATIVE_START_B_ACCEPTANCE.md) |
| HIM 模型实机接入 | [docs/S10_HIM_DEPLOYMENT.md](docs/S10_HIM_DEPLOYMENT.md) |
| 已上传文件的索引 | [docs/GITHUB_FILE_INDEX_ZH.md](docs/GITHUB_FILE_INDEX_ZH.md) |
| RL 训练、MuJoCo 全程仿真 | 私有仓库 `bowenwan6/s10-rl-sprint` |

**当前状态：**
- 实机跑过的 policy 只有官方 57D、speedturn2000、HIM 1500。
- 9 月导航方案（J3100 + 1150）在 MuJoCo 里跑完 30/30 WP，用时 713 s；还没上真机。
- 新狗 + x_nav：ROS 1 网关、运控 SDK、`/teach` 已部署到 AGX，等现场实测。

> 以下是 9 月 18 日之前的说明，保留原文。与上文冲突时以上文为准。

## 真机与录制研究

### 2026-09-18 现场文件交付

- [GitHub 文件索引](docs/GITHUB_FILE_INDEX_ZH.md)：app、v3 点云、MuJoCo 场景、真机适配、现场照片和报告的入口及上传范围。
- [地图与 MuJoCo 包](deliverables/S10_v3_Map_MuJoCo_20260916/README.md)包含 v3 全场点云；精细碰撞场景覆盖 Start＋B＋B 后短平台。
- 点云、二进制网格/数据、图片/PDF、视频、分享 ZIP 和原始/预览照片使用 Git LFS；克隆后执行 `git lfs pull` 获取文件内容。

### 当前真机：48 号（2026-09-11）

- [48 号环境、账号与传感器](docs/S10_48_SETUP_ZH.md)：使用本队 `golai`，Windows 登录 `ssh s10-48-golai`。
- [48 号官方 SLAM：当前状态、使用方法与算法](docs/S10_48_SLAM_ZH.md)：手机连接 48 号 Wi-Fi，打开 `http://10.21.41.1:8080/`；103 转发至 AGX 网页，遥控器控制行走。
- [手机建图与实时定位工具](tools/s10_mapping_web/README.md)包含部署和维护说明。9 月 11 日已加入 PTP 启动检查及测量时间检查；后续开机、闭环及定位验收状态以 SLAM 指南为准。
- 当前不再使用 50 号。`xwy` 属于 50 号的历史环境；51 号交接文档是未采用的计划。

### 实机采集与研究交接

- [历史数据、运动验证、地图与 RL 参考结论](docs/S10_DATA_RESEARCH_ZH.md)
- [录制复核与重建交接](docs/S10_RECORDING_REVIEW_AND_RECONSTRUCTION_ZH.md)、[楼梯匹配方法](docs/S10_STAIRS_MATCHING_GUIDE_ZH.md)、[高台匹配方法](docs/S10_LEDGE_MATCHING_GUIDE_ZH.md)
- [基础步态平面匹配与软参考](docs/S10_BASIC_FLAT_MATCHING_ZH.md)：5 段、34 秒 Isaac Sim 试验，3 段未摔倒，转向和侧向 2 段侧翻，尚未训练 RL。
- [采集工具与本地演示](tools/s10_gait_capture/README.md)、[数据格式](tools/s10_gait_capture/LOCAL_COPY.md)、[50 号部署及清理记录](tools/s10_gait_capture/DEPLOYMENT_050.md)
- [50 号 106 SLAM 调查（历史）](docs/S10_SLAM_106_RESEARCH_ZH.md)、[下一步验证事项](docs/TODO.md)

9 月 9 日的新 17 段原始录制位于采集工作站 `D:/S10Data/050/2026-09-09/`，不随 Git 分发。
仓库保留工具、索引、报告和部分派生结果；完整回放仍需另行准备原始数据及缓存。

本分支的比赛、导航和仿真入口已对齐 `main`。旧 WASD、Windows 键盘控制和指定模型回放
属于 [合并前版本 cc93d39](https://github.com/bowenwan6/goai26-s10-racing/tree/cc93d39f8a939c88627f73534ca42c30d7705746)，
当前不再提供 `run_race.sh --manual`、`replay_waypoint.sh` 或 `replay_velocity_profile.sh`。
`docs/WASD_*`、`submission/WASD_*` 及 `policies/README.md` 保留为历史记录；当前比赛与分段测试按下文操作。

## Current status

Judge-facing submission details are collected in
[`docs/SUBMISSION.md`](docs/SUBMISSION.md). Dependency, data and model provenance are disclosed in
[`docs/THIRD_PARTY.md`](docs/THIRD_PARTY.md), and the required post-competition scope is described
in [`docs/OPEN_SOURCE_PLAN.md`](docs/OPEN_SOURCE_PLAN.md). The complete implementation-linked
architecture, policy contracts, safety design and evidence record are in
[`docs/TECHNICAL_DESIGN.md`](docs/TECHNICAL_DESIGN.md).

### Version status

The **Ver1.0 competition release** is commit `1e390f5` (tag `v1.0-sim-release`; the submitted
build is `3660b81`, tag `sim-contest-submission`). `main` has since moved on to the September
real-robot work described at the top of this file. Ver1.0 combines the reviewed Gate 16 v1.5
stable-fallback controller with measured per-leg speed scheduling, while preserving strict
ordered-gate and safety checks. The experimental continuous-stairs policy is shipped only for
contract traceability and remains disabled in the competition configuration.

The generated course contains **33 waypoints**, spans **224.21 m horizontally**, and gains
**6.70 m**. The official evaluator counts gates inside a **0.20 m horizontal radius** and in
strict order. Ver1 navigation uses a tighter **0.18 m internal acceptance radius**, giving
0.02 m of margin before the official threshold.

Ver1.0 completed one uninterrupted full-stack test run from **WP0 to WP32**
(seed 6, replay capture enabled):

| Result | Value |
|---|---:|
| Ordered target gates | 33/33 (WP0–WP32) |
| Official simulator elapsed | 436.058 s (7:16.058) |
| Recorder wall elapsed | 708.40 s |
| Distance travelled | 257.49 m |
| Final WP32 distance | 0.177 m |
| Maximum tilt | 57.9° |
| Recorder stalls | 3 |

This is a Ver1.0 test-harness result: the production stack ran continuously, while the
segment harness supplied only the deterministic WP0 start pose and independent telemetry. All
33 internal 0.18 m entries were observed in order; the official simulator logged every 0.20 m
event and computed its elapsed time from the MuJoCo simulation clock. It is evidence of
complete-course simulation autonomy, not an official submission result or a claim that every
seed succeeds. In particular, the frozen WP16 policy remains probabilistic: separate seed 8
repetitions have failed both at WP16 and later at WP27→WP28.

## What is actually deployed

The official **57-dimensional proprioceptive locomotion policy** remains the normal controller.
Only WP15→WP16 uses the Gate 16 bundle: a 174D observation drives the frozen base and
heightmap-gated residual ONNX pair at 50 Hz, producing a 16D joint command. The current runtime
ports the v1.5 confidence-fallback contract from
`goai-s10-gate16-policy@216b77a`: the confidence-gated fast adapter remains available in code,
but the competition configuration disables it and explicitly binds both router and SDK runner
to the stable frontal fallback. The low-level Gate 16 runner executes the bundled command-profile JSON
only when the confidence adapter is enabled: its front-support phases are inferred from left/right
height-map edge distance, not from physical wheel contact. The accepted competition configuration
forces stable fallback, so those fast-profile phases are not selected. Independently, the router
uses simulated wheel centres, deck height and contacts to prove all four wheels are clear before
handoff. The policy stays in its native frontal frame; adaptive mirroring is not enabled by the
competition configuration.
The official controller owns the moving approach. In the accepted forced-fallback configuration,
ownership changes only after `d=0.62–0.70 m`, measured `v=0.08–0.20 m/s`, `|yaw|≤6°`, lateral
error ≤0.25 m and `|yaw_rate|≤0.10 rad/s` hold for 0.10 s; the target command is 0.18 m/s and
the low-level fallback cap is 0.15 m/s. The narrower `d=0.60–0.65 m`, `v=0.23–0.27 m/s`,
`|yaw|≤5°` fast window remains configured but cannot be selected while the fast adapter is
disabled. Gate 16 shadow inference cannot slow the approach; on the ownership edge its
previous-action field is seeded by inverse-decoding the measured actuator state, avoiding a
discontinuity between the unrelated official and Gate 16 policy histories.
After all four wheels are verified on the upper platform, the residual is disarmed and joint
ownership returns to the official policy; the existing follower resumes at 0.5 m/s.

The delivered `policy/stairs_stable/policy.onnx` uses the SDK's native 57D/16D runner on five
measured stair-ascent legs. Once it is within 0.50 m of the target, all wheel centres and the
base must be above the target platform with stable attitude for 0.25 s; ownership then returns
to the official policy to close the waypoint. Segment indices follow the official zero-based log:
WP6→7 uses stairs_stable, while WP4→5, WP5→6 and upper-platform WP18→19 remain official-owned.
Gate 16 remains isolated from this path.

| Layer | Responsibility |
|---|---|
| Perception | MuJoCo ray-cast lidar, horizontal scan, storey-aware body-frame height map, and ground-truth odometry |
| Navigation | Strict ordered-gate tracking, pure pursuit, terrain classification, body-clear local planning, barrier bypass, corner retreat, step commitment, run-up, and stall recovery |
| Locomotion | Official SDK ONNX policy normally; v1.5 stable-fallback Gate 16 runtime around the frozen base+residual pair only for WP15→16 |
| Integration | 50 Hz strategy router, four-wheel clearance verification, safe handoff, and single-owner arbitration at `/JOINTS_CMD` |

`training/s10_rl/` contains a perceptive-observation/export research scaffold. It is not used
by this result. The disabled fast adapter and its mirror bands are retained for future gated
testing; the shipped competition default makes no arbitrary-angle robustness claim.

## Architecture

```text
 /ground_truth/odom    /scan    /perception/heightmap
          │              │                │
          └──────────────┴────────────────┘
                         ▼
              s10_auto_nav / follower
       pure pursuit + terrain + avoidance + recovery
                         │ /cmd_vel
                         ▼
                strategy router (50 Hz)
               ╱                       ╲
   official rl_deploy             Gate 16 base+residual
     normal segments                 WP15→16 only
               ╲                       ╱
                  joint ownership gate
                         │ /JOINTS_CMD
                         ▼
      s10_perception / MuJoCo simulator (1 kHz)
```

The simulator is subclassed rather than forked. Physics, joint handling, track markers, and
the upstream timer remain in the contest implementation. Two reviewed integration headers
are applied to the SDK by `scripts/patch_upstream.py`:

- `integration/ros_cmd_interface.hpp` adds `/cmd_vel`, autonomous stand-up, command timeout,
  and `/robot_state` observability.
- `integration/joint_command_owner.hpp` ensures only one controller reaches `/JOINTS_CMD`
  during an explicit strategy handover.

The low-level launch file remains conservative when invoked directly. The documented competition
entry point, `scripts/run_race.sh`, enables the reviewed `strategy_gate16.yaml` configuration by
default. The normal follower and official locomotion policy retain control everywhere except the
bounded Gate 16 state sequence. Set `S10_STRATEGY_ROUTER=0` only for an explicit
official-policy-only diagnostic; that is not the validated competition configuration.

## Repository layout

```text
├── src/
│   ├── s10_perception/       MuJoCo sensors, odometry, segment spawn, replay capture
│   ├── s10_auto_nav/         follower, planner, terrain logic, recorder, strategy router
│   └── s10_bringup/          launch files, generated course, tuned parameters
├── integration/              ROS command bridge and joint-command ownership gate
├── policy/gate16/            pinned Gate 16 models, v4 manifest and command profiles
├── policy/stairs_stable/      pinned 57D stair-ascent model and manifest
├── training/
│   ├── s10_climb/            direct-MuJoCo diagnostic/strategy sandbox
│   └── s10_rl/               optional perceptive-policy observation/export scaffold
├── scripts/                  setup, patch, build, race, segment, and pit-failure tools
├── docker/                   reproducible Ubuntu 24.04 / ROS 2 Jazzy environment
├── compose.yaml              portable build/run service with persistent colcon volume
└── upstream/                 contest SDK checkout (generated locally, never committed)
```

## Environment and prerequisites

The submitted code is designed for the contest's Ubuntu/ROS environment and can also be run
through Docker on Linux, macOS, and Windows with WSL 2.

| Component | Supported/required |
|---|---|
| Host CPU | x86_64/AMD64 or ARM64/AArch64 |
| Container runtime | Docker Engine with Compose v2, or Docker Desktop |
| Native operating system | Ubuntu 24.04 LTS |
| ROS | ROS 2 Jazzy |
| Python | 3.12; NumPy `<2.0`, MuJoCo, SciPy, PyYAML, ONNX Runtime |
| Rendering | OSMesa software rendering by default; NVIDIA GPU is optional |
| Resources | 8 GB RAM minimum; 20 GB free disk space recommended for image and build volume |
| External material | Access to the official `goai_embodied_future_material` repository |

The Docker image installs the compiler, colcon, ROS messages, MuJoCo Python runtime, OSMesa,
and ONNX Runtime. The official SDK is not redistributed here; it is checked out separately at
the exact revision used for validation:
`13dd084be6cb5e2514098bc87e586d00dfe580b2`.

The two active Gate 16 ONNX files are part of this repository and are checked at setup time:

| Asset | SHA-256 |
|---|---|
| `policy/gate16/policy.onnx` | `5c1b388f951b282693b4497cd4fd2fd1b53fe1bcd989758c0af42f140a20fb16` |
| `policy/gate16/climb_residual.onnx` | `de61441facb21f0301787b26f168f8447fdc0d83c337eeea884537bbf2468889` |

The Docker base image is pinned by digest and the validated Python packages are locked in
[`docker/requirements.lock`](docker/requirements.lock). Full licenses, model provenance and the
one remaining contributor-license action are listed in
[`docs/THIRD_PARTY.md`](docs/THIRD_PARTY.md).

## Reproduce with Docker (recommended)

These commands keep all generated build output in a Docker volume. Only source code and pinned
policy assets live in Git.

```bash
# 1. Clone the submission.
git clone https://github.com/bowenwan6/goai26-s10-racing.git
cd goai26-s10-racing

# 2. Obtain the official contest SDK using the evaluator/team credentials.
#    Use a full clone so the pinned revision is available offline in the container.
git clone https://github.com/DeepRoboticsLab/goai_embodied_future_material.git \
  upstream/goai_embodied_future_material

# 3. Build the reproducible Ubuntu 24.04 / ROS 2 Jazzy image.
docker compose build

# 4. Select the pinned SDK revision, apply the reviewed integration, and regenerate the course.
S10_UPSTREAM_OFFLINE=1 docker compose run --rm s10 scripts/setup_upstream.sh

# 5. Build the official SDK and all submission packages.
docker compose run --rm s10 scripts/build.sh

# 6. Verify upstream revision, patches, course, policy checksums, and installed ROS packages.
docker compose run --rm s10 scripts/verify_install.sh

# 7. Start the complete WP0->WP32 competition stack.
docker compose run --rm s10 scripts/run_race.sh --headless
```

The last command starts `rl_deploy`, MuJoCo, perception, navigation, and the Gate 16 router in
one container and one ROS domain. It uses the competition speed schedule from `nav.yaml` and
enables `strategy_gate16.yaml`; after Gate 16, joint ownership returns to the official policy.
The official simulator prints ordered waypoint events and the final MuJoCo-clock elapsed time.
Press `Ctrl-C` after the evaluator reports the final waypoint if the hosting evaluator does not
terminate the process itself.

### Visualize perception input

The preliminary-round package requires the perception-control input to be visible as points or
rays. On native Ubuntu 24.04 with a graphical desktop, the bundled RViz configuration shows the
horizontal scan, lidar point cloud and body-frame height-map points while the same autonomy stack
runs:

```bash
scripts/run_race.sh viz:=true rviz:=true
```

Keep visualization off for a timed headless lap. It adds display overhead but does not replace or
change the inputs consumed by the controller.

On a clean machine the first MuJoCo XML/model load may take several seconds. During that initial
load the router explicitly commands zero and retains official joint ownership until odometry,
lidar, and height-map data have all arrived. The normal 0.5 s stale-sensor stop and 3.0 s abort
thresholds apply unchanged after the first complete sensor set.

`docker/run.sh` is an equivalent lightweight wrapper for environments that do not use Compose:

```bash
S10_UPSTREAM_OFFLINE=1 REBUILD=1 CONTAINER_NAME=s10-setup \
  docker/run.sh scripts/setup_upstream.sh
CONTAINER_NAME=s10-build docker/run.sh scripts/build.sh
CONTAINER_NAME=s10-check docker/run.sh scripts/verify_install.sh
CONTAINER_NAME=s10-race docker/run.sh scripts/run_race.sh --headless
```

The default Docker bridge network is portable because every ROS process runs in the same
container. Set `DOCKER_NETWORK=host` only when RViz or another ROS process on the host must join
the DDS domain. Docker Desktop may require host networking to be enabled in its settings.

### Capture an independently reviewable full run

The production stack plus recorder can be run from WP0 through WP32 with one deterministic seed.
Write into the ignored bind-mounted `results/` directory, then move the completed artifacts out
of the checkout:

```bash
docker compose run --rm s10 scripts/run_segment.sh \
  --start 0 --end 32 --seeds 1 --seed-from 6 --max-time 2400 \
  --out /ws/results/full_run --tag full --router \
  --router-params /ws/src/s10_bringup/config/strategy_gate16.yaml

mkdir -p ../s10-evidence
mv results/full_run ../s10-evidence/
```

Each run produces a full log, per-tick CSV telemetry, per-run JSON, and an aggregate JSON.
Success must be established from the ordered official waypoint events and telemetry—not merely
from a process exit code.

### Native Ubuntu 24.04

Install ROS 2 Jazzy (desktop or ros-base), colcon, a C++17 toolchain, Git, Python 3.12, and the
Python packages listed above. Then run:

```bash
scripts/setup_upstream.sh
scripts/build.sh
scripts/verify_install.sh
scripts/run_race.sh --headless
```

For a viewer, omit `--headless` on a machine with a working OpenGL display. Do not run plain
`colcon build` in a checkout already using the Docker build volume; `scripts/build.sh` selects
the matching build, install, and log roots consistently.

## Reproducible segment experiments

`scripts/run_segment.sh` launches the production policy, simulator, perception, follower,
local planner, and recorder. Its only test-specific changes are a storey-aware spawn at the
starting waypoint and a sliced course. A segment run is not an official scored lap.

```bash
# Continuous WP17→WP32, one seed, no resets after spawning at WP17.
CONTAINER_NAME=s10-wp17-32 docker/run.sh scripts/run_segment.sh \
  --start 17 --end 32 --seeds 1 --max-time 1200 \
  --out /ws/results/wp17_32 --tag verify

# Full WP0→WP32 test with the stable Gate 16 policy enabled.
CONTAINER_NAME=s10-wp0-32 docker/run.sh scripts/run_segment.sh \
  --start 0 --end 32 --seeds 1 --max-time 2400 \
  --out /ws/results/wp0_32 --tag verify --router \
  --router-params /ws/src/s10_bringup/config/strategy_gate16.yaml
```

Each seed writes a per-tick CSV, a per-run JSON summary, and a complete ROS/simulator log.
The driver also writes an aggregate JSON file. `results/` is intentionally gitignored.

The recorder's `reached` field says that the final target ended the run; it is not a substitute
for independently checking every intermediate gate in strict order. For acceptance work,
score the CSV at the current 0.18 m internal radius and also retain the official simulator's
independent 0.20 m events. Always retain the per-gate closest distances.

### One-shot real-time video

Video generation additionally requires FFmpeg and a TrueType font on the host. The script finds
Arial on macOS and DejaVu Sans on common Linux distributions automatically; use
`S10_VIDEO_FONT=/absolute/path/font.ttf` elsewhere.

```bash
# MuJoCo run, lightweight capture, offline 1920x1080 render, and MP4 encode.
# Arguments: OUTPUT_DIR [START=0] [END=32] [SEED=6] [MAX_TIME=2400]
S10_VIDEO_TIMING=simulation scripts/run_realtime_video.sh \
  /absolute/path/outside/the/repository/wp0_wp32 0 32 6 2400
```

The command deliberately stores raw evidence and video outside the repository. During the
experiment, the simulator records lightweight robot state plus a monotonic wall-clock timestamp
for every captured frame. After the run, `render_replay_3d.py` renders those states offline and
writes an ffconcat timeline from the recorded timestamps without high-resolution rendering
perturbing control timing. `S10_VIDEO_TIMING=simulation` uses the official MuJoCo clock and
trims at the evaluator's logged WP0→WP32 elapsed time; the default `wall` mode retains the
machine's monotonic elapsed timeline (for example, a 900 s run produces a 900 s video). The
final MP4 is H.264, 1920x1080, 30 fps. The script exits without rendering if the experiment
fails, so a video file cannot disguise a failed run.

Offline rendering uses four deterministic workers by default; set `S10_RENDER_JOBS` to match
available Docker memory. If capture has already succeeded and rendering was interrupted, rerun
the same command with `S10_SKIP_CAPTURE=1` to resume from the retained replay without repeating
the experiment. Before encoding, the script checks that every replay frame exists.

## Important run semantics

- A normal race has no spawn override. Segment spawning happens only when `S10_SPAWN_XY` is
  set by `run_segment.py`.
- A WP17 segment does **not** drive out of the WP16 pit. The harness finds legal ground on the
  WP17 storey, faces WP18, initializes the official crouched pose, and lets the SDK stand up.
- Waypoints advance only after entering the internal 0.18 m radius. The old 0.35 m follower radius is no longer
  used because it could invalidate the ordered tail.
- `S10_USE_VIEWER=0` disables the viewer. Headless wall-clock time and accelerated MuJoCo
  simulation time are different measurements; reports must say which one they use.
- Name long-running containers. An orphan on the shared ROS domain can continue publishing
  `/cmd_vel` into a later experiment.

## ROS interfaces

Published by `s10_perception` in addition to upstream `/IMU_DATA` and `/JOINTS_DATA`:

| Topic | Type | Nominal rate | Meaning |
|---|---|---:|---|
| `/ground_truth/odom` | `nav_msgs/Odometry` | 50 Hz | MuJoCo base pose and twist |
| `/scan` | `sensor_msgs/LaserScan` | 50 Hz | Horizontal ring from the ray-cast lidar |
| `/perception/lidar` | `std_msgs/Float32MultiArray` | 50 Hz | 8×64 range image |
| `/perception/heightmap` | `std_msgs/Float32MultiArray` | 50 Hz | 13×9 yaw-aligned terrain grid relative to the base |

Core autonomy and SDK integration topics:

| Topic | Type | Producer → consumer |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | follower/router → SDK command bridge |
| `/robot_mode` | `std_msgs/UInt8` | optional operator override → SDK command bridge |
| `/robot_state` | `std_msgs/UInt8` | SDK command bridge → observers |
| `/nav/progress` | `std_msgs/Float32` | follower → recorder/router |
| `/nav/terrain` | `std_msgs/String` | follower → recorder/router |
| `/nav/finished` | `std_msgs/Bool` | follower → recorder/router |

When `strategy_router:=true`, the follower moves to `/strategy/nav_cmd_vel`, the router becomes
the sole `/cmd_vel` publisher, and coordinates explicit requests to the SDK's always-present
joint-ownership gate. Direct `race.launch.py` use defaults off for diagnostics; the documented
competition wrapper `scripts/run_race.sh` defaults it on.

Ground-truth odometry is a simulation interface. A hardware deployment must supply an
equivalent odometry source; this repository does not currently implement hardware localization.

## Key shipped parameters

Tuning lives in [`src/s10_bringup/config/nav.yaml`](src/s10_bringup/config/nav.yaml).

| Parameter | Value | Purpose |
|---|---:|---|
| `max_forward` | 2.1 m/s | Guarded ceiling on the validated clear, level legs |
| `terrain_max_forward` | 1.2 m/s | General terrain-aware ceiling |
| `max_lateral` | 0.4 m/s | Lateral command ceiling |
| `max_yaw_rate` | 0.7 rad/s | Yaw-rate ceiling |
| `forward_slew` | 5.0 m/s² | Forward command-rate limit |
| `climb_speed` | 1.1 m/s | Non-Gate-16 short-step command |
| `corner_retreat_speed` | 0.35 m/s | WP26/WP27 safe retreat speed |
| `route_hint_speed` | 0.75 m/s | Body-clear WP31/WP32 connector speed |
| `lookahead` | 1.5 m | Base pure-pursuit target distance |
| `score_radius` | 0.18 m | Internal gate acceptance radius; official evaluator remains 0.20 m |
| `advance_radius` | 0.18 m | Outer approach/advance bound; intentionally no wider than internal scoring |
| `pivot_threshold_deg` | 30° | Hold translation while a new leg is far off heading |
| `stall_timeout` | 2.5 s | Fast low-speed wedge watchdog |
| `progress_timeout` | 12.0 s | Slow no-progress watchdog for oscillating stalls |
| `corridor_half_width` | 0.45 m | Body-clear local-planner corridor |
| `max_step` | 0.35 m | Terrain-classifier step threshold, not a universal climb limit |

Several later-course behaviors are intentionally route-specific because they encode measured
geometry: corner retreat at WP26/WP27, the WP28 run-up, and committed terrain handling at
WP28/WP30. The reasoning and supporting measurements are recorded beside the parameters.

## Optional perceptive-policy scaffold

`training/s10_rl/observation.py` defines a possible 174-D observation: the official 57-D
proprioceptive prefix followed by a 13×9 height map. `export_onnx.py` validates tensor names and
dimensions for a future trained checkpoint.

This is not wired into the current SDK runner. Exporting a graph alone does not make it the
deployed policy; a compatible trained checkpoint and a deliberate runner update are both
required.

## Development and verification

```bash
# Build the packages affected by autonomy changes.
CONTAINER_NAME=s10-build docker/run.sh scripts/build.sh \
  --packages-select s10_perception s10_auto_nav s10_bringup

# Run the current navigation and perception tests in the built container environment.
CONTAINER_NAME=s10-tests docker/run.sh bash -lc \
  'source "$S10_INSTALL_BASE/setup.bash" && \
   python3 -m pytest -q src/s10_perception/test src/s10_auto_nav/test'

# Source/configuration checks.
ruff check .
ruff format --check .
scripts/extract_waypoints.py --check
scripts/patch_upstream.py --check
```

For the v1.5 fallback plus speed-scheduling revision, the navigation/perception suite passes
**382 tests**, the training/observation suite passes **29 tests**, and the three affected ROS
packages build cleanly. Ruff lint and formatting checks also pass. The simulation-timed replay
overlay tests run in the core suite while final media remains outside the repository.

Never hand-edit `src/s10_bringup/config/course.yaml`; regenerate it from the upstream scene.
Never edit `upstream/` directly; put SDK integration in `integration/` and apply it through
`scripts/patch_upstream.py`.

See [CONTRIBUTING.md](CONTRIBUTING.md) for conventions and review requirements.

## Build the submission ZIP

After the final reviewed branch is merged and pushed to `main`, create the compressed self-test
code package directly from the clean Git revision. The output directory must be outside the
repository:

```bash
scripts/package_submission.sh /absolute/path/to/resources/submission_20260820
```

The script rejects dirty tracked files and non-`main` revisions, excludes untracked SDK/build/log/
video material by using `git archive`, tests the ZIP, checks required runtime files, and writes a
SHA-256 sidecar plus a commit/evidence manifest. The MP4 is uploaded separately; see
[`docs/SUBMISSION.md`](docs/SUBMISSION.md) for the exact filename and checksum.

## Acknowledgements

Built on contest material published by [DEEP Robotics](https://github.com/DeepRoboticsLab) for
GOAI 2026 Track 4. The simulator, robot model, track scene, and deployment SDK are theirs;
the perception, navigation, experiment harnesses, and integration layer in this repository are
ours.

Licensed under [BSD-3-Clause](LICENSE), matching upstream.

## HIM ONNX / S10 实机接入

HIM 1500 遥控器入口：`scripts/start_s10_him1500_handset.cmd`。
HIM 仿真使用 `scripts/patch_him_upstream.py` 和 `scripts/run_him.sh`；
默认比赛入口保持现有导航与 Gate16 控制。
接口、构建、起身接管与验证结果见 [S10 HIM 接入说明](docs/S10_HIM_DEPLOYMENT.md)。
