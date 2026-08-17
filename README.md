<div align="center">

# goai26-s10-racing

**Perception-driven autonomous navigation for the DEEP Robotics Lynx S10**

GOAI 2026 · Track 4 *Embodied Future* · Challenge 2 — S10 Perception Racing Contest

[![License](https://img.shields.io/badge/license-BSD--3--Clause-blue.svg)](LICENSE)
[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E.svg?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-E95420.svg?logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/24.04/)
[![MuJoCo](https://img.shields.io/badge/MuJoCo-simulation-000000.svg)](https://mujoco.org/)
[![Python](https://img.shields.io/badge/Python-3.12-3776AB.svg?logo=python&logoColor=white)](https://www.python.org/)

</div>

---

## Current status

The generated course contains **33 waypoints**, spans **224.21 m horizontally**, and gains
**6.70 m**. Gates count only inside a **0.2 m horizontal radius** and must be taken in order.

The current `main` has completed one uninterrupted full-stack run from **WP17 to WP32**:

| Result | Value |
|---|---:|
| Ordered target gates | 15/15 (WP18–WP32) |
| Recorder elapsed time | 641.1 s |
| Distance travelled | 119.59 m |
| Maximum tilt | 41.9° |
| Recorder stalls | 2 |

That is a segment acceptance result, not a complete-lap claim. The harness legally test-spawns
the robot at WP17 and does not reset it afterward. **WP15→WP16 remains unresolved**, so this
repository does not yet claim a successful WP0→WP32 autonomous run.

## What is actually deployed

The contest SDK's shipped **57-dimensional proprioceptive locomotion policy is unchanged**.
Perception is used by the navigation layer to decide velocity commands; it is not appended to
the deployed policy observation.

| Layer | Responsibility |
|---|---|
| Perception | MuJoCo ray-cast lidar, horizontal scan, storey-aware body-frame height map, and ground-truth odometry |
| Navigation | Strict ordered-gate tracking, pure pursuit, terrain classification, body-clear local planning, barrier bypass, corner retreat, step commitment, run-up, and stall recovery |
| Locomotion | Official SDK ONNX policy at 50 Hz, receiving `/cmd_vel` through the ROS command bridge |
| Integration | Autonomous startup plus single-owner arbitration at the SDK's `/JOINTS_CMD` write point |

`training/s10_rl/` contains a perceptive-observation/export research scaffold. It is useful for
future retraining, but it is **not** the policy used for the validated WP17→WP32 result.

## Architecture

```text
 /ground_truth/odom    /scan    /perception/heightmap
          │              │                │
          └──────────────┴────────────────┘
                         ▼
              s10_auto_nav / follower
       pure pursuit + terrain + avoidance + recovery
                         │
                      /cmd_vel
                         ▼
        contest rl_deploy + ros_cmd_interface.hpp
          official locomotion policy (50 Hz)
                         │
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

The strategy router is available but **disabled by default**. The validated WP17→WP32 run uses
the normal follower and official locomotion policy; it does not depend on an unfinished
special-purpose WP16 climb policy.

## Repository layout

```text
├── src/
│   ├── s10_perception/       MuJoCo sensors, odometry, segment spawn, replay capture
│   ├── s10_auto_nav/         follower, planner, terrain logic, recorder, strategy router
│   └── s10_bringup/          launch files, generated course, tuned parameters
├── integration/              ROS command bridge and joint-command ownership gate
├── training/
│   ├── s10_climb/            direct-MuJoCo diagnostic/strategy sandbox
│   └── s10_rl/               optional perceptive-policy observation/export scaffold
├── scripts/                  setup, patch, build, race, segment, and pit-failure tools
├── docker/                   reproducible Ubuntu 24.04 / ROS 2 Jazzy environment
└── upstream/                 contest SDK checkout (generated locally, never committed)
```

## Quick start with Docker

Docker is the recommended path on macOS and is also useful on Linux. You need Git credentials
for the contest material and a host Python environment capable of running the course extractor.

```bash
# 1. Clone this repository.
git clone <this-repository-url>
cd goai26-s10-racing

# 2. Host-side dependencies used while preparing the upstream checkout.
python3 -m pip install "numpy<2.0" mujoco pyyaml

# 3. Clone, patch, and validate the contest SDK; regenerate course.yaml.
scripts/setup_upstream.sh

# 4. Build the SDK and all project packages into the persistent Docker volume.
CONTAINER_NAME=s10-build docker/run.sh scripts/build.sh

# 5. Run the competition-style stack without a viewer.
CONTAINER_NAME=s10-race docker/run.sh scripts/run_race.sh --headless
```

`docker/run.sh` builds `s10-racing:dev` when necessary, mounts this checkout at `/ws`, and
stores colcon output in the `s10-racing-build` named volume. This avoids synchronized-folder
corruption and keeps generated build files out of the repository.

### Native Ubuntu 24.04

With ROS 2 Jazzy and the dependencies installed locally, the corresponding native commands
are:

```bash
scripts/setup_upstream.sh
scripts/build.sh
scripts/run_race.sh             # viewer
scripts/run_race.sh --headless  # no viewer
```

Do not run `colcon build` with its defaults in a checkout that already uses the Docker build
volume. `scripts/build.sh` consistently selects the intended build and install roots.

## Reproducible segment experiments

`scripts/run_segment.sh` launches the production policy, simulator, perception, follower,
local planner, and recorder. Its only test-specific changes are a storey-aware spawn at the
starting waypoint and a sliced course. A segment run is not an official scored lap.

```bash
# Continuous WP17→WP32, one seed, no resets after spawning at WP17.
CONTAINER_NAME=s10-wp17-32 docker/run.sh scripts/run_segment.sh \
  --start 17 --end 32 --seeds 1 --max-time 1200 \
  --out /ws/results/wp17_32 --tag verify

# Full recorded WP0→WP32 attempt. This currently encounters the unresolved WP16 problem.
CONTAINER_NAME=s10-wp0-32 docker/run.sh scripts/run_segment.sh \
  --start 0 --end 32 --seeds 1 --max-time 2400 \
  --out /ws/results/wp0_32 --tag verify
```

Each seed writes a per-tick CSV, a per-run JSON summary, and a complete ROS/simulator log.
The driver also writes an aggregate JSON file. `results/` is intentionally gitignored.

The recorder's `reached` field says that the final target ended the run; it is not a substitute
for independently checking every intermediate gate in strict order. For acceptance work,
score the CSV against `src/s10_bringup/config/course.yaml` at the 0.2 m radius and retain the
per-gate closest distances.

### Segment frame capture

```bash
CONTAINER_NAME=s10-video docker/run.sh scripts/run_segment.sh \
  --start 17 --end 32 --seeds 1 --max-time 1200 \
  --out /ws/results/video --tag capture --video --video-hz 10

ffmpeg -framerate 10 -i results/video/17_32_capture_seed0_frames/%05d.png \
  -c:v libx264 -crf 18 -pix_fmt yuv420p results/video/wp17_32.mp4
```

Direct rendering runs in the simulator process and may change timing at high resolution.
The simulator also has a low-cost `replay` mode for external offline-rendering workflows, but
the complete in-repository workflow above deliberately uses the PNG mode exposed by
`run_segment.sh`.

## Important run semantics

- A normal race has no spawn override. Segment spawning happens only when `S10_SPAWN_XY` is
  set by `run_segment.py`.
- A WP17 segment does **not** drive out of the WP16 pit. The harness finds legal ground on the
  WP17 storey, faces WP18, initializes the official crouched pose, and lets the SDK stand up.
- Waypoints advance at the scorer's 0.2 m radius. The old 0.35 m follower radius is no longer
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
joint-ownership gate. This is optional and is not the default race configuration.

Ground-truth odometry is a simulation interface. A hardware deployment must supply an
equivalent odometry source; this repository does not currently implement hardware localization.

## Key shipped parameters

Tuning lives in [`src/s10_bringup/config/nav.yaml`](src/s10_bringup/config/nav.yaml).

| Parameter | Value | Purpose |
|---|---:|---|
| `max_forward` | 0.7 m/s | Official policy's validated command ceiling |
| `max_lateral` | 0.4 m/s | Lateral command ceiling |
| `max_yaw_rate` | 0.7 rad/s | Yaw-rate ceiling |
| `lookahead` | 1.4 m | Pure-pursuit target distance |
| `score_radius` | 0.2 m | Gate acceptance radius |
| `advance_radius` | 0.2 m | Outer approach/advance bound; intentionally no wider than scoring |
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

At the merge that established the WP17→WP32 result, the combined navigation and perception
suite passed **326 tests**.

Never hand-edit `src/s10_bringup/config/course.yaml`; regenerate it from the upstream scene.
Never edit `upstream/` directly; put SDK integration in `integration/` and apply it through
`scripts/patch_upstream.py`.

See [CONTRIBUTING.md](CONTRIBUTING.md) for conventions and review requirements.

## Acknowledgements

Built on contest material published by [DEEP Robotics](https://github.com/DeepRoboticsLab) for
GOAI 2026 Track 4. The simulator, robot model, track scene, and deployment SDK are theirs;
the perception, navigation, experiment harnesses, and integration layer in this repository are
ours.

Licensed under [BSD-3-Clause](LICENSE), matching upstream.
