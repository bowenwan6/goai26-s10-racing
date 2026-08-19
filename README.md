<div align="center">

# goai26-s10-racing

**Perception-driven autonomous navigation for the DEEP Robotics Lynx S10**

GOAI 2026 · Track 4 *Embodied Future* · Challenge 2 — S10 Perception Racing Contest

[![License](https://img.shields.io/badge/license-BSD--3--Clause-blue.svg)](LICENSE)
[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E.svg?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-E95420.svg?logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/24.04/)
[![MuJoCo](https://img.shields.io/badge/MuJoCo-simulation-000000.svg)](https://mujoco.org/)
[![Python](https://img.shields.io/badge/Python-3.12-3776AB.svg?logo=python&logoColor=white)](https://www.python.org/)
[![Version](https://img.shields.io/badge/version-Ver1_candidate-6f42c1.svg)](#version-status)

</div>

---

## Current status

### Version status

`main` contains the reviewed Gate 16 v1.5 confidence-fallback integration with its competition
default fixed to the stable frontal fallback. The `bw-test-sub7min-autonomy-v1` branch is the
current **Ver1.0 speed candidate**: it retains that Gate 16 controller, keeps the experimental
stairs policy disabled, and adds measured per-leg speed scheduling without relaxing ordered-gate
or safety checks.

The generated course contains **33 waypoints**, spans **224.21 m horizontally**, and gains
**6.70 m**. The official evaluator counts gates inside a **0.20 m horizontal radius** and in
strict order. Ver1 navigation uses a tighter **0.18 m internal acceptance radius**, giving
0.02 m of margin before the official threshold.

The speed candidate completed one uninterrupted full-stack test run from **WP0 to WP32**
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

This is a Ver1 candidate test-harness result: the production stack ran continuously, while the
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
to the stable frontal fallback. The strategy router executes the bundled command-profile JSON
from measured wheel centres: `settle` begins only when both physical front wheels have cleared the
edge and reached deck height, lasts 30 policy steps, and then changes to `push` until the router's
independent four-wheel check succeeds. The policy stays in its native frontal frame; adaptive
mirroring is not enabled by the competition configuration.
The official controller owns the moving approach until the full `d=0.60–0.65 m`,
`v=0.23–0.27 m/s`, `|yaw|≤5°` envelope is held. Gate 16 shadow inference cannot slow the
approach; on the ownership edge its previous-action field is seeded by inverse-decoding the
measured actuator state, avoiding a discontinuity between the unrelated official and Gate 16
policy histories.
After all four wheels are verified on the upper platform, the residual is disarmed and joint
ownership returns to the official policy; the existing follower resumes at 0.5 m/s.

The delivered `policy/stairs57/policy.onnx` is staged behind a separate single-owner path but
is **disabled by default**. Its tensor contract and checksum pass, while isolated full-stack
WP18→19 and WP17→18 trials with the current SDK action decoder fell at 61–68°; enabling it
without the teammate's exact working runner/normalization contract would be unsafe. Gate 16
remains isolated from this experimental path.

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

The strategy router remains **disabled by default** for conservative compatibility. Enable the
reviewed stable policy explicitly with `strategy_gate16.yaml`; the normal follower and official
locomotion policy retain control everywhere except the bounded Gate 16 state sequence.

## Repository layout

```text
├── src/
│   ├── s10_perception/       MuJoCo sensors, odometry, segment spawn, replay capture
│   ├── s10_auto_nav/         follower, planner, terrain logic, recorder, strategy router
│   └── s10_bringup/          launch files, generated course, tuned parameters
├── integration/              ROS command bridge and joint-command ownership gate
├── policy/gate16/            pinned Gate 16 models, v4 manifest and command profiles
├── policy/stairs57/           staged 57D stair model and fail-closed manifest
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

# Full WP0→WP32 test with the stable Gate 16 policy enabled.
CONTAINER_NAME=s10-wp0-32 docker/run.sh scripts/run_segment.py \
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
**380 tests**; the three affected ROS packages also build cleanly. The simulation-timed replay
overlay tests pass separately while the final media is produced outside the repository.

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
