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
# 1. Dependencies
sudo apt install libevdev-dev
sudo adduser "$USER" input && newgrp input     # keyboard capture, for manual driving
pip install "numpy<2.0" mujoco pyyaml

# 2. Clone and fetch the contest SDK
git clone <this-repo> && cd goai26-s10-racing
scripts/setup_upstream.sh

# 3. Build
scripts/build.sh

# 4. Race
scripts/run_race.sh
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
scripts/build.sh --packages-select s10_auto_nav     # rebuild one package
BUILD_PLATFORM=arm scripts/build.sh                 # cross-build for the robot
scripts/extract_waypoints.py --check                # confirm the course is current
scripts/patch_upstream.py --revert                  # restore a pristine SDK checkout

S10_MUJOCO_XML=/abs/path/model.xml scripts/run_race.sh    # custom scene
```

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
