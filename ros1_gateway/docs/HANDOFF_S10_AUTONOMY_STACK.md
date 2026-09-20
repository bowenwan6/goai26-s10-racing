# S10 real-robot autonomy stack: handoff

Interfaces, functions and background for whoever continues this work.
Written 2026-09-20. Everything here was checked against the code and the field logs of 2026-09-18…20; where something is only believed, it says so.

**Status marks:** ✅ verified on the real robot · 🧪 verified offline only (mock robot / simulation / synthetic data) · 📝 written, never run · ❌ not written · ❓ unknown, needs a measurement

No password, key, licence code or public address is in this file, and none may be added. Ask the project owner for access.

---

## 0. Read this first

**What the project is.** GOAI 2026, Track 4 "Embodied Future", Challenge 2: the DEEP Robotics Lynx S10 (wheeled-legged quadruped) has to drive a 30-waypoint outdoor course (stairs, ledges, slopes, a rock creek bed) autonomously. The August round was simulation only (released as Ver1.0). September is the real robot.

**What runs today on the real robot (dog 050):**

| Layer | State |
|---|---|
| Lidar + IMU into ROS 1 on our computer | ✅ byte-identical to the robot's own data, 10 Hz / 200 Hz, 10-minute soak clean |
| Vendor SLAM (x_nav) mapping, pose output `/base_link/odom` 10 Hz | ✅ maps; **no map has ever been saved** (the computer reset before "save" both times) |
| Data-collection web app `/teach` (mapping recording, waypoint marks, taught path) | ✅ mapping recording used twice; marks and taught path never used on the robot |
| Navigation node (the team's route runner) | 🧪 simulation 30/30, ROS 1 end-to-end with fake data; never fed real data |
| Motion control node (velocity, stand / lie, gait switch) | 🧪 35/35 checks against a mock robot (+ 7 in dry run); on the robot only in dry run. **The robot has never moved under our software.** |
| Remote ↔ autonomous hand-over | 📝 status read works ✅; mode switch never tried; not wired into the flow ❌ |

**The three things most likely to hurt you:**

1. **The robot is shared.** Boards 103 and 106 belong to everybody. We change nothing on 103, and on 106 only one user-level process in one directory, removed by `robot_session.sh down`. Run `down` (and `unkeys`) before every hand-back.
2. **No motion without an operator on site holding the remote, confirming each step.** The remote is the only real emergency stop. Our "Nav stop" is a software zero-velocity latch.
3. **Our on-robot computer (AGX) resets by itself**, several times a day, twice in the middle of mapping. Hardware reset reason `SYS_RESET_N` (power or reset line; thermal, watchdog and software resets report other codes). Cause not found. Save early, fsync what matters, and do not go on stairs until it is understood.

**Where to go next:** the staged test plan, with pass criteria and the list of fixes required before the first motion, is [NAV_TEST_PLAN_20260920.md](NAV_TEST_PLAN_20260920.md). Section 8 of this file is its short form.

---

## 1. Background

### 1.1 Robots and computers

| Item | What it is | Notes |
|---|---|---|
| Dog **050** (SN `CS10100050`) | Current robot, shared with other teams | Wi-Fi `S10 PRO-050-5G` |
| Dog 048 | Previous robot, broken | Older docs and the old phone pages (`/field`, `/native-nav`, …) refer to it |
| **102** = AGX Orin, `10.21.33.102` | **Ours.** Ubuntu 24.04 aarch64, ROS 2 Jazzy, 54 GB eMMC. Mounted on the dog, moved from 048 to 050 on 2026-09-19 | Users: `golai` (no sudo, in the `docker` group) and an admin account with sudo |
| **103** `10.21.33.103` | Motion board: locomotion, PTP master, ASDU server | Never modified by us |
| **106** `10.21.33.106` | Localisation board: vendor lidar driver, official SLAM (kept **off** on 050 by decision) | Only our lidar tap runs here, as the normal user |

- **103 and 106 have identical SSH host keys on every dog** (vendor image), and 106 has the same IP. Host-key pinning cannot tell dogs apart: identify the dog by the serial in 106 `/var/opt/robot/conf/robot_manufacturing_info.toml` (`robot_session.sh` prints it).
- **Clock:** the AGX follows the robot's PTP clock (`s10-ptp4l` + `s10-phc2sys` systemd units, internet NTP off; `scripts/agx_ptp/` has setup and rollback). Before this the robot was 34 s ahead of the AGX. Lidar stamp minus AGX clock is about −0.13 s. After a cold boot the AGX clock reads 1970 until PTP locks.
- **Reaching the AGX:** on site over the dog's Wi-Fi/LAN at `10.21.33.102`; remotely through a userspace Tailscale SSH alias (`s10-48-remote`), which works only while the AGX has internet and is slow (relayed). A Mac on another network cannot open the web pages directly; `scripts/mac_tunnel.sh` forwards them to `localhost:18080` (app) and `localhost:18000` (x_nav).

### 1.2 Two ways to make the dog walk

| | A. RL policies through the contest SDK | B. Vendor gaits through `/NAV_CMD` |
|---|---|---|
| What commands the joints | Our ONNX policy → `/JOINTS_CMD` at 50 Hz, inside the SDK process | The robot's own controller (`libCD1RunPolicy.so`, black box) |
| What we send | 16 joint targets | Body velocity (m/s, rad/s) + a gait code |
| Needs | SDK mode licence per SoC, a patched SDK runner on the robot, reboot | Nothing installed on 103 |
| Used in | August simulation contest; September MuJoCo navigation (J3100 walk + 1150 stairs) | **The real dog, now** |

Decision (2026-09-19): on the shared real dog we use **B**. The navigation code does not care: it asks for an "owner" (`official` = walking actor, `stairs_stable` = stairs actor) and our ROS 1 wrapper maps that to a gait (`flat` 0x3002 / `stairs` 0x3003). Plugging the RL policies back in later means replacing that mapping and the control node, not the planner.

### 1.3 The course and its map

- 30 official waypoints. **The official order is the reverse of the 30 waypoint photos**: WP01 = door "Start" (last photo), WP30 = far end (first photo).
- Reference map "v3": official-SLAM map `0914_fr_v3-20260914-142008` of dog 048. Everything validated in simulation lives in this frame: `tools/wp_match/out/route_v2.json` (waypoints placed from photo timestamps, uncertainty 1.5–3 m, 256.6 m long), `course_terrain.npz`, `map_surface.npz`, `overrides_v1.json`, 16 stairs zones, the MuJoCo scene.
- Hard terrain (gabion ledge, low platform / yellow bumps, rock creek bed) is driven in the stairs gait, slowly, without detours.
- x_nav localises only in maps it built itself, in its own frame. The chosen flow is therefore **teach-and-repeat in the x_nav map**: map with loop closure → save → localise → mark the 30 waypoints and the stairs switch points → drive the taught path once → generate the route from that. The old v3 route is a reference, not an input. Design: `docs/NEW_SLAM_XNAV_INTEGRATION_ZH.md` and `docs/S10_NAVIGATION_PLANNING_REDESIGN_ZH.md` in the main repo.

### 1.4 Frames and geometry

| Fact | Value | State |
|---|---|---|
| `/LIDAR/POINTS` frame `lidar_link` | The vendor driver's **merged** cloud of the front and back lidars, already in `base_link` (URDF: front lidar x=+0.22341, back x=−0.22341, z=−0.0001). Extrinsics for us: identity | ✅ ground plane fitted on live data: tilt 0.12° / −0.26° |
| Body height above ground | lying: 0.080 m ✅. Standing: ❓ (`body_z_offset: 0.27` in `config/nav.yaml` is a placeholder) | measure with `tools/check_ground_plane.py` while standing |
| IMU `frame_id` | empty string | ✅ |
| x_nav pose `/base_link/odom` | `nav_msgs/Odometry`, map frame of the current x_nav map, 10 Hz. Whether its **twist** is filled ❓ | the runner uses forward speed and yaw rate |
| Route z | `z_reference: ground` = pose z − `body_z_offset`. The same offset must be used by the route tool and by `nav.yaml` | |

### 1.5 Glossary

| Term | Meaning |
|---|---|
| WP | Waypoint (red dot on the course), scored within 0.20 m |
| SWIN / SWOUT | Marked switch points: hand over to the stairs gait/actor before the first step, hand back after the last |
| route_v2 | Our route file: waypoints + one taught centreline per segment + gait, speed, corridor |
| maneuver / zone | Arc-length interval of the route where the stairs actor is needed |
| owner | Who drives: `official` (walk) or `stairs_stable` (stairs). On the real dog = gait `flat` / `stairs` |
| shadow mode | The navigation node computes and reports but never publishes velocity |
| dry run | The control node reads feedback and logs what it would send, creates no robot-side publisher |
| armed | Control node started with `--enable-motion` |
| ASDU | The robot's JSON-over-UDP/TCP service protocol (heartbeat, status, use mode, …) |
| tap | Our read-only forwarder of the lidar topic from 106 to the AGX |

---

## 2. Data flow

```
106  vendor lidar driver ── /LIDAR/POINTS (ROS 2, host-local) ──► tap (user process) ──TCP 47631──┐
103  /IMU, /MOTION_INFO  (ROS 2 DDS, domain 0) ───────────────────────────────────────────────────┤
                                                                                                   ▼
AGX  s10_ros1_gateway (ROS 2 → ROS 1, one way) ──► ROS 1 /LIDAR/POINTS 10 Hz, /IMU 200 Hz
                          │                                   │
                          ▼                                   ▼
      x_nav container (SLAM only) ── /base_link/odom ──► s10_rl_nav_ros1  (route follower + route runner)
      web UI :8000                                        │  /rl_nav/cmd_vel   /rl_nav/gait_request
                          │                               ▼
      teach worker + web app :8080 (/teach)        s10_ros1_control  (limits, timeouts, gait switch at standstill,
      records bags / marks / taught path                  │           single velocity source, dry run by default)
      to the external SSD                                 ▼
                                             ROS 2 /NAV_CMD  /GAIT  /MOTION_STATE ──► 103 native gaits
```

ROS 1 master: the x_nav container's `roscore` owns port 11311 (the container must start before the gateway). ROS 1 runtime for our nodes: ROS-O ("one", Noetic line) user-space bundle at `~/ros1_gateway/ros1` (`source ros1/ros1_env.sh`).

---

## 3. Interfaces

### 3.1 Robot-native ROS 2 (package `drdds`, domain 0, Fast DDS)

Message definitions are copied in `src/drdds/msg/` (hashes in `SOURCE_SHA256.txt`). Every message is `MetaType header` (`uint64 frame_id`, `builtin_interfaces/Time stamp`) + `data`.

| Topic | Type | Direction | Content |
|---|---|---|---|
| `/LIDAR/POINTS` | `sensor_msgs/PointCloud2` | out of 106, **host-local by vendor design** | 10 Hz, ~45–55 k points, `point_step` 26: `x y z` float32, `ring` uint16 @16, `timestamp` float64 @18 |
| `/IMU` | `sensor_msgs/Imu` | out | 200 Hz |
| `/ODOM` | `nav_msgs/Odometry` | out of 106 official localisation | **not running on 050** |
| `/MOTION_INFO` | `drdds/MotionInfo` | out of 103 | `vel_x vel_y vel_yaw height`, `motion_state.state`, `gait_state.gait`, `payload`, `remain_mile` |
| `/NAV_CMD` | `drdds/NavCmd` | **into** 103 | `x_vel y_vel yaw_vel` (m/s, rad/s). Developer guide: fixed 10 Hz, obeyed only in RL control with a navigation gait |
| `/GAIT` | `drdds/Gait` | into 103 | `gait`: `0x3002` navigation flat, `0x3003` navigation stairs (`0x1001` basic and `0x1003` stairs are the remote's "regular" gaits). Switch only at a standstill |
| `/MOTION_STATE` | `drdds/MotionState` | into 103 | `state`: 1 stand, 4 lie down, 17 RL control, 2 joint damping (soft e-stop: the robot drops; **we never send it**), 0 idle, 3 boot damping |
| `/NODECTL_CMD_103`, service `/NODECTL_QUERY_<soc>` | `drdds/NodeCtlCmd`, `drdds/srv/NodeCtlQuery` | vendor service manager | Used only by the older `native_transfer/control_source_admin.py` (query / stop `handler.service`). **We do not stop vendor services.** |

- Two native `/NAV_CMD` publishers always exist on 050 (103 `handler`, 106 `localPlanner`). Measured: **0 messages while the dog is idle** ✅. Whether they send in navigation mode ❓.
- ❓ Whether `/NAV_CMD` is accepted in ASDU use mode 0 (remote) or only in mode 1 (navigation). ❓ What the robot does when `/NAV_CMD` stops abruptly: the "0.5 s command timeout" exists only in our mock robot, the SDK text does not document it.
- DDS environment on the AGX: `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `ROS_DOMAIN_ID=0`, `FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds_ethernet.xml`. On 106: `/opt/robot/fastdds.xml`.
- 106 quirk: `/usr/bin/taskset` carries a file capability, so glibc strips `LD_LIBRARY_PATH` and `rclpy` breaks under it. The tap sets its CPU affinity itself.

### 3.2 ASDU service protocol (developer guide V1.0.1)

`10.21.33.103:30004` UDP (DTLS by default; **encryption is disabled on this robot**, plain UDP answers from the AGX ✅), `:30003` TLS.

- APDU = 16-byte header + one JSON ASDU. Header: sync `eb 91 eb 90`, body length (u16 LE), message id (u16 LE), format `0x01` = JSON, packet number, version `0x01`, 5 reserved bytes.
- ASDU: `{"PatrolDevice": {"Type", "Command", "Time": "YYYY-MM-DD HH:MM:SS", "Items": {...}}}`.

| Message | Type | Command | Items |
|---|---|---|---|
| Heartbeat (send ≥ 1 Hz; the robot then reports status to the sender at 2 Hz) | `0x00100064` | `0x00000005` | – |
| BasicStatus report | `0x00100064` | `0x00f00000` | `MotionState`, `Gait`, `HES`, `ControlUsageMode`, `PowerManagement`, `Sleep`, `Model`, `Version`, `Charge` |
| **Use-mode switch** | `0x00100002` | `0x00500002` | `Mode`: 0 regular (remote, axis commands), 1 navigation, 2 assisted |
| Motion state / gait / axis commands | `0x00100001` | `0x00200002` / `0x00300002` / `0x00100002` | not implemented by us on purpose |

Tool: `tools/asdu_mode.py status` (read-only, ✅: mode 0, reports at ~2 Hz) and `set-mode N --i-am-on-site` (📝 never run). Standard library only, so it also runs from a laptop on the robot network if the AGX is dead.

### 3.3 Sensor gateway (ours)

| Part | Where | What |
|---|---|---|
| `tap/s10_lidar_tap.py` + `run_tap_106.sh` | 106, `/home/user/ros1_gateway_tap/` | rclpy raw (serialized) subscription to `/LIDAR/POINTS`, only while the gateway has accepted it; forwards CDR bytes over one TCP connection; bounded queue, oldest dropped and counted; `nice 19`, cores 0–3; never publishes into ROS 2 |
| Tap framing | TCP `10.21.33.102:47631`, peer allow-list `[10.21.33.106]` | prefix `<4sHHII>` = magic `S10L`, version 1, kind, header length, payload length; JSON header; kinds HELLO 1, MESSAGE 2, HEARTBEAT 3, WELCOME 4 |
| `src/s10_ros1_gateway` (C++) | AGX | per topic: DDS generic subscription or tap; conversion with `ros1_bridge` `convert_2_to_1_generic` (upstream generated code, commit in the build script); publishes with `topic_tools::ShapeShifter`. Creates **no** ROS 2 publisher, service or parameter server. ROS 1 `Header.seq` is 0 |
| `config/topics_keep_ros2_names.yaml` (default) / `topics.yaml` | | `/LIDAR/POINTS` (tap), `/IMU` (dds), `/ODOM` (dds); the second file renames to `/lidar_points`, `/imu/data`, `/odom` |
| `config/gateway.yaml` | | tap listen address/port, allowed peers, status topic `/s10_ros1_gateway/status`, status file `run/status.json` |
| Status JSON | | per route: `rate_hz`, `published`, `convert_errors`, `stamp_backwards`, `stamp_minus_local_clock_s`, `ros1_subscribers`, `last_frame_id`; tap: `state`, `peer`, `frames`, `protocol_errors`, `rejected_connections`, `tap_reported` (received / sent / dropped …). `scripts/health_check.sh [--hz N]` prints it; `/ODOM` is optional |

✅ Verified on 050: 60 s digest audit against an independent reference on 106: 592/592 clouds and 11845/11845 IMU messages identical. 10-minute soak: gateway ~30 % of one core, 67 MiB flat; tap 8.4 % CPU, 0 drops; vendor lidar driver load unchanged; ~120–200 Mbit/s on the robot LAN (too much for ROS 1 subscribers over Wi-Fi).

Offline path: `tools/mcap_to_ros1_bag.py` (ROS 2 MCAP → ROS 1 bag, serialization-level, read-only source, never overwrites, writes a conversion report) and the audit tools `dump_ros2_digest.py`, `dump_ros1_digest.py`, `compare_digests.py`, `audit_offline.sh` (🧪 synthetic data; no real MCAP converted yet).

### 3.4 Mapping and localisation: vendor x_nav (BWTON "机器人智能巡检管控平台")

Used **only as the SLAM engine**. Its planner and its web-page robot controls are not used.

| Item | Value |
|---|---|
| Container | `nav`, image `…/embodiedai/nav:3.2.1.arm.beta` (arm64), host network, privileged, `restart: "no"`; compose in `/opt/data/compose/` (copy in `vendor/x_nav/`) |
| Licence | hash file `/opt/data/config/ssd_whitelist.conf.hash`, bound to **this AGX's eMMC**: the image runs nowhere else. Not in git |
| Config | `/opt/data/nav_map/x_nav.yaml` (= `/nav_map` in the container). Ours differs from the vendor file in two lines: `sensor_lidar: "EXTERNAL_ROS1"`, `robot_model: "S10"` (NULL crashed the controller; these values start no built-in driver or robot SDK). SLAM inputs `lid_topic: /LIDAR/POINTS`, `imu_topic: /IMU`, `time_sync_en: false` |
| Ports (all interfaces, developer page uses the vendor default password: change it) | 8000 web UI, 9000 Flask API, 8765 foxglove; its `roscore` on 11311 |
| Processes | `robot_web_controller.py` spawns `/nav/x_slam <mode> <map dir>`; seen live: `/nav/x_slam mapping /nav_map/<name>`. "Select map" restarts it as `x_slam localization <map dir>`. "Save map" is the ROS service `/x_nav/slam/service` (`save_map`). SLAM family: FAST-LIO front end + LIO-SAM-style back end (closed binary) |
| Output | `/base_link/odom` (`nav_msgs/Odometry`, 10 Hz, node `/x_nav`). Also `/cmd_vel` and `/web_cmd` from its web page and `w_nav` planner: our control node **ignores** `/cmd_vel` unless told `source x_nav` |
| Maps | `/opt/data/nav_map/<name>/` on the AGX eMMC. Expected content (from the x86 image listing, medium confidence): `global_map.pcd`, ground map, downsized pcds, keyframe pcds, `poses_*.txt`. **Nothing is persisted before "save map".** None saved so far |
| Missing in the ARM image | `map_manager` (the `x_nav` package): waypoints / virtual obstacles of the vendor page may depend on it. Not needed by us |

**Operator steps in the web UI** (`http://10.21.33.102:8000`, vendor manual text in the session notes):
mapping = 地图选择 → 新增地图 → type a name → **Enter** → drive with the remote (red = live cloud, 刷新点云 shows the green global map) → 地图选择 → **保存地图**. Localisation = 地图选择 → 刷新地图 → pick the map → 刷新点云 → 设置位姿 (click the robot's real position, drag the heading) → red and green clouds must overlap. Do **not** use 单点导航 / 动作指令 / the arrow pad.

**Rebuilding a map from a recorded bag** (📝 never tried; only possible on this AGX): stop our gateway and confirm 0 publishers on `/LIDAR/POINTS` and `/IMU`; fresh 新增地图; then `rosbag play -d 3 -r 1.0 <all chunks in order> --topics /LIDAR/POINTS /IMU` (bags first, `--topics` last; never play the recorded odom; no `--clock`); press 保存地图 the moment it ends; a fresh 新增地图 for every attempt. The rebuilt frame starts at the bag-start pose, so trajectories recorded that night are **not** in the rebuilt map's frame.

### 3.5 Data-collection app `/teach` (ours)

Code: main repo `tools/s10_mapping_web/` (`teach_worker.py`, `teach_core.py`, `teach.html`, `teach.js`, `teach-worker.sh`, `server.py`, `TEACH_GUIDE_ZH.md`). On the AGX: `~/s10_mapping_web/`. **Records only; never commands the robot.**

| Piece | Interface |
|---|---|
| Web server | `server.py`, systemd user unit `s10-mapping-web.service`, bound to `10.21.33.102:8080`. `/` = login + entry; `/teach`; old 48-only pages redirect to `/teach` (their files are kept; `index_48_legacy.html`). Login cookie + CSRF token |
| Browser API | `GET /phone/teach/status`, `GET /phone/teach/sessions`, `GET /phone/teach/file?session=&name=`, `POST /phone/teach/submit` (JSON action, header `X-CSRF-Token`), `POST /phone/login` |
| Worker | `teach_worker.py` on `127.0.0.1:8091` (`/status`, `/sessions`, `/action`), ROS 1 node. Options: `--pose-topic` (default `/base_link/odom`; Odometry, PoseStamped or PoseWithCovarianceStamped), `--lidar-topic`, `--imu-topic`, `--data-dir`, `--min-free-gb-mapping 20`, `--stop-free-gb 3`, `--fake` (demo robot). Exit code 3 = ROS master restarted (the wrapper restarts it) |
| Actions | `session_new {label, map_name}`, `session_open {session_id}`, `record_start {mode: mapping|survey|path}`, `record_stop`, `mark {kind: WP|SWIN|SWOUT|WP_PASS|PATH_START|PATH_END|NOTE, wp_id, note}`, `redo`, `trail_clear` |
| Recording modes | `mapping`: lidar + IMU + pose + marks, 2 GB chunks, ~1.5 GB/min. `survey` and `path`: pose + IMU + marks (small) |
| Mark rule | 3 s still sample; pass = position spread ≤ 2 cm and heading ≤ 1°, ≥ 10 samples, no gap > 0.5 s. Marks are also published on `/teach/mark` (`std_msgs/String`) so they are inside the bags |
| Data | `<data dir>/sessions/<YYYYMMDD-HHMMSS-label>/`: `session.json`, `marks.jsonl` (rows: `seq, kind, wp_id, note, wall, mode, pose [x,y,z,yaw], result{passed, std_xy, yaw_std_deg, reasons…}`; a `VOID` row cancels `target`), `recordings.jsonl`, `<mode>_<stamp>_N.bag`, `<mode>_<stamp>.trail.csv` (`wall_time,x,y,z,yaw`, one row per 5 cm) |
| Storage | External exFAT SSD, fstab automount at `/mnt/s10ssd` (needs sudo to unmount: `sync; sudo umount /mnt/s10ssd; sudo udisksctl power-off -b /dev/sda`). `teach-worker.sh` picks `/mnt/s10ssd/s10_teach` if mounted, else `~/teach`; **the worker must be restarted after plugging or unplugging the SSD**. The page shows "未接硬盘！" in red when it writes to the eMMC |

Known defect: the trail and marks writers do not `fsync`; an AGX reset emptied `trail.csv` of session `v5_5` (the poses are still in the bags).

### 3.6 Planning core: `s10_auto_nav` (team code, pure Python)

Source of truth: main repo `src/s10_auto_nav/s10_auto_nav/`, branch `rl/maneuver-router`. The AGX runs a **copy** under `ros1_gateway/nav/s10_auto_nav/` made by `nav/sync_auto_nav.sh <checkout>` (records the commit in `nav/COMMIT`, currently `e1aaf73`; leaves out the ROS 2 node files; **wipes the copy with `rm -rf`**, so change the runner upstream and re-sync). Dependencies: numpy, scipy, yaml.

**Route file `route_v2`** (`route_v2.py`, schema string `s10_route_v2`):

```json
{"schema": "s10_route_v2", "map_id": "<map name>", "frame": "map", "z_reference": "ground",
 "waypoints": [{"id": "WP01", "position": [x, y, z_ground], "yaw": 0.26, "radius_xy": 0.2, "tol_z": 0.2,
                "confidence": "high|medium|low"}],
 "segments":  [{"id": "WP01-WP02", "from": "WP01", "to": "WP02", "gait": "flat|stairs", "speed_limit": 0.2,
                "allow_detour": true, "corridor_half_width": 0.8, "centerline": [[x, y, z], ...]}]}
```

Validation: exactly one segment between consecutive waypoints; centreline XY spacing ≤ 0.30 m; centreline ends within 0.5 m (xy) and 0.5 m (z) of their waypoints; gait ∈ {flat → 0x3002, stairs → 0x3003}.

| Module | Main API | Role |
|---|---|---|
| `route_v2.py` | `RouteV2.load/from_dict`, `RoutePath` (`point_at`, `tangent_yaw_at`, `project`, `gait_at`, `speed_limit_at`, `corridor_at`, `waypoint_s`), `RouteTracker` (windowed projection), `TerrainCrossCheck` | Route model, arc-length geometry |
| `route_follower.py` | `RouteFollowerCore(route, RouteFollowerConfig, controller=)`, `.step(t, (x,y,z,yaw), height13x9, mask, scan_ranges, scan_angles, pitch=, roll=) → FollowerOutput` | ROS-free follower. `FollowerOutput`: `command (vx,vy,wz)`, `status`, `reason`, `target_id`, `reached`, `s`, `d`, `speed_limit`, `finished`. Status: moving = `RUNNING`, `DETOUR`, `ASTAR`, `GATE`; stops = `BLOCKED`, `OFF_CORRIDOR`, `STALE_INPUT`, `DONE`. `BLOCKED no_route_progress` after 20 s without 0.10 m of progress |
| `route_planner.py` | `LocalGridBuilder`, `LocalGrid`, `RoutePlanner` | Local grid (0.1 m, 8 m, fuses the last N frames) from the height grid + scan; Frenet lateral-offset candidates (quintic) + bounded A*; never leaves the corridor. UNKNOWN cells block like OBSTACLE cells |
| `pure_pursuit.py` | `PurePursuitController(PursuitGains)` | Steering + heading-aware speed; pivots in place above `pivot_threshold` |
| `waypoints.py` | `Course.update(xyz)` | Ordered gate scoring: `radius_xy`, `tol_z` |
| `rl_nav/prepare.py` | CLI `rl_nav_prepare --route route_v2.json --terrain course_terrain.npz [--overrides …] [--method first|centre] --out dir` | Offline: writes `route_rl.json`, `maneuvers.json`, `map_surface.npz`, `prepare_report.json` |
| `rl_nav/route_prep.py` | `sanitize_route` (first version), `centre_route` | Moves waypoints/lines off hazards using a clearance field of the map |
| `rl_nav/maneuvers.py` | `Maneuver` (`id, s0, s1, s_first, s_last, policy, prior_point, prior_normal_yaw, skew_limit_deg, warnings…`), `annotate`, `route_hazards`, `save/load` (schema `s10_rl_maneuvers_v1`) | Where the stairs actor is needed, with the map's guess of the first edge |
| `rl_nav/capability.py` | `PolicyProfile` / `ActorProfile` (`policy_profile.json`) | What each actor can take, as numbers: step up/down, slope, side slope, min speed, entry skew, lateral error |
| `rl_nav/map_check.py` | `MapSurface` (ground, known mask, obstacle height raster), `unexplained`, `unexpected_ahead` | Tells a **new** obstacle from terrain the map already has |
| `rl_nav/map_planner.py` | `MapPlanner.plan` | Path on the map back onto the route (recovery) |
| `rl_nav/edge_tracker.py` | `detect_edge`, `EdgeTracker` | Step edges from the 13×9 grid |
| `rl_nav/route_runner.py` | `RouteRunner(path, maneuvers, RunnerParams, surface=, planner=).step(NavInput) → NavOutput` | **The decision layer**, below |

**Perception contract** (shared by simulation and robot): height grid 13×9 cells of 0.15 m, robot yaw frame, x −0.6…1.2 m, y −0.6…0.6 m, value = highest return, invalid when < 3 points or spread > 0.08 m (mixed levels are never averaged), unknown ≤ −1; conservative scan = 72 bins of 5°, range = observed extent shortened by returns in the body-height band (−0.25…0.75 m), NaN where nothing was seen; points in the robot yaw frame (gravity-levelled with roll/pitch).

**Route runner** (`NavInput`: `t, x, y, z, yaw, pitch, roll, yaw_rate, v_forward, grid, valid, follower output, s_gate, owner reported, points`; `NavOutput`: `command, owner, mode, reason, info`):

| Mode | Behaviour |
|---|---|
| WALK | The follower drives; forward command held ≥ `walk_floor` |
| TRACK | Follower refuses (BLOCKED / OFF_CORRIDOR for `track_after` = 3 s; at once when detours are forbidden): pure pursuit on the taught line at `walk_floor` speed |
| APPROACH → ALIGN → CLIMB | Before a zone: slow pursuit; turn in place to the edge heading (strafing against lateral error); stairs actor with heading corrections ≤ 12°, slower near waypoints and drops; hand back only when slower than `settle_v` |
| DESCEND | Step off a ledge straight, retry on stall |
| RECOVER / BACKUP / DETOUR / WAIT | Recovery on a **measured** deviation only: off route > 0.8 m, new obstacle in the corridor, stall; needs the map surface |
| HOLD / DONE | Stopped for an operator / finished |

Design rule (user decision, see the memory note "first version is nominal"): the nominal controller is the first simulation version, unchanged; robustness is added only as recovery triggered by measured deviation, and smoothness is always reported against the first version over many seeds.

**Important consequences for the real robot:**
- Without `map_surface.npz` the runner has **no obstacle check of its own** (`_obstacle` returns None). A blocked follower leads to TRACK after 3 s, which drives on at `walk_floor`. In `config/nav.yaml` `walk_floor` is 0.0, so TRACK currently stands still but still outputs lateral/yaw commands. Required before first motion: TRACK fully zero and recoverable (plan, Stage 2). A map surface has to be built from a saved x_nav `global_map.pcd` (❌ no tool yet).
- Without `maneuvers.json` the runner never asks for the stairs gait. In the kinematic test a `stairs` segment without a zone ended in HOLD (`centerline_blocked_detour_forbidden`); `teach_to_route.py` therefore writes a zone for every SWIN…SWOUT pair and marks only those segments `stairs`.
- A mapping walk is **not** a route (out-and-back passes overlap). Routes come from a taught path recorded in localisation mode.

### 3.7 ROS 1 navigation node (ours): `nav/`

| File | API |
|---|---|
| `nav_core.py` (ROS-free) | `FrameTransform.load(json)` (`route_from_xnav`, identity by default) · `Perception(base_from_lidar, max_points).observe(cloud_xyz, pose6) → {grid, valid, scan, points_yaw}` · `decode_pointcloud2(msg)` · `conservative_scan(points)` · `flat_observation(body_z_offset)` (tests) · `RouteBundle.from_dir(dir)` (looks for `route_rl.json` or `route_v2.json`, optional `maneuvers.json`, `map_surface.npz`, `policy_profile.json`) · `NavCore(bundle, cfg).step(t, pose8, obs, gait_reported, obs_age) → StepResult(command, owner, gait_request, mode, reason, status, reached, finished)` · `OWNER_TO_GAIT = {official: flat, stairs_stable: stairs}` |
| `s10_rl_nav_ros1.py` | rospy node `rl_nav`. CLI: `--config config/nav.yaml --route-dir DIR [--transform file] [--shadow] [--autostart]` |

| Topic | Type | Direction |
|---|---|---|
| `pose_topic` (`/base_link/odom`) | Odometry / PoseStamped / PoseWithCovarianceStamped (twist finite-differenced when absent) | in |
| `/LIDAR/POINTS` | PointCloud2 | in (perception runs in the cloud callback) |
| `/s10_control/gait` | String `flat | stairs | switching | none` | in → owner reported |
| `/s10_control/state` | String JSON | in: a fault or latched stop holds the runner |
| `/rl_nav/cmd` | String `start | pause | reset` | in (waits for `start` unless `--autostart`) |
| `/rl_nav/cmd_vel` | Twist (`linear.x`, `linear.y`, `angular.z`) | out, 20 Hz, not in `--shadow` |
| `/rl_nav/gait_request` | String `flat | stairs` | out every tick (known defect: also in `--shadow`) |
| `/rl_nav/status` | String JSON: `mode, reason, maneuver, cmd, owner_requested, gait_request, gait_reported, target, s, d, follower, follower_reason, reached, total, progress, obs_fresh, pose_age, obs_age, obs_ms …` | out |
| `/nav/progress`, `/nav/finished` | Float32, Bool (latched) | out |

`config/nav.yaml`: topics, `control_rate 20`, `pose_timeout 0.3`, `obs_timeout 0.5`, `max_points`, `body_z_offset` (❓ placeholder), `lidar_extrinsics` (identity ✅), `route_from_xnav_file`, follower gains, and `runner_params` for the **vendor gaits** (walk 0.30, climb 0.15, approach/detour/recover 0.20 m/s; the simulation's 0.6 / 0.5 / 0.3 are the RL policies' numbers). Unknown top-level keys are silently ignored (known gap).

Route tools: `tools/teach_to_route.py <session> --map-id NAME --out DIR [--body-z-offset] [--flat-speed 0.20] [--stairs-speed 0.15]` → `route_v2.json` (waypoints = last passing mark per WP; centreline = taught trail between them; `stairs` where a SWIN…SWOUT pair overlaps), `maneuvers.json` (one zone per pair, no edge prior), `report.json` (skipped marks, re-test differences). `--straight X Y Z YAW LENGTH` makes a two-waypoint test route. 🧪 synthetic sessions only.

### 3.8 Motion control node (ours): `src/s10_ros1_control` (C++, roscpp + rclcpp in one process)

The **only** program of ours allowed to publish to the robot. `scripts/start_control.sh` = dry run; `--enable-motion` = armed (creates the three ROS 2 publishers after a 3 s discovery window).

| ROS 1 in | Effect |
|---|---|
| velocity sources (`cmd_sources`: `rl_nav: /rl_nav/cmd_vel`, `x_nav: /cmd_vel`) | Only the **active** source (`cmd_source`, default `rl_nav`) is forwarded; others are counted and dropped. `/web_cmd` `source <name>` switches |
| `/rl_nav/gait_request` (`flat` / `stairs`) | Gait switch only in RL control: zero velocity → measured still ≥ 1 s → `/GAIT` → wait for `/MOTION_INFO` to confirm (10 s, else latched stop) |
| `/web_cmd` (String) | `cmd4` stand (state 1 → settle 2 s → 17 → nav gait, each step confirmed), `cmd3` lie down (zero → still ≥ 1 s → 4), `Nav stop` (latch zero), `Nav continue`, `cmd1` navigation gait, `cmd2` ignored, `source <name>` |

| ROS 1 out | Content |
|---|---|
| `/s10_control/state` (5 Hz JSON) | `enable_motion, publishers_created, fault, latched_stop, sequence, moving, cmd_source, gait, gait_request, feedback{fresh, age_s, state, gait, vel, height}, cmd_vel{age_s, in, out, received, rejected, clamped, ignored{…}}, nav_cmd_sent, nav_cmd_publishers, nav_cmd_foreign_1s, nav_cmd_foreign_total, limits, last_event` |
| `/s10_control/gait` (10 Hz) | `flat | stairs | switching | none` |
| `logs/control-events-*.jsonl` | every command sent or "would send", sequences, faults |

Rules: velocity only in RL (17) + navigation gait, fresh `/MOTION_INFO` (< 0.5 s), fresh command (< 0.5 s), no latch, no fault, no sequence running; clamped to `limits` (0.30 / 0.10 / 0.50; hard caps 1.0 / 0.5 / 1.0); fixed 10 Hz; after any stop zeros for 1 s, then silence. Command stamps follow the robot clock. `exclusive_mode: messages`: another publisher that actually **sends** `/NAV_CMD` latches a fault (restart needed); idle native publishers are tolerated. `publishers` mode refuses as soon as another publisher exists.

Known gaps before arming (from the plan review): no `--config` option in the start scripts, so per-stage speed caps cannot be loaded; while armed, the open x_nav web page can publish `/web_cmd` (stand / lie, `Nav continue`, `source x_nav`) → use a private `web_cmd_topic` and `rl_nav` as the only source.

### 3.9 Policies (background; not used on the real dog now)

Full table with evidence: main repo `docs/POLICIES_AND_APPS_ZH.md`. Training and the MuJoCo harness: private repo `s10-rl-sprint`.

| Policy | I/O | Role | State |
|---|---|---|---|
| Official 57D | 57 → 16 | General walking (SDK default) | ✅ on robots |
| speedturn2000, HIM 1500 | 57 → 16, 342 → 16 | Trials on dog 048 | ✅ tried; HIM failed the first stair (wheel speed protection) |
| **J3100** | 59 → 16, free-running phase, cycle 0.6 s | Walk actor, `official` slot of `rl_nav` | 🧪 MuJoCo only |
| **1150** | 59 → 16, command-gated phase 1.5 Hz | Stairs / slopes, `stairs_stable` slot | 🧪 MuJoCo only; wheel speed exceeds the 30 rad/s diagnostic line on stairs |
| Gate16 v1.5, stairs_stable (model_599) | 174 → 16, 57 → 16 | August contest | history |
| Native gaits 0x3002 / 0x3003 | – | **What the real dog uses now** | 📝 never commanded by us |

Actor contract (from `s10-rl-sprint/scripts/tools/sim2sim_mujoco.py`, `phase_policies.py`): 50 Hz. Observation 57 = base angular velocity × 0.25 (3), projected gravity (3), command `vx vy wz` (3), joint position − default (16, wheel positions zeroed), joint velocity × 0.05 (16), last action (16); the 59-D actors append `sin φ, cos φ`. Action 16 in policy order (12 leg joints FL, FR, HL, HR × hipx, hipy, knee, then 4 wheels): target = action × `[0.125, 0.25, 0.25]×4 + [5.0]×4` + default `[0, −0.3, 0.6]×2 + [0, 0.3, −0.6]×2 + [0]×4`; legs are position targets (kp 80, kd 2, ±50 N·m), wheels velocity targets (kd 0.6, ±14 N·m). Switching actor clears the previous action.

Measured capability (MuJoCo, `policy_profile.json`): J3100 stalls on ≥ 4 cm steps or 10° slopes below ~0.5 m/s, slides on side slopes (0.45 m per 5 m at 6°); 1150 climbs 12–18 cm steps (21 cm falls) and 20° slopes at 0.3 m/s, barely turns on stairs. Full course with both: 30/30 waypoints in 713 s (first version). GPU-server evaluation over many seeds: first version 4/24 complete, `rl_nav` v3 19/32 (one commit earlier 23/32); section results barely depend on the seed; judge on ≥ 32 seeds.

SDK side (path A): `integration/joint_command_owner.hpp` is the single gate for `/JOINTS_CMD`: subscribes `/strategy/joint_owner` (`official | stairs_stable | stop | gate16… | climb`), `/strategy/climb_joints` (16 targets), `/perception/heightmap`; publishes `/joints/owner`. `official → stairs_stable` is an atomic moving hand-over; every other change passes through SafeHold (0.25 s, legs held, wheels damped), so never hand back while sliding. Robot diagnostic stop lines: leg / wheel speed 25.76 / 30 rad/s, torque 45 / 12 N·m. To run `rl_nav` with the RL actors on a robot, the SDK runner still needs both slots to accept 59-D observations with their phase clocks, and 1150's wheel speed problem must be solved.

Older alternative for path B inside ROS 2: `native_transfer/` (`NativeGaitRouter`: zero → measured still → one gait request → fresh acknowledgement → velocity; 78 local tests; read-only observation on 106 passed; never commanded the robot). `s10_ros1_control` re-implements those rules in the ROS 1 stack.

---

## 4. Operating the stack

All from the Mac, in `ros1_gateway/` (the scripts SSH to the AGX and, through it, to 106):

| Command | Does |
|---|---|
| `scripts/robot_session.sh keys` | One time per Mac: installs this Mac's public key on 106, 103 and the AGX admin account (three password prompts typed by the operator; passwords are never stored) |
| `scripts/robot_session.sh up` | Prints the dog's serial; deploys + starts the 106 tap; on the AGX: x_nav container → gateway → control (**dry run**) → teach worker; sets `run/session_active`; health check |
| `scripts/robot_session.sh status` | Identity, tap, gateway health |
| `scripts/robot_session.sh down [--agx]` | Clears `session_active`, stops the tap and deletes `/home/user/ros1_gateway_tap` on 106 (verifies nothing is left) |
| `scripts/robot_session.sh unkeys` | Removes our Mac and AGX keys from 106 / 103 (restores `authorized_keys` exactly) |
| `scripts/robot_session.sh autostart` | One time: AGX key for the 106 tap + systemd user unit `s10-stack.service` → `scripts/agx_boot.sh` at every boot. The 106 tap is started at boot **only while `session_active` exists** |
| `scripts/deploy_nav.sh [--no-test]` | rsync to the AGX (no `--delete`: the AGX also holds `src/ros1_bridge`), rebuild the control node, re-run the mock and nav tests there on private ports |
| `scripts/mac_tunnel.sh &` | Self-reconnecting tunnel: `localhost:18080` app, `localhost:18000` x_nav |
| On the AGX: `start_gateway.sh`, `stop_gateway.sh`, `start_control.sh [--enable-motion]`, `stop_control.sh`, `start_nav.sh --route-dir DIR [--shadow] [--autostart]`, `stop_nav.sh`, `health_check.sh [--hz N]`, `agx_build.sh` | pid files in `run/`, logs in `logs/` |

URLs on the robot network: app `http://10.21.33.102:8080/`, x_nav `http://10.21.33.102:8000`.

AGX layout: `~/ros1_gateway/` (this tree + `ros1/` ROS-O bundle + `ws/` colcon workspace + `run/`, `logs/`), `~/s10_mapping_web/` (web app), `/opt/data/{compose,nav_map,config}` (x_nav), `/mnt/s10ssd/s10_teach/sessions/` (recordings). System changes made on the AGX (it is ours): Docker + compose, `golai` in the `docker` group, linuxptp units, fstab entry for the SSD, the user unit above. `scripts/agx_power_log.py` (fsync'ed rail voltage / current / temperature logger for the reset hunt) is written and called by `agx_boot.sh`, but **was not confirmed running**.

Recordings on the SSD (`s10_teach/sessions/`), both cut short by AGX resets, last chunk repaired as `*.recovered.bag` (originals kept): `20260920-002445-v4_room` (indoor room, ~6 min, 8.5 GB, `trail.csv` complete) and `20260920-013314-v5_5` (outdoor with stairs, ~11 min, 14 GB, `trail.csv` empty). Topics: `/LIDAR/POINTS`, `/IMU`, `/base_link/odom`, `/teach/mark`. They are raw sensor bags, **not maps**; no marks, no taught path.

---

## 5. Tests

| Test | Command | Proves |
|---|---|---|
| Control node vs mock robot | `docker run --rm --network none -e REBUILD=1 -v "$PWD":/src:ro -v /tmp/out:/out s10-ros1-gateway-test bash /src/tests/control/run_control_test.sh enabled` (and `dry_run`) | 35 + 7 checks: stand order and confirmations, clamps, 10 Hz, timeouts, Nav stop / continue, lie down only when still, source selection, gait switch only at standstill, silent native publisher tolerated, sending foreign publisher → fault, dry run sends nothing |
| Navigation core, kinematic | `python3 tests/nav/test_nav_core.py --stairs-zones [--first N --last M]` | Full v3 draft route 30/30 on synthetic flat ground, stairs requested in every zone. Plumbing only |
| ROS 1 node end to end | `docker run --rm --network none -v "$PWD":/src:ro -v /tmp/out:/out s10-ros1-nav-test bash /src/tests/nav/run_nav_ros1_test.sh` | PointCloud2 → grid → runner → `cmd_vel` → fake robot reaches WP02 (0.16 m), waits for `start` |
| Gateway end to end / offline conversion | `tests/e2e/run_e2e.sh`, `tools/audit_offline.sh` | Synthetic ROS 2 → ROS 1 identical |
| Web app | `python3 test_teach.py`, `test_teach_ros.py` in `tools/s10_mapping_web/` | 43 checks + real roscore/rosbag |
| Team nav code | main repo: `PYTHONPATH=.:src/s10_auto_nav:src/s10_perception python -m pytest -q src/s10_auto_nav/test sim_full_course/tests tests_real` | |

Images: `docker/Dockerfile.builder` (ROS 2 Jazzy + ROS-O + `ros1_bridge` from source + our packages → `s10-ros1-gateway-test`), `docker/Dockerfile.navtest` (+ scipy, yaml → `s10-ros1-nav-test`; no matplotlib inside). The same tests run on the AGX through `deploy_nav.sh` (private ROS master ports 11399 / 11312, domain 78).

---

## 6. What is not known yet (measure, do not assume)

| Unknown | How to find out |
|---|---|
| Does `/NAV_CMD` need ASDU use mode 1? Do the remote sticks still work in mode 1? | 0.10 m/s probe on site (plan Stage 5) |
| Does the robot stop by itself when `/NAV_CMD` stops (AGX reset mid-motion)? | Operator runs `kill -9` on the control node at 0.10 m/s, yaw first. If it does not stop: halt and ask the vendor |
| Standing body height; is x_nav's Odometry twist filled; pose jitter vs the 0.3 s timeout | From the recorded bags (plan Stage 0) and live |
| Can x_nav save a map on the ARM beta image; where exactly; is `/base_link/odom` the same in localisation mode | Map 2–3 min, press save at once, look in `/opt/data/nav_map/<name>` |
| Can `x_slam` rebuild a map from a replayed bag | Time-boxed trial on the AGX (plan Stage 3R) |
| Why the AGX resets (`SYS_RESET_N`) | Power logger + supply/cabling check; is it load-related (both mapping losses happened while walking, recording to a bus-powered USB SSD) |
| What 103's remote-control console code (folder `golai` on 103) does | Read it, read-only; the remote ↔ autonomous switch is meant to hook in there |
| False-alarm rate of the follower's local grid on real lidar | Offline shadow replay of `v4_room` (plan Stage 1) |

---

## 7. Gotchas already paid for

- x_nav's `roscore` owns 11311 after an AGX reboot: **container first, gateway second** (`agx_boot.sh` does it).
- A process started over SSH without `setsid`/`nohup` dies with the session (the teach worker did).
- `rsync --delete` from the Mac deleted `src/ros1_bridge` on the AGX once: never `--delete` into `src/`.
- `rosbag play`: bag files first, `--topics` last, otherwise the bag names are swallowed and it exits at once.
- `ros2 bag record` ignores SIGINT when backgrounded in a non-interactive shell; run it in the foreground with `timeout -s INT`.
- Fast DDS default shared memory is too small for clouds in tests: use the 106-like profile in `tests/e2e/`.
- The first vendor image was x86-only; NULL values in `x_nav.yaml` crash its controller (ROS 1 parameters cannot hold None).
- exFAT + power cut: closed 2 GB bag chunks survive, the open one needs `rosbag reindex` (on a copy), unflushed CSVs are lost.
- macOS may offer to "repair" the SSD: copy the data first.
- The old phone pages (`/field`, `/native-nav`, …) are tied to dog 048 (their SSH key is rejected by 050). `/field`'s "load map" would change the shared robot's active map. Only `/teach` is for 050.
- Tests that passed offline hide configuration mistakes that only matter on real data (example: `walk_floor: 0.0` makes TRACK a standstill).

---

## 8. Next steps, in order (details and pass criteria: [NAV_TEST_PLAN_20260920.md](NAV_TEST_PLAN_20260920.md))

| Stage | Robot? | Content |
|---|---|---|
| 0 | no | Copy both sessions from the SSD to the Mac with checksums; `tools/bag_inspect.py` ❌: flatness, standing height, twist verdict, gaps, first standstill |
| 1 | no | `tools/bag_to_route.py` ❌ + `tools/replay_shadow.py` ❌: replay `v4_room` through Perception + NavCore in shadow; binary plumbing gates, false alarms as a triage report. Proves nothing about closed-loop behaviour |
| 2 | no | Pre-motion fixes: `--config` for the start scripts + `control_zero / probe / flat.yaml` (0 / 0.10 / 0.20 m/s), private `web_cmd_topic`, `--shadow` without gait requests, tick exception guard, zero-twist fallback, pose-jump hold, TRACK zero and recoverable (upstream, then sync), `fsync` in the teach worker, `straight_from_live_pose.py`, operator checklist |
| 3 | yes, no autonomous motion | Save a map in the first 10 minutes; localise; teach a short flat route (3–5 WPs, 6–10 m); drive it twice by remote with the node in shadow and control in dry run |
| 4 | yes, limit 0 | First commands to the robot: stand, gait switch, lie; prove the remote takes the robot back, also with our node gone |
| 5 | yes, 0.10 m/s | 0.6 m probe; ASDU-mode answer; software stops; remote override; `kill -9` command-loss test |
| 6 | yes, ≤ 0.20 m/s | First closed loop: straight 2 m, then the taught route. No stairs until the AGX resets are explained |

Also open: the app's navigation page (start / pause / status) ❌; a tool that builds `map_surface.npz` from a saved x_nav map ❌; stairs gait inside a route on the robot; pushing `ros1_gateway` changes to GitHub on a `ros1/<topic>` branch by PR (the owner confirms every merge).

---

## 9. Where things live

| Place | Content |
|---|---|
| GitHub `bowenwan6/goai26-s10-racing`, `main` | Navigation stack (`src/s10_auto_nav`), SDK integration (`integration/`), web app (`tools/s10_mapping_web`), `ros1_gateway/` (this tree), simulation, maps (Git LFS), docs. Entry docs: `docs/POLICIES_AND_APPS_ZH.md`, `docs/REPO_GUIDE_ZH.md`. Branch rules: `nav/*`, `rl/*`, `codex/*`, `ros1/*`; everything reaches `main` by PR |
| GitHub `bowenwan6/s10-rl-sprint` (private) | Isaac Lab training, MuJoCo full-course harness (`scripts/tools/route_follow_mujoco.py`, `route_rl_real_stack.py`), J3100 / 1150. Its tools contain absolute paths of the owner's Mac |
| Owner's Mac, `~/Documents/ChatGPT/GOAI/` | Worktrees: `wt-rl-nav` (`rl/maneuver-router`), `s10-field-assistant` (`codex/field-assistant`), `wt-repo-org`, `wt-nav-planner`, `wt-nav-sim`, `wt-wp-match`; `s10-real-readiness/ros1_gateway` = the working copy of this tree (**not a git checkout**; synced to the repo by PR) |
| Only local / not in git | Raw bags and MCAPs, the x_nav licence file, the reproduction pack `S10_Nav_FullV3_Repro_20260919.zip`, SDK copies on the AGX |
| This directory | `README_ZH.md` (gateway, control, x_nav deployment, acceptance record), `docs/NAV_TEST_PLAN_20260920.md` (+ `.raw.json`), this file |
| Vendor documents | Robot developer guide V1.0.1 (ASDU) and the S10 SDK guide (ROS 2 topics); x_nav operator manual. Ask the owner |

**First day for a new person:** get access from the owner → read this file, `README_ZH.md`, `docs/POLICIES_AND_APPS_ZH.md` and the test plan → build the two Docker images and run the three offline tests of section 5 → run `python3 tests/nav/test_nav_core.py --stairs-zones` → only then touch the robot, starting with `robot_session.sh status`.
