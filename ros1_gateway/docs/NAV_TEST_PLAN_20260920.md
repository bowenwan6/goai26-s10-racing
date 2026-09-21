# S10 自动导航测试计划（用录制数据 + 上机分级）

2026-09-20。由一次多代理调查生成：3 个只读调查（x_nav 重建/定位、我们的导航栈回放可行性、安全与运维约束）→ 3 份独立方案 → 3 个角度的对抗审查（41 条问题）→ 合成。调查期间 AGX 不可达、SSD 未接入，所以凡是“要读 bag 才知道”的项仍是未知。阶段正文保留英文原文。

## 结论

结论先说：两个录制都不是“地图”。它们只是原始传感器 bag（/LIDAR/POINTS、/IMU，加上建图时 x_nav 的实时位姿）。两次都在按“保存地图”之前 AGX 复位了，/opt/data/nav_map 里没有任何地图，也没有 WP 标记和示教路径。今天在 x_nav 里选不了任何一个。

所以“用录好的地图测自动导航”要拆成两条并行的线：

1. 离线线（只用 Mac，零风险）：用 v4_room。
   - 把 bag 回放进我们的导航栈，shadow 模式，Docker，--network none。路线从 bag 自己的 /base_link/odom 生成，只用于回放。
   - 能证明：坐标系和 yaw 符号、路线投影、高度栅格和 scan 是否正常、真实雷达上的误报率、CPU 开销、站立高度 H、x_nav 的 twist 有没有填。
   - 不能证明：导航是否 work。bag 里的狗不会响应我们的指令。
   - v4_room 通过后再跑 v5_5，只统计误报，不作为门槛。

2. 上机线：默认不用任何录制，现场重新建一张小图。
   - 在平坦、无人的室内（最好就是 v4 那个房间）用遥控器走 2-3 分钟，立刻按“保存地图”。
   - 这是拿到地图最快、最确定的办法，大约 20 分钟，不需要新代码。
   - 它同时回答 x_nav 的四个未知：地图存在哪里、ARM beta 镜像能不能保存、定位模式下 /base_link/odom 的表现、twist 是否填充。
   - 如果你现在还在狗旁边，今天就可以做：先确认 agx_power_log.py 在跑，然后 新增地图 -> 走 2-3 分钟 -> 保存地图 -> ls /opt/data/nav_map/<name> -> 选择地图 -> 设置位姿 -> 看 /base_link/odom 的频率和 twist。做完把 SSD 插到 Mac 上。

可选实验：在我们的 AGX 上把 v4_room 的 bag 回放进 x_slam 重建并保存地图。
   - 最多试 2 次，约 1.5-2 小时，失败就放弃。
   - 这是任何一个录制真正变成地图的唯一途径。成功后，以后建图途中再复位也不会丢数据。
   - 第一次运动不依赖它。

第一次运动的顺序，所有运动指令都由操作员本人在现场发出，自动化工具和脚本不会自行发出任何运动指令：
   1. 遥控驾驶 + control DRY RUN 的 shadow。
   2. armed 零速度：站立、步态切换、遥控器接管。
   3. 0.10 m/s 短探测。这一步回答 ASDU 模式的问题。
   4. 你手动 kill -9 的指令丢失测试。
   5. 直线 2 m。
   6. 定位模式下示教的短平地路线。
   - AGX 复位原因查明之前不上楼梯。v5_5 暂不用于上机。

上机前必须先修的问题，都在离线做完并通过 mock 测试：
   - control 的限速和 web_cmd：
     - 每个阶段用单独的 control 配置，限速 0 / 0.10 / 0.20。现在所有方案里硬限幅都还是 0.30/0.10/0.50，启动脚本也不能换配置。
     - armed 时把 web_cmd 换成私有 topic，并去掉 x_nav 指令源。否则 x_nav 网页（密码 123，所有网卡都在监听）能让狗站立或趴下、解除 Nav stop、切换速度源。
   - nav 节点：
     - --shadow 现在仍然会发 gait_request，要改成不发。
     - tick() 加异常保护。
     - twist 为零时用位姿差分兜底。
     - 加位姿跳变保护。
     - HOLD 之后可以恢复。
   - TRACK 的行为：
     - 保持零速，并且 vy/wz 也清零。
     - follower 拒绝后可以恢复，不再锁死成 HOLD。
     - 原因：没有 map_surface 时 runner 自己不做障碍检查，而 --straight 路线一被挡就直接进 TRACK，给 TRACK 正速度会让狗走向挡路的东西。
   - 采集助手：trail/marks 写入要加 fsync。v5_5 的 trail.csv 就是因为没有 fsync 才丢的。
   - 恢复清单：要有 AGX 死机时的分支。从现场笔记本直接 ssh 到 106 去掉 tap，并在笔记本上跑 asdu_mode.py。

部署方式：现场走狗的局域网，设置 S10_AGX_SSH 为直连主机，不走 Tailscale 中继。在没有运动的时段一次性部署，之后重启 dry-run control，并从 /s10_control/state 确认 enable_motion:false。

## 离线阶段用哪个录制

The offline stage uses 20260920-002445-v4_room first.

Reasons for v4_room:
- It is the shortest recording: about 6 min in 4 chunks, 8.5 GB.
- It is the only session whose trail.csv survived. The last buffered block is probably missing, so compare it with odom only over the overlapping time span.
- It is presumably an indoor flat room, which makes replay events easy to classify.

"Flat room" is inferred only from the session name. Stage 0 has to confirm it from the bag (|z| range < 0.15 m, |pitch| < 8 deg). A BLOCKED event near a wall, near furniture or near the operator counts as correct, not as a false alarm.

If v4_room turns out not to be flat or clear, it still serves for the plumbing checks. False-alarm interpretation then becomes weaker.

20260920-013314-v5_5 is the second offline set:
- It is about 11 min, 7 chunks and 14 GB, probably outdoors. Its trail.csv is empty, so odom comes from the bags.
- It is run only after the tools work on v4_room.
- It is used only for false-alarm statistics, binned by recorded speed and yaw rate.
- It has no pass threshold and never gates anything.

## 上机阶段用哪个

Neither recording can be used as it stands, because no saved map exists.

The default for the on-robot test is a fresh map of a flat, clear indoor room with no public access, ideally the v4_room room. The walk is 2-3 minutes by remote, with 保存地图 pressed at once. Waypoints and the taught path are then recorded in localisation mode on that saved map.

The first motion steps (stand, the 0.10 m/s probe, straight 2 m) need only a live x_nav pose and a --straight route. They need no saved map.

A rebuilt v4_room map (bag replay into x_slam on our AGX) replaces the fresh map only if all of these hold:
- The rebuild succeeds within 2 attempts.
- Dog 050 is physically in that room.
- The room is unchanged since the recording.
- A live 设置位姿 relocalisation with the real dog shows the red live cloud overlapping the green map. This is the real acceptance test. Trajectory RMS against the old odom is only a proxy.

Facts that decide the choice:
- Where the dog is now, and whether the v4 room is available. If the answer is no, use any flat room with at least 5 m of clear straight space and a fresh map.
- Whether the closed x_slam binary accepts replayed old-stamped data. Only a trial can tell.
- Whether v4_room is really flat. Stage 0 measures this from the bag.

v5_5 is never used on the robot while the AGX resets are unexplained and its content (stairs, outdoor) is unknown.

The old v4_room trail.csv must never be used as a route in a rebuilt map. The rebuilt frame starts at the bag-start pose, not where 新增地图 was pressed that night.

## 为什么这样定

- Base plan: Plan 2, the shortest credible chain. Two of the three critiques chose it. Its mechanisms match the code:
  - --flat-speed really caps the follower.
  - Runner changes go upstream and come back via nav/sync_auto_nav.sh.
  - The rebuild is a time-boxed gamble with a live re-map fallback.
  - First motion does not depend on the bags.
  Grafted from Plan 1: the offline harness design. Grafted from Plan 3: safety detail. Dropped from Plan 3: the gate.sh mechanism and the pass/fail soak gate.
- Both blockers from the safety critique are fixed with existing config keys, and no vendor service is touched. I checked that control.yaml already has limits.*, start_latched, web_cmd_topic and cmd_sources, and that control.cpp:137-150 parses an arbitrary cmd_sources map. The only missing piece is a --config option in start_control.sh and start_nav.sh, which hard-code their configs at start_control.sh:40 and start_nav.sh:30.
- Probe geometry was re-simulated for this plan (kinematic NavCore, scratchpad/finalchk):
  - --straight ... 0.6 --radius 0.10 --flat-speed 0.10 travels 0.50 m in 5.0 s at vmax 0.10.
  - 0.8 m gives 0.70 m in 7.0 s, which is too close to the 8 s cap.
  So the probe is LENGTH 0.6, radius 0.10, with expected travel of LENGTH minus radius. The original 0.3 m probe ends after 0.10 m in 1 s and cannot meet any of its criteria.
- TRACK stays at speed 0 for the first robot runs.
  - Without map_surface.npz, the runner's own obstacle check returns None (route_runner.py:338-341).
  - --straight routes have allow_detour=False, so a blocked centreline enters TRACK immediately.
  - A positive TRACK speed would therefore walk the dog into whatever blocks the route.
  - The refusal is made recoverable instead, and vy/wz are zeroed too.
  - Push-through is revisited only after a map_surface is built from a saved global_map.pcd.
- Replay thresholds are split in two, following the feasibility critique and the 'first version is nominal' memory.
  - Binary plumbing checks gate: node survival, frame and yaw sign, s against s_truth, H, the twist verdict, and CPU cost.
  - False-alarm percentages are a triage report with snapshots, time-boxed to one day. They gate nothing numerically.
- The AGX resets are treated as a condition to instrument and live with on flat ground.
  - The power logger comes first.
  - Every on-site block must leave a persisted artefact within the first 10 minutes: a saved map backed up, or an fsynced trail.
  - The kill -9 command-loss result, not a clean soak, decides whether a reset mid-motion is tolerable.
  - Stairs stay blocked until the reset cause is found.
- All three plans share one defect: the rebuild command put the bags after --topics, so rosbag exits immediately. The correct form is `rosbag play <bags...> --topics /LIDAR/POINTS /IMU`. It is scripted and dry-tested on the Mac before the on-site attempt.
- User preference 'work fast: implement, then verify':
  - All desk work (Stages 0-2) is built in one batch and verified once against the existing suites: 36/36, 30/30 and the ROS e2e test.
  - Ordering is kept only where there is a safety reason: shadow before armed, zero velocity before the probe, the probe and kill -9 before routes, flat before stairs.

## 阶段

### Stage 0 - Secure the data and triage the bags (Mac only)

- **需要机器人：** No robot and no AGX. The user plugs the SSD into the Mac, where it mounts at /Volumes/PortableSSD.
- **用时：** Build takes about 3 h. Copy and run take about 30-40 min.

Make a second copy of the only real-lidar data, which sits on exFAT after two power cuts. Replace guesses with numbers.

PROVES:
- what each recording contains, and whether v4_room is really flat
- standing height H in the remote-driven gait
- whether the Odometry twist is filled
- pose jitter against pose_timeout 0.3 s
- loop-closure jumps and chunk gaps
- whether the dog was still at bag start, and where the first standstill is
- leg or wheel self-returns

CANNOT PROVE: anything about navigation.

GATE: the SSD is mounted on the Mac. Do not run fsck or a macOS repair before copying.

**步骤**

- Copy both sessions to ~/s10_data/sessions/ with rsync -a. About 22.5 GB; the Mac has 173 GB free. Then chmod -R a-w the copies and write SHA256SUMS per session with shasum -a 256. Eject the SSD. All later work mounts the copy into Docker with -v ~/s10_data/sessions:/data:ro --network none. rosbag reindex is run only on a copy, never on the original.
- Inside Docker image s10-ros1-nav-test, run tools/bag_inspect.py on each session. It writes odom.npz, report.json and the per-plot npz data. The image has no matplotlib, PIL or cv2, so PNGs are rendered on the Mac host by tools/plot_bag_report.py (matplotlib 3.8.4).
- Per chunk: start time, end time, message counts and rates, gaps between consecutive chunks, and truncation of each *.recovered.bag.
- Twist verdict: compare twist.linear.x and twist.angular.z with finite differences of the pose. The verdict is filled, zero or wrong-frame.
- Pose timing: odom inter-arrival p50/p99/max, and the header lag between each cloud and its nearest odom.
- Jump list: consecutive 10 Hz samples with |dxy| > 0.25 m, |dz| > 0.15 m or |dyaw| > 20 deg.
- Standing height H: fit the ground plane with the function from tools/check_ground_plane.py, only while recorded speed is above 0.1 m/s and only on cells within 2 m that fit the dominant plane. Report the median, the spread and the levelled-ground tilt.
- Start conditions: IMU gyro and accel variance in the first 3 s, and the time offset of the first standstill lasting at least 2 s. That offset becomes the -s value for the rebuild. Also report the offset of the first pose from the origin.
- Motion profile: pauses longer than 8 s and longer than 20 s, reverse and strafe intervals, and the speed and yaw-rate envelope.
- z range and pitch range as the flat-room check. Stairs and slope candidates for v5_5.
- Body-frame self-return histogram within 1.2 m.
- v4_room only: compare trail.csv with odom over the overlapping time span.
- Send traj.png and z_profile.png of both sessions to the user. Ask where each walk was, and where in the room v4_room starts.

**要新写的**

- tools/bag_inspect.py
- tools/plot_bag_report.py
- an importable plane-fit function in tools/check_ground_plane.py
- optional tools/copy_session.sh

**通过标准**

- Checksums of the copies equal the source. Every chunk opens. The recovered chunks show the known counts: v4_room 777/15524/777 and v5_5 515/10294/515.
- Inter-chunk gaps are under 0.5 s. Larger gaps are listed and become forced route splits.
- A flat-room verdict for v4_room: |z| range < 0.15 m and |pitch| < 8 deg. If it fails, v4_room is kept for plumbing only.
- H is measured with a spread of at most 0.03 m. The verdict is either H >= 0.35 m or H < 0.35 m. If H < 0.35 m, the scan-band lower bound must become a config value, set in Stage 2.
- The twist verdict is recorded. If it is zero, the finite-difference fallback becomes mandatory.
- Odom inter-arrival p99 < 0.2 s and max < 0.3 s. If not, pose_timeout is revisited.
- The -s offset and the start-stillness verdict are recorded for the rebuild.

**风险**

- exFAT damage may make a chunk unreadable. Use the readable chunks and start the route after the bad one.
- A lying start could be misread as the standing height, so the fit uses moving samples only.
- H here is the remote-driven gait. Navigation gait 0x3002 may differ. It is re-measured in Stage 4.

### Stage 1 - Offline shadow replay on v4_room, then v5_5 (Mac, Docker --network none)

- **需要机器人：** No. Bags are never played into the AGX roscore on :11311.
- **用时：** Build takes 1.5 days. v4_room runs about 5 min offline plus 7 min e2e. v5_5 runs about 10 min. Triage takes at most 1 day.

PROVES:
- plumbing and perception on real S10 lidar: frames, yaw sign, s-projection against ground truth
- the node survives a whole bag
- CPU cost
- a false BLOCKED / DETOUR / OFF_CORRIDOR / HOLD triage

CANNOT PROVE: closed-loop tracking, the response to a truly blocked path, gait switching, /NAV_CMD acceptance, the ASDU mode, or anything about a saved map or localisation mode. The report header must say so. Routes built here are REPLAY ONLY.

GATE: Stage 0 is done, which gives odom.npz, jump and gap lists, and H.

**步骤**

- Run tools/bag_to_route.py on v4_room odom.
- bag_to_route pipeline: split at jumps and gaps; decimate at 5 cm; remove each reverse or strafe spur as a whole out-and-back, splitting into separate route directories if the residual gap exceeds 0.30 m; optionally smooth with an arc window of at most 0.3 m and assert deviation < 0.03 m; resample at 0.20 m.
- bag_to_route waypoints: one every 3 m indoors, 5 m for v5_5. Force one at the start, the end, turnaround tips and after in-place turns over 60 deg. Minimum segment length is 1.0 m.
- bag_to_route arguments: --body-z-offset defaults to the value in config/nav.yaml.
- bag_to_route outputs: route pieces, report.json, truth.csv mapping receive time to s_truth, and pieces.json listing (route_dir, t0, t1).
- Run tools/replay_shadow.py in offline mode. It emulates the node: latest pose by bag receive time, Perception.observe per cloud when pose age <= pose_timeout, and NavCore.step at 20 Hz.
- replay_shadow builds a fresh NavCore per piece from pieces.json, because NavCore has no start index.
- Pass A uses config/nav_replay.yaml. This is nav.yaml plus runner_params.stall_time 1e9, plus the adjusted scan band if H < 0.35 m. Pass B uses production nav.yaml and lists what would have latched.
- Classify events. BLOCKED with reason 'no_route_progress' is a standstill artefact, caused by the follower's 20 s timeout, which NavCore does not expose. Those events and the following 1.5 s of TRACK are excluded from the ratios.
- Log every HOLD onset with its cause (off_route or stall). Restart the core at the next piece after a HOLD.
- Events near walls or the operator are 'correct'.
- A definite false positive is a cell marked OBSTACLE or UNKNOWN that the recorded footprint drove over within 5 s.
- Steering-sign check: at sampled ticks, deepcopy the NavCore and step the copy once. Offset the pose laterally by +/-0.15 m, or rotate the yaw by +/-20 deg, and feed core.flat_observation in place of the real cloud. Score the sign of wz and vy. The main replay state is never perturbed. This is reported strictly as a sign check.
- Per-event snapshot data (npz: decimated cloud, local grid, route slice, the next 5 s of recorded path) is rendered to PNG on the host. Events are binned by recorded speed and yaw rate.
- Run one real-time ROS e2e pass with tests/nav/run_replay_e2e.sh: roscore -p 11312, then s10_rl_nav_ros1.py --shadow --autostart --config config/nav_replay.yaml --route-dir <piece>, then `rosbag play <all chunks in order> --topics /LIDAR/POINTS /base_link/odom`.
- e2e command rules: bags first and --topics last; rate 1.0; no --clock, no -l, and use_sim_time unset. Record /rl_nav/status and compare it with pass A. Sample CPU with psutil.
- Repeat the route build and pass A on v5_5. This produces a report only.
- Time-box the false-alarm triage to one day. The output is a fix list, not a tuning loop.

**要新写的**

- tools/bag_to_route.py
- tools/replay_shadow.py plus tools/plot_replay_report.py on the host
- config/nav_replay.yaml
- tests/nav/test_bag_to_route.py
- tests/nav/run_replay_e2e.sh

**通过标准**

- GATING, binary: the node and the harness run through every piece of v4_room without an exception. In the e2e pass, /rl_nav/status stays at 20 +/- 1 Hz for the whole bag and /rl_nav/cmd_vel has 0 messages.
- GATING: the body-frame direction of recorded velocity has a median within 10 deg of forward while driving forward. The wz sign agrees with the recorded yaw rate more than 85% of the time when |w| > 0.15 rad/s.
- GATING: |s - s_truth| p99 < 0.3 m with no jump to another pass. Every piece reaches 100% progress in pass A, moving ticks only.
- GATING: levelled ground sits at -H +/- 0.03 m with tilt < 2 deg. Scan bins under 1 m in open floor are < 5%. If not, the scan band is fixed before any robot use.
- GATING: the steering-sign check is correct on at least 95% of perturbed single steps.
- GATING: on the Mac, tick p99 < 25 ms and per-cloud p99 < 50 ms. Ticks with pose_age > 0.3 s are < 1%. AGX cost is measured live in Stage 3.
- REPORT ONLY, no threshold: WALK share, RUNNING/GATE share, BLOCKED + OFF_CORRIDOR share inside the 0.30 m/s and 0.6 rad/s envelope, the definite-false-positive list, and pivot-while-moving share. Every event comes with a snapshot and a class: correct, standstill artefact, fast-turn artefact, or defect.
- The offline pass A and the e2e pass agree on the WALK share within 2 percentage points and on the waypoints reached.

**风险**

- A good replay may be mistaken for proof that navigation works. It is not.
- Human teleop turns are faster than our envelope, poses are about 0.1 s stale, and the same cloud is re-fed at two poses. All of this inflates false alarms, which is why events are binned.
- UNKNOWN cells block candidates exactly as OBSTACLE cells do, so the false-alarm rate on real data may be high. The output is a fix list, not a tuning spiral.

### Stage 2 - Pre-motion fixes, per-stage control configs, scripts (desk work, parallel with Stage 1)

- **需要机器人：** No. Nothing is deployed to the AGX in this stage.
- **用时：** 1.5-2 days

Remove the defects that would make the first on-robot run unsafe or misleading. Make the speed caps and the armed-session attack surface real rather than procedural.

PROVES, against mocks: the fixes work and the existing suites stay green.

CANNOT PROVE: behaviour on the real robot.

GATE: none.

**步骤**

- Add a --config <yaml> option to scripts/start_control.sh and scripts/start_nav.sh. Both currently hard-code their config.
- Ship config/control_zero.yaml: limits 0/0/0, start_latched true.
- Ship config/control_probe.yaml: limits 0.10/0.05/0.20.
- Ship config/control_flat.yaml: limits 0.20/0.05/0.20.
- In all three, set web_cmd_topic to /s10_control/web_cmd and set cmd_sources to rl_nav only. The vendor page's /web_cmd and w_nav's /cmd_vel are then not heard while armed.
- The operator reads max_vx, max_vy and max_wz from the control node's 'start' event before sending cmd4.
- nav/s10_rl_nav_ros1.py: --shadow also suppresses /rl_nav/gait_request.
- scripts/start_nav.sh --shadow refuses to start when /s10_control/state shows enable_motion true.
- Node: wrap tick() in try/except. On an exception it publishes a zero command and a status carrying the error. Add tick_ms to /rl_nav/status.
- Node: fall back to a finite-difference twist when the Odometry twist is all zero.
- Node: add a pose-jump hold. A jump over 0.25 m or 20 deg between consecutive poses gives a zero command and a paused state that needs an operator 'start'.
- Node: store each observation with its capture pose and feed it to the follower once.
- Node: add --start-wp, or 'resume at the nearest unscored waypoint'. 'reset' optionally keeps the cursor.
- nav/nav_core.py: warn or raise on unknown top-level config keys.
- nav/nav_core.py: warn when the route's source.body_z_offset differs from cfg body_z_offset.
- nav/nav_core.py: expose no_progress_timeout.
- nav/nav_core.py: share the pose-message conversion with the replay harness.
- Runner change: TRACK and DESCEND keep speed 0 for the first runs. With track speed 0, vy and wz are also zeroed.
- Runner change: a follower refusal is recoverable. It must not become a sticky HOLD 'no map to plan on', and it auto-resumes when the follower drives again.
- Make the runner change UPSTREAM in wt-rl-nav/src/s10_auto_nav/s10_auto_nav/rl_nav/route_runner.py, then re-sync with nav/sync_auto_nav.sh. A local edit is wiped by the sync, which runs rm -rf. Deploy the runner and the yaml together, because an unknown runner_params key raises at start-up.
- tools/teach_to_route.py: nearest_index searches a forward window only.
- tools/teach_to_route.py: --body-z-offset defaults to the value in config/nav.yaml.
- s10-field-assistant teach_worker.py: flush and os.fsync the trail and marks writers every N rows.
- s10-field-assistant teach_worker.py: show the active data dir on the page.
- Document that the worker must be restarted after any SSD plug or unplug, because it picks its data dir only at start.
- Add a bag_to_route mode that rebuilds a taught path from a path-mode bag when trail.csv is missing.
- Build tools/straight_from_live_pose.py and scripts/replay_rebuild.sh.
- Build tools/compare_odom.py and tools/run_report.py.
- Write docs/ARMED_SESSION_CHECKLIST_ZH.md, including the dead-AGX restore branch.
- Fix stale texts: the comment at start_control.sh:8-10, README_ZH.md:221 and :237, and design doc line 301.
- Re-run all suites. Re-run Stage 1 pass A as a regression check.

**要新写的**

- --config option in both start scripts
- three control_*.yaml configs
- node and nav_core fixes
- upstream runner change, then sync
- teach_to_route fixes
- teach_worker fsync
- straight_from_live_pose.py
- replay_rebuild.sh
- compare_odom.py
- run_report.py
- ARMED_SESSION_CHECKLIST_ZH.md
- new tests in tests/nav and tests/control

**通过标准**

- Existing suites stay green: control vs mock 36/36, kinematic 30/30, ROS 1 nav e2e.
- New control mock tests pass. With control_zero.yaml, no non-zero /NAV_CMD is ever sent.
- With the private web_cmd_topic, a 'cmd4', 'Nav continue' or 'source x_nav' on /web_cmd is ignored. /cmd_vel is not forwarded.
- New nav tests pass: a blocked centreline gives cmd exactly (0,0,0), and the runner resumes WALK when the block clears, with no restart from WP0.
- New nav tests pass: a tick exception gives a zero command plus a status; zero-twist Odometry still yields v_smooth > 0; shadow publishes 0 gait_request messages; a pose jump gives a paused hold.
- start_nav.sh --shadow exits non-zero against a mock control node reporting enable_motion true.
- The exact command line of replay_rebuild.sh is dry-tested on the Mac (Docker, private roscore, dummy subscriber, tiny synthetic bag). It plays only /LIDAR/POINTS and /IMU, and it refuses when a publisher exists.

**风险**

- The runner copy changes first-version nominal behaviour. Keep the change minimal and upstream.
- Whether zero limits and start_latched interact badly with the stand sequence in control.cpp is unverified. The mock test must cover the stand sequence under control_zero.yaml.
- The teach_worker fix lives in another repo (s10-field-assistant) and is deployed separately.

### Stage 3 - On-site, NO autonomous motion: deploy, save a map in the first 10 minutes, localise, teach, live shadow (remote-driven, control DRY RUN)

- **需要机器人：** Yes. The user is on site and drives with the remote only. The control node stays in DRY RUN, so nothing from our stack reaches the robot. The user performs every AGX write.

Part A alone (map, save, localisation check) needs no new code and can be done today if the user is still with the dog.
- **用时：** Part A alone takes about 20-30 min. The full stage takes 2.5-3 h, or two shorter sessions. Budget 5-10 min of recovery per AGX reset: boot, docker start nav, PTP sync from 1970, gateway, re-select the map, set pose.

Get a real saved map and a real taught route. Measure x_nav live.

PROVES:
- where and whether save works on the ARM beta image
- /base_link/odom rate, twist and continuity in localisation mode
- live perception, and the dry-run command stream on a taught line
- AGX CPU cost

CANNOT PROVE: that the robot tracks our commands.

GATE for Part A: the user is on site, the dog is identified by serial, and a quiet slot is agreed.

GATE for Parts B-D: Stage 2 is deployed and the Stage 1 gating checks passed.

**步骤**

- Deploy once, in a no-motion slot, over the dog's LAN: set S10_AGX_SSH to a direct host and run deploy_nav.sh --no-test on the dog. Do not use the Tailscale relay.
- After the deploy, run stop_control.sh and then start_control.sh without a flag. Confirm enable_motion:false in /s10_control/state.
- Confirm agx_power_log.py is running and that logs/power/*.csv grows.
- Restart the teach worker after any SSD plug or unplug.
- PART A, first 10 minutes: run robot_session.sh up.
- PART A: 新增地图, walk 2-3 min in the flat test room by remote, then press 保存地图 AT ONCE.
- PART A: run ls /opt/data/nav_map/<name>, then copy the folder to the SSD or the Mac.
- If a rebuilt v4_room map exists from Stage 3R and the dog is in that room, select it instead.
- 选择地图 + 设置位姿.
- Read-only localisation checks: rostopic hz /base_link/odom; 20 samples of twist while the user walks the dog; a 60 s standing drift; a 3 m out-and-back return error.
- PART B: run tools/check_ground_plane.py live for H in the remote-driven gait and compare it with the Stage 0 value.
- Set body_z_offset in config/nav.yaml only if pose z and H are in the same frame. Otherwise keep it flagged.
- Every route tool uses the same offset.
- PART C: teach in localisation mode with /teach. Use survey marks WP01..WP0n: 3-5 waypoints over 6-10 m, flat, no SWIN/SWOUT.
- PART C: record one path-mode recording.
- Record to ~/teach on the eMMC, because path-mode bags are small. Alternatively, plug the SSD in only for that recording and restart the worker.
- Run tools/teach_to_route.py <session> --map-id <map> --out <dir>.
- PART D: run start_nav.sh --shadow with the taught route and control in DRY RUN. The user sends 'start' and drives the route by remote, twice.
- PART D: run tools/run_report.py on the recorded /rl_nav/status plus the control events.
- PART D: check that dry_run_would_send shows only zero or forward commands within the clamps and only 'flat' gait requests. Record the AGX tick_ms and obs_ms.
- Handover checklist:
  - robot lying
  - control in dry run
  - stop_nav.sh
  - robot_session.sh down while the AGX is alive, because it is the ssh jump host
  - 103 phone forwarder removed if it was used
  - an explicit 'keys stay' or 'unkeys' decision recorded
  - no ASDU change was made in this stage
- Dead-AGX branch, run from the operator laptop on the robot network:
  - ssh user@10.21.33.106 to stop and remove ~/ros1_gateway_tap
  - remove the 's10-ros1-gateway-agx' line from authorized_keys on 106
  - power off the AGX
  Without this, run/session_active stays set and the next AGX boot reinstalls the 106 tap by itself.

**要新写的**

- Nothing new beyond Stage 2. This stage uses robot_session.sh, start_nav.sh --shadow, teach_to_route.py, check_ground_plane.py and run_report.py.

**通过标准**

- The saved map folder is present on the host after 保存地图, and a backup copy exists. This artefact must exist within about 10 minutes of session start.
- Localisation mode: /base_link/odom runs at 10 +/- 1 Hz with inter-arrival p99 < 0.2 s. Standing drift is < 0.03 m and < 1 deg over 60 s. The out-and-back return error is < 0.10 m. No pose jump exceeds 0.25 m while walking.
- A twist verdict is recorded for localisation mode. If the twist is zero, the fallback is active and v_smooth tracks the finite-difference speed within 0.05 m/s.
- teach_to_route output passes RouteV2 validation. The trail file is complete, because the fsync fix is deployed.
- Live shadow, twice: all waypoints are scored in order, no HOLD latches, and the commanded wz sign agrees with the driven yaw rate more than 85% of the time.
- On the AGX, tick p99 < 40 ms and obs p99 < 80 ms.
- Every BLOCKED or OFF_CORRIDOR event is explained. This is report-level: a share above 5% triggers a review before Stage 6, not an automatic fail.
- Any AGX reset is captured by the power CSV up to within 1 s. A reset here does not block the flat low-speed stages, but it is logged with its rail values.

**风险**

- The AGX may reset while walking; both earlier losses happened that way. Hence save first and extend later.
- The x_nav UI on :8000 is open to anyone on the robot network. It is harmless while control is in dry run.
- Whether save works on the ARM beta image, where map_manager is missing, is unknown until tried.
- The robot is shared. Only the 106 tap is added by 'up', and it must be removed by 'down' before handover.

### Stage 3R (optional experiment, no motion, time-boxed to 2 attempts) - Rebuild a saved map from the v4_room bags by replay into x_slam on our AGX

- **需要机器人：** AGX only. The x_nav licence is bound to its eMMC. The dog is lying and nothing reaches 103 or 106. The user performs every step: stop the gateway, press 新增地图 and 保存地图, run the script.

This stage can run in dead time when nobody needs the dog. It is not required for any later stage.
- **用时：** 1.5-2 h, including a possible second attempt. v4_room is about 6.3 min per pass at rate 1.0.

Find out whether x_slam can turn the v4_room bag into a saved map. This is the only way a recording becomes 'the map', and it would make future resets non-destructive.

PROVES:
- replay tolerance of the closed binary
- the saved-map layout
- localisation-mode odom under replay

CANNOT PROVE: that the map relocalises with the real dog. That is tested live in Stage 3.

GATE:
- Stage 0 gave the -s offset and the chunk order.
- replay_rebuild.sh is dry-tested on the Mac.
- The power logger is running.
- robot_session.sh down is done: zero 106 footprint and no session_active, so a reset cannot restart the tap.
- The nav node is stopped, and control is stopped or confirmed enable_motion:false.

**步骤**

- Read-only first: ls ~/ros1_gateway/run. If roscore.pid exists, the master is ours, not the container's. In that case stop only the gateway pid, not the roscore.
- Read-only: run df on the eMMC. Play the bags from the SSD, or from an eMMC copy if at least 12 GB is free. Record rebuilt_odom to the eMMC, never to the SSD.
- Stop the gateway. Verify that rostopic info /LIDAR/POINTS and /IMU show 0 publishers and that rostopic hz shows no data. /IMU arrives over DDS even with the tap off.
- In the x_nav UI: 新增地图, name v4room_rb1. Check that pgrep -fa x_slam shows 'x_slam mapping /nav_map/v4room_rb1'.
- Run scripts/replay_rebuild.sh. It first checks the preconditions.
- The script then runs: rosbag play -d 3 -r 1.0 -s <first-standstill offset> <chunk_0 chunk_1 chunk_2 chunk_3.recovered> --topics /LIDAR/POINTS /IMU.
- Command rules: bags FIRST and --topics LAST; all chunks in ONE command; never play /base_link/odom or /teach/mark; never use --clock, --loop or a rate above 1.
- In parallel the script runs rosbag record -O ~/rebuilt_odom /base_link/odom /x_nav/slam/keyframe_pose.
- Attempt 1 runs at rate 1.0, which is the condition x_slam already handled live. Attempt 2 runs at 0.5 only if attempt 1 was CPU-bound.
- Use a fresh 新增地图 for every attempt. Never replay twice into one x_slam instance.
- When play ends, press 保存地图 immediately. Verify that /opt/data/nav_map/v4room_rb1/ has a global map .pcd, keyframe pcds and poses_*.txt. Back the folder up.
- Run tools/compare_odom.py to rigidly align the rebuilt odom with the recorded odom.
- Localisation replay: 选择地图, then run rosbag play --pause. First try 设置位姿 at the bag-start spot and then unpause. If that fails, play 2 s, pause, set the pose, and resume. Accept either order. Record /base_link/odom rate, twist and continuity.
- If odom never starts, or 'loop back' or stale-stamp messages appear twice, stop and abandon the rebuild. Stage 3 makes a fresh map anyway.
- /use_sim_time is attempted only in a later dedicated session with all our nodes stopped.
- Afterwards run start_gateway.sh and health_check.sh. Select the map again, which gives a fresh x_slam, before any live use.

**要新写的**

- scripts/replay_rebuild.sh
- tools/compare_odom.py
- Both are built in Stage 2.

**通过标准**

- During replay, /base_link/odom publishes at the play rate times 10 Hz, +/- 10%, for the whole play.
- After rigid alignment, the rebuilt trajectory matches the recorded one with the same shape and RMS < 0.30 m. This is a proxy only, because SLAM run-to-run repeatability limits tighter bounds.
- After save, the folder exists with a global map .pcd larger than 1 MB and pose files, and it is backed up.
- Localisation replay: odom is published continuously, with no jump over 0.25 m after initial convergence. A twist verdict is recorded.
- Real acceptance is deferred to Stage 3: live 设置位姿 with the dog in the room gives red and green overlap.
- A reset during the attempt costs only time. The power log must cover it.

**风险**

- x_slam may compare stamps with the wall clock. Off the robot network the AGX clock can read 1970, so prefer doing this on the robot network with PTP up.
- The dog may have been moving at bag start, which is why the first-standstill -s offset is used.
- The rebuilt frame origin is the play-start pose, so the old trail.csv is NOT in this frame.
- The room may have changed since the recording.
- Whether save_map works without map_manager is unknown.
- w_nav and other consumers see the replayed clouds. None can move the robot, because control is in dry run with cmd_source rl_nav. The argument 'no /MOTION_INFO with the gateway stopped' is wrong: the control node reads ROS 2 directly.

### Stage 4 - First armed session at ZERO velocity (control_zero.yaml), navigator NOT running

- **需要机器人：** Yes. This is the first time our node publishes to the robot: stand, gait and lie only.

The user is on site holding the remote as the only e-stop, with a second person on the laptop. The user types every command and confirms every step. Nothing arms the control node, sends cmd4/cmd3 or 'start', or sets the ASDU mode unless the operator on site runs that command.
- **用时：** 45-60 min on site

PROVES:
- 103 accepts /MOTION_STATE and /GAIT from our AGX
- the stand sequence, Nav stop and continue, and gait switching at standstill
- the remote can take the robot back, also with our control node gone
- standing height in gait 0x3002

CANNOT PROVE: anything about velocity commands.

GATE:
- Stage 3 Parts A-D are done: live shadow passed, control confirmed in dry run, deploy done.
- The other teams confirm that no native navigation task is running.
- From the operator laptop on the robot network, `python3 asdu_mode.py status` works without the AGX.
- stop_nav.sh reports nav is not running.
- The SSD is unplugged and nothing is recording.

**步骤**

- Read the ASDU status, read-only. It should be mode 0.
- Run stop_control.sh, then start_control.sh --enable-motion --config config/control_zero.yaml.
- Read the 'start' event: limits 0/0/0, web_cmd_topic /s10_control/web_cmd, source rl_nav only.
- Check that the 'armed' event is present and the fault is empty.
- The user publishes 'cmd4' on the PRIVATE web_cmd topic. Watch state 1, then a 2 s settle, then RL(17), then gait 0x3002. Each step is confirmed on /MOTION_INFO within 15 s.
- Test 'Nav stop' and 'Nav continue'.
- Request stairs, then flat, on /rl_nav/gait_request at standstill. Each is confirmed within 10 s.
- While standing in 0x3002, run tools/check_ground_plane.py (read-only) and record H. Decide the scan band and body_z_offset now.
- Remote take-back, standing still: try the sticks and the mode switch.
- Repeat the take-back after stop_control.sh. This simulates the AGX being gone: graceful this time, kill -9 later.
- The user sends 'cmd3' to lie down.
- Run stop_control.sh, then start_control.sh without a flag (dry run).
- Read the ASDU status again. It should still be mode 0.
- Run the handover checklist.

**要新写的**

- Nothing new. This stage uses control_zero.yaml and the checklist from Stage 2.

**通过标准**

- The node arms without a fault. Each stand step is confirmed within 15 s with at most 1 resend, and there is no sequence_failed.
- The gait switch is confirmed within 10 s in both directions, only at standstill.
- Nav stop latches a zero command, and Nav continue releases it.
- No non-zero /NAV_CMD appears in the control event log during the whole session.
- The remote regains control of the standing robot within 2 s of operator action, with our control node stopped. This is demonstrated, not assumed.
- H in 0x3002 is measured with a spread of at most 0.03 m. The verdict against 0.35 m is recorded.
- The ASDU mode is 0 before and after. The robot ends lying, with control back in dry run.

**风险**

- 103 may reject AGX-sent commands. Timestamp check failures were seen on dog 48. That would latch sequence_failed, which is safe, and the right response is to stop and diagnose without changing 103.
- An AGX reset while the robot is standing leaves it standing in RL with nobody commanding. That is why the take-back is proven here first.
- With the private web_cmd topic, the vendor page can no longer stand or lie the dog. The page is still kept on the operator's device only.

### Stage 5 - Velocity probe at 0.10 m/s, ASDU-mode answer, remote override, command-loss (kill -9) test

- **需要机器人：** Yes. This stage involves MOTION.

Conditions:
- The user is on site with the remote in hand.
- The floor is flat, with at least 5 m of free space ahead, and nobody else is within 3 m.
- The user starts every run and issues every kill.
- The SSD is unplugged, and the only recording is a small status/odom bag on the eMMC.
- **用时：** 1.5 h on site

Answer the three unknowns that only motion can answer:
- (a) Is /NAV_CMD obeyed in ASDU mode 0, or only in mode 1?
- (b) Are direction and scale correct?
- (c) What does the robot do when /NAV_CMD stops abruptly, which is what an AGX reset mid-motion looks like?

PROVES: the three answers above, plus stop latencies.

CANNOT PROVE: route tracking.

GATE: Stage 4 passed. control_probe.yaml has limits 0.10/0.05/0.20.

**步骤**

- Arm with control_probe.yaml and stand as in Stage 4.
- Run tools/straight_from_live_pose.py --length 0.6 --radius 0.10 --flat-speed 0.10. It reads the live pose itself and prints the heading in degrees, which the operator compares with the dog.
- Simulation verified that this route travels 0.50 m in 5 s.
- Rehearsal: run the route once with control in DRY RUN. Check that dry_run_would_send shows vx > 0 and |wz| < 0.1. Then arm.
- The user sends 'start', with a cap of 8 s. Tape-measure the travel and compare it with the odom delta and the direction.
- If the robot does not move in mode 0, the ASDU question is answered. The user runs asdu_mode.py set-mode 1 --i-am-on-site.
- Before moving in mode 1, and standing still, verify the remote sticks, the mode switch and the e-stop. If the sticks are dead in mode 1, the e-stop is the only stop (joint damping; the robot drops). In that case stay on flat ground at 0.10 m/s.
- On a 2 m route at 0.10 m/s, which gives about 18 s of motion, test each stop method once: private 'Nav stop', /rl_nav/cmd pause, and stop_nav.sh. Measure each from odom with run_report.py.
- Remote override: on one 0.10 m/s run, the operator takes over by remote while our node is still commanding. Expect the gait to leave 0x3002 and our node to log velocity_stop with reason not_navigation_mode.
- Command loss 1: the USER runs kill -9 on the control node during a pure in-place yaw of 0.2 rad/s.
- Command loss 2: the USER runs kill -9 during 0.10 m/s forward motion on the 2 m route, twice. Measure stop time and distance. The operator is ready on the remote.
- Restore: robot lying, ASDU mode 0 verified by a status read, control in dry run, stop_nav.sh, robot_session.sh down.
- After any AGX reset, the first action is an asdu status read from the laptop, and mode 0 if no test is running.

**要新写的**

- Nothing new. This stage uses straight_from_live_pose.py, control_probe.yaml and the run_report.py probe mode from Stage 2.

**通过标准**

- Probe: the robot moves forward within 1 s of start, and not sideways or backward. Travel is LENGTH minus radius, 0.50 +/- 0.10 m, by tape. Tape and odom agree within 0.10 m. Achieved speed is 0.10 +/- 0.05 m/s.
- The ASDU mode that works is written down. If mode 1 was used, the remote and e-stop were verified in mode 1, and mode 0 was restored and verified.
- Each software stop brings the measured speed under 0.05 m/s within 1.0 s.
- Remote override works against an active /NAV_CMD stream.
- kill -9: the robot stops by itself within 1.0 s and 0.15 m, in the yaw test and in both forward repeats. If it does NOT stop by itself, the plan halts: no autonomous route is driven until a robot-side or vendor answer exists.
- The measured stop distance d_loss is recorded. Every later stage keeps clearance of at least 2 x d_loss, scaled by speed, so x2 again when running at 0.20 m/s.
- There is no unexpected fault latch and no foreign /NAV_CMD.

**风险**

- This is the first real motion. It is limited by control_probe.yaml hard clamps, by --flat-speed 0.10 as a second layer, by a short route, and by the operator on the remote.
- The robot may keep walking after command loss; the SDK does not document this. Hence the yaw-first order and the 5 m of free space.
- set-mode 1 changes shared state on 103 and must be restored. The laptop path exists for the case where the AGX dies.
- A foreign /NAV_CMD publisher latches our fault within about 1 s, but it keeps driving the robot. Only the remote stops that.

### Stage 6 - First closed-loop autonomy on flat ground: straight 2 m, then the taught route on the saved map

- **需要机器人：** Yes. This stage involves MOTION.

Conditions:
- The user is on site with the remote, plus a spotter who can send /rl_nav/cmd pause.
- The flat room keeps clearance of at least 2 x d_loss, scaled to 0.20 m/s, from edges and people.
- The user starts every run.
- No stairs, no outdoor ground and no gait switch.
- **用时：** 1.5-2 h on site. It can share a half day with Stage 5.

This is the actual answer to 'does our autonomous navigation work': closed-loop tracking in localisation mode on a saved map, through s10_ros1_control and the native gait.

PROVES: flat-ground tracking at up to 0.20 m/s on a short route.

CANNOT PROVE: stairs, the full course, or long-duration robustness.

GATE:
- Stage 5 passed, including the self-stop on kill -9.
- A saved map is selected, the pose is set, and red and green overlap.
- body_z_offset is verified and identical in nav.yaml and the route tool.
- control_flat.yaml has limits 0.20/0.05/0.20.

**步骤**

- Straight 2 m: run straight_from_live_pose.py --length 2.0 --radius 0.10 --flat-speed 0.20. Do a dry-run rehearsal first, then run it armed, twice.
- Taught route from Stage 3 (3-5 waypoints, 6-10 m): first one shadow pass with control in DRY RUN, remote-driven.
- Then run the taught route armed and autonomous, twice, with allow_detour off.
- Optionally run it twice more with allow_detour on.
- After each run, use run_report.py to report: cross-track error p95 and max from the status d; heading error at stop; overshoot after DONE; tape against odom; the number of stops and heading reversals; and takeovers. Report every HOLD and BLOCKED with its reason.
- Run the handover checklist, including the dead-AGX branch if needed.

**要新写的**

- Nothing new.

**通过标准**

- Each run reaches DONE without a takeover. Takeovers and failures are listed separately and are never averaged.
- Straight 2 m: cross-track p95 <= 0.10 m and max <= 0.20 m; heading error at stop < 10 deg; overshoot after DONE < 0.15 m. End-point error within 0.20 m is only a sanity bound, because it holds by construction.
- Taught route: all waypoints are scored in order; cross-track p95 <= 0.15 m; no HOLD; at most 1 explained BLOCKED per run.
- Commanded speeds stay within 0.20/0.05/0.20. There are 0 control faults and 0 sequence failures.
- An AGX reset mid-run is tolerated only because Stage 5 showed that the robot stops by itself. The reset is logged, and the run is repeated.

**风险**

- A false BLOCKED now gives a recoverable zero-speed stop, which fails the run but is safe.
- A localisation jump is caught by the new pose-jump hold. The operator watches the red and green overlap.
- People may enter the area. The spotter pauses the run, and the remote is the final stop.
- Deferred until the SYS_RESET_N cause is found and the flat runs are repeatable: a gait switch inside a route on flat ground, one stair flight at 0.15 m/s with a spotter, the v5_5 area, and the full course. A reset on stairs leaves an uncommanded robot whose only stop is a joint-damping drop.

## 要新写 / 要改的东西

- `tools/bag_inspect.py (+ tools/plot_bag_report.py on the Mac host)`：Read-only census and parameter extraction from a session's bags.
  - bag_inspect.py runs in Docker s10-ros1-nav-test (--network none, /data:ro) with the rosbag Python API and numpy only. Input is --session <dir>, with chunks sorted and *.recovered.bag last.

Outputs go to a scratch dir, never the SSD.

odom.npz holds t_recv, t_hdr, xyz, rpy and twist.

report.json holds:
- per-chunk start, end, counts, rates, gaps and truncation
- twist verdict (filled / zero / wrong-frame, correlation against finite difference)
- odom inter-arrival p50/p99/max
- cloud-to-odom header lag
- jump list (|dxy| > 0.25 m, |dz| > 0.15 m, |dyaw| > 20 deg)
- H median and spread, and levelled tilt. Uses an importable plane fit from tools/check_ground_plane.py, moving samples only (speed > 0.1 m/s), cells within 2 m of the dominant plane.
- first-3-s IMU variance, and the first standstill of at least 2 s (start offset)
- pauses longer than 8 s and longer than 20 s
- reverse and strafe intervals
- speed and yaw-rate histograms
- z and pitch ranges
- stairs and slope candidates (|pitch| > 12 deg sustained, or |dz/ds| > 0.25)
- self-return histogram within 1.2 m
- trail.csv against odom over the overlapping span only

plot_bag_report.py runs on the host with matplotlib and renders traj.png, z_profile.png and selfreturn.png. The Docker image has no plotting library.
- `tools/bag_to_route.py + tests/nav/test_bag_to_route.py`：Build REPLAY-ONLY RouteV2 pieces from recorded odom. Also rebuild a taught path from a path-mode bag when trail.csv is missing.
  - Inputs:
- --odom odom.npz | --bags ... | --trail trail.csv
- --t0 / --t1
- --body-z-offset (default read from config/nav.yaml and recorded in source)
- --wp-spacing 3.0 | 5.0
- --radius 0.30
- --tol-z 0.30
- --speed 0.30
- --detour | --no-detour (corridor 0.8 | 0.5)

Pipeline:
1. Split at jumps and at chunk gaps over 0.5 s.
2. Decimate at 5 cm.
3. Remove each reverse or strafe spur as a whole out-and-back, from the first forward pass of the point reversed back to. If the residual gap exceeds 0.30 m, split into a new piece.
4. Optionally smooth with an arc moving average of at most 0.3 m, asserting max deviation < 0.03 m.
5. Resample at 0.20 m.
6. Place waypoints every wp-spacing, forced at the start, the end, turnaround tips (heading change over 120 deg within 1.5 m of arc) and after in-place turns over 60 deg. Minimum segment length is 1.0 m. Ids run from R001.

Outputs per piece: route_v2.json (map_id 'replay:<session>', status 'REPLAY ONLY', no maneuvers.json), report.json (self-overlap, tips, cuts, stairs candidates) and truth.csv (t_recv mapped to s_truth). One pieces.json holds [(route_dir, t0, t1)].

Validate each piece with RouteV2.load.

Tests cover synthetic out-and-back, in-place turn, pause, jump and reverse spur.
- `tools/replay_shadow.py + tools/plot_replay_report.py + config/nav_replay.yaml + tests/nav/run_replay_e2e.sh`：Deterministic offline replay of pose and clouds through Perception and NavCore, plus comparison with one real-time ROS e2e run.
  - Offline mode runs in Docker. It takes pieces.json and builds a fresh NavCore per piece. It keeps the latest pose by receive time, runs Perception.observe per cloud when pose age <= pose_timeout, and calls NavCore.step(t, pose, obs, None, obs_age) at 20 Hz. It shares the pose conversion with nav_core.

Pass A uses nav_replay.yaml: nav.yaml plus runner_params.stall_time 1e9, plus a scan-band override if H < 0.35 m. Pass B uses production nav.yaml.

ticks.jsonl holds, per tick: t, pose, recorded v and w (finite difference and twist), s, s_truth, mode, follower status and reason, cmd, tick_ms, obs_ms, n_points, grid valid fraction, ground median, scan NaN count and minimum, and free_length of the d=0 candidate.

Event classification:
- A 'no_route_progress' BLOCKED is a standstill artefact, together with the following 1.5 s of TRACK.
- A HOLD is logged with its cause (off_route or stall).
- An event during a recorded yaw rate over 0.6 rad/s or speed over 0.30 m/s is outside the envelope.

Event data is written as npz: decimated cloud, local grid, route slice and the next 5 s of path. The host renders PNG and HTML.

Steering-sign check: at sampled ticks, deepcopy the core. Step the copy once with the pose laterally offset by +/-0.15 m or the yaw rotated by +/-20 deg, feeding core.flat_observation. Score the sign of wz and vy. Never mutate the main state.

The --compare-status-bag option diffs a recorded /rl_nav/status against pass A.

run_replay_e2e.sh is modelled on run_nav_ros1_test.sh. It starts roscore on 11312 and the node with --shadow --autostart. It plays `rosbag play <bags...> --topics /LIDAR/POINTS /base_link/odom` at rate 1.0 with no --clock, and samples CPU with psutil.
- `scripts/start_control.sh, scripts/start_nav.sh (--config option) + config/control_zero.yaml, control_probe.yaml, control_flat.yaml`：Make the per-stage speed caps real in the only authoritative limiter. Remove the vendor web page and /cmd_vel as motion surfaces while armed.
  - Both scripts accept --config <yaml>, defaulting to the current file.

The three control configs are copies of control.yaml with:
- limits 0/0/0 plus start_latched true (zero)
- limits 0.10/0.05/0.20 (probe)
- limits 0.20/0.05/0.20 (flat)

All three set web_cmd_topic to /s10_control/web_cmd and cmd_sources to {rl_nav: /rl_nav/cmd_vel}, with cmd_source rl_nav.

start_nav.sh --shadow refuses to start when /s10_control/state shows enable_motion true.

Mock tests to add in tests/control:
- The stand sequence works under control_zero.yaml.
- No non-zero /NAV_CMD is sent.
- /web_cmd 'cmd4', 'Nav continue' and 'source x_nav' are ignored.
- /cmd_vel is not forwarded.

Also fix the stale arming comment at start_control.sh:8-10.
- `nav/s10_rl_nav_ros1.py, nav/nav_core.py (ours) and wt-rl-nav/.../rl_nav/route_runner.py (upstream, then nav/sync_auto_nav.sh)`：Pre-motion fixes found in the code review.
  - Node:
- --shadow suppresses gait_request.
- tick() is wrapped in try/except. On an exception it publishes a zero command and a status carrying the error. tick_ms is added to the status.
- The twist falls back to finite differences when the Odometry twist is all zero.
- Pose-jump hold: a jump over 0.25 m or 20 deg gives a zero command and a paused state that needs an operator 'start'.
- Each observation is stored with its capture pose and fed to the follower once.
- --start-wp, or resume at the nearest unscored waypoint. 'reset' optionally keeps the cursor.

nav_core:
- Warn or raise on unknown top-level config keys.
- Warn when the route's source.body_z_offset differs from the cfg value.
- Expose no_progress_timeout.
- Share the pose-message conversion.
- Make the scan-band lower bound a config value.

Runner, upstream:
- TRACK and DESCEND speed becomes an explicit parameter, default 0.
- When it is 0, vy and wz are also 0.
- A follower refusal does not latch the HOLD 'no map to plan on'. It is a recoverable stop that resumes WALK when the follower drives again.
- Deploy the runner and the yaml together, because an unknown runner_params key raises at start-up.

Tests to add in tests/nav/test_nav_core.py, as listed in the Stage 2 pass criteria.
- `tools/straight_from_live_pose.py`：Build the --straight test route from the live pose without hand-typed numbers.
  - Read-only ROS 1 subscriber to /base_link/odom. It averages 1 s of pose and refuses if the pose is moving or stale.

It calls the same build_straight as teach_to_route, with --length, --radius (default 0.10), --flat-speed and --map-id. The body-z-offset is read from config/nav.yaml.

It prints x, y, z and the heading in DEGREES, plus the expected travel (LENGTH minus radius), for the operator to compare with the dog.

It publishes nothing.
- `scripts/replay_rebuild.sh + tools/compare_odom.py`：User-run x_nav map rebuild from a bag on the AGX, plus trajectory comparison.
  - replay_rebuild.sh refuses to run unless ALL of these hold:
- /LIDAR/POINTS and /IMU have 0 publishers.
- A fresh 'x_slam mapping|localization' process exists.
- run/session_active is absent.
- The nav node is not running.
- /s10_control/state shows enable_motion false, or control is stopped.
- The teach worker is idle.
- agx_power_log.py is running.

It warns if run/roscore.pid exists, because the master is then ours and the roscore must not be stopped.

It builds the ordered chunk list and runs: rosbag play -d 3 -r ${RATE:-1.0} [-s OFFSET] [--pause] <bags...> --topics /LIDAR/POINTS /IMU. Bags go first and --topics goes last.

It records /base_link/odom and /x_nav/slam/keyframe_pose to the eMMC, and prints 'PRESS 保存地图 NOW' at the end.

The exact command line is dry-tested on the Mac in Docker with a synthetic bag.

compare_odom.py does a rigid 2D or 3D alignment of two odom tracks and reports RMS, max and end-pose error. It writes npz for host plotting.
- `s10-field-assistant/tools/s10_mapping_web/teach_worker.py + tools/teach_to_route.py`：Stop losing taught paths on an AGX reset. Fix the out-and-back mark snapping.
  - teach_worker:
- The trail and marks writers flush and os.fsync every N rows, the same pattern as agx_power_log.py.
- The page shows the active data dir.
- Document that the worker must be restarted after any SSD plug or unplug, because the data dir is chosen at start only.

teach_to_route:
- nearest_index searches a forward window only.
- --body-z-offset defaults to the value in config/nav.yaml.
- Add a test with an out-and-back trail.
- `tools/run_report.py`：One report tool for live shadow runs, probe runs and closed-loop runs. It shares code with replay_shadow.py.
  - Inputs are a recorded /rl_nav/status bag or jsonl, logs/control-events-*.jsonl and /base_link/odom.

Outputs:
- mode and status shares
- cross-track p95 and max
- heading error at stop
- overshoot after DONE
- stops and heading reversals
- stop latency per stop method
- distance and time after kill -9 (last control event to speed < 0.05 m/s)
- commanded against achieved speed
- stand-sequence timings and faults
- AGX tick_ms and obs_ms percentiles
- `docs/ARMED_SESSION_CHECKLIST_ZH.md`：A one-page operator checklist for Stages 4-6. It replaces the gate.sh idea.
  - Contents:
- Who holds the remote.
- The stop ladder: remote e-stop > private Nav stop > /rl_nav/cmd pause > stop_nav.sh > automatic timeouts > stop_control.sh.
- What to read in the 'start' event: limits, web_cmd topic and sources.
- Hand-filled measured values: H, the ASDU mode that works, and d_loss.
- The clearance rule of 2 x d_loss, scaled by speed.
- The handover list:
  - robot lying
  - ASDU 0 verified by a status read
  - control in dry run
  - stop_nav
  - robot_session down
  - 103 forwarder removed
  - keys stay or unkeys decided
- The dead-AGX branch, run from the operator laptop:
  - asdu_mode.py status, and set-mode 0 --i-am-on-site
  - ssh user@10.21.33.106 to stop and remove the tap
  - remove the s10-ros1-gateway-agx key line on 106
  - power off the AGX

The checklist states that it is an aid, not a safety barrier.

## 必须先确认的事

- SSD on the Mac: it mounts, and both sessions copy with matching checksums. Do NOT accept a macOS repair or fsck before copying.
- From the bags (Stage 0):
  - whether v4_room is really flat
  - H
  - the twist verdict
  - odom jitter against 0.3 s
  - jumps and chunk gaps
  - stillness at bag start, and the first-standstill offset
- AGX, read-only, when reachable:
  - whether the deployed agx_boot.sh has the agx_power_log.py line, and the logger is running
  - whether s10-stack.service is enabled
  - whether /s10_control/state shows enable_motion false
  - whether ~/ros1_gateway/run/roscore.pid exists, which tells who owns the master
  - df on the eMMC
  - which data dir the teach worker is using
  - docker exec nav grep -n app.route /catkin_ws/src/x_nav_control/scripts/*.py
- A direct LAN or Wi-Fi ssh host entry for the AGX (S10_AGX_SSH) works on site. No step may depend on the Tailscale relay.
- Mock test: the control node runs the stand sequence correctly with limits 0/0/0 plus start_latched true. It also runs correctly with a single cmd source and a private web_cmd_topic.
- The replay_rebuild.sh command line is dry-tested on the Mac, with bags first and --topics last.
- Before any set-mode 1, `python3 asdu_mode.py status` works from the operator laptop on the robot network without the AGX. The tool uses only the standard library and UDP to 10.21.33.103:30004.
- Direct ssh from the operator laptop to user@10.21.33.106 works, for the dead-AGX tap removal.
- The other teams confirm that no native navigation task (103 handler or 106 localPlanner) runs during armed slots.
- Before the probe: a kinematic simulation of the exact probe route. Already done for LENGTH 0.6, radius 0.10 and speed 0.10: 0.50 m in 5 s. Re-run it if the parameters change.
- The same body_z_offset is used in config/nav.yaml and in every route tool call.
- Before Stage 6:
  - the kill -9 self-stop result and d_loss
  - remote override against an active /NAV_CMD
  - the remote and e-stop in ASDU mode 1, if mode 1 is needed

## 只有用户能回答的问题

- 你现在还在狗（050）旁边吗？还能待多久？
  - 如果还在，今天就可以做 Part A：确认电源日志在跑 -> 新增地图 -> 走 2-3 分钟 -> 立刻保存地图 -> ls /opt/data/nav_map/<name> -> 选择地图 -> 设置位姿 -> 看 /base_link/odom。
  - 做完把 SSD 插到 Mac 上。
- 能现在把 PortableSSD 插到这台 Mac 上吗？所有离线工作都等它。Mac 有 173 GB 空闲，需要 22.5 GB。
- 狗现在在哪里？v4_room 那个房间还能用来做上机测试吗？需要平坦、无公众、直线净空至少 5 m、录制后没有大变动。不能用的话，有没有别的平坦房间？
- v4_room 录制开始时，狗在房间的哪个位置？当时是站着不动的吗？这一点会成为重建地图的原点和“设置位姿”的位置。
- v5_5 是在哪里走的？室外吗？有楼梯或坡吗？回到起点了吗？Stage 0 会把轨迹图发给你辨认。
- 那天晚上在 x_nav 界面里输入过哪些地图名？手动删除过地图吗（比如 111）？
- AGX 能不能用台式电源单独供电，还是只能接狗的 12 V？谁负责狗上的 AGX 供电和接线，能不能检查 12.2 V 输入和接头？
- 和其他队约定的时间段是什么？他们能保证我们 armed 期间不跑原生导航任务吗？armed 测试时有第二个人在场吗？需要一人拿遥控器、一人看电脑。
- runner（nav/s10_auto_nav，是队里 s10_auto_nav 的拷贝）的修改：可以改上游 wt-rl-nav 再用 sync_auto_nav.sh 同步回来吗？还是必须只在我们自己的 wrapper 和 config 里改？
- 每次交还狗时：106/103 上的 SSH key 是保留，还是执行 unkeys？
- TRACK 的行为，我们的建议是首次上机保持零速、可恢复的停止。原因是没有 map_surface 时 runner 自己不做障碍检查。等以后从保存的 global_map.pcd 生成 map_surface，再决定要不要“低速推过去”。你同意吗？

## 被否决的想法（及原因）

- Using either recording directly as 'the map' for an on-robot test. No saved x_nav map exists. Poses recorded during mapping must not become map-frame routes (design doc :108). The old trail.csv frame does not match a rebuilt map.
- Using v5_5 for any on-robot work now. Its content is unknown, it is probably outdoors, its trail.csv is empty, it is twice as long, and stairs are forbidden while SYS_RESET_N is unexplained.
- Plan 3's gate.sh and config/gates JSON mechanism.
  - It deadlocks as written: arming requires G7, but G7 is earned by the stage that needs arming.
  - A self-written file is not an interlock.
  - It blocks zero-risk shadow runs behind days of tooling.
  - It conflicts with the user's 'implement then verify' preference.
  It is replaced by a printed checklist plus real interlocks: per-stage control configs, shadow refusing an armed control node, --enable-motion, and --i-am-on-site.
- The static soak as a pass/fail gate, and Plan 2's claim that 'the rebuild doubles as a soak'.
  - Both losses happened while walking, and the idle reset rate is unknown.
  - A clean lying soak proves little and can block forever on an intermittent fault.
  - The replay is confounded: 0.5x rate, gateway stopped, SSD read rather than write, no vibration.
  - The power logger stays mandatory as instrumentation.
- Giving TRACK a positive speed ('push on at 0.15 m/s') for the first robot runs.
  - Without map_surface.npz the runner has no obstacle check of its own.
  - --straight routes enter TRACK immediately when blocked.
  - The dog would therefore walk into whatever blocks the route.
- Setting probe speed with 'max_vx in a nav probe yaml' (Plans 1 and 3). nav.yaml has no such key, NavCore silently ignores unknown keys, and start_nav.sh could not load another yaml. The probe uses --flat-speed plus the hard clamp in control_probe.yaml.
- The 0.3 m probe route with radius 0.20. It ends after 0.10 m in 1 s (simulated) and cannot meet its own criteria. It is replaced by LENGTH 0.6, radius 0.10: 0.50 m in 5 s, simulated.
- 'End point within 0.20 m' as the closed-loop pass criterion. It is true by construction, because the run ends on entering the gate radius. It is replaced by cross-track p95 and max, heading error at stop, overshoot, and tape against odom.
- Percentage replay thresholds as gates: WALK >= 95%, BLOCKED <= 2%, false positives < 0.5-1%. They are guesses for a first run on real data and invite an open-loop tuning spiral. Only the binary plumbing checks gate. The rates are a triage report time-boxed to one day.
- Plan 1's claim that every BLOCKED in v4_room is a false positive by construction. Walls, furniture and the operator can legitimately block a 0.5-0.8 m corridor. 'Flat room' is inferred only from the session name until Stage 0 verifies it.
- A perturbed replay with a constant +/-0.3 m pose offset on the main replay state. Against 0.30 m gates it misses every gate at 0.31 m and latches HOLD even with stall_time 1e9 (simulated). Levelling the real cloud with a false pose also makes the perception events meaningless. It is replaced by a stateless single-step sign check on a deepcopy with a flat synthetic observation.
- The rebuild command as written in all three plans: `rosbag play ... --topics /LIDAR/POINTS /IMU <bags>`. --topics swallows the bag names and rosbag exits at once. The bags go first and --topics goes last.
- Rendering PNGs inside the s10-ros1-nav-test container. The image has no matplotlib, PIL or cv2, and nothing can be installed with --network none. Docker writes npz and JSON, and the Mac host renders.
- Editing nav/route_runner.py locally (Plan 3). The file is actually nav/s10_auto_nav/rl_nav/route_runner.py, and sync_auto_nav.sh wipes local edits with rm -rf. Runner changes go upstream and come back through the sync.
- Plan 1 Stage 6's third on-AGX replay with our shadow node, done to measure CPU cost. That is about 40 min of play time on a shared dog. AGX CPU cost is measured in the live shadow session instead.
- Plan 2's safety argument that 'with the gateway stopped there is no /MOTION_INFO'. It is wrong, because the control node reads ROS 2 directly. The rebuild script checks enable_motion false instead.
- Hand-typing X Y Z YAW for the first armed route. A degrees/radians slip or a stale pose would give a 0.5 rad/s pivot and a walk in the wrong direction. It is replaced by straight_from_live_pose.py plus a dry-run rehearsal of every new route.
- Writing rebuilt_odom or other session logs to the USB SSD during the rebuild or during motion. They are recorded to the eMMC. The SSD stays unplugged during motion.
- Trying /use_sim_time or --clock on the shared x_nav master during the first rebuild attempts. It could break the running rospy controller, w_nav and our own nodes. It is allowed only in a later dedicated session with all our nodes stopped.
- Setting an x_nav container CPU limit during a session. It requires recreating the vendor container, which is a separate user decision outside any test session.

## 没有定论的分歧

- Which plan is the base.
  - The technical and feasibility critiques chose Plan 2. The safety critique chose Plan 3.
  - This plan merges them: Plan 2's chain, plus Plan 3's safety content, minus gate.sh.
  - The dispute about the gate mechanism remains. The safety critique wanted the gates kept with corrected wiring, as a checklist aid. The feasibility critique wanted them dropped.
  - I chose a printed checklist plus in-code interlocks. If the user prefers mechanical gates, the corrected wiring is: arming requires the dry-run session gate; non-shadow nav requires the zero-velocity gate; routes longer than the probe or faster than 0.10 m/s require the probe and kill -9 gate. Gate files would live in ~/ros1_gateway/run/gates, which deploy excludes.
- Weight of the AGX soak.
  - The safety critique wants soak A (no SSD) and then soak B (SSD) before the rebuild and before any walking session.
  - The feasibility critique says a lying soak proves little and must not gate.
  - I made the logger mandatory and the soak non-gating. The rebuild runs with the session down, so a reset costs only time.
  - This cannot be resolved from files. Only the power log from real sessions will show whether idle resets occur.
- First rebuild attempt rate.
  - The x_nav investigation and all three plans say -r 0.5.
  - The feasibility critique says 1.0 first, because that is the only condition x_slam has handled on this AGX, and a wall-clock data-rate check in the closed binary cannot be excluded.
  - I chose 1.0 first, then 0.5. Only a trial or the vendor can decide.
- Bag-derived thresholds for the rebuild trajectory match.
  - Plan 3 used RMS < 0.15 m, and 0.10 m for the localisation replay.
  - Plan 2 used 0.30 m and 0.20 m.
  - The feasibility critique says the tighter limits exceed SLAM repeatability.
  - I use 0.30 m as a proxy only. Live relocalisation with the dog is the acceptance test. No data yet supports either number.
- Whether 'navigation mode' for /NAV_CMD (sdk_guide_v101.txt:2505) means ASDU ControlUsageMode 1, or RL(17) plus gait 0x3002. The texts do not say. The Stage 5 probe decides.
- Robot behaviour on abrupt /NAV_CMD loss, and whether the remote sticks work in ASDU mode 1.
  - Neither SDK text documents this.
  - The 0.5 s timeout exists only in our mock robot.
  - Stage 5 measures it.
  - If the robot does not self-stop, no plan variant offers a mitigation beyond asking the vendor.
- Whether x_slam initialises acceptably from a bag that starts with the dog already mapping and possibly moving.
  - Plan 3 made a stationary start a hard precondition.
  - Plan 2 uses a -s first-standstill offset, which I adopted.
  - Whether a mid-walk standstill is enough for gravity and IMU initialisation in this closed binary is unknown.
- Order of 设置位姿 against the data flow in localisation mode: set the pose first and then play, or play first and then set the pose. The manual does not say, and the binary is closed. Stage 3R tries both.
- Free-space requirement for the forward kill -9 test.
  - The plans said at least 3 m. The safety critique said at least 5 m.
  - I require 5 m ahead of the start.
  - If the chosen room cannot offer 5 m, the user must decide between another location and accepting 3 m at 0.10 m/s with the operator on the remote.
- Whether bag-derived H (remote-driven gait) equals the standing height in navigation gait 0x3002. Nobody has data.
  - The scan-band decision is provisional after Stage 0 and final after the Stage 4 measurement.
  - The critiques differ on how much weight the bag value deserves.

## 审查中的阻断级问题

- （SAFETY AND SHARED-ROBOT RULES …）方案 1, 2, 3：The speed caps the plans rely on (0.10 m/s probe, 0.20 m/s first routes, yaw 0.20 rad/s) are not enforced by the only authoritative limiter. s10_ros1_control keeps clamping at 0.30 / 0.10 / 0.50 in every plan. The caps exist only as a route speed_limit or a 'probe yaml', and neither start script can load an alternative config. The runner's non-WALK modes also ignore the route speed_limit.

- （SAFETY AND SHARED-ROBOT RULES …）方案 1, 2, 3：While the control node is armed, the x_nav web UI is a second, unauthenticated motion console on the robot network, and every plan handles it only by procedure ('keep the UI away from others'). It can do more than stand/lie: 'Nav continue' from the page un-latches an operator's 'Nav stop', and 'source x_nav' hands velocity to w_nav's /cmd_vel. Other teams use the dog's network (their hotspot units are on 103).
