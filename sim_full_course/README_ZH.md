# S10 全程仿真地形与规划级测试台（sim_full_course）

地图 `0914_fr_v3-20260914-142008`，坐标系 `map`（v3 原始），单位米。**仅用于仿真/规划验证，不连接机器人。**

## 组成

| 模块 | 作用 |
|---|---|
| `build_terrain.py` | 从 v3 全场点云沿建图轨迹 10 m 走廊提取 0.10 m 2.5D 地面高度场、障碍层（地面上 0.1–1.2 m）、未知掩码、多层标记 |
| `build_mujoco_scene.py` | 生成全程 `hfield` + S10 机器人（取自既有包）MuJoCo 场景并校验加载 |
| `route.py` | route_v2 读取/几何（弧长 s、投影、横向偏差）；由建图轨迹生成的**占位路线**（PLACEHOLDER） |
| `terrain.py` `robot.py` | 地形查询；运动学机器人（原生限速＋加速度限幅、换步态需停车） |
| `sensors.py` | 由地形射线投射生成点云 → 调用**真实** `points_in_yaw_frame` / `height_grid` / `conservative_scan` |
| `obstacles.py` `scenarios.py` | 按路线弧长 s＋横向偏移注入箱体/圆柱；封堵走廊等场景 |
| `controller.py` | 控制器接口＋纯追踪参考控制器（不绕障） |
| `harness.py` | 运行循环、指标、CSV/JSON/PNG 输出 |
| `route_check.py` | 不仿真，直接检查路线 footprint 与障碍层/台阶/未知区 |

## 运行

```bash
V=<home>/Documents/ChatGPT/GOAI/wt-nav-sim-artifacts/.venv/bin/python   # Python 3.12, mujoco 3.13.0, numpy 2.5.3
cd <worktree>
$V -m sim_full_course.build_terrain          # ~15 s → artifacts/terrain/*.npz, *.png
$V -m sim_full_course.build_mujoco_scene     # → artifacts/mujoco/scene_full_course*.xml
$V -m sim_full_course.route                  # 重新生成占位路线（已提交）
$V -m sim_full_course.harness                # 全程 nominal（默认用匹配好的 route_v2.json）
$V -m sim_full_course.harness --scenario detour_box --s0 160 --s1 185 --obstacle-s 172
$V -m sim_full_course.harness --controller my_pkg.mod:MyController --collision-mode record
$V -m sim_full_course.route_check            # 路线静态检查
$V -m pytest -q sim_full_course/tests
```

- 路线默认 `GOAI/tools/wp_match/out/route_v2.json`（环境变量 `NAV_SIM_ROUTE` 可改），不存在时退回 `routes/placeholder_route_v2.json`。
- 产物目录默认 `GOAI/wt-nav-sim-artifacts/`（`NAV_SIM_ARTIFACTS` 可改）；每次运行输出 `runs/<name>/{log.csv, summary.json, topview.png}`。
- 场景：`nominal`、`detour_box`（中心线上 0.5 m 箱）、`offset_cylinder`（左 0.30 m 立柱）、`slalom`（两箱交错）、`blocked_corridor`（整条走廊封死，应停车报告）、`low_curb`（0.12 m 横向路缘）。`--obstacle-s` 用全路线弧长。
- `--collision-mode block`（默认，撞入障碍的移动被拒绝、机器人停下）/ `record`（只计数，便于跑完全程统计）。

## 控制器接口

```python
controller.reset(route)                                # route: sim_full_course.route.Route
controller.on_feedback({"gait": ..., "gait_switching": bool})   # 可选；每拍先调用
vx, vy, yaw_rate, gait_request, status = controller.step(t, pose_xyzyaw, height_grid, valid_mask, scan)
```

- `pose_xyzyaw`：map 系 (x, y, z_base, yaw)，可带位姿噪声；`height_grid` (13, 9)：行 = X −0.6…1.2，列 = Y −0.6…0.6（0.15 m），z 相对机体原点（平地约 −0.43），无效格为 −1；`valid_mask` 同形状 bool；`scan` (72,)：bin i 覆盖 [−π + 5°·i, −π + 5°·(i+1))，NaN = 未知。
- 测试台把命令限制在原生范围：flat vx ≤ 0.20、stairs vx ≤ 0.15、|vy| ≤ 0.05、|yaw_rate| ≤ 0.20；加速度 0.2 m/s²、0.1 m/s²、0.3 rad/s²；不许倒车（vx ≥ 0）；另受当前路段 `speed_limit` 限制。换步态：请求后强制停车，静止 3 s 完成。
- `status == "DONE"/"ABORT"` 结束运行。

## 指标（summary.json）

按顺序到达的 WP（XY 0.2 m、Z 0.2 m；越过 WP 1 m 仍未到达记为 missed）、最大/平均横向偏差、走廊越界拍数、到静态障碍层/注入障碍的最小间隙（footprint 0.9×0.5 m）、碰撞次数、台阶超限次数（footprint 内相邻格高差 > flat 0.10 / stairs 0.22 m）、停滞（< 0.02 m/s 超过 4 s）、换步态次数与位置、步态与路段不符拍数、高度栅格有效率、scan 已知率、`runtime_would_publish_frac`（`runtime.py` 要求整格全有效且 scan 全已知才发布）。

## 模拟了什么 / 没有模拟什么

- 地面：每格取“最低稠密簇”的上层（v3 点云地面有 0.1–0.4 m 的多次扫描叠层），5×5 中值去低尖峰；沿建图轨迹与机器人高度（base − 0.43）中位差约 −0.01 m、90% 在 ±0.05 m 内。未知格从不当作自由空间；`ground_filled` 只为高度场取值。
- 障碍层：相对 3×3 邻域最高地面 +0.1 m 到地面 +1.2 m 的点（≥ 2 点）。建图机器人车身扫过的区域被清空（那里是操作员/行人/自身回波）。它包含灌木、高草、扶手及 0.7–1.2 m 的悬垂物，**偏保守**。
- 多层标记（`multi_level`）：地面上方 0.3–2.0 m 有间隙后的第二层，或最低层与机器人实际行驶高度不符（此时改用行驶高度层）。仅作标记，不参与碰撞。
- 传感器：射线投射含地形/障碍遮挡、10% 随机丢点、1 cm 测距噪声；近场点均匀覆盖高度栅格区域（真实安装的近身覆盖尚未验证），可选车身自遮挡 `--self-occlusion`；外参假设为 base 前 0.20 m、上 0.12 m、水平。
- 机器人：纯运动学，z = footprint 地面拟合平面 + 0.43 m，pitch/roll 由平面得到。**没有**动力学、腿/轮接触、打滑、楼梯攀爬成败、原生步态内部行为、时延、里程计漂移（只有可选高斯位姿噪声）、动态障碍。
- MuJoCo 全程场景只是 0.10 m 粗高度场（可加障碍层），机器人零力矩会塌下；楼梯接触物理仍以既有 Start+B 精细 mesh 场景（`deliverables/S10_v3_Map_MuJoCo_20260916`）为准。
- 占位路线：建图关键帧反向时间 1789368238–1789368895，WP 等距约 9 m，楼梯段由坡度/台阶启发式标注，**不是**照片匹配的 WP。
