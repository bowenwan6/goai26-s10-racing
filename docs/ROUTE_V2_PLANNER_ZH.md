# route_v2 导航规划层（S10）

状态：**仅离线/单元/运动学仿真验证**，未上机、未接 MuJoCo、未经实云回放。默认行为不变：不给 `route_v2_path` 时，follower 与 native runtime 走原来的 Course + LocalPlanner + 0.35 m 走廊逻辑。

## 1. 解决的问题

| 旧行为 | route_v2 行为 |
|---|---|
| `Course.lookahead_point` 的 carrot 在“机器人→当前 WP”直线上，绕障后直切下一 WP | carrot 在**示教中心线**（`segments[].centerline`）的横向偏移路径上，绕障后回到中心线 |
| `LocalPlanner` 只按与目标方位的偏差打分，单帧、全堵无搜索 | Frenet 横向偏移采样 + 足迹扫掠碰撞 + 横向代价；全堵时有界 A*；A* 失败即 STOP |
| native 距当前直线段 >0.35 m 即 fault | 用该段 `corridor_half_width + 0.10 m`（相对中心线，含 z 楼层校验）；旧 draft 仍为 0.35 m 原规则 |
| gait 来自 WP `kind` | gait 来自 route_v2 段（`kind` = 驶入该 WP 的段的 gait），settle→switch→confirm 不变 |

## 2. 模块

| 文件 | 内容 |
|---|---|
| `src/s10_auto_nav/s10_auto_nav/route_v2.py` | 读取/校验 `route_v2.json`（忽略未知字段）；`RoutePath`（水平弧长 s、投影 `(s,d,段,切向)`、按 s 查 gait/限速/走廊）；`RouteTracker`（只在上次 s 附近窗口内投影，z 门限 0.6 m，防止吸附到平行腿/另一楼层）；转换为旧 `Course`/course YAML/native 点表；`TerrainCrossCheck`（只 warn/hold） |
| `src/s10_auto_nav/s10_auto_nav/route_planner.py` | 局部栅格（高度图+扫描，UNKNOWN/FREE/SCAN_CLEAR/OBSTACLE，可选多帧融合）与 `RoutePlanner` |
| `src/s10_auto_nav/s10_auto_nav/route_follower.py` | 无 ROS 的 follower 核心 `RouteFollowerCore.step()`，仿真/节点共用 |
| `src/s10_auto_nav/s10_auto_nav/route_sim.py` | 2.5D 运动学沙盒（盒子障碍、合成 13×9 高度图、72 bin 扫描），测试与 harness 冒烟用 |
| `follower_node.py` | 新参数 `route_v2_path`（默认空）、`route_v2_body_z_offset`、`route_v2_fuse_frames`、`route_v2_lookahead`；非空时 `_control_step` 交给核心，Course 仍负责有序 XYZ 到点 |
| `native_transfer/route_v2_bridge.py` | `load_route`（旧 draft 或 route_v2）、`CorridorMonitor`、`TerrainMonitor`、`native_router_for` |
| `native_transfer/router.py` | 新增可选 `speed_caps`（每目标限速，只能降低 gait 上限；默认 None 行为不变） |
| `native_transfer/runtime.py` | `--route` 可直接给 route_v2；自动设置 follower 的 `route_v2_path`；走廊与地形交叉检查换成 bridge |
| `src/s10_auto_nav/s10_auto_nav/waypoints.py` | `Waypoint` 可选 `radius`/`height_tolerance`（YAML `radius_xy`/`tol_z`），缺省即旧行为 |

## 3. 规划算法（每 tick）

1. **有序到点**：只检查当前目标，XY ≤ `radius_xy` 且 |z_base − (z_ground+offset)| ≤ `tol_z` 才推进，每 tick 至多一个。
2. **投影**：`RouteTracker` 在 `[s−1.0, s+2.5]`（不越过未到达目标 s+1.0）内投影；只有距离近似相等的**不同局部极小**（如 switchback 两腿、内角两侧）才按与上次 s 的接近程度裁决。
3. **候选**：目标偏移 `d_t ∈ linspace(−W, W, 17)`（0.1 m 间隔）（`allow_detour=false` 时只有 0）；五次多项式从当前 d 过渡到 `d_t`，长度 `max(1.0, |Δd|/0.5)`，水平视界 3.5 m，不越过未到达的 WP（接近 WP 时偏移在 1 m 内回零并以 WP 点为终点）。已选中的过渡**锚定在选中位置**，由纯追踪跟随（每 tick 重新以机器人为起点、或以实测航向为初始斜率都会出问题：前者永不产生横移，后者与纯追踪形成正反馈）。
4. **碰撞**：矩形足迹 0.9×0.5 m + 0.05 m 余量，按纯追踪实际会采用的航向（指向前方 lookahead 点）扫掠；起点航向差 >10°（与控制器原地转向阈值一致）时扫掠整个转动角。OBSTACLE 与 UNKNOWN 都阻挡，例外：机器人当前足迹内豁免；未来足迹**后半部**落在当前车头之前的 UNKNOWN 不阻挡（车尾只跟随车身已覆盖/旁边的地面）；不许绕行的 stairs 段沿示教线 UNKNOWN 不阻挡（见 §7）。候选有效条件：前 `commit_length`（flat 1.0 m / stairs 0.4 m）无碰撞。
5. **代价**：`w_obs·proximity(足迹到障碍/未知) + w_free·(1−free/H) + w_lat·|d_t| + w_smooth·|d_t−prev| − w_progress·free/H + 换边惩罚`，默认 1.0/5.0/1.0/0.3/0.5/1.0；`w_lat > w_smooth` 保证无障碍时 d=0 胜出并回线；上一目标仍有效且代价差 < 0.3 时保持（防止原地转向引起的来回切换）。
6. **兜底**：全部候选无效且允许绕行 → 6×6 m 窗口 8 邻接 A*（圆盘膨胀 0.3 m、走廊外禁止、横向偏移计入代价、目标为前方 5→3 m 中心线点；若都在障碍阴影 UNKNOWN 中则取可达**已知**格中 s 进展最大者，至少 0.8 m）；再用矩形足迹复检。若所有候选都只是被 UNKNOWN 挡住、需要 >10° 转向且原地转向扫掠无碰撞 → `ALIGN`（零前进，原地转向，下一帧看清弯道后重新规划）。失败 → `BLOCKED` + 原因。`allow_detour=false` → 直接 `BLOCKED(centerline_blocked_detour_forbidden)`。从不规划倒车，输出 vx ≥ 0。
7. **速度**：`min(控制器上限, 段 speed_limit) × speed_scale`；speed_scale 随“到第一个真实障碍的距离”递减（下限 0.3），未知区域只要求超过 commit 长度（再往前靠机器人移动后继续观测）。
8. **看门狗**：处于运动状态但 s 20 s 未前进 0.1 m → `BLOCKED(no_route_progress)`；native router 自身 15 s 无进展 fault 仍在。

### 局部栅格

- 世界对齐、以机器人为中心、0.1 m、8×8 m，格点对齐世界网格（多帧融合时每帧栅格化结果缓存复用）。
- 高度图：孤立无效格（8 邻域中 ≥6 个有效且高差 ≤ 当前台阶上限）用邻格中位数补；车头前 0.9 m 内无效格 = UNKNOWN（即使扫描线扫过），更远的无效格与 ROI 外一样按扫描处理；相邻格高差 > `max_step_flat` 0.12 m / `max_step_stairs` 0.22 m / 不许绕行 stairs 段 `max_step_taught` 0.35 m 时，标记**离机器人脚下地面更远**的那一格（箱顶、落差下沿）。
- 机器人当前足迹外扩 0.15 m 置为 FREE（Nav2 式 footprint clearing：车身下方与紧邻处被自身遮挡）。
- 扫描：NaN bin 不产生任何信息；有限距离的端点（≤6 m）标 OBSTACLE，但只标 bin 中心射线两侧 ±0.06 m（子射线仍全部用于标记空旷；原先整格 5° 都标会把示教线旁 0.39 m 的立柱抹进路线）（保守：端点可能只是地面观测尽头），端点后为 UNKNOWN；+inf 视作 4 m 内无回波。stairs 段忽略扫描（台阶立面在机身高度带内），ROI 外即 UNKNOWN。
- 融合（`fuse_frames>1`，默认 1）：新帧对其“已知”格覆盖旧帧；新帧 UNKNOWN 不抹掉旧帧 FREE（如机身下方自遮挡）；SCAN_CLEAR 只能升级 UNKNOWN；超过 `max_age` 2 s 过期；z 跳变 >0.5 m 清空。

### 地形交叉检查（只 warn/hold，从不切 gait）

- flat 段：俯仰 ≥12° 持续 0.5 s → **HOLD**（零速）；高度图呈台阶/坡（rise≥0.12 m 且跨度≥80%，或拟合坡度≥12°）→ WARN。
- stairs 段：俯仰<4° 且高度图平坦 → WARN。
- runtime 与 follower 核心各自运行；HOLD 持续会触发 router 的 15 s 无进展 fault，需要人工处理。

## 4. 仿真/harness 入口（无 ROS）

```python
from s10_auto_nav.route_follower import RouteFollowerCore
core = RouteFollowerCore.from_file("route_v2.json", body_z_offset=0.42, native=True)
out = core.step(t, (x, y, z_base, yaw), height13x9, valid13x9, scan72, scan_angles,
                pitch=pitch, roll=roll, current_gait="flat")   # 或 0x3002/0x3003/None
vx, vy, wz = out.command            # 机体系 m/s, rad/s；非运动状态时为 0
out.gait_request, out.gait_code     # 驶入当前目标的段的 gait
out.status, out.reason              # RUNNING/DETOUR/ASTAR/GATE/ALIGN/BLOCKED/OFF_CORRIDOR/
                                    # HOLD_TERRAIN/WAIT_GAIT/STALE_INPUT/DONE
out.target_id, out.segment_id, out.s, out.d, out.reached, out.plan.path, out.plan.carrot
```

- `height13x9`：机器人 yaw 系（x 前 y 左）、相对机身参考点的高度，X-major，与 `real_transfer.geometry.height_grid` 相同；`valid13x9` 为有效掩码（未知可为任意值）。
- `scan72`：`conservative_scan` 语义（NaN=未知）；`scan_angles` 省略时用 bin 中心 `−π+(i+0.5)·5°`。
- 没有新观测的 tick 传 `None`；0.5 s 无新观测 → `STALE_INPUT`。
- `current_gait` 不等于所需 gait → `WAIT_GAIT`（harness 应模拟 router：停稳→切换→确认后再给速度）。
- 参考实现：`route_sim.run_closed_loop(core, KinematicBody(...), duration=..)`。

## 5. native 用法

`native_transfer/runtime.py --route <route_v2.json>`（旧 draft 仍可用）。runtime 自动：点表与 gait 取自路线段；每目标限速 = 段 `speed_limit`（只降不升）；follower 设置 `route_v2_path` 与 `route_v2_body_z_offset`；扫描角度在 route_v2 模式下取 bin 中心；走廊 = 段宽 + 0.10 m 且 z 必须匹配本层；地形交叉检查结果写入日志 `sensors.terrain_check`，HOLD 时置零指令。**感知准入门槛未放宽**（仍要求 117 格全有效且 72 bin 全已知才交给 follower）。

## 6. 验证情况

已验证（本机，`python3 -m pytest`）：
- 单元：投影（直线/直角/switchback/两层楼）、跟踪器连续性、合同校验、转换、逐 WP 半径；规划器（无障碍 d=0、线上障碍在走廊内绕行、不许绕行则停、全堵走 A*、A* 失败停、UNKNOWN 阻挡、全未知不“发明”空地、原地转向扫掠、栅格构建/融合）；router 按段 gait 与 settle→switch→confirm、限速只降不升、段宽走廊、层不符拒绝、旧 0.35 m 规则逐字保持；follower 节点分支（ROS 以测试内 stub 代替）。
- 运动学闭环（独轮车+小横移，native 增益 0.2 m/s、10° 原地转向阈值）：直线上 0.4 m 箱体，绕行最大 |d|≈0.77 m < 0.8 m 走廊，箱后回线 |d|<0.1 m，全部 WP 到达，无倒车，旋转矩形机身与箱体无重叠；另测 8 种箱体位置/尺寸/偏置组合：6 种绕行并回线；2 种（起步正前方 1.5 m 的箱体、0.7 m 宽箱体使 0.8 m 走廊只剩约 0.15 m 余量）安全停车 `BLOCKED(astar_path_fails_footprint_check)`；8 种均无机身接触、无倒车。
- 真实路线 `tools/wp_match/out/route_v2.json`（30 WP / 29 段 / 256.6 m / z −0.46→5.88 m）：可加载；每个中心线点投影回自身；带 5 cm 噪声与整段走廊宽度正弦偏移的连续跟踪无跳变、无层错配（路线自身最近的非相邻段水平距离 3.6 m）。测试通过 `S10_ROUTE_V2` 环境变量或默认路径定位，缺失时跳过。

未验证 / 已知局限：
- 未在 MuJoCo、实云回放或实机上运行；参数（权重、commit 长度、步高阈值）只在合成场景调过。
- 单帧 13×9 高度图只覆盖 1.8×1.2 m：ROI 外只有扫描“无机身高度回波”证据（SCAN_CLEAR），**看不到 ROI 外的负障碍（坑/落差）**；`require_ground_within` 可强制近距离必须是高度图验证过的地面，但单帧下会阻止几乎所有绕行，建议配合 `fuse_frames>1` 并用实云验证后再开。
- stairs 段只用高度图，视野约 0.75 m，速度随之保守；楼梯踏步 >0.22 m/0.15 m 格会被判为障碍而停车。
- 障碍面距车头 <约 1 m 才出现时（例如起步正前方 1.5 m 的箱体），或障碍几乎占满走廊时可能无可行候选且 A* 复检失败 → 安全停车而非绕行。
- A* 以圆盘近似机身再用矩形复检，窄通道可能保守失败。
- 扫描端点一律视为障碍（conservative_scan 无法区分“观测尽头”和“真实障碍”），稀疏地面会造成假障碍/提前绕行或停车。
- 地形交叉检查阈值未用实数据标定；高度图证据只报警不拦截。
- 回线后的纯追踪在 WP 处仍会按 0.5 m 刹车距离减速（与旧 Course 一致，carrot 不越过未到达 WP）。

## 7. 全场仿真调试（2026-09-19）

用 `sim_full_course`（v3 高度场 + 合成雷达，运动学机器人）从 Start 开始跑，每次停在第一个误报 BLOCKED 处定位原因、修一处、再跑。按出现顺序：

| 位置 | 原因 | 修改 | 代价 / 注意 |
|---|---|---|---|
| 开阔沥青 | 单帧 ~5 % 零散无效高度格全是 UNKNOWN，所有候选不通 | 孤立空洞填补 | 成片空洞（可能是坑）仍为 UNKNOWN |
| Start | 车身下/旁被自身遮挡的格 | 足迹 +0.15 m 置 FREE | — |
| Start | 转向时车尾角扫到 ROI 边缘 UNKNOWN | 车尾跟随区 UNKNOWN 不阻挡 | OBSTACLE 仍阻挡 |
| WP02→03 窄道（立柱 0.39 m / 绿篱 0.6 m） | 扫描端点抹宽 + 候选 0.2 m 间隔错过唯一缝隙 + 5° 就按原地转向扫掠 | 端点 ±0.06 m；17 个偏移；原地转向阈值 10° | **现场必查**：地图里缝隙只有 ~1 m |
| 各处 | ROI 最远两排稀疏无效，比 ROI 外还“难走” | 0.9 m 外无效格按扫描处理 | 每 tick 重判，近了仍严格 |
| WP03 楼梯脚急弯 | stairs 忽略扫描，拐弯后 ROI 外全 UNKNOWN | `ALIGN` 原地转向；stairs commit 0.4 m | — |
| B 楼梯 / 乱石 / 石笼 | 立面格无效、石块台阶 >0.22 m 被当障碍 | 不许绕行的 stairs 段：沿示教线 UNKNOWN 不阻挡；台阶上限 0.35 m | **安全取舍**：这类段上新出现的、矮于 0.35 m 的物体看不见；更高的（人）仍会停 |

仿真侧（不影响实机代码）：v3 高度场 3×3 去尖刺（地面层 0.3 m 厚造成沥青上 8–13 cm 尖刺）；示教线 0.45 m 内的静态障碍格视为建图时的临时物（人）并清除；仿真适配器启用 5 帧融合。

**基准（2026-09-19，`runs_final`，运动学仿真，非实机）**

| 运行 | 结果 |
|---|---|
| 全场，旧纯追踪参考控制器（不避障） | **30/30 WP，256.5 m，0 碰撞**：仿真地形修正后示教线本身可通行 |
| 全场，route_v2 规划器 | 24/30 WP，218.5/256.6 m，1 次碰撞记录；停在 WP25 前 `gate_approach_blocked` |
| 平地段 s160–185 无障碍 / 箱体 / 偏置圆柱 | 都到 24.7/25 m，绕行最大 0.62/0.83/0.76 m、与障碍最小间距 0.18/0.57 m、0 碰撞；**都卡在终点前 0.3 m** `gate_approach_blocked` |
| 平地段 slalom / 堵死走廊 | 12.1 m / 10.9 m 处安全停车（堵死走廊为预期行为，间距 0.89 m） |
| 平地段 0.12 m 路沿 | 4 次碰撞：0.12 m 路沿低于扫描高度带、在高度图步高阈值边缘，未可靠识别 |

已知未解决：
1. **到点前最后一段**：机器人在航点前仍横向偏 ~0.4 m 时，`min_horizon` 内直接对准航点的直线检查失败（需 ~50° 转向）。应让 gate 前的回线更早完成，或到点检查改为经过航点的走廊判定。
2. **0.12 m 路沿**：需要高度图多帧融合后再标定 `max_step_flat`，或用先验地图变化检测。

**调试中途曾停在**：WP23→WP24 沥青段一个 ~0.3×0.2 m、比四周低 10–16 cm 的凹坑（可能是排水口或地图噪声），平地台阶上限 0.12 m 判为障碍，而建图时机器人直接压过。

**下一步建议（需要决定）**：用先验地图做变化检测（VT&R 思路）——对示教走廊预存地面高度（`match_wps.py` 已算出），把“观测高度 − 先验高度”的突变而不是原始高度突变当作障碍。这样示教时走过的台阶、石块、凹坑都不再误报，而新出现的箱子、人、路障都会被发现；也能撤回上表中 0.35 m 的安全取舍。
