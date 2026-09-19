# S10 导航规划重构：以走过的路线作为全局策略

整理日期：**2026-09-18（北京时间）**。对象：`s10_auto_nav` 的 follower / local planner / terrain / router，以及 `native_transfer` 的 NativeGaitRouter 与路线草稿。

本文是**设计建议**，不是已实现或已验收的功能。所有带数字的结论标注了来源：**[实测]** 表示本次在本地数据上直接计算得出，**[代码]** 表示从源码读出，**[文献]** 表示来自公开论文或开源实现。没有任何一条来自实机新实验——本次没有连接机器人。

配套：[导航/建图/感知/Router 设计参考](S10_NAVIGATION_MAPPING_SENSOR_DESIGN_REFERENCE_ZH.md)（现状基线）、[文件索引](S10_NAVIGATION_SOURCE_INDEX_ZH.md)。

---

## 0. 先看结论

1. **你的想法是对的，而且比你以为的更接近已完成。** 现在 Start→B 路线里 B 段的 XY **已经就是走过的轨迹**——逐点核对，9 个航点分别精确落在关键帧 16、24、28、32、36、40、44、48、52 上，间隔正好 4 帧，XY 距离 0.000 m。**[实测]** 真正被丢掉的是 Z：它被换成了重建模型的地面高度，于是引入了一个至今为 null 的未知量 `body_z_offset`。

2. **所以要做的不是"改用走过的路线"，而是"不要再把走过的路线洗掉"。** 轨迹本身就带着机身参考点的 Z、朝向、坡度、以及机器人当时实际停在哪。把这些一起保留下来，`body_z_offset` 这个未知量直接消失。**[实测]** 现在轨迹 Z 与草稿地面 Z 的差是 +0.441 m，标准差 0.057 m，范围 0.349–0.566 m——**这 0.217 m 的散布已经超过 Z 容差 0.20 m 的整个预算**。

3. **有一个必须避开的陷阱：1042 m 的建图轨迹不是路线。** 它是一次反复走遍全场的采集，自交严重。**[实测]** 全程 93.6% 的位姿在 1.0 m 内存在另一条相距 5 m 以上的通过（即使加上高度判据也一样），最多有 5 条不同的通过重叠。而 Start→B 这一段的自交是 **0.0%**。所以必须先从建图轨迹里**抽出一条具体的遍历**作为路线，再按弧长单调推进，不能用"最近点"去定位自己在路线上的位置。

4. **现在真正的设计问题不在算法，而在于 route 知识存放的位置。** `nav.yaml` 用 8 组按航点编号索引的数组承载路线知识，follower_node.py 里有 193 处引用。**[代码]** 而 `native_transfer/runtime.py` 上真机时把这 8 组**全部设成 `[-1]` 关掉了**——也就是说，今天跑在真机上的 follower 是一个被摘掉全部路线知识的通用内核。这些知识必须以某种方式回来，而它们显然不该以"航点 28 要助跑"的形式回来。

5. **建议的分工**：路线层承载"这条路怎么走"（来自被教过的轨迹＋标注），局部层只负责"眼前有没有新东西挡路"（在路线走廊内做有界搜索），router 层不变，只是改成读路线标注而不是读航点编号。

6. **这不是一个新想法，它有名字、有综述、有腿式机器人上的验证。** 文献里叫 **teach-and-repeat**，2026 年的综述把它的地图形式命名为 **topometric**（局部子图链＋相对定位）[8.1]。你提的"局部用简单算法"在 Barfoot 组的两篇里是已发表且经 5 km 野外验证的做法：**在贴着被教路径的曲线坐标系里规划、显式惩罚横向偏离、把障碍感知化简为相对 teach 阶段的变化检测** [8.2]。腿式机器人上的证据是 Mattamala 等在 ANYmal C 上的工作：同样的分层，局部反应层 10 Hz、CPU 上 **< 2 ms** [8.1]。ROS 2 的 Nav2 Route Server 把这套东西做成了产品，它官方文档的七种"实用架构"里，**第 6 种就是 teach-and-repeat，第 7 种就是多层楼** [8.4]。

---

## 1. 现状诊断：route 知识存放在错误的位置

### 1.1 八组按航点编号索引的数组

`src/s10_bringup/config/nav.yaml` **[代码]**：

| 参数 | 值 | follower_node.py 引用数 |
|---|---|---:|
| `waypoint_speed_limit_indices` | `[16, 24, 25, 26, 27, 28, 29, 31, 32]` | 7 |
| `fast_flat_waypoints` | `[2, 10, 14, 22]` | 26 |
| `corner_retreat_waypoints` | `[26, 27]` | 19 |
| `committed_terrain_waypoints` | `[28, 30]` | 6 |
| `committed_runup_waypoints` | `[28]` | 41 |
| `same_level_corridor_waypoints` | `[29]` | 33 |
| `route_hint_waypoints` + `route_hint_points` | `[31, 32]` + `[29.35, 17.8, 30.55, 18.5]` | 37 |
| `corner_preview_waypoints` | `[13, 19, 20, 21, 22, 23]` | 24 |

33 个航点里有 19 个被至少一组点名（2、10、13、14、16、19–32 中的多数）。follower_node.py 共 1841 行，其中 193 处引用这些集合。

这些参数每一个都是**有证据的**——注释里记录了具体的失败运行、测量数值和取值理由，工程上完全站得住。问题不是它们错，而是它们**描述的是路线，却存放在控制器里**。换一张图、换一条路线、甚至只是在中间插一个航点，这 8 组数组和 193 处分支同时失效。

### 1.2 真机上它们已经全被关掉了

`native_transfer/runtime.py` 第 139–147 行 **[代码]**：

```python
"fast_flat_waypoints": [-1],
"corner_retreat_waypoints": [-1],
"committed_terrain_waypoints": [-1],
"committed_runup_waypoints": [-1],
"same_level_corridor_waypoints": [-1],
"route_hint_waypoints": [-1],
"corner_preview_waypoints": [-1],
"waypoint_speed_limit_indices": [-1],
```

这是本次最有说服力的一条证据：**上真机的路径已经用实际行动否定了这套承载方式。** 不是因为这些行为不需要，而是因为它们绑定在旧赛道的航点编号上，换到 v3 地图的 Start→B 路线就没有意义。

于是当前真机 follower 是通用内核：pure pursuit ＋ 候选航向扫描 ＋ 地形分类 ＋ stall/recovery，全局限速 0.20 m/s。**它没有任何关于"前面那段是楼梯""这个平台很短""这个弯要提前转"的知识。** 这些知识得回来。

### 1.3 router 其实已经做对了一半

`strategy/router.py` 里有 `segment_policies: dict[tuple[int, int], str]` **[代码]**，源码注释写得很清楚：

> The mapping lives here rather than as an `if segment == (15, 16)` somewhere in the follower because the moment that ...

也就是说，**"按路段查表、而不是在控制器里写 if"这个原则，router 已经采用了。** follower 只是没有享受到同样的待遇。所以这次重构不是引入一个新范式，而是把 router 已有的做法推广到整个规划栈，并把 key 从"航点编号对"换成"路线弧长区间"。

---

## 2. 你的想法已经实现了一半（实测证据）

### 2.1 路线的 XY 已经来自走过的轨迹

`native_transfer/prepare_routes.py` 从 `official_policy_v1/bundle/course_full.yaml` 取前 13 个航点。把这 13 个航点逐个与 v3 优化轨迹 `trajectory.csv`（1435 个关键帧）做最近点匹配 **[实测]**：

| 航点 | 匹配到的关键帧 | XY 距离 | 草稿 Z | 轨迹 Z | 差 |
|---:|---|---:|---:|---:|---:|
| 0 | traj[0] | 0.496 | −0.429 | −0.003 | +0.427 |
| 1 | traj[123] | 0.319 | −0.354 | +0.103 | +0.456 |
| 2 | traj[1416] | 1.044 | −0.420 | +0.065 | +0.485 |
| 3 | traj[1413] | 0.174 | −0.379 | +0.030 | +0.408 |
| 4 | **traj[16]** | **0.000** | −0.349 | +0.102 | +0.451 |
| 5 | **traj[24]** | **0.000** | +1.266 | +1.624 | +0.357 |
| 6 | **traj[28]** | **0.000** | +1.441 | +1.877 | +0.436 |
| 7 | **traj[32]** | **0.000** | +2.221 | +2.601 | +0.380 |
| 8 | **traj[36]** | **0.000** | +2.385 | +2.875 | +0.489 |
| 9 | **traj[40]** | **0.000** | +2.822 | +3.302 | +0.479 |
| 10 | **traj[44]** | **0.000** | +3.768 | +4.117 | +0.349 |
| 11 | **traj[48]** | **0.000** | +4.566 | +4.923 | +0.357 |
| 12 | **traj[52]** | **0.000** | +4.634 | +5.052 | +0.418 |

航点 4–12 的关键帧编号是 16、24、28、32、36、40、44、48、52——**间隔恒为 4，XY 误差恒为 0.000**。B 段路线就是轨迹每隔 4 个关键帧取一个点。航点 0–3 是手工放的接近点（坐标是 0、4、8 这样的整数），它们匹配到 1416/1413 是因为那是收尾时的返回段。

### 2.2 被丢掉的是 Z，而且代价正好吃掉整个容差

18 个 Start→B 航点的 dZ = 轨迹 Z − 草稿地面 Z **[实测]**：

| 统计量 | 值 |
|---|---:|
| 均值 | +0.441 m |
| 标准差 | 0.057 m |
| 最小 / 最大 | +0.349 / +0.566 m |
| 极差 | **0.217 m** |

如果机身参考点相对地面的高度是常数，这一列应该是常数——那个常数就是 `body_z_offset`。它不是常数，极差 0.217 m。当前 native 的 Z 到点容差是 **0.20 m**，也就是说**仅"用重建地面高度＋一个常数偏移去近似机身高度"这一步引入的误差，就已经超过了整个 Z 容差预算**。

这不是重建做得差。这是一个不必要的转换：轨迹本来就记录了机身参考点在 map 系里的 Z，把它换成"地面 Z ＋ 未知常数"，等于把一个已知量换成了两个未知量。

### 2.3 平台长度：两个来源的结论不一致

把两者放进同一个 B 弧长坐标系（原点取草稿航点 4，匹配误差 0.048 m）**[实测]**：

| 来源 | 平台 1 | 平台 2 |
|---|---|---|
| 手工草稿（重建台阶边缘 ± 0.70 m 余量） | 7.76–9.51 = **1.75 m** | 15.19–15.76 = **0.58 m** |
| 走过的轨迹（\|坡度\| < 12° 的连续段） | 7.25–10.50 = **3.25 m** | 13.00–17.75 = **4.75 m** |

设计参考文档本身已经把平台 2 的 0.58 m 标为风险项（"长约 0.575 m；整机几何和停稳空间尤其需要复核"），因为它要在这段里完成两次停稳＋两次切换。轨迹给出的是 4.75 m。

**必须说清楚这个对比的边界**：两者测的不是同一件事。草稿测的是重建出的单级台阶边缘位置；我这里测的是 1.0 m 窗口平滑后的持续坡度，会把短平台抹掉也会把缓段拉长，而且轨迹关键帧中位间距 0.83 m **[实测]**，对台阶级别的分辨本来就偏粗。**结论不是"草稿错了"，而是"两个独立来源在这里分歧很大，必须现场判定"**，并且如果要用轨迹做分段，应该改用 bag 里 10 Hz 的 `/ODOM`（177.8 s、1777 条）而不是 0.83 m 间距的关键帧。

### 2.4 自动分段能恢复出正确的结构

只用轨迹本身（不碰重建、不碰仿真几何），按 |坡度| > 12° 判 stairs、短于 1.5 m 的段并入邻段 **[实测]**：

```
flat   -13.50 ..   1.25  (14.75 m)   +0.8°
stairs   1.25 ..   7.25  ( 6.00 m)  +14.4°
flat     7.25 ..  10.50  ( 3.25 m)   +4.0°
stairs  10.50 ..  13.00  ( 2.50 m)  +14.5°
flat    13.00 ..  17.75  ( 4.75 m)   +6.2°
stairs  17.75 ..  24.00  ( 6.25 m)  +14.4°
flat    24.00 ..  27.50  ( 3.50 m)   −0.2°
```

flat→stairs→flat→stairs→flat→stairs→flat，与手工草稿的模式序列结构一致。**这说明"从被教过的轨迹自动生成路段标注"是可行的**，不需要人工逐段指定。阈值（12°）和平滑窗口（1.0 m）是可调的工程选择，不是测量结果。

### 2.5 全场坡度分布

全程 1042 m 按 0.5 m 重采样 **[实测]**：

| 指标 | 值 |
|---|---|
| 每 0.5 m 转角 | 中位 2.3°，P90 15.9°，最大 176.6°（原地掉头） |
| 每 0.5 m 坡度 | 中位 1.0°，P90 10.5°，最大 28.0° |
| \|坡度\| > 3° | 24.9% |
| \|坡度\| > 7° | 15.2% |
| \|坡度\| > 12° | 7.8% |
| \|坡度\| > 16° | 2.4% |
| \|坡度\| > 20° | 0.4% |

这一列直接连到 RL 那条线的结论：交付的 A18@50 在约 10.6° 上坡、指令低于约 0.2 m/s 时会停住（0.15 m/s 只爬到 46%，0.2 m/s 到 94%）。**路线上有 7.8% 的里程比 12° 更陡**，而 pure pursuit 现在在每个航点前都会刹车到很低的速度。这两件事会在同一个地方相遇。路线层知道坡度，因此**可以在坡段设速度下限**——这是一个具体的、现在就能做的修复，比重训策略便宜得多。

---

## 3. 必须避开的陷阱：建图轨迹 ≠ 路线

1042 m 的 v3 建图轨迹是一次反复走遍全场的采集，**自交非常严重** **[实测]**。对每个位姿，问"是否存在另一个位姿，XY 距离小于阈值、但沿路径相距超过 5 m"：

| XY 阈值 | 全程 1042 m 建图轨迹 | Start→B 段（关键帧 0–55） |
|---|---:|---:|
| 0.3 m | 56.0% | **0.0%** |
| 0.5 m | 83.8% | **0.0%** |
| 1.0 m | 93.6% | **0.0%** |
| 2.0 m | 97.0% | — |

加上高度判据（|ΔZ| < 0.3 m）后数字不变——**多层结构并不能消解这个歧义，因为重叠主要来自同一层上的往返**。最多有 5 条不同的通过落在同一个 1.0 m 邻域里。

两个直接后果：

1. **路线必须从建图轨迹里"抽出"，而不是等于建图轨迹。** 需要一次明确的"从 A 到 B 的遍历"，可以由人在 App 上圈选，也可以按时间区间截取。Start→B 恰好就是关键帧 0–55 这一段连续遍历，所以它的自交是 0.0%。
2. **路线上的自我定位必须用单调弧长游标，不能用最近点。** 即使抽出来的路线自身不自交（Start→B 就不自交），一旦以后路线包含往返，最近点匹配就会跳。teach-and-repeat 的标准做法是只在上一次已知弧长 s 附近的窗口里搜索，这同时解决歧义和计算量。现有 `Course` 的游标已经是单调的 **[代码]**，这个性质要保留下来，不要在重构时换成"全局最近点"。

---

## 4. 建议的三层设计

### 4.1 职责划分

沿用设计参考 15.1 节已经提出的三层，但把"路线层到底存什么"具体化：

| 层 | 输入 | 输出 | 关键约束 |
|---|---|---|---|
| **路线层**（全局，离线生成） | 被教过的轨迹 ＋ 地图 ID/hash ＋ 现场标注 | 按弧长参数化的路径 ＋ 逐段标注 | 只描述路线，不含任何控制器状态；改路线不改代码 |
| **局部层**（在线，10–50 Hz） | 路线上的当前弧长 s ＋ 实时点云 | 在走廊内的横向偏移与速度 | **不能离开走廊**；只处理"教路时不存在的东西" |
| **执行层**（router） | 路段标注 ＋ 机身反馈 | 唯一速度输出 ＋ gait 切换 | 保持现有停稳/回执/互斥/故障锁定逻辑不变 |

核心变化只有一句话：**把 follower 里 193 处按航点编号的分支，换成路线层的逐段标注。**

### 4.2 路线的数据结构

路线不是航点列表，是**按弧长参数化的折线＋区间标注**。建议直接扩展现有 `start_b.draft.json`，保持它已有的 metadata / 溯源字段：

```yaml
# 设计示意。字段名可调，重点是"按弧长区间标注"而不是"按航点编号列表"。
route_id: start_b_v2
map_id: 0914_fr_v3-20260914-142008
map_sha256: 87a80cf2a88c4d8ab782772ff5b437e6df1c74c277364ae000ef90028dde81ea
taught_from:
  source: .sessions/session_0/poses.txt      # 优化后位姿，不是 lio_odom.pose
                                             # 交付包里的 maps/v3/trajectory.csv 是同一份数据
  keyframe_range: [0, 55]                    # 抽出来的那一次遍历
  z_reference: body                          # 关键：机身参考点，不是地面
  resample_spacing_m: 0.25

path:            # x y z yaw s，由 poses.txt 重采样得到；Z 直接来自轨迹
  - [0.000, 0.500, -0.003, 0.02, 0.00]
  - ...

segments:        # 按弧长区间，不按航点编号
  - s: [0.00, 14.75]
    gait: flat
    speed: {max: 0.60, min: 0.00}
    corridor_half_width: 0.80
    storey: ground
  - s: [14.75, 20.75]
    gait: stairs
    speed: {max: 0.15, min: 0.10}     # 下限：坡上不许刹到停
    corridor_half_width: 0.25         # 楼梯上几乎不许横向偏移
    storey: B_flight_1
    entry_staging: {s: 14.20, require_settled: true}
  - ...

annotations:     # 逐点/逐区间的附加事实，全部带证据来源
  - {s: 7.25, kind: landing_start, evidence: traj_grade_2026-09-18}
  - {s: 39.5, kind: known_blocked_in_sim, note: vertical_proxy geometry}
```

三个设计要点：

1. **`z_reference: body`。** 路径的 Z 直接来自 `poses.txt`，就是机器人当时机身参考点的高度。`body_z_offset` 这个字段消失，2.2 节那 0.217 m 的散布随之消失。
2. **标注的 key 是弧长区间 `s: [a, b]`，不是航点编号。** 在路线中间插一个点不会让任何标注失效。这是 1.3 节里 router 已经在用的做法，只是把 key 从 `(15,16)` 换成 `[a,b]`。
3. **速度既有上限也有下限。** 现在只有上限。2.5 节那个 10.6° 上坡停住的问题，在这里是一行 `min: 0.25`。

### 4.3 路线上的自我定位

这是 teach-and-repeat 相对于"朝一个全局目标走"的核心差别，也是本次最需要新写的一块：

```
输入：map 系下的当前位姿（来自厂商定位）＋ 上一拍的弧长 s_prev
1. 只在 [s_prev − 1.0, s_prev + 3.0] 的窗口里搜索最近点  ← 单调游标，解决 3 节的歧义
2. 投影得到 (s, lateral, heading_error)
3. s 落在哪个 segment 区间，就取那一段的 gait / 速度 / 走廊 / 楼层
4. lateral 超过走廊 → 不是"绕路"，是"偏离路线"，交给恢复逻辑而不是局部规划器
```

这就是 8.1 节 topometric 综述里"我在哪个子图附近、相对它偏了多少"的那个函数。有一条结论值得特别注意：Krajník 等证明了**只校正航向、重放被教的速度剖面，横向误差有界不发散** [8.1]——也就是说这套结构对定位精度的要求比直觉低得多。对你们尤其重要，因为绝对定位精度至今没有实测，`/ODOM` 的协方差还是全零。

**顺带解决一个已知的老问题。** `waypoints.py` 的 docstring 详细记录了到点半径的反复：0.35 m 会让 carrot 在转角处提前甩到下一段，丢了 17、21、22 号门（最近距离 0.338 / 0.320 / 0.336 m）；改成 0.18 m 之后又要处理"没到点就不能推进游标"的僵持 **[代码]**。Nav2 Route Server 对同一问题的做法不是调半径，而是**取入边与出边的角平分向量、看点积符号变化**来判定节点通过，几何上无歧义，只在首末节点退回半径判据 [8.4]。改成弧长参数化之后这件事更自然：**推进的判据是 s 越过了区间边界，不是"离某个点足够近"**。原来的 0.18 m 半径可以保留为比赛计分用的独立检查，与控制推进解耦。

输出的 `(s, lateral, heading_error)` 就是局部层的全部全局输入。注意 **lateral 的语义变了**：现在 `pure_pursuit` 的 `cross_track` 是相对"到目标点的连线"，会随着游标跳变；相对被教过的路径投影则是连续的，因为路径本身连续。

### 4.4 局部层：走廊内的横向偏移搜索

现在的 `LocalPlanner.plan` 在目标方位角 ±90° 范围内取 31 个候选航向，按 `1.6 × 净空 − 1.0 × 偏离` 打分 **[代码]**。它的问题是**搜索空间没有被路线约束**：±90° 意味着它可以选一个完全离开已知可通行区域的航向，而它只能看到水平雷达环（对台阶和落差是瞎的）。源码注释里记录的失败——走下平台边缘、撞进矮墙、在楼梯前左右横跳——都是这个形状。

建议换成**沿被教过路径的横向偏移候选**：

```
候选 = {路径在 s+lookahead 处的点，横向偏移 d}，d ∈ {−0.8, −0.6, …, +0.6, +0.8} ∩ 走廊
对每个候选：
  - 用实时点云检查机身宽度的走廊是否净空（沿用现有 clearances 的走廊扫描）
  - 用被教过的路径高度作为地面先验，判定高度图里哪些是"新出现的东西"
打分 = 净空 − w·|d|        ← |d| = 偏离被教路线的程度，不是偏离目标方位
d = 0 恒为最高优先；只有它被挡住才考虑偏移
```

相对现在的做法，这有四个直接好处：

1. **搜索空间不可能离开走廊。** 这不是靠打分惩罚实现的，是靠候选集合的构造实现的——这是与"±90° 航向扫描＋惩罚偏离"的本质区别。源码注释里那些"走下平台边缘"的失败，在这个结构下不可能发生。
2. **d = 0 是被证明可通行的。** 机器人物理上从那里走过。所以"什么都不做"永远是一个有效答案，不存在"没有候选"的死锁。
3. **高度图有了先验。** 现在 `terrain_relief` 要从单帧高度图里同时估计"地面在哪"和"障碍有多高"，靠平面拟合，在多层结构上会失败（WP28/29/30 那三组 hack 就是为此而生）。有了被教过的路径高度，地面先验是已知的，高度图只需要回答"相对先验多出了什么"。
4. **楼梯上可以直接把走廊收到 ±0.25 m**，等于关掉横向搜索，而不需要一个 `stairs` 专用代码分支。

这就是你说的"用一些 algo/简单的操作"——它确实简单：一次 9 个候选的打分，没有动力学模型，没有优化器，可以完全复用现有的 `clearances()` 走廊扫描实现。

**这个方案的已发表版本**：Sehn 等的两篇 [8.2] 就是它的严谨形式，且已在 5 km 野外试验中对照 MPC 验证过。三点与上面独立得到的结论一致：贴着被教路径的**曲线坐标系**、代价里**显式的横向偏离项**、障碍感知作为**相对 teach 阶段的变化检测**。第三点比本文"用被教路径高度作地面先验"的说法更一般，建议直接采用他们的表述来写规格。

**如果 9 个候选不够用**，扩展路径是 CMU AEDE 的候选路径库 [8.3]：343 条路径分 7 组、36 个朝向，全部碰撞几何**离线**预计算成"体素 → 经过它的路径 ID 列表"，运行时每个障碍点只做一次 O(1) 查表投票。它的三个细节值得现在就借鉴，即使只做 9 个候选：

- **按组投票而不是选单条**（每组 49 条），所以一组胜出是因为多数成员畅通，对稀疏噪声点云鲁棒——你们的 `/NAV_POINTS` 只有约 3350 点，正需要这个性质。
- **阻塞判据是"管内 ≥ 2 个点"**，不是 1 个。单点不杀候选。
- **无解时的降级是缩短、不是停下**：逐步缩小路径长度再缩小搜索范围，最后才报失败。这与 `terrain.py` 里 `MIN_TERRAIN_FRACTION` 注释的推理（"Reaching zero is unrecoverable"）是同一个思路。

### 4.5 router 与 gait 切换

**不建议改动 `NativeGaitRouter` 的状态机。** 它的"输出零 → 测得停稳 → 发一次 gait → 等新鲜回执 → 持续确认 → 才输出速度"次序是已经通过 78 项本地测试和两项隔离 ROS 联调的部分 **[代码/文档]**，是整个栈里最不该动的地方。

唯一的改动是**它从哪里得知下一段要用什么 gait**：现在是路线 JSON 的逐航点 `kind`，改成按弧长区间查 `segments[].gait`。切换点也从"上一个航点"变成"区间边界前的 staging 弧长"，这样 4.2 里的 `entry_staging` 可以显式写出停稳位置，而不是隐含地取"上一个航点"。

**多层高度图有现成的解法，不必自己硬编码。** `elevation_mapping_cupy` 有 `enable_overlap_clearance` ＋ `overlap_clear_range_z`，官方配置注释写的就是 "used for multi floor setting"：把机器人当前 z 带之外的地图格丢掉，于是局部高度图只表示当前这一层 [8.5]。`nav.yaml` 里 `same_level_corridor_waypoints: [29]` 和 `committed_terrain_waypoints: [28, 30]` 这三组硬编码，本质上就是在手工实现这个功能。**是否引入这个依赖要单独评估**（它要 GPU，而感知目前跑在 106 板上），但至少说明"按 z 带裁剪"是这个问题的标准答案，值得在自己的实现里照做。

2.3 节的平台长度分歧在这里有直接影响：如果平台 2 真的只有 0.58 m，就放不下两次停稳，应该整段保持 stairs；如果是 4.75 m 就放得下。**这必须现场量，不能靠选一个更喜欢的数字。**

### 4.6 走廊宽度的量级（初步核对）

用 v3 的二维栅格（0.05 m/格）沿 Start→B 的被教路径做垂线，量到第一个非 free 格的距离 **[实测]**：

| 指标 | 左 | 右 | min(左,右) |
|---|---:|---:|---:|
| 中位 | 2.00 m | 2.00 m | 1.95 m |
| P10 | 1.00 m | 1.35 m | 0.95 m |
| 最小 | 0.30 m | 0.10 m | 0.10 m |

两侧都 ≥ 0.25 m 的比例 98%，≥ 0.50 m 为 93%，≥ 0.80 m 为 91%。

**这个数只能当量级参考，不能当净空证明**：二维栅格是多层场地的投影，上层平台会压到下层上；而且栅格 63% 是 unknown **[实测]**，unknown 在这里被当作"非 free"处理，所以结果偏保守。它支持的结论仅仅是：平地段 ±0.8 m 的走廊是合理的量级，楼梯段必须收窄（最小值 0.10 m 正是出现在窄处）。真正的走廊宽度要用整机包络在现场定。

### 4.7 这个设计做不到什么

必须先写下来，避免以后被当成通用能力：

1. **只能去走过的地方。** 路线是被教出来的，没走过的区域没有路线。对巡逻/比赛这种"走一条固定路线"的任务这正好；如果以后要"去任意目标点"，需要另外一层（多条路线组成的图，或厂商的全局规划器），这是另一个项目。
2. **被搬离路线后不能自己回去。** 弧长游标假设机器人一直在路线附近。开机在别处、被人抬走、定位丢失后重获，都需要一个明确的**重新接入**流程：先确认全局定位，再确认落在某个 s 的走廊内，才允许继续。不能让游标自己去全局搜索最近点（3 节）。
3. **不处理路线本身变错。** 如果场地改了，被教的路线就是错的，局部层的走廊约束会让它更固执地走错路。路线必须和地图 hash 绑定，地图换了就作废。
4. **走廊约束会拒绝一些其实可行的绕行。** 这是有意的取舍：用"可能绕不过去"换"不会走到没走过的地方去"。当前阶段（定位和感知契约都还没验收）这个方向是对的，以后可以放宽。

---

## 5. 局部规划器的选型

你的直觉（"简单操作"）是对的，下面是把它放在文献坐标系里的位置，以及为什么不选另外几种。

### 5.1 现在这个搜索为什么会走下平台：用真实代码复现

`LocalPlanner.clearances` 的 docstring 明确写了非回波会被丢弃（"Drop non-returns so they cannot masquerade as obstacles at range_max"）**[代码]**。而落差是**没有回波**的。把这两件事放在一起，用仓库里的真实 `LocalPlanner` 跑一个场景 **[实测]**：

> 目标方向有一堵 0.9 m 处的墙；左边（正角度）是真实的开阔地面，3.5 m 处有回波；右边（负角度）是平台边缘，**完全没有回波**。

```
chosen heading -54.0 deg, clearance 4.00 m, speed_scale 1.00
  heading  -90 deg -> clearance 4.00 m   <- 悬崖侧
  heading  -60 deg -> clearance 4.00 m   <- 悬崖侧
  heading   +0 deg -> clearance 0.85 m
  heading  +60 deg -> clearance 3.49 m   <- 真实开阔地面
  heading  +90 deg -> clearance 3.49 m
```

**规划器选了悬崖侧（−54°），因为悬崖比真实的开阔地面"更空"**（4.00 对 3.49）。这不是打分权重没调好，是搜索空间里根本没有"这块地能不能走"这个信息。

唯一能纠正它的是高度图，而 `nav.yaml` 里 `same_level_corridor_waypoints: [29]` 和 `committed_terrain_waypoints: [28, 30]` 这两组参数存在的**全部理由**就是高度图在多层结构上给出了错误答案（注释原文："WP30 reported a nonexistent 0.9-1.0 m drop"、"reports a nonexistent 0.73-0.79 m rise and steers the robot into the side structure"）**[代码]**。也就是说：唯一的纠正机制，恰恰在最需要它的地方失效，于是只能用航点编号硬编码绕开。

**横向偏移搜索从结构上消除这个失败**：候选集合由被教过的路径生成，机器人没走过的地方不会出现在候选里。这不是给悬崖加一个惩罚项，是让悬崖不进入候选。

### 5.2 为什么不选其他方案

| 方案 | 不选的理由 |
|---|---|
| **保留 ±90° 航向扫描，加高度图惩罚** | 已经试过了，就是现在这套。高度图在多层结构上不可靠（上一节），加惩罚等于把正确性押在最不可靠的输入上。 |
| **DWA / TEB** | 需要可信的运动学/动力学模型。你们的执行器是厂商黑盒 RL 策略，实测在约 10.6° 上坡、低速指令下会直接停住——这种行为没法用速度空间模型描述。而且路线已知时，它们的自由度是浪费。 |
| **MPPI** | 同上，外加采样成本。10 Hz 控制率、106 板的算力、以及"必须能解释每一个决策"的验收要求，都不适合。 |
| **Nav2 全栈（含 costmap＋behavior tree）** | 引入一整套新的未知（代价地图层、恢复行为、TF 树），而你们的执行链已经绑定厂商 DDS 与 `NavCmd`，且定位/感知契约本身还没验收。**但 Route Server 的数据模型应当借鉴**：有向边挂任意元数据、边代价插件栈、进入/离开边时触发的 operations（官方举例就包括"换模式"）、角平分线的节点到达判据。它官方文档的第 6、7 种架构正是 teach-and-repeat 与多层楼 [8.4]。借数据模型，不借实现。 |
| **学一个 router 网络** | 9 月 6 日已经讨论过这个方向。现在更不合适：连规则版的输入契约（NAV_POINTS 语义、定位质量）都还没验收，学出来的东西无法归因。 |

---

## 6. 迁移路径

这套改动可以完全增量地做，每一步都能独立验证，任何一步失败都不影响上一步。**每一步都不改 `NativeGaitRouter` 的状态机。**

### 第 1 步：把 Z 换回来（半天，零风险，收益最大）

改 `prepare_routes.py`：航点 Z 直接取自 `poses.txt` 的机身参考点高度，不再取重建地面高度。

- 交付：新的 `start_b.draft.json`（新 route_id，不覆盖旧文件），以及一份新旧 Z 对照表。
- 验证：`body_z_offset` 从配置里删掉；2.2 节那 0.217 m 的散布消失。
- 注意：**Z 容差 0.20 m 的含义变了**——以前是"重建地面＋未知偏移"的容差，现在是"机器人当时站在哪"的容差。数值可以不变，但要在文档里改口径。
- 风险：几乎没有。这一步不改任何控制逻辑，只改路线里的一列数。

### 第 2 步：路线格式升级为弧长＋区间标注（1–2 天）

把路线从"航点列表"改成 4.2 的结构，并写一个 `route_locate(pose, s_prev) -> (s, lateral, heading_error, segment)`。

- 交付：新格式的路线文件、`route_locate` 及其单元测试（含 3 节的自交用例）。
- 验证：拿 Start→B 的 56 个关键帧回放，确认 s 单调、lateral 连续、segment 切换点与 2.4 节的自动分段一致。
- 这一步**只增加一个只读的定位函数，先不接进控制回路**，可以完全离线验证。

### 第 3 步：路线标注驱动 gait 与限速（1–2 天）

让 `NativeGaitRouter` 从 `segments[].gait` 取模式，从 `segments[].speed` 取上下限。

- 交付：router 的输入源改动 ＋ 隔离 ROS 联调（沿用现有 domain 211 的 `isolated_acceptance.py`）。
- 验证：合成反馈下，切换点与路线标注一致；速度下限在坡段生效。
- **这一步就能解决 2.5 节的低速上坡停住问题**，且不需要任何新的感知能力。

### 第 4 步：局部层改为走廊内横向偏移搜索（3–5 天）

新写一个与现有 `LocalPlanner` 并列的实现，**先在 shadow 模式下跑**（`real_transfer/shadow.py` 已经有这个机制）。

- 交付：新局部规划器 ＋ 用 9/17 的 bag 做的离线对照（同一段输入，两个规划器各自输出什么）。
- 验证：走廊约束下不产生离开走廊的候选；d=0 在无障碍时恒被选中。
- 只有 shadow 对照通过后才接进执行链。

### 第 5 步：删掉 8 组航点编号数组（1 天，收尾）

确认第 3、4 步覆盖了原有行为之后，从 `nav.yaml` 和 `follower_node.py` 删掉那 8 组参数和 193 处引用。

- 这一步是**收益兑现**：follower_node.py 应该会显著变短，且不再包含任何航点编号。
- 旧赛道的竞赛配置如果还要保留，应该以"另一条路线的标注文件"的形式保留，而不是以代码分支的形式。

### 6.1 八组参数分别搬到哪里

这是第 5 步能否安全删除的检查表。每一行都要在删除前确认新位置确实覆盖了原行为。

| 现有参数 | 原意图 | 新位置 | 备注 |
|---|---|---|---|
| `waypoint_speed_limit_indices/values` | 特定航点附近限速 | `segments[].speed.max` | 直接对应，弧长区间比航点编号更精确 |
| `fast_flat_waypoints` ＋ 4 个门槛 | 确认平直时才放开到 2.1 m/s | `segments[].speed.max` ＋ 现有的在线门槛 | 路线给上限，门槛（航向误差/横向误差/倾角）仍然在线判定，保留 |
| `corner_retreat_waypoints` | 窄平台上直角转弯先后退 | `segments[].maneuver: corner_retreat` ＋ staging 弧长 | 是一个"路段进入动作"，不是航点属性 |
| `committed_terrain_waypoints` | 已知可通行、不许被高度图否决 | `segments[].trust_taught_ground: true` | 语义更清楚：相信被教过的地面先验 |
| `committed_runup_waypoints` ＋ 3 个参数 | 最后一级台阶需要助跑 | `segments[].maneuver: runup` ＋ 参数 | 41 处引用，是搬迁工作量最大的一项 |
| `same_level_corridor_waypoints` | 拒绝跨层的高度图标签 | `segments[].storey` | 有了楼层 ID，跨层点自动被排除，不需要点名航点 |
| `route_hint_waypoints` ＋ `route_hint_points` | 硬编码两个中间点 | **直接并入路径本身** | 这两个点本来就该是路线的一部分；以弧长参数化后它们不再是"提示"，就是路径 |
| `corner_preview_waypoints` ＋ 3 个参数 | 宽弯中边走边转 | `segments[].maneuver: corner_preview` | 触发条件可改为由路径曲率自动判定，不必逐段点名 |

最后两行是这次重构收益最直观的地方：`route_hint_points: [29.35, 17.8, 30.55, 18.5]` 是两个写死在控制器参数里的世界坐标——它们本来就是路线上的点，只是因为路线格式装不下才被挤到 `nav.yaml` 里。

### 不建议现在做的事

- **不要重训策略去解决低速上坡。** 第 3 步的速度下限便宜得多，而且 RL 那边的结论已经很明确：单次训练之间的差异已经落在种子噪声里，再调参没有信息量。
- **不要引入 Nav2 全栈。** 你们的执行链已经绑定厂商 DDS 接口和 `NavCmd`，Nav2 的代价地图/行为树会引入一整套新的未知，而路线已知的情况下它的全局规划器无事可做。可以借鉴 Route Server 的数据模型，不必引入实现。
- **不要在现场定位和感知契约验收之前做第 4 步。** 局部层的输入是 `/NAV_POINTS`，而它的处理语义（去畸变、自体滤除、地面覆盖）在设计参考 14.2 节里仍然是未验收项。局部规划器再好也救不了一个语义未知的输入。

---

## 7. 与其他两条线的接口

| 线 | 接口 | 说明 |
|---|---|---|
| **RL 策略** | 路线层的 `speed.min` | A18@50 在约 10.6° 上坡、指令 < 0.2 m/s 时停住；路线知道坡度，可以在坡段抬高下限。这是导航侧的便宜解法，`pure_pursuit.py` 的 `brake_distance` 注释里已经记录过同一现象。 |
| **RL 策略** | 路线层的 `gait` / 策略选择 | 现在是 flat/stairs 两个厂商模式。以后若接自训策略，是在同一张表里多一个枚举值，不是多一个代码分支。 |
| **新 SLAM** | `taught_from.source` ＋ `map_sha256` | 换 SLAM 或重建地图后，路线必须重新绑定并换新 route_id。路线以弧长参数化之后，重绑定是"把标注映射到新路径"，比重标 33 个航点编号容易。 |
| **App** | `segments[]` | 现场助手可以直接显示"当前在第几段、这段是什么模式、限速多少"，而不是显示一个航点编号。 |

---

## 8. 参考文献

全部链接已由本次研究逐条访问核对。Wiley / Annual Reviews / Science / SAGE 对自动访问返回 403，这些条目改用 Crossref 的 DOI 元数据核对，标题、期刊、年份一致。**没有引用任何无法核实的条目。**

### 8.1 Teach-and-repeat（本设计的主线）

- A. Krawciw, T. D. Barfoot, **Local Maps Are All You Need: A Review of Topometric Teach and Repeat Navigation**, *Annual Review of Control, Robotics, and Autonomous Systems* 9:301–324, 2026. [doi:10.1146/annurev-control-032724-020548](https://doi.org/10.1146/annurev-control-032724-020548)
  **先读这一篇。** 它把这类系统命名为 **topometric**，并给出组件分类：局部子图链 → 相对定位 → 局部度量控制。定位问题被化简为"我在哪个子图附近、相对它偏了多少"，正好是本文 4.3 节要写的那个函数。
- P. Furgale, T. D. Barfoot, **Visual teach and repeat for long-range rover autonomy**, *Journal of Field Robotics* 27(5):534–560, 2010. [doi:10.1002/rob.20342](https://doi.org/10.1002/rob.20342)
  奠基工作。teach 阶段建一串**相对**位姿相连的局部子图，repeat 阶段只对最近的子图定位。**全局漂移因此不进入控制回路**——这正是你们现在最担心的厂商定位问题。
- M. Mattamala, N. Chebrolu, M. Fallon, **An Efficient Locally Reactive Controller for Safe Navigation in Visual Teach and Repeat Missions**, *IEEE RA-L* 7(2):2353–2360, 2022. [doi:10.1109/LRA.2022.3143196](https://doi.org/10.1109/LRA.2022.3143196) · [arXiv:2201.03938](https://arxiv.org/abs/2201.03938) · [代码](https://github.com/ori-drs/field_local_planner)
  **腿式机器人上的直接证据。** 在被教路径与机器人之间插一层局部反应控制器，理由和本文 4.4 节完全一样：被教过的路径上会出现教路时不存在的障碍。10 Hz、CPU 上 **< 2 ms**，在 ANYmal C 上验证。
- D. Baril et al., **Kilometer-scale autonomous navigation in subarctic forests**, *Field Robotics* 2:1628–1660, 2022. [doi:10.55417/fr.2022050](https://doi.org/10.55417/fr.2022050) · [arXiv:2111.13981](https://arxiv.org/abs/2111.13981)
  18.8 km 自主复走，点云配准，无 GNSS。说明这条路线在长距离和恶劣条件下成立。
- T. Krajník et al., **Navigation without localisation: reliable teach and repeat based on the convergence theorem**, IROS 2018. [arXiv:1711.05348](https://arxiv.org/abs/1711.05348)
  证明：**只校正航向、重放被教的速度剖面，横向误差有界不发散。** 对定位精度的要求比直觉低得多。这条对你们特别有用，因为绝对定位精度至今没有实测。
- K. M. Papais, W. Zhao, T. D. Barfoot, **Degeneracy-Resilient Teach and Repeat … Using FMCW Lidar**, 2026. [arXiv:2603.10248](https://arxiv.org/abs/2603.10248)
- R. Xiao et al., **LiDAR Teach, Radar Repeat**, accepted *IEEE T-RO*, 2026. [arXiv:2605.02809](https://arxiv.org/abs/2605.02809)
- **VT&R3** 参考实现（C++，camera/LiDAR/radar），UTIAS ASRL：[github.com/utiasASRL/vtr3](https://github.com/utiasASRL/vtr3)

### 8.2 走廊内的局部避障（本文 4.4 节的已发表形式）

- J. Sehn, T. D. Barfoot, J. Collier, **Off the Beaten Track: Laterally Weighted Motion Planning for Local Obstacle Avoidance**, accepted *IEEE Trans. Field Robotics*, 2024. [arXiv:2309.09334](https://arxiv.org/abs/2309.09334)
- J. Sehn, Y. Wu, T. D. Barfoot, **Along Similar Lines: Local Obstacle Avoidance for Long-term Autonomous Path Following**, 2022. [arXiv:2211.02047](https://arxiv.org/abs/2211.02047)

  **这两篇就是本文 4.4 节所提方案的严谨版本，而且已经在 5 km 野外试验中验证过。** 三个要点与本文独立得出的结论一致：（1）在**贴着被教路径的曲线坐标系**里规划，而不是在世界系里搜航向；（2）代价函数**显式惩罚横向偏离**，因为被教过的路径是唯一有证据可通行的地面；（3）障碍感知化简为**相对 teach 阶段的变化检测**——只报告教路时不存在的东西。第（3）点正好是本文 4.4 节"用被教路径高度作地面先验"的更一般说法，建议直接采用他们的表述。

### 8.3 候选路径库局部规划器（CMU AEDE）

- C. Cao, H. Zhu, F. Yang, Y. Xia, H. Choset, J. Oh, J. Zhang, **Autonomous Exploration Development Environment and the Planning Algorithms**, *ICRA 2022*. [doi:10.1109/ICRA46639.2022.9812330](https://doi.org/10.1109/ICRA46639.2022.9812330) · [arXiv:2110.14573](https://arxiv.org/abs/2110.14573) · [代码](https://github.com/HongbiaoZ/autonomous_exploration_development_environment) · [cmu-exploration.com](https://www.cmu-exploration.com/)

  论文只给了高层描述，**实现细节要读源码**（`noetic` 分支）。本次已读出的关键数字：

  | 项 | 值 | 位置 |
  |---|---|---|
  | 路径总数 / 分组 | **343 条 / 7 组**，每条 3.0 m，极坐标三次样条 | `paths/path_generator.m` |
  | 角度参数 | `angle=27°`，`deltaAngle=9°`，`scale=0.65` | 同上 |
  | 预计算体素网格 | 161 × 451 = **72,611 格 @ 0.02 m** | 同上 |
  | 碰撞半径 | `searchRadius = 0.45 m` | 同上 |
  | 运行时朝向数 | **36 个，10° 一档** → 343×36 = 12,348 候选 | `src/localPlanner.cpp` |
  | 判定阻塞 | 0.45 m 管内 **≥ 2 个障碍点** | `pointPerPathThre` |
  | 降级策略 | 无解时逐步缩小 `pathScale`（步长 0.25，下限 0.75）再缩 `pathRange`（步长 0.5，下限 1.0） | 同上 |

  **核心思想**：所有碰撞几何**离线**预计算成"体素 → 经过该体素的路径 ID 列表"，运行时每个障碍点只做一次 O(1) 查表并给对应路径投票。因此计算量与杂乱程度无关、完全确定、可逐条调试。按组投票（每组 49 条）而不是选单条，是它对传感器噪声鲁棒的原因。
  地形侧的接口极简：`/terrain_map` 是一个 **5 Hz 的点云，intensity 即通行代价**；局部规划器只用 `obstacleHeightThre = 0.2 m`（硬障碍）和 `costHeightThre = 0.1 m`（软代价）两个门限。

- F. Yang et al., **FAR Planner: Fast, Attemptable Route Planner using Dynamic Visibility Update**, *IROS 2022*（Best Student Paper）。[doi:10.1109/IROS47612.2022.9981574](https://doi.org/10.1109/IROS47612.2022.9981574) · [arXiv:2110.09460](https://arxiv.org/abs/2110.09460) · [代码](https://github.com/MichaelFYang/far_planner)
- C. Cao et al., **TARE: A Hierarchical Framework for Efficiently Exploring Complex 3D Environments**, *RSS 2021*（Best Paper）。[doi:10.15607/RSS.2021.XVII.018](https://doi.org/10.15607/RSS.2021.XVII.018) · [PDF](https://www.roboticsproceedings.org/rss17/p018.pdf)
  TARE 的"近处细、远处粗"分层论证，就是本文全局/局部分层的理由。
- H. Zheng et al., **SCAN-Planner: Spatial Collision-Aware Local Planning for Route-Guided Long-Range Quadruped Navigation**, 2026. [arXiv:2606.19555](https://arxiv.org/abs/2606.19555)
  最新的 route-guided 四足局部规划器，含**偏航相关的双圆柱足迹**（比球形包络贴合得多）与楼梯处理，值得对照。

### 8.4 路线图 / 拓扑-度量（路线数据结构的参考）

- **Nav2 Route Server**（`nav2_route`），S. Macenski & J. Wallace。[源码与设计 README](https://github.com/ros-navigation/navigation2/tree/main/nav2_route) · [配置文档](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/route_server/configuring_route_server/) · [图编辑工具](https://docs.nav2.org/rolling/tutorials/general_tutorials/route_server_tools/)

  **本文 4.2 节路线数据结构的最佳参考实现。** 几个可以直接借用的设计：
  - 节点和**有向边都可以挂任意元数据**（限速、惩罚、语义标签、operation），这正是本文"标注挂在弧长区间上"的对应物。
  - 边代价是**插件栈**（距离/时间/代价地图/惩罚/语义/动态边），任一插件都可以根据实时状态把一条边判为不可用。
  - **operations 插件**在进入/离开边、到达节点时触发，README 举的例子包括"开门、在节点等待净空、调整最高速度、开灯、**换模式**"——你们的 gait 切换就是这个。
  - **节点到达判据不是半径**，而是取入边与出边的角平分向量、看点积符号变化，几何上无歧义。**这一条值得单独看**：`waypoints.py` 的 docstring 里记录的 0.35 m / 0.18 m 半径来回改动、以及"转角处 carrot 提前甩到下一段"的问题，用角平分判据可以从根上消掉。
  - README 里给的七种 "Practical Architectures" 中，**第 6 种就是 teach-and-repeat**（遥控走一遍 → 自动采节点 → 存图 → 逐节点标注 operation → 之后自主执行），**第 7 种是多层楼**（节点作为楼梯/电梯终端）。你们的两个需求都在官方文档里有对应架构。
  - 规划耗时基准（README 原文，1000 次随机起终点）：100 节点 0.0031 ms、10k 节点 0.232 ms、90k 节点 3.75 ms、250k 节点 11.36 ms、**1,000,000 节点 44.07 ms**，对比自由空间全局规划器的典型 50–400 ms。

- W. Churchill, P. Newman, **Experience-based navigation for long-term localisation**, *IJRR* 32(14):1645–1661, 2013. [doi:10.1177/0278364913499193](https://doi.org/10.1177/0278364913499193)
- L. Zhang et al., **Topological local-metric framework for mobile robots navigation: a long term perspective**, *Autonomous Robots*, 2018. [doi:10.1007/s10514-018-9724-7](https://doi.org/10.1007/s10514-018-9724-7)
  几何**只存在边里**（相邻节点间的相对位姿），把全局一致性放松为局部一致性。
- J.-F. Tremblay et al., **Topological mapping for traversability-aware long-range navigation in off-road terrain**, 2024. [arXiv:2410.01925](https://arxiv.org/abs/2410.01925)
- K. Muravyev, K. Yakovlev, **NavTopo: Leveraging Topological Maps for Autonomous Navigation**, ICR 2024. [arXiv:2410.11492](https://arxiv.org/abs/2410.11492)

### 8.5 高程建图与可通行性

- T. Miki, L. Wellhausen, R. Grandia, F. Jenelten, T. Homberger, M. Hutter, **Elevation Mapping for Locomotion and Navigation using GPU**, IROS 2022. [arXiv:2204.12876](https://arxiv.org/abs/2204.12876) · [代码](https://github.com/leggedrobotics/elevation_mapping_cupy)

  **这里有一个对你们的多层问题直接可用的东西**：配置项 `enable_overlap_clearance`，配合 `overlap_clear_range_xy: 4.0`、`overlap_clear_range_z: 2.0`，官方配置注释写的就是 *"used for multi floor setting"*——把机器人当前 z 带之外的地图格丢掉，于是局部高度图只表示**当前这一层**。
  `nav.yaml` 里 `same_level_corridor_waypoints: [29]` 和 `committed_terrain_waypoints: [28, 30]` 这三组硬编码，本质上是在手工实现这个功能。默认参数：0.04 m 分辨率、8 m × 8 m 机器人中心、`map_acquire_fps: 5.0`、`max_drift: 0.1`。
- P. Fankhauser, M. Hutter, **A Universal Grid Map Library**, in *ROS: The Complete Reference, Vol. 1*, Springer 2016. [doi:10.1007/978-3-319-26054-9_5](https://doi.org/10.1007/978-3-319-26054-9_5) · [代码](https://github.com/ANYbotics/grid_map)
  多层栅格（`elevation` / `variance` / `traversability` / `step_height` / 法向量…同一张网格上的多个命名浮点矩阵），机器人中心的二维循环缓冲。建议把它作为感知与规划之间的**唯一契约**。
- M. Wermelinger, P. Fankhauser, R. Diethelm, P. Krüsi, R. Siegwart, M. Hutter, **Navigation planning for legged robots in challenging terrain**, *IROS 2016*. [doi:10.1109/IROS.2016.7759199](https://doi.org/10.1109/IROS.2016.7759199) · [代码](https://github.com/leggedrobotics/traversability_estimation)
  几何式可通行性配方：坡度（拟合平面法向）＋粗糙度（高度标准差）＋台阶高度 → 加权到 [0,1]。与你们 `terrain.py` 的 `Relief`（rise / drop / slope 三分）思路一致，可作为对照。
- J. Frey, M. Mattamala, N. Chebrolu, C. Cadena, M. Fallon, M. Hutter, **Fast Traversability Estimation for Wild Visual Navigation**, *RSS 2023*. [arXiv:2305.08510](https://arxiv.org/abs/2305.08510) · 期刊版 *Autonomous Robots* 2025 [doi:10.1007/s10514-025-10202-x](https://doi.org/10.1007/s10514-025-10202-x)
- L. Wellhausen, M. Hutter, **ArtPlanner: Robust Legged Robot Navigation in the Field**, 2023. [arXiv:2303.01420](https://arxiv.org/abs/2303.01420) · [代码](https://github.com/leggedrobotics/art_planner)
  DARPA SubT 决赛中 4 台 ANYmal、90 分钟自主运行、零规划/运动失败。

### 8.6 多层 / 楼层感知的地图表示

- R. Triebel, P. Pfaff, W. Burgard, **Multi-Level Surface Maps for Outdoor Terrain Mapping and Loop Closing**, *IROS 2006*. [doi:10.1109/IROS.2006.282632](https://doi.org/10.1109/IROS.2006.282632)
  每个格子存一个**表面片列表**（均值高度＋垂直范围），保留二维索引的同时表示桥、天桥和叠层。
- B. Yang, J. Cheng, B. Xue, J. Jiao, M. Liu, **Efficient Global Navigational Planning in 3D Structures based on Point Cloud Tomography**, *IEEE/ASME Trans. Mechatronics*, 2024. [arXiv:2403.07631](https://arxiv.org/abs/2403.07631)
  把点云切成水平层，每层每格同时记地面高度与天花板高度，规划是**跨层图搜索**并显式建立层间转移边（楼梯、坡道）。在四足上验证。
- Y. Tang et al., **Path Planning on Multi-level Point Cloud with a Weighted Traversability Graph**, 2025. [arXiv:2504.21622](https://arxiv.org/abs/2504.21622)
- Z. Zhu et al., **Multi-Floor Exploration for Ground Robots via an Incremental Reachable Graph and Structural Priors**, 2026. [arXiv:2605.23350](https://arxiv.org/abs/2605.23350)
- S. Qi, W. Lin, Z. Hong, H. Chen, W. Zhang, **Perceptive Autonomous Stair Climbing for Quadrupedal Robots**, *IROS 2021*. [doi:10.1109/IROS51168.2021.9636302](https://doi.org/10.1109/IROS51168.2021.9636302)

### 8.7 按路线切换步态 / 策略

- S. Chamorro, V. Klemm, M. de la Iglesia Valls, C. Pal, R. Siegwart, **Reinforcement Learning for Blind Stair Climbing with Legged and Wheeled-Legged Robots**, 2024. [arXiv:2402.06143](https://arxiv.org/abs/2402.06143)
  **与你们当前架构最接近的一篇**：单个 RL 控制器，用一个**布尔观测位**作为显式的爬楼梯模式开关，部署时完全不需要外感知。这正是"由路线标注翻转那一位"而不是"由实时地形分类器翻转"的架构。（这篇已在 RL 训练计划的参考列表里。）
- D. Hoeller, N. Rudin, D. Sako, M. Hutter, **ANYmal parkour: Learning agile navigation for quadrupedal robots**, *Science Robotics* 9(88):eadi7566, 2024. [doi:10.1126/scirobotics.adi7566](https://doi.org/10.1126/scirobotics.adi7566) · [arXiv:2306.14874](https://arxiv.org/abs/2306.14874)
  显式技能选择：多个专用策略，由高层导航策略按**各技能的能力边界**选择，并始终保留"走路"作为回退。
- J. Lee, M. Bjelonic, A. Reske, L. Wellhausen, T. Miki, M. Hutter, **Learning robust autonomous navigation and locomotion for wheeled-legged robots**, *Science Robotics* 9(89):eadi9641, 2024. [doi:10.1126/scirobotics.adi9641](https://doi.org/10.1126/scirobotics.adi9641) · [arXiv:2405.01792](https://arxiv.org/abs/2405.01792)
  **平台最接近 S10 的一篇**：全局规划 → 具备通行性意识的局部导航 → RL 运动策略三层，导航层只发速度指令，轮行/行走的切换由策略自己学。公里级实地任务。
- T. Miki et al., **Learning robust perceptive locomotion for quadrupedal robots in the wild**, *Science Robotics* 7(62):eabk2822, 2022. [doi:10.1126/scirobotics.abk2822](https://doi.org/10.1126/scirobotics.abk2822) · [arXiv:2201.08117](https://arxiv.org/abs/2201.08117)
  belief encoder 学习**该多信任外感知**，地图错误或被遮挡时平滑退回本体感知。这是"切换时机判断错了也只是退化、不是失效"的机制来源。

**关于切换门禁的一个诚实说明**：上述论文都没有给出形式化的切换门禁规范。它们共同的工程做法是——（1）有一个永远安全的默认策略作为回退，（2）只在低速、且在几何上明确定义的进入点切换，不在迈步中途切，（3）有一个感知置信度信号可以否决切换。**门禁是你们自己的工程，文献不会代劳。** 你们现有的"停稳 → 发一次 → 等新鲜回执 → 持续确认"次序已经满足前两条。

### 8.8 未能核实、因此没有引用的内容

- 关于"拓扑 vs 度量规划"常被引用的"约 6% 路径次优 / 100× 规划加速 / 6× 内存下降"，出现在 NavTopo 的二手摘要里，但**不在原文摘要中**，本次未在正文核实，因此没有作为事实引用。要引用数字请用 8.4 节 Nav2 Route Server 的基准表，那组数字是逐字核对过的。
- `docs.nav2.org` 改过 URL 结构，旧的 `/configuration/packages/...` 与 `/concepts/...` 路径现在 404，可用路径都在 `/rolling/` 下。
- AEDE 仓库的文件路径在 `noetic` 与 `melodic`/ROS 2 分支之间不同；8.3 节引用的路径来自 `noetic` 分支。
