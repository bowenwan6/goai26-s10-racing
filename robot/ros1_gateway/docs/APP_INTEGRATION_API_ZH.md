# S10 自动导航 — App 集成接口说明

写给做 App 集成的同学。2026-09-20 首次真机自动导航跑通（048 号狗，WP01→WP02，4.7 m）之后整理。

状态标记：✅ 真机验证过 · 🧪 只离线测过 · ❓ 未知

---

## 1. 先看这一段：最短集成路径

**App 不需要自己实现导航。** 最省事的集成方式是调用 AGX 上的一条命令：

```
ssh golai@10.21.33.102  bash ~/ros1_gateway/scripts/nav_run.sh --speed 0.5 --route short
```

它自己完成：检查传感器 → 选地图 → 发初始位姿 → 判断正向/返程 → 等站立 → 上电 → 切导航模式 → 开跑 → 每秒一行状态到 stdout → 结束后**一定**恢复常规模式（遥控器拿回控制权）。
中途停：`nav_session.sh stop`。

如果要更细的控制（按钮级），直接订阅/发布下面第 4 节的 ROS 1 话题。两种方式可以混用，但**同一时刻只能有一个导航运行**。

---

## 2. 拓扑：谁在哪台机器上

| 机器 | 地址 | 跑什么 | 归属 |
|---|---|---|---|
| **AGX Orin (102)** | `10.21.33.102`，另一网口 `<OFFSITE_NET_IP>` | ROS 1 master (11311)、x_nav 容器、传感器网关、运控桥接、导航节点、采集助手网页(8080)、遥控网页代理(8089/8090) | 我们的，可随意改 |
| **运控板 (103)** | `10.21.33.103` | 厂商 `motion_master` + `robot_server`（ASDU 服务端 30004）、你的遥控后端 `asdu_remote.py`(8090) / 管理页(8089) | 厂商 + 你的，**共用，用完要还原** |
| **定位板 (106)** | `10.21.33.106` | 厂商雷达驱动；我们放一个点云转发脚本（`~/ros1_gateway_tap/`，用完删除） | 厂商，**共用** |

ROS 1 master 在 AGX：`ROS_MASTER_URI=http://127.0.0.1:11311`（roscore 由 x_nav 容器起）。
AGX 上加载 ROS 1 环境：`source ~/ros1_gateway/ros1/ros1_env.sh`。

---

## 3. 最关键的一条：使用模式（踩过的坑）

✅ **`/NAV_CMD` 速度话题只在「导航模式」下生效。** 开发指南 2.3.1 原话。

我们第一次实验：狗在运动状态 17 + 平地导航步态，收到 1169 条速度指令，一动不动。原因就是使用模式还是 0（常规/遥控）。

| 使用模式 | 值 | 效果 |
|---|---|---|
| 常规（遥控） | 0 | 遥控器/轴指令有效，`/NAV_CMD` **被忽略** |
| 导航 | 1 | `/NAV_CMD` 有效；遥控器摇杆很可能失效 ❓（我们还没实测确认） |
| 辅助 | 2 | 未用 |

切换方式：ASDU 请求 `Type=0x00100002 / Command=0x00500002`，`Items.Mode = 0|1|2`，走 UDP `10.21.33.103:30004`。
我们的实现：`tools/asdu_mode.py`（纯 Python 标准库，可直接抄）。

```bash
bash ~/ros1_gateway/scripts/nav_session.sh usemode status   # 只读，返回 MotionState/Gait/HES/ControlUsageMode
bash ~/ros1_gateway/scripts/nav_session.sh usemode nav      # -> 1，成功打印 MODE_OK 1
bash ~/ros1_gateway/scripts/nav_session.sh usemode normal   # -> 0
```

⚠️ **端口加密：** 048 出厂 `enableTls = true`，30004 是 DTLS，明文发不进去。必须 `/var/opt/robot/conf/robot_server/Network.toml` 改成 `enableTls = false` 并**给狗断电重启**（改完不重启无效，我们踩过）。启动日志出现 `UdpTlsServer ... mode: plain` 才算生效。
**共用的狗，用完要改回 `true`**：`robot_session.sh tls restore`。

⚠️ **App 集成注意：** 你的遥控后端 `asdu_remote.py` 和我们的导航都往 30004 发。导航期间使用模式=1，你的摇杆轴指令会被狗忽略；我们 `stop` 的瞬间切回 0，你的页面若在自动接管会立刻重新生效。建议 App 里做**互斥**：导航运行时遥控页显示「导航中」并暂停自动接管。

---

## 4. ROS 1 话题接口（AGX，master 11311）

### 4.1 传感器（网关输出，✅ 稳定）

| 话题 | 类型 | 频率 | 说明 |
|---|---|---|---|
| `/LIDAR/POINTS` | `sensor_msgs/PointCloud2` | 10 Hz | 前后雷达合并点云，frame `lidar_link`，**已在 base_link 系**（外参为单位阵，实测确认） |
| `/IMU` | `sensor_msgs/Imu` | 200 Hz | 狗自身 IMU。**roll/pitch 用这个，不要用 x_nav 的**（x_nav 俯仰有 −3.5° 偏差，实测） |
| `/ODOM` | `nav_msgs/Odometry` | 10 Hz | 厂商里程计 |

### 4.2 定位（x_nav 输出）

| 话题 | 类型 | 频率 | 说明 |
|---|---|---|---|
| `/base_link/odom` | `nav_msgs/Odometry` | 10 Hz | SLAM 位姿。**只在建图或定位模式下才有** |
| `/x_nav/global_map`, `/x_nav/current_pointcloud` | PointCloud2 | — | 绿色地图 / 红色实时云，做可视化用这两个 |
| `/x_nav/slam/state` | String | — | SLAM 状态 |

⚠️ **x_nav 的 z（高度）不可靠：** 同一位置两次读数差 0.5 m。平面 x/y 正常。所以我们路线里把高度判定关掉（`--tol-z 5`）。做 3D 显示时别信 z。

### 4.3 x_nav 控制（这是操作地图的正规入口）

| 话题 | 类型 | 发什么 |
|---|---|---|
| `/node_cmd` | `std_msgs/String` | `launch_mapping#<地图名>` 开始建图<br>`launch_navigation#<地图名>` 切到该地图定位<br>`save_map#<地图名>` 保存地图<br>`exit_mapping` 退出 |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | 设置初始位姿，frame `map`。等同网页「设置位姿」✅ |

网页 `http://10.21.33.102:8000` 做的事就是发这些。App 可以直接发，不必嵌网页。

### 4.4 我们的导航节点（`nav/s10_rl_nav_ros1.py`）

**订阅（App 可发）：**

| 话题 | 类型 | 值 |
|---|---|---|
| `/rl_nav/cmd` | `std_msgs/String` | `start` 开始/继续 · `pause` 暂停（发零速） · `reset` 复位到路线起点 |

**发布（App 可订阅）：**

| 话题 | 类型 | 说明 |
|---|---|---|
| `/rl_nav/status` | `std_msgs/String`（JSON） | 每 tick 一条，20 Hz。**做 UI 主要看这个**，字段见下 |
| `/nav/progress` | `std_msgs/Float32`（latched） | 0.0–1.0 进度 |
| `/nav/finished` | `std_msgs/Bool`（latched） | 跑完置 true |
| `/rl_nav/cmd_vel` | `geometry_msgs/Twist` | 导航算出的速度（给控制节点，App 一般不用管） |
| `/rl_nav/gait_request` | `std_msgs/String` | `flat` / `stairs` |

`/rl_nav/status` JSON 字段：

```json
{"t":93.05,"shadow":false,"running":true,"pose_age":0.06,"obs_age":0.08,"obs_ms":44.3,
 "att":"imu","mode":"WALK","reason":"follower","maneuver":null,
 "cmd":[0.195,-0.027,-0.054],"owner_requested":"official",
 "gait_request":"flat","gait_reported":"flat",
 "target":"WP02","s":1.0,"d":-0.06,
 "follower":"DETOUR","follower_reason":"",
 "reached":1,"total":2,"progress":0.5,"why":"","obs_fresh":true}
```

| 字段 | 含义 | UI 建议 |
|---|---|---|
| `mode` | 运行模式，见下表 | 主状态大字 |
| `reached` / `total` | 已到达 / 总路点数 | `1/2` |
| `progress` | 0–1 | 进度条 |
| `target` | 当前目标路点 id | 「前往 WP02」 |
| `s` | 路线弧长位置（m） | |
| `d` | 横向偏离路线（m），正负=左右 | 超过 0.3 标黄，0.8 标红 |
| `cmd` | `[前进 m/s, 横移 m/s, 转向 rad/s]` | 小箭头 |
| `gait_reported` | 狗实际步态 `flat`/`stairs`/`switching` | 图标 |
| `pose_age` | 位姿多久没更新（s） | >0.3 报警 |
| `follower` / `follower_reason` | 路径跟随器内部状态，排障用 | 折叠区 |

`mode` 取值：

| mode | 含义 | 狗在动吗 |
|---|---|---|
| `WALK` | 正常沿路线走 | 是 |
| `TRACK` | 偏离路线，第一版**原地不动**等人 | 否 |
| `APPROACH` / `ALIGN` | 接近路点 / 对正朝向 | 是（慢） |
| `CLIMB` / `DESCEND` | 上 / 下台阶 🧪 | 是（0.15 m/s） |
| `DETOUR` / `RECOVER` / `BACKUP` | 绕行 / 恢复 / 后退 | 是 |
| `WAIT` / `HOLD` | 停下等人 | 否 |
| `DONE` | 跑完 | 否 |

⚠️ **`HOLD` 之后单独发 `start` 不会继续**，必须重新 `arm`（这是第一版故意的设计）。

### 4.5 运控桥接节点（`s10_ros1_control`，ROS 1 ↔ 厂商 ROS 2）

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/s10_control/state` | String（JSON） | 出 | 5 Hz，桥接和狗的状态，字段见下 |
| `/s10_control/gait` | String | 出 | 10 Hz，`flat`/`stairs`/`switching`/`none` |
| `/rl_nav/cmd_vel` | Twist | 入 | 速度源（配置里可改成别的话题） |
| `/rl_nav/gait_request` | String | 入 | `flat`/`stairs`，节点负责「停稳→切换→确认」整套流程 ✅ |
| `/web_cmd` 或 `/s10_control/web_cmd` | String | 入 | `cmd4` 起立→RL→导航步态 · `cmd3` 趴下 · `Nav stop`/`Nav continue` · `source <名字>` 切速度源 |

`/s10_control/state` 关键字段：

```json
{"enable_motion":true,"publishers_created":true,"fault":"","latched_stop":false,
 "sequence":"none","moving":false,"cmd_source":"rl_nav","gait":"flat","gait_request":"flat",
 "feedback":{"fresh":true,"age_s":0.02,"state":17,"gait":12290,"vel":[0,0,0],"height":0.419},
 "cmd_vel":{"age_s":0.006,"in":[0,0,0],"out":[0,0,0],"received":1918,"rejected":0,"clamped":0,
            "ignored":{"rl_nav":0}},
 "nav_cmd_sent":1169,"nav_cmd_publishers":0,"nav_cmd_foreign_1s":0,"nav_cmd_foreign_total":0,
 "limits":[0.5,0.2,0.8]}
```

- `enable_motion`：false = 只读模式（默认），true = 已上电能发指令
- `fault`：非空 = 已自锁，必须重启节点
- `feedback.state`：狗运动状态，`17` = RL 控制（能走），`1` 站立，`4` 趴下，`0` 空闲
- `feedback.gait`：`12290`=0x3002 平地导航，`12291`=0x3003 楼梯导航，`4097`=0x1001 遥控普通
- `limits`：当前限速 `[前进, 横移, 转向]`

⚠️ **独占保护：** 节点检测到**别的程序**也在发 `/NAV_CMD` 就立刻自锁停机（`fault`）。App 不要自己发 `/NAV_CMD`。

---

## 5. 厂商原生接口（开发指南摘要）

### 5.1 ROS 2 话题（drdds，域 0，在 103 上）

| 话题 | 类型 | 说明 |
|---|---|---|
| `/NAV_CMD` | `drdds/msg/NavCmd` | `x_vel`(±1.67 m/s) `y_vel`(±0.4) `yaw_vel`(±1.0 rad/s)，**10 Hz 固定频率**，仅导航模式生效 |
| `/MOTION_STATE` | `drdds/msg/MotionState` | 1 站立，4 趴下，17 RL 控制，2 阻尼（**永远别发**） |
| `/GAIT` | `drdds/msg/Gait` | `0x3002` 平地导航步态，`0x3003` 楼梯导航步态 |
| `/MOTION_INFO` | `drdds/msg/MotionInfo` | 20 Hz 反馈：状态、步态、速度、机身高度 |

⚠️ 切步态**必须先停稳**（厂商要求）。我们的控制节点已实现：归零 → 等静止 ≥1 s → 发 `/GAIT` → 等反馈确认 → 恢复。✅

### 5.2 ASDU 服务协议（UDP 30004）

APDU = 16 字节头 + JSON/XML 正文。头：同步字 `eb 91 eb 90` + 长度(LE) + 消息id + 格式(0x01=JSON) + 包id + 版本(0x01) + 5 字节保留。

| 用途 | Type | Command |
|---|---|---|
| 心跳（≥1 Hz，发了才会收到状态上报） | `0x00100064` | `0x00000005` |
| 状态上报（狗主动发给心跳来源 IP，约 2 Hz） | — | `0x00f00000` |
| **使用模式切换** | `0x00100002` | `0x00500002` |
| 运动状态转换 | `0x00100001` | `0x00200002` |
| 真实轴指令（仅导航模式） | `0x00100001` | `0x00110002` |

状态上报字段：`MotionState`、`Gait`、`HES`（硬急停）、`ControlUsageMode`、`Model`、`Charge`。

实现参考：`tools/asdu_mode.py`（115 行，纯标准库）。你的 `asdu_udp_control.py` 已有同样的编解码。

---

## 6. 采集助手 HTTP API（建图/标点/示教，`:8080`）

网页 `http://10.21.33.102:8080/teach`。后端 `teach_worker.py` 在 `127.0.0.1:8091`，前面由 `server.py` 加登录和 CSRF。

| 方法 | 路径 | 说明 |
|---|---|---|
| POST | `/phone/login` | 登录，拿 cookie + csrf |
| GET | `/phone/teach/status` | 状态：位姿/点云/IMU 频率、磁盘、当前录制、轨迹点、已标点位 |
| GET | `/phone/teach/sessions` | 会话列表 |
| GET | `/phone/teach/file?session=&name=` | 下载会话内文件 |
| POST | `/phone/teach/submit` | 动作，header 带 `X-CSRF-Token`，body 见下 |

`submit` 的 action：

| action | 参数 | 作用 |
|---|---|---|
| `session_new` | `label`, `map_name` | 新建会话 |
| `session_open` | `session_id` | 打开已有会话 |
| `record_start` | `mode`: `mapping`\|`survey`\|`path` | 开始录制。`mapping` 录点云+IMU+位姿（~1.5 GB/分钟），另两个只录位姿 |
| `record_stop` | — | 停止 |
| `mark` | `kind`: `WP`\|`SWIN`\|`SWOUT`\|`PATH_START`\|`PATH_END`\|`NOTE`, `wp_id`, `note` | 打点。采样 3 s，位置抖动 ≤2 cm 且朝向 ≤1° 才算通过 |
| `redo` | — | 作废上一个点 |
| `trail_clear` | — | 清空轨迹显示 |

数据落盘：`<data_dir>/sessions/<YYYYMMDD-HHMMSS-标签>/`
- `marks.jsonl` — 每行一个标记，含 `seq/kind/wp_id/pose[x,y,z,yaw]/result{passed,std_xy,yaw_std_deg}`。**每条立即 fsync** ✅
- `<mode>_<时间>.trail.csv` — `wall_time,x,y,z,yaw`，每 5 cm 一行，**每 2 s fsync** ✅
- `<mode>_<时间>_N.bag` — rosbag，2 GB 分卷
- 存储位置：插了 SSD 就写 `/mnt/s10ssd/s10_teach`，否则 `~/teach`。**插拔后要重启 worker**

---

## 7. 路线文件格式 `route_v2.json`

由示教会话生成：`nav_session.sh route <session> --map-id <地图> --tol-z 5`

```json
{"schema":"s10_route_v2","map_id":"v6_room","frame":"map","z_reference":"ground",
 "waypoints":[
   {"id":"WP01","position":[-0.038,-0.024,0.049],"yaw":0.1198,
    "radius_xy":0.2,"tol_z":5.0,"terrain":"","confidence":"high",
    "mark_seq":1,"mark_std_xy":0.0034}],
 "segments":[
   {"id":"WP01-WP02","from":"WP01","to":"WP02","gait":"flat","speed_limit":0.7,
    "allow_detour":true,"corridor_half_width":0.8,"length_m":5.0,
    "centerline":[[x,y,z], ...]}]}
```

- `position` 的 z 是**地面高度** = 位姿 z − 站立高度（048 实测 0.41 m）
- `radius_xy` 到点判定半径，默认 0.2 m（实测停下时超调约 0.02 m）
- `gait` 为 `stairs` 的段限速 0.15 m/s 且禁止绕行
- 配套 `maneuvers.json` 描述台阶区（按路线弧长）

**返程：** 把 waypoints/segments/centerline 全部反转、yaw 加 π 即可，`scripts/nav_run.py` 的 `reverse_route()` 就是这么做的（🧪 仅平地路线，下台阶不支持）。

---

## 8. 命令行入口（App 可以直接 exec）

AGX 上，`~/ros1_gateway/scripts/`：

| 命令 | 作用 |
|---|---|
| `nav_run.sh [--speed 0.5] [--route short\|full\|<dir>] [--at start\|end] [--shadow]` | **一条命令跑完整轮**，stdout 每秒一行 |
| `nav_session.sh route <session> --map-id <map> [--tol-z 5] [--flat-speed 0.5] [--last-wp WP02]` | 示教 → 路线 |
| `nav_session.sh shadow <route>` | 影子模式（只算不动） |
| `nav_session.sh arm <速度\|zero\|probe\|flat> <route>` | 上电，导航处于暂停 |
| `nav_session.sh usemode status\|nav\|normal` | 使用模式 |
| `nav_session.sh go` / `pause` / `stop` | 开始 / 暂停 / 结束并恢复 |
| `nav_session.sh watch` | 每秒一行状态 |
| `health_check.sh` | 传感器健康 |

Mac 上 `robot_session.sh nav|navstop|up|down|status|keys|unkeys|autostart|tls`。

`arm` 的速度档：`zero` = 0/0/0（全链路上电但动不了，用于自检）、`probe` = 0.10/0/0.30、`flat` = 0.20/0.10/0.50、或直接给数字如 `0.5`（同时设控制限幅和导航行走速度）。硬上限 1.0 m/s。

---

## 9. 实测数据与已知限制

**✅ 2026-09-20 首次自动导航（048）**
- WP01 → WP02，4.7 m，限速 0.10 m/s，用时 38 s，平均 0.12 m/s
- 终点误差 0.22 m（到点半径 0.2 m，进圈即判定，之后滑行一小段）
- 全程无故障，`DONE` 后自动停

**站立高度：** 048 实测 0.414 m（地面拟合）/ 0.419 m（狗上报）。趴下 0.07 m。**换狗要重测**：`tools/check_ground_plane.py`

**❌ 已知限制（第一版设计如此，不是 bug）**
1. **没有障碍物检测** — 没有 `map_surface.npz`，路线上不能有人和物
2. **雷达盲区 0.6 m** — 狗身下和身前看不到地面，我们用最近已知地面高度填充（`blind_radius: 1.1`）。盲区内的小物体检测不到
3. **`TRACK` 模式原地不动** — 偏离路线时不会自己绕回，等人用遥控器推回去
4. **x_nav 无全局重定位** — AGX 重启后必须人工设初始位姿，或把狗放回路线端点
5. **台阶段 🧪 从没在真台阶上跑过** — 逻辑只在仿真验证
6. **遥控器在导航模式下是否失效 ❓** — 还没实测

**⚠️ AGX 会自己重启** — 电源记录显示重启前 5 V/19 V 电压平稳、温度 51 °C，判断是外部断电或复位线触发（怀疑供电接头松动）。重启后我们的服务自动恢复，但 x_nav 要重新选图设位姿。

---

## 10. 给 App 的建议

**必须做的互斥**
- 导航运行期间，遥控页面要停止自动接管（使用模式 1 下摇杆无效，但 `stop` 切回 0 的瞬间会立刻生效）
- 不要让两个导航同时跑（控制节点会自锁）
- 急停按钮请直接调厂商硬急停或 `/s10_control/web_cmd` 发 `cmd3`（趴下），不要只依赖 `pause`

**页面建议**
1. **状态页** — 订阅 `/rl_nav/status` + `/s10_control/state`，显示 mode、进度、偏差 `d`、位姿年龄
2. **地图页** — `/x_nav/global_map`（绿）+ `/x_nav/current_pointcloud`（红）+ 路线 centerline + 狗当前位姿
3. **采集页** — 直接用现成的 `/teach`，或调它的 HTTP API 自己做
4. **一键跑** — exec `nav_run.sh`，把 stdout 逐行推给前端

**别碰的**
- x_nav 网页上的「单点导航」「动作指令」「方向键」——它自己的规划器，会和我们抢 `/cmd_vel`
- `/NAV_CMD`——只能由控制节点发
- 103 上的厂商服务和配置（除了已知的 `enableTls`，且用完要还原）

---

## 11. 代码位置

| 内容 | 路径 |
|---|---|
| 网关 / 控制节点 / 导航节点 / 脚本 | AGX `~/ros1_gateway/`，Mac `s10-real-readiness/ros1_gateway/` |
| 规划核心（纯 Python） | `nav/s10_auto_nav/`（主仓 `src/s10_auto_nav`，分支 `rl/maneuver-router`，commit e1aaf73）|
| 采集助手网页 | AGX `~/s10_mapping_web/`，Mac `s10-field-assistant/tools/s10_mapping_web/` |
| 你的遥控后端 | 103 `~/golai/` |
| 更详细的背景 | `docs/HANDOFF_S10_AUTONOMY_STACK.md`（英文，~425 行） |
| 实验步骤 | `docs/EXPERIMENT_048_ZH.md`、`docs/ROOM_NAV_RUNBOOK_ZH.md` |

有不清楚的直接问，或者看 `nav_run.py`（270 行，整个流程都在里面，最好的参考）。
