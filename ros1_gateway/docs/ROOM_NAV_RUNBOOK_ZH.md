# 室内试验步骤：建图 → 定位 → 标点 → 步态切换点 → 示教 → 自动导航

适用：AGX（102）接在狗上，103/106 在线。全程一人拿遥控器，随时可以接管。
电脑上的命令都在 Mac 的 `ros1_gateway/` 目录下执行；写着“AGX 上”的命令先 `ssh s10-48-remote`。

状态说明：✅ 已在机器上验证，🧪 只做过离线测试，❓ 要靠这次试验回答。

---

## 0. 每次上电后（约 2 分钟）

| 步骤 | 在哪 | 命令 | 看到什么算成功 |
|---|---|---|---|
| 0.1 | Mac | `bash scripts/robot_session.sh status` | 打印 106 的序列号。**先确认是哪台狗** |
| 0.2 | Mac，**换狗或还狗后只做一次** | `bash scripts/robot_session.sh keys`，再 `bash scripts/robot_session.sh autostart` | 三行 `key login OK`；`AGX to 106 key login OK`。密码由你在终端输入，不保存 |
| 0.3 | Mac | `bash scripts/robot_session.sh up` | 最后打印 `HEALTH_OK`、`teach worker running`、`x_nav container: nav Up` |
| 0.4 | 手机或 Mac | 打开 `http://10.21.33.102:8080/teach` | 顶部三个灯（位姿 / 点云 / IMU）变绿。位姿灯要等 x_nav 进入建图或定位后才会绿 |

- AGX 开机会自己启动 x_nav、网关、控制节点（只读模式）、采集后台。`up` 之后 106 上的点云转发也会开机自启。
- 不在机器人 Wi-Fi 里时，Mac 上跑 `bash scripts/mac_tunnel.sh`，然后用 `http://localhost:18080/teach` 和 `http://localhost:18000`。
- **硬盘：** 页面右上角显示“硬盘剩余”才是录到外接 SSD。显示红色“未接硬盘”时是录到 AGX 自身，建图每分钟约 1.5 GB。插拔硬盘后要在 AGX 上执行 `bash ~/s10_mapping_web/teach-worker.sh stop; bash ~/s10_mapping_web/teach-worker.sh start`。

**断网不影响建图和录制。** ✅ x_nav 建图在 AGX 的容器里跑，录包由 AGX 上的后台进程做，都不挂在浏览器或 SSH 上（已确认：进程没有终端，用户会话常驻）。手机、Mac 断开后重新打开页面即可，录制一直在继续。轨迹和标点现在每 2 秒落盘一次，AGX 突然断电最多丢 2 秒。

---

## 1. 建图（x_nav）

1. 狗用遥控器站起来，停在起点。**起点 = 以后地图的原点，记住这个位置和朝向。**
2. 浏览器打开 x_nav：`http://10.21.33.102:8000` → 地图选择 → 新增地图 → 输入名字（例如 `room1`）→ **回车**。红色点云开始出现。
3. `/teach` 页 → 新建会话（地图名填同一个名字）→ ① 建图 → 开始采集。这一步是备份原始点云，不是 x_nav 建图必需的。
4. 用遥控器慢走（约 0.3 m/s），转弯慢一点，走一圈回到起点，让首尾重合。室内 2–5 分钟足够。
5. 回到起点站稳 → x_nav 页：地图选择 → **保存地图**。**没按保存之前地图不会落盘，AGX 一重启就没了，走完立刻保存。**
6. `/teach` 页 → 结束采集。

不需要“导入”：x_nav 的地图是边走边建的，保存后就在 AGX 的 `/opt/data/nav_map/room1/`。录下来的包只是备份。

## 2. 切到定位

1. x_nav 页：地图选择 → 刷新地图 → 选 `room1` → 刷新点云（绿色 = 已保存的地图）。
2. 设置位姿：在图上点狗的真实位置，拖出朝向。红色（实时）和绿色（地图）点云要重合。
3. 用遥控器走几米再看一次，仍然重合才算定位成功。`/teach` 页的位姿灯应为绿色。

**之后的标点、示教、导航都必须在“定位”状态下做**，这样坐标才在同一张地图里。不要用 x_nav 页上的 单点导航 / 动作指令 / 方向键。

## 3. 标点（WP）

`/teach` 页 → ②标点（会自动开始一段小录制）。

1. 遥控器把狗开到第 1 个点，**停稳**。
2. 按“标 WP01（停稳 3 秒）”。页面采样 3 秒，显示通过 / 不通过（位置抖动 ≤ 2 cm、朝向 ≤ 1° 才通过）。不通过就站稳再按一次，以最后一次通过的为准。
3. 编号自动递增，依次 WP02、WP03…。标错了按“作废上一个”。室内试验 3–5 个点就够，点间距 2–5 m。
4. 狗的朝向也会被记录，标点时让狗朝着下一个点的方向。

## 4. 步态切换点（SWIN / SWOUT）

只在路线上有需要“楼梯步态”的地段时才标。纯平地的房间试验可以整段跳过。

1. 走到该地段**之前**约 0.5 m，停稳 → 按“▲ 进台阶”（记为 SWIN）。
2. 走过该地段，到**之后**约 0.5 m，停稳 → 按“▼ 出台阶”（记为 SWOUT）。全部标完按“结束标点”。
3. SWIN 和 SWOUT 必须成对。导航时狗会在 SWIN 前停稳 ≥ 1 秒再切步态（厂商要求静止切换），SWOUT 后再停稳切回平地步态。

第一次带动作的试验建议**不标**切换点，先把平地跑通；第二轮再加一对来验证切换。

## 5. 示教路径

`/teach` 页 → ③示教。

1. 把狗开回 WP01，停稳 → 开始示教。
2. 用遥控器按你希望的路线依次经过每个 WP（以及 ▲ → ▼），到最后一个 WP 停稳 → 结束示教。
3. 页面上能看到走过的轨迹线。走歪了就再示教一遍，生成路线时用最后一次。

## 6. 生成路线（AGX 上）

```bash
bash ~/ros1_gateway/scripts/nav_session.sh route latest --map-id room1
```

- 输出在 `~/routes/<采集编号>/`：`route_v2.json`、`maneuvers.json`、`report.json`。
- 看打印的 `report`：`waypoints` 数量对不对，`length_m` 是否合理，`warnings` 和 `skipped` 是否为空。

## 7. 影子运行（不发任何运动指令）🧪

```bash
bash ~/ros1_gateway/scripts/nav_session.sh shadow
bash ~/ros1_gateway/scripts/nav_session.sh watch
```

用遥控器沿路线走一遍，看 `watch` 每秒一行：

- `wp 1/4 → 2/4 …` 逐个到达；`d`（离路线的横向距离）保持在 0.3 m 内。
- `cmd=[vx, vy, wz]` 的方向合理：狗偏左时应该给出向右修正。
- `gait flat/…` 在切换点前后按预期变化（影子模式只显示，不会真的切）。

这一步通过，说明位姿、点云、路线、到点判定整条链是通的。任何一项不对，先不要往下做。

## 8. 带动作的运行 ❓（第一次，人在旁边，手在遥控器上）

三档限速，逐档升级，每档都从 `arm` 开始：

| 档 | 限速（前进 / 横移 / 转向） | 目的 |
|---|---|---|
| `zero` | 0 / 0 / 0 | 全链路上电但狗动不了：确认狗进入导航状态、指令在发、没有故障 |
| `probe` | 0.10 m/s / 0 / 0.30 rad/s | 第一次真的动。走 1–2 m |
| `flat` | 0.20 / 0.10 / 0.50 | 完整走一遍房间路线 |

```bash
bash ~/ros1_gateway/scripts/nav_session.sh arm zero     # 或 probe / flat；此时导航是“暂停”的，狗不会动
bash ~/ros1_gateway/scripts/nav_session.sh usemode nav  # 把狗切到“导航模式”：开发指南 2.3.1 写明 /NAV_CMD 只在导航模式下生效
bash ~/ros1_gateway/scripts/nav_session.sh watch        # 另开一个终端看状态
bash ~/ros1_gateway/scripts/nav_session.sh go           # 开始
bash ~/ros1_gateway/scripts/nav_session.sh pause        # 随时暂停（发零速）
bash ~/ros1_gateway/scripts/nav_session.sh stop         # 每次试验结束都执行：停导航、撤销上电、控制节点回到只读
```

- 开始前把狗放在 WP01 附近（1 m 内），朝向大致对着路线。
- `stand` 之后看 `watch` 里 `robot_state=17`、`gait …/flat` 才算进入了导航状态。如果狗已经用遥控器站着但 `robot_state` 不是 17，用 `navmode` 代替 `stand`。
- **紧急情况：直接用遥控器接管或按遥控器上的停止。** `pause` 和 `stop` 是正常手段，不是急停。
- `arm` 后如果 `watch` 显示 `fault=...`：说明有别的程序也在发运动指令，控制节点已自锁，执行 `stop` 后查原因，不要重试。

这次试验要回答的两个问题：

1. ✅ 已回答（2026-09-20，048）：狗在状态 17 + 平地导航步态下收到 1169 条速度指令也不动。开发指南 1.2.2 / 2.3.1：必须先用 ASDU“使用模式切换”切到导航模式（1）。`usemode nav` 做这件事，`stop` 会切回常规模式（0）。狗站起来用遥控器即可（站好后就是状态 17），平地导航步态由我们的步态请求自动切换。
2. ❓ 我们的指令中断后狗会不会自己停。在 `probe` 档直线行走时，在 AGX 上执行 `kill -9 $(cat ~/ros1_gateway/run/control.pid)`，看狗在多长距离内停下。**人站在狗前方 3 m 以外的侧面，前方留空。**

已知的限制（第一版就是这样，不是故障）：

- 没有障碍物检测：路线上不能有人和物。
- 状态 `TRACK` 时狗原地不动（第一版故意如此）。用遥控器把狗推回路线附近再 `go`。
- 状态 `HOLD`：导航认为需要绕行或定位不可信，停下等人。`pose jump`：定位跳变，确认红绿点云重合后 `go` 继续。
- 站立高度用的是 0.27 m 的估计值。狗站稳后在 AGX 上跑一次 `source ~/ros1_gateway/ros1/ros1_env.sh; ROS_MASTER_URI=http://127.0.0.1:11311 python3 ~/ros1_gateway/tools/check_ground_plane.py`，把打印的离地高度告诉我，我更新配置。
- AGX 曾经自己重启（原因未明，电源记录器在记）。重启后一切会自动起来，但**x_nav 要重新选地图、重新设置位姿**。

## 9. 收工

```bash
bash ~/ros1_gateway/scripts/nav_session.sh stop     # AGX 上
bash scripts/robot_session.sh down                   # Mac 上：停掉并删除 106 上的点云转发
bash scripts/robot_session.sh unkeys                 # 还狗时才做：删掉我们在 103/106 上的公钥
```

拔外接硬盘前，在 AGX 上用管理员账号执行：`sync; sudo umount /mnt/s10ssd; sudo udisksctl power-off -b /dev/sda`。

## 10. 出问题时发给我的东西

- `bash ~/ros1_gateway/scripts/nav_session.sh watch` 的几行输出。
- `~/ros1_gateway/logs/` 里最新的 `nav-*.log`、`control-*.log`、`control-events-*.jsonl`。
- `~/routes/<编号>/report.json`。
