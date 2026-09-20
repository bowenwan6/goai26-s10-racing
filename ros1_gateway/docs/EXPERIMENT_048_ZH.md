# 048 室内自动导航实验（2026-09-20）

路线已建好：`~/routes/20260920-175142-v6_room-short`（WP01 → WP02，5 m，纯平地）。
完整路线（含上台阶，今天不带动作跑）：`~/routes/20260920-175142-v6_room`。

## 一条命令跑完（推荐）

在 **Mac** 上，任何目录：

```bash
bash ~/Documents/ChatGPT/GOAI/s10-real-readiness/ros1_gateway/scripts/robot_session.sh nav --speed 0.5
```

它自动做：检查点云 → x_nav 没在定位就自动选地图 → 没有位姿就自动发初始位姿（狗要站在路线起点 WP01、朝着路线；放在终点就加 `--at end`）→ 判断狗在路线起点还是终点（在终点就自动走**返程**，仅平地路线）→ 等狗站立 → 上电 → 切导航模式 → 开始 → 每秒一行状态 → 结束 / HOLD / 故障 / Ctrl-C 时**一律**停导航、撤销上电、切回常规模式（遥控器恢复）。

- `--route short`（默认，WP01↔WP02 平地）或 `--route full`（到 WP03，含台阶；只能正向）。
- `--speed` 是前进限速 m/s（0.1–1.0）。`--shadow` 只看不动。
- 另一个终端里随时可以停：`robot_session.sh navstop`。紧急情况用遥控器急停。
- 定位一旦在跑（AGX 没重启、地图没换）就不会再去动它；只有 AGX 重启后第一次运行才需要狗站在路线端点上。

下面是分步做法（排查问题时用）。命令都在 AGX 上执行（`ssh s10-48-remote`）。一人全程拿遥控器，站在狗的侧面，前方留空。

---

## 0. 门槛检查（不通过就不要往下做）

```bash
bash ~/ros1_gateway/scripts/nav_session.sh usemode status
```

- **通过：** 打印出 `MotionState=… Gait=… ControlUsageMode=0 …` 这样的状态行。
- **不通过：** 打印 `NO_STATUS`。说明 103 的服务端口还在加密，狗切不了导航模式，`/NAV_CMD` 不会生效（开发指南 2.3.1）。见文末“端口加密”。

> 2026-09-20 19:11 检查结果：**不通过**。048 的 `Network.toml` 仍是 `enableTls = true`，`robot_server` 19:09 的启动日志仍写“明文 TCP/UDP 端口已屏蔽”，103 上最近 40 分钟没有任何配置文件被改过。

## 1. 上电后的准备

| 步骤 | 做什么 | 成功标志 |
|---|---|---|
| 1.1 | 狗用遥控器站起来，放在建图起点（WP01），朝着 WP02 | 站稳 |
| 1.2 | x_nav 页 `http://10.21.33.102:8000` → 地图选择 → 切换 `v6_room` → 刷新点云 | 绿色地图出现 |
| 1.3 | 设置位姿：点左侧“设置位姿”，在狗的真实位置按住鼠标，朝狗头方向拖一小段松开。狗在建图起点时就是图中心、朝建图时的前进方向 | 红色实时点云和绿色地图重合，位置信息约 10 hz |
| 1.4 | `bash ~/ros1_gateway/scripts/health_check.sh` | `HEALTH_OK`，点云 10 Hz |

AGX 每次重启后 1.2、1.3 都要重做。

## 2. 影子运行（不发运动指令，约 2 分钟）

```bash
bash ~/ros1_gateway/scripts/nav_session.sh shadow ~/routes/20260920-175142-v6_room-short
bash ~/ros1_gateway/scripts/nav_session.sh watch
```

- 狗站着不动时：前几秒应显示 `WALK`，`cmd` 约为 `[0.18, 0.09, 0.18]`（前进 + 小幅修正）。约 8 秒后变 `HOLD` 是正常的（没进展就停）。
- 用遥控器沿路线走到 WP02：`wp` 应从 `1/2` 变 `2/2`。
- 一直显示 `BLOCKED` 或 `d` 大于 0.5：定位没对上，回到 1.3。

## 3. 带动作运行

每一档都是同样的四条命令，只换档位名：

```bash
bash ~/ros1_gateway/scripts/nav_session.sh arm probe ~/routes/20260920-175142-v6_room-short
bash ~/ros1_gateway/scripts/nav_session.sh usemode nav      # 必须看到 MODE_OK 1
bash ~/ros1_gateway/scripts/nav_session.sh go
bash ~/ros1_gateway/scripts/nav_session.sh stop             # 结束：停导航、撤销上电、切回常规模式
```

| 档 | 限速（前进 / 横移 / 转向） | 通过标准 |
|---|---|---|
| `probe` | 0.10 m/s / 0 / 0.30 rad/s | 狗朝 WP02 走，`s` 在增加，到 WP02 停下，`watch` 显示 `DONE` |
| `flat` | 0.20 / 0.10 / 0.50 | 同上，全程横向偏差 `d` 小于 0.3 m |

另开一个终端一直开着 `watch`；再开一个终端，预先敲好下面这条不回车：

```bash
bash ~/ros1_gateway/scripts/nav_session.sh pause
```

**安全：**

- **导航模式下遥控器摇杆很可能不起作用**（指南：常规模式才执行轴指令）。紧急时用遥控器的急停 / 趴下键，或回车执行 `pause`，然后 `stop`。
- `arm` 本身不会让狗动；`go` 之后才动。
- `watch` 出现 `fault=…`：有别的程序在发运动指令，控制节点已自锁。执行 `stop`，不要重试，把输出发给我。
- `HOLD`：8 秒内前进不到 0.3 m，导航自己停了。要重新 `arm` 才会再走，单独 `go` 不会解除。
- `pose jump`：定位跳变。确认红绿点云重合后 `go` 继续。
- 没有障碍物检测，狗身前 0.6 m 内是雷达盲区：路线上不能有人和物。

## 4. 这次要记录的结果

1. `usemode nav` 之后，`probe` 档狗动不动。
2. 导航模式下遥控器摇杆是否还有效（狗停着时轻推一下试）。
3. 走直线时执行 `pause`，狗多远停下。
4. 到 WP02 的最终误差（`watch` 最后一行的 `s`、`d`）。

出问题时发我：`watch` 的最后几行，以及 `~/ros1_gateway/logs/` 里最新的 `nav-*.log`、`control-events-*.jsonl`。

## 5. 收工

```bash
bash ~/ros1_gateway/scripts/nav_session.sh stop
bash ~/ros1_gateway/scripts/nav_session.sh usemode status   # 确认 ControlUsageMode=0
```

还狗时在 Mac 上再执行 `robot_session.sh down` 和 `unkeys`。

---

## 端口加密（第 0 步不通过时）

现状（只读检查）：

- `/var/opt/robot/conf/robot_server/Network.toml`：`enableTls = true`。文件注释写明 `false` = 明文模式，改后重启 `robot_server` 生效，也可以在 `user_conf.toml` 里覆盖。
- 启动日志：“明文 TCP/UDP 端口已屏蔽（ENABLE_PLAIN_TCP_UDP 未定义），仅启用 TLS/DTLS”。括号里的说法像是编译选项，所以**只改这个开关不一定够**，开发指南也写着要联系技术支持。
- 050 上明文是通的，我们的工具和朋友的遥控网页都是在 050 上验证的。

可选做法：

1. 换回 050 做带动作的实验（最快，零改动）。
2. 由负责 048 的人关掉加密：先备份 `Network.toml`，把 `enableTls` 改为 `false`（或在 `user_conf.toml` 里加 `[Network]` `enableTls = false`），然后**给狗断电重启**（不要单独停 `robot_server`）。重启后再跑第 0 步。048 是共用的，用完要把文件还原。
3. 看厂商手柄 / App 里有没有“导航模式”开关。有的话切过去，跳过 `usemode nav`，直接 `arm` → `go`。

我不会自己去改 103 上的厂商配置或重启厂商服务；需要我准备“改动 + 还原”各一条命令的话告诉我。
