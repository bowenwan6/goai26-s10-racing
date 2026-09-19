# S10 ROS 1 网关与运控 SDK

2026-09-19。为 SLAM 厂商（x_nav，ROS 1 Noetic）提供机器人的雷达、IMU 和里程计，并把导航输出的 `/cmd_vel`、`/web_cmd` 转换成 S10 原生运动指令。

**当前状态：** 代码、离线测试和 AGX 原生部署已完成。**尚未在接通机器人网络的状态下做实机验收，也没有做过任何运动测试。**

## 1. 组成

```text
106 定位板（厂商系统，不改）                      102 AGX：/home/golai/ros1_gateway（golai 用户空间）
rsdriver ─SHM─> s10_lidar_tap ──TCP 47631──────> s10_ros1_gateway ──> ROS 1 /LIDAR/POINTS
yesense  /IMU  ───────────── DDS ──────────────>   (ros1_bridge 转换)──> ROS 1 /IMU
localization /ODOM ───────── DDS ──────────────>                   ──> ROS 1 /ODOM
                                                  roscore :11311
103 运控 <── /NAV_CMD /MOTION_STATE /GAIT ──────── s10_ros1_control <── ROS 1 /cmd_vel、/web_cmd（x_nav）
         ── /MOTION_INFO ─────────────────────────>
```

| 组件 | 作用 | 位置 |
|---|---|---|
| `s10_ros1_gateway` | ROS 2 → ROS 1 **单向**转换；绝不向 ROS 2 发布 | AGX |
| `s10_lidar_tap.py` | 106 的点云只在本机发布（厂商设计），tap 只读订阅后经 TCP 转给网关 | 106 `/home/user/ros1_gateway_tap/` |
| `s10_ros1_control` | ROS 1 起立/趴下/停止/速度 → S10 原生话题，带安全检查 | AGX |
| ROS 1 运行环境 | ROS-O（Noetic 系列，roscpp 1.17.3）解压在用户目录，不需要 sudo | AGX `ros1/` |

## 2. 为什么这样设计（调查证据）

- 106 厂商更新日志：“开启激光雷达驱动仅所在主机发布dds数据”；雷达配置 `dds.network_interface: ""`。102 用 DDS 收不到 `/LIDAR/POINTS`（实测 10 秒 0 条），`/IMU`、`/ODOM` 可以（200 Hz / 10 Hz）。因此点云用 106 上只读 tap 转发，其余两路直接走 DDS。
- `/LIDAR/POINTS` 有两个发布者：`./rslidar`（rsdriver）和 `./hsLidar`（禾赛 SDK V2.2.2，多品牌互斥的待机驱动），由 DDS GUID 中的进程号对应得出。
- AGX 自带的 rslidar_sdk（v1.5.19，`78d2abb`，ROS 2 构建，`POINT_TYPE=XYZI`）输出没有 ring/timestamp，不能替代厂商点云。
- AGX 没有 Docker，golai 没有 sudo；安装 Docker 会改动转发防火墙规则。因此 ROS 1 采用用户空间运行环境。
- ros1_bridge 官方声明不支持 Ubuntu 24.04 + Jazzy。实测：master `611755f` 在 Jazzy + ROS-O（arm64）上编译通过，并通过下文所有测试。
- 厂商给的 `ros1_humble_bridge:3.0.0.arm` 是另一台机器人的**运动控制**桥（/cmd_vel、电机/关节指令，domain 69、CycloneDDS），不含我们的传感器话题，**不要部署**。

## 3. 转换保证

- header 时间戳、`frame_id`、`child_frame_id` 原样保留；不伪造 TF；IMU 的空 `frame_id` 原样保留。
- PointCloud2 的 fields（x,y,z,intensity,ring@16 uint16,timestamp@18 float64）、`point_step=26`、`row_step`、`is_dense`、`is_bigendian` 与二进制 data 逐字节不变。
- 转换使用 ros1_bridge 生成的代码（`convert_2_to_1_generic`），不逐点解码。
- ROS 1 的 `Header.seq` 在 ROS 2 中不存在，固定为 0，**不能用它判断丢包**。
- ROS 1 消息 md5：PointCloud2 `1158d486…`、Imu `6a62c6da…`、Odometry `cd5e73d1…`，与 Noetic 官方一致。

## 4. 使用（在 AGX 上，golai 用户）

```bash
ssh s10-48-remote        # 或现场：ssh golai@10.21.33.102
cd ~/ros1_gateway
```

启动（默认保留原话题名，给 x_nav 用）：

```bash
bash scripts/start_gateway.sh
```

`--names ros1` 改用 `/lidar_points`、`/imu/data`、`/odom`；`--advertise <IP>` 设置 ROS 1 客户端访问 AGX 的地址（默认 `10.21.33.102`，机器人内网/热点；在场馆 Wi-Fi 上的电脑用 AGX 的 Wi-Fi 地址）。

在 106 上启动点云 tap（`user` 账号）：

```bash
bash ~/ros1_gateway_tap/run_tap_106.sh start
```

检查：

```bash
bash scripts/health_check.sh --hz 10
```

停止（顺序无要求）：

```bash
bash scripts/stop_gateway.sh
```

```bash
bash ~/ros1_gateway_tap/run_tap_106.sh stop
```

## 5. 运控 SDK

默认**空跑**：读取机器人反馈，只记录“将要发送”的指令，不创建任何 ROS 2 发布器。

```bash
bash scripts/start_control.sh
```

实测运动时（现场有人持遥控器、场地空旷、没有别的导航程序在控制）：

```bash
bash scripts/start_control.sh --enable-motion
```

| ROS 1 输入 | 机器人动作 |
|---|---|
| `/web_cmd` `cmd4` | 站立(1) → 确认 → 停 2 s → RL 控制(17) → 确认 → 导航平地步态 0x3002 → 确认 |
| `/web_cmd` `cmd3` | 速度归零 → 实测静止 ≥1 s → 趴下(4) → 确认 |
| `/web_cmd` `Nav stop` | 软件停止：发零速并锁住，忽略 `/cmd_vel`（**不是**关节泄力的软急停；遥控器才是急停） |
| `/web_cmd` `Nav continue` | 解除锁停 |
| `/cmd_vel` | 仅在 RL(17)+导航步态、反馈新鲜时转发；限幅 0.3 m/s、0.1 m/s、0.5 rad/s；10 Hz 固定频率；0.5 s 无新指令即归零，零速保持 1 s 后停止发送 |

其他保护：发现第二个 `/NAV_CMD` 发布者时锁定故障，需重启节点；启动时如已有别的 `/NAV_CMD` 发布者，拒绝进入运动模式。状态见 ROS 1 话题 `/s10_control/state`（JSON），事件日志在 `logs/control-events-*.jsonl`。

停止：

```bash
bash scripts/stop_control.sh
```

## 6. 离线录包转换

转换后用官方解码器审计：`tools/audit_offline.sh <ROS2 bag 目录> <新的 .bag> <报告目录>`（ROS 2 端 Jazzy rosbag2_py，ROS 1 端官方 ros:noetic 镜像的 rosbag info 与 rosbag）。

```bash
.venv/bin/python tools/mcap_to_ros1_bag.py <ROS2 bag 目录> <新的 .bag> --topics config/topics.yaml
```

源目录只读，输出路径必须不存在，不能写在源目录里。同时生成 `<bag>.conversion.json`（源和输出的 SHA256、工具版本、命令、每个话题的条数）。

## 7. 已完成的验证

| 测试 | 环境 | 结果 |
|---|---|---|
| 合成数据端到端（点云经 tap，IMU/ODOM 经 DDS，ROS 1 rosbag 录制后逐条比对） | Mac Docker 隔离网络 | E2E_OK |
| 同上，30 s | **AGX 原生**，隔离 ROS 域 77 | 点云 300/300、IMU 6000/6000、ODOM 300/300，零差异；点云转换平均 11 ms |
| 运控 19 项（起立顺序、限幅、10 Hz、超时、锁停、趴下、第二控制源、空跑不发送） | Docker 与 **AGX 原生**，模拟机器人 | 全部通过 |
| 脚本启动/健康检查/停止 | AGX | 通过，无残留进程 |
| 离线转换全流程（rosbag2_py 写合成 MCAP → 转换 → 官方 Jazzy 与官方 Noetic 容器分别解码 → 逐条比对） | Mac Docker | AUDIT_OK：2000/100/100 条全部一致，录包时间一致 |

### 新狗（050，SN `CS10100050`）实机验收 2026-09-19

AGX 已装到 050 号狗；106 软件版本与 48 号相同（robot_drivers 3.1.41、localization 3.5.14、slam 3.5.1），点云同样仅本机发布。103/106 与 48 号使用**相同的 SSH 主机密钥**（厂商镜像），主机密钥不能区分设备，身份以 106 的 `robot_manufacturing_info.toml` 序列号为准。

| 项目 | 结果 |
|---|---|
| 实时数据 | `/LIDAR/POINTS` 经 tap 10.00 Hz；`/IMU` 经 DDS 200 Hz；`/ODOM` 无（106 官方定位服务未开启，按决定不开启，x_nav 用自己的 SLAM） |
| 与 ROS 2 逐条比对（60 s，106 本机官方订阅做参考） | 点云 592/592、IMU 11845/11845 字段完全一致（含数据 SHA256、ring、逐点 timestamp），无倒退；AUDIT_OK |
| 10 分钟连续运行 | AGX 网关 CPU 均值 30.3%（最高 36.9%，单核计）、内存 67 MiB 无增长；tap 收发 13621/13621、丢弃 0；每分钟频率 9.96–10.00 Hz |
| 106 负载 | tap CPU 均值 8.4%、内存 185 MiB；厂商 rslidar 13.4%（加 tap 前 14.1–14.6%）、yesense 4.0%（之前 3.7%）；eth0 发送 117 Mbit/s |
| 一键复原 | `scripts/robot_session.sh down` 后 106 无我们的目录和进程、厂商服务正常；`up` 20 s 内恢复 HEALTH_OK |
| 时钟 | 原先机器人时间比 AGX 快 34 s；AGX 改为 PTP 跟随 103（`scripts/agx_ptp/`，linuxptp + `s10-ptp4l`/`s10-phc2sys`，关闭互联网 NTP），偏差数十至数百 ns；点云时间戳比本机时间早 0.13 s（采集与传输延迟），IMU 0.01 s |
| x_nav 容器 | 容器内 `/LIDAR/POINTS` 10 Hz、`/IMU` 200 Hz；建图（地图“111”）可运行，授权通过；SLAM 输出 `/base_link/odom` 10 Hz |
| 运控 SDK（空跑） | 读到新狗反馈（state 0、高度 0.07 m）；识别出 2 个原厂 `/NAV_CMD` 发布者（103 handler、106 localPlanner），因此开启运动前须先与其他使用者约定暂停它们 |
| 采集助手 `/teach` | worker 运行：位姿（`/base_link/odom`）10 Hz、点云 10 Hz、IMU 199 Hz；页面 `http://10.21.33.102:8080/teach` |

106 的 tap 用 `os.sched_setaffinity` 绑定 0–3 核，而不是 `taskset`：106 的 `/usr/bin/taskset` 带 `cap_sys_nice` 文件能力，glibc 会清除 `LD_LIBRARY_PATH`，导致 rclpy 无法加载。

**尚未完成：** 运动分级测试（需先暂停原厂控制源）；另一台机器经机器人 Wi-Fi 订阅 ROS 1 与手机访问 `/teach`（未在机器人热点下验证）；厂商 `map_manager`（手册中导航点、虚拟障碍物的保存可能依赖它）与 S10 外参；楼梯步态切换；旧狗录包的离线转换。

## 8. 厂商 x_nav 容器（2026-09-19 已做的系统改动）

用户用 `ysc` 账号交互输入密码，执行了 [vendor/x_nav/agx_root_setup.sh](vendor/x_nav/agx_root_setup.sh)：

- 先写 `/etc/docker/daemon.json`（`"iptables": false`），再安装 Ubuntu 的 `docker.io` 29.1.3、`docker-compose-v2` 2.40.3（附带 `containerd`、`runc`、`ubuntu-fan`）；未升级或删除任何已有包。
- 安装前后 iptables 规则一致（仅计数器变化），`ip_forward` 仍为 1。备份：`/var/backups/s10-docker-20260919-145617/`。
- `/opt/data/nav_map/x_nav.yaml`：厂商原文件（话题已是 `/LIDAR/POINTS`、`/IMU`，与网关“保留原名”模式一致）。
- `/opt/data/compose/docker-compose.yml`：厂商原文件，仅 `restart` 改为 `"no"`（验收前不开机自启）。
- 已拉取 `registry.cn-hangzhou.aliyuncs.com/embodiedai/nav:3.2.1`（`sha256:381e862a…`），随后 `docker logout`，AGX 上不保留仓库密码。
- **没有**启动容器；没有拉取 `ros1_humble_bridge`（它是另一台机器人的运动控制桥）。
- docker、containerd 服务随系统开机启动（Ubuntu 包默认）；x_nav 容器本身不会自启。

启动 x_nav（`ysc`，先启动网关）：

```bash
cd /opt/data/compose && sudo docker compose up -d
```

授权：AGX 没有 NVMe，厂商的 `nvme id-ctrl` 取不到序列号；已提供 eMMC 序列号 `0x57c95305` / CID `4501004447343036340157c95305cc00`。厂商随后给出 `ssd_whitelist.conf.hash`（SHA256 `83125a05…`），已放到 `/opt/data/config/`；是否与本机匹配只能由 x_nav 校验，试启动时日志里未出现授权相关输出。

**镜像：** `nav:3.2.1` 是 x86-64 单架构镜像（清单只有 amd64，`/bin/bash`、`/nav/w_nav` 均为 x86-64 ELF），AGX 无法运行，已删除。现用厂商提供的 `nav:3.2.1.arm.beta`（`sha256:3ec63779…`，arm64，容器内 `uname -m` = aarch64）。

**配置修改（相对厂商原文件，仅两处）：** `x_nav.sensor_lidar`、`x_nav.robot_model` 原为 `NULL`，ROS 1 参数服务器不能存空值，导致 `robot_web_controller.py` 启动即崩溃（`TypeError: cannot marshal None`）。改为 `"EXTERNAL_ROS1"`、`"S10"`：两者都不在 x_nav 的内置列表中，因此不会启动其自带雷达驱动（MID_360/RSHELIOS）或机器人 SDK（GO2/G1/B2/X30 等），点云来自本网关，运动经 `s10_ros1_control`。

**2026-09-19 试启动（未接机器人网络，60 s 后自动停止删除）：** 节点 `x_nav_control`、`w_nav`、`foxglove_bridge`、`nav_state_logger`、`pointcloud_downsampler` 正常；端口 8000（网页）、9000（控制接口）、8765（foxglove）监听在所有网卡；`w_nav` 订阅本网关的 `/LIDAR/POINTS`；`/cmd_vel` 由 `w_nav`、`x_nav_control` 发布，`/web_cmd` 由 `x_nav_control` 发布（均只在 ROS 1）。日志保存在 AGX `~/ros1_gateway/logs/x_nav_trials/`。

**需要厂商确认：**

1. `x_nav/map_manager` 无法启动：镜像内没有 `x_nav` 软件包（厂商自己的 compose 从 `/home/msi/BWT/digitaltwins-x-nav` 挂载源码，我们没有这个目录）。需要包含 `x_nav` 的镜像，或提供源码。
2. `w_nav` 启动时报告 `lidar_to_base: t=(0.2,0,0) rpy=(0,-20°,0)`；`x_nav.yaml` 的 SLAM 外参为 MID-360 默认值。S10 的 `/LIDAR/POINTS` 在 `lidar_link` 下，106 官方 SLAM 外参为单位阵，需要厂商按 S10 给出正确值。
3. `sensor_lidar`、`robot_model` 的取值是否符合厂商预期；`lidar_type: 7` 是否读取 `timestamp`（float64 秒）字段。
4. ARM 镜像里没有 `/nav/jluwb_node`（UWB）；我们没有 UWB，可将 `auto_run_commands_flags` 第 5 项设为 `false`。
5. foxglove（8765）和网页（8000/9000）对所有网卡开放，开发页密码仍为默认 `"123"`。

## 9. 回滚

网关、tap、运控 SDK：全部在 `~/ros1_gateway`（AGX）和 `~/ros1_gateway_tap`（106）里，没有 systemd 服务、没有开机自启。停止进程后删除这两个目录即可。

Docker 与 x_nav（`ysc`）：

```bash
cd /opt/data/compose && sudo docker compose down
```

```bash
sudo systemctl disable --now docker.service docker.socket containerd.service
```

需要完全移除时再执行 `sudo apt-get purge docker.io docker-compose-v2 containerd runc ubuntu-fan`，并删除 `/var/lib/docker`、`/etc/docker/daemon.json`、`/opt/data`。

## 10. 已知限制

- AGX 时钟比机器人时间慢约 14 s（2026-09-18 实测）。网关不改时间戳；运控节点用机器人时钟填指令时间戳。x_nav 若比较传感器时间与本机时间，可能判为过期，需要同步 AGX 时钟（属于系统改动，需先确认）。
- 点云约 200 Mbit/s；通过 Wi-Fi 订阅的 ROS 1 客户端可能跟不上，全速建议有线或在 AGX 本机运行。
- ROS-O 与 ros1_bridge 在 Ubuntu 24.04 上不是官方支持组合，以本目录测试结果为准。
