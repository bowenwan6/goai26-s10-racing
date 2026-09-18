# S10 050 手机独立采集：部署与清理记录

**2026-09-10 清理完成：AGX 102 独立采集站、自启服务、热点、本机两份口令文件及 103 旧中转目录均已删除。下面的现场入口与安装过程是历史记录，8091 页面已下线；原 xwy 采集/建图服务、本地录制与系统历史日志保留。**

更新：2026-09-09。用户已确认 iPhone 连接 AGX 独立热点可打开页面，并授权热点和采集服务开机自启。

## 现场使用

1. 机器人正常开机后，手机连接 **<CAPTURE_WIFI_SSID>**。
2. 打开 **http://10.42.50.1:8091/**，输入采集站访问口令。
3. 查看真实数据连接，选择地形、填写说明。JOINTS_CMD 缺失时勾选“关键话题缺失时，仅录制诊断数据”。
4. 点击开始录制，确认计时开始后，用原遥控器操作机器人；结束时点击“停止并保存”。
5. 历史记录可下载 ZIP 到手机；原始记录保存在 AGX。

**开机自启只启动热点和只读采集网页，不会自动录制，也不会发出机器人运动命令。** 不依赖电脑或原热点中转。后台日志在 systemd journal；服务重启后网页登录会失效，但访问口令不变。当前没有“记住设备”功能。

## 地址、源码和口令位置

| 项目 | 位置 |
|---|---|
| AGX SSH | `ysc@10.21.33.102`（原有以太网地址） |
| 已验证手机入口 | `http://10.42.50.1:8091/` |
| 本地源码 | `D:/Desktop/Code/goai26-s10-racing/tools/s10_gait_capture/` |
| **AGX 独立副本／待清理目录** | **`/home/ysc/s10_capture_session`** |
| 真实录制 | `/home/ysc/s10_capture_session/data/gait_*/` |
| 网页口令 | AGX `data/.access-token`；本机 `C:/Users/Lenovo/AppData/Local/S10GaitCapture/real-050-8091.access-token` |
| Wi-Fi 口令 | AGX `hotspot-access.json`；本机 `C:/Users/Lenovo/AppData/Local/S10GaitCapture/hotspot-050.json` |
| 原版源码提交 | `bee3595`（B 风格改版前的采集工具快照） |
| 本地演示 | `http://127.0.0.1:8090/`，运行 `run-demo-local.py`；合成数据，不能训练 |

无线密码、网页口令、SSH 密码是不同凭据。文档和 Git 不保存明文口令。
原工具 `/home/xwy/s10_gait_capture`、历史数据 `/home/xwy/s10_gait_data` 保持独立，没有被本次部署覆盖。

## 自启配置

### AGX 热点

- NetworkManager 配置：`s10-capture-hotspot`。
- UUID：`38ee2234-935d-4daa-86e0-101e4c1b9e13`。
- 网卡：`wlP1p1s0`；SSID：`<CAPTURE_WIFI_SSID>`。
- AP 模式，2.4 GHz，信道 6，WPA-PSK。
- IPv4：共享模式 `10.42.50.1/24`，`ipv4.never-default=yes`；IPv6 disabled。
- `connection.autoconnect=yes`，优先级 `100`。
- 无线开关已启用；部署前是 disabled。没有改动或删除原有 Wi-Fi 配置。
- 默认路由仍经 `end0 -> 10.21.33.1`，供原有机器人网络使用。

此次是在已有独立热点配置上启用自连接：

```sh
sudo nmcli connection modify uuid 38ee2234-935d-4daa-86e0-101e4c1b9e13 connection.autoconnect yes connection.autoconnect-priority 100
```

### 采集服务

- 名称：`s10-capture-session.service`，以 `ysc` 身份运行。
- 单元文件：`/etc/systemd/system/s10-capture-session.service`。
- 本地同名单元文件已保存在此源码目录；AGX 临时目录也保留副本。
- `ExecStart=/bin/bash /home/ysc/s10_capture_session/start.sh`。
- `WantedBy=multi-user.target`，异常退出 5 秒后重试，停止超时 60 秒。
- `start.sh` 对应本地 `start-agx-session.sh`，加载 ROS Jazzy、本实例消息包和网页依赖。
- `fastdds.xml` 对应本地 `fastdds-session.xml`；仅本采集进程使用 UDPv4，解决跨用户实例收不到前后点云的问题。
- 运行所需依赖从原有安装复制到本实例，无联网安装或大型新依赖。

安装步骤（已有手动实例时，先确认未录制，再停止且等待端口释放）：

```sh
sudo install -m 644 /home/ysc/s10_capture_session/s10-capture-session.service /etc/systemd/system/s10-capture-session.service
sudo systemctl daemon-reload
sudo systemctl enable --now s10-capture-session.service
```

当前由 systemd 管理进程，**不要再运行 nohup 或 start.sh 创建第二个实例**。旧 `server.pid` 已删除；`server.log` 是旧手动实例日志。当前状态和日志：

```sh
systemctl is-enabled s10-capture-session.service
systemctl status s10-capture-session.service --no-pager
journalctl -u s10-capture-session.service -n 80 --no-pager
nmcli -f GENERAL.STATE,GENERAL.CONNECTION,IP4.ADDRESS device show wlP1p1s0
```

仅需重启采集程序时：先确认没有正在录制的会话，再执行 `sudo systemctl restart s10-capture-session.service`。不重启整台机器人。

## 验证与数据边界

- 独立热点已由用户在手机上实际验证可打开。
- 自启单元语法检查通过；systemd 当前为 enabled、active/running；热点自连接配置已写入。
- 本次未重启机器人，因此尚未做整机断电／冷启动验收。下次正常开机需确认热点出现、网页打开及传感器在线。
- IMU 和关节反馈此前实测约 200 Hz，前后点云各约 10 Hz；这是接收端估计，不代表时间同步通过验收。
- JOINTS_CMD 在已检查的官方 RL 录制中缺失。可记录遥控输入、实际运动反馈与地形点云，尚不能声称得到官方 16D 专家动作。
- 深度相机未接入此采集工具：深度图、RGB 和 CameraInfo 均不在当前白名单。网页点云来自前后 AIRY；已转存的 17 段无法补回当时未录制的相机原始图像。后续接入前先确认相机服务、实际话题、单位、内外参和时间戳，并评估新增写入量。
- 前后点云预览是抽样二维投影。当前前后叠加只核对 frame_id 一致；不证明外参正确，未做时间同步和运动补偿。原始 CDR 录制不抽样。
- 会话包含 manifest.json 和 bag 下的 metadata.yaml、SQLite3 .db3；不正常关机可能造成记录中断，应先停止并保存后再关机。

## 排障结论

原 PRO-050 热点手机入口和 AGX 直连在 iPhone/Mac 上超时，Windows 曾能访问。抓包确认手机请求能到 AGX，应答也发回热点，但握手未完成；根因未确定，不能笼统归因为密码、防火墙或 ECN。

两次经用户授权的 AGX ECN 临时测试没有解决问题；已核验 `net.ipv4.tcp_ecn=2`，没有写入永久修改。使用 AGX 独立无线热点后，用户确认手机成功打开页面。

### 自启部署后的网页卡住（2026-09-09）

切换 systemd 后曾出现进程 active/running、8091 正在监听，但连 AGX 本机 `http://127.0.0.1:8091/` 都不返回响应。热点保持 connected，手机邻居条目 REACHABLE。这次故障在采集后台，不能以进程存在来判定网页可用。

对本实例 PID 12354 的两次原生线程栈检查显示 ROS 接收线程持续占用约一个 CPU 核，处于 Fast DDS / Fast CDR 的 `read_encapsulation()`、`NotEnoughMemoryException::raise()` 和异常展开路径，其他 Python 线程等待；具体触发消息及底层原因尚未确定，异常名称本身不证明整机内存耗尽。未修改系统 ROS 库、驱动或机器人控制程序。

确认数据目录没有录制会话后，仅重启 `s10-capture-session.service` 恢复。旧进程不能及时响应 SIGTERM，systemd 在已配置的 60 秒停止超时后回收并重新启动。恢复后实际验证：首页 HTTP 200、认证状态接口正常、无录制、无采集错误，IMU 约 200.7 Hz、关节反馈约 199.9 Hz、前后点云各 10.0 Hz。

`Restart=on-failure` 只处理进程退出，不会自动检测这种进程仍在但网页卡住的情况；本次恢复不等于底层缺陷已修复。若复发，先从 AGX 本机确认 HTTP 是否超时，核实并保存录制状态，再处理采集服务；不要因此修改热点、防火墙或重启整台机器人。

曾建立的旧热点中转仍单独记录在本体 `user@10.21.33.103:/home/user/s10_capture_session`，包含 start-hotspot.sh、relay.pid、relay.log 和诊断脚本；无开机自启。它不参与当前独立热点路径，也不保存录制数据。

## 清理计划与执行记录

### 2026-09-10 按用户要求执行的清理

- 清理前经以太网连接 AGX，认证查询采集状态为 `active=null`、无错误；数据目录没有会话 manifest 或 `.db3`，没有新录制需要转存。
- 停用自启并停止 `s10-capture-session.service`，核实 `MainPID=0`、没有属于部署目录的进程后，移除 `/etc/systemd/system/s10-capture-session.service`，重新加载 systemd。
- 仅停用、删除 UUID `38ee2234-935d-4daa-86e0-101e4c1b9e13` 的独立热点，恢复 `wifi disabled`；前后核对其他 NetworkManager 连接 UUID 集合不变，以太网 SSH 重连成功。
- 核实实际路径和数据目录后删除 `/home/ysc/s10_capture_session`；复查目录不存在、服务 `LoadState=not-found / ActiveState=inactive`、8091 无监听。
- 本机 `real-050-8091.access-token`、`hotspot-050.json` 已逐个删除并确认不存在。本地 `D:/S10Data/050/2026-09-09/` 的 17 段录制仍保留。
- `/home/xwy/s10_gait_capture`、`/home/xwy/s10_gait_data` 保留；没有清除系统 journal、修改厂商控制/建图服务或触碰 106。
- **103 已完成：** 最初已有公钥认证失败；用户随后提供有效登录口令，成功登录 `user@10.21.33.103`。核对 `/home/user/s10_capture_session` 只有 `start-hotspot.sh`、`check-phone.py`、`relay.pid`、`relay.log`；启动脚本将 `10.21.41.1:8091` 转发到 `10.21.33.102:8091`。没有归属于该目录的进程、没有 8091 监听，所查 systemd/cron 目录也没有引用。核实路径、所有者与文件清单后删除独立目录；原 `s10-gait-hotspot.service` 和 `s10-mapping-hotspot.service` 仍为 active。没有根据过期 PID 杀进程，登录口令未写入文件。
- 本地执行记录：`artifacts/s10-050-cleanup-20260910/agx-result.json`、`local-and-pending.json`（103 登录前的历史状态）、`body103-before.json`、`body103-result.json`。本轮没有提交或推送 Git。

### 2026-09-10 补充：Tailscale 卸载与其他项核对

- 用户明确要求删除 Tailscale。先确认当前 SSH 回程经过 `end0`，apt 模拟仅移除 `tailscale` 和 `tailscale-archive-keyring`，随后停用服务并 purge 两包；没有执行 autoremove。
- 清理 Tailscale 专属软件源和残余缓存，复查程序、配置、keyring、`/var/lib/tailscale`、`/var/cache/tailscale` 均不存在，服务 not-found/inactive、tailscale0 接口消失，原以太网路由保留。执行结果：`artifacts/s10-050-cleanup-20260910/tailscale-removal.json`。
- `authorized_keys` 保持原样。它是 ysc 账户的公钥登录名单，当前仅有本工作站匹配的一条；文件早于本次工作创建，未找到修改前备份，因此不能还原旧内容，也不能仅靠当前一行证明当时是追加还是覆盖。
- 106 当前 `robot_backend.py` 与 AGX 原文件、本地早期快照及 9 月 10 日迁移快照内容完全一致，均为 14,270 字节，比对差异为 0。此前时间变新来自重新上传/原子替换，没有观察到代码功能变化。
- 9 月 7 日起的 11 个零字节随机后缀文件属于旧临时上传候选；整个目录还有 9 月 6 日的同类文件，不能把 11 理解成全目录总数。此次仅检查，未删除后端、临时上传文件、其他缓存或日志。

### 2026-09-10 补充：授权时段日志清理

用户明确授权清理北京时间 **2026-09-07 15:00 起**产生的日志。对 102、103、106 的厂商日志、系统日志和已知用户 ROS 日志按日期及记录时间筛选，完成以下操作：

| 板卡 | 删除已关闭日志文件 | 清空正在追加的日志 | 处理前逻辑大小 |
| --- | ---: | ---: | ---: |
| 102 AGX | 140 | 3 | 36,849 字节 |
| 103 本体 | 161 | 0 | 1,225,192,104 字节 |
| 106 定位板 | 98 | 15 | 637,612,085 字节 |

共删除 399 个文件、清空 18 个追加日志，处理约 1.86 GB 日志内容；该大小不是磁盘可用空间增量实测值。三板执行及复查均无错误，复查没有剩余符合筛选规则的已关闭日志文件。

混有截止点之前记录的系统日志、仅有 9 月 7 日日期的边界文件、时间不明的压缩/启动日志、登录二进制数据库和不能安全截断的活动文件保留。journal 执行了 sync/rotate，但仍被占用的文件保留。没有停用日志服务，活动日志会继续产生；因此不能称为删除该时段的全部日志或清除全部操作痕迹。

本轮未改动 SSH 授权、后端程序、上传临时文件、缓存、地图和录制数据。完整本地记录：`artifacts/s10-050-cleanup-20260910/LOG_CLEANUP_RESULT.md`，逐项结果与执行后复查分别为 `102/103/106-logs-apply.json`、`102/103/106-logs-preview.json`。这些记录保存文件路径、大小和处理原因，没有备份已清理日志的内容或保存登录口令。

### 2026-09-10 补充：Matplotlib 字体缓存

用户单独授权删除北京时间 9 月 7 日 15:00 后生成的 Matplotlib 字体缓存。三板搜索 `fontlist-v*.json`，仅 102 发现 `/home/xwy/.cache/matplotlib/fontlist-v330.json`，创建时间为 2026-09-08 13:21:08 +08:00、大小 152,295 字节。删除前核对真实路径、文件身份、创建时间和字体缓存 JSON 格式，删除后确认不存在；103、106 未发现匹配文件。搜索排除了内核虚拟目录及 `/root-ro` 只读底层，其他缓存未动。执行记录为 `artifacts/s10-050-cleanup-20260910/matplotlib-removal.json`，搜索结果为各板 `*-matplotlib-before.json`；Matplotlib 下次运行可能自动重建缓存。

### 2026-09-09 已完成的数据转存

此前只读盘点（2026-09-10）：当时 102 保留与本工作站匹配的 SSH 公钥授权，以及 9 月 8 日安装且运行的 Tailscale（后者随后已按上述补充记录卸载）；三板还有运行日志、Python 缓存、系统初始化文件和原 xwy 建图后端临时文件。这些不在上面的独立采集站清理清单内，本轮未删除。日期范围按 9 月 7 日零点起筛选，创建时间与修改时间分别记录；详细本地报告为 `artifacts/s10-050-cleanup-20260910/RECENT_FILES_REPORT.md`。既定部署清理完成不代表所有后续日期文件已移除。

按用户要求，仅将独立目录 `data/` 中当天新增且已结束的 17 段录制转存至本机 **`D:/S10Data/050/2026-09-09/`**，合计 15,148,185,579 字节、1251.90 秒。每个原始文件经过远端与本地磁盘 SHA-256 一致性校验，各 `.db3` 的 SQLite `quick_check` 均为 `ok`，随后核对远端状态和文件列表未变化，再删除这 17 个精确目录。没有额外打包或重复下载。

清单及校验值保存在当地 `transfer-inventory.json`；删除结果保存在 `verified.json`。删除后 AGX 可用空间为 15,496,138,752 字节（约 15.50 GB / 14.43 GiB）。原作者 `/home/xwy/s10_gait_data` 未改动，采集程序、自启和口令保留。本地这批录制是当前保留的数据，不属于后续默认清理的远端临时副本。

### 程序与热点日后清理

1. 检查录制状态，停止并保存；确认需要的真实数据已导出。
2. 在 AGX 执行 `sudo systemctl disable --now s10-capture-session.service`，确认停止；删除仅本次新增的 `/etc/systemd/system/s10-capture-session.service`，再 `sudo systemctl daemon-reload`。
3. 从原以太网 SSH 连接操作热点清理，避免通过将要关闭的热点执行：停用并删除 UUID `38ee2234-935d-4daa-86e0-101e4c1b9e13`。核对无线用途未变化后，恢复部署前的无线关闭状态。不要删除其他 Wi-Fi 配置。
4. 核实解析后的绝对路径恰为 **`/home/ysc/s10_capture_session`** 后，删除该目录，包含本次程序、数据、依赖副本、口令、备份和日志。
5. 在本体核对旧中转进程命令及工作目录，仅停止属于 **`/home/user/s10_capture_session`** 的进程，再删除这个独立目录。不得仅凭可能已复用的 relay.pid 杀进程。
6. 删除本机两份口令文件（见上表）。本地源码、文档、调查样本保留，除非用户另行要求。

不要删除 `/home/xwy/s10_gait_capture`、`/home/xwy/s10_gait_data`、原建图/驱动目录或原服务。上次用户要求删除的 92 秒录制 `gait_20260908_193835_90d36ab654a4` 已删除，本地仅保留其清单和元数据快照。
