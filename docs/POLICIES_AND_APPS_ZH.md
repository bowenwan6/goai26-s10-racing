# S10 policy 与 app 总览

2026-09-19 整理。本文回答三件事：
- 我们有哪些 policy、哪些 app / 工具；
- 它们放在哪里；
- 各自做到哪一步。

细节看表中链接。仓库目录和分支说明见 [REPO_GUIDE_ZH.md](REPO_GUIDE_ZH.md)，导航设计见 [NAVIGATION_DESIGN_ZH.md](NAVIGATION_DESIGN_ZH.md)。
标“已归档”的文档 2026-09-20 合并文档时删除，可从标签取回：`git show docs-archive-20260920:docs/<文件名>`。

**状态标记：** ✅ 实机用过 / 已部署　🧪 只在仿真　📝 待开发 / 待实测　🗄 历史

## 0. 现状一览

- **实机跑过的 policy 只有三个：** 官方 57D（AGX 版）、speedturn2000、HIM 1500。HIM 1500 没能上台阶。
- **9 月导航方案**（`rl_nav`）：J3100 负责步行，1150 负责台阶。只在 MuJoCo 里跑通（30/30 WP，713 s），还没上真机。
- **8 月仿真赛 Ver1.0**：官方 57D + Gate16 v1.5 + stairs_stable。33/33 WP，436.058 s（seed 6）。
- **第一次长距离真机运行（09-21 上午，048 号狗，strategy0）：** 赛道 WP10 → WP29，569 s。66% 的时间在台阶步态里按默认 0.30 m/s 限速走（操作员自己在台阶步态下中位速度 0.73 m/s）；平地段平均 0.69 m/s，被“前方可见空间不足”的速度缩放和侧移回线拖慢。详见 `ros1_gateway/docs/PIPELINE_AND_PLANNING_ZH.md` §10。
- **strategy1（09-21 中午，只在仿真里）：** 回线改成转向为主、加速更快、台阶步态区按地形给速度、示教线可局部替换 / 删点 / 顶墙点、可从任意 WP 续跑、网页急停也能停终端启动的运行。新路线 279.7 m、29 个点、直行 86%、总转向 2209°，独立净空检查 0 违规；仿真 29/29、413 s（strategy0 约 470 s）。**还没上真机。**
- **真机自主导航首次成功（09-20 19:27:46–19:28:25，048 号狗）：** 室内地图 `v6_room` 上 WP01 → WP02（直线 4.67 m，路线长 5.0 m），用时 38.2 s，模式 `DONE`，无故障、无人工干预。
  - 限幅 0.10 m/s，实测平均约 0.12 m/s（原厂步态不会精确跟随指令）；停在离 WP02 0.22 m 处，按 0.20 m 判定半径进入时计到达。
  - 动关节的是原厂控制器（state 17 + 平地步态 0x3002）配合“导航”使用模式，不是 J3100 / 1150。
  - 依赖条件：切导航模式、用机器人 IMU 取姿态、高度网格盲区填充、平地台阶上限 0.12 m、放宽 z 容差、实测站立高度 0.41 m。
  - 还没验证：台阶段（一次尝试因“25 s 没有进展”停住）、返程路线、比探测速度更高的速度。
- **传感器链路（09-19 在 050 号实测）：** 点云 10 Hz、IMU 200 Hz，与 106 本机 ROS 2 数据逐条一致；连续 10 分钟稳定。
- **x_nav：** 能建图、能保存地图、能重定位（发 `/initialpose` 初始化）；输出位姿 `/base_link/odom`（10 Hz）。
- **狗的归属：** 09-19 用 050 号，09-20 已交还并换到 048 号。每换一只狗，密钥、自启、站立高度、原厂 `/NAV_CMD` 发布者都要重做和重测。

## 1. Policy

### 1.1 总表

维度写作 `57→16`，意思是输入 57 维观测、输出 16 维关节指令。

| 名称 | 维度 | 用途 | 状态 | 文件 |
|---|---|---|---|---|
| 官方 57D（AGX SDK 版） | 57→16 | 通用行走 | ✅ 机上默认 | `vendor/contest_material/…/S10_sdk_deploy/policy/policy.onnx` |
| 官方 57D（本地 SDK 版 model0） | 57→16 | 通用行走；训练热启动起点 | 🧪 基线 | s10-rl-sprint `pretrained/s10/policy.onnx` |
| speedturn2000 | 57→16 | 高速 + 转向 | ✅ 48 号实机试过 | `policies/s10_general_speedturn_57d_model2000.onnx` |
| HIM 1500 | 342→16（57 维 × 6 帧） | 通用 | ✅ 实机试过，上台阶失败 | 不在仓库，在 AGX 的独立 SDK 副本里 |
| **J3100** | 59→16，自由相位 0.6 s | 步行，`rl_nav` 的 `official` 槽 | 🧪 | 复现包 `policies/J3100_walk.onnx` |
| **1150** | 59→16，命令门控相位 1.5 Hz | 台阶、陡坡、横坡，`rl_nav` 的 `stairs_stable` 槽 | 🧪 不能在 Isaac Lab 里微调，见 1.2 | 复现包 `policies/1150_stairs.onnx` |
| Gate16 v1.5（base + residual） | 174→16 | 0.377 m 高台（WP15→16） | 🗄 Ver1.0 | [`policy/gate16/`](../policy/gate16/) |
| stairs_stable（model_599） | 57→16 | 上楼 | 🗄 Ver1.0 提交配置 | [`policy/stairs_stable/`](../policy/stairs_stable/) |
| stairs57（model1800）及其他 8 月模型 | 57→16 | 上楼、下楼、通用 | 🗄 | [`policies/`](../policies/README.md) |
| Sprint A18@50 等 | 57→16 | 全向移动 + 停车 | 🧪 候选，未过验收 | s10-rl-sprint `sprint_results/releases/` |
| Phase 族 A500…E2100 | 59→16 | 平地 | 🧪 实验 | `Jackdev` 分支 `policies/phase_*/` |
| 决赛重训候选（Native1000、楼梯 750、高台 BC01） | 57→16 | 平地、楼梯、高台 | 🧪 实验 | s10-rl-sprint `docs/finals/` |
| 原生步态 0x3002 平地 / 0x3003 楼梯 | — | 厂商内置 | 📝 代码就绪，未现场验收 | 本体 `libCD1RunPolicy.so`（黑盒） |

### 1.2 实测要点

**实机**
- **官方 57D（AGX 版）：** 数据来自 50 号实机（`S10_REAL_ROBOT_QUICKSTART_ZH.md`（已归档））。
  - 起身 + 零速支撑：roll / pitch 最大 2.20° / 2.81°。
  - 右移 −0.5 m/s 持续 1 s：能看到平移，关节力矩峰值 29.85。
- **speedturn2000：** 数据来自 48 号实机（`Jackdev` 分支 `docs/S10_SPEEDTURN_2000_TRIAL_ZH.md`）。
  - 零速支撑 5 s 通过。
  - 前后 ±2.0 m/s 完成，轮速峰值 27.1 rad/s。
  - +2.1 m/s 时右前轮 31.9 rad/s，触发 30 rad/s 诊断线，进入阻尼。
- **HIM 1500：** 09-16 实机（`S10_HIM_DEPLOYMENT.md`（已归档））。
  - 起身、1 s 零指令、趴下都通过；ARM 与本地推理最大误差 4.8×10⁻⁷。
  - 上第一级台阶时两个前轮约 31 rad/s，触发保护，爬阶未通过。

**9 月导航用的两个 policy（MuJoCo）**

数据来源：s10-rl-sprint `sprint_results/v4/route_nav/`，复现包 `S10_Nav_FullV3_Repro_20260919/README.md`。

- **J3100（步行）**
  - 0.6–0.8 m/s 能过 3–8 cm 的坎和 8–12° 的坡；0.4 m/s 时 4–15 cm 坎和 10° 坡都失败。
  - 横坡：6° 时每 5 m 下滑 0.45 m，18° 摔倒。
  - 小于 0.15 rad/s 的转向指令基本不响应。
- **1150（台阶）**
  - 0.3 m/s 能过 12–18 cm 台阶（21 cm 摔倒）和 20° 坡。
  - 几乎不能转：在台阶上，0.3 / 0.5 rad/s 的指令只转出约 0.03 rad/s，指令为 +0.5 时摔倒。
  - 在 12–18 cm 台阶上轮速峰值 37–47 rad/s，超过 30 rad/s 诊断线。元数据写明 `hardware_validated=false`。
- **两者配合跑全程**
  - 30/30 WP，用时 713.0 s，每个 WP 的最近距离都 ≤ 0.18 m。
  - 轮速超过 30 rad/s 累计 1.94 s（峰值 57.9），倾角超过 15° 累计 21.8 s。

**8 月仿真赛（Ver1.0）**
- **Gate16 v1.5**（`TECHNICAL_DESIGN.md`（已归档） §9）
  - 低层矩阵 39/45，其中摔倒 2/45。
  - 比赛时用稳定 fallback：交接窗口 d = 0.62–0.70 m，指令 0.18 m/s。
  - 全程：seed 6 通过；seed 8 两次失败（Gate16 处、WP27→28）；seed 10 卡在 WP29 前。
- **stairs_stable（model_599）：** 只验证过三段（[`policy_manifest.json`](../policy/stairs_stable/policy_manifest.json)）。
  - WP6→7 通过；单独跑 WP17→19 通过。
  - 从 WP16 连续跑到 WP32 时，在 WP18 差 0.32 m。
- **stairs57（model1800）：** 规则楼梯 33/33；但接入 SDK 全栈后倾倒到 61–68°，已停用。

**楼梯 policy 训练（09-20，GPU 服务器）**

详见 s10-rl-sprint `docs/finals/s10_course_stairs_20260920.md`。

- **59 维是部署合同：** 上机的两个 actor 都是 59 维 = 仓库里的 57 维观测 + 跟随器那个 1.5 Hz 门控相位的 sin / cos。换 policy 必须也是 59 维。
- **1150 不能在 Isaac Lab 里接着训：** 平地、干净观测、名义机器人、0.3 m/s 下它直接塌（机身高 0.373 m，69% 因机身触地终止、31% 因倾角终止）；同样条件下官方 model0 站得住（0.454 m，0% / 0.2%）。
- **反过来是通的：** model0 补零到 59 维后能在 MuJoCo 赛道上走 WP21→WP24，4/4 航点用时 105.2 s，比 J3100 的 110.5 s 快，力矩峰值也更低。
- **带定位噪声（5 cm / 2°）的分段基线，种子 0–11：** B 楼梯 WP05→WP08 9/12（3 次在 60–63° 倾角摔倒）；平台段 WP26→WP29 11/12（1 次卡住）。种子 0–3 是 8/8。
- **全程 WP01→WP30，种子 0–11：** 完成 8/12，3 次 HOLD，1 次在 B 楼梯摔倒。
- **训练进度：** 第一阶段（生成的台阶，从 model0 热启动）第 800 次迭代到课程等级 8/10（约 16 cm 立面）、成功率 99%；第二阶段用赛道自己的台阶切片，**还在训，暂时打不过 1150**。
- **地形：** 从重建里切出 17 块爬升区（A 段 9 级、B 段 26 级分三窗、WP09 / WP10 高台、WP15 / 16 / 19 / 20 台阶、WP26–29 平台、WP30 下行）。

**RL 训练候选（s10-rl-sprint）**
- **Sprint A18@50**（`sprint_results/DELIVERY_ZH.md`、`AUDIT_LOG.md`）
  - 由 speedturn2000 微调。
  - 比基线好：不达标项 30（基线 38）；零指令漂移 0.28 m（基线 0.94 m）。
  - 但 10.6° 坡、0.15 m/s 只爬到 55%。未过验收，未上机。
- **决赛重训**（`docs/finals/development_progress.md`）
  - Native1000：跟踪 50/160（官方为 0/160），停车只有 1/170。
  - 楼梯 750 候选：九组各 32/32。
  - 高台 BC01：上台 23/27，停稳只有 1/27。

### 1.3 policy 怎么上真机

1. **链路：** 关节 / IMU 反馈 → AGX 上的 `rl_deploy` → ONNX Runtime → `/JOINTS_CMD` → 已开启 SDK 模式的本体。
2. **SDK 模式：** 按本机 SoC SN 申请授权码，重启后生效。
3. **换模型：** 在 AGX 上用 `scripts/prepare_s10_sdk_copy.py` 做一份独立 SDK 副本，再用 Windows GUI 起身、遥控。
4. **诊断停止线：** 腿 / 轮速度 25.76 / 30 rad/s，腿 / 轮力矩 45 / 12 N·m。越线即 R2 阻尼。
5. **多个 policy 共用：** [`integration/joint_command_owner.hpp`](../integration/joint_command_owner.hpp) 保证 `/JOINTS_CMD` 同一时刻只有一个控制源。切换要经过 SafeHold（0.25 s）。
6. **runner 支持的观测维度：** 44dd04d 版支持 57 / 174 / 342；`Jackdev` 版支持 57 / 59 / 174。
7. **`rl_nav` 上机的前提（未完成）：**
   - SDK runner 两个槽都要支持 59 维观测和各自的相位时钟。
   - 1150 的轮速超线问题要先解决。
8. **原生步态路线（备选）：** `native_transfer` 请求 0x3002（平地）或 0x3003（楼梯）。代码和 78 项本地测试已就绪，现场验收还没做（`NATIVE_START_B_ACCEPTANCE.md`（已归档））。

## 2. App 与工具

### 2.1 手机页面

由 AGX 上的 [`tools/s10_mapping_web/server.py`](../tools/s10_mapping_web/server.py) 提供，端口 8080。

| 页面 | 用途 | 依赖 | 状态 | 说明 |
|---|---|---|---|---|
| `/` 建图首页 | 官方 drmap 建图启停、保存、点云预览 | 106 后端（SSH） | ✅ 48 号 | [README](../tools/s10_mapping_web/README.md) |
| `/localization` | 在点云底图上显示位置和朝向 | 106 `/ODOM` | ✅，但底图还是旧图 `indoor_loop_01` | 同上 |
| `/heightmap` | 显示 `/elevation_map_raw` | 106 `heightmap.py` | ✅ 已部署，交互未验收 | `evidence/backups/s10-mapping-web/2026-09-13/README.md` |
| `/field` 现场助手 | 自检、切图、静止检查、限时录包、标点草稿、航点复测 | 106 field worker | ✅ 48 号；v3 现场未验收 | [FIELD_GUIDE_ZH.md](../tools/s10_mapping_web/FIELD_GUIDE_ZH.md) |
| `/imu-check` | IMU 曲线、静止采样、复零 | 106 | ✅ 09-15 | [IMU_DIAGNOSTIC_GUIDE_ZH.md](../tools/s10_mapping_web/IMU_DIAGNOSTIC_GUIDE_ZH.md) |
| `/native-nav` | 原生步态导航测试：只读检查、切步态、Start/B 任务、停止 | 106 native-nav 服务 | ✅ 已部署，**没实际走过** | [NATIVE_NAV_GUIDE_ZH.md](../tools/s10_mapping_web/NATIVE_NAV_GUIDE_ZH.md) |
| **`/teach` 采集助手（新）** | 新 SLAM 用：建图采集（带回环）、标 WP01–30 和切换点、录示教路径。只记录，不控制机器人 | AGX ROS 1（不连 103 / 106） | ✅ 真机上运行；手机经 103 上的用户态转发访问（交还前删除） | [TEACH_GUIDE_ZH.md](../tools/s10_mapping_web/TEACH_GUIDE_ZH.md) |

- **新狗和共用的狗上只能用 `/teach`。**
  - 其余几个页面要通过 SSH 调 106 上的后台。这个后台只装在 48 号的 106 上，AGX 用的专用密钥 `backend_key` 也只加到了 48 号。
  - **主机密钥拦不住：** 050 号的 103 / 106 与 48 号主机密钥相同（厂商镜像），106 的 IP 也相同。
  - **09-19 已实测连不上：** 这把专用密钥被 050 号的 106 拒绝（`Permission denied`），所以这几个页面操作不了新狗。它们只适用于 48 号，新狗上仍不要用。
  - `/field` 的“加载地图”会调用 `drmap map activate`，改变整机状态。

### 2.2 真机导航栈（ROS 2，`src/s10_auto_nav`）

| 名称 | 用途 | 状态 | 说明 |
|---|---|---|---|
| `rl_nav` 节点（ROS 2 版） | 第一版控制器作为名义行为，测到偏离才做恢复。输出 `/cmd_vel` 和 joint owner（`official` 槽 = J3100，`stairs_stable` 槽 = 1150） | 🧪 只在 MuJoCo。真机上跑的是同一份逻辑的 ROS 1 版，见 2.3 | [NAVIGATION_DESIGN_ZH.md §2](NAVIGATION_DESIGN_ZH.md) |
| `rl_nav_prepare` | 离线：route_v2 + 地图 → `route_rl.json`、`maneuvers.json`、`map_surface.npz` | 🧪 有单测 | `rl_nav/prepare.py` |
| route_v2 跟线 + Frenet 局部规划 | 沿示教中心线走，A* 兜底 | 🧪 | [NAVIGATION_DESIGN_ZH.md §1](NAVIGATION_DESIGN_ZH.md) |
| `native_transfer` | 原厂 flat / stairs 步态 + 我们的 follower，发 `/NAV_CMD`、`/GAIT` | ✅ 已部署 106 / 102，只读观测通过；📝 没发过真实指令 | [README](../native_transfer/README_ZH.md) |
| HIM SDK 部署 | 独立 SDK 副本 + HIM 1500 + Windows GUI | ✅ 09-16 实测 | `S10_HIM_DEPLOYMENT.md`（已归档） |
| `real_transfer` + `tests_real` | 只读采集、影子计算、离线回放 | 🗄 09-12；测试 164 项通过 | [README](../real_transfer/README_ZH.md) |

### 2.3 新狗 / 新 SLAM（ROS 1，[`ros1_gateway/`](../ros1_gateway/README_ZH.md)）

| 名称 | 用途 | 运行在 | 状态 |
|---|---|---|---|
| `s10_ros1_gateway` | ROS 2 → ROS 1 单向转发点云、`/IMU`、`/ODOM`，字节不变 | AGX | ✅ 新狗实测：60 s 内点云 592/592、IMU 11845/11845 与 106 本机 ROS 2 参考逐条一致；10 分钟 CPU 约 30%（单核）、内存 67 MiB 不增长 |
| 106 点云 tap | 106 的点云只在本机发布；tap 只读订阅，经 TCP 转给 AGX | 106 用户目录 | ✅ 新狗实测：10 分钟 13621/13621、丢 0；106 上 CPU 8.4%，厂商雷达驱动负载不变 |
| `scripts/robot_session.sh` | 一条命令部署 / 启动 / 撤掉 106 tap 和 AGX 网关 | Mac | ✅ 新狗实测 up → down → up；down 后 106 无残留，厂商服务正常 |
| `scripts/agx_ptp/` | AGX 时钟改为 PTP 跟随 103（`s10-ptp4l` / `s10-phc2sys`），关闭互联网 NTP；带回滚脚本 | AGX | ✅ 偏差百纳秒级（原先机器人比 AGX 快 34 s） |
| `s10_ros1_control` | ROS 1 `/cmd_vel`、`/web_cmd` → 原厂 `/NAV_CMD`、`/MOTION_STATE`、`/GAIT`；默认空跑 | AGX | 🧪 19 项模拟测试；新狗上只空跑过（能读到反馈，发现 2 个原厂 `/NAV_CMD` 发布者）；📝 **从未做过运动测试** |
| MCAP → ROS 1 bag 转换 + 审计 | 离线转换，用 ROS 2 / ROS 1 官方解码器逐条比对 | Mac（Docker） | ✅ 合成数据；📝 真实录包待转 |
| x_nav（厂商容器） | ROS 1 SLAM / 定位 | AGX Docker | ✅ 建图、保存、重定位都跑通（发 `/initialpose` 初始化），输出 `/base_link/odom` 10 Hz；📝 `map_manager` 仍缺 |
| **ROS 1 导航运行时** `nav/` | 把主仓的路线跟随器（`src/s10_auto_nav`）按记录的提交同步成纯 Python 版，在 ROS 1 下运行，输出 `/rl_nav/cmd_vel` 和步态请求 | AGX | ✅ **09-20 首次自主跑通 4.7 m**；📝 台阶、长路线、提速未做 |
| **一键运行** `robot_session.sh nav` / `nav_session.sh` | 选地图 → 发初始位姿 → 等站立 → 上电 → 切导航模式 → 跑 → 结束或异常时一律切回遥控模式。`--shadow` 只算不发 | Mac → AGX | ✅ 现场用过 |
| `tools/teach_to_route.py` | `/teach` 会话 → `route_v2.json` + 切换点生成的爬坡动作 | Mac / AGX | ✅ 已用它生成首条真机路线 |
| `tools/teach_line.py` | 多次示教 → 地图障碍栅格 + 走过的走廊 → 拉直并圆角的参考线；WP 用触碰圆盘，台阶步态区 = 操作员用了台阶步态**且**地图上有台阶 / 坡 | Mac / AGX | ✅ 生成了 09-21 真机跑的路线 |
| `tools/verify_route_clearance.py` | 独立的机身扫掠净空检查，写 `clearance.json`；有硬违规的路线导航程序拒绝执行 | Mac / AGX | ✅ 新路线 0 违规 |
| `scripts/loc_keeper.py` | 不经厂商网页恢复地图和位姿 | AGX | 🧪 离线 / 台架验证 |
| `web_entry/`（只存我们的补丁） | 队友网页适配器与 103 上页面的补丁：速度从配置读、路线列表、定位恢复、网页急停、从某 WP 起跑。队友的原始代码不入库 | AGX / 103 | 🧪 台架验证：终端启动的影子运行约 3 s 交还 |
| `tools/asdu_mode.py` | 读机器人状态；切换“使用模式”（遥控 ↔ 导航），切导航需要 `--i-am-on-site` | Mac / AGX | ✅ 首次自主运行靠它进导航模式 |
| 开机自启 `s10-stack.service` | AGX 用户级服务：x_nav 容器 → 网关 → 运控空跑 → 采集助手。106 的 tap 只在会话激活时启动 | AGX | ✅ 已装 |
| `/teach` 采集助手 | 见 2.1 | AGX | 见 2.1 |

- **`/ODOM`：** 新狗上 106 的官方定位按决定不开，定位用 x_nav 自己的 SLAM。
- 整体计划见 [NAVIGATION_DESIGN_ZH.md §3](NAVIGATION_DESIGN_ZH.md)；新狗验收记录见 [ros1_gateway/README_ZH.md](../ros1_gateway/README_ZH.md)。

### 2.4 仿真与评估

| 名称 | 用途 | 在哪 | 状态 |
|---|---|---|---|
| [`sim_full_course`](../sim_full_course/README_ZH.md) | v3 点云 → 2.5D 地形；运动学机器人；与真机共用感知合同；可加障碍 | 本仓库 | 🧪 纯追踪 30/30；route_v2 规划器 24/30 |
| `route_follow_mujoco`（第一版） | 跟线 + ClimbRouter + J3100 / 1150 跑完 30 个 WP | s10-rl-sprint `scripts/tools/` | 🧪 30/30，713 s |
| 复现包 full_v3 | 上一项的冻结副本（代码、policy、地图、结果） | 本地 `GOAI/S10_Nav_FullV3_Repro_20260919/`，未上传 | 🧪 复跑结果逐 tick 一致 |
| `route_rl_real_stack` / `route_rl_matrix` | 用 `rl_nav` 驱动 MuJoCo，加定位偏置、推离路线、障碍，多种子对比 | s10-rl-sprint | 🧪 在跑 |
| policy 能力测量 | 测台阶、坡、横坡、转向，写入 `policy_profile.json` | s10-rl-sprint | 🧪 |
| Isaac Lab 训练 | 任务 A / B / C 与验收脚本 | s10-rl-sprint（GPU 服务器） | 🧪 A18@50 未过验收 |

### 2.5 地图与数据

| 名称 | 用途 | 状态 |
|---|---|---|
| [`tools/wp_match`](../tools/wp_match/README_ZH.md) | 照片时间戳 → v3 关键帧位姿 → 30 个 WP 和中心线 → `route_v2.json` | 草稿，误差半径 1.5–3 m，要现场重测 |
| [`data/deliverables/S10_v3_Map_MuJoCo_20260916/`](../data/deliverables/S10_v3_Map_MuJoCo_20260916/README.md) | v3 全场点云、Start/B 精细 MuJoCo 场景、离线预览 | ✅（Git LFS） |
| `data/map-reviews/0914_fr_v3-20260914-142008/` | v3 地图的审阅、重建和 policy 试验 | 资料 |
| `data/waypoint-photos-20260914/` | 30 张 WP 照片。**正式顺序与照片顺序相反**：WP01 是最后一张 | 资料（LFS） |

### 2.6 运维

| 名称 | 用途 | 状态 |
|---|---|---|
| [`tools/s10_remote_access`](../tools/s10_remote_access/) | Tailscale 用户态 + OpenSSH，`ssh s10-48-remote` | ✅ |
| systemd 单元 | 102 网页；103 热点转发；106 field worker、native-nav | ✅（48 号） |
| 106 PTP 时间门禁 | 给 5 个原厂服务加 drop-in | ✅（改动了厂商板） |
| SDK 副本和台架 GUI | `prepare_s10_sdk_copy.py`、`start_s10_gui.cmd` | ✅ |
| `Jackdev` 手动 ONNX 前端 | 多个模型的起身、方向控制、趴下 | ✅ speedturn2000 试过 |

### 2.7 历史：8 月仿真赛

包括比赛全栈（MuJoCo 感知 + follower + strategy_router / Gate16 + SDK）、分段实验、视频、打包脚本和 WASD 手控。入口见 `README_V1_ARCHIVE.md`（已归档） 和 `SUBMISSION.md`（已归档）。

## 3. 代码在哪

| 位置 | 内容 |
|---|---|
| 本仓库 `main` | 导航栈、手机页面、ROS 1 网关、仿真、地图包、文档 |
| `bowenwan6/s10-rl-sprint`（私有） | Isaac Lab 训练、MuJoCo 全程仿真工具、J3100 / 1150 实验 |
| 本仓库 `Jackdev` 分支 | Jack 的 phase policy、手动 ONNX 前端、speedturn2000 实机记录（与 `main` 分叉，未合并） |
| 只在本地 | 复现包 zip（97 MB）；原始 MCAP / 录包；x_nav 授权文件；AGX 上的 SDK 副本和 HIM 1500 模型 |

## 4. 已知缺口（按先后）

1. **真机导航接下来要做的（09-20 首跑之后）：**
   - 分级限速：运控硬限幅现在固定 0.30 / 0.10 / 0.50，需要按阶段可配。
   - 上电期间隔离 `/web_cmd`，避免 x_nav 网页发起立 / 趴下或改速度来源。
   - 台阶要先查清 AGX 复位原因；长路线、提速逐级放开。
   - 只有真跑能回答：`/NAV_CMD` 停发后狗是否自己停；导航模式是否每次都必须切。
2. **J3100 / 1150 仍然只在仿真里。** 真机上关节控制在 103 内部，我们下发的是速度指令 + 原厂步态；1150 在台阶上轮速还超 30 rad/s 诊断线，而且不能在 Isaac Lab 里微调（见 1.2），要换只能重训一个 59 维的。
3. **x_nav 的问题：**
   - `map_manager` 仍缺（手册里导航点、虚拟障碍物的保存可能依赖它）；
   - 外参是默认值；
   - 8000 / 9000 / 8765 端口对所有网卡开放，开发页用默认密码。
4. **户外还没做：** WP 重测、x_nav 地图与 v3 的配准、整条赛道的路线重建。
5. **说法不一致，待核实：**
   - 没有 Ver1.0 带 stairs_stable 跑全程的证据。
   - “stairs_stable”一名多用：Ver1.0 的 model_599、另一个模型 stable499、9 月 `rl_nav` 里装 1150 的槽名。
   - 两个“官方 57D”是不同文件。
   - 手工修过的路线段数有 8 段、9 段两种说法。
6. **s10-rl-sprint 的工具写死了本机路径**（`/Users/…/GOAI/wt-*`），换机器要改。
7. **J3100 缺训练记录**；横坡数字只写在 README 里，没有原始 JSON。
