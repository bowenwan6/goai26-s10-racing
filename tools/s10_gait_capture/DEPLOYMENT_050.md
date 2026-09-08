# 50 号机器人临时采集实例（使用结束后待删除）

2026-09-08 用户同意部署独立实例，并要求记住目录，日后删除。现在不删除，不设置自动删除。

- 主机：AGX `10.21.33.102`，用户 `ysc`。
- **待删除的完整目录：`/home/ysc/s10_capture_session`**。
- 页面：http://10.21.33.102:8091 。这是真实 ROS 采集；`http://127.0.0.1:8090` 是本地合成演示。
- 数据：`/home/ysc/s10_capture_session/data/`，每轮一个 `gait_*` 目录，含 manifest.json 和 bag/*.db3、metadata.yaml。
- 独立访问口令：机器人 `data/.access-token`；本地副本 `C:/Users/Lenovo/AppData/Local/S10GaitCapture/real-050-8091.access-token`。不是 SSH 密码，不修改原建图网页登录密码。
- 启动脚本：目录内 `start.sh`；日志 `server.log`；进程号 `server.pid`。没有开机自启。
- 复用了已有源代码、消息编译包及已安装网页依赖，没有联网安装。
- 通信配置 `fastdds.xml` 仅用于此采集进程，使用 UDPv4。默认传输下新用户实例收不到点云，改为 UDP 后前后点云均收到；未修改原驱动或控制配置。
- 本地启动脚本与通信配置副本：`start-agx-session.sh`、`fastdds-session.xml`（部署时分别命名 start.sh、fastdds.xml）。

## 部署验证

当前未开始录制，API `mode=ros`、`active=null`、`error` 为空。实测接收 IMU/关节反馈约 200 Hz，备用关节约 10 Hz，运动状态约 20 Hz，前后点云各约 10 Hz。JOINTS_CMD 尚未收到，不能称为已获得专家动作。

状态快照：仓库 `artifacts/s10-readonly-probe/own-instance-status.json`。频率是这次采集端估计，不是同步精度证明。部署时磁盘剩余约 14.47 GB。

## 使用

用户用原遥控器控制机器人，网页只负责记录。选择地形、填写说明，确认连接后开始录制，结束时点击停止并保存。若 JOINTS_CMD 仍缺失，需要勾选“关键话题缺失时，仅录制诊断数据”；录得的是状态/传感器轨迹，不能直接声称为状态与动作配对数据。

## 日后清理

等用户明确说使用结束或要求删除此实例时执行，当前不执行：

1. 先检查是否仍在录制，停止并保存；确认需保留的真实数据已导出。
2. 核对 server.pid 对应的命令和 `/proc/<PID>/cwd` 确属上述目录，然后停止仅此采集进程。
3. 核实解析后的绝对路径恰为 `/home/ysc/s10_capture_session`，删除这个目录（包含程序、口令、日志和录制数据）。
4. 删除本地口令副本。仓库中的源码、调查资料保留，除非用户另行要求删除。

不要删除 `/home/xwy/s10_gait_capture`、`/home/xwy/s10_gait_data` 或原驱动/建图目录。


2026-09-08：按用户“删除”指令，删除录制 gait_20260908_193835_90d36ab654a4 及同名导出 ZIP（如有）；采集实例仍保留。本地 manifest/metadata 仅为已删除记录的调查快照，不包含完整 bag。
