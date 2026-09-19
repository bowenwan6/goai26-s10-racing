# 048 原厂速度来源交接与明日准备

2026-09-17 23:26–23:35，北京时间。用户明确说明机器人已趴稳，允许去掉两个速度发布程序，明天才做运动测试。今晚仅停止竞争来源并只读核对；未起立、切换 gait 或发布速度。

## 已完成

- 103：通过已安装 `drdds/msg/NodeCtlCmd` 文档化接口，向 `/NODECTL_CMD_103` 发送一次 `action=stop,module_name=handler.service`。`/NODECTL_QUERY_103` 确认状态 4（inactive），PID 0；原 handler 日志停止增长。机器人 `motion_master` PID 2318、`robot_server` PID 7643 持续 active。
- 将 103 `/var/opt/robot/conf/dr_nodectl/dr_nodectl_CD1.yaml` 中唯一 `handler.service` 条目的 `autostart` 改为 false。备份 `/home/user/goai_native_control_20260917/dr_nodectl_CD1.before.yaml`；其余条目保持。未重启 dr_nodectl 或机器人；跨重启行为仍须明天检查。
- 106：发现 `planner.service` 原启动脚本同时运行 `pcl_remove` 和 `localPlanner`，不能整项删除。新增 `/etc/systemd/system/planner.service.d/90-goai-perception-only.conf`，仅把 ExecStart 改为原 `pcl_remove`，保留其原 CPU/调度设置，再重启这一个服务。原二进制、脚本与全局规划器均保留。
- `localPlanner` 进程不再运行；`pcl_remove` PID 994936 运行，`/NAV_POINTS` 约 3864 点、年龄 0.065 秒；`/ODOM` 年龄 0.092 秒。定位会话 `975155295050497a85ac86e9b09196f4` 保持不变。
- 新的只读 DDS 发现 `/NAV_CMD` 发布者为 **0**。停止 handler 后最初 4 秒内旧 DDS 发现缓存仍显示两端，脚本因此明确报检查未通过；没有盲目重复 stop。随后新建观察节点确认两项变更生效、发布者为 0。证据 `handler-stop.jsonl`、`handler-settled.jsonl` 保留这一过程。

“存在发布者”与“当时正持续发送非零速度”不同；此次先前只能确认两个发布端存在。程序停止前，实际运动反馈近零且新鲜。没有以来源退出推断物理停车能力通过。

原厂自动规划和 handler 的遥控避障辅助现被停用。原厂运控与遥控通信服务仍运行，但真实遥控接管/停车必须明天现场验证。旧现场助手中依赖 PLANNER_STATUS 的切图/部分静止门禁可能因 localPlanner 停止而阻断；不能伪造空闲状态绕过。当前 v3 已加载，无需为了明日检查重新建图。若需切图/重定位，应在无自主任务时由工程师处理服务依赖。

## 明天的顺序

1. 把机器人放到 v3 原建图场地的 Start，按路线朝向摆正，先保持趴稳；在 app 的“自动导航测试”点“不运动”的实机检查。
2. 确认加载的确实为 v3，SLAM 回到新鲜、稳定的全局定位。结合现场助手/实时位置图检查点云与墙、楼梯等地标叠合，以及 XY、朝向、Z/楼层；建议连续观察/采集约 30 秒。只有 ODOM 持续输出，或显示 MapOK，不能替代这些检查。仍处于局部或丢定位就先解决重定位。
3. 定位确认后，现场持遥控器起立并验证人工接管/停车，进入原厂 RL 状态。本程序不自动起立。
4. 在净空平地验证原厂普通/楼梯导航模式的原地切换，再分别做约 0.20–0.30m、上限 0.10m/s 的响应与停止检查；这些尚未做过。
5. 完成定位参考点、机身高度、点云到机身坐标、地形输入、路线与楼梯切换点的实机核对。这些工程确认不会由网页复选框自动写成通过。
6. 先跑 Start 平地短段，再测 B 的首段楼梯，再扩大到完整 B 和 Start→B。按原验收计划分阶段记录，不把明天开测等同于已经具备全程能力。

机器人目前不在原场景内，局部定位状态可能与场景不匹配有关；趴着时运动状态不满足行走条件也不能据此判定运控损坏。搬到起点可以开始做上述验收，不能跳过它们直接执行路线。

## 恢复办法（今晚不执行）

- 106：移除本次唯一 drop-in `90-goai-perception-only.conf`，`systemctl daemon-reload`，在无自主任务且已现场接管时重启 `planner.service`，即可恢复原脚本与 localPlanner。原单元和脚本备份在本目录及 106 `/home/user/goai_native_control_20260917/`。
- 103：恢复该 YAML 的 handler `autostart:true`（备份可供差异核对）；需要立即恢复 handler 时，通过同一官方 NodeCtlCmd 接口对 `handler.service` 发 `start`，并查询其状态。不得在我们的 follower 运行时恢复竞争来源。
- 明日开机后先重新发现 `/NAV_CMD` 发布者数量；未对原厂升级、其他入口重新拉起服务等情形作保证，程序仍会拒绝并发控制。
