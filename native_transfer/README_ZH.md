# 官方原生策略：我们的 follower / router 接入

当前交付是可测试的接入实现。实机策略调用、Start 和 Area B 通过率尚未验收。

复用 `WaypointFollowerNode` 的完整跟随逻辑与 `strategy.Router` 的基础检查；`NativeGaitRouter` 根据路段选择 0x3002/0x3003，管理停稳、切换、反馈确认及输出权限。官方策略在机身运行，不加载外部 SDK actor，也不发布关节指令。

## 已验证

- 78 项本地单元与回归测试通过。
- 在 102 的 localhost / ROS domain 211 中，以真实 ROS 消息、真实 follower 和合成反馈，分别验证平地及楼梯模式的切换、反馈等待、限速与取消。两项均通过，均未连接机器人控制域。
- 从 106 原装消息定义生成了 102 独立 overlay，Gait、NavCmd、MotionInfo 的序列化往返通过；没有覆盖既有 SDK 工作区。
- 106 的只读观察运行成功，读取到运控、里程计和定位状态，未创建 NAV_CMD/GAIT 发布器。观测到 MOTION_INFO 约 1.6ms、ODOM 约 29ms 的数据年龄。

首轮实机进程使用 **106**。102 上存在消息时间戳检查失败，暂不作为实机控制进程；106 已能直接读取 `/NAV_POINTS`（base_link，约 3350 点、数据年龄约 46ms）和原始 `/LIDAR/POINTS`。这不等于坐标外参、点云处理和地形投影已通过验收。

## 路线和验收次序

`config/start.draft.json` 为 5 个目标；`config/b.draft.json` 为 14 个目标；`config/start_b.draft.json` 为 18 个目标，结束于 B 顶部平台，不扩展至 B 后路线。

`kind` 表示到该目标的路段使用的模式。B 的三段楼梯使用 stairs，两处中间平台与最终平台使用 flat。根据现有模型插入了整机越过台阶后的切换点，以及下一段楼梯前的预切换点，暂取 0.70m 距离。**位置、机身清空距离、地面 Z 到 ODOM 的转换及横向净空均需现场复核。** `prepare_routes.py` 可从原始文件重建草稿，不修改地图或模型。

1. 现场净空、持遥控器并进入原生 RL 状态后，先分别做平地、楼梯的零速度步态反馈测试。
2. 两个模式分别做 0.20–0.30m 的受控速度响应试验，验证方向、停止和取消。该入口仅接受两点短路线，限速 0.10m/s，最多 8s；不能借此运行整条路线。
3. 策略调用及停止验收通过、全局定位稳定、路线与感知校验完成后，先 Start 平地、再 B 首段楼梯，再整个 B。
4. 各段重复两次通过后串联 Start→B。XY/Z 到点容差各 0.20m；平地上限 0.20m/s、楼梯 0.15m/s。过程错误、人工接管和失败单列，不计为成功。

详细计划见 `../docs/NATIVE_START_B_ACCEPTANCE.md`。

## 运行方式

106 已部署到 `/home/user/goai_native_start_b_20260917/code`；102 副本位于 `/home/golai/goai_native_start_b_20260917/code`，原生消息 overlay 位于同目录上一层的 `native_ws`。

106 的只读检查示例（输出文件必须是新文件）：

```bash
bash /home/user/goai_native_start_b_20260917/code/native_transfer/run_on_106.sh \
  --config native_transfer/config/field.pending.json \
  --route native_transfer/config/start_b.draft.json \
  --output /home/user/goai_native_start_b_20260917/runs/observer-new.jsonl \
  --probe-gait flat --duration 15
```

此示例不创建原生运动发布器。去掉 `--probe-gait` 可检查完整 follower 输入链。默认 pending 配置故意保留未确认项，不能开启运动。

正式测试使用经实测填写的独立 field 配置，才使用 `--enable-motion`。它只开放 `/native_start_b/arm`（std_srvs/Trigger），不会自动启动。`/native_start_b/cancel` 始终可用；触发停止后必须重启测试会话。没有自动起立、解除急停或切回外部 SDK 的路径。

零速度策略测试用 `--probe-gait flat` 或 `--probe-gait stairs`。速度验收用 `--velocity-probe` 与现场采集的两点短路线。正式路线使用普通运行模式，Start→B 低速测试需预留约数分钟，运行参数可设 `--duration 600`。

路线测试还需在 106 单独运行 `map_context.py`，其生命周期覆盖整个试验。该程序只读取激活地图、localization 服务会话和新鲜全局模式日志，发布诊断元数据；未确认全局模式、地图改变或服务重启均不放行。

## 尚未通过的现场条件

- 本次机器人尚未放到 Start，未下发真实步态、速度或导航目标。
- 当前定位状态为 3／局部模式，不能当作 v3 全局位置使用。
- NAV_CMD 仍有两个原生发布端。正式试验前必须识别并处理竞争控制源；程序拒绝在当前状态开启输出。本次未停用原有服务。
- 外参、ODOM 参考点、body_z_offset、平台切换点和点云投影仍未标为通过。
- 点云转高度图/扫描采用保守检查，缺失和混合高度保持未知；未知数据可能阻止运行，需要现场验证感知输入，不能直接放宽阈值充当通过。
- 零速度步态反馈成功也不等于实机速度响应成功。程序日志始终区分已确认 gait、实际输出与待验收的速度响应。

本实现的取消与软件看门狗不替代现场遥控急停。发布器进程意外退出后的机身超时行为属于首次受控速度测试的验收项。
