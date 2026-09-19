# 阶段 1：策略依赖与迁移风险审计

基线：`defbb719ac3822941479caccdee2f35ef0547cf4`。只审计已下载源码；文档中其他机器/其他编号机器人的实验不作为 48 号的硬件验证。

## 数据与控制边界

```text
旧仿真：真值位姿 + 模拟感知 + 仿真赛道坐标
       → 完整 follower → router → SDK actor/关节仲裁 → 执行

本轮：真实采集文件或合成快照 + 明确验证过的配置
       → 时间/地图/坐标/有效性检查 → 原 pursuit/local planner 核心
       → 候选指令与阻止原因 → 仅写本地结果文件（无运动出口）
```

## 发现与处理

下表 follower/waypoints 路径相对 `src/s10_auto_nav/s10_auto_nav/`；配置路径相对 `src/s10_bringup/config/`。

| 风险 | 固定版本证据 | 本轮处理；后续缺口 |
|---|---|---|
| 真值位姿依赖 | `follower_node.py:497` 订阅 ground_truth/odom | 适配 ODOM，要求地图身份与正常全局状态；参考点/初始化/实景配准待验证 |
| 感知失效仍继续走 | `follower_node.py:59,1530,1647,1681` 超时后忽略感知，UNKNOWN 保留速度 | 新层缺测/过期直接不给候选运动并锁定；实际机器人停车 watchdog 尚未实现 |
| 位姿新鲜度不足 | `follower_node.py:510,569` 缓存位姿，循环先检查是否 None | 同时核对源戳、接收戳，拒绝重发、乱序、时间和位姿跳变 |
| 多层环境串层 | `waypoints.py:92,100` 只用 XY 判断到达 | 新平地层检查 XYZ；高度不匹配不前进游标；三维路线拓扑待建 |
| 固定赛道坐标/动作 | `course.yaml:13–16` 来源仿真 overlay；`nav.yaml:90,111` 冲台阶、绝对 route hints | 不加载旧 course/nav，不启用恢复冲刺；真实航点为空，等待现场标注 |
| 模拟高度图不等于真实高程图 | `src/s10_perception/s10_perception/heightmap.py` 向下射线采样 | 显式外参/姿态转换、valid mask、混合层拒绝；多帧融合、遮挡、自车滤除及去畸变待补 |
| -1 数值歧义 | `docs/TECHNICAL_DESIGN.md §4.2` 裁剪深度和 void 共用 -1 | 数值与有效性分开，裁剪边界不放行；后续 RL 接口不得丢掉 mask |
| 无回波不等于自由空间 | `local_planner.py` 忽略非有限范围 | 区分 no-return 与 unknown，要求完整基座对齐 scan；实际 scan 来源未确认 |
| 控制接口单位混淆 | `TECHNICAL_DESIGN.md §3.2` 的 cmd_vel 是 SDK actor 输入；`S10_REAL_ROBOT_QUICKSTART_ZH.md:660` 区分 STEER 比例和 NAV_CMD | 不建立命令出口，不将 m/s 写成 STEER 比例；48 号本体接口、导航模式、唯一控制权待验 |
| 特殊 actor 更多依赖 | `TECHNICAL_DESIGN.md §3.2,4.2,10.2` 有关节、轮接触、174D 观测和 owner ACK | Gate16/楼梯 actor/关节接管均不启用；权重/SDK、关节单位/顺序、力矩及硬件验证待补 |

## 复用的真实范围

- 原 PurePursuitController：航向、横向修正、指令变化率限制。
- 原 LocalPlanner：水平 scan 的候选方向/净空选择。
- 原地形、step-commit、router：运行既有独立回归，未接入新执行链。
- 新层：保守输入门槛、XYZ 航点和只写文件的平地影子计算。

不能把核心成功解释为完整 follower/router/ONNX 链路完成了实景任务。原始源码、配置和测试没有为“让测试通过”而修改，哈希见 source_snapshot.json。

## 现场必须核实

ODOM 原点/子坐标系、odom_child_from_base、base_from_cloud、雷达去畸变/自车滤除、水平 scan 和无回波语义、板间时钟、全局定位与地图绑定、真实航点基座高度，以及本体控制权。未核实项在 config.pending.json 中保持空值或 false。

## 方法来源与安全边界

坐标单位采用 [REP-103](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst)，map/odom/base_link 的角色采用 [REP-105](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst)。时间门槛和扰动量级是本项目离线测试设计，不是标准规定。

Sim-to-Real 技能用于接口合同和渐进验证；Safety System 技能促使完全不实现运动出口、故障后不自动恢复；Point Cloud Processing/TF2 技能用于保留未知区域和明确外参。本轮没有进行功能安全认证。
