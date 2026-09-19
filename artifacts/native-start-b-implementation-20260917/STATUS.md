# 2026-09-17 执行状态

已完成代码实现、部署与不移动检查；实机运动验收等待机器人就位。

| 项目 | 结果 |
|---|---|
| 原有 follower + router 接入 | 已实现；不使用外部 SDK actor |
| 平地 / 楼梯切换 | 0x3002 / 0x3003；停稳、切换、稳定反馈后放行 |
| 本地测试 | 78 passed，见 unit-tests.txt |
| 隔离 ROS 联调 | 两种 gait 均通过，见 wire-tests.jsonl；合成反馈，不是机身调用证明 |
| 106 实机只读运行 | 成功；运控/ODOM 时间戳新鲜；没有原生运动发布器 |
| 102 实机只读运行 | 检出运控/ODOM 时间戳问题；暂不作为首轮控制进程 |
| 点云入口 | 106 的 NAV_POINTS / base_link 有新鲜数据；原始 LIDAR/POINTS 同样可读 |
| 官方策略实机调用与速度响应 | 未执行、未通过 |
| Start / B / Start→B | 未执行、未通过 |

部署位置：106 `/home/user/goai_native_start_b_20260917/code`；102 `/home/golai/goai_native_start_b_20260917/code`。未改机器人原有服务配置，没有停止 SLAM/规划/运控服务，没有下发真实步态、速度、起立或目标点。

Start→B 当前为 18 点草稿；来源是已有地图重建和路段模型，地面 Z、机器人参考点与切换距离仍需实测校验。现场配置继续保持 pending。

NAV_CMD 的两个原生发布端分别关联规划器话题（PLANNER_STATUS 等）及遥控避障处理话题（HANDLE_STEER、OOA_STATUS、HANDLER_POINTS_DEBUG 等）；第二个发布端的服务单元还需核对。正式试验需要处理两者的控制权，不能直接把它们当作充电服务或全部当作不存在。

观察日志：observer-102.jsonl（80 条）、observer-106.jsonl（76 条）、observer-106-full.jsonl（50 条），全部确认 native_publishers_created=false。full 观察记录中的外参和配对失败来自未填写的 field 配置，不是一次运行失败后被标为通过。

第一次现场顺序：净空平地验证两种原生步态回执 → 各自短距离速度/停止测试 → Start → B 首段 → 整个 B → 串联。每一阶段单独记录结果。
