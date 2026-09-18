# S10 真机迁移：阶段 1–2 离线交付

地图：`1209_01_F-20260912-175844`。日期：2026-09-12。

**这是只读采集、输入适配、平地导航核心影子计算与离线测试工具，不是可接管机器人的部署包。**

原离线交付的策略固定在 `defbb719ac3822941479caccdee2f35ef0547cf4`；当时复制的 147 个原文件逐个哈希验证未改变，见 `source_snapshot.json`。当时的工作区是已下载文件子集，不包含 SDK、策略权重或完整赛道资产。

## 2026-09-18 仓库整合

本目录现已整理进 `goai26-s10-racing` 的 `codex/field-assistant` 分支。`source_snapshot.json` 与下方 run-01/02/03 结果保留为 9 月 12 日的历史证据；当前 README、follower 和 XYZ 航点接入已有后续变更，不能再以历史 147 文件哈希全部不变作为当前分支的检查条件。`scripts/real_transfer_verify.py` 保留用于核验原始离线快照。

当前分支在仓库根目录建立 Python 环境并安装 `real_transfer/requirements.txt` 后，可运行整合后的本地测试：

```bash
PYTHONPATH=.:src/s10_auto_nav:src/s10_perception python -m pytest -q tests_real
```

本次整合校验结果另存于 `evidence/github-sync-20260918/`。以下绝对工作区路径、旧版本结果和现场未完成事项均为原交付记录。

## 已完成

| 工作 | 结果 |
|---|---|
| 原策略依赖审计 | 完成，见 [AUDIT_ZH.md](AUDIT_ZH.md) |
| 位姿适配 | 校验四元数和坐标系，显式将 ODOM 参考点转换到基座；不猜测外参 |
| 点云适配 | 解析 PointCloud2 大小端、字段偏移、行填充；经完整姿态和外参转换后生成 yaw 对齐的 13×9 高度图 |
| 高度有效性 | 独立 validity mask/count；缺测、跨层混合、垂直面、裁剪边界均不作为可走地面；不补洞 |
| 输入准入 | 源时间与单调接收时间、地图身份、定位模式、服务会话、位姿/时间跳变检查；故障后不自动恢复 |
| 策略复用 | 原 `PurePursuitController` 和 `LocalPlanner` 核心，加平地限制、XYZ 航点和候选速度限制 |
| 采集与回放 | `ros_observer` 只订阅和存文件；`replay` 在电脑生成候选指令与拒绝原因 |

新执行链没有加载完整 `follower_node`、自动倒退/冲台阶、router、Gate16 或楼梯 actor。原文件全部保留不改。

## 在这台电脑复测

### 本次结果

最终复测见 [run-03/summary.json](results/run-03/summary.json)：原仓库可用范围 **304 通过、55 跳过**；新增 **128 通过、0 跳过**；合计 **432 通过**。新代码 lint、格式检查、编译检查和 CLI 帮助入口通过；147 个原文件哈希无变化。

另有 4 个依赖模型/SDK 的测试文件未纳入可运行集合，以及 1 个原测试因未保护的 rclpy 导入无法在本机执行。首次完整扩展回归的失败原样保存在 [run-01](results/run-01/summary.json)，没有删测试或改原源码来获得通过；最终清单记录了排除理由。[run-02](results/run-02/summary.json) 中新脚本的格式检查失败也保留，后续仅修正该行格式。

### 重跑命令

```sh
cd <home>/Documents/ChatGPT/GOAI/s10-real-readiness
.venv/bin/python scripts/real_transfer_verify.py --output-dir real_transfer/results/my-check-01
```

输出目录必须用新名字，避免覆盖证据。测试只在本机运行，不连接机器人。独立虚拟环境和 `requirements.txt` 固定了依赖；每次的 `summary.json` 记录版本、源码哈希、测试结果和未测范围。

测试包括原导航/路由/几何回归、新输入适配与异常注入，以及实际 MuJoCo 地面/台阶采样。**几何采样不是 S10 全场动力学行走。** 固定 10 个种子、20/100 ms 延迟、±1 cm 高度噪声是合成接口测试；300 ms 延迟被拒绝。这不是实测网络延迟或硬件成功率。

## 明确未完成

1. ROS 2 节点真实运行：本机无 rclpy/ros2/colcon，Docker daemon 未运行。本轮没有启动 Docker、连接机器人或部署脚本。
2. 完整赛道/ONNX 回归：缺 SDK、模型及场景，相关测试未执行。
3. 真机标定：ODOM 参考点、雷达外参/去畸变/自车滤除、时钟和基座对齐水平 scan 待现场核实。
4. 完整在线高程图：目前是单帧保守投影，无时序累积、遮挡推理、地面分类或精确台阶重建。要求全部 117 格有效，真实遮挡下可能持续拒绝计算；不能靠补平或降低检查绕过。
5. 全局定位和真实航点仍未验收/标记。`config.pending.json` 中空外参、空航点和 false 标志是有意保留的阻止条件。
6. **未实现任何运动发布器、接管或使能开关。** `transport_command=[0,0,0]` 只是结果文件中的零值，没有给机器人发停车命令。机器人端 watchdog、物理急停和停车距离必须另行验证。

阶段 1 完成；阶段 2 的离线原型和可用核心回归完成，但不代表完整策略已接入真机。

## 下一次现场补什么

先将机器人放到本次地图内可辨认的位置并保持静止，人员掌握可靠急停。地图切换和运动另外确认。

- 新鲜 `/ODOM`、当前地图的正常全局定位状态/服务会话；记录实际 frame_id、child_frame_id 和物理参考点。
- `/LIDAR/POINTS_MERGED` 的真实字段、坐标系、时间戳、安装外参；确认去畸变和机身点滤除。
- 是否已有合适的 LaserScan：**没有假定真机 `/scan` 可用**。若没有，后续需三维点云投影及可靠的自由/未知空间表示，不能把无点方向当作无障碍。
- 经现场确认的平地短段航点，Z 对应同一个基座参考点，不是地面高度；楼梯另外测量。

确认后填写配置副本，先采集再回电脑回放，不修改 pending 配置掩盖未完成项。采集会消耗板上 CPU/磁盘，只安排短窗口，默认 10 秒、最多 30 秒。

## 回放接口

有已验证配置与采集文件后，在工作目录运行：

```sh
PYTHONPATH=.:src/s10_auto_nav:src/s10_perception .venv/bin/python -m real_transfer.replay \
  --config verified-profile.json --input capture.jsonl --output new-shadow-result.jsonl
```

快照包含 `wall_time`、`monotonic_time` 和 `inputs`；pose/cloud/scan/localization 保留源 stamp 和本机单调 received 时间。完整示例见 `tests_real/conftest.py`，其中全部是合成输入，不是真实航点。LaserScan JSON 中 `null + no_return=true` 为已确认的无回波；单独 null 是未知并拒绝。

轴为 x 前、y 左、z 上，单位米/弧度，四元数 xyzw；高度图 x-major，值为 terrain_z-base_z，不执行 Gate16 的反号/偏置转换。来源：[REP-103](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst)、[REP-105](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst)。

只读采集采用 sensor-data QoS 以兼容 Best Effort 发布器；这不是网络可靠性保证。见 [ROS 2 QoS 官方说明](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst)。

平面 0.2 m/s、偏航 0.3 rad/s、传感器 250 ms 等限制均为本离线平地测试配置，不是厂商安全限值、真实标定结果或楼梯推荐速度。
