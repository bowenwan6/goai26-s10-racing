# 第一梯段：实录运动与弧形楼梯匹配

2026-09-09。本次完成 `gait_20260909_145331_616f9d8038d9` **0.053–28.953 秒的第一梯段运动学匹配**：接近、完整上行、到达平台。不是整条 93 秒录制的全部梯段，也不是自主控制策略通过试验。

## 直接看结果

- `matched_kinematic_replay.mp4`：约 29 秒、20 fps 的机器人与楼梯共同回放。
- `matched_replay.usdc`：约 4.4 MB、自包含的完整 USD 场景，内含 17 个机器人视觉网格、楼梯、地面、相机和 578 帧动画。无需另找机器人网格。可在 Isaac Sim 中打开并拖动时间轴查看；机器人是运动学动画，不是受控动力学 articulation。
- `matched_stairs.usda`：单独的 11 级静态碰撞楼梯，米制、Z 向上。
- `contact_matched_motion.npz`：5,780 个原关节时间样本，含匹配后的机身 XYZ/四元数、实测关节角/速度、点云原始位置估计和接触修正量。
- `scene_fit.png`、`curvature.png`：地形与点云拟合检查。
- `final_validation.json`、`contact_alignment.json`、`wheel_geometry_check.json`：数值验收证据。

USD 已用 OpenUSD 26.8 重新打开，检查自包含、动画范围、17 个视觉网格和 275 个静态碰撞盒。视频和接触检查在本机 MuJoCo 3.11.0 完成；**本轮没有启动 Isaac Sim，也没有运行策略或物理跟踪 rollout**。

## 匹配到了什么

| 项目 | 结果 |
|---|---|
| 原始输入 | 前后雷达各 290 帧，约 10 Hz；原关节约 200 Hz；IMU |
| 楼梯级数 | 第一梯段 11 级；终点四轮均越过最后一级，处于平台上 |
| 记录尺寸 | 高 15 cm、深 55 cm，保留在原 manifest 中 |
| 点云拟合尺寸 | 高 15.62 cm、深 56.94 cm；这是带标定误差的几何估计，不改写现场测量 |
| 初始第一边缘 | 约距机身原点 45–50 cm，取决于固定尺寸或自由尺寸拟合 |
| 运动 | 先后退约 1.2 m，再接近上楼，末尾前移约 6.7 m、上升约 1.7 m |
| 接近初始曲率证据 | 弧形边缘拟合中位误差 1.50 cm，直线为 4.45 cm；P90 3.87 / 10.53 cm |
| 最终点云验证 | 145 个留出扫描、105,340 个近距离地形点；残差中位 1.60 cm、P90 5.75 cm |
| 原始位置估计对比 | 相同验证点在接触修正前的中位 / P90 为 1.44 / 5.03 cm |
| 接触约束 | 全部 5,780 个关节时刻无大于 2 mm 的模型地形穿插；20 Hz 视频复核亦无穿插 |
| 终点轮底到平台 | 约 0.75–1.92 cm；这是模型几何距离，不是测得的接触力 |

弧线使用局部二次曲线 `x = edge + k*depth + b*y + c*y²`。初始静止扫描与全段拟合的曲率不同，因此不把单个“圆半径”作为已知真值，也不声称完整现场楼梯一定是同心半圆。楼梯横向宽度 5 m、顶部平台延伸长度 3 m 为局部回放范围假设；不代表测量出的场地边界。

级数用终点地形作了区分：保持其他拟合参数不变，末端验证点对 10 / 11 / 12 级的距离中位数约为 17.1 / 3.1 / 9.2 cm，支持 11 级。

## 相比第一次，解决了什么

第一次只在约 2 Hz 的抽样点云中独立拟合直线楼梯，容易把重复台阶错配一级。本次读取原始约 10 Hz 双雷达，先按 IMU 姿态对齐，再用整个局部环境做点到平面配准，保留连续位移。使用起终点静止扫描建立固定参考地图，避免把每一时刻的运动误差都吸收到地形里。

由此区分了真实后退与匹配跳变，重建了第一梯段连续路径。弧形边缘解决了横向位置不同导致的台阶边缘偏移。地形尺寸用注册后的近距离点云拟合，远处低分辨率点和超出粗地形带的物体不参与该拟合。

仅用点云位置回放仍有少数明显穿插。因此最终对**估计的机身平移**施加不穿插约束，并做短时平滑；关节角、关节速度、接触修正前的机身方向全部保持不变。没有把实际关节反馈伪装成教师目标动作。

## 精度边界：修正量公开保留

接触修正相对点云位置估计：中位 **0.71 cm**、P95 **6.00 cm**、最大 **13.33 cm**；有 5 个原关节样本超过 12 cm。字段 `lidar_root_position_m` 和 `root_contact_correction_m` 保留修正前位置与差值。这些较大修正是低置信度位置区间，不可当作实测世界轨迹真值。

最终相邻 5 ms 样本最大位移约 1.16 cm，估计瞬时速度峰值约 2.32 m/s。回放连续且通过几何检查，但高频位置/速度和接触切换仍是重建量；不能未经动力学跟踪评估就用于高权重速度模仿奖励。

点云验证的留出指交替原始扫描不参加地形尺寸拟合；它们仍共享固定地图和配准流程，不是独立定位真值。残差统计限定近距离、宽度和高度范围，不能解释成全场景定位精度。当前仍信任消息声明的 base_link，未完成外参和跨板时钟的独立实测标定。

## 文件字段

- `time_s`、`joint_source_ns`：相对录制开始的真实关节源时间及纳秒源戳。
- `joint_position_rad`、`joint_velocity_rad_s`：SDK 的 16 关节顺序，经 JOINT_DIR / POS_OFFSET_DEG 转换，轮子累计角保留。
- `root_position_m`：最终估计位置；原关节时刻取样。
- `root_quaternion_wxyz`：IMU 姿态与点云配准方向修正；世界航向原点采用本段初始航向。
- `lidar_time_s`、`lidar_root_poses`：原约 10 Hz 点云时刻的位置/方向估计。
- `lidar_root_position_m`、`root_contact_correction_m`：最终位置的来源和修正量。

坐标原点为初始机身原点，地面因此在约 z=-0.437 m；不要导入后再无条件把地面移动到 z=0 而不同时移动机器人。

## 复现

在仓库根目录，按以下顺序执行。原始录制与 SDK 路径沿用现有复核工具；不连接机器人，不改原始记录。

```powershell
python -s artifacts/s10-recording-review-20260909/reconstruct_stairs.py
python -s artifacts/s10-recording-review-20260909/refine_stair_trajectory.py
python -s artifacts/s10-recording-review-20260909/fit_curved_stairs.py
python -s artifacts/s10-recording-review-20260909/build_stair_scene.py
python -s artifacts/s10-recording-review-20260909/export_stair_motion.py
.venv-win/Scripts/python.exe artifacts/s10-recording-review-20260909/constrain_stair_motion.py
.venv-win/Scripts/python.exe artifacts/s10-recording-review-20260909/constrain_stair_motion.py --smooth
python -s artifacts/s10-recording-review-20260909/validate_stair_match.py
.venv-win/Scripts/python.exe artifacts/s10-recording-review-20260909/render_stair_motion.py --contact
.venv-win/Scripts/python.exe artifacts/s10-recording-review-20260909/export_stair_usd.py
```

先普通约束一次，再执行一次 `--smooth`，不要反复对最终结果重复平滑。自检覆盖合成 ICP 位姿恢复、弧形边缘参数恢复、地形距离、接触约束投影、关节数据不变、无穿插和点云一致性。科学计算复用本机 Anaconda；rosbags 沿用 `tmp/s10-analysis-deps`；新增 OpenUSD 验证/导出依赖 usd-core 26.8 只装在该本地临时依赖目录。

这次的“匹配完成”限于第一梯段的**几何与运动学参考重建**。剩余两个梯段未处理；原厂低层动作仍未录到；Isaac 动力学跟踪、模仿学习和自主上楼是后续独立验证。
