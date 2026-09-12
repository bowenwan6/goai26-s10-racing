# S10 基础步态：平面匹配、软参考用法与本次经验

更新：2026-09-11。输入只使用新批次 `2026-09-09` 的录制。**基线已实际运行 Isaac Sim 5.1 自由基座动力学，完成 5 段、34 秒测试；另完成两组各 34 秒的地面接触参数对照，仍未解决直接 PD 的转向、侧向侧翻。** 第 2.1 节解释它与楼梯运动学匹配的区别。原始示范没有因此被判为不合格；当天后续已在本地训练仓库完成少量软参考 PPO 更新，见第 8 节，尚未形成可部署策略。

## 1. 平面加专家动作，是否就够了

近水平地面片段可以先放在平面上测试，结果支持从这些片段启动软参考训练实验。但“复制关节运动”只能提供运动风格和局部姿态的引导；同样的关节运动在另一套接触、摩擦、惯量和控制器中，不一定产生相同的机身速度与转向。

本批录的是 `/JOINTS_DATA` 的实测角度、角速度、力矩，以及 IMU、MOTION_INFO、STEER，缺少 `JOINTS_CMD`。因此导出的是**实测运动参考，不是原厂策略 action 标签**。尤其不能把实测关节角直接当成 16 维网络输出去做行为克隆。

草地是一种材质，坡面是一种几何；两者要分别标注。草地平地可以在第一轮近似成平面，但坡面/起伏片段不能仅靠把机器人整体抬高后就变成平地示教。此次从已有 264 秒近水平运动候选中抽取 34 秒，尚未处理全部 58 个候选，也未把 121.700 秒坡面/起伏候选混入平面实验。

## 2. 本次匹配了什么

交互入口：[本机对照页面](http://127.0.0.1:8767/basic_flat_match/index.html)。静态文件入口：[结果报告](../artifacts/s10-recording-review-20260909/basic_flat_match/REPORT.md)、[逐段指标](../artifacts/s10-recording-review-20260909/basic_flat_match/summary.json)。

| 名称 | 录制简称 | 原始 source 时间 | 内容候选 | Isaac 腿角 RMSE | 机身完整姿态 RMS |
|---|---|---|---|---:|---:|
| forward | 144907 | 12–18 s | 前进 | 2.63° | 4.12° |
| backward | 144907 | 18–24 s | 后退 | 2.80° | 11.68° |
| turn | 144907 | 26–34 s | 转向，仿真侧翻 | 4.86° | 123.35° |
| sideways | 144907 | 62–68 s | 侧向，仿真侧翻 | 4.63° | 90.51° |
| second_recording | 151931 | 24–32 s | 另一条录制的直行对照 | 2.48° | 3.01° |

**最终本地静态平板场景只有 3 段未摔倒；转向和侧向两段侧翻。** 摔倒阈值为机身倾角 >60°或根高度 <0.15 m，转向在片段内 1.14 s 首次触发，侧向在 3.74 s 触发。三个未摔倒片段最大倾角不超过 7.46°；50 Hz 接触记录中，没有非轮刚体的合力超过 1 N。转向、侧向分别有 91、100 个采样时刻出现非轮接触合力。接触检查包含自接触，且没有覆盖每一个 1 ms 的碰撞瞬间。

转向没有匹配好：实录 IMU 平均 yaw 角速度约 1.607 rad/s，MuJoCo 对照约 0.888 rad/s，明显偏慢。Isaac 早期默认地面试跑约 0.754 rad/s、未摔倒，但最终本地静态平板上发生侧翻；以最终报告和视频为准，不能沿用早期“5 段未摔倒”的临时结果。完整姿态误差使用四元数最短角，跨多圈时应同时看图中的连续展开 yaw；侧翻后的平均角速度不再适合作为正常转速指标。

最终直行平均速度也存在偏差：forward 约 0.920 对 MOTION_INFO 报告 1.060；第二条直行约 1.243 对报告 1.373；后退约 -1.137 对报告 -1.218。这里把报告值解释为机身坐标 m/s，是本轮检查假设，尚未独立标定，不自动升级为速度字段有效认证。

建议先用 `forward` 和 `second_recording` 做软参考训练试验，随后加入 `backward`；`turn`、`sideways` 保留诊断，暂不加入首轮试验组。明确名单在 `basic_flat_match/rl_selection.json`，不要用通配符把所有参考 NPZ 一起喂给训练。四段来自同一条 144907，不应随机拆帧后分别算训练/验证集；第二条录制可作独立录制对照，但这里仍没有进行模型泛化测试。

### 2.1 侧翻诊断，以及为什么楼梯动画能匹配

**`rl_selection.json` 中的 `diagnostic_only` 是本轮直接 PD 试验分组，不是原始录制的质量判决。** 转向、侧向仍保留为实测运动软参考候选，人工确认仍待完成；不应因为这个重放控制器摔倒就删除示范。

对照 [楼梯匹配指南](S10_STAIRS_MATCHING_GUIDE_ZH.md)，两种测试实际做了不同的事：

| 项目 | 已有楼梯匹配 | 本轮平地动力学测试 |
|---|---|---|
| 机身位置 | 前后雷达多帧配准估计局部 XYZ，结合地形拟合 | 只设置初始状态，后续由物理求解器决定 |
| 姿态与关节 | 每帧直接放置 IMU 姿态和实测关节角 | PD 跟踪实测关节状态，机身自由运动 |
| 计算 | `render_batch_stairs.py` 写入 `qpos` 后调用 `mj_forward` | `run_basic_flat_isaac.py` 施加力矩并调用 `world.step` |
| 能验证什么 | 运动和地形的几何关系、参考字段与质量 | 这套模型、接触和控制组合能否重现动作 |

楼梯结果是运动学复核动画；根位置由约 10 Hz 点云估计后对齐真实关节时间，第一梯段还做过最大约 13 cm 的接触约束平移修正。它没有验证自由动力学走楼梯。逐帧指定机身姿态的动画，也不会因受力失衡而自行侧翻。

原始两段 IMU 最大倾角分别约 **3.28°（转向）、3.93°（侧向）**，没有显示侧翻。Isaac 基线却在片段内 **1.14 s、3.74 s** 首次触发摔倒阈值。转向在 1.14 s 时，腿关节角 RMSE 仍仅约 3.2°，机身 roll 已约 59.6°。这说明腿角跟得近，不能保证机身平衡。相同关节参考及增益的 MuJoCo 对照未摔倒，但转向速度仍偏慢，也不能称为完整复现。

当前控制是关节层面的闭环 PD，有 q/dq 反馈；**没有依据机身倾斜、角速度或实际平移速度调整参考动作的反馈策略**。实录的 `JOINTS_DATA` 是运动结果，缺少原厂 `JOINTS_CMD`；把实测 q/dq 当目标，并不等于重放原厂控制器。即使 q/dq 完全相同，本轮 PD 输出也会变为零，而真实动作仍可能需要支撑力矩；本轮没有使用实测力矩前馈。基线转向力矩无饱和，侧向饱和比例很低，目前没有证据将主要原因归为力矩限幅。

检查安装版本发现，`FixedCuboid` 为地面设置了 `contactOffset=0.1 m`、`torsionalPatchRadius=1 m`、`minTorsionalPatchRadius=0.8 m`；S10 轮半径约 0.081 m。这是值得对照的接触设置，但不能仅凭数值认定根因。扭转参数的意义见 [PhysX PxShape 文档](https://nvidia-omniverse.github.io/PhysX/physx/5.4.0/_api_build/class_px_shape.html)，接触检测距离的意义见 [碰撞文档](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.0/dev_guide/rigid_bodies_articulations/collision.html)；接触检测距离不等同于机器人实际悬空高度。

保持模型、初始化、参考和控制增益相同，实际完成了以下对照：

| 设置 | 转向 / 侧向 | 其他三段 |
|---|---|---|
| 基线：上述平板默认值 | 均侧翻 | 均未触发摔倒阈值 |
| 两个扭转半径改为 0，检测距离仍 0.1 m | 均侧翻 | 均未触发摔倒阈值 |
| 在上一组基础上，检测距离改为 0.002 m | 均侧翻 | 后退和第二条直行也摔倒；前进最大倾角 25.8°、X 位移仅 0.22 m |

**两组修改均未解决问题，不作为推荐参数。** 目前证据支持“这套直接 PD、模型和接触组合未能稳定复现”，尚不能锁定单一物理参数。楼梯参考重建与动力学验收应分别记录：软参考可以先使用关节和 IMU；若需要世界轨迹，再按楼梯方法补局部点云配准。当前平地包的 `root_position_valid=false` 不能提前改成有效。

对照输出分别在 [去扭转参数记录](../artifacts/s10-recording-review-20260909/basic_flat_match/ground_contact_check/isaac_asset.json) 和 [小检测距离记录](../artifacts/s10-recording-review-20260909/basic_flat_match/ground_contact_check_small_offset/isaac_asset.json)，各目录的 `<片段名>/isaac.json` 是结果。所有对照参考数组已逐字段检查，与基线完全相同；基线指标及人工标注保持原样。复现时需已有本机同名 Docker 容器及仓库挂载：

```powershell
docker start s10-basic-flat-match
docker exec s10-basic-flat-match /isaac-sim/python.sh artifacts/s10-recording-review-20260909/run_basic_flat_isaac.py --ground-torsion 0 0 --output-dir artifacts/s10-recording-review-20260909/basic_flat_match/ground_contact_check
docker exec s10-basic-flat-match /isaac-sim/python.sh artifacts/s10-recording-review-20260909/run_basic_flat_isaac.py --ground-torsion 0 0 --ground-contact-offset 0.002 --output-dir artifacts/s10-recording-review-20260909/basic_flat_match/ground_contact_check_small_offset
docker stop s10-basic-flat-match
```

可用 `--only turn` 单独复现一段；未传地面参数时保留原平板默认设置。`--output-dir` 隔离结果并复制既有参考，不重新裁剪源录制。

## 3. 参考是怎样对齐的

1. **时间**：`source_time_s = (source_stamp_ns - manifest.started_wall_ns) / 1e9`。每个包保留精确 int64 源纳秒，以及片段内从 0 开始的 `time_s`。原生关节样本另存用于追溯。
2. **采样**：此次选中区间关节/IMU 约 200 Hz，参考导出为 200 Hz 网格。线性插值关节和连续值，IMU 四元数先统一符号，再邻帧线性插值并归一化；只消除片段初始 yaw。没有把 50 Hz 实测数据称为 200 Hz 实测。没有进行 50 Hz 降采样；后续若要降采样，需先考虑抗混叠。
3. **连续性**：关节和 IMU 插值间隔不得超过 60 ms，MOTION_INFO 不得超过 120 ms；区间保持 motion_state=17、gait_state=4097。离散状态以前值保持取样。检测到无效区间应重新裁剪，不能删掉间隔后把两端拼在一起。
4. **关节**：沿用 SDK `mujoco_simulation_ros2.py` 的 `JOINT_DIR` 和 `POS_OFFSET_DEG`：`q_model = q_raw * direction + offset`，`dq_model = dq_raw * direction`。关节按名称重排，不能沿用 Isaac 自动返回的数组顺序。
5. **轮子**：保留速度，将累计角度减去片段第一帧作为显示相位；控制和软奖励都不要求轮子累计角度逐帧相同。
6. **初始状态**：关节 q/dq、IMU 姿态/角速度取首帧；用 SDK FK 将最低轮心置于地面上方 0.083 m（半径 0.081 m 加 2 mm 间隙）。初始 XY=0，初始根线速度暂按 MOTION_INFO 的 vx/vy 机身坐标假设旋转到世界坐标。只有初始化设置基座，之后不锁定、不逐帧移动基座。

没有测得的世界 XYZ 不填成真实路径。数据包中 `root_position_valid`、`contact_label_valid`、`command_valid`、`reported_velocity_valid` 都为 false。STEER 的归一化原值、源戳和接收戳单独保留，但未完成时延/量纲标定，不能直接当 m/s 或 rad/s 的训练指令。

IMU 的姿态和角速度本轮按机身轴约定使用，安装外参尚未独立核验；较小的重力方向误差不能证明安装关系已完成标定。

## 4. 动力学与环境约定

模型来自仓库 SDK 的 `S10_description/s10_mjcf/mjcf/S10.xml`。Isaac 使用官方 MJCF 导入器创建可驱动 articulation；已核对 16 个关节名称、17 个刚体质量和无固定根关节，详见 `basic_flat_match/asset_check.json`。

模型的腿/轮按机器人顺序交错排列：FL、FR、HL、HR，每组 hipx、hipy、knee、wheel。当前 Isaac 的 DOF 顺序却按关节层级排列；脚本通过每个 `joint_names` 查索引，避免静默错位。训练代码的 `POLICY_ORDER` 又是 12 个腿关节在前、4 个轮关节在后，接入时需要再次按名称映射。

| 参数 | 本轮设置 |
|---|---|
| 场景 | 本地 200×200×0.1 m 静态薄板，上表面 Z=0，单位米，重力 9.81 m/s² |
| 地面 | 静/动摩擦系数 1，恢复系数 0；是测试假设 |
| 腿控制 | `torque = 80*(q_ref-q) + 2*(dq_ref-dq)`，限幅 ±50 Nm |
| 轮控制 | `torque = 0.6*(dq_ref-dq)`，限幅 ±14 Nm |
| 物理步长、力矩更新 | 1 ms；参考 200 Hz，步间线性插值 |
| 结果记录 | 50 Hz 状态、接触合力；饱和比例按 1 kHz 累计 |
| Isaac 求解 | PhysX CPU，numpy；16/4 次位置/速度迭代 |

关闭导入器默认关节驱动后显式施加上述力矩，避免隐式 PD 与显式 PD 叠加。不使用实测力矩前馈，也没有给机身加外部姿态修正力。MuJoCo 用相同目标和增益作对照，两者接触实现不完全相同。

本机镜像名称为 `rl-training:isaaclab-2.3.2`，直接回放使用 Isaac Sim 5.1.0。2026-09-11 后续核验发现镜像实际包含 Isaac Lab；旧回放容器把仓库挂载到整个 `/workspace`，遮住了 `/workspace/isaaclab`，因此此前在该容器中未找到 `isaaclab` 包。训练容器应只挂载到 `/workspace/rl_training`。CUDA 能识别 RTX 4060 Laptop 8 GiB，但本机 Docker/WSL 的渲染与 PhysX CUDA 初始化仍有错误日志，本次明确使用 **CPU 物理**完成计算；没有验证 GPU 并行训练或 RTX 渲染。

## 5. 怎么作为 RL 的软参考

现成读取与奖励示例在 [match_basic_flat.py](../artifacts/s10-recording-review-20260909/match_basic_flat.py) 的 `soft_motion_reward()`。它比较四类量：12 个腿关节角、4 个轮速、机身重力方向、机身角速度。默认权重为 0.5/0.2/0.2/0.1，误差尺度为 0.25 rad、5 rad/s、0.25（单位重力向量）、1 rad/s，均为**尚未训练调优的起点**。

```python
import sys
from pathlib import Path
import numpy as np

review = Path('artifacts/s10-recording-review-20260909')
sys.path.insert(0, str(review))
from match_basic_flat import soft_motion_reward

ref = dict(np.load(review/'basic_flat_match/forward/reference.npz'))
i = 200  # 片段内 1 秒，对应这条录制的 source 13 秒
# q、dq 按 ref['joint_names'] 排列；示例用同帧实测值，实际替换成仿真状态。
r_motion = soft_motion_reward(ref, i, ref['joint_position_rad'][i],
    ref['joint_velocity_rad_s'][i], ref['projected_gravity'][i],
    ref['angular_velocity_body_rad_s'][i])
assert np.isclose(r_motion, 1.)
```

接入 PPO 时将相同四项改为 torch 批量运算；奖励只读示教，策略输出的是自己的动作。总奖励应同时包含指令速度/转向跟踪、稳定站立、合理力矩和动作平滑等任务项，再加入适度权重的 `r_motion`。允许策略偏离实录以适应仿真的接触条件，不能再用外部 PD 强制把 RL rollout 拉回参考姿态后评价策略。

更大数据集也可考虑运动先验方法：[AMP 作者项目页](https://xbpeng.github.io/projects/AMP/) 展示了将动作数据形成的风格奖励与任务奖励结合的路线。本次实现是四项显式跟踪奖励示例，没有训练 AMP 判别器；该研究不构成本批参考训练效果的证明。

开始可按连续片段随机初始化示教相位、设置对应机器人状态；片段结束应结束 episode 或转入已有稳定控制。不要把首尾不连续的转向/侧向片段强行循环，也不要一直执行末帧非零轮速。

本项目基础观测定义在 `training/s10_rl/observation.py`：机身角速度、重力、速度指令、关节状态和**策略自己的上一动作**，不含雷达。软参考可仅用于训练奖励；如果训练时把未来示教或相位输入策略，部署时必须提供同样信息，或另行训练可部署学生。不能在实机运行时假定知道未来实录动作。

在 STEER 时序/量纲未校准之前，不适合宣称已经完成“原始指令→原厂动作”的监督学习。第一轮应使用定义清楚的仿真指令做任务训练，或者先校准实录 STEER；不要把归一化指令与报告速度混为一列。

`reference_valid` 仅检查数据/控制连续性，不代表人工认可、无打滑或 RL 已可用。所有包仍标 `training_ready=false`。人工复核后可以选择片段进入训练试验，但还要用完成率、速度/航向误差、摔倒、力矩和留出录制评估训练结果。

## 6. 队友如何复现

从仓库根目录运行。需要本机 SDK 和新 17 段生成的 `decoded/*.npz`、`.meta.json`，以及 `basic_gait_clips.json`。原始包从团队 Google Drive 获取，目录与解码步骤见 [复核交接文档](S10_RECORDING_REVIEW_AND_RECONSTRUCTION_ZH.md)。直接复现本次仿真可以使用已导出的 5 个 `reference.npz`；只有重新裁剪才需要 decoded。

本机 `.venv-win` 提供 NumPy/MuJoCo，`python -s` 为带 Matplotlib 的 Anaconda；换电脑可以统一到一个装有这些包的 Python 环境。`--check-asset` 额外使用已有 usd-core，视频使用 ffmpeg。

```powershell
# 如果人工筛选有更新，先重建筛选索引。
python -s -B artifacts/s10-recording-review-20260909/organize_basic_gait.py
.venv-win/Scripts/python.exe -B artifacts/s10-recording-review-20260909/match_basic_flat.py prepare
.venv-win/Scripts/python.exe -B artifacts/s10-recording-review-20260909/match_basic_flat.py selfcheck

# 可选的 MuJoCo 动力学对照。
.venv-win/Scripts/python.exe -B artifacts/s10-recording-review-20260909/match_basic_flat.py mujoco
```

启动 Docker Desktop 后，本机已有 `s10-basic-flat-match` 容器时执行 `docker start s10-basic-flat-match`。新机器首次创建（镜像需自行具备，仓库未分发镜像）：

```powershell
docker run -d --name s10-basic-flat-match --gpus all --ipc=host -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=N --mount "type=bind,source=$($PWD.Path),target=/workspace" -w /workspace --entrypoint /bin/bash rl-training:isaaclab-2.3.2 -lc 'sleep infinity'

# 实际执行 Isaac；增加 --only forward 可先跑一段。
docker exec s10-basic-flat-match /isaac-sim/python.sh artifacts/s10-recording-review-20260909/run_basic_flat_isaac.py

# 本机生成图表、视频以及资产核验。
python -s -B artifacts/s10-recording-review-20260909/report_basic_flat.py
.venv-win/Scripts/python.exe -B artifacts/s10-recording-review-20260909/report_basic_flat.py --video
.venv-win/Scripts/python.exe -B artifacts/s10-recording-review-20260909/report_basic_flat.py --check-asset

# 若复核服务尚未运行。
.venv-win/Scripts/python.exe -B artifacts/s10-recording-review-20260909/review_server.py --port 8767
```

打开 `http://127.0.0.1:8767/basic_flat_match/index.html`。每段有视频、曲线、指标和跳回原录制的精确时间链接。**视频左侧**固定机身 XYZ，仅更新实录 q/IMU 姿态，用于看清动作；**右侧**为 Isaac 自由动力学记录。两侧均用 MuJoCo 渲染器画 SDK 网格，便于本机观看。固定 XYZ 的左侧没有物理接触求解，视频不是相机录像或恢复后的实录世界路径。

产物位于 `artifacts/s10-recording-review-20260909/basic_flat_match/`：每段的 `reference.npz`、`source.json` 为参考及来源，`isaac.npz/.json` 为物理结果，`isaac_contacts.npz` 为采样接触合力，`mujoco.npz/.json` 为对照，`comparison.png/.mp4` 为复核。根目录的 `isaac_flat_scene.usdc` 是含碰撞与关节的静态物理资产；动作靠脚本执行，不是打开 USD 就自动播放。

源码、方法、JSON 指标、HTML 与曲线图可进 Git；参考 NPZ、物理日志 NPZ、USD、视频是可再生成产物，当前忽略规则仍将其留在本地。队友若不重跑，需把整个 `basic_flat_match` 结果文件夹一并放到 Drive；不能只传 README 后假定 NPZ/视频会从 Git 出现。本次未自动上传新产物，也未提交或推送这些变更。

## 7. 这次留下的经验

- **分段在地形名称之前。** 从 basic 录制中选近水平片段，保留 source 时间、原始索引和复核入口；草地与坡面分开判断。
- **先检查初态和关节顺序。** Isaac、SDK 和 policy 的数组排列不同。本次按名称映射，并核验质量与自由根，避免漂亮动画掩盖错误模型。
- **关节误差与机身误差分开验收。** 转向腿角误差不大，但机身会偏航甚至侧翻；侧向也在最终场景侧翻。接触/摩擦、轮子侧滑、模型参数和原厂闭环修正可能有关，本轮没有通过参数辨识确定原因。
- **不为了通过而改示教。** 五段用相同增益，没有对转向单独放大轮速或删除失败结果。让 RL 在任务奖励约束下学习必要修正，比宣称原动作已完全复现更符合本次证据。
- **环境故障单独处理。** 本机 Docker 再次出现 `dockerInference` 和 `docker-secrets-engine/engine.sock` 旧 socket 无法访问；在 Docker 停止后备份并重建对应运行目录恢复，镜像仍在。没有做 factory reset。MJCF 导入器对绝对 meshdir 处理出错，改用相对路径；导入生成的空 `worldBody` articulation 被移除。接触传感器的多路径模式需要同数量的过滤列表。在线默认场景改为本地支撑面后，原生 plane 出现材质索引警告；最终使用上表面 Z=0 的静态薄板，消除了该警告。此更改也改变了转向/侧向结果，因此完整保留最终失败，不能把几何上同样平坦当成接触实现完全等价。
- **当前只是软参考的起点。** 三段在 1 kHz PD 下未摔倒，不证明 50 Hz 网络决策与真实执行器延迟下同样稳定。软参考接入可以与侧向/转向的直接 PD 诊断分别推进；坡面需另配几何和分组采样。

## 8. 2026-09-11：接入本地训练仓库

后续已在相邻 `rl_training` 仓库加入 `scripts/reinforcement_learning/rsl_rl/s10_flat_reference.py` 与 `s10_flat_reference_smoke.py`，复用已有 S10 57D 平地环境和官方 actor 初始化。使用前进、后退、正向转向、正向侧移共 26 秒作为候选状态库，第二条录制的 8 秒独立留出。该分组单独记录在训练输出 `manifest.json`，不改写前面直接 PD 对照的 `rl_selection.json`。

本轮采用同类动作的完整状态帧相似度作为软奖励，不追赶固定录制相位，不写回机身或专家关节状态；actor 仍为 57D、动作 16D。世界 XYZ 仍无有效标签，也不把未经标定的 STEER/MOTION_INFO 当作训练指令真值。它没有实现完整时序运动先验或 AMP。

接入用法和实际短测记录位于训练仓库 `docs/finals/s10_flat_reference_smoke_20260911.md`，结果位于该仓库 `artifacts/s10_flat_reference_20260911/`。原数据、人工确认状态、官方模型与原 PD 指标保持原样；本轮只做少量 PPO 更新和固定指令短测，尚未形成可部署的基础步态升级。

最终 `smoke_all_groups` 已实际完成 8 个环境、4 次 PPO 更新。训练前后相同初态、四类仿真指令各 4 秒均无提前终止；转向目标轴 RMSE 从 0.36838 增至 0.39311 rad/s，因此只认定数据与奖励接入通过，不认定性能改善。该短测也没有证明原实录逐帧动力学已复现。
