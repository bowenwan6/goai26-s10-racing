# S10 楼梯数据匹配：使用方法与经验

更新：2026-09-10。适用于 `D:/S10Data/050/2026-09-09/` 本批楼梯记录。

## 1. 这版能否作为训练参考

**可以启动软参考跟踪实验，但不能仅凭回放判断训练一定能成功。** 当前数据较适合提供大致行进路线、上/下楼方向、身体朝向、腿部动作节奏和轮速。轮子尚未完全贴合，机身 XYZ 也来自重建，因此不宜硬性约束每一帧轮端位置、接触时刻或绝对高度。

建议采用“模仿参考运动 + 仿真任务奖励”：允许策略为站稳、触地和通过楼梯偏离参考。物理接触与力矩限制由仿真执行。参考运动引导控制策略的思路有 [DeepMimic 原论文](https://arxiv.org/abs/1804.02717) 和 [四足多行为运动模仿研究](https://arxiv.org/abs/2303.15331) 支持；这些方法背景不构成本批数据的训练效果证明。

| 字段或目标 | 本批建议 | 原因 |
|---|---|---|
| 12 个腿关节角 | 中等权重，允许偏离 | 真实反馈经 SDK 映射，仍有机器人模型差异 |
| 四个轮关节速度 | 中低权重 | 实测轮速有意义，但打滑/离地不代表真实前进速度 |
| 机身 roll/pitch、朝向 | 中低权重 | 基于 IMU，并受点云方向修正和安装关系影响 |
| 路线 XY、前进方向 | 低权重、宽容差 | 点云能恢复整体位移，但存在局部噪声与漂移 |
| 机身绝对 Z | 很低权重，几何不一致处归零 | 高度受地形、根位姿与接触误差共同影响 |
| 根速度/加速度 | 暂不直接作为高权重目标 | 由约 10 Hz 的估计位置插值/差分会放大噪声 |
| 轮端位置、逐轮触地标签 | 不作为精确监督 | 目前并未测得可靠接触状态 |
| 轮子累计转角 | 不做位置模仿目标 | 保留原值供复核，但对称轮子的累计自转相位无须逐帧追齐 |
| 原厂动作目标 action | 不可使用 | 本批没有 JOINTS_CMD；实际关节反馈不是期望动作 |

先确认学生能在物理仿真里稳定跟踪一段上楼和一段下楼，再扩大数据量。以完成率、摔倒、轮子打滑、力矩和关节跟踪误差评价，不以动画是否完全重合评价。

## 2. 已处理范围

第一梯段的旧结果保留在 [第一梯段报告](../artifacts/s10-recording-review-20260909/stairs_reconstruction/REPORT.md)：`145331` 的约 0–29 秒，11 级候选，含接触约束的机身平移修正。

本轮新增结果在 [批次报告](../artifacts/s10-recording-review-20260909/stairs_batch/REPORT.md)，集中回放入口为 [本地视频索引](../artifacts/s10-recording-review-20260909/stairs_batch/index.html)。

| 参考包 | 源时间 | 覆盖内容 |
|---|---|---|
| `145331_flight2` | 29–59 s | 前一平台行走/接近、第二梯段、上方平台 |
| `145331_flight3` | 59–93.2 s | 平台转向/接近、第三梯段、梯段内停顿、上方平台 |
| `152557_down` | 0–40.396 s | 接近、下楼、退出控制前的中断上下文；末端不作为正向模仿样本 |
| `154654_up` | 0–52.89 s | 接近、上楼与停顿、后续平台/调整候选 |
| `154856_up_down` | 0–84.12 s | 上楼、平台转向、沿前一梯段下行、连接区、下一下行梯段、退出段 |

总计新增 5 个参考包，包含 7 个上/下楼候选阶段。精确时长、样本数与掩码通过率见批次报告及 `phase_ledger.csv`。自动阶段与楼梯级数是候选，不把整段 success 标签自动传播到所有子区间。

排除范围：

- `145241`：活动区间 1.5–7 s 已被人工标为“废数据”，其余主要为诊断上下文，本轮不恢复为专家样本。
- `152557`：40.396 s 之后退出控制，不作为正常下楼示教。
- `154654`：52.89 s 之后包含控制退出、恢复和可能人工处理；恢复后的短片段没有另行确定的楼梯通行。

## 3. 文件怎么用

每个新增参考包位于 `artifacts/s10-recording-review-20260909/stairs_batch/<name>/`：

| 文件 | 用途 |
|---|---|
| `matched_replay.mp4` | 20 fps 复核视频；检查路线、姿态、阶段和异常 |
| `matched_replay.usdc` | 自包含的机器人 + 地形动画；在 Isaac Sim 中打开并查看时间轴 |
| `training_reference.npz` | 使用者优先读取，含逐通道建议权重和有效掩码 |
| `reference_motion.npz` | 保留原关节样本时刻的基础参考，无接触强制修正 |
| `trajectory.npz` | 约 10 Hz 点云轨迹和每帧变换 |
| `registration_quality.json` | 每帧配准残差、匹配比例等原始指标 |
| `fitted_flights.json` | 各梯段朝向、级数候选、高/深/曲率与几何误差 |
| `terrain_fit.usda`、`terrain_boxes.json` | 简化楼梯几何；无物理接触验收，不能当作已验证训练环境 |
| `terrain_proxy.npz`、`terrain_proxy.usda` | 直接从点云构成的有缺口、有噪声的地形；用于交叉复核 |
| `wheel_proxy_check.npz`、`replay_check.json` | 轮底到近似地形高度的几何差值；不是实测触地标签 |
| `summary.json` | 区间、原始录制、来源、基础质量指标 |

第一梯段使用 `contact_matched_motion.npz`，新增批次使用 `training_reference.npz`。不要假定两者字段完全相同：第一梯段有 `root_contact_correction_m`，本批有质量掩码与分通道建议权重。

完整 USD 已用 OpenUSD 重新打开验证，含 17 个机器人视觉网格及动画；不依赖另外复制机器人 meshes。**它是运动学复核动画，机器人不是可直接驱动的动力学 articulation，地形也未完成接触参数验收。** 本轮没有启动 Isaac Sim、训练策略或做 PPO rollout。

## 4. 训练字段与读取示例

以下示例只是读取准备好的参考数据，不启动训练：

```python
from pathlib import Path
import numpy as np

folder = Path('artifacts/s10-recording-review-20260909/stairs_batch/145331_flight2')
ref = np.load(folder / 'training_reference.npz')
legs = [i for i in range(16) if i % 4 != 3]
wheels = [3, 7, 11, 15]

t = ref['time_s']
leg_q = ref['joint_position_rad'][:, legs]
wheel_dq = ref['joint_velocity_rad_s'][:, wheels]
root_xyz = ref['root_position_m']
root_quat_wxyz = ref['root_quaternion_wxyz']
valid = ref['reference_valid']
xy_weight = ref['root_xy_weight']
z_weight = ref['root_z_weight']
assert not ref['contact_label_valid'].any()
```

各字段语义：

- `time_s` 是相对该录制 manifest 开始时刻的**源时间**。如第二梯段从 29 s 开始，不能误认为数组第一帧对应录制 0 s。
- `joint_source_ns` 保留关节纳秒源戳；`joint_position_rad` 按 SDK 的 `JOINT_DIR` 与 `POS_OFFSET_DEG` 转换；速度只乘方向。不代表已经转换成某个训练项目的 policy joint order。
- `root_position_m` 是各参考包第一帧机身原点下的局部 XYZ，方向采用原录制初始航向；高度原点不是地面。四元数顺序为 wxyz。
- 根位置来自约 10 Hz 点云，再轻微平滑并插到真实关节源时刻；它没有变成 200 Hz 实测位置。原关节位置和速度没有平滑或重新采样。
- `reference_valid` 是配准质量、倾角和上下文规则通过的候选掩码，不是人工确认成功、无打滑或动态可跟踪的证明。
- `phase_candidate` 保留接近、上/下楼、平台和中断上下文。时刻来自原复核候选，不能当作准确触地时刻。
- `contact_label_valid` 始终为 false。

建议权重是初始试验启发式，尚未经训练调参：腿关节 0.5、机身方向 0.25、路线 XY 0.25、机身 Z 0.05；无效区间归零。Z 还要求轮端与近似地形没有明显冲突，否则归零。这些数字不是可直接跨奖励项比较的物理量，接入时还要按各误差单位与尺度归一化。

不要把 `reference_valid=false` 的间隔删掉后把前后帧拼接；应按连续有效区间采样。每个参考包有独立位置原点，不能直接把第二、第三梯段的 XYZ 数组接到第一梯段末尾。若要训练整条多梯段通行，需要另做跨包刚体坐标连接；当前按片段训练即可。

训练/验证按完整录制或现场障碍划分。同一录制的第二、第三梯段高度相关，不宜一段用于训练、另一段当作独立泛化测试；同一障碍的上行/下行也可能有泄漏。

## 5. 如何复现

代码目录：`artifacts/s10-recording-review-20260909/`。

原始数据必须保留 `gait_.../manifest.json`、`bag/bag_0.db3` 结构。实际原始位置从解码缓存中的 `raw_path` 读取；换机器或改路径应修改 `review.py` 中 RAW，并重新生成本机缓存，不要沿用另一台电脑缓存里的路径。消息定义使用仓库 `tools/s10_gait_capture/vendor_ws/src/drdds/msg/`。

本机已有环境：

- `python -s`：Anaconda 的 NumPy/SciPy/Matplotlib；`-s` 避免用户目录另一个 NumPy 版本覆盖。
- `.venv-win/Scripts/python.exe`：MuJoCo 回放。
- `tmp/s10-analysis-deps`：已有 rosbags，以及本次沿用的 usd-core 26.8。
- ffmpeg：视频编码。

新机器可在一个独立 Python 3.12 环境中安装 NumPy `<2`、SciPy、Matplotlib、rosbags、MuJoCo、usd-core，统一用这个解释器执行下面两类命令，不必复刻本机两个环境。机器人 SDK 和 meshes 按项目主 README 准备；无需 ROS 在线环境或机器人网络连接。

首次没有 decoded 缓存时先运行既有 `review.py`。它会解码 RAW 目录中的记录，保留接收和源时间；已存在缓存则复用。不要为了重跑批次覆盖人工 `human_reviews.jsonl`。

```powershell
# 首次才需要；RAW 指向你的原始数据目录。
python -s artifacts/s10-recording-review-20260909/review.py

# 全批次：配置已列出 5 个参考包及排除原因。
python -s artifacts/s10-recording-review-20260909/batch_match_stairs.py
python -s artifacts/s10-recording-review-20260909/build_batch_terrain.py
python -s artifacts/s10-recording-review-20260909/fit_batch_flights.py
.venv-win/Scripts/python.exe artifacts/s10-recording-review-20260909/render_batch_stairs.py
python -s artifacts/s10-recording-review-20260909/summarize_stair_batch.py

# 或每个步骤都加 --job，只重做一个包。
python -s artifacts/s10-recording-review-20260909/batch_match_stairs.py --job 145331_flight2
```

批次配置在 `stair_batch.json`：录制 ID、源时间起止、候选阶段、排除原因。缓存绑定录制 ID 和起止范围，发现不匹配会报错；若调整某个包的时段，使用新的包名/输出目录，不要把别的片段的缓存拿来继续。

最小检查：

```powershell
python -s artifacts/s10-recording-review-20260909/batch_match_stairs.py --selfcheck
python -s artifacts/s10-recording-review-20260909/build_batch_terrain.py --selfcheck
python -s artifacts/s10-recording-review-20260909/fit_batch_flights.py --selfcheck
```

实际导出阶段还检查时间单调、关节/位置有限值、USD 可重新打开、动画存在。视频是 20 Hz 显示，训练 NPZ 保留原关节采样时刻。

## 6. 这次的方法与经验

### 高、深记录有用，但不足以单独恢复全程位置

尺寸约束缩小了几何搜索范围。还必须确定边缘朝向、曲率、级数、平台连接，以及机器人在每一时刻的位置。重复台阶下，错一整级仍可能有很小的单帧拟合残差。

第一次使用单帧直线楼梯拟合出现错级；换成原始约 10 Hz 前后雷达、IMU 姿态辅助和多帧环境配准后，才能区分真实后退、前进和转向。轮速不能独自替代这个位置估计，因为轮子可能打滑或离地。

### 弧形要允许，但不能把整个现场强行解释成同一个半圆

第一段静止点云支持轻微曲率，直线模型边缘误差更大。新增梯段使用局部曲线边缘，保留各段朝向和平台转向。曲率、级数和尺寸均保留为拟合候选；触及参数搜索边界或残差偏大的模型需降置信度，而不是称为现场测量。

`154856` 的第一次下行与前面的上行在重建位置上重合，回放共用该梯段几何，避免上、下楼各选一套互不一致的楼梯。后续下行转向到另一处，则保持为另一局部梯段。没有把不同录制中的相似楼梯自动宣称为同一个物理障碍。

### 不穿插不等于贴合，也不等于物理可执行

第一梯段曾通过接触约束平移机身，消除了穿插，但局部最大修正约 13 cm，用户仍能看到轮子未充分贴合。这说明“无穿插”只是几何条件，不能代替接触时机、支撑关系、动力学跟踪和真实位姿。

新增批次保留点云估计与原关节运动，不为了画面整齐逐帧把轮子吸到地面；对不一致区间降低/关闭高度参考权重。未来若做精确重定向，应把位置修正量、平滑、接触约束和点云残差一起报告，而不是只展示最终零穿插。

### 点云网格与训练地形应区分

直接栅格化点云会出现孔洞、噪声和锯齿。网格保留观测/插值掩码，缺测最多只向 17 cm 内近邻补洞，不跨大空白造地面。它用于证据和交叉复核。

最终动画采用简洁的局部台阶几何；保留原网格以检查拟合是否过度。当前台阶采用条带盒近似弧边，宽度和延伸范围是建模选择，未测得完整场地边界。未验证的 USD 视觉地形不直接宣称是可靠的物理训练关卡。

### 三种误差不要混成一个“精度”

1. 配准残差：当前点云贴合参考地图的程度，主要用于轨迹质量筛选。
2. 楼梯几何残差：留出点云贴合简化楼梯模型的程度，包含模型简化与地形噪声。
3. 轮端几何差值：机器人模型轮底相对近似地面的差值；边缘侧接触和未知区域会影响它。

本批楼梯拟合中位误差约 3–6 cm，P90 约 10–17 cm；比轨迹配准残差大。不能用约 1 cm 的配准残差宣称轮端也有 1 cm 精度。验证使用交替扫描，但共享配准地图，仍不是独立定位真值。

### 保留原始数据、时间和失败上下文

源时间与接收时间不要混用；关节观测不要冒充期望动作；步态切换时刻不要冒充触地时刻。记录中控制退出、倾斜异常、人工废弃判断均应保留，并在训练采样时用掩码排除。失败记录可以用于失败识别或恢复研究，但不能未经复核就作为成功动作奖励。

## 7. 交付与下一步

Git 保存脚本、配置、说明、指标 JSON、阶段 CSV 与小图。原始录制、NPZ、视频、USD 二进制、生成 XML 等通过既有队内数据存储交付或本地重新生成，不要把整个 artifacts 目录强推 Git。

本轮没有开始训练。接下来优先选一上、一下两个连续高质量区间，建立物理跟踪基线，观察能否在允许参考误差的情况下稳定通过；再逐步增加其余片段。对高度权重为零的区间让仿真任务和实际接触决定动作，不用人工猜出的高度或触地标签补齐。
