# S10 高台实录匹配：使用方法与判读约定

更新：2026-09-10。输入为 `D:/S10Data/050/2026-09-09` 的高台实录。当前流程完成了点云台沿估计、真实关节参考导出、MuJoCo 自由动力学匹配及 Isaac 用地形导出；尚未在 Isaac Sim 中执行回放。

当前先看 [短高台的参考裁剪与证据](../artifacts/s10-ledge-batch-20260910/cropped_ascent/REPORT.md)：用户补充实物台深只够容纳机器狗，前方是草地，已按实录首次回平窗口重新导出 10 段参考。每段保留 0.2–0.5 s 的首次近回平窗口，裁去后续动作；裁剪依据是实录关节/IMU，原始数据不变。

[此前上台匹配](../artifacts/s10-ledge-batch-20260910/ascent/REPORT.md)的 10/10 是深 3 m 测试场景的结果，不能作为真实短台或新参考的通过率；新裁剪尚未重跑动力学。实际台深尚无可靠数值，不能直接使用旧 USDA 当作真实场景。[更早完整循环结果](../artifacts/s10-ledge-batch-20260910/REPORT.md)中的 2/12 还包含下台、倒退和旧判据，同样不能代表实录上台成功率。本流程不会把仿真通过直接写成人工确认或训练合格。

上台专项已修正判据：四轮分别建立过台面接触，仍在台面附近，至少三轮当前有台面支撑，倾角小于 25°并持续约 0.2 秒。轮心离台缘的余量另行报告；不再要求余量必须大于 8.1 cm 才承认上台。另有小范围增益对照，实际采用值保存在每段 `selected.json` 的增益字段及 `selected_alignment.json` 的 `controller` 字段中。原 `reference.npz` 不变，基座保持自由动力学。

## 1. 文件入口和运行

从仓库根目录 `D:/Desktop/Code/goai26-s10-racing` 执行：

```powershell
# 当前推荐：按实录裁剪，提取边界点云并生成可视化，输出 cropped_ascent。
python -s -B artifacts/s10-ledge-batch-20260910/crop_ascent.py
.venv-win/Scripts/python.exe -B artifacts/s10-ledge-batch-20260910/crop_ascent.py --clouds
python -s -B artifacts/s10-ledge-batch-20260910/crop_ascent.py --plot

# 历史复现：3 m 深测试场景的上台匹配，不是实物短台验收。
.venv-win/Scripts/python.exe -B artifacts/s10-ledge-batch-20260910/ascent.py
python -s -B artifacts/s10-ledge-batch-20260910/ascent_report.py

# 只重做片段裁剪和原始点云几何估计，不跑动力学。
.venv-win/Scripts/python.exe -B artifacts/s10-ledge-batch-20260910/batch_match.py --prepare-only

# 一条录制或一个连续片段；零时移失败后可扩大时间搜索。
.venv-win/Scripts/python.exe -B artifacts/s10-ledge-batch-20260910/batch_match.py --only 150705 --wide
.venv-win/Scripts/python.exe -B artifacts/s10-ledge-batch-20260910/batch_match.py --only 150146_D1 --wide

# 复现本次全批次搜索。
.venv-win/Scripts/python.exe -B artifacts/s10-ledge-batch-20260910/batch_match.py --wide

# 导出实录 FK、仿真关键帧和 USDA；生成对照图和报告。
.venv-win/Scripts/python.exe -B artifacts/s10-ledge-batch-20260910/render_results.py
python -s -B artifacts/s10-ledge-batch-20260910/plots.py
```

本机 `.venv-win` 已有 NumPy、MuJoCo、rosbags；第二个 `python` 是带 matplotlib/Pillow 的 Anaconda。没有新增依赖。`batch_match.py` 开始时执行一个轻量自检，覆盖 PointCloud2 的行填充/大小端，以及上台、前进下台、上台后倒退的判定分支。

依赖的已有输入为：

- `artifacts/s10-recording-review-20260909/decoded/`：各录制的 `.meta.json`、200 Hz 传感器 `.npz`、点云预览 `.clouds.json`；由现有 `review.py` 生成。
- `artifacts/s10-expert-analysis/simulation_review/free_base.xml`：经过关节映射检查的 S10 自由基座模型。
- `upstream/goai_embodied_future_material/src/S10_sdk_deploy/`：SDK 中的关节方向、零位和模型网格。
- 原始录制目录下的 `manifest.json` 和 `bag/*.db3`：以 SQLite `mode=ro` 打开。

模型 XML 中的网格路径仍是本机绝对路径，换电脑需重新按本机 SDK 生成基础模型或更新网格路径。原始数据和生成缓存需另行提供；只复制方法文档不能独立回放。

## 2. 先划分动作，再放高台

最新上台裁剪定义在 [cropped_ascent/clips.json](../artifacts/s10-ledge-batch-20260910/cropped_ascent/clips.json)，历史完整阶段定义在 [clips.json](../artifacts/s10-ledge-batch-20260910/clips.json)。`start/end/peak/reverse_start/reverse_peak` 全部使用原始记录时间，定义为 `(传感器 source_stamp_ns − manifest.started_wall_ns) / 1e9`。不要把接收时间、播放器时间和裁剪后从零计时混用。

裁剪规则寻找首次持续至少 0.2 s 的近回平区间（俯仰/侧倾各 <10°、四轮 FK 竖直高度差 <4.5 cm），最多保留该区间前 0.5 s。它是参考边界候选，不是实测接触判据；不能因为裁后姿态较平就补上人工成功标签。末帧轮速未必为零，参考结束时应结束 episode 或切到已有站立控制，不能持续循环末帧。原参考逐样本保留，没有为了站稳而修改末帧速度。

| mode | 初始支持面 | 台体相对前方台沿的位置 | 完成目标 |
|---|---|---|---|
| `up` | 低地 | 台沿后方，即机器人前进方向一侧 | 四轮上到台面，末段仍保持 |
| `down` | 台面 | 台沿前方的反方向，即机器人后侧 | 四轮越沿落到前方低地 |
| `cycle` | 低地 | 与 `up` 相同 | 连续完成上台，再倒退回原低地 |

这里“前方台沿”指片段起点机器人 +X 方向的待通过边界。连续上台/倒退过程不在中间重置机器人，也不把台上等待删除。前进下台可以单独裁剪，但初态应放在台面；不能把上台的初始基座高度原样套用。

动作判断同时看四类证据：

1. SDK 映射后的关节轨迹及四轮相对机身的 FK 高度，定位抬腿、抬头和回平。
2. IMU 俯仰/侧倾及控制状态，判断是否仍处于原受控过程。当前姿态约定中负俯仰为抬头。
3. 前后雷达中台面、低地、立面如何出现/消失，判断越过哪个边界。
4. 映射后轮速的方向。当前 SDK 模型轮轴为 `0 -1 0`，负轮速对应向 +X 滚动，正轮速对应倒退；这只适用于当前映射。

关节 + IMU 能恢复相对机身的姿态，不能单独确定轮子在世界坐标中的接触位置。整段录制的 `terrain=ledge`、`outcome=success` 也不能替代逐动作标注。

## 3. 台沿距离怎么估

匹配使用原始约 10 Hz 点云，预览图使用现有约 2 Hz 下采样点云。点云消息声明为 `base_link`，当前不额外重复施加雷达安装变换；这一声明和 SDK 机身原点在物理上是否一致仍待标定核验。

用同时间 IMU 去掉滚转和俯仰的影响，再在机器人前方局部区域拟合台面前缘和支持地面。初始距离与沿方向取起点附近多帧估计的中位数。输出 `geometry.json` 保存各帧时间、法向、残差、可见高度及支持平面。多帧距离范围还包含机器人在这段时间内的运动，不能直接当成静态测距误差。

本次 30 cm 高台前缘点云并非严格竖直，可能包含真实外形和传感器误差，批处理允许更大的前缘倾斜量；地面细化后仍必须保持接近水平。当前生成的碰撞体依旧是方块，外形与方块的接触差异可能被拟合出的距离/时序修正部分吸收。

当真实台沿被遮挡后，后方物体不能接替成“同一台沿”。当前局部连续跟踪在估计间隔超过 0.35 秒或相邻距离跳变超过 0.18 m 时停止；适用范围是本批次近距离单台沿。几何估计为 `unresolved` 时应先查点云并选择更早、台沿可见的起点，再跑动力学。

前进下台常看不到立面，改为使用后方高支持面和前方低支持面，在横向条带内给出“最远高地可见点—最近低地可见点”的区间。`edge_bracket_width_m` 和 `bounds` 必须一并保留；区间中点只是本轮搜索的名义起点，不是独立精确测得的台沿。

## 4. 专家参考和动力学

本批次没有 `JOINTS_CMD`、`SLAM_ODOM`、`TF`。导出的 `reference.npz` 表示**测得的关节状态**，不是原控制器的动作或已恢复的世界位移。

- 关节位置按 SDK 的 `q_model = q_raw × JOINT_DIR + POS_OFFSET` 转换，速度只乘方向；轮关节去掉片段起始累计相位。
- 关节和 IMU 参考插值到 200 Hz；四元数先处理符号连续性，再插值归一化并去掉初始 yaw。离散控制状态和步态使用前值保持，不能线性插值成虚假中间状态。
- 初始基座高度由 SDK FK 将最低轮心放在支持面上方 `0.081 + 0.002 m`。前进下台的支持面再加记录台高。
- 初始线速度沿用 `MOTION_INFO` 的报告值；障碍控制中该字段可能为零，并不证明机器人真的静止。未将轮速积分伪装为实录世界轨迹。
- 自由动力学步长 1 ms；腿部 PD 为 `kp=80, kd=2`，轮部速度跟踪 `kd=0.6`，按模型力矩范围限幅；基座不锁定。仿真记录以 50 Hz 导出。
- 台高取 `manifest.parameters.height_cm`；旧仿真的台深 3 m、台宽 2 m 是历史测试假设，并非实测外形。用户已明确实际深度只够机器狗，后接草地；新裁剪配置中深度留空，需量取/可靠估计近远两侧边界后重建有限深度场景。

每段先跑零距离/零时序修正基线，再只改变台沿距离和参考动作延迟。`--wide` 对零时移未完成的普通片段尝试全部 13 个距离 × 7 个时移；先有零时移完整成功的片段无需扩大时序。150921 已知中断案例保留其真实结果属性，不用成功优先来选择。

正距离修正把前方台沿移远，负值移近；正动作延迟表示关节参考更晚开始。延迟改变动作的执行时刻，但比较姿态时仍使用同一个原始 source 时间，从而暴露而不是掩盖时间误差。不要将“仿真更容易成功”解释成雷达外参已经被纠正。

## 5. 看结果和交接 Isaac Sim

每个片段目录的文件用途：

| 文件 | 用途 |
|---|---|
| `configuration.json` | 源录制、原始时间段、模式及判断证据 |
| `geometry.json` | 点云名义台沿及各帧证据 |
| `reference.npz` | 原始时间、映射后的关节位置/速度、IMU、运动状态 |
| `baseline.json/.npz` | 零修正的结果和轨迹 |
| `search.json` | 全部搜索结果，包括失败和初始碰撞非法的方案 |
| `selected.json/.npz` | 选中参数下的阶段结果、姿态、轮心和接触时间 |
| `selected_alignment.json` | 初态、距离修正、时序修正、沿方向和关节名称 |
| `selected_scene.xml` | 本机 MuJoCo 模型和匹配初态 keyframe |
| `selected_terrain.usda` | 米制、Z-up 的地面与高台，带碰撞；不含机器人和动作 |
| `recorded_kinematics.npz` | 实录关节 + IMU 算出的机身相对骨架 |
| `comparison.png`、`diagnostics.png` | 实录/仿真关键帧与连续曲线 |

后续在 Isaac 中，应先按实物深度修正高台，旧 `selected_terrain.usda` 是 3 m 深场景，只能用于复现旧实验。随后使用 S10 模型，按 `selected_alignment.json` 核对机器人初态，**按关节名称匹配**参考列，按 source 时钟执行，并核对驱动、力矩和碰撞参数。新参考边界不能自动继承旧匹配通过状态；采用旧延迟时还需处理参考区间边界，不能继续读取已裁掉的后续动作。对照相同 source 时间的基座姿态、接触顺序和四轮高度。旧 USDA 高台位置已包含选中距离修正，不要重复叠加 offset。

MuJoCo XML 中 `matched_start` keyframe 只描述可用初态；加载模型后需显式使用该 keyframe，不能假定加载 XML 就自动从该姿态开始。批处理回放通过数组直接赋值初态。

尚未编写或验证本机 Isaac 控制回放脚本；不能把导出 USDA 称为 Isaac 通过。仿真几何通过、人工确认动作结果、专家样本训练资格是三项独立状态。

## 6. 本次经验应怎么沿用

上一次 151135 的零修正回放会侧翻，只把台沿移近 6 cm 后能完成上台；与此同时接近距离与原点云的 RMSE 变大。这说明距离影响接触时机，也说明成功参数可能在补偿外参或接触模型误差。

本次应沿用的是“逐段估沿—检查方向—比较零修正—搜索距离/时序—分别验收上/下台”的方法，而不是复用一个固定 6 cm 常数。上台通过但倒退失败时，保留它作为局部匹配结果，查看倒退开始前的仿真位置和姿态是否已偏离实录；不要在中间重置基座后宣称整个连续动作成功。

本次短台信息还修正了一个关键假设：近侧距离正确，不代表平台另一侧边界正确。台上后续低头、轮高分离可能包含离台和草地接触，不能继续统一解释成上台控制失败；先按实录裁剪，再用实际有限深度验证。150705 约 12.95/30.15 s 的旧“回摆”描述不再作为已确定动作标签，需结合远侧边界复核。

传给训练流程前，先人工确认每个候选动作的实录结果，再决定是否采用反馈状态作为模仿参考。当前批处理统一保留 `training_ready=false`，没有更改现有人工复核文件。
