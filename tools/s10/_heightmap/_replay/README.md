# S10 离线高度图回放

从 SQLite 原始双 AIRY PointCloud2、IMU、实录关节状态按历史到达顺序回放；提供单帧表面与短时 ICP+IMU 局部图的对照。运行不连接设备，不启动 SDK、训练或动力学。所有输出在本目录 `output/`。

## 从干净进程复现

仓库根目录 PowerShell，使用本机 `D:/Anaconda/python.exe`（`python -s`），已有 NumPy、SciPy、matplotlib、ffmpeg。脚本按绝对 `--repo` 追加已有 `tmp/s10-analysis-deps` 中的 rosbags，不安装包、不运行旧分析入口。

```powershell
Set-Location D:\Desktop\Code\goai26-s10-racing
python -s -B tools/s10/_heightmap/_replay/selfcheck.py

# A：先看独立单帧；从起点读取，不从 9 s 预填地图。
python -s -B tools/s10/_heightmap/_replay/replay.py --recording 151135 --end 17 --stage-a

# B/C：首段 0–17 s 连续运行，视频重点 9–15.5 s。
python -s -B tools/s10/_heightmap/_replay/replay.py --recording 151135 --end 17 --video-windows 9:15.5
python -s -B tools/s10/_heightmap/_replay/analyze_results.py --recording 151135

# 首段后验证第二段：0–49 s 连续运行，含较早动作和两段重点窗口。
python -s -B tools/s10/_heightmap/_replay/replay.py --recording 150146 --end 49 --video-windows 25:30.5,42.5:47.5
python -s -B tools/s10/_heightmap/_replay/analyze_results.py --recording 150146

# 仅重新出图/视频，不重做配准。
python -s -B tools/s10/_heightmap/_replay/replay.py --recording 151135 --render-only --video-windows 9:15.5
```

输入默认 `D:/S10Data/050/2026-09-09/`。`--raw` 可改录制父目录，`--repo D:/Desktop/Code/goai26-s10-racing` 指向只读资源所在原仓库；独立 worktree 不必复制原始数据、缓存或模型。`--recording` 必须唯一匹配 ID。

解码缓存按录制 ID 与 `--end` 检查，范围改变可用 `--label _another_run` 建立新的本工具输出目录。已有本工具同名结果会重导；旧分析目录不会被写入。默认 `--end` 是**接收时钟**的秒数，视频窗口也按接收时间展示，同时打印当时最新点云的源时间，并保留末端 0.25 s 接收延迟余量。

## 输出与查看

每段位于 `output/gait_<完整录制 ID>/`。

| 文件 | 内容 |
|---|---|
| `replay_9_15.5.mp4` / 第二段两个 MP4 | 同步原始点云、实录 FK/姿态、前后单帧 H、融合、滚动 H/V/state/age；10 fps，编码后 ffprobe 检查 |
| `key_*.png` | 视频关键帧，含失败区间，灰色是未知 |
| `A_*.png`, `stage_A.npz/json` | 独立单帧前/后/融合检查；与滚动门槛无关 |
| `observations.npz` | 50 Hz 的网格与输入/测量时间、位姿质量；不是 50 Hz 新雷达测量 |
| `metrics.csv` | 每次查询的分区域覆盖、年龄 P95、更新间隔、配准质量 |
| `frames.csv` | 每个原始点云的整数源/接收/采用 IMU/关节时间、配准接受/失败原因、局部段、表面/冲突计数 |
| `single_frames.npz` | 每个可处理原始扫描独立 H/V/age、5 cm 表面、原始点显示样本及姿态；配准失败仍保留 |
| `audit.json`, `decoded.npz` | 原始字段/频率/接收差审计、原始本地回放输入缓存 |
| `run.json` | 实际配置、耗时、帧数和失败分类 |
| `summary.json`, `phase_metrics.csv` | 阶段候选、首次可见候选、有效比例、盲区/年龄统计 |
| `timeline.png`, `consistency.png` | 连续覆盖/位姿、关节与俯仰、台面斜率及台沿内部一致性 |
| `surface_diagnostics.json`, `kinematic_diagnostics.npz` | 事后诊断中间值；从不作为回放输入 |

## 几何与时间约定

- +X 前、+Y 左、+Z 上，米；采用消息中已在 `base_link` 的点。已检查本地采集驱动外参配置与 merger 只拼接的源码，不重复安装变换。配置快照和 frame_id 不能证明现场外参/机身原点标定准确。
- 世界局部轴去掉首次已到达 IMU 的 yaw；IMU 去 roll/pitch，使重力方向固定。查询网格只随当前 yaw 转动。H=`terrain_world_z-base_world_z`，裁剪 `[-1,1]`。
- x 为 `linspace(-.6,1.2,13)`，y 为 `linspace(-.6,.6,9)`，`index=xi*9+yi`，NPZ 的 H/V/age 为 `(N,117)`，可 reshape 为 `(N,13,9)`。
- V=0 时 H=0 为占位，age=正无穷；没有“零高度地面”的含义。有效 age=`(query_receive_ns-oldest_support_source_ns)/1e9`。非空支持不超过 TTL；高度/可信度矛盾、姿态无效也会提前失效。
- `state` 为 0 未知、1 支持来自当前最新扫描、2 需要历史测量支持。即使输出多次相同 H，也不改变 `measurement_ns`。`metrics` 的 `*_memory` 另表示 age>0.30 s 的有效格比例；两者一个是扫描身份，一个是年龄门槛。
- `metrics.csv` 分开记录 `support_changed_cells`（移动查询导致支持改变也算）与 `new_measurement_cells`（该次查询确实接纳新扫描的高度）；`new_latest_lidar_frames` 表示最新扫描身份改变。50 Hz 重复查询不会被计为 50 Hz 雷达测量。
- epoch 时间用 int64 保留。显示秒数统一减 `manifest.started_wall_ns` 后再除 1e9，严禁先转浮点 epoch。`time_ns` 是 50 Hz 查询的接收时钟，`latest_cloud_source_ns`、各输入来源和高度支持时间另存。
- SQLite `messages.id` 保留写入/到达顺序，不按源时间重排。多个分卷按文件序；若接收钟倒退，调度用累积最大值保序，原 rx 不改，audit 记录逆序计数。
- 每帧仅查询**已经到达**且 source<=该帧 source 的最后一条 IMU/关节消息，不插值、不偷看未来、不校正整段时间偏移。过旧/缺失 IMU 的扫描无效；关节缺失用保守局部腿部包络过滤。
- 本地配置是 `ts_first_point: true`，扫描周期约 100 ms，字段中没有逐点时间。这里每帧按 header 起始姿态刚性变换，**未完成扫描去畸变**。receive-source 包含采集/打包/传输及可能的时钟偏差，不是纯网络延迟。

## 地图和质量门槛

主要阈值集中在 `replay.py:CFG`，每次运行存入 `run.json` 和 NPZ 的 `config_json`。它们是这一轮验证的可调工程门槛，不是传感器精度标定。

| 参数 | 当前值和依据 |
|---|---|
| 内部格/策略查询邻域 | 5 cm；节点在 ±6.5 cm 方形支持邻域内选实测表面，相当于 13 cm footprint。没有无证据的大面积插值；15 cm 策略节点仍会漏掉细边缘 |
| 每格证据 | 至少 3 点；P90–P10<=4.5 cm；层间跳变<=7 cm；邻域至少 6 点与二维展宽、法向 z>=.88、局部平面 RMS<=2.2 cm |
| 台阶/立面 | 相互矛盾或多层格标未知，选实测中位附近高度，不把上下层取平均；滚动单元遇到高度差>8 cm 先失效 |
| 自体 | SDK XML 关节链和原映射做 FK；机身盒约 .62×.228×.18 m、腿线段半径 3.7 cm、轮半径 8.1 cm/半宽 2.5 cm。保留轮底切面附近地面，无 65 cm 径向裁剪 |
| 离群/范围 | 四近邻中第 4 点距离<20 cm；原始本地缓存 .08–8 m，地形估计半径 3 m、高度 -1.5–.6 m，较远结构仍可帮助 ICP |
| 时间 | IMU 最大源滞后 40 ms、关节 60 ms；最新帧窗口 300 ms 覆盖实测约 123 ms 打包接收差与 100 ms 周期；地图 TTL 1.5 s |
| 短时 ICP | 复用旧代码纯 `target` 法向函数（AST 提取，不导入模块）；8 个已到达扫描参考，12 cm source voxel，固定 IMU 旋转，只求平移，最多 12 次迭代 |
| 接受门槛 | 至少 100 个平面对应；几何近邻重叠>=35%；平面残差 P90<=6.5 cm；平移法方程条件数<=80；相邻运动<=2 m/s×dt+3.5 cm |
| 失败与查询 | 失败立即切断旧图、建立新 segment；同周期两传感器 source 差<25 ms 可初始化参考，后续扫描通过前滚动 V=0；查询外推最多 250 ms，无加速度/轮速积分 |
| 信任衰减 | confidence>=.3；pose_sigma 是按残差/外推距离构造的启发式质量预算，非统计 1σ；累计质量预算与每秒 2 cm 老化不超过 6 cm 才保留 |

局部平移只用于短时运动可行性评估，不是已经验证的真机 LIO。`segment` 改变后不能比较两段的绝对平移或继续旧图；日志中的累计数值仅保留画图连续性。没有独立地图/接触/位姿真值，不能由 ICP 残差推导绝对高度精度。

## NPZ 的使用边界

- `H/V/age` 始终是带位姿门槛的**滚动查询**。`pose_valid=False` 时滚动 V 全零，禁止将失败单帧直接当作可信世界地图。
- `front_H/V`、`rear_H/V`、`single_H/V` 是最新单帧的诊断对照；位姿可靠时变换到当前查询位置；`single_in_current_frame=False` 时只作源扫描附近的回退显示，不能当成已经移到当前机身的 actor 输入。更严格的原始单帧接口在 `single_frames.npz`。
- `pose` 为 `(x,y,z,qx,qy,qz,qw)`，`pose_age` 是距最新源帧的时间。`confidence` 是点数/离散度门槛，`icp_p90` 是匹配面的内部残差。
- `measurement_ns` 取查询支持中最老 source，`arrival_ns` 取最晚 arrival，`sensor` 为选定高度来源（0 前/1 后），`frame_id` 是所选表面扫描；state 使用最老测量的时间判断是否需要历史支持。
- 区域统计：ahead 为 x=.6–1.2（45 格），front_wheels 为 x=.15–.45（27 格），rear_wheels 为 x=-.6–-.15（36 格），body 为 |x|<=.15 且 |y|<=.3（15 格）。前/后轮区域是固定空间条带，不是实测接触位置；body 与轮区边界有重叠。
- 阶段标签、首次台面候选、台沿拟合只在 `analyze_results.py` 中做事后诊断。近台面候选阈值是 x=.25–1.2 m、H=-.2–.35 m、至少 8 个 5 cm 表面单元、横向展宽>20 cm，避免将零星点称为台面；远处可见高表面的身份另标不确定。
- 下台窗口另用相同 footprint/点数门槛、H<-.55 m 标记前方较低表面候选；`window_visibility` 只陈述该窗口中最早满足条件的到达数据，不能把录制初期另一轮动作的可见台面继承成这次的提前量。诊断 ROI 与策略格一样随机身 yaw，避免第二段转向近 180° 后把世界 -X 当作机身后方。
- 输出为离线参考事件时钟，耗时在 run.json；未证明算法能在机器人上实时完成 50 Hz。视频是 10 fps 摘取 50 Hz 查询，没有提高雷达测量频率。

实际结论、失败区间与后续工作见 [PROGRESS.md](PROGRESS.md)。
