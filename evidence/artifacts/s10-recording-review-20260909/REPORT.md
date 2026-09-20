# S10 录制整理 · 2026-09-09

17 段 · 1251.903 秒 · 27 个通行候选 + 4 个静止/中断后上下文组 · 393 个内部片段 · 56 个复核区间

入口：[全部录制与同步预览](index.html)；[集中复核问题](review_queue.html)；[完整录制清单](recordings.csv)；[通行索引](passages.csv)；[内部片段索引](segments.csv)。

## 已检查事实

- 原始目录下实际为 17 个录制目录、17 个 SQLite bag，合计 519,678 条消息、1251.902941 秒。标签：基础 7、楼梯 5、高台 5；整段 success 7、failure 4、unlabeled 6；人工 events 均为空。
- 全部包含 JOINTS_DATA、JOINTS_DATA_10HZ、IMU、MOTION_INFO、前后点云；部分含 GAIT、STEER、HANDLE_STEER。实际消息类型共六种，无相机图像/压缩图像话题；原始目录未发现视频文件。没有 SLAM_ODOM、JOINTS_CMD 或 TF。
- 所有消息已实际解码，解码异常 0；数据库各话题计数与 manifest 一致。17 份 metadata.yaml 的总消息数正确，但 files 内消息数均为实际值的两倍。清单按数据库实际计数，未改写源 metadata。
- [原有转存校验记录](D:/S10Data/050/2026-09-09/verified.json)记载 15,148,185,579 字节，远端/本机 SHA256 一致及 SQLite quick_check=ok。本轮引用该已有结果，没有重新下载、复制原始 bag 或计算全量哈希。
- height_cm/depth_cm 仅作为 manifest 原值保存；例如楼梯 15/55 cm、高台 30/32/38 cm，均未认定为测量真值。

## 时间、同步和数据问题

- 索引时间基准是 `source_stamp_ns − manifest.started_wall_ns`，原点和区间纳秒字符串保留在 JSON/CSV。区间采用 `[start,end)`，末端包含录制终点；少数首条源消息早于开始记录，负时间原样保存在话题统计中，索引从 0 开始。控制指令 GAIT 单独按接收秒标注。
- IMU 源间隔通常约 5 ms；MOTION_INFO 约 50 ms；点云与 JOINTS_DATA_10HZ 约 100 ms。主 JOINTS_DATA 有 200/50 Hz 两档，各段范围见下表及 [5 秒窗口](sampling_windows.csv)。`150921`、`152557` 在退出控制附近降到 50 Hz，`154654`、`154856` 主关节流为约 50 Hz。
- IMU 的 receive−source 中位数约 10–12 ms，主关节约 1.5–3.4 ms，MOTION_INFO 约 1.3–4.6 ms，前点云约 119–129 ms；这些包含时钟偏差、处理与缓冲，不能直接解释为纯传输延迟。未对齐/消除各话题时钟偏移。
- GAIT 的源戳为 0，不能与传感器按源时间拼接；实际反馈用 MOTION_INFO。STEER/HANDLE_STEER 源间隔呈约 80 微秒短间隔与约 1 秒跳跃混合，且相对接收时钟有数秒偏移；中位间隔的倒数不是实际发布速率。这些是事件/指令流，不能按连续传感器“掉帧”解释。
- 机身三路在各自有效覆盖内没有超过 `max(0.1s, 5×中位源间隔)` 的长源时间间隔；存在接收卡顿但源序列连续的情况，单列在 inventory.json。此结论不包含提前结束后的空白区间。
- **150921：** 约 16–19s 大倾角，16.813s 状态17→2，18.814s→0；IMU/JOINTS_DATA/MOTION_INFO 接收分别在约63.748/63.743/63.716s结束。前点云源63.723→87.923s间隔24.200s、后点云源63.723→88.623s间隔24.900s；前者旧帧直到接收85.468s才到达（约21.744s延迟）。前后流的延迟接收与源空档分别保留。
- **152557：** 约40s大roll倾斜，40.396s状态17→2，42.398s→0；随后姿态多次变化。**154654：** 52.891s→4、54.491s→0、106.005s→1、107.606s恢复17。姿态变化不自动等于人工搬动。
- 控制退出后，MOTION_INFO 部分速度字段仍保持旧常值，不能用这些值推断机身继续移动。全部原始数值仍保留。

## 候选结构与结果边界

- 三层保持为原始录制 → 候选连续通行/上下文 → 内部片段。通行候选不是已核实的通行数量：基础段中的往返和长等待是否分成独立测试仍待确认；151538全段近静止，作为上下文保留。
- 自动候选由0.5秒窗的IMU姿态、角速度和关节运动提出，结合状态退出/数据终止；步态切换只作为独立控制事件。之后模型检查全部17段信号图与每段12帧点云速览，细化有证据的活动区。此检查不等于人工逐帧确认。
- 145331的多个上楼活动区以及两处平台/重新接近候选暂保留在同一连续组；154856在转向、姿态方向改变附近提出上/下行两组，其48.5–55s平台候选保留在下行组内；150146/150705重复高台动作簇提出候选分组，允许人工合并。没有按每次步态码变化拆通行。
- `terrain`/`action_phase`/`direction` 与 control_timeline 分开；方向只填写 up_candidate/down_candidate 或 unknown。障碍、梯段、平台的正式ID均为null，没有证据确认是同一障碍的记录不做关联。
- 子片段结果为 uncertain、interrupted_control 或 interrupted_data；控制退出/缺流是可检查事实，地形通行是否失败/完成仍需人工判断。没有继承 manifest success/failure，也没有判为合格专家样本。所有 human_review_status=pending。
- 自动边界一般约±0.5–1s；精确状态转换保留原采样时刻。预览抽样上限2Hz点云、10Hz姿态/关节，不能精确判定逐轮接触。原始bag路径与时间范围保留，未切出新bag。

## 逐录制概览

表中时间为源秒，简称取自完整ID的时分秒；点击原录制ID进入同步预览。所有参数均为未验证记录值。

| 原录制 ID | 秒 | 整段标签/结果 | 主关节源Hz（5s窗） | 通行候选+上下文 / 片段 |
|---|---:|---|---|---|
| [gait_20260909_144907_fd57c06d30d3](viewer.html?id=gait_20260909_144907_fd57c06d30d3) | 120.006 | basic/unlabeled | 200–200 | 1+0 / 15 |
| [gait_20260909_145241_9cae0ba51169](viewer.html?id=gait_20260909_145241_9cae0ba51169) | 16.699 | stairs/unlabeled | 200–200 | 1+0 / 7 |
| [gait_20260909_145331_616f9d8038d9](viewer.html?id=gait_20260909_145331_616f9d8038d9) | 93.260 | stairs/success | 200–200 | 1+0 / 27 |
| [gait_20260909_145914_2e0541078764](viewer.html?id=gait_20260909_145914_2e0541078764) | 39.444 | basic/success | 200–200 | 1+0 / 13 |
| [gait_20260909_150146_9b19dc6690ec](viewer.html?id=gait_20260909_150146_9b19dc6690ec) | 114.640 | ledge/success | 200–200 | 6+0 / 57 |
| [gait_20260909_150705_4fe2b5f8eebd](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd) | 93.256 | ledge/success | 200–200 | 5+0 / 45 |
| [gait_20260909_150921_9ed8457a8250](viewer.html?id=gait_20260909_150921_9ed8457a8250) | 89.367 | ledge/success | 50–200 | 1+1 / 18 |
| [gait_20260909_151135_f45a816f4062](viewer.html?id=gait_20260909_151135_f45a816f4062) | 26.319 | ledge/success | 200–200 | 1+0 / 19 |
| [gait_20260909_151220_d3ace6be729a](viewer.html?id=gait_20260909_151220_d3ace6be729a) | 13.336 | ledge/success | 200–200 | 1+0 / 11 |
| [gait_20260909_151538_271bf7a9d4d2](viewer.html?id=gait_20260909_151538_271bf7a9d4d2) | 24.863 | basic/failure | 200–200 | 0+1 / 1 |
| [gait_20260909_151616_30765a77bc55](viewer.html?id=gait_20260909_151616_30765a77bc55) | 56.935 | basic/failure | 200–200 | 1+0 / 17 |
| [gait_20260909_151734_72594dc0ad20](viewer.html?id=gait_20260909_151734_72594dc0ad20) | 58.367 | basic/failure | 200–200 | 1+0 / 17 |
| [gait_20260909_151931_7a0441190ea8](viewer.html?id=gait_20260909_151931_7a0441190ea8) | 120.005 | basic/unlabeled | 200–200 | 1+0 / 26 |
| [gait_20260909_152145_fc06025e3eb8](viewer.html?id=gait_20260909_152145_fc06025e3eb8) | 69.700 | basic/failure | 200–200 | 1+0 / 34 |
| [gait_20260909_152557_7ddbfc67c108](viewer.html?id=gait_20260909_152557_7ddbfc67c108) | 120.004 | stairs/unlabeled | 50–200 | 1+1 / 19 |
| [gait_20260909_154654_65c5ae680a24](viewer.html?id=gait_20260909_154654_65c5ae680a24) | 111.551 | stairs/unlabeled | 50–50 | 2+1 / 34 |
| [gait_20260909_154856_5809460fe5ab](viewer.html?id=gait_20260909_154856_5809460fe5ab) | 84.152 | stairs/unlabeled | 50–50 | 2+0 / 33 |

逐段的通行范围、全部片段编号、观察依据和具体问题见 [逐段记录](RECORDING_NOTES.md)；机器可读格式见 [passages.json](passages.json)、[segments.json](segments.json)、[review_queue.json](review_queue.json)。

## 浏览与复现

双击 `index.html` 用本机浏览器打开；选择录制后可播放、跳转、慢放，切换源/接收时间、点云侧视/俯视/立体投影及截面宽度。关节采用SDK模型运动学骨架，机身固定原点、移除初始yaw；关节数字通道映射及IMU外参仍未实物验证，不是世界轨迹，不做物理仿真或地形重建。

如需HTTP本地预览，在项目根目录运行（仅绑定本机）：

```powershell
.venv-win\Scripts\python.exe -B -m http.server 8766 --bind 127.0.0.1 --directory artifacts/s10-recording-review-20260909
```

脚本：review.py 复用既有消息定义与解码帮助函数、server.py已测试的点云抽样；build.py复用validate_motion.py中的关节校准/模型和运动学。match_stairs.py已检查，其地形拟合/物理搜索不适用于本轮，因此未执行。plots.py用本机Anaconda的 `python -s -B` 绘制科学图表。其余脚本用项目 `.venv-win`。解码缓存存在时review.py直接复用；build.py的 `--reuse-replay` 用于仅更新索引/报告时保留已生成运动学采样。

保留一个可运行自检 `build.py`：索引对全部录制覆盖完整且连续不重叠、时间范围有效、最近样本选择边界、四元数基本运算、无整段结果传播、全部人工状态仍待复核。结果见 [validation.json](validation.json)。原始文件只读；无远端/机器人连接、训练或与本轮无关的源代码改动。
