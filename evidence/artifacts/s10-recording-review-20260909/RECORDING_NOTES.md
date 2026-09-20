# 逐段清点与候选片段

以下“观察”均指模型对信号图/点云速览的初览，人工状态全部待复核；片段原始文件与精确纳秒边界见 segments.json。

## gait_20260909_144907_fd57c06d30d3

[同步预览](viewer.html?id=gait_20260909_144907_fd57c06d30d3&t=0.000) · [信号](previews/gait_20260909_144907_fd57c06d30d3_signals.png) · [点云](previews/gait_20260909_144907_fd57c06d30d3_clouds.jpg)

原始：`D:\S10Data\050\2026-09-09\gait_20260909_144907_fd57c06d30d3`；时长 120.005768s；标签 `basic/unlabeled`，参数 `{}`；人工events为空。

候选通行/上下文：01：0.000–120.006s，candidate_continuous_passage，uncertain

实际控制（源秒）：0.011–119.961：state 17 / gait 0x1001

观察：模型检查完整信号图与12帧点云速览：主要为连续近水平地面；约4–23s报告vx多次正负切换，24–58s和60–82s有关节周期运动/转向，82–98s近静止。仅确认信号与抽样地面形态，不能确认每次移动的完成条件。

数据问题：/HANDLE_STEER receive-source 中位数 -3.102s；不得按源时间直接与机身话题混合；/STEER receive-source 中位数 -3.103s；不得按源时间直接与机身话题混合；metadata.files 消息数 112466 与数据库实际 56233 不一致；按实际表计数

| 片段 | 通行组 | 源秒 | 地形/动作候选 | 上下方向 | 子结果 |
|---|---|---|---|---|---|
| [001](viewer.html?id=gait_20260909_144907_fd57c06d30d3&t=0.000&end=3.500) | 01 | 0.000–3.500 | flat_candidate / 等待候选 | unknown | uncertain |
| [002](viewer.html?id=gait_20260909_144907_fd57c06d30d3&t=3.500&end=23.500) | 01 | 3.500–23.500 | flat_candidate / 行走/调整候选 | unknown | uncertain |
| [003](viewer.html?id=gait_20260909_144907_fd57c06d30d3&t=23.500&end=50.000) | 01 | 23.500–50.000 | flat_candidate / 转向候选 | unknown | uncertain |
| [004](viewer.html?id=gait_20260909_144907_fd57c06d30d3&t=50.000&end=51.000) | 01 | 50.000–51.000 | flat_candidate / 行走/调整候选 | unknown | uncertain |
| [005](viewer.html?id=gait_20260909_144907_fd57c06d30d3&t=51.000&end=58.500) | 01 | 51.000–58.500 | flat_candidate / 转向候选 | unknown | uncertain |
| [006](viewer.html?id=gait_20260909_144907_fd57c06d30d3&t=58.500&end=59.000) | 01 | 58.500–59.000 | flat_candidate / 行走/调整候选 | unknown | uncertain |
| [007](viewer.html?id=gait_20260909_144907_fd57c06d30d3&t=59.000&end=60.000) | 01 | 59.000–60.000 | flat_candidate / 等待候选 | unknown | uncertain |
| [008](viewer.html?id=gait_20260909_144907_fd57c06d30d3&t=60.000&end=82.500) | 01 | 60.000–82.500 | flat_candidate / 行走/调整候选 | unknown | uncertain |
| [009](viewer.html?id=gait_20260909_144907_fd57c06d30d3&t=82.500&end=98.500) | 01 | 82.500–98.500 | flat_candidate / 等待候选 | unknown | uncertain |
| [010](viewer.html?id=gait_20260909_144907_fd57c06d30d3&t=98.500&end=101.500) | 01 | 98.500–101.500 | flat_candidate / 行走/调整候选 | unknown | uncertain |
| [011](viewer.html?id=gait_20260909_144907_fd57c06d30d3&t=101.500&end=103.500) | 01 | 101.500–103.500 | flat_candidate / 等待候选 | unknown | uncertain |
| [012](viewer.html?id=gait_20260909_144907_fd57c06d30d3&t=103.500&end=104.000) | 01 | 103.500–104.000 | flat_candidate / 行走/调整候选 | unknown | uncertain |
| [013](viewer.html?id=gait_20260909_144907_fd57c06d30d3&t=104.000&end=119.500) | 01 | 104.000–119.500 | flat_candidate / 转向候选 | unknown | uncertain |
| [014](viewer.html?id=gait_20260909_144907_fd57c06d30d3&t=119.500&end=120.000) | 01 | 119.500–120.000 | flat_candidate / 行走/调整候选 | unknown | uncertain |
| [015](viewer.html?id=gait_20260909_144907_fd57c06d30d3&t=120.000&end=120.006) | 01 | 120.000–120.006 | unknown / 数据中断/不可判断 | unknown | interrupted_data |

待确认：
- [3.00–24.00s](viewer.html?id=gait_20260909_144907_fd57c06d30d3&t=3.000&end=24.000)：前后往返是否应计为独立通行？各次终点是什么？
- [23.00–59.00s](viewer.html?id=gait_20260909_144907_fd57c06d30d3&t=23.000&end=59.000)：这段是否为原地转向/侧移测试？
- [98.00–120.01s](viewer.html?id=gait_20260909_144907_fd57c06d30d3&t=98.000&end=120.006)：尾段移动是否在录制结束前完成？

## gait_20260909_145241_9cae0ba51169

[同步预览](viewer.html?id=gait_20260909_145241_9cae0ba51169&t=0.000) · [信号](previews/gait_20260909_145241_9cae0ba51169_signals.png) · [点云](previews/gait_20260909_145241_9cae0ba51169_clouds.jpg)

原始：`D:\S10Data\050\2026-09-09\gait_20260909_145241_9cae0ba51169`；时长 16.698717s；标签 `stairs/unlabeled`，参数 `{'height_cm': 15, 'depth_cm': 55}`；人工events为空。

候选通行/上下文：01：0.000–16.699s，candidate_continuous_passage，uncertain

实际控制（源秒）：-0.010–16.640：state 17 / gait 0x1001

观察：0.7/2.1s前点云有连续阶梯；约2.5s起pitch下降至约−12°，2–6s轮关节明显转动，10s后基本静止；实际步态全程0x1001。上楼方向为点云+姿态的候选推断，后半段停在何处未确认。

数据问题：/HANDLE_STEER receive-source 中位数 -3.241s；不得按源时间直接与机身话题混合；/STEER receive-source 中位数 -3.241s；不得按源时间直接与机身话题混合；metadata.files 消息数 15270 与数据库实际 7635 不一致；按实际表计数

| 片段 | 通行组 | 源秒 | 地形/动作候选 | 上下方向 | 子结果 |
|---|---|---|---|---|---|
| [001](viewer.html?id=gait_20260909_145241_9cae0ba51169&t=0.000&end=1.500) | 01 | 0.000–1.500 | stairs_candidate / 等待候选 | unknown | uncertain |
| [002](viewer.html?id=gait_20260909_145241_9cae0ba51169&t=1.500&end=2.000) | 01 | 1.500–2.000 | stairs_candidate / 行走/调整候选 | unknown | uncertain |
| [003](viewer.html?id=gait_20260909_145241_9cae0ba51169&t=2.000&end=3.000) | 01 | 2.000–3.000 | stairs_candidate / 行走/调整候选 | unknown | uncertain |
| [004](viewer.html?id=gait_20260909_145241_9cae0ba51169&t=3.000&end=6.500) | 01 | 3.000–6.500 | stairs_candidate / 上楼尝试候选 | up_candidate | uncertain |
| [005](viewer.html?id=gait_20260909_145241_9cae0ba51169&t=6.500&end=7.500) | 01 | 6.500–7.500 | stairs_candidate / 等待候选 | unknown | uncertain |
| [006](viewer.html?id=gait_20260909_145241_9cae0ba51169&t=7.500&end=10.000) | 01 | 7.500–10.000 | stairs_candidate / 上楼尝试候选 | up_candidate | uncertain |
| [007](viewer.html?id=gait_20260909_145241_9cae0ba51169&t=10.000&end=16.699) | 01 | 10.000–16.699 | stairs_candidate / 等待候选 | unknown | uncertain |

待确认：
- [1.50–7.00s](viewer.html?id=gait_20260909_145241_9cae0ba51169&t=1.500&end=7.000)：是否使用基础步态尝试上楼，实际接触从何时开始？
- [6.00–16.70s](viewer.html?id=gait_20260909_145241_9cae0ba51169&t=6.000&end=16.699)：是在台阶上等待、卡住、放弃，还是完成后停止？

## gait_20260909_145331_616f9d8038d9

[同步预览](viewer.html?id=gait_20260909_145331_616f9d8038d9&t=0.000) · [信号](previews/gait_20260909_145331_616f9d8038d9_signals.png) · [点云](previews/gait_20260909_145331_616f9d8038d9_clouds.jpg)

原始：`D:\S10Data\050\2026-09-09\gait_20260909_145331_616f9d8038d9`；时长 93.260023s；标签 `stairs/success`，参数 `{'height_cm': 15, 'depth_cm': 55}`；人工events为空。

候选通行/上下文：01：0.000–93.260s，candidate_continuous_passage，uncertain

实际控制（源秒）：0.024–9.144：state 17 / gait 0x1001；9.144–29.544：state 17 / gait 0x1003；29.544–38.064：state 17 / gait 0x1001；38.064–59.004：state 17 / gait 0x1003；59.004–64.744：state 17 / gait 0x1001；64.744–93.144：state 17 / gait 0x1003

观察：首段约13–25s持续负pitch及腿关节活动；之前前点云阶梯明显，过程中后点云出现阶梯，支持上楼候选。
观察：25–39.5s姿态回平，约31–34s发生轮式移动；35s点云前方再次有阶梯。可能是中间平台后接下一梯段，也可能重新接近；保留在同一通行候选。
观察：40–55s再次持续负pitch及腿关节运动，42.7/50.5s附近前后点云见阶梯。
观察：55–68s姿态大体回平，60–63s有快速轮关节活动与转向；58.3/66.1s前方仍见阶梯；平台还是重试接近需确认。
观察：68–86.5s负pitch及腿关节活动；75–79s在倾斜姿态下近静止，不能据此标作平平台。89.4s抽样点云前方趋平。

数据问题：/GAIT 源时间缺失/非递增；/HANDLE_STEER receive-source 中位数 -3.368s；不得按源时间直接与机身话题混合；/STEER receive-source 中位数 -3.368s；不得按源时间直接与机身话题混合；metadata.files 消息数 86864 与数据库实际 43432 不一致；按实际表计数

| 片段 | 通行组 | 源秒 | 地形/动作候选 | 上下方向 | 子结果 |
|---|---|---|---|---|---|
| [001](viewer.html?id=gait_20260909_145331_616f9d8038d9&t=0.000&end=3.500) | 01 | 0.000–3.500 | unknown / 等待候选 | unknown | uncertain |
| [002](viewer.html?id=gait_20260909_145331_616f9d8038d9&t=3.500&end=8.000) | 01 | 3.500–8.000 | unknown / 行走/调整候选 | unknown | uncertain |
| [003](viewer.html?id=gait_20260909_145331_616f9d8038d9&t=8.000&end=10.500) | 01 | 8.000–10.500 | unknown / 等待候选 | unknown | uncertain |
| [004](viewer.html?id=gait_20260909_145331_616f9d8038d9&t=10.500&end=12.500) | 01 | 10.500–12.500 | unknown / 行走/调整候选 | unknown | uncertain |
| [005](viewer.html?id=gait_20260909_145331_616f9d8038d9&t=12.500&end=13.000) | 01 | 12.500–13.000 | stairs_candidate / 行走/调整候选 | unknown | uncertain |
| [006](viewer.html?id=gait_20260909_145331_616f9d8038d9&t=13.000&end=13.500) | 01 | 13.000–13.500 | stairs_candidate / 转向候选 | unknown | uncertain |
| [007](viewer.html?id=gait_20260909_145331_616f9d8038d9&t=13.500&end=24.500) | 01 | 13.500–24.500 | stairs_candidate / 上楼候选 | up_candidate | uncertain |
| [008](viewer.html?id=gait_20260909_145331_616f9d8038d9&t=24.500&end=25.000) | 01 | 24.500–25.000 | stairs_candidate / 行走/调整候选 | unknown | uncertain |
| [009](viewer.html?id=gait_20260909_145331_616f9d8038d9&t=25.000&end=28.000) | 01 | 25.000–28.000 | landing_or_approach_unknown / 平台行走/下一段接近候选 | unknown | uncertain |
| [010](viewer.html?id=gait_20260909_145331_616f9d8038d9&t=28.000&end=30.000) | 01 | 28.000–30.000 | landing_or_approach_unknown / 平台/平地等待候选 | unknown | uncertain |
| [011](viewer.html?id=gait_20260909_145331_616f9d8038d9&t=30.000&end=34.000) | 01 | 30.000–34.000 | landing_or_approach_unknown / 平台行走/下一段接近候选 | unknown | uncertain |
| [012](viewer.html?id=gait_20260909_145331_616f9d8038d9&t=34.000&end=39.500) | 01 | 34.000–39.500 | landing_or_approach_unknown / 平台/平地等待候选 | unknown | uncertain |
| [013](viewer.html?id=gait_20260909_145331_616f9d8038d9&t=39.500&end=41.500) | 01 | 39.500–41.500 | stairs_candidate / 行走/调整候选 | unknown | uncertain |
| [014](viewer.html?id=gait_20260909_145331_616f9d8038d9&t=41.500&end=54.500) | 01 | 41.500–54.500 | stairs_candidate / 上楼候选 | up_candidate | uncertain |
| [015](viewer.html?id=gait_20260909_145331_616f9d8038d9&t=54.500&end=55.000) | 01 | 54.500–55.000 | stairs_candidate / 转向候选 | unknown | uncertain |
| [016](viewer.html?id=gait_20260909_145331_616f9d8038d9&t=55.000&end=55.500) | 01 | 55.000–55.500 | landing_or_approach_unknown / 平台行走/下一段接近候选 | unknown | uncertain |
| [017](viewer.html?id=gait_20260909_145331_616f9d8038d9&t=55.500&end=59.500) | 01 | 55.500–59.500 | landing_or_approach_unknown / 平台/平地等待候选 | unknown | uncertain |
| [018](viewer.html?id=gait_20260909_145331_616f9d8038d9&t=59.500&end=60.000) | 01 | 59.500–60.000 | landing_or_approach_unknown / 平台行走/下一段接近候选 | unknown | uncertain |
| [019](viewer.html?id=gait_20260909_145331_616f9d8038d9&t=60.000&end=64.000) | 01 | 60.000–64.000 | landing_or_approach_unknown / 平台行走/下一段接近候选 | unknown | uncertain |
| [020](viewer.html?id=gait_20260909_145331_616f9d8038d9&t=64.000&end=66.000) | 01 | 64.000–66.000 | landing_or_approach_unknown / 平台/平地等待候选 | unknown | uncertain |
| [021](viewer.html?id=gait_20260909_145331_616f9d8038d9&t=66.000&end=68.000) | 01 | 66.000–68.000 | landing_or_approach_unknown / 平台行走/下一段接近候选 | unknown | uncertain |
| [022](viewer.html?id=gait_20260909_145331_616f9d8038d9&t=68.000&end=75.000) | 01 | 68.000–75.000 | stairs_candidate / 上楼候选 | up_candidate | uncertain |
| [023](viewer.html?id=gait_20260909_145331_616f9d8038d9&t=75.000&end=78.500) | 01 | 75.000–78.500 | stairs_candidate / 等待候选 | unknown | uncertain |
| [024](viewer.html?id=gait_20260909_145331_616f9d8038d9&t=78.500&end=79.000) | 01 | 78.500–79.000 | stairs_candidate / 上楼候选 | up_candidate | uncertain |
| [025](viewer.html?id=gait_20260909_145331_616f9d8038d9&t=79.000&end=86.000) | 01 | 79.000–86.000 | stairs_candidate / 上楼候选 | up_candidate | uncertain |
| [026](viewer.html?id=gait_20260909_145331_616f9d8038d9&t=86.000&end=86.500) | 01 | 86.000–86.500 | stairs_candidate / 行走/调整候选 | unknown | uncertain |
| [027](viewer.html?id=gait_20260909_145331_616f9d8038d9&t=86.500&end=93.260) | 01 | 86.500–93.260 | unknown / 等待候选 | unknown | uncertain |

待确认：
- [24.00–40.00s](viewer.html?id=gait_20260909_145331_616f9d8038d9&t=24.000&end=40.000)：第一段后是否为中间平台并继续同一次上楼？还是返回同一障碍重试？
- [54.00–69.00s](viewer.html?id=gait_20260909_145331_616f9d8038d9&t=54.000&end=69.000)：第二处连接区是否为平台；有无转向或人工调整？
- [74.00–81.00s](viewer.html?id=gait_20260909_145331_616f9d8038d9&t=74.000&end=81.000)：倾斜姿态停顿是在楼梯上等待还是卡住/干预？
- [85.00–93.26s](viewer.html?id=gait_20260909_145331_616f9d8038d9&t=85.000&end=93.260)：最后一段是否完成；三段阶梯可否赋予梯段/平台编号？

## gait_20260909_145914_2e0541078764

[同步预览](viewer.html?id=gait_20260909_145914_2e0541078764&t=0.000) · [信号](previews/gait_20260909_145914_2e0541078764_signals.png) · [点云](previews/gait_20260909_145914_2e0541078764_clouds.jpg)

原始：`D:\S10Data\050\2026-09-09\gait_20260909_145914_2e0541078764`；时长 39.444364s；标签 `basic/success`，参数 `{}`；人工events为空。

候选通行/上下文：01：0.000–39.444s，candidate_continuous_passage，uncertain

实际控制（源秒）：0.029–39.429：state 17 / gait 0x1001

观察：信号和点云初览前21s主要为近水平地面行走/转向；实际基础步态。
观察：21.4s前方出现台阶状边缘；24–27s pitch约−12°且腿轮活动，随后报告vx转负；27.9s前方边缘仍可见。不能把基础标签解释为全程平地，也不能据整段成功判断越障完成。

数据问题：/HANDLE_STEER receive-source 中位数 -3.706s；不得按源时间直接与机身话题混合；/STEER receive-source 中位数 -3.706s；不得按源时间直接与机身话题混合；metadata.files 消息数 36600 与数据库实际 18300 不一致；按实际表计数

| 片段 | 通行组 | 源秒 | 地形/动作候选 | 上下方向 | 子结果 |
|---|---|---|---|---|---|
| [001](viewer.html?id=gait_20260909_145914_2e0541078764&t=0.000&end=1.000) | 01 | 0.000–1.000 | flat_candidate / 等待候选 | unknown | uncertain |
| [002](viewer.html?id=gait_20260909_145914_2e0541078764&t=1.000&end=8.000) | 01 | 1.000–8.000 | flat_candidate / 行走/调整候选 | unknown | uncertain |
| [003](viewer.html?id=gait_20260909_145914_2e0541078764&t=8.000&end=10.500) | 01 | 8.000–10.500 | flat_candidate / 等待候选 | unknown | uncertain |
| [004](viewer.html?id=gait_20260909_145914_2e0541078764&t=10.500&end=15.000) | 01 | 10.500–15.000 | flat_candidate / 行走/调整候选 | unknown | uncertain |
| [005](viewer.html?id=gait_20260909_145914_2e0541078764&t=15.000&end=16.500) | 01 | 15.000–16.500 | flat_candidate / 转向候选 | unknown | uncertain |
| [006](viewer.html?id=gait_20260909_145914_2e0541078764&t=16.500&end=21.000) | 01 | 16.500–21.000 | flat_candidate / 行走/调整候选 | unknown | uncertain |
| [007](viewer.html?id=gait_20260909_145914_2e0541078764&t=21.000&end=24.000) | 01 | 21.000–24.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [008](viewer.html?id=gait_20260909_145914_2e0541078764&t=24.000&end=24.500) | 01 | 24.000–24.500 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [009](viewer.html?id=gait_20260909_145914_2e0541078764&t=24.500&end=26.500) | 01 | 24.500–26.500 | ledge_candidate / 姿态变化/越障候选 | unknown | uncertain |
| [010](viewer.html?id=gait_20260909_145914_2e0541078764&t=26.500&end=27.000) | 01 | 26.500–27.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [011](viewer.html?id=gait_20260909_145914_2e0541078764&t=27.000&end=28.000) | 01 | 27.000–28.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [012](viewer.html?id=gait_20260909_145914_2e0541078764&t=28.000&end=29.000) | 01 | 28.000–29.000 | ledge_candidate / 等待候选 | unknown | uncertain |
| [013](viewer.html?id=gait_20260909_145914_2e0541078764&t=29.000&end=39.444) | 01 | 29.000–39.444 | unknown / 等待候选 | unknown | uncertain |

待确认：
- [21.00–29.00s](viewer.html?id=gait_20260909_145914_2e0541078764&t=21.000&end=29.000)：是否接触/尝试上台后退回？障碍是什么，是否完成？
- [29.00–39.44s](viewer.html?id=gait_20260909_145914_2e0541078764&t=29.000&end=39.444)：尾段静止是在等待还是已结束通行？

## gait_20260909_150146_9b19dc6690ec

[同步预览](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=0.000) · [信号](previews/gait_20260909_150146_9b19dc6690ec_signals.png) · [点云](previews/gait_20260909_150146_9b19dc6690ec_clouds.jpg)

原始：`D:\S10Data\050\2026-09-09\gait_20260909_150146_9b19dc6690ec`；时长 114.639897s；标签 `ledge/success`，参数 `{'height_cm': 30}`；人工events为空。

候选通行/上下文：01：0.000–19.000s，candidate_continuous_passage，uncertain；02：19.000–37.000s，candidate_continuous_passage，uncertain；03：37.000–50.000s，candidate_continuous_passage，uncertain；04：50.000–75.000s，candidate_continuous_passage，uncertain；05：75.000–93.000s，candidate_continuous_passage，uncertain；06：93.000–114.640s，candidate_continuous_passage，uncertain

实际控制（源秒）：0.005–6.565：state 17 / gait 0x1001；6.565–8.945：state 17 / gait 0x1003；8.945–31.125：state 17 / gait 0x1002；31.125–38.125：state 17 / gait 0x1001；38.125–48.298：state 17 / gait 0x1002；48.298–67.538：state 17 / gait 0x1001；67.538–72.598：state 17 / gait 0x1002；72.598–86.138：state 17 / gait 0x1001；86.138–90.838：state 17 / gait 0x1002；90.838–103.578：state 17 / gait 0x1001；103.578–109.738：state 17 / gait 0x1002；109.738–114.588：state 17 / gait 0x1001

观察：全段速览多次出现单个抬高边缘与平台；13/16/27/70/106s负pitch突变，45/89s正pitch突变，伴腿轮活动；中间有姿态回平和转向。19/37/50/75/93s为活动簇间候选通行分界，合并/拆分与同一高台身份均未人工确认。
观察：43s前点云边缘后下落，45s正pitch与轮速峰值配合；下台候选。
观察：81.2s前点云可见高低边缘，89s正pitch峰值，90.8s前地面形态改变；下台候选。

数据问题：/GAIT 源时间缺失/非递增；/HANDLE_STEER receive-source 中位数 +0.395s；不得按源时间直接与机身话题混合；/STEER receive-source 中位数 +0.395s；不得按源时间直接与机身话题混合；metadata.files 消息数 106446 与数据库实际 53223 不一致；按实际表计数

| 片段 | 通行组 | 源秒 | 地形/动作候选 | 上下方向 | 子结果 |
|---|---|---|---|---|---|
| [001](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=0.000&end=2.000) | 01 | 0.000–2.000 | ledge_candidate / 等待候选 | unknown | uncertain |
| [002](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=2.000&end=6.000) | 01 | 2.000–6.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [003](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=6.000&end=9.000) | 01 | 6.000–9.000 | ledge_candidate / 等待候选 | unknown | uncertain |
| [004](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=9.000&end=11.000) | 01 | 9.000–11.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [005](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=11.000&end=12.500) | 01 | 11.000–12.500 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [006](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=12.500&end=13.000) | 01 | 12.500–13.000 | ledge_candidate / 姿态变化/越障候选 | unknown | uncertain |
| [007](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=13.000&end=15.500) | 01 | 13.000–15.500 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [008](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=15.500&end=16.500) | 01 | 15.500–16.500 | ledge_candidate / 姿态变化/越障候选 | unknown | uncertain |
| [009](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=16.500&end=17.000) | 01 | 16.500–17.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [010](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=17.000&end=19.000) | 01 | 17.000–19.000 | ledge_candidate / 等待候选 | unknown | uncertain |
| [011](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=19.000&end=19.500) | 02 | 19.000–19.500 | ledge_candidate / 等待候选 | unknown | uncertain |
| [012](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=19.500&end=21.000) | 02 | 19.500–21.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [013](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=21.000&end=22.000) | 02 | 21.000–22.000 | ledge_candidate / 等待候选 | unknown | uncertain |
| [014](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=22.000&end=26.000) | 02 | 22.000–26.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [015](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=26.000&end=27.000) | 02 | 26.000–27.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [016](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=27.000&end=28.000) | 02 | 27.000–28.000 | ledge_candidate / 姿态变化/越障候选 | unknown | uncertain |
| [017](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=28.000&end=29.000) | 02 | 28.000–29.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [018](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=29.000&end=29.500) | 02 | 29.000–29.500 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [019](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=29.500&end=31.000) | 02 | 29.500–31.000 | ledge_candidate / 等待候选 | unknown | uncertain |
| [020](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=31.000&end=32.000) | 02 | 31.000–32.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [021](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=32.000&end=33.500) | 02 | 32.000–33.500 | ledge_candidate / 等待候选 | unknown | uncertain |
| [022](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=33.500&end=34.000) | 02 | 33.500–34.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [023](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=34.000&end=37.000) | 02 | 34.000–37.000 | ledge_candidate / 转向候选 | unknown | uncertain |
| [024](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=37.000&end=44.000) | 03 | 37.000–44.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [025](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=44.000&end=44.500) | 03 | 44.000–44.500 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [026](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=44.500&end=45.500) | 03 | 44.500–45.500 | ledge_candidate / 下台候选 | down_candidate | uncertain |
| [027](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=45.500&end=46.000) | 03 | 45.500–46.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [028](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=46.000&end=48.000) | 03 | 46.000–48.000 | ledge_candidate / 等待候选 | unknown | uncertain |
| [029](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=48.000&end=49.000) | 03 | 48.000–49.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [030](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=49.000&end=50.000) | 03 | 49.000–50.000 | ledge_candidate / 转向候选 | unknown | uncertain |
| [031](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=50.000&end=52.500) | 04 | 50.000–52.500 | ledge_candidate / 转向候选 | unknown | uncertain |
| [032](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=52.500&end=55.000) | 04 | 52.500–55.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [033](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=55.000&end=57.000) | 04 | 55.000–57.000 | ledge_candidate / 等待候选 | unknown | uncertain |
| [034](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=57.000&end=66.500) | 04 | 57.000–66.500 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [035](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=66.500&end=67.500) | 04 | 66.500–67.500 | ledge_candidate / 等待候选 | unknown | uncertain |
| [036](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=67.500&end=69.000) | 04 | 67.500–69.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [037](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=69.000&end=69.500) | 04 | 69.000–69.500 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [038](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=69.500&end=70.000) | 04 | 69.500–70.000 | ledge_candidate / 姿态变化/越障候选 | unknown | uncertain |
| [039](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=70.000&end=71.000) | 04 | 70.000–71.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [040](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=71.000&end=74.500) | 04 | 71.000–74.500 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [041](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=74.500&end=75.000) | 04 | 74.500–75.000 | ledge_candidate / 转向候选 | unknown | uncertain |
| [042](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=75.000&end=76.500) | 05 | 75.000–76.500 | ledge_candidate / 转向候选 | unknown | uncertain |
| [043](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=76.500&end=88.000) | 05 | 76.500–88.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [044](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=88.000&end=89.000) | 05 | 88.000–89.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [045](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=89.000&end=89.500) | 05 | 89.000–89.500 | ledge_candidate / 下台候选 | down_candidate | uncertain |
| [046](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=89.500&end=90.000) | 05 | 89.500–90.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [047](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=90.000&end=92.500) | 05 | 90.000–92.500 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [048](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=92.500&end=93.000) | 05 | 92.500–93.000 | ledge_candidate / 转向候选 | unknown | uncertain |
| [049](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=93.000&end=94.500) | 06 | 93.000–94.500 | ledge_candidate / 转向候选 | unknown | uncertain |
| [050](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=94.500&end=100.000) | 06 | 94.500–100.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [051](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=100.000&end=103.500) | 06 | 100.000–103.500 | ledge_candidate / 等待候选 | unknown | uncertain |
| [052](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=103.500&end=105.000) | 06 | 103.500–105.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [053](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=105.000&end=106.000) | 06 | 105.000–106.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [054](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=106.000&end=107.000) | 06 | 106.000–107.000 | ledge_candidate / 姿态变化/越障候选 | unknown | uncertain |
| [055](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=107.000&end=108.000) | 06 | 107.000–108.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [056](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=108.000&end=110.500) | 06 | 108.000–110.500 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [057](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=110.500&end=114.640) | 06 | 110.500–114.640 | ledge_candidate / 等待候选 | unknown | uncertain |

待确认：
- [10.00–19.00s](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=10.000&end=19.000)：13s与16s两次动作是上台后的退回/重试，还是两个独立通行？
- [25.00–37.00s](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=25.000&end=37.000)：此处上台后是否完成，37s能否作为下一通行准备边界？
- [42.00–53.00s](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=42.000&end=53.000)：45s是否下台；是否与前面同一高台？
- [68.00–77.00s](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=68.000&end=77.000)：70s动作完成与下一次接近的分界？
- [87.00–96.00s](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=87.000&end=96.000)：89s是否下台；转向后是否重过同一高台？
- [104.00–114.64s](viewer.html?id=gait_20260909_150146_9b19dc6690ec&t=104.000&end=114.640)：末次动作是否完成？记录30cm是否对应所有这些障碍？

## gait_20260909_150705_4fe2b5f8eebd

[同步预览](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=0.000) · [信号](previews/gait_20260909_150705_4fe2b5f8eebd_signals.png) · [点云](previews/gait_20260909_150705_4fe2b5f8eebd_clouds.jpg)

原始：`D:\S10Data\050\2026-09-09\gait_20260909_150705_4fe2b5f8eebd`；时长 93.255846s；标签 `ledge/success`，参数 `{'height_cm': 32}`；人工events为空。

候选通行/上下文：01：0.000–23.000s，candidate_continuous_passage，uncertain；02：23.000–38.000s，candidate_continuous_passage，uncertain；03：38.000–48.000s，candidate_continuous_passage，uncertain；04：48.000–78.000s，candidate_continuous_passage，uncertain；05：78.000–93.256s，candidate_continuous_passage，uncertain

实际控制（源秒）：-0.002–7.538：state 17 / gait 0x1001；7.538–22.658：state 17 / gait 0x1002；22.658–25.578：state 17 / gait 0x1001；25.578–32.058：state 17 / gait 0x1002；32.058–37.698：state 17 / gait 0x1001；37.698–47.358：state 17 / gait 0x1002；47.358–57.958：state 17 / gait 0x1001；57.958–58.618：state 17 / gait 0x1003；58.618–80.518：state 17 / gait 0x1002；80.518–93.218：state 17 / gait 0x1001

观察：点云反复出现前方台沿与后方地面；11–18、28–36、40–43.5、63–65.5、81–84s有多次姿态/关节活动簇。23/38/48/78s为候选通行分界；第一簇同时有正负pitch与大roll，不能逐峰当作成功上/下台。66–80s长时间近静止但0x1002仍保持。

数据问题：/GAIT 源时间缺失/非递增；metadata.files 消息数 85804 与数据库实际 42902 不一致；按实际表计数

| 片段 | 通行组 | 源秒 | 地形/动作候选 | 上下方向 | 子结果 |
|---|---|---|---|---|---|
| [001](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=0.000&end=0.500) | 01 | 0.000–0.500 | ledge_candidate / 等待候选 | unknown | uncertain |
| [002](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=0.500&end=5.500) | 01 | 0.500–5.500 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [003](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=5.500&end=8.500) | 01 | 5.500–8.500 | ledge_candidate / 等待候选 | unknown | uncertain |
| [004](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=8.500&end=10.500) | 01 | 8.500–10.500 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [005](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=10.500&end=11.000) | 01 | 10.500–11.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [006](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=11.000&end=11.500) | 01 | 11.000–11.500 | ledge_candidate / 姿态变化/越障候选 | unknown | uncertain |
| [007](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=11.500&end=12.500) | 01 | 11.500–12.500 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [008](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=12.500&end=13.000) | 01 | 12.500–13.000 | ledge_candidate / 姿态变化/越障候选 | unknown | uncertain |
| [009](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=13.000&end=13.500) | 01 | 13.000–13.500 | ledge_candidate / 转向候选 | unknown | uncertain |
| [010](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=13.500&end=14.000) | 01 | 13.500–14.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [011](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=14.000&end=16.500) | 01 | 14.000–16.500 | ledge_candidate / 姿态变化/越障候选 | unknown | uncertain |
| [012](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=16.500&end=18.000) | 01 | 16.500–18.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [013](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=18.000&end=20.000) | 01 | 18.000–20.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [014](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=20.000&end=21.000) | 01 | 20.000–21.000 | ledge_candidate / 等待候选 | unknown | uncertain |
| [015](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=21.000&end=23.000) | 01 | 21.000–23.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [016](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=23.000&end=28.000) | 02 | 23.000–28.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [017](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=28.000&end=28.500) | 02 | 28.000–28.500 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [018](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=28.500&end=30.500) | 02 | 28.500–30.500 | ledge_candidate / 姿态变化/越障候选 | unknown | uncertain |
| [019](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=30.500&end=31.000) | 02 | 30.500–31.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [020](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=31.000&end=32.000) | 02 | 31.000–32.000 | ledge_candidate / 等待候选 | unknown | uncertain |
| [021](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=32.000&end=34.000) | 02 | 32.000–34.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [022](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=34.000&end=35.000) | 02 | 34.000–35.000 | ledge_candidate / 姿态变化/越障候选 | unknown | uncertain |
| [023](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=35.000&end=36.000) | 02 | 35.000–36.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [024](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=36.000&end=38.000) | 02 | 36.000–38.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [025](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=38.000&end=40.000) | 03 | 38.000–40.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [026](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=40.000&end=40.500) | 03 | 40.000–40.500 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [027](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=40.500&end=43.000) | 03 | 40.500–43.000 | ledge_candidate / 姿态变化/越障候选 | unknown | uncertain |
| [028](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=43.000&end=43.500) | 03 | 43.000–43.500 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [029](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=43.500&end=45.500) | 03 | 43.500–45.500 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [030](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=45.500&end=47.500) | 03 | 45.500–47.500 | ledge_candidate / 等待候选 | unknown | uncertain |
| [031](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=47.500&end=48.000) | 03 | 47.500–48.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [032](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=48.000&end=54.000) | 04 | 48.000–54.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [033](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=54.000&end=60.500) | 04 | 54.000–60.500 | ledge_candidate / 等待候选 | unknown | uncertain |
| [034](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=60.500&end=63.000) | 04 | 60.500–63.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [035](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=63.000&end=64.000) | 04 | 63.000–64.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [036](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=64.000&end=65.000) | 04 | 64.000–65.000 | ledge_candidate / 姿态变化/越障候选 | unknown | uncertain |
| [037](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=65.000&end=65.500) | 04 | 65.000–65.500 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [038](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=65.500&end=78.000) | 04 | 65.500–78.000 | ledge_candidate / 等待候选 | unknown | uncertain |
| [039](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=78.000&end=80.500) | 05 | 78.000–80.500 | ledge_candidate / 等待候选 | unknown | uncertain |
| [040](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=80.500&end=81.000) | 05 | 80.500–81.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [041](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=81.000&end=82.000) | 05 | 81.000–82.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [042](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=82.000&end=83.000) | 05 | 82.000–83.000 | ledge_candidate / 姿态变化/越障候选 | unknown | uncertain |
| [043](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=83.000&end=83.500) | 05 | 83.000–83.500 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [044](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=83.500&end=84.000) | 05 | 83.500–84.000 | ledge_candidate / 等待候选 | unknown | uncertain |
| [045](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=84.000&end=93.256) | 05 | 84.000–93.256 | ledge_candidate / 等待候选 | unknown | uncertain |

待确认：
- [10.00–19.00s](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=10.000&end=19.000)：是否发生上台、下台、失稳或连续重试？
- [27.00–37.00s](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=27.000&end=37.000)：两组动作是否同一通行内的调整/退回？
- [39.00–45.00s](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=39.000&end=45.000)：此簇是否完成高台通行？
- [62.00–84.00s](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=62.000&end=84.000)：64s后长停顿与82s动作是什么关系，是否卡住或人工处理？
- [84.00–93.26s](viewer.html?id=gait_20260909_150705_4fe2b5f8eebd&t=84.000&end=93.256)：末尾是否完成/放弃；32cm标签是否对应同一高台？

## gait_20260909_150921_9ed8457a8250

[同步预览](viewer.html?id=gait_20260909_150921_9ed8457a8250&t=0.000) · [信号](previews/gait_20260909_150921_9ed8457a8250_signals.png) · [点云](previews/gait_20260909_150921_9ed8457a8250_clouds.jpg)

原始：`D:\S10Data\050\2026-09-09\gait_20260909_150921_9ed8457a8250`；时长 89.367105s；标签 `ledge/success`，参数 `{'height_cm': 38}`；人工events为空。

候选通行/上下文：01：0.000–16.813s，candidate_continuous_passage，interrupted_control；02：16.813–89.367s，post_interruption_context，uncertain

实际控制（源秒）：-0.002–8.388：state 17 / gait 0x1001；8.388–13.968：state 17 / gait 0x1002；13.968–16.813：state 17 / gait 0x1001；16.813–18.814：state 2 / gait 0x0000；18.814–63.714：state 0 / gait 0x0000

观察：前方点云有台沿，约11s短负pitch；16–19s大幅roll/pitch倾斜，16.813s状态17→2，18.814s→0。整段manifest仍为success；事件归因和此前是否有成功通行需人工确认。
观察：退出控制后约19–30s关节近静止而机身倾斜；30–48s姿态再变化，可能有人处理但无人工事件证据。MOTION_INFO速度字段维持常值，不能推断持续运动。
观察：IMU/JOINTS_DATA/MOTION_INFO在约63.7s后无数据；前后点云有长源时间间隔，末尾恢复部分点云。

数据问题：/GAIT 源时间缺失/非递增；/IMU 接收流提前结束 63.748s（距录制末尾 25.620s）；/JOINTS_DATA 接收流提前结束 63.743s（距录制末尾 25.624s）；/JOINTS_DATA 5s 窗源时间采样率范围 50.0–200.0Hz（事件话题不视为掉帧）；/MOTION_INFO 接收流提前结束 63.716s（距录制末尾 25.651s）；/rslidar_front/points 有 1 个源时间长间隔，见 topics.csv / inventory.json；/rslidar_rear/points 有 1 个源时间长间隔，见 topics.csv / inventory.json；实际运动状态 2：16.813–18.814s（源时间）；实际运动状态 0：18.814–63.714s（源时间）；metadata.files 消息数 43756 与数据库实际 21878 不一致；按实际表计数

| 片段 | 通行组 | 源秒 | 地形/动作候选 | 上下方向 | 子结果 |
|---|---|---|---|---|---|
| [001](viewer.html?id=gait_20260909_150921_9ed8457a8250&t=0.000&end=4.500) | 01 | 0.000–4.500 | ledge_candidate / 等待候选 | unknown | uncertain |
| [002](viewer.html?id=gait_20260909_150921_9ed8457a8250&t=4.500&end=5.500) | 01 | 4.500–5.500 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [003](viewer.html?id=gait_20260909_150921_9ed8457a8250&t=5.500&end=6.000) | 01 | 5.500–6.000 | ledge_candidate / 姿态变化/越障候选 | unknown | uncertain |
| [004](viewer.html?id=gait_20260909_150921_9ed8457a8250&t=6.000&end=10.000) | 01 | 6.000–10.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [005](viewer.html?id=gait_20260909_150921_9ed8457a8250&t=10.000&end=10.500) | 01 | 10.000–10.500 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [006](viewer.html?id=gait_20260909_150921_9ed8457a8250&t=10.500&end=11.500) | 01 | 10.500–11.500 | ledge_candidate / 姿态变化/越障候选 | unknown | uncertain |
| [007](viewer.html?id=gait_20260909_150921_9ed8457a8250&t=11.500&end=12.000) | 01 | 11.500–12.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [008](viewer.html?id=gait_20260909_150921_9ed8457a8250&t=12.000&end=15.500) | 01 | 12.000–15.500 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [009](viewer.html?id=gait_20260909_150921_9ed8457a8250&t=15.500&end=16.000) | 01 | 15.500–16.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [010](viewer.html?id=gait_20260909_150921_9ed8457a8250&t=16.000&end=16.500) | 01 | 16.000–16.500 | unknown / 大倾角/恢复待确认 | unknown | uncertain |
| [011](viewer.html?id=gait_20260909_150921_9ed8457a8250&t=16.500&end=16.813) | 01 | 16.500–16.813 | unknown / 控制退出后的记录 | unknown | interrupted_control |
| [012](viewer.html?id=gait_20260909_150921_9ed8457a8250&t=16.813&end=19.500) | 02 | 16.813–19.500 | unknown / 控制退出后的记录 | unknown | interrupted_control |
| [013](viewer.html?id=gait_20260909_150921_9ed8457a8250&t=19.500&end=30.000) | 02 | 19.500–30.000 | unknown / 控制退出后的记录 | unknown | uncertain |
| [014](viewer.html?id=gait_20260909_150921_9ed8457a8250&t=30.000&end=45.000) | 02 | 30.000–45.000 | unknown / 控制退出后的记录 | unknown | uncertain |
| [015](viewer.html?id=gait_20260909_150921_9ed8457a8250&t=45.000&end=48.000) | 02 | 45.000–48.000 | unknown / 控制退出后的记录 | unknown | uncertain |
| [016](viewer.html?id=gait_20260909_150921_9ed8457a8250&t=48.000&end=63.750) | 02 | 48.000–63.750 | unknown / 控制退出后的记录 | unknown | uncertain |
| [017](viewer.html?id=gait_20260909_150921_9ed8457a8250&t=63.750&end=64.000) | 02 | 63.750–64.000 | unknown / 控制退出后的记录 | unknown | uncertain |
| [018](viewer.html?id=gait_20260909_150921_9ed8457a8250&t=64.000&end=89.367) | 02 | 64.000–89.367 | unknown / 数据中断/不可判断 | unknown | interrupted_data |

待确认：
- [10.00–20.00s](viewer.html?id=gait_20260909_150921_9ed8457a8250&t=10.000&end=20.000)：11s是否已经成功一次？16–19s大倾角是否摔倒/放弃/人工接管，整段success如何解释？
- [29.00–49.00s](viewer.html?id=gait_20260909_150921_9ed8457a8250&t=29.000&end=49.000)：是否人工扶正或搬动？
- [62.00–89.37s](viewer.html?id=gait_20260909_150921_9ed8457a8250&t=62.000&end=89.367)：后段是否设备停机/连接断开，尾部点云恢复发生了什么？

## gait_20260909_151135_f45a816f4062

[同步预览](viewer.html?id=gait_20260909_151135_f45a816f4062&t=0.000) · [信号](previews/gait_20260909_151135_f45a816f4062_signals.png) · [点云](previews/gait_20260909_151135_f45a816f4062_clouds.jpg)

原始：`D:\S10Data\050\2026-09-09\gait_20260909_151135_f45a816f4062`；时长 26.318587s；标签 `ledge/success`，参数 `{'height_cm': 38}`；人工events为空。

候选通行/上下文：01：0.000–26.319s，candidate_continuous_passage，uncertain

实际控制（源秒）：0.011–9.691：state 17 / gait 0x1001；9.691–15.791：state 17 / gait 0x1002；15.791–26.241：state 17 / gait 0x1001

观察：1–12s前点云见台沿；约12–13.5s负pitch达约−40°并有腿轮活动，14.3s后前点云趋平、后方保留高低结构；支持上台候选。19–23s有后续移动/转向，不能自动解释为已完全通过。

数据问题：/GAIT 源时间缺失/非递增；metadata.files 消息数 24166 与数据库实际 12083 不一致；按实际表计数

| 片段 | 通行组 | 源秒 | 地形/动作候选 | 上下方向 | 子结果 |
|---|---|---|---|---|---|
| [001](viewer.html?id=gait_20260909_151135_f45a816f4062&t=0.000&end=2.500) | 01 | 0.000–2.500 | ledge_candidate / 等待候选 | unknown | uncertain |
| [002](viewer.html?id=gait_20260909_151135_f45a816f4062&t=2.500&end=3.500) | 01 | 2.500–3.500 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [003](viewer.html?id=gait_20260909_151135_f45a816f4062&t=3.500&end=4.500) | 01 | 3.500–4.500 | ledge_candidate / 等待候选 | unknown | uncertain |
| [004](viewer.html?id=gait_20260909_151135_f45a816f4062&t=4.500&end=7.500) | 01 | 4.500–7.500 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [005](viewer.html?id=gait_20260909_151135_f45a816f4062&t=7.500&end=9.500) | 01 | 7.500–9.500 | ledge_candidate / 等待候选 | unknown | uncertain |
| [006](viewer.html?id=gait_20260909_151135_f45a816f4062&t=9.500&end=10.500) | 01 | 9.500–10.500 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [007](viewer.html?id=gait_20260909_151135_f45a816f4062&t=10.500&end=11.500) | 01 | 10.500–11.500 | ledge_candidate / 等待候选 | unknown | uncertain |
| [008](viewer.html?id=gait_20260909_151135_f45a816f4062&t=11.500&end=12.500) | 01 | 11.500–12.500 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [009](viewer.html?id=gait_20260909_151135_f45a816f4062&t=12.500&end=13.000) | 01 | 12.500–13.000 | ledge_candidate / 上台候选 | up_candidate | uncertain |
| [010](viewer.html?id=gait_20260909_151135_f45a816f4062&t=13.000&end=13.500) | 01 | 13.000–13.500 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [011](viewer.html?id=gait_20260909_151135_f45a816f4062&t=13.500&end=16.500) | 01 | 13.500–16.500 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [012](viewer.html?id=gait_20260909_151135_f45a816f4062&t=16.500&end=18.000) | 01 | 16.500–18.000 | ledge_candidate / 等待候选 | unknown | uncertain |
| [013](viewer.html?id=gait_20260909_151135_f45a816f4062&t=18.000&end=18.500) | 01 | 18.000–18.500 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [014](viewer.html?id=gait_20260909_151135_f45a816f4062&t=18.500&end=19.000) | 01 | 18.500–19.000 | ledge_candidate / 转向候选 | unknown | uncertain |
| [015](viewer.html?id=gait_20260909_151135_f45a816f4062&t=19.000&end=20.000) | 01 | 19.000–20.000 | ledge_candidate / 上台候选 | up_candidate | uncertain |
| [016](viewer.html?id=gait_20260909_151135_f45a816f4062&t=20.000&end=21.500) | 01 | 20.000–21.500 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [017](viewer.html?id=gait_20260909_151135_f45a816f4062&t=21.500&end=22.000) | 01 | 21.500–22.000 | ledge_candidate / 上台候选 | up_candidate | uncertain |
| [018](viewer.html?id=gait_20260909_151135_f45a816f4062&t=22.000&end=23.500) | 01 | 22.000–23.500 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [019](viewer.html?id=gait_20260909_151135_f45a816f4062&t=23.500&end=26.319) | 01 | 23.500–26.319 | ledge_candidate / 等待候选 | unknown | uncertain |

待确认：
- [11.00–15.00s](viewer.html?id=gait_20260909_151135_f45a816f4062&t=11.000&end=15.000)：实际上台接触、完成区间在哪里？
- [18.00–26.32s](viewer.html?id=gait_20260909_151135_f45a816f4062&t=18.000&end=26.319)：后续是在台面行走/转向，还是又接触其他地形？

## gait_20260909_151220_d3ace6be729a

[同步预览](viewer.html?id=gait_20260909_151220_d3ace6be729a&t=0.000) · [信号](previews/gait_20260909_151220_d3ace6be729a_signals.png) · [点云](previews/gait_20260909_151220_d3ace6be729a_clouds.jpg)

原始：`D:\S10Data\050\2026-09-09\gait_20260909_151220_d3ace6be729a`；时长 13.335593s；标签 `ledge/success`，参数 `{'height_cm': 38}`；人工events为空。

候选通行/上下文：01：0.000–13.336s，candidate_continuous_passage，uncertain

实际控制（源秒）：0.017–5.257：state 17 / gait 0x1001；5.257–13.307：state 17 / gait 0x1002

观察：前点云初始有台沿；约8.8–10s负pitch达约−42°，有腿轮活动，10.6s后前点云趋平而机身仍倾斜。截断前是否四轮完成越障无法仅凭速览确认。

数据问题：/GAIT 源时间缺失/非递增；metadata.files 消息数 12384 与数据库实际 6192 不一致；按实际表计数

| 片段 | 通行组 | 源秒 | 地形/动作候选 | 上下方向 | 子结果 |
|---|---|---|---|---|---|
| [001](viewer.html?id=gait_20260909_151220_d3ace6be729a&t=0.000&end=1.500) | 01 | 0.000–1.500 | ledge_candidate / 等待候选 | unknown | uncertain |
| [002](viewer.html?id=gait_20260909_151220_d3ace6be729a&t=1.500&end=3.000) | 01 | 1.500–3.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [003](viewer.html?id=gait_20260909_151220_d3ace6be729a&t=3.000&end=4.000) | 01 | 3.000–4.000 | ledge_candidate / 转向候选 | unknown | uncertain |
| [004](viewer.html?id=gait_20260909_151220_d3ace6be729a&t=4.000&end=8.500) | 01 | 4.000–8.500 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [005](viewer.html?id=gait_20260909_151220_d3ace6be729a&t=8.500&end=9.000) | 01 | 8.500–9.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [006](viewer.html?id=gait_20260909_151220_d3ace6be729a&t=9.000&end=9.500) | 01 | 9.000–9.500 | ledge_candidate / 上台候选 | up_candidate | uncertain |
| [007](viewer.html?id=gait_20260909_151220_d3ace6be729a&t=9.500&end=10.000) | 01 | 9.500–10.000 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [008](viewer.html?id=gait_20260909_151220_d3ace6be729a&t=10.000&end=10.500) | 01 | 10.000–10.500 | ledge_candidate / 行走/调整候选 | unknown | uncertain |
| [009](viewer.html?id=gait_20260909_151220_d3ace6be729a&t=10.500&end=11.000) | 01 | 10.500–11.000 | ledge_candidate / 上台候选 | up_candidate | uncertain |
| [010](viewer.html?id=gait_20260909_151220_d3ace6be729a&t=11.000&end=12.500) | 01 | 11.000–12.500 | ledge_candidate / 上台候选 | up_candidate | uncertain |
| [011](viewer.html?id=gait_20260909_151220_d3ace6be729a&t=12.500&end=13.336) | 01 | 12.500–13.336 | ledge_candidate / 等待候选 | unknown | uncertain |

待确认：
- [8.00–13.34s](viewer.html?id=gait_20260909_151220_d3ace6be729a&t=8.000&end=13.336)：末次上台是否完成，还是在台沿/倾斜姿态下结束录制？是否与151135同一38cm高台？

## gait_20260909_151538_271bf7a9d4d2

[同步预览](viewer.html?id=gait_20260909_151538_271bf7a9d4d2&t=0.000) · [信号](previews/gait_20260909_151538_271bf7a9d4d2_signals.png) · [点云](previews/gait_20260909_151538_271bf7a9d4d2_clouds.jpg)

原始：`D:\S10Data\050\2026-09-09\gait_20260909_151538_271bf7a9d4d2`；时长 24.862878s；标签 `basic/failure`，参数 `{'height_cm': 38}`；人工events为空。

候选通行/上下文：01：0.000–24.863s，stationary_context，uncertain

实际控制（源秒）：0.034–24.834：state 17 / gait 0x1001

观察：完整信号图中roll/pitch近恒定、关节速度仅微小噪声，点云近地面形态基本稳定；实际状态17/步态0x1001。manifest为failure，但无可定位失败动作证据。

数据问题：metadata.files 消息数 22378 与数据库实际 11189 不一致；按实际表计数

| 片段 | 通行组 | 源秒 | 地形/动作候选 | 上下方向 | 子结果 |
|---|---|---|---|---|---|
| [001](viewer.html?id=gait_20260909_151538_271bf7a9d4d2&t=0.000&end=24.863) | 01 | 0.000–24.863 | flat_candidate / 等待候选 | unknown | uncertain |

待确认：
- [0.00–24.86s](viewer.html?id=gait_20260909_151538_271bf7a9d4d2&t=0.000&end=24.863)：全段近静止：failure标记原因是什么？是否只是无动作/未开始？

## gait_20260909_151616_30765a77bc55

[同步预览](viewer.html?id=gait_20260909_151616_30765a77bc55&t=0.000) · [信号](previews/gait_20260909_151616_30765a77bc55_signals.png) · [点云](previews/gait_20260909_151616_30765a77bc55_clouds.jpg)

原始：`D:\S10Data\050\2026-09-09\gait_20260909_151616_30765a77bc55`；时长 56.934772s；标签 `basic/failure`，参数 `{}`；人工events为空。

候选通行/上下文：01：0.000–56.935s，candidate_continuous_passage，uncertain

实际控制（源秒）：0.012–56.912：state 17 / gait 0x1001

观察：速览以近水平地面为主；1–24s与37–54s轮式移动，24–37s明显转向/调整；roll/pitch幅值较小，状态始终17。没有从这些信号定位到manifest failure的明确发生时刻。

数据问题：/HANDLE_STEER receive-source 中位数 -0.423s；不得按源时间直接与机身话题混合；/STEER receive-source 中位数 -0.424s；不得按源时间直接与机身话题混合；metadata.files 消息数 53702 与数据库实际 26851 不一致；按实际表计数

| 片段 | 通行组 | 源秒 | 地形/动作候选 | 上下方向 | 子结果 |
|---|---|---|---|---|---|
| [001](viewer.html?id=gait_20260909_151616_30765a77bc55&t=0.000&end=1.000) | 01 | 0.000–1.000 | flat_candidate / 等待候选 | unknown | uncertain |
| [002](viewer.html?id=gait_20260909_151616_30765a77bc55&t=1.000&end=3.500) | 01 | 1.000–3.500 | flat_candidate / 行走/调整候选 | unknown | uncertain |
| [003](viewer.html?id=gait_20260909_151616_30765a77bc55&t=3.500&end=4.500) | 01 | 3.500–4.500 | flat_candidate / 转向候选 | unknown | uncertain |
| [004](viewer.html?id=gait_20260909_151616_30765a77bc55&t=4.500&end=9.500) | 01 | 4.500–9.500 | flat_candidate / 行走/调整候选 | unknown | uncertain |
| [005](viewer.html?id=gait_20260909_151616_30765a77bc55&t=9.500&end=11.000) | 01 | 9.500–11.000 | flat_candidate / 转向候选 | unknown | uncertain |
| [006](viewer.html?id=gait_20260909_151616_30765a77bc55&t=11.000&end=12.000) | 01 | 11.000–12.000 | flat_candidate / 行走/调整候选 | unknown | uncertain |
| [007](viewer.html?id=gait_20260909_151616_30765a77bc55&t=12.000&end=16.500) | 01 | 12.000–16.500 | flat_candidate / 转向候选 | unknown | uncertain |
| [008](viewer.html?id=gait_20260909_151616_30765a77bc55&t=16.500&end=17.500) | 01 | 16.500–17.500 | flat_candidate / 行走/调整候选 | unknown | uncertain |
| [009](viewer.html?id=gait_20260909_151616_30765a77bc55&t=17.500&end=30.500) | 01 | 17.500–30.500 | flat_candidate / 转向候选 | unknown | uncertain |
| [010](viewer.html?id=gait_20260909_151616_30765a77bc55&t=30.500&end=32.000) | 01 | 30.500–32.000 | flat_candidate / 行走/调整候选 | unknown | uncertain |
| [011](viewer.html?id=gait_20260909_151616_30765a77bc55&t=32.000&end=33.500) | 01 | 32.000–33.500 | flat_candidate / 转向候选 | unknown | uncertain |
| [012](viewer.html?id=gait_20260909_151616_30765a77bc55&t=33.500&end=35.000) | 01 | 33.500–35.000 | flat_candidate / 行走/调整候选 | unknown | uncertain |
| [013](viewer.html?id=gait_20260909_151616_30765a77bc55&t=35.000&end=36.500) | 01 | 35.000–36.500 | flat_candidate / 转向候选 | unknown | uncertain |
| [014](viewer.html?id=gait_20260909_151616_30765a77bc55&t=36.500&end=42.000) | 01 | 36.500–42.000 | flat_candidate / 行走/调整候选 | unknown | uncertain |
| [015](viewer.html?id=gait_20260909_151616_30765a77bc55&t=42.000&end=43.000) | 01 | 42.000–43.000 | flat_candidate / 转向候选 | unknown | uncertain |
| [016](viewer.html?id=gait_20260909_151616_30765a77bc55&t=43.000&end=53.000) | 01 | 43.000–53.000 | flat_candidate / 行走/调整候选 | unknown | uncertain |
| [017](viewer.html?id=gait_20260909_151616_30765a77bc55&t=53.000&end=56.935) | 01 | 53.000–56.935 | flat_candidate / 等待候选 | unknown | uncertain |

待确认：
- [23.00–38.00s](viewer.html?id=gait_20260909_151616_30765a77bc55&t=23.000&end=38.000)：该转向区是常规动作还是失败/重试节点？
- [45.00–56.93s](viewer.html?id=gait_20260909_151616_30765a77bc55&t=45.000&end=56.935)：failure发生在哪个具体动作？末尾近静止是完成还是停止？

## gait_20260909_151734_72594dc0ad20

[同步预览](viewer.html?id=gait_20260909_151734_72594dc0ad20&t=0.000) · [信号](previews/gait_20260909_151734_72594dc0ad20_signals.png) · [点云](previews/gait_20260909_151734_72594dc0ad20_clouds.jpg)

原始：`D:\S10Data\050\2026-09-09\gait_20260909_151734_72594dc0ad20`；时长 58.366903s；标签 `basic/failure`，参数 `{}`；人工events为空。

候选通行/上下文：01：0.000–58.367s，candidate_continuous_passage，uncertain

实际控制（源秒）：0.020–58.320：state 17 / gait 0x1001

观察：轮式行走与转向交替，近地面点云并非处处平直，有局部起伏/边缘；roll/pitch多为个位数度，实际状态始终17。不能凭basic标签当作整段平地，也未定位整段failure对应事件。

数据问题：/HANDLE_STEER receive-source 中位数 -0.506s；不得按源时间直接与机身话题混合；/STEER receive-source 中位数 -0.508s；不得按源时间直接与机身话题混合；metadata.files 消息数 55230 与数据库实际 27615 不一致；按实际表计数

| 片段 | 通行组 | 源秒 | 地形/动作候选 | 上下方向 | 子结果 |
|---|---|---|---|---|---|
| [001](viewer.html?id=gait_20260909_151734_72594dc0ad20&t=0.000&end=1.000) | 01 | 0.000–1.000 | unknown / 等待候选 | unknown | uncertain |
| [002](viewer.html?id=gait_20260909_151734_72594dc0ad20&t=1.000&end=8.000) | 01 | 1.000–8.000 | unknown / 行走/调整候选 | unknown | uncertain |
| [003](viewer.html?id=gait_20260909_151734_72594dc0ad20&t=8.000&end=10.000) | 01 | 8.000–10.000 | unknown / 转向候选 | unknown | uncertain |
| [004](viewer.html?id=gait_20260909_151734_72594dc0ad20&t=10.000&end=11.000) | 01 | 10.000–11.000 | unknown / 行走/调整候选 | unknown | uncertain |
| [005](viewer.html?id=gait_20260909_151734_72594dc0ad20&t=11.000&end=13.500) | 01 | 11.000–13.500 | unknown / 转向候选 | unknown | uncertain |
| [006](viewer.html?id=gait_20260909_151734_72594dc0ad20&t=13.500&end=18.500) | 01 | 13.500–18.500 | unknown / 行走/调整候选 | unknown | uncertain |
| [007](viewer.html?id=gait_20260909_151734_72594dc0ad20&t=18.500&end=19.500) | 01 | 18.500–19.500 | unknown / 转向候选 | unknown | uncertain |
| [008](viewer.html?id=gait_20260909_151734_72594dc0ad20&t=19.500&end=20.500) | 01 | 19.500–20.500 | unknown / 行走/调整候选 | unknown | uncertain |
| [009](viewer.html?id=gait_20260909_151734_72594dc0ad20&t=20.500&end=23.500) | 01 | 20.500–23.500 | unknown / 转向候选 | unknown | uncertain |
| [010](viewer.html?id=gait_20260909_151734_72594dc0ad20&t=23.500&end=24.500) | 01 | 23.500–24.500 | unknown / 行走/调整候选 | unknown | uncertain |
| [011](viewer.html?id=gait_20260909_151734_72594dc0ad20&t=24.500&end=37.500) | 01 | 24.500–37.500 | unknown / 转向候选 | unknown | uncertain |
| [012](viewer.html?id=gait_20260909_151734_72594dc0ad20&t=37.500&end=38.500) | 01 | 37.500–38.500 | unknown / 行走/调整候选 | unknown | uncertain |
| [013](viewer.html?id=gait_20260909_151734_72594dc0ad20&t=38.500&end=39.500) | 01 | 38.500–39.500 | unknown / 转向候选 | unknown | uncertain |
| [014](viewer.html?id=gait_20260909_151734_72594dc0ad20&t=39.500&end=46.500) | 01 | 39.500–46.500 | unknown / 行走/调整候选 | unknown | uncertain |
| [015](viewer.html?id=gait_20260909_151734_72594dc0ad20&t=46.500&end=50.500) | 01 | 46.500–50.500 | unknown / 转向候选 | unknown | uncertain |
| [016](viewer.html?id=gait_20260909_151734_72594dc0ad20&t=50.500&end=56.500) | 01 | 50.500–56.500 | unknown / 行走/调整候选 | unknown | uncertain |
| [017](viewer.html?id=gait_20260909_151734_72594dc0ad20&t=56.500&end=58.367) | 01 | 56.500–58.367 | unknown / 等待候选 | unknown | uncertain |

待确认：
- [11.00–15.00s](viewer.html?id=gait_20260909_151734_72594dc0ad20&t=11.000&end=15.000)：此处是转向还是越过局部起伏/边缘？
- [22.00–39.00s](viewer.html?id=gait_20260909_151734_72594dc0ad20&t=22.000&end=39.000)：连续转向区是否包含失败/重试？
- [47.00–58.37s](viewer.html?id=gait_20260909_151734_72594dc0ad20&t=47.000&end=58.367)：末段failure的具体位置及原因？

## gait_20260909_151931_7a0441190ea8

[同步预览](viewer.html?id=gait_20260909_151931_7a0441190ea8&t=0.000) · [信号](previews/gait_20260909_151931_7a0441190ea8_signals.png) · [点云](previews/gait_20260909_151931_7a0441190ea8_clouds.jpg)

原始：`D:\S10Data\050\2026-09-09\gait_20260909_151931_7a0441190ea8`；时长 120.004829s；标签 `basic/unlabeled`，参数 `{}`；人工events为空。

候选通行/上下文：01：0.000–120.005s，candidate_continuous_passage，uncertain

实际控制（源秒）：-0.001–119.954：state 17 / gait 0x1001

观察：近地面点云以平缓面为主；1–7、24–44、62–90、94s后有移动，7.5–23.5与44.5–62s长等待；尾段有周期关节活动/转向，状态始终17。停顿不自动切成新通行。

数据问题：/HANDLE_STEER receive-source 中位数 -0.665s；不得按源时间直接与机身话题混合；/STEER receive-source 中位数 -0.665s；不得按源时间直接与机身话题混合；metadata.files 消息数 111932 与数据库实际 55966 不一致；按实际表计数

| 片段 | 通行组 | 源秒 | 地形/动作候选 | 上下方向 | 子结果 |
|---|---|---|---|---|---|
| [001](viewer.html?id=gait_20260909_151931_7a0441190ea8&t=0.000&end=1.000) | 01 | 0.000–1.000 | flat_candidate / 等待候选 | unknown | uncertain |
| [002](viewer.html?id=gait_20260909_151931_7a0441190ea8&t=1.000&end=4.500) | 01 | 1.000–4.500 | flat_candidate / 行走/调整候选 | unknown | uncertain |
| [003](viewer.html?id=gait_20260909_151931_7a0441190ea8&t=4.500&end=6.500) | 01 | 4.500–6.500 | flat_candidate / 转向候选 | unknown | uncertain |
| [004](viewer.html?id=gait_20260909_151931_7a0441190ea8&t=6.500&end=7.500) | 01 | 6.500–7.500 | flat_candidate / 行走/调整候选 | unknown | uncertain |
| [005](viewer.html?id=gait_20260909_151931_7a0441190ea8&t=7.500&end=23.500) | 01 | 7.500–23.500 | flat_candidate / 等待候选 | unknown | uncertain |
| [006](viewer.html?id=gait_20260909_151931_7a0441190ea8&t=23.500&end=44.000) | 01 | 23.500–44.000 | flat_candidate / 行走/调整候选 | unknown | uncertain |
| [007](viewer.html?id=gait_20260909_151931_7a0441190ea8&t=44.000&end=44.500) | 01 | 44.000–44.500 | flat_candidate / 等待候选 | unknown | uncertain |
| [008](viewer.html?id=gait_20260909_151931_7a0441190ea8&t=44.500&end=62.000) | 01 | 44.500–62.000 | flat_candidate / 等待候选 | unknown | uncertain |
| [009](viewer.html?id=gait_20260909_151931_7a0441190ea8&t=62.000&end=63.000) | 01 | 62.000–63.000 | flat_candidate / 行走/调整候选 | unknown | uncertain |
| [010](viewer.html?id=gait_20260909_151931_7a0441190ea8&t=63.000&end=65.000) | 01 | 63.000–65.000 | flat_candidate / 转向候选 | unknown | uncertain |
| [011](viewer.html?id=gait_20260909_151931_7a0441190ea8&t=65.000&end=82.000) | 01 | 65.000–82.000 | flat_candidate / 行走/调整候选 | unknown | uncertain |
| [012](viewer.html?id=gait_20260909_151931_7a0441190ea8&t=82.000&end=83.500) | 01 | 82.000–83.500 | flat_candidate / 转向候选 | unknown | uncertain |
| [013](viewer.html?id=gait_20260909_151931_7a0441190ea8&t=83.500&end=90.500) | 01 | 83.500–90.500 | flat_candidate / 行走/调整候选 | unknown | uncertain |
| [014](viewer.html?id=gait_20260909_151931_7a0441190ea8&t=90.500&end=91.000) | 01 | 90.500–91.000 | flat_candidate / 行走/调整候选 | unknown | uncertain |
| [015](viewer.html?id=gait_20260909_151931_7a0441190ea8&t=91.000&end=93.500) | 01 | 91.000–93.500 | flat_candidate / 等待候选 | unknown | uncertain |
| [016](viewer.html?id=gait_20260909_151931_7a0441190ea8&t=93.500&end=94.000) | 01 | 93.500–94.000 | flat_candidate / 行走/调整候选 | unknown | uncertain |
| [017](viewer.html?id=gait_20260909_151931_7a0441190ea8&t=94.000&end=95.500) | 01 | 94.000–95.500 | flat_candidate / 转向候选 | unknown | uncertain |
| [018](viewer.html?id=gait_20260909_151931_7a0441190ea8&t=95.500&end=96.500) | 01 | 95.500–96.500 | flat_candidate / 行走/调整候选 | unknown | uncertain |
| [019](viewer.html?id=gait_20260909_151931_7a0441190ea8&t=96.500&end=98.000) | 01 | 96.500–98.000 | flat_candidate / 转向候选 | unknown | uncertain |
| [020](viewer.html?id=gait_20260909_151931_7a0441190ea8&t=98.000&end=99.000) | 01 | 98.000–99.000 | flat_candidate / 行走/调整候选 | unknown | uncertain |
| [021](viewer.html?id=gait_20260909_151931_7a0441190ea8&t=99.000&end=112.000) | 01 | 99.000–112.000 | flat_candidate / 转向候选 | unknown | uncertain |
| [022](viewer.html?id=gait_20260909_151931_7a0441190ea8&t=112.000&end=113.000) | 01 | 112.000–113.000 | flat_candidate / 行走/调整候选 | unknown | uncertain |
| [023](viewer.html?id=gait_20260909_151931_7a0441190ea8&t=113.000&end=117.000) | 01 | 113.000–117.000 | flat_candidate / 转向候选 | unknown | uncertain |
| [024](viewer.html?id=gait_20260909_151931_7a0441190ea8&t=117.000&end=119.500) | 01 | 117.000–119.500 | flat_candidate / 行走/调整候选 | unknown | uncertain |
| [025](viewer.html?id=gait_20260909_151931_7a0441190ea8&t=119.500&end=120.000) | 01 | 119.500–120.000 | flat_candidate / 转向候选 | unknown | uncertain |
| [026](viewer.html?id=gait_20260909_151931_7a0441190ea8&t=120.000&end=120.005) | 01 | 120.000–120.005 | unknown / 数据中断/不可判断 | unknown | interrupted_data |

待确认：
- [6.00–25.00s](viewer.html?id=gait_20260909_151931_7a0441190ea8&t=6.000&end=25.000)：长等待前后是否为独立测试通行？
- [43.00–64.00s](viewer.html?id=gait_20260909_151931_7a0441190ea8&t=43.000&end=64.000)：第二次长等待是否结束上一通行？
- [94.00–120.00s](viewer.html?id=gait_20260909_151931_7a0441190ea8&t=94.000&end=120.005)：尾段动作是什么，是否完成？

## gait_20260909_152145_fc06025e3eb8

[同步预览](viewer.html?id=gait_20260909_152145_fc06025e3eb8&t=0.000) · [信号](previews/gait_20260909_152145_fc06025e3eb8_signals.png) · [点云](previews/gait_20260909_152145_fc06025e3eb8_clouds.jpg)

原始：`D:\S10Data\050\2026-09-09\gait_20260909_152145_fc06025e3eb8`；时长 69.699572s；标签 `basic/failure`，参数 `{}`；人工events为空。

候选通行/上下文：01：0.000–69.700s，candidate_continuous_passage，uncertain

实际控制（源秒）：-0.102–69.648：state 17 / gait 0x1001

观察：比其他basic段有更明显roll/pitch起伏：约4–7.5、11.5–13.5、28–31.5、40–43s；有轮关节活动，点云局部起伏/边缘但不足以确认为楼梯或高台。状态始终17；61–65s近静止后又移动。

数据问题：/HANDLE_STEER receive-source 中位数 -0.767s；不得按源时间直接与机身话题混合；/STEER receive-source 中位数 -0.767s；不得按源时间直接与机身话题混合；metadata.files 消息数 65944 与数据库实际 32972 不一致；按实际表计数

| 片段 | 通行组 | 源秒 | 地形/动作候选 | 上下方向 | 子结果 |
|---|---|---|---|---|---|
| [001](viewer.html?id=gait_20260909_152145_fc06025e3eb8&t=0.000&end=2.500) | 01 | 0.000–2.500 | unknown / 等待候选 | unknown | uncertain |
| [002](viewer.html?id=gait_20260909_152145_fc06025e3eb8&t=2.500&end=3.000) | 01 | 2.500–3.000 | unknown / 行走/调整候选 | unknown | uncertain |
| [003](viewer.html?id=gait_20260909_152145_fc06025e3eb8&t=3.000&end=4.000) | 01 | 3.000–4.000 | unknown / 转向候选 | unknown | uncertain |
| [004](viewer.html?id=gait_20260909_152145_fc06025e3eb8&t=4.000&end=4.500) | 01 | 4.000–4.500 | unknown / 行走/调整候选 | unknown | uncertain |
| [005](viewer.html?id=gait_20260909_152145_fc06025e3eb8&t=4.500&end=6.000) | 01 | 4.500–6.000 | unknown / 姿态变化/越障候选 | unknown | uncertain |
| [006](viewer.html?id=gait_20260909_152145_fc06025e3eb8&t=6.000&end=6.500) | 01 | 6.000–6.500 | unknown / 转向候选 | unknown | uncertain |
| [007](viewer.html?id=gait_20260909_152145_fc06025e3eb8&t=6.500&end=7.500) | 01 | 6.500–7.500 | unknown / 行走/调整候选 | unknown | uncertain |
| [008](viewer.html?id=gait_20260909_152145_fc06025e3eb8&t=7.500&end=9.500) | 01 | 7.500–9.500 | unknown / 行走/调整候选 | unknown | uncertain |
| [009](viewer.html?id=gait_20260909_152145_fc06025e3eb8&t=9.500&end=11.500) | 01 | 9.500–11.500 | unknown / 转向候选 | unknown | uncertain |
| [010](viewer.html?id=gait_20260909_152145_fc06025e3eb8&t=11.500&end=12.500) | 01 | 11.500–12.500 | unknown / 姿态变化/越障候选 | unknown | uncertain |
| [011](viewer.html?id=gait_20260909_152145_fc06025e3eb8&t=12.500&end=13.500) | 01 | 12.500–13.500 | unknown / 行走/调整候选 | unknown | uncertain |
| [012](viewer.html?id=gait_20260909_152145_fc06025e3eb8&t=13.500&end=17.000) | 01 | 13.500–17.000 | unknown / 转向候选 | unknown | uncertain |
| [013](viewer.html?id=gait_20260909_152145_fc06025e3eb8&t=17.000&end=18.000) | 01 | 17.000–18.000 | unknown / 行走/调整候选 | unknown | uncertain |
| [014](viewer.html?id=gait_20260909_152145_fc06025e3eb8&t=18.000&end=22.000) | 01 | 18.000–22.000 | unknown / 转向候选 | unknown | uncertain |
| [015](viewer.html?id=gait_20260909_152145_fc06025e3eb8&t=22.000&end=23.500) | 01 | 22.000–23.500 | unknown / 行走/调整候选 | unknown | uncertain |
| [016](viewer.html?id=gait_20260909_152145_fc06025e3eb8&t=23.500&end=27.500) | 01 | 23.500–27.500 | unknown / 转向候选 | unknown | uncertain |
| [017](viewer.html?id=gait_20260909_152145_fc06025e3eb8&t=27.500&end=28.000) | 01 | 27.500–28.000 | unknown / 姿态变化/越障候选 | unknown | uncertain |
| [018](viewer.html?id=gait_20260909_152145_fc06025e3eb8&t=28.000&end=30.500) | 01 | 28.000–30.500 | unknown / 转向候选 | unknown | uncertain |
| [019](viewer.html?id=gait_20260909_152145_fc06025e3eb8&t=30.500&end=31.500) | 01 | 30.500–31.500 | unknown / 行走/调整候选 | unknown | uncertain |
| [020](viewer.html?id=gait_20260909_152145_fc06025e3eb8&t=31.500&end=32.500) | 01 | 31.500–32.500 | unknown / 转向候选 | unknown | uncertain |
| [021](viewer.html?id=gait_20260909_152145_fc06025e3eb8&t=32.500&end=34.500) | 01 | 32.500–34.500 | unknown / 姿态变化/越障候选 | unknown | uncertain |
| [022](viewer.html?id=gait_20260909_152145_fc06025e3eb8&t=34.500&end=40.000) | 01 | 34.500–40.000 | unknown / 转向候选 | unknown | uncertain |
| [023](viewer.html?id=gait_20260909_152145_fc06025e3eb8&t=40.000&end=41.500) | 01 | 40.000–41.500 | unknown / 转向候选 | unknown | uncertain |
| [024](viewer.html?id=gait_20260909_152145_fc06025e3eb8&t=41.500&end=42.000) | 01 | 41.500–42.000 | unknown / 姿态变化/越障候选 | unknown | uncertain |
| [025](viewer.html?id=gait_20260909_152145_fc06025e3eb8&t=42.000&end=43.000) | 01 | 42.000–43.000 | unknown / 转向候选 | unknown | uncertain |
| [026](viewer.html?id=gait_20260909_152145_fc06025e3eb8&t=43.000&end=54.500) | 01 | 43.000–54.500 | unknown / 转向候选 | unknown | uncertain |
| [027](viewer.html?id=gait_20260909_152145_fc06025e3eb8&t=54.500&end=55.500) | 01 | 54.500–55.500 | unknown / 行走/调整候选 | unknown | uncertain |
| [028](viewer.html?id=gait_20260909_152145_fc06025e3eb8&t=55.500&end=61.000) | 01 | 55.500–61.000 | unknown / 转向候选 | unknown | uncertain |
| [029](viewer.html?id=gait_20260909_152145_fc06025e3eb8&t=61.000&end=61.500) | 01 | 61.000–61.500 | unknown / 行走/调整候选 | unknown | uncertain |
| [030](viewer.html?id=gait_20260909_152145_fc06025e3eb8&t=61.500&end=65.000) | 01 | 61.500–65.000 | unknown / 等待候选 | unknown | uncertain |
| [031](viewer.html?id=gait_20260909_152145_fc06025e3eb8&t=65.000&end=65.500) | 01 | 65.000–65.500 | unknown / 行走/调整候选 | unknown | uncertain |
| [032](viewer.html?id=gait_20260909_152145_fc06025e3eb8&t=65.500&end=66.500) | 01 | 65.500–66.500 | unknown / 转向候选 | unknown | uncertain |
| [033](viewer.html?id=gait_20260909_152145_fc06025e3eb8&t=66.500&end=69.500) | 01 | 66.500–69.500 | unknown / 行走/调整候选 | unknown | uncertain |
| [034](viewer.html?id=gait_20260909_152145_fc06025e3eb8&t=69.500&end=69.700) | 01 | 69.500–69.700 | unknown / 等待候选 | unknown | uncertain |

待确认：
- [4.00–14.00s](viewer.html?id=gait_20260909_152145_fc06025e3eb8&t=4.000&end=14.000)：前两次姿态起伏经过什么地形，是否其中一次失败？
- [27.00–44.00s](viewer.html?id=gait_20260909_152145_fc06025e3eb8&t=27.000&end=44.000)：中段起伏与多次转向是否属于同一通行？
- [60.00–69.70s](viewer.html?id=gait_20260909_152145_fc06025e3eb8&t=60.000&end=69.700)：停顿后再次移动是恢复/重试还是正常结束？

## gait_20260909_152557_7ddbfc67c108

[同步预览](viewer.html?id=gait_20260909_152557_7ddbfc67c108&t=0.000) · [信号](previews/gait_20260909_152557_7ddbfc67c108_signals.png) · [点云](previews/gait_20260909_152557_7ddbfc67c108_clouds.jpg)

原始：`D:\S10Data\050\2026-09-09\gait_20260909_152557_7ddbfc67c108`；时长 120.004243s；标签 `stairs/unlabeled`，参数 `{}`；人工events为空。

候选通行/上下文：01：0.000–40.396s，candidate_continuous_passage，interrupted_control；02：40.396–120.004s，post_interruption_context，uncertain

实际控制（源秒）：0.014–40.396：state 17 / gait 0x1001；40.396–42.398：state 2 / gait 0x0000；42.398–119.998：state 0 / gait 0x0000

观察：约11s后pitch约+15°，15/25s前点云近处有阶梯纹理、后方地面倾斜，35s后点云见阶梯；轮关节活动与正pitch支持下楼候选，但实际步态一直0x1001。
观察：约40s roll升至约70–90°，40.396s状态17→2，42.398s→0；45/55s点云呈明显侧倾，关节活动降至近静止。是否摔倒及人工处置待确认。
观察：控制退出后机身姿态多次变化（约62/79–83/117s），与原地静止状态不同；不能仅凭姿态变化认定搬动。

数据问题：/GAIT 源时间缺失/非递增；/HANDLE_STEER receive-source 中位数 -1.002s；不得按源时间直接与机身话题混合；/JOINTS_DATA 5s 窗源时间采样率范围 50.0–200.0Hz（事件话题不视为掉帧）；/STEER receive-source 中位数 -1.002s；不得按源时间直接与机身话题混合；实际运动状态 2：40.396–42.398s（源时间）；实际运动状态 0：42.398–119.998s（源时间）；metadata.files 消息数 85048 与数据库实际 42524 不一致；按实际表计数

| 片段 | 通行组 | 源秒 | 地形/动作候选 | 上下方向 | 子结果 |
|---|---|---|---|---|---|
| [001](viewer.html?id=gait_20260909_152557_7ddbfc67c108&t=0.000&end=8.000) | 01 | 0.000–8.000 | unknown / 等待候选 | unknown | uncertain |
| [002](viewer.html?id=gait_20260909_152557_7ddbfc67c108&t=8.000&end=8.500) | 01 | 8.000–8.500 | unknown / 行走/调整候选 | unknown | uncertain |
| [003](viewer.html?id=gait_20260909_152557_7ddbfc67c108&t=8.500&end=11.000) | 01 | 8.500–11.000 | stairs_candidate / 行走/调整候选 | unknown | uncertain |
| [004](viewer.html?id=gait_20260909_152557_7ddbfc67c108&t=11.000&end=12.500) | 01 | 11.000–12.500 | stairs_candidate / 下楼候选 | down_candidate | uncertain |
| [005](viewer.html?id=gait_20260909_152557_7ddbfc67c108&t=12.500&end=18.500) | 01 | 12.500–18.500 | stairs_candidate / 等待候选 | unknown | uncertain |
| [006](viewer.html?id=gait_20260909_152557_7ddbfc67c108&t=18.500&end=19.000) | 01 | 18.500–19.000 | stairs_candidate / 下楼候选 | down_candidate | uncertain |
| [007](viewer.html?id=gait_20260909_152557_7ddbfc67c108&t=19.000&end=28.500) | 01 | 19.000–28.500 | stairs_candidate / 下楼候选 | down_candidate | uncertain |
| [008](viewer.html?id=gait_20260909_152557_7ddbfc67c108&t=28.500&end=30.000) | 01 | 28.500–30.000 | stairs_candidate / 等待候选 | unknown | uncertain |
| [009](viewer.html?id=gait_20260909_152557_7ddbfc67c108&t=30.000&end=39.500) | 01 | 30.000–39.500 | stairs_candidate / 下楼候选 | down_candidate | uncertain |
| [010](viewer.html?id=gait_20260909_152557_7ddbfc67c108&t=39.500&end=40.000) | 01 | 39.500–40.000 | unknown / 姿态变化/越障候选 | unknown | uncertain |
| [011](viewer.html?id=gait_20260909_152557_7ddbfc67c108&t=40.000&end=40.396) | 01 | 40.000–40.396 | unknown / 控制退出后的记录 | unknown | interrupted_control |
| [012](viewer.html?id=gait_20260909_152557_7ddbfc67c108&t=40.396&end=43.000) | 02 | 40.396–43.000 | unknown / 控制退出后的记录 | unknown | interrupted_control |
| [013](viewer.html?id=gait_20260909_152557_7ddbfc67c108&t=43.000&end=56.000) | 02 | 43.000–56.000 | unknown / 控制退出后的记录 | unknown | uncertain |
| [014](viewer.html?id=gait_20260909_152557_7ddbfc67c108&t=56.000&end=62.000) | 02 | 56.000–62.000 | unknown / 控制退出后的记录 | unknown | uncertain |
| [015](viewer.html?id=gait_20260909_152557_7ddbfc67c108&t=62.000&end=79.000) | 02 | 62.000–79.000 | unknown / 控制退出后的记录 | unknown | uncertain |
| [016](viewer.html?id=gait_20260909_152557_7ddbfc67c108&t=79.000&end=83.000) | 02 | 79.000–83.000 | unknown / 控制退出后的记录 | unknown | uncertain |
| [017](viewer.html?id=gait_20260909_152557_7ddbfc67c108&t=83.000&end=117.000) | 02 | 83.000–117.000 | unknown / 控制退出后的记录 | unknown | uncertain |
| [018](viewer.html?id=gait_20260909_152557_7ddbfc67c108&t=117.000&end=120.000) | 02 | 117.000–120.000 | unknown / 控制退出后的记录 | unknown | uncertain |
| [019](viewer.html?id=gait_20260909_152557_7ddbfc67c108&t=120.000&end=120.004) | 02 | 120.000–120.004 | unknown / 数据中断/不可判断 | unknown | interrupted_data |

待确认：
- [8.00–20.00s](viewer.html?id=gait_20260909_152557_7ddbfc67c108&t=8.000&end=20.000)：是否基础步态下楼，11–19s停顿在何处？
- [30.00–43.00s](viewer.html?id=gait_20260909_152557_7ddbfc67c108&t=30.000&end=43.000)：40s附近是否摔倒/人工急停，通行应判失败还是中断？
- [55.00–64.00s](viewer.html?id=gait_20260909_152557_7ddbfc67c108&t=55.000&end=64.000)：是否首次人工扶正/搬动？
- [78.00–84.00s](viewer.html?id=gait_20260909_152557_7ddbfc67c108&t=78.000&end=84.000)：第二次姿态变化是否人工干预？
- [116.00–120.00s](viewer.html?id=gait_20260909_152557_7ddbfc67c108&t=116.000&end=120.004)：末尾姿态变化是什么动作？

## gait_20260909_154654_65c5ae680a24

[同步预览](viewer.html?id=gait_20260909_154654_65c5ae680a24&t=0.000) · [信号](previews/gait_20260909_154654_65c5ae680a24_signals.png) · [点云](previews/gait_20260909_154654_65c5ae680a24_clouds.jpg)

原始：`D:\S10Data\050\2026-09-09\gait_20260909_154654_65c5ae680a24`；时长 111.551425s；标签 `stairs/unlabeled`，参数 `{}`；人工events为空。

候选通行/上下文：01：0.000–52.891s，candidate_continuous_passage，interrupted_control；02：52.891–107.606s，post_interruption_context，uncertain；03：107.606–111.551s，candidate_continuous_passage，uncertain

实际控制（源秒）：-0.008–7.932：state 17 / gait 0x1001；7.932–52.891：state 17 / gait 0x1003；52.891–54.491：state 4 / gait 0x0000；54.491–106.005：state 0 / gait 0x0000；106.005–107.606：state 1 / gait 0x0000；107.606–107.607：state 17 / gait 0x0000；107.607–111.507：state 17 / gait 0x1001

观察：4.6s前点云有连续阶梯；12–34s负pitch与腿关节活动，多处阶梯形点云，24–26.5s仍保持倾斜但活动降低，不能把这处停顿硬标平台。
观察：34–38s正pitch短段，之后接近平姿且多次调整，前方仍有边缘/阶梯结构；52.891s退出到状态4，54.491s到0。是否到平台、放弃或人工停下尚不确定。
观察：长段状态0期间仍有姿态和少量关节变化；约62–67s、73–86s、99–106s需核对是否人工处理；106.005s状态1，107.606s恢复17。

数据问题：/GAIT 源时间缺失/非递增；/HANDLE_STEER receive-source 中位数 -2.251s；不得按源时间直接与机身话题混合；/STEER receive-source 中位数 -2.252s；不得按源时间直接与机身话题混合；实际运动状态 4：52.891–54.491s（源时间）；实际运动状态 0：54.491–106.005s（源时间）；实际运动状态 1：106.005–107.606s（源时间）；metadata.files 消息数 68358 与数据库实际 34179 不一致；按实际表计数

| 片段 | 通行组 | 源秒 | 地形/动作候选 | 上下方向 | 子结果 |
|---|---|---|---|---|---|
| [001](viewer.html?id=gait_20260909_154654_65c5ae680a24&t=0.000&end=1.500) | 01 | 0.000–1.500 | unknown / 等待候选 | unknown | uncertain |
| [002](viewer.html?id=gait_20260909_154654_65c5ae680a24&t=1.500&end=4.000) | 01 | 1.500–4.000 | unknown / 行走/调整候选 | unknown | uncertain |
| [003](viewer.html?id=gait_20260909_154654_65c5ae680a24&t=4.000&end=10.500) | 01 | 4.000–10.500 | unknown / 等待候选 | unknown | uncertain |
| [004](viewer.html?id=gait_20260909_154654_65c5ae680a24&t=10.500&end=11.500) | 01 | 10.500–11.500 | unknown / 行走/调整候选 | unknown | uncertain |
| [005](viewer.html?id=gait_20260909_154654_65c5ae680a24&t=11.500&end=12.500) | 01 | 11.500–12.500 | stairs_candidate / 行走/调整候选 | unknown | uncertain |
| [006](viewer.html?id=gait_20260909_154654_65c5ae680a24&t=12.500&end=24.000) | 01 | 12.500–24.000 | stairs_candidate / 上楼候选 | up_candidate | uncertain |
| [007](viewer.html?id=gait_20260909_154654_65c5ae680a24&t=24.000&end=24.500) | 01 | 24.000–24.500 | stairs_candidate / 上楼候选 | up_candidate | uncertain |
| [008](viewer.html?id=gait_20260909_154654_65c5ae680a24&t=24.500&end=25.500) | 01 | 24.500–25.500 | stairs_candidate / 等待候选 | unknown | uncertain |
| [009](viewer.html?id=gait_20260909_154654_65c5ae680a24&t=25.500&end=26.500) | 01 | 25.500–26.500 | stairs_candidate / 上楼候选 | up_candidate | uncertain |
| [010](viewer.html?id=gait_20260909_154654_65c5ae680a24&t=26.500&end=32.500) | 01 | 26.500–32.500 | stairs_candidate / 上楼候选 | up_candidate | uncertain |
| [011](viewer.html?id=gait_20260909_154654_65c5ae680a24&t=32.500&end=33.000) | 01 | 32.500–33.000 | stairs_candidate / 行走/调整候选 | unknown | uncertain |
| [012](viewer.html?id=gait_20260909_154654_65c5ae680a24&t=33.000&end=34.000) | 01 | 33.000–34.000 | stairs_candidate / 转向候选 | unknown | uncertain |
| [013](viewer.html?id=gait_20260909_154654_65c5ae680a24&t=34.000&end=34.500) | 01 | 34.000–34.500 | landing_or_stairs_unknown / 姿态变化/越障候选 | unknown | uncertain |
| [014](viewer.html?id=gait_20260909_154654_65c5ae680a24&t=34.500&end=36.000) | 01 | 34.500–36.000 | landing_or_stairs_unknown / 等待候选 | unknown | uncertain |
| [015](viewer.html?id=gait_20260909_154654_65c5ae680a24&t=36.000&end=36.500) | 01 | 36.000–36.500 | landing_or_stairs_unknown / 行走/调整候选 | unknown | uncertain |
| [016](viewer.html?id=gait_20260909_154654_65c5ae680a24&t=36.500&end=37.000) | 01 | 36.500–37.000 | landing_or_stairs_unknown / 姿态变化/越障候选 | unknown | uncertain |
| [017](viewer.html?id=gait_20260909_154654_65c5ae680a24&t=37.000&end=38.500) | 01 | 37.000–38.500 | landing_or_stairs_unknown / 行走/调整候选 | unknown | uncertain |
| [018](viewer.html?id=gait_20260909_154654_65c5ae680a24&t=38.500&end=39.000) | 01 | 38.500–39.000 | landing_or_stairs_unknown / 行走/调整候选 | unknown | uncertain |
| [019](viewer.html?id=gait_20260909_154654_65c5ae680a24&t=39.000&end=43.500) | 01 | 39.000–43.500 | landing_or_stairs_unknown / 等待候选 | unknown | uncertain |
| [020](viewer.html?id=gait_20260909_154654_65c5ae680a24&t=43.500&end=44.500) | 01 | 43.500–44.500 | landing_or_stairs_unknown / 行走/调整候选 | unknown | uncertain |
| [021](viewer.html?id=gait_20260909_154654_65c5ae680a24&t=44.500&end=45.500) | 01 | 44.500–45.500 | landing_or_stairs_unknown / 等待候选 | unknown | uncertain |
| [022](viewer.html?id=gait_20260909_154654_65c5ae680a24&t=45.500&end=52.500) | 01 | 45.500–52.500 | landing_or_stairs_unknown / 行走/调整候选 | unknown | uncertain |
| [023](viewer.html?id=gait_20260909_154654_65c5ae680a24&t=52.500&end=52.891) | 01 | 52.500–52.891 | unknown / 控制退出后的记录 | unknown | interrupted_control |
| [024](viewer.html?id=gait_20260909_154654_65c5ae680a24&t=52.891&end=54.500) | 02 | 52.891–54.500 | unknown / 控制退出后的记录 | unknown | interrupted_control |
| [025](viewer.html?id=gait_20260909_154654_65c5ae680a24&t=54.500&end=61.500) | 02 | 54.500–61.500 | unknown / 控制退出后的记录 | unknown | uncertain |
| [026](viewer.html?id=gait_20260909_154654_65c5ae680a24&t=61.500&end=67.000) | 02 | 61.500–67.000 | unknown / 控制退出后的记录 | unknown | uncertain |
| [027](viewer.html?id=gait_20260909_154654_65c5ae680a24&t=67.000&end=73.000) | 02 | 67.000–73.000 | unknown / 控制退出后的记录 | unknown | uncertain |
| [028](viewer.html?id=gait_20260909_154654_65c5ae680a24&t=73.000&end=86.000) | 02 | 73.000–86.000 | unknown / 控制退出后的记录 | unknown | uncertain |
| [029](viewer.html?id=gait_20260909_154654_65c5ae680a24&t=86.000&end=99.000) | 02 | 86.000–99.000 | unknown / 控制退出后的记录 | unknown | uncertain |
| [030](viewer.html?id=gait_20260909_154654_65c5ae680a24&t=99.000&end=106.000) | 02 | 99.000–106.000 | unknown / 控制退出后的记录 | unknown | uncertain |
| [031](viewer.html?id=gait_20260909_154654_65c5ae680a24&t=106.000&end=107.500) | 02 | 106.000–107.500 | unknown / 控制退出后的记录 | unknown | interrupted_control |
| [032](viewer.html?id=gait_20260909_154654_65c5ae680a24&t=107.500&end=107.606) | 02 | 107.500–107.606 | unknown / 行走/调整候选 | unknown | uncertain |
| [033](viewer.html?id=gait_20260909_154654_65c5ae680a24&t=107.606&end=109.000) | 03 | 107.606–109.000 | unknown / 行走/调整候选 | unknown | uncertain |
| [034](viewer.html?id=gait_20260909_154654_65c5ae680a24&t=109.000&end=111.551) | 03 | 109.000–111.551 | unknown / 等待候选 | unknown | uncertain |

待确认：
- [23.00–28.00s](viewer.html?id=gait_20260909_154654_65c5ae680a24&t=23.000&end=28.000)：倾斜停顿是在梯段内还是中间平台？
- [33.00–40.00s](viewer.html?id=gait_20260909_154654_65c5ae680a24&t=33.000&end=40.000)：是否已经上到平台/转向，还是仍处于障碍？
- [50.00–56.00s](viewer.html?id=gait_20260909_154654_65c5ae680a24&t=50.000&end=56.000)：退出控制是完成后主动停下、失败，还是人工接管？
- [61.00–68.00s](viewer.html?id=gait_20260909_154654_65c5ae680a24&t=61.000&end=68.000)：该姿态变化是否人工处理？
- [73.00–86.00s](viewer.html?id=gait_20260909_154654_65c5ae680a24&t=73.000&end=86.000)：长倾斜区是否机器人被搬动？
- [99.00–111.55s](viewer.html?id=gait_20260909_154654_65c5ae680a24&t=99.000&end=111.551)：扶正/站立/恢复控制后是否开始新通行？

## gait_20260909_154856_5809460fe5ab

[同步预览](viewer.html?id=gait_20260909_154856_5809460fe5ab&t=0.000) · [信号](previews/gait_20260909_154856_5809460fe5ab_signals.png) · [点云](previews/gait_20260909_154856_5809460fe5ab_clouds.jpg)

原始：`D:\S10Data\050\2026-09-09\gait_20260909_154856_5809460fe5ab`；时长 84.152420s；标签 `stairs/unlabeled`，参数 `{}`；人工events为空。

候选通行/上下文：01：0.000–40.000s，candidate_continuous_passage，uncertain；02：40.000–84.152s，candidate_continuous_passage，uncertain

实际控制（源秒）：0.013–18.313：state 17 / gait 0x1001；18.313–67.973：state 17 / gait 0x1003；67.973–84.123：state 17 / gait 0x1001

观察：初始前点云连续阶梯；20–36.5s负pitch与腿关节活动；25.5–29.5s有倾斜停顿/再调整，仍保留同一上行候选。
观察：36.5–40s姿态趋平且明显转向，38.6s点云近处地面平缓；42s以后pitch转为持续正值。40s提出上行/下行通行候选分界，需确认是否同一楼梯返回。
观察：42–48.5s正pitch约+15–20°、周期轮腿运动，前后点云阶梯结构；下楼候选。
观察：48.5–55s姿态回平、短时轮式行走/转向，52.6s前点云近处较平且远处下降；保留在下行通行内连接两个楼梯活动区。
观察：56–65s再次持续正pitch及周期轮腿运动，59.6/66.6s后点云有阶梯；支持第二下行活动区，未逐级编号。

数据问题：/GAIT 源时间缺失/非递增；/HANDLE_STEER receive-source 中位数 -2.409s；不得按源时间直接与机身话题混合；/STEER receive-source 中位数 -2.413s；不得按源时间直接与机身话题混合；metadata.files 消息数 53008 与数据库实际 26504 不一致；按实际表计数

| 片段 | 通行组 | 源秒 | 地形/动作候选 | 上下方向 | 子结果 |
|---|---|---|---|---|---|
| [001](viewer.html?id=gait_20260909_154856_5809460fe5ab&t=0.000&end=15.000) | 01 | 0.000–15.000 | unknown / 等待候选 | unknown | uncertain |
| [002](viewer.html?id=gait_20260909_154856_5809460fe5ab&t=15.000&end=16.000) | 01 | 15.000–16.000 | unknown / 行走/调整候选 | unknown | uncertain |
| [003](viewer.html?id=gait_20260909_154856_5809460fe5ab&t=16.000&end=19.500) | 01 | 16.000–19.500 | unknown / 等待候选 | unknown | uncertain |
| [004](viewer.html?id=gait_20260909_154856_5809460fe5ab&t=19.500&end=22.000) | 01 | 19.500–22.000 | stairs_candidate / 行走/调整候选 | unknown | uncertain |
| [005](viewer.html?id=gait_20260909_154856_5809460fe5ab&t=22.000&end=25.500) | 01 | 22.000–25.500 | stairs_candidate / 上楼候选 | up_candidate | uncertain |
| [006](viewer.html?id=gait_20260909_154856_5809460fe5ab&t=25.500&end=26.000) | 01 | 25.500–26.000 | stairs_candidate / 上楼候选 | up_candidate | uncertain |
| [007](viewer.html?id=gait_20260909_154856_5809460fe5ab&t=26.000&end=26.500) | 01 | 26.000–26.500 | stairs_candidate / 行走/调整候选 | unknown | uncertain |
| [008](viewer.html?id=gait_20260909_154856_5809460fe5ab&t=26.500&end=27.000) | 01 | 26.500–27.000 | stairs_candidate / 等待候选 | unknown | uncertain |
| [009](viewer.html?id=gait_20260909_154856_5809460fe5ab&t=27.000&end=27.500) | 01 | 27.000–27.500 | stairs_candidate / 上楼候选 | up_candidate | uncertain |
| [010](viewer.html?id=gait_20260909_154856_5809460fe5ab&t=27.500&end=28.000) | 01 | 27.500–28.000 | stairs_candidate / 上楼候选 | up_candidate | uncertain |
| [011](viewer.html?id=gait_20260909_154856_5809460fe5ab&t=28.000&end=29.500) | 01 | 28.000–29.500 | stairs_candidate / 行走/调整候选 | unknown | uncertain |
| [012](viewer.html?id=gait_20260909_154856_5809460fe5ab&t=29.500&end=36.000) | 01 | 29.500–36.000 | stairs_candidate / 上楼候选 | up_candidate | uncertain |
| [013](viewer.html?id=gait_20260909_154856_5809460fe5ab&t=36.000&end=36.500) | 01 | 36.000–36.500 | stairs_candidate / 行走/调整候选 | unknown | uncertain |
| [014](viewer.html?id=gait_20260909_154856_5809460fe5ab&t=36.500&end=37.000) | 01 | 36.500–37.000 | landing_candidate / 平台等待候选 | unknown | uncertain |
| [015](viewer.html?id=gait_20260909_154856_5809460fe5ab&t=37.000&end=40.000) | 01 | 37.000–40.000 | landing_candidate / 平台转向候选 | unknown | uncertain |
| [016](viewer.html?id=gait_20260909_154856_5809460fe5ab&t=40.000&end=41.000) | 02 | 40.000–41.000 | landing_candidate / 平台等待候选 | unknown | uncertain |
| [017](viewer.html?id=gait_20260909_154856_5809460fe5ab&t=41.000&end=42.000) | 02 | 41.000–42.000 | landing_candidate / 平台行走/调整候选 | unknown | uncertain |
| [018](viewer.html?id=gait_20260909_154856_5809460fe5ab&t=42.000&end=48.000) | 02 | 42.000–48.000 | stairs_candidate / 下楼候选 | down_candidate | uncertain |
| [019](viewer.html?id=gait_20260909_154856_5809460fe5ab&t=48.000&end=48.500) | 02 | 48.000–48.500 | stairs_candidate / 转向候选 | unknown | uncertain |
| [020](viewer.html?id=gait_20260909_154856_5809460fe5ab&t=48.500&end=49.500) | 02 | 48.500–49.500 | landing_candidate / 平台转向候选 | unknown | uncertain |
| [021](viewer.html?id=gait_20260909_154856_5809460fe5ab&t=49.500&end=55.000) | 02 | 49.500–55.000 | landing_candidate / 平台行走候选 | unknown | uncertain |
| [022](viewer.html?id=gait_20260909_154856_5809460fe5ab&t=55.000&end=56.000) | 02 | 55.000–56.000 | stairs_candidate / 行走/调整候选 | unknown | uncertain |
| [023](viewer.html?id=gait_20260909_154856_5809460fe5ab&t=56.000&end=65.000) | 02 | 56.000–65.000 | stairs_candidate / 下楼候选 | down_candidate | uncertain |
| [024](viewer.html?id=gait_20260909_154856_5809460fe5ab&t=65.000&end=65.500) | 02 | 65.000–65.500 | stairs_candidate / 行走/调整候选 | unknown | uncertain |
| [025](viewer.html?id=gait_20260909_154856_5809460fe5ab&t=65.500&end=66.000) | 02 | 65.500–66.000 | unknown / 行走/调整候选 | unknown | uncertain |
| [026](viewer.html?id=gait_20260909_154856_5809460fe5ab&t=66.000&end=68.500) | 02 | 66.000–68.500 | unknown / 等待候选 | unknown | uncertain |
| [027](viewer.html?id=gait_20260909_154856_5809460fe5ab&t=68.500&end=69.000) | 02 | 68.500–69.000 | unknown / 行走/调整候选 | unknown | uncertain |
| [028](viewer.html?id=gait_20260909_154856_5809460fe5ab&t=69.000&end=70.000) | 02 | 69.000–70.000 | unknown / 转向候选 | unknown | uncertain |
| [029](viewer.html?id=gait_20260909_154856_5809460fe5ab&t=70.000&end=70.500) | 02 | 70.000–70.500 | unknown / 转向候选 | unknown | uncertain |
| [030](viewer.html?id=gait_20260909_154856_5809460fe5ab&t=70.500&end=76.000) | 02 | 70.500–76.000 | unknown / 行走/调整候选 | unknown | uncertain |
| [031](viewer.html?id=gait_20260909_154856_5809460fe5ab&t=76.000&end=78.500) | 02 | 76.000–78.500 | unknown / 转向候选 | unknown | uncertain |
| [032](viewer.html?id=gait_20260909_154856_5809460fe5ab&t=78.500&end=79.000) | 02 | 78.500–79.000 | unknown / 行走/调整候选 | unknown | uncertain |
| [033](viewer.html?id=gait_20260909_154856_5809460fe5ab&t=79.000&end=84.152) | 02 | 79.000–84.152 | unknown / 等待候选 | unknown | uncertain |

待确认：
- [24.00–30.00s](viewer.html?id=gait_20260909_154856_5809460fe5ab&t=24.000&end=30.000)：上行中停顿是在梯段内、平台还是调整/重试？
- [36.00–43.00s](viewer.html?id=gait_20260909_154856_5809460fe5ab&t=36.000&end=43.000)：是否到顶平台转身后沿同一楼梯返回；40s通行分界是否合适？
- [48.00–56.00s](viewer.html?id=gait_20260909_154856_5809460fe5ab&t=48.000&end=56.000)：这里是否中间平台，是否需分配平台编号？
- [64.00–70.00s](viewer.html?id=gait_20260909_154856_5809460fe5ab&t=64.000&end=70.000)：下楼是否已完成，何时离开最后阶梯？
- [70.00–84.15s](viewer.html?id=gait_20260909_154856_5809460fe5ab&t=70.000&end=84.152)：后续行走/转向是否为同一次通行的退出段？

