# 项目报告事实核查与改写记录

核查日期：2026-09-02。当前正文为 6 页导师审阅稿，后续补充上限为 8 页。

前一轮对照部署源码、固定版本训练代码、模型元数据、选模记录及原始实验记录，完成 20 项具体核查。本轮增加补充研究材料的来源、范围与计划审阅（F21–F27），保留 P1–P6，并新增 R1–R3。原稿中的“不包含”“未实现”“不适用于”等范围限定并非模糊陈述；有证据支持的限定予以保留。对已查明的内容采用确定陈述，对确实缺失的证据使用 P1–P6 占位符。

原稿保存在 [上一版报告](<workspace>/academic_assets/source_bundle/evidence/previous_report.md)。本记录中的“原稿表述／问题”引用原稿或概括其信息缺口；新补查的结果明确标注。


**版本说明。** 下列 F01–F27 为历次事实核查记录，其中章节位置对应当时的详细稿。六页精简版保留关键结论，其余细节移入补充材料；当前章节对应关系和新增结果解释见本文件 §6。C++／YAML 原码仍在附件，正文不再重复展示。

## 1. 历次已查明并纳入报告材料的内容

| 编号 | 原稿表述／问题 | 核查结论与本版处理 | 证据／新版位置 |
|---|---|---|---|
| F01 | 系统整体使用学习策略，容易与 router 自身是否学习混淆。 | Router 是确定性规则状态机；普通运动和 Gate 16 使用神经网络 actor。决策链未接入 learned router 或 LLM。按实现分层陈述。 | [router.py](<workspace>/academic_assets/source_bundle/source/deployment/src/s10_auto_nav/s10_auto_nav/strategy/router.py)、[节点适配](<workspace>/academic_assets/source_bundle/source/deployment/src/s10_auto_nav/s10_auto_nav/strategy_router_node.py)；§1.2 |
| F02 | Bundle 保留多个策略及实验开关，文件存在不等于运行启用。 | 部署启用 Gate 16 fallback；stairs57、fast adapter、mirroring、front-tuck profile 关闭。正常赛段为官方 57D actor，WP15→16 使用 174D base＋residual。正文加入实际 YAML 节选。 | [strategy_gate16.yaml](<workspace>/academic_assets/source_bundle/source/deployment/src/s10_bringup/config/strategy_gate16.yaml)、[run_race.sh](<workspace>/academic_assets/source_bundle/source/deployment/scripts/run_race.sh)、[runner](<workspace>/academic_assets/source_bundle/source/deployment/integration/gate16_policy_runner.hpp)；§3.2、§6.1 |
| F03 | 原稿 §5.2：“这些量可能供 router 或训练评测使用”。 | 已确认实际机身线速度、轮接触、绝对航点和 router 状态不在 174D actor observation 内。它们由上层路由、任务控制和仿真评测使用。删除“可能”，列出完整输入切片、缩放和顺序。 | [runner](<workspace>/academic_assets/source_bundle/source/deployment/integration/gate16_policy_runner.hpp)、[训练观测](<workspace>/academic_assets/source_bundle/source/training/training/mujoco_s10/official_policy_env.py)、[router](<workspace>/academic_assets/source_bundle/source/deployment/src/s10_auto_nav/s10_auto_nav/strategy/router.py)；§3.1 |
| F04 | 模型层结构与“最终模型”身份需要直接证据。 | Checkpoint 的 iteration=120、objective_mode=speed、terrain_mode=official_track；residual 隐藏层为 512/512/256/128，base 为 512/256/128，critic 为 256/128/64。Checkpoint 与发布选择的 SHA-256 一致；ONNX tensor shapes 和 ELU 算子已核对。 | [模型审计](<workspace>/academic_assets/source_bundle/evidence/model_audit.json)、[selection](<workspace>/academic_assets/source_bundle/evidence/selection.json)、[导出 manifest](<workspace>/academic_assets/source_bundle/evidence/climb_policy_manifest.json)；§3.1、§4.1；完整导出审计见附件 |
| F05 | “合成感知”需要给出具体生成方式，而非含糊描述真实性。 | LiDAR 首次相交查询包含几何遮挡；近水平扫描行的仰角为 +2.142857°。13×9 高度图来自独立向下射线，随 yaw 对齐，非 LiDAR 重建。空洞编码和高度转换已写出公式。实现未添加测量噪声或扫描畸变模型。 | [lidar.py](<workspace>/academic_assets/source_bundle/source/deployment/src/s10_perception/s10_perception/lidar.py)、[heightmap.py](<workspace>/academic_assets/source_bundle/source/deployment/src/s10_perception/s10_perception/heightmap.py)、[感知缓存](<workspace>/academic_assets/source_bundle/source/deployment/integration/gate16_perception_buffer.hpp)；§2.1 |
| F06 | 原稿 §6.2：“没有证据支持质量、摩擦、驱动延迟或 LiDAR 噪声随机化”。 | 固定发布 trainer 的 official_track 分支与 launch recipe 使用固定地形、固定动力学，仅随机化入口状态。通用 depth_range 不生成该分支地形。改为明确的实现陈述，并给出台阶高差 0.37691055 m。 | [发布环境](<workspace>/academic_assets/source_bundle/source/training/training/mujoco_s10/official_policy_env.py)、[trainer](<workspace>/academic_assets/source_bundle/source/training/training/mujoco_s10/train_official_policy_residual.py)、[recipe](<workspace>/academic_assets/source_bundle/source/training/training/run_gate16_speed_first_pipeline.sh)；§4.1 |
| F07 | “PPO 微调”“anchor”“speed reward”等概述不足以复现。 | 明确高斯 residual、采样动作 likelihood ratio、冻结参考 actor、按 16 维平均的 anchor MSE、40 次 critic warm-up、GAE 及终止 bootstrap 语义；补齐实际 reward 权重与前后轮进展公式。zero_anchor_coef 在 warm-start 时锚定初始 residual，并非零动作。 | [trainer](<workspace>/academic_assets/source_bundle/source/training/training/mujoco_s10/train_official_policy_residual.py)、[环境奖励](<workspace>/academic_assets/source_bundle/source/training/training/mujoco_s10/official_policy_env.py)、[speed objective](<workspace>/academic_assets/source_bundle/source/training/training/mujoco_s10/speed_objective.py)；§4.2、§5.1 |
| F08 | 配置参数与实际训练记录容易被写成同一种证据。 | 表格统一标为“发布训练配置”。12×32=384 transitions/iteration，120×384=46,080 为本阶段的配置推算；不等于全部训练历史。实际硬件、墙钟时长和原始曲线保留 P2。 | [recipe](<workspace>/academic_assets/source_bundle/source/training/training/run_gate16_speed_first_pipeline.sh)、[模型 iteration](<workspace>/academic_assets/source_bundle/evidence/model_audit.json)；§4.2 |
| F09 | 原稿只写训练横向偏置 ±0.02 m，未交代横向中心；训练与正式评测的 yaw 条件未并列。 | **新发现：**发布训练 reset 默认中心 y=33.365 m，yaw command=clip(−1.5ψ,−0.8,0.8)；45 个正式 case 均为 y=32.49969 m、cmd_yaw=0。部署台沿中心同为 32.49969 m。新增训练／评测／部署条件对照表，并保留速度和入口距离差异。 | [固定版本 reset](<workspace>/academic_assets/source_bundle/source/training/training/mujoco_s10/official_policy_env.py)、[45-case 原始行](<workspace>/academic_assets/source_bundle/evidence/formal_summary.json)、[部署配置](<workspace>/academic_assets/source_bundle/source/deployment/src/s10_bringup/config/strategy_gate16.yaml)；§4.1、§5.3 |
| F10 | 原稿 §9.1：“未必等待后续 router DONE 状态锁存”。 | **已查明：**结束 JSON 的 router mode=navigate，actual owner=official。记录器达到 0.18 m 后立即结束 launch，未等待 DONE；按序 33 个官方航点与独立 timer 证明完成。删除“未必”。 | [结束 JSON](<workspace>/academic_assets/source_bundle/evidence/accepted_run/00_32_seed8.json)、[原始日志](<workspace>/academic_assets/source_bundle/evidence/accepted_run/00_32_seed8.log)、[归档摘要](<workspace>/academic_assets/source_bundle/evidence/accepted_run/RUN_SUMMARY.md)；§7.1 |
| F11 | 392.257 s、392.258 s 和 674.01 s 分属不同计时口径。 | 官方起点 sim_time=0.001 s，终点=392.258 s，差值=392.257 s；674.01 s 是 recorder 墙钟时间。分别列出。该数字对应 Mac ARM64、seed 8 的一次提交成功样本。 | [原始 timer](<workspace>/academic_assets/source_bundle/evidence/accepted_run/00_32_seed8.log)、[重算结果](<workspace>/academic_assets/source_bundle/evidence/recomputed_metrics.json)；§7.1 |
| F12 | 局部表格需要从逐案例数据复算，而非只照录摘要。 | 从 45 行重新计算：38 成功、2 跌倒、5 超时；成功耗时 6.133684±3.097990 s（总体标准差）；失败按 17 s 计，均值 7.824 s；前轮至四轮完成均值 4.165263 s。正文按合理精度显示。 | [45-case 原始记录](<workspace>/academic_assets/source_bundle/evidence/formal_summary.json)、[重算结果](<workspace>/academic_assets/source_bundle/evidence/recomputed_metrics.json)；§7.2 |
| F13 | “Drop-free”“四轮完成”等名称可能被理解为接触力或整圈完成指标。 | Drop-free 依据轮心几何支撑，不以接触力定义，20/45=44.44%。局部成功使用四轮越沿和高度；部署另加接触数、姿态及持续确认。三者不混用。 | [支撑指标](<workspace>/academic_assets/source_bundle/source/training/training/mujoco_s10/front_retention.py)、[评测 runtime](<workspace>/academic_assets/source_bundle/source/training/training/mujoco_s10/evaluate_official_track_skill.py)、[router](<workspace>/academic_assets/source_bundle/source/deployment/src/s10_auto_nav/s10_auto_nav/strategy/router.py)；§5.1、§6.2、§7.2 |
| F14 | “正式评测”容易被理解为独立测试或泛化证明。 | 45-case 和速度筛选矩阵进入候选选择流程，因此本文将其明确称为 validation set。Checkpoint 内部 eval_success=0.6667 对应另一 27-case 分布，不能替换 38/45。独立测试保留 P3。 | [速度选模代码](<workspace>/academic_assets/source_bundle/source/training/training/mujoco_s10/select_gate16_fastest.py)、[recipe](<workspace>/academic_assets/source_bundle/source/training/training/run_gate16_speed_first_pipeline.sh)、[发布 selection](<workspace>/academic_assets/source_bundle/evidence/selection.json)；§5.2–5.3、§7.2–7.3 |
| F15 | 原稿指出缺少 speed_core 曲线，但归档里实际存在一份训练 CSV。 | **新核查：**corrected_front_tuck_train.csv 有 80 次迭代、累计 30,720 transitions，属于后续 rear_push 实验；其 summary 保留 best_iteration=0，发布选择仍为 speed_core。它不能补足 iteration 120 的原始日志，P2 继续保留。 | [后续训练 CSV](<workspace>/academic_assets/source_bundle/evidence/later_experiment/corrected_front_tuck_train.csv)、[对应 summary](<workspace>/academic_assets/source_bundle/evidence/later_experiment/corrected_front_tuck_training_summary.json)、[selection](<workspace>/academic_assets/source_bundle/evidence/selection.json)；§5.2 |
| F16 | 标称 50 Hz 容易被写成实测端到端实时保证。 | 已确定：RL 每步为 20×1 ms 物理积分；ROS/SDK 配置为 50 Hz。ROS node clock、C++ monotonic clock 与仿真成绩时钟分别使用；freshness 采用 callback 接收时刻，wheel state 没有独立 age 字段。实测延迟与 jitter 保留 P5。 | [router node](<workspace>/academic_assets/source_bundle/source/deployment/src/s10_auto_nav/s10_auto_nav/strategy_router_node.py)、[runner](<workspace>/academic_assets/source_bundle/source/deployment/integration/gate16_policy_runner.hpp)、[训练环境](<workspace>/academic_assets/source_bundle/source/training/training/mujoco_s10/official_policy_env.py)；§6.3 |
| F17 | “力矩保护”须限定阈值与作用域，不能概括为全程相同保护。 | 训练腿／轮力矩分别裁剪为 ±50／±14 Nm。Router 的 50 Nm、1 s watchdog 在 Gate 16／stairs 的 CLIMB／VERIFY_CLEAR 中检查绝对值；普通导航清空累计状态。逐执行器时序测量属于 P5。 | [训练环境](<workspace>/academic_assets/source_bundle/source/training/training/mujoco_s10/official_policy_env.py)、[router watchdog](<workspace>/academic_assets/source_bundle/source/deployment/src/s10_auto_nav/s10_auto_nav/strategy/router.py)；§3.2、§6.3 |
| F18 | 测试通过数量与负面结果必须共同报告。 | 378 项源码测试和 29 项 observation-contract 测试通过属于对应子集；独立 fixture 的 5 项失败导致 9 个 pytest setup errors。另一个五次 Mac 批次为 3 次启动失败、2 次途中倾倒，条件不同，单独列出。未改写成“全部测试通过”。 | [validation report](<workspace>/academic_assets/source_bundle/evidence/validation_report.md)、[独立失败批次](<workspace>/academic_assets/source_bundle/evidence/independent_failed_batch.md)；完整历史验证记录保留于附件 |
| F19 | 构建环境与包文件描述不够具体。 | 已核对三个 ROS 包 version=0.1.0；perception/nav 为 ament_python，bringup 为 ament_cmake。保留 package.xml、Docker digest、锁定 Python 依赖和运行脚本；正文只列关键版本，其余移至附件。Ubuntu/Python 环境版本依据归档验证文档，未将本次编辑机器当作当年训练机器。 | [perception package](<workspace>/academic_assets/source_bundle/source/deployment/src/s10_perception/package.xml)、[nav package](<workspace>/academic_assets/source_bundle/source/deployment/src/s10_auto_nav/package.xml)、[bringup package](<workspace>/academic_assets/source_bundle/source/deployment/src/s10_bringup/package.xml)、[Dockerfile](<workspace>/academic_assets/source_bundle/source/deployment/docker/Dockerfile)、[依赖锁](<workspace>/academic_assets/source_bundle/source/deployment/docker/requirements.lock)、[环境归档](<workspace>/academic_assets/source_bundle/evidence/THIRD_PARTY.md)；§1.2；运行脚本与构建细节见附件 |
| F20 | 原稿 §6.1 说明关键 recipe 文件与发布来源无差异；这不能扩展为整个当前训练工作区相同。 | 本轮直接从 5ef14fa Git 对象导出训练选集；当前工作区 official_policy_env.py 含后续修改，未将这些新逻辑混入发布事实。部署选集全部逐字节对照 3660b81 Git 对象；原码节选检查空白归一化后匹配，YAML 各行匹配。 | [文件来源与哈希](<workspace>/academic_assets/source_bundle/evidence/source_manifest.json)、[附件生成／核查脚本](<workspace>/academic_assets/source_bundle/academic_assets/build_source_bundle.py)；§1.1；完整源码索引见附件 |

## 2. 无法从现存材料补齐的项目

检索范围为本机项目目录中的训练仓库、固定 release、比赛提交归档及实验记录。下列结论表示“在本次可访问材料中未找到充分证据”，不表示团队从未做过对应工作。请提供原始文件或逐次数据；材料到位后可删除占位符并填写确定结果。

| 占位符 | 原稿中的不确定性／缺口 | 已有答案 | 仍需团队提供的内容 |
|---|---|---|---|
| **P1** | §6.1：“本地没有完整保存 base 的训练日志及所有中间 selection”；早期课程、BC/DAgger 与最终权重的关系未完整恢复。 | 最终 base 文件名、最终 residual 的 warm-start 路径、iteration 和发布哈希均已确认；BC/DAgger 代码存在。 | Base 实际训练方法与初始化；各 residual 阶段的运行配置、训练日志、数据集版本／哈希、selected checkpoint／selection.json；将所有阶段连接到最终 SHA-256 的对应表。不能仅依据代码存在填写“已执行”。 |
| **P2** | §10.2 未提供完整训练成本与 speed_core 原始曲线。 | 发布 recipe 超参数、iteration=120、本阶段配置推算 46,080 interactions 已填写；另有后续 80-iteration CSV。 | speed_core 的原始 stdout／CSV／TensorBoard；GPU 和 CPU 型号、驱动/CUDA/PyTorch 等软件版本；起止时间、总墙钟、训练与评测交互数；若提供总成本，列明前期训练是否计入。 |
| **P3** | §9.3：“证据不足以给出统一协议下的整圈成功率、置信区间”；缺少 independent held-out 测试。 | 一次 33/33 整圈、45-case 选模结果和另一失败批次分别报告。 | 固定代码/模型/配置/硬件下所有整圈尝试的 seeds、总次数、逐次成功/失败及用时；未参与调参选模的入口/地形集合及逐案例结果。置信区间可在收到数据后计算。 |
| **P4** | §9.3 缺少相对 baseline 的因果增益，§11 的研究价值尚无受控比较支撑。 | 现有材料能够说明实现和样本表现，不能给出模块贡献量。 | 在同一协议下，official-only、关闭 residual、关闭入口门控等 baseline/ablation 的完整逐次结果及唯一变更项。未完成的实验应保留占位符，不能填写推测提升。 |
| **P5** | §8／§11 提到时序限制，缺少实际 latency、jitter、torque-duration 数据。 | 配置频率、时钟来源、缓存阈值和 watchdog 实现已确定；现存逐 tick telemetry 不含各关节力矩和网络推理延迟列。 | 指定目标硬件和负载；base/residual 推理延迟、控制周期 jitter、消息采样/接收时间与同步方式；逐执行器力矩及超限持续时间。提供测量脚本与原始轨迹后再填均值和分位数。 |
| **P6** | §10.3：Gate 16 模型来源可追踪，但贡献者署名／独立授权文件不齐。 | 发布 manifest 和 THIRD_PARTY 记载模型源仓库、集成 commit、bundle commit、运行用途与哈希。 | Gate 16 模型贡献者的正式署名，以及团队保存的书面使用/分发授权或贡献者认可的许可证文件。若没有文件，继续保留 [待补 P6]。 |

## 3. 写作与范围处理

- 将“可能供……使用”“未必等待……”替换为已核实的数据流和运行结束状态。
- 将“尚缺……”“证据不足……”拆成确定的当前结果与编号待补项，不用主观判断填补证据。
- 将“正式评测”限定为参与选模的固定 validation matrix；将整圈结果限定到固定版本、seed 与计时口径。
- 正文报告仿真实验；真机实验不属于本版范围，因此没有添加“真机成功率”占位值。
- 将开发中但未部署的 phase/DAgger/fusion 候选与最终 speed_core 区分。PPO 与 residual RL 作为已有方法引用，不将其基础方法声称为项目首创。
- 源码与训练资料仅用于事实核查和附件整理；本轮未重新训练模型、执行整圈或重跑历史测试。

## 4. 引用与证据维护

外部方法定义核对原始论文：[PPO](https://arxiv.org/abs/1707.06347)、[Residual RL](https://arxiv.org/abs/1812.03201)。归档比赛性能来自本项目原始数据；新增研究结果按 E7 文档引用，原始产物留 R1。两类结论均不从外部论文外推。

附件中的 [source_manifest.json](<workspace>/academic_assets/source_bundle/evidence/source_manifest.json) 提供文件原始路径、固定版本、SHA-256 及派生材料说明。归档文件中的历史绝对路径保留为来源记录，不承诺它们在解压机器上存在。报告正文、PDF 与核查清单在附件顶层。


## 5. 补充研究资料审阅：本轮更新

新增来源 E7 为用户提供的 [external_industry_consultation_brief.md](<workspace>/academic_assets/source_bundle/evidence/external_industry_consultation_brief.md)。文件标注版本为 2026-09-03；该标签属于来源元数据，不能替代实际训练／测试日期。正文以“补充研究记录”注明来源，不将这些数据声称为本轮从 checkpoint 或逐案例 JSON 独立验证的结果。

| 编号 | 审阅问题 | 本轮处理 |
|---|---|---|
| F21 | 补充资料称真实地图／导航链路未完成，旧报告已有比赛导航；两者不能合并为同一部署版本。 | §1.1 明确版本边界：旧比赛版本使用 3660b81 与归档 MuJoCo residual；新增 Isaac 通用策略／Teacher 是后续研究线。model_796、model_499 与 Teacher 不归入 392.257 s 成绩。 |
| F22 | 补充合同 174D／187D、200 Hz 与旧报告 174D／1 ms 存在差异。 | §4.3 单列 Isaac Lab＋RSL-RL PPO 研究：174D 扩展官方 57D；Teacher 新增 3+4+4+2=13D 特权量；187D／187D／16D；physics 200 Hz、policy 50 Hz。旧 MuJoCo residual 的 20×1 ms 说明保持。具体源码与模型证据留 R1。 |
| F23 | model_796 的统计只出现在补充文档，原始 JSON 未提供。 | §7.3 以“补充材料记录”列出三 seed 成功率 55%／50%／60% 和分地形数量。文中列出的 7 类计数合计 33/60，与三 seed 比例一致；算术一致不等于核实实际运行。原始 seeds／逐 case 结果留 R1。 |
| F24 | model_499 的 132/132、46/132 和 10.4 s 不能从已有不同 WP17→18 文件代替核查。 | 按 E7 明确的规则楼梯矩阵、窄走廊续训、单次 WP18→19 回归分别引用；不据模型名相同拼接其他协议，不外推多初态鲁棒率。原始矩阵及回归文件留 R1。 |
| F25 | 补充材料有“已排除训练量不足”“整体架构合理”等判断，其证据强度不等于唯一根因证明。 | 正文保留可描述的表现：多轴控制和困难地形未同时通过、抬轮 credit 与成功事件存在问题。§8 明确观察与根因不同，不将某个因素确认为唯一原因，不复制所有试验及结论矩阵。 |
| F26 | 补充资料列出的是原定方案及希望专家讨论的问题，不是团队已确认的新执行指令。 | §8.1 提炼三类技术帮助；§8.2 标明“已记录的改进路线”，保留 Teacher→Student 和 LLC→HLC 的进入条件。路线选择、目标优先级与资源/时间安排留 R2/R3，不代替用户决定，也未运行新实验。 |
| F27 | E7 引用的 development_progress、development_experience、s10_interface_contract、s10_stairs_57d_training 未随材料提供。 | 在给定位置读取失败，并在项目目录按同名文件搜索（含隐藏/忽略目录）未找到。报告在 §4.3／§7.3 设置 R1，待补文件名与需求完整列于 [REPORT_PENDING_ITEMS_ZH.md](<workspace>/academic_assets/source_bundle/REPORT_PENDING_ITEMS_ZH.md)；不把 E7 内的“已验证”标签自动提升为本轮独立核验。 |

正文新增 §8“当前技术瓶颈、所需支持与改进计划”，包括三项重点困难、对应支持需求、三步原定路线及 R2/R3 两个实际留白框。为保持 8 页，将完整依赖／哈希、构建命令、历史测试细目和重复的缺项说明集中至附件。正文保留关键代码、PPO 目标和奖励定义。

新增待补编号：R1＝研究原始证据；R2＝目标、困难优先级与路线决定；R3＝合作支持、资源、负责人、预算、时间节点和通过／停止条件。P1–P6 的既有缺项继续有效。


## 6. 六页导师审阅稿：解释与证据对应

本轮删除正文中的包名、文件路径、镜像名及重复代码；将12张表缩减为6张，核心结构调整为以下六部分。细粒度公式和参数保留于附件中的精简前详细稿，源码、版本和哈希继续保留。

| 当前章节 | 对应证据 | 本轮写作处理 |
|---|---|---|
| §1 研究问题与架构 | E1–E3；F01、F05、F19–F21 | 表1改为两列三行；合并导航与感知概述，明确已知赛道和仿真条件 |
| §2 运动控制与交接 | E3–E4；F02–F04、F13、F16–F17 | 合并网络、动作接口与交接逻辑；强调局部完成和实际交还不同 |
| §3 残差训练 | E4–E5；F06–F08、F15 | 保留残差、PPO、奖励核心公式；完整归一化与配置移至附录 |
| §4 实验与解释 | E1–E2、E4–E6；F09–F14、F18 | 合并分布差异、结果和证据边界；增加下述18个成功案例的解释 |
| §5 多地形研究 | E7；F21–F25、F27 | 继续按补充文档归属引用；新增的研究判断不提升为原始实验核验 |
| §6 支持与计划 | E7；F26–F27 | 分开技术判断、原定路线、R2/R3决策；独立待补清单给出两页扩充预算 |

**新增的派生结果。** 从45-case逐条记录计算：success=1且drop_free=0的案例为18个；38个成功减去20个成功且保持支撑的案例，同样得到18。正文据此指出成功率未覆盖前轮几何支撑保持这一过程指标。该结论不把几何指标等同于接触力或完整的动态稳定性。结果写入 evidence/recomputed_metrics.json。

**机制假设的边界。** 历史最大进展奖励不因前轮掉落撤回，这是代码事实；它是否导致支撑丢失，需要奖励消融验证。训练／部署条件不同是配置事实；它是否导致系统失败，需要匹配状态分布的独立测试。通用与专用策略表现不同来自E7记录，但协议和初始化不同，因此未归因为单一网络容量、PPO或探索因素。

**保留的未知项。** P1–P6及R1–R3继续有效。相关工作与研究贡献定位并入R2，交由导师／团队确定；本轮没有新增“首次”“优于现有方法”等未经比较的主张。
