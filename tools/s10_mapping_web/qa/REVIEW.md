# 离线现场助手独立检视与测试

日期：2026-09-13。角色：独立 reviewer；生产代码由 implementer 修改。最终本地评审已完成。

## 范围与当前结论

基线 `889e4d8`，只在电脑上的开发树运行模拟测试。**本地未发现剩余阻断项；不是实机部署批准或现场安全验收。** 本报告中的“通过”不代表已部署、已用真实手机验收，或机器人定位/运动安全通过。reviewer没有连接真机、切换地图、实机录制或发布运动。

已独立执行旧功能基线：

- `python3 -B tools/s10_mapping_web/test_server.py` → `MAPPING_WEB_CHECK_OK`。
- `node tools/s10_mapping_web/test_localization.cjs` → `LOCALIZATION_VIEW_CHECK_OK`。

最终执行：独立QA **48/48** 同时通过 Python **3.12、3.14**；实现者自测 **5/5** 经reviewer重跑通过；旧 Python/JS 回归 **2/2**；本地浏览器完整流程 `BROWSER_ALL_PASS`。测试不把任务`SUCCEEDED`等同于质量`passed:true`。

## 阻断验收矩阵

| 编号 | 检查项 | 失败注入/复现 | 必须满足的结果 | 状态 |
| --- | --- | --- | --- | --- |
| S01 | 登录、CSRF、权限边界 | 未登录 GET/POST/下载，缺错 CSRF，未知 RPC/action | 不执行动作、不下载敏感文件；返回明确状态 | 本地HTTP测试通过 |
| S02 | 路径/SQL/XSS | artifact/name/id 注入 `../`、绝对路径、SQL、HTML；目录内符号链接逃逸 | 限定本任务注册产物；参数化 SQL；页面用安全文本 | 本地core/HTTP/浏览器通过 |
| S03 | 无运动功能 | 静态搜索及 mock 调用观测 | 无运动 publisher、导航目标、任意 shell、bag play | 代码复查和mock通过；无真机运动测试 |
| P01 | 幂等 | 双击同 key+内容，两客户端并发，同 key不同内容，已接受后响应丢失 | 一个操作；冲突拒绝；相同 key 可查询原任务 | core并发、HTTP409/202、Unix响应丢失、浏览器双击通过 |
| P02 | 服务解耦 | 关闭客户端、页面刷新；102服务重启 | 已接受任务不取消；状态和产物可重查 | 独立worker/客户端断开/页面刷新通过；实机102重启尚未做 |
| P03 | worker恢复 | RUNNING时终止worker再起；模拟boot改变 | 不虚假成功、不自动重切图/补录；明确中断并留证据 | 本地真实子进程重启与boot gate通过；未给机器人断电 |
| P04 | 真持久存储 | 持久分区不可用/无权限；Linger=no | 验证实际持久路径和0700权限；system service不依赖登录 | 模板/权限逻辑复查；实机启动/掉电验收尚未做。master确认overlay upper在ext4，不把overlay误判易失 |
| P05 | 互斥 | 旧锁活动、任务持久预留、外部切图 | 不启动冲突操作；核对真实 vendor 状态 | 独立锁/并发/身份测试通过；legacy锁整合代码复查 |
| C01 | 自检真实状态 | 缺依赖、低空间、传感器过期、未知身份 | 失败/未知分开，公网不可达不算故障 | 本地fake状态及门禁通过；真实ROS运行自检未执行 |
| C02 | 切图真实阶段 | 异常IMU、平移、导航unknown；旧图局部但已停稳 | 非法拒绝；合法到mock drmap；加载不冒充定位 | 8项独立robot/parser测试含正向通过；未真实切图 |
| C03 | 定位自动检查 | 局部里程计、坏/重复/过期源戳、参考帧改变 | 不错误合格；保留完整证据与原因 | core与worker通过；现场几何精度尚未核验 |
| C04 | 人工叠合授权 | map/loc/boot/标定改变、掉定位后恢复、坏点云 | 确认绑定准确检查；故障后不自行续绿 | 独立worker测试通过；人眼真实地标确认尚未做 |
| C05 | 录制 | 零必需消息、坏metadata、错类型、源倒退；总量/父管道/截止时间 | worker限时/总字节；失败留部分；不显示完整 | 9项审计mock/真实本地guardian子进程通过；实机MCAP负载/QoS和硬盘故障尚未做 |
| C06 | 标点 | 空child/外参、179/-179度、局部/故障/标定改变 | 后端新采样；不明语义仅草稿 | core/worker/浏览器通过；实际参考点依旧未验证 |
| C07 | 导出 | 缺录制/无有效点、ROSdown、Range与越界 | 缺项可见；流式鉴权；留原件 | 独立worker、HTTP 700KB跨块下载/206/416、浏览器通过；真机大包手机实测尚未做 |
| U01 | 手机离线UI | 非安全HTTP无SW/randomUUID、390px、长地图hash | 全本地资源，无主体溢出 | Chromium实际非安全origin与严格clientWidth通过；真实手机未测 |
| U02 | Wi-Fi断开 | 30秒任务中模拟断联＋刷新、过期localStorage | 旧绿灯失效；查询恢复；不重放写动作 | 浏览器完整流程通过；真实Wi-Fi切换未做 |
| U03 | 页面回归 | 旧建图、定位、高度图源码 | 旧身份/时序门槛保留；未知格不填0 | 旧两套测试通过；heightmap/localization源码未改变；实际老页面现场操作未重测 |

## 实测限制

本机 Python3.12与3.14 / Node24.2.0，不替代ROS Jazzy的实机兼容验收。浏览器采用本地loopback演示服务和非真实测试凭据；模拟传感器显著标注。测试域名 `http://s10-phone.invalid:18080` 由Playwright fixture仅转到 `127.0.0.1:18080`，不访问外部DNS或机器人。浏览器实际`isSecureContext=false`，`crypto.randomUUID`与`navigator.serviceWorker`不存在；不是用localhost安全例外冒充手机HTTP。

测试使用明确解释器 `/opt/homebrew/bin/python3 -B tools/s10_mapping_web/qa/test_independent_core.py`，避免工具会话 PATH 差异。首轮共14个独立测试；提交接口因一处 SQL 占位错误导致8个ERROR，位姿两类测试5个subtest失败，其余通过。已通知 implementer；以下是开发中首轮发现，不代表最终版本结论。

## 缺陷与复测记录

| ID | 优先级 | 复现 / 影响 | 修复验收 | 当前状态 |
| --- | --- | --- | --- | --- |
| R01 | 阻断 | `Store(temp).submit(selfcheck)` 抛 `table jobs has 11 columns but 12 values were supplied`，任何任务不能登记 | 新建/并发/幂等提交均实际运行 | 已修；Python3.12/3.14独立通过 |
| R02 | 高 | 31个静止新样本中间一帧 child_frame 从base_link变lidar_link，`pose_summary`仍passed | 整个采样窗口的参考帧及子帧一致，改变要拒绝 | 已修；Python3.12/3.14独立通过 |
| R03 | 高 | `localization_reasons`对pose.stamp=NaN/+Inf、started_at=NaN/-Inf返回空原因 | 来源/启动时戳必须有限数且时序有效 | 已修；Python3.12/3.14独立通过 |
| R04 | 高 | `artifact_info`静态只查最终symlink和size，疑似可读祖先symlink外部文件 | 注册后改job目录为symlink时下载必须拒绝 | 已修；祖先symlink越界独立测试通过 |
| R05 | 中 | 登记文件后改成等字节内容，静态逻辑仍返回旧SHA | 已注册不可变文件变更后拒绝或重新登记，不能回旧校验和 | 已修；stat指纹检测改写独立通过 |
| R06 | 高 | 厂商 `code=False` 被Python当作整数0，返回正常 | 精确整数0，不允许bool | 已修；独立通过 |
| R07 | 高 | pose.child_frame=''，标定reference_frame=''且verified=True，返回navigation_valid=True | 必须明确非空且核验的参考帧 | 已修；独立通过 |
| R08 | 高 | 人工叠合后标定version改变，仍可有效导航标点 | 标定profile/hash与检查/确认/标点/导出绑定，变更后作废 | 已修；独立通过 |
| R09 | 高 | aligned_cloud.age=NaN时人工确认仍成功 | 来源时间、接收age及点内容有限有效、同帧同时序 | 已修；独立通过 |
| R10 | 高 | 确认→定位丢失使录制拒绝→状态恢复，旧确认自动恢复通行 | 已观测定位/断流故障应持久撤销确认；恢复后重新检查 | 已修；独立通过；新增每秒监测与重启撤销 |
| R11 | 中 | sqlite连接`with`仅结束事务而不close，长轮询会延迟释放连接 | 使用finally close的连接上下文，重复查询不累计连接 | 已改finally close；代码复查通过 |
| R12 | 高 | load_map在IMU age/源时差NaN、源时间未来10秒、空/NaN角速度或源错误时进入vendor.run | 所有非法传感器证据在调用厂商前拒绝 | 已修；6个mock子场景独立通过 |
| R13 | 高 | aligned_cloud.error='source time discontinuity'仍可人工确认 | 点云的源错误必须阻断确认 | 已修；独立通过 |
| R14 | 阻断 | 真实目标PCD显示29,897点、684,115字节，旧RPC只读600,000字节必截断 | 有界完整大响应或分块 | 已修为双方4MiB＋完整换行检查；29,897点Unix RPC独立通过 |
| R15 | 高 | 仅零陀螺不能排除匀速直行；planner进程active不代表无导航任务 | 3秒新位姿静止＋同完整新鲜规划块无目标/零速度；未知拒绝 | 已修；独立正/反例通过，不是安全停车认证 |
| R16 | 中 | 390px移动视口被长串撑到555px；innerWidth随之扩大掩盖断言 | 严格scrollWidth<=clientWidth，长SHA也换行 | 已修；完整浏览器复测和390px截图通过 |
| R17 | 中 | master指出已不存在的localStorage会话会导致恢复循环失败 | 清理陈旧编号、可恢复实际会话或新建 | 实现已修；浏览器404保存编号恢复通过 |
| R18 | 阻断 | master实机只读发现ROS setup.bash在set-u下因AMENT_TRACE_SETUP_FILES未定义退出 | source ROS前不启nounset，source后再启 | 已修；master等效只读source+imports通过；新系统服务尚未部署 |

### 迭代与最终实际执行结果

- Python3.12：core 15/15通过；worker 10项中9通过，R13失败；真实adapter gate 1项6子场景通过。
- 独立 fake worker 子进程：2/2通过。请求发送后断开仍只登记一个任务；运行任务时终止再启动，历史任务保持`INTERRUPTED`且不重放，socket权限0600。
- 录制监护/审计：9/9通过。实际本地休眠子进程10秒停止、目录总字节限制、父管道EOF均正确结束；mock bag的缺失/损坏metadata、错话题类型、零消息与源倒退被拒。
- 上述进程均已结束，临时数据由测试夹具回收；没有ROS、SSH、真机服务调用。

最终总跑取代第二轮中间状态：

| 测试组 | 数量 | 最终结果 |
| --- | --- | --- |
| 独立core | 15 | 通过 |
| 独立worker | 10 | 通过 |
| 独立HTTP | 3 | 通过 |
| 独立进程/RPC | 3 | 通过 |
| 独立录制guardian/bag审计 | 9 | 通过 |
| 独立真实adapter gate/planner parser | 8 | 通过 |
| 独立QA合计 | **48** | **Python3.12、3.14均全通过（约14.6s）** |
| 实现者自测，由reviewer重跑 | 5 | 通过 |
| 旧Python/JS回归脚本 | 2 | 通过 |
| Chromium完整手机流程 | 1套 | `BROWSER_ALL_PASS` |

成功截图：`browser-phone.png`，实际390×4244px。失败网络环境白页已移出交付目录，未混为验收截图。reviewer开的18080 demo、fake worker、浏览器及测试子进程均已退出；只保留QA脚本/报告/成功截图。

原始数据目录未修改、backups未修改，生产代码修复由实现者完成。本地不存在未解决阻断；**真实设备部署、原场地图叠合/定位、系统冷启动、MCAP性能、手机无WAN和Wi-Fi重连仍是必须由现场有人值守完成的独立关卡。**

真实手机 Safari/Android、户外内部时间同步冷启动、持久分区掉电、106负载与话题QoS、原场地新图全局定位/人眼地标确认仍需现场验收。

## 技能依据

Code Reviewer 的安全→性能→正确性优先级用于独立审查；Safety System、TF2 Transforms 和 Mobile Offline Storage 分别用于不发运动/失败关闭、坐标身份及时序、持久任务与禁止重放写动作。Playwright E2E 技能用于本地浏览器实测；均不构成厂家或功能安全认证。
