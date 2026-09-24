# 自动导航测试页（2026-09-17）


> [!NOTE]
> Paths below in `code` are generated locally and are not tracked in this repository; rerun the tool to produce them.


23:35 更新：用户确认机器人趴稳、明天才测试，已授权停用两个竞争来源。103 handler 已停止并关闭其配置自启；106 localPlanner 已停用，同时保留原 pcl_remove 点云程序。新观察 `/NAV_CMD` 发布者为 0，定位与运控服务保持。下文 148 条采集中的“两来源”是此前历史状态。当前机器人不在原场景，明天先回到 Start 确认全局定位，再由遥控器起立和验证策略；不能仅搬到起点就直接运行路线。恢复方法与明日步骤见 控制权交接记录 `../../../s10-real-readiness/artifacts/native-control-handoff-20260917/README.md`。

入口：手机连接 **<ROBOT_WIFI_SSID>**，打开 **http://10.21.41.1:8080/native-nav**；也可在首页点击“自动导航测试”。沿用现有账号，部署后可能需要重新登录。页面与依赖均在机器人上，无 CDN 或互联网请求。

## 怎么使用

1. 点击 **运行实机检查 · 不运动**，等待约 15 秒。页面显示原厂模式、前进速度、读数时间、当前地图与全局定位；“检查完成”不代表能行走。
2. 选择 **原地切换普通模式** 或 **原地切换楼梯模式**。这两项只发零速度并验证模式回执，站姿仍可能变化。技术预检通过且现场有人持遥控器、范围净空、机器人站稳后，确认三项条件再开始。
3. 工程师完成实际速度响应、超时停车、坐标与点云、路线与切换位置的验收后，再选 **Start 平地段**、**Area B 楼梯段**或 **Start → Area B**。机器人要在对应起点且朝向正确。Start→B 到 B 上方平台结束，不延伸到后续赛道。
4. 使用原有 follower 和 router：平地普通模式、楼梯楼梯模式；每次切换先输出零速度、等停稳和原厂模式回执，之后才允许行走。平地上限 0.20 m/s，楼梯上限 0.15 m/s。
5. 测试期间保持页面前台在线。离开、锁屏、网络丢失导致约 4 秒无心跳时，运行程序锁定取消并请求零速度；恢复页面不会自动续跑。停止按钮始终固定在底部；断线时保留最后一次任务的停止入口。**软件停止回执不代表机器人已经停稳，遥控器接管优先。**
6. 在“记录与使用说明”下载本次 JSON 结果。原地模式反馈、实际速度响应、路线完成分别验收；不能互相替代。结果不会自动将工程门槛设为通过。

## 当前哪些能用

已部署到 102/106。主页、独立页面、登录返回原页、状态读取、只读检查、幂等提交、并发拒绝、取消与结果导出均已接通。原地切换和路线执行已接到真实原厂 runtime，当前受技术与现场条件门禁约束。

本次实机检查取得 148 条状态；检查取消另取得 24 条。两次均为 observer，全部记录 `native_publishers_created:false`，无原厂模式请求或行走输出。实测仍有两个其他 `/NAV_CMD` 发布者，`LOCATION_STATUS=3` 且原厂日志为局部模式，运动状态为 0；因此当前不能启动模式切换／导航。工程坐标、感知、路线、速度响应和停车验收仍待完成。**仅把机器人放到 Start 不会自动解锁以上条件。**

## 检查与体验修正

- 12 项 Python 检查：HTTP 登录/CSRF、未知请求/路径拒绝、现场确认不能绕过门禁、来源冲突、过期数据、幂等任务、共享操作锁、取消、心跳所有权、服务重启不恢复测试、损坏/过期心跳文件。
- Playwright 在 320、390、1280 像素宽度检查：没有横向溢出，关键按钮至少 48 像素，刷新不重发，断线仍能点击停止，没有 JS 异常或外部资源请求。浏览器测试使用明确的本机合成数据，不是实机行走验收。
- 106 上独立 ROS 域 211 + localhost 测试：真实 follower/router、消息编码、模式回执、网页本地心跳过期后零速度输出。未连接机器人控制域。
- 原建图后端与定位页面回归通过。实际热点地址返回新入口和登录页，未登录 API 返回 401；102 通过原有受限 SSH 通道调用 106，完成只读检查、重复请求返回同一任务、拒绝第二任务和取消。
- 原有 native router、几何与 shadow 回归共 121 项通过（需在 PYTHONPATH 中包含 `src/s10_auto_nav` 与 `src/s10_perception`）。
- 修正了历史读数误当实时、连接正常误当可运动、结束任务误当物理停车、停止按钮随页面滚动丢失、断线后反复提交等易混淆之处。

证据位于 `qa/native-nav-evidence/`：手机/桌面截图、实机 JSON、隔离 ROS 日志。没有执行 Start→B 实机运动，没有修改原厂 policy、SLAM 地图或原定位/规划服务。

## 工程维护

- 102：`/home/golai/s10_mapping_web/{server.py,index.html,native_nav.html,native_nav.js}`，继续使用原 `s10-mapping-web` 用户服务。
- 106：`/home/user/s10_mapping_web/native_nav.py`，新 `s10-native-nav.service` 只运行任务管理器，启动时不创建运动程序，重启只将未完成记录标为中断。
- 106 控制代码：`/home/user/goai_native_start_b_20260917/code/native_transfer/`。网页只允许固定测试类型，不接受任意配置、shell、ROS 参数或文件路径。
- 工程配置从 `config/field.accepted.json` 读取；不存在则使用 `field.pending.json`。只有依据现场证据完成工程验收后才可建立 accepted 配置，不能为消除网页提示伪造通过。每次现场确认只写本次任务的配置副本，不写回工程配置。速度响应的短距离验收仍使用原工程工具，并不在网页里提供“标记通过”按钮。
- 任务与 JSONL 留在 `/var/opt/robot/data/s10_field_assistant/native_navigation/`，socket 0600，目录 0700。共享原 `operation.lock`，测试与建图/现场写操作互斥。原 field worker 未重启。
- 应用新增 API：`GET /phone/native/status`、`GET /phone/native/report?id=…`、`POST /phone/native/submit`。POST action 仅 `submit/cancel/heartbeat`，复用登录和 CSRF。心跳只有发起页面的随机 owner 可以续期；任一已登录现场页面都能取消当前测试。
- 106 备份在 `/home/user/goai_native_app_20260917/before/`；102 在 `/home/golai/goai_native_start_b_20260917/app-before/`。本地原网页/后端备份在 `backups/s10-native-nav/2026-09-17/`。回退应先确认无运动任务，再停止新管理器、恢复备份与原网页服务；不要回退原厂 SLAM 数据。
