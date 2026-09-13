# S10 离线手机现场助手 v1

状态：**本地开发/评审版本，尚未部署到机器人，尚未完成真机或手机无外网验收。**

第一版只产出定位证据、原始短录制、地图绑定位置草稿/待审航点。网页不发布运动命令、不设导航目标、不播放 bag；停止录制、结束任务均不是停车或急停。使用者始终携带原厂手柄，负责机器人运动和接管。

## 现场六步

部署验收通过后，手机连接 `S10 PRO-048-5G`，打开 `http://10.21.41.1:8080/field`。沿用原网页登录，无需互联网、Tailscale、Codex、在线地图或CDN。

1. **现场自检**：目标填 `1209_01_F-20260912-175844`，新建会话。核对机器人身份、地图文件、106存储、传感器源时间、录制依赖。互联网不列为故障。参考点/外参未知明确显示未验证，不能以此生成有效导航航点。
2. **加载本次图**：在地图覆盖内的明确位置，手柄停稳，确认可接管且没有其他导航任务，勾选后点击。后台另取3秒新鲜位姿/IMU静止证据及厂商同一完整新鲜规划器监视块（无目标、空闲、命令和运动速度均为零）。证据未知阻断，不由复选框覆盖。旧图不必先正常全局定位，静止数值检查与全局验收分开。官方切图最多等待新会话状态360秒，不伪造百分比，不自动重切或回退。
3. **静止检查＋人工叠合**：保持停稳采样30秒，然后“加载目标地图显示资源”。灰色地图与青色同图实时扫描叠合；橙色为实时位姿。可拖动平移、缩放、跟随，在XY俯视和XZ侧视切换，必须看楼层/坡道和地标，不能只看XY。确认后保存人工核对。过期、掉定位、换图、服务/机器重启、标定改变均不能自动恢复旧绿灯。
4. **限时原始录制**：先静止、再手柄平地直行、转向各10–30秒。106在本地用rosbag2/MCAP录制原始消息，页面关闭不取消。到时监护进程发录制器SIGINT并等待元数据收尾；有512MiB总文件预算、2GiB磁盘保留门槛、16MiB录制缓存。缓存和停止收尾存在有界额外写入，512MiB不是硬文件系统配额。掉定位由人停稳，程序仅记录失败证据，不控制机器人。
5. **标点**：到首段起点、转弯、终点/安全停靠点，停稳后填名称和楼层并点击。后端采样3秒新位姿，检查独立源戳、参考帧一致、地图/会话身份和稳定性。当前实测 `/ODOM.child_frame_id` 为空、协方差全0，**不能推断为base_link或厘米级精度，当前只能保存草稿**。Z是消息参考点高度，不是地面。未验证参考点配置保持false，不能为了“绿灯”改为true。
6. **汇总导出**：先用手柄停稳，点击结束。即使定位/ROS不可用也应生成已有证据与失败报告。任务详情提供报告、航点审阅JSON及逐个原始文件下载（HTTP单区间Range续传，带SHA256）。下载大bag时需同时保存metadata.yaml及各mcap文件，并按原目录恢复。尚未实现一键ZIP打包或从手机下载后自动验证SHA，原文件留在106，不自动删除。

只有数据合格还不够：`waypoints-review.json` 总是 `navigation_ready:false`，需要电脑离线影子回放和人工路线审阅。此网页不提供自主跑的按钮。不同楼层不自动连线，楼梯和全程导航不是本版功能。

## 手机锁屏/断线

任务以106的SQLite为主本，不保存在手机后台。手机请求被接受后返回持久job_id；同一幂等键相同请求返回同一任务，不同请求拒绝。页面重开恢复任务查询，不自动重发写操作。若“未确认此次请求结果”，先看任务记录，不能反复点击。

worker重启将原QUEUED/RUNNING标记INTERRUPTED，不自动重跑；清除人工确认。录制有独立监护进程/父进程生命管道，worker离开也停止文件录制；系统服务使用KillMode=control-group作为第二层。部分文件保留，需要手工在副本恢复，不能声称跨断电录制。

仅依赖普通HTTP网页，不使用Service Worker、randomUUID或CDN。HTTP不加密，保留原登录/HttpOnly/SameSite Cookie/CSRF/受限SSH，只在专用机器人局域网开放8080；不新增公网监听。

## 架构与API

103热点转发不改；102仍为Python stdlib/Paramiko网页服务，新增`/field`、本地`field.js`；106新增系统级worker，Unix socket `/run/s10-field/worker.sock` 模式0600，持久根 `/var/opt/robot/data/s10_field_assistant` 模式0700。不会依赖用户linger或公网network-online。

- `POST /phone/field/submit`：`{action,key,session_id?,params}`，复用登录及`X-CSRF-Token`。新会话仅允许`action:selfcheck`与`params.target_map`。
- `GET /phone/field/health|live|list`，以及`job?job_id=…`、`session?session_id=…`、`preview?session_id=…`。
- `GET /phone/field/download?artifact_id=…`：只访问登记的本任务完成文件，无任意路径、任意命令入口。
- 受限SSH新增单个`action:field`转发白名单RPC，不能执行任意shell。原建图start/save与worker共享`operation.lock`，并核对数据库持久预留，避免后台已登记而还未开始时发生竞态。

操作params：selfcheck `{target_map}`；load_map `{stationary:true,remote_ready:true}`；localization_check `{stationary:true}`；confirm_overlay `{check_id,confirmed:true}`；record `{kind:'stationary'|'straight'|'turn',seconds:10…30,remote_ready:true}`；waypoint `{name,floor,segment:'flat',stationary:true,draft:true}`；finish `{}`。

有效校验结果与 `robot_id + boot_id + 地图三文件内容SHA身份 + localization InvocationID` 绑定；人工确认还绑定标定配置SHA。确认检查必须5分钟内，授权至多30分钟；观察到掉定位即持久撤销，恢复正常不会自行续绿。每次动作仍取新鲜数据验证。静止门限0.10m/5°和切图辅助门限0.03m/2°是初版数据检查门限，不是厂商安全性能或精度认证。

录制必需话题：`/ODOM`、`/IMU`、`/LIDAR/POINTS_MERGED`、`/ALIGNED_POINTS`；可选 `/tf`、`/tf_static`、`/LOCATION_STATUS/MATCHING_ERROR`。缺少可选静态TF不阻断证据录制，但外参仍未确认。真实高度图页面保持原样，本版未把原始高度图接入13×9策略输入，也未把未知格补零。合格bag需真实类型、非空且可读元数据、必需消息数/实际时段/源header/时序间断检查通过。

## 本地演示与测试

Python需3.12或更高，生产106是3.12.3。电脑默认`python3`可能是3.8，请使用明确解释器。

```bash
python3.12 -B tools/s10_mapping_web/field_demo.py --port 18080 --password field-demo-only
```

打开 `http://127.0.0.1:18080/field`，用户名`demo`，密码为命令中临时演示密码。只监听loopback，单独临时目录和fake worker，无真实SSH/配置读取，无自动真实→fake回退。页面常驻“演示”水印，fake录制永远不生成真实合格bag；退出自动清理临时演示数据。

```bash
python3.12 -B tools/s10_mapping_web/test_server.py
node tools/s10_mapping_web/test_localization.cjs
python3.12 -B tools/s10_mapping_web/qa/test_independent_core.py
python3.12 -B tools/s10_mapping_web/qa/test_independent_worker.py
python3.12 -B tools/s10_mapping_web/qa/test_independent_robot.py
```

独立评审结果见`qa/REVIEW.md`，以实际最新测试输出为准。演示跑通不代表手机无WAN、真机录制负载、服务冷启动、官方切图权限已验收。

## 部署与回滚清单（本轮未执行）

由维护者有人值守、机器人停稳且无活动建图/录制时进行；先比对2026-09-13 Git备份及实机最新哈希，避免覆盖新的高度图改动。任何下一步真机切图/录制测试仍需master安排，不能因下列示例认为已部署。

1. **106准备**：比对ROS Jazzy依赖/MCAP和数据分区挂载。复制`field_core.py, field_worker.py, field_robot.py, field_preview.py, field_recorder.py, field-worker.sh`及更新的`robot_backend.py`到已有app目录；保留原`heightmap.py/backend_key/known_hosts/受限authorized_keys`。
2. 用明确路径创建数据子目录，owner user:user、0700；`/etc/s10-field/config.json`由root维护、user可读不可写。参考模板填写核对后的`robot_id`和106 `/etc/machine-id`去掉换行后的SHA256；其值是设备绑定，不是SSH凭据。标定仍保持false，除非确有独立核验文件与版本。
3. 安装`/etc/systemd/system/s10-field-worker.service`模板并检查systemd unit。它是系统级服务User=user，不需要开启用户linger。先人工启动只读检查，再确定是否enable；启动不切图/不录制。检查Unix socket0600、数据0700、mount正常、health不是demo；服务启动失败不改成临时目录凑合。
4. **102更新**：备份当前server/index后安装`server.py, field_core.py, field.html, field.js`（server导入core供格式校验；不在102运行RobotAdapter）；保留`localization.html/js`、`heightmap.html`和认证配置/密钥。重新启动原用户网页服务前先确保无旧网页正在执行start/save；103不需要改动。
5. 真机只读自检和地图预览先验收，再由人值守逐项授权切图、30秒检查、10秒录制。监测CPU/RSS/磁盘写入及定位/雷达频率，验证老建图/保存/定位/高度图无回归。不给在运行中的机器人故障注入掉电、磁盘写满或改时钟。
6. 最后在手机关蜂窝数据、机器人无WAN情况下验首次登录、地图、每步、下载、锁屏重连；机器人冷启动后服务/内部授时需验证。未过此关，不承诺可以不带电脑出发。

回滚：先手柄停稳，确认没有活动建图/录制；新任务结果/原始文件保留数据根，不删除。停止新worker并确认其录制子进程已正常退出；恢复备份106后台和102 server/index后重启原网页服务。旧高度图源码和原认证不覆盖。数据库INTERRUPTED及历史报告保留，回滚不自动切回旧地图；当前active地图由现场人员单独确认，必要时按官方流程人工恢复。

本实现采用Safety System的失效关闭/人工接管边界、TF2的参考点与时间一致性规则、Mobile Offline Storage的机器人主本和重连幂等模式；没有宣称经过功能安全认证。
