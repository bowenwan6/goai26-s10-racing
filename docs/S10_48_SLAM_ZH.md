# 48 号 S10：官方定位与 SLAM 使用

更新：2026-09-11。已在 `<ROBOT_WIFI_SSID>` 网络下登录 48 号 AGX 和 106 定位板，读取安装文件、配置和服务，完成 DDS 收包及官方命令预览。当前使用 **48 号、本队 golai**；`xwy` 是此前 50 号其他队伍的账号，48 号没有该账号。

## 1. 官方 SLAM 还在吗

**在，安装于 106 定位板，不在 AGX。** 它不依赖 `xwy` 的网页或工作区。

| 项目 | 48 号本次核实结果 |
|---|---|
| AGX | `10.21.33.102`，本队 `golai`，本机别名 `s10-48-golai` |
| 定位板 | `10.21.33.106`，厂商账号 `user`，主机名 `host` |
| 官方软件包 | `slam 3.5.1`、`slam-common-lib 1.1.3` |
| SLAM 程序 | `/opt/robot/share/slam/bin/slam_ddsnode`，通过厂商 `drsec exec` 运行 |
| 官方入口 | `/usr/local/bin/drmap`，实际进入厂商安装的 `map_manager` Python CLI |
| 服务 | `mapping.service` inactive/dead；`localization.service` active/running |
| 官方网页 | 组件已安装；`drmap server status` 返回未运行 |
| 地图 | 早期室外地图质量未通过；新室内地图已验证8条回环约束 |
| 激活地图 | `indoor_loop_01-20260911-203447`，20:49按用户要求激活 |

已完成安装核实、用户走场结果检查及时间保护部署。20:37保存的新室内地图已验证8条回环约束，20:49按用户要求激活，20:50定位日志持续报告正常、全局模式。未发送运动指令；绝对定位精度仍待实测。

### 2026-09-11 20:49：加载室内地图定位

已执行`drmap --format json map activate indoor_loop_01-20260911-203447`，确认`maps/active`指向新室内目录，官方定位服务重启成功，启动PTP检查通过。新服务InvocationID为`709ce99a45ea49c2bc6923a5bde46e5e`，检查时无自动重启。

20:50:10日志报告`上报状态=0(正常)`、`运行状态=全局`，坐标约`(1.586, 1.947, -0.334) m`、航向20.9°，匹配误差指标约0.038、内点率0.998。该匹配指标不是独立实测定位误差，不能据此宣称3.8 cm定位精度。

随后已增加[手机实时定位页面](http://10.21.41.1:8080/localization)，沿用原网页登录。红色定位针底部对应实际位置，箭头表示朝向；约每秒读取一次官方`/ODOM`并更新标记，不刷新整页。页面默认固定视野，“跟随狗”打开后才移动显示范围。地图不一致、非正常全局定位或数据过期时隐藏标记。此页面在AGX提供服务，电脑无需持续连接；原本地`map-full-3d.html`仍为离线文件。

### 2026-09-11 20:37：室内回环复测

用户通过手机保存了`indoor_loop_01-20260911-203447`。92个关键帧、142,598个完整点云点，PCD约2.28 MB，含原始关键帧的目录文件合计约51.09 MB。该会话日志记录8条加入优化的回环约束，与保存的`loops.txt`和最终心跳计数一致；60条状态心跳均报告同步正常，后处理完成。建图、回环、保存流程通过。

保存轨迹累计约40.17 m，首帧至最后关键帧水平距离1.16 m、朝向差约38°。用户确认实际停在起点旁一米多，未精确测量；首尾距离与描述基本吻合，不能将其直接解释为定位误差。20:49已激活此图，见上方加载结果。完整证据、点云和可视化见[室内复测报告](../artifacts/s10-48-indoor-loop-20260911-203447/REVIEW.md)。下文早期室外地图和时间检查保留为历史记录。

## 2. 连接和只读检查

Windows 登录定位板，按提示输入已单独交接的密码，不把密码放进命令或文档：

```powershell
ssh user@10.21.33.106
```

在 106 上：

```bash
dpkg-query -W slam slam-common-lib
systemctl status mapping.service localization.service --no-pager
drmap --format json mapping status
drmap map list
drmap mapping start --help
```

`drmap mapping status` 空闲时返回`phase=idle`。安装的CLI期待`/var/opt/robot/log/slam.log`，实际日志曾写入`/var/opt/robot/log/2026_0911/slam.2026_0911.log`，所以CLI或网页的厂商状态字段可能缺失；必须同时看systemd和实际消息，不能据一个`running`字段判定同步正常。

仓库的 [check_s10_slam.py](../scripts/check_s10_slam.py) 已补齐官方 CLI、时钟和机型配置调查，不依赖旧队伍。Windows 仓库根目录上传：

```powershell
scp scripts/check_s10_slam.py user@10.21.33.106:check_s10_slam.py
```

在 106 执行：

```bash
python3 ~/check_s10_slam.py --inventory --output slam-inventory.json
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/robot/fastdds.xml
python3 ~/check_s10_slam.py --seconds 8
```

`--output` 要求新文件名，避免覆盖旧证据。脚本只读取和订阅，不启动驱动或发布运动命令；`fresh`检查采样窗口中的时间范围、倒退和跳变，不是定位精度验收。`collected_at`使用板上时钟；106日期目前正确，历史3月记录仍按当时板上时间保留。

AGX 也可运行该脚本（已部署至 `~/provisioning/check_s10_slam.py`），DDS 配置改用 `$HOME/.ros/fastdds_ethernet.xml`。查询官方安装文件时应在 106 上运行，不能根据 AGX 的 `/opt/robot` 不存在判断 SLAM 缺失。

## 3. 使用官方建图入口

当前命令是 **`drmap mapping start` / `drmap mapping stop`**；地图名是位置参数，旧的 `drmap mapping -n ...` 用法不适用于当前 CLI。手机入口在本节后半部分；它复用官方接口并沿用 xwy 的跨板分工，不依赖 xwy 账号。

以下预览已在 48 号实机执行通过，不启动／停止服务：

```bash
drmap --format json --dry-run mapping start s10_48_test --no-activate --no-rviz --indoor
drmap --format json --dry-run mapping stop
```

现场准备好后，在 106 使用第 2 节的 ROS 环境并执行（本次未执行）：

```bash
python3 ~/check_s10_slam.py --seconds 6 --require-ready
# 上一步退出码为0后再执行：
# 室内；室外时用 --outdoor 替换 --indoor
drmap mapping start s10_48_test --no-activate --no-rviz --indoor
systemctl status mapping.service --no-pager
journalctl -u mapping.service -n 60 --no-pager
ros2 topic echo /SLAM_ODOM nav_msgs/msg/Odometry --once --qos-reliability best_effort
```

`--no-activate` 只保证不把新地图设为导航地图，**开始建图仍会停止原定位流程**。启动过程还可能保存／停止已有建图会话，不要重复启动。Python CLI 对部分 systemctl 调用不检查返回码，“启动成功”文字不代替服务与收包检查。

确认雷达、IMU 和建图位姿持续更新后，现场人员通过遥控器缓慢走完整个场地，尽量重新经过已走过的区域形成回环。不需要启动本队 `rl_deploy`。结束时：

```bash
drmap mapping stop
systemctl status mapping.service localization.service --no-pager
journalctl -u mapping.service -n 100 --no-pager
drmap map list
```

源码确认，`stop` 调用 `slam_command` 保存，随后停止 mapping 并尝试恢复 localization；保存命令返回非零会报错。不要用杀进程、关闭 SSH 或单独 `systemctl stop mapping` 替代保存。当前 CLI 的成功返回仍不代替最终文件和后处理检查，旧 `mapping_stop.sh` 对保存失败的处理也更弱，优先使用上述 CLI。

记录 **start 输出的完整地图路径**，通常为 `/var/opt/robot/data/maps/<名称>-<106系统时间>/`。确认本次目录的 `full_cloud.pcd`、`occ_grid.yaml`、`occ_grid.pgm` 非空，日志报告保存／后处理完成，再复制或使用。`--no-activate` 下 `maps/active` 仍指向旧地图；106 日期不正确，也不能仅凭日期猜目录。

若要把新地图用于厂商定位，核对结果后另行执行 `drmap map activate <完整地图目录名>`，它会更新 active 并重启定位。20:49已按用户要求使用此入口激活上述室内地图。

### 手机使用：103 入口、102 网页、106 官方建图

**电脑不需要一直连接。** `drmap` 的建图服务运行在狗上；电脑只是启动命令或显示界面的客户端。48 号现已部署手机入口 [http://10.21.41.1:8080/](http://10.21.41.1:8080/)，手机连接 **<ROBOT_WIFI_SSID>** 即可打开。用户名 `golai`，网页登录密码已单独交接，未写入仓库。

```text
手机浏览器 → 103 热点网关 10.21.41.1:8080
             → 102 AGX / golai：网页、点云累计、轨迹显示
               → 106 / user：官方 SLAM + 按需限频读取
遥控器 → 本体运动控制
```

这沿用本地 `xwy` 副本确认的分工：`HOTSPOT_README.md` 明确写了“103 网页转发 → Orin 网页 → 106 厂商 LIO”。103 是运动控制板，只承担入口转发；不能把“在 103 操作”理解为把点云处理或 SLAM 搬到运动板。48 号用原生 `systemd-socket-proxyd` 转发 HTTP，后台在本队 AGX 账号运行。

1. 手机登录后，先看到“雷达视野 · 尚未建图”，雷达和 IMU 应显示测量延迟且没有时间异常。此时显示的是当前扫描，**不是已建好的地图**。
2. 填写地图名称，选择室内／室外，点“开始建图”。页面调用官方接口，不启动 RViz、不自动激活新地图。此操作会暂停原定位。
3. 等待建图点云和位姿更新，再用遥控器走场。页面显示抽样累计点云和近期轨迹；没有用一个虚构的百分比表示建图质量。
4. 停稳后点“结束并保存”，等待文件和本次服务日志确认后处理完成。保存失败保留建图进程；地图实际保存在 **106 的 `/var/opt/robot/data/maps/<本次目录>/`**。
5. “已保存地图”可以查看目录和文件完整性状态。当前手机版没有迁移 xwy 的标点、地图 ZIP 下载和最终坐标修正功能；完整数据可通过 SSH/SFTP 从 106 取回。

手机熄屏、断网或关闭页面不会停止 systemd 中的建图。106 数据读取约在最后一次查看后 30 秒退出，重开页面自动重连。2026-09-11 已修正原来只检查接收时间的问题：页面同时检查消息测量时间、显示测量延迟并拦截异常输入；后端在暂停定位之前额外采样 3 秒复查。延迟以 **106 的系统时钟** 计算，包含采集、传输和排队时间，并非手机网络延迟或纯同步误差。

网页只保存、停止由本页创建且 InvocationID 匹配的会话；从其他终端启动的建图在这里仅预览。遇到“地图已保存但定位恢复失败”时，保留文件并在 106 检查 `systemctl status localization.service`，必要时执行 `systemctl restart localization.service`。

**限频不改变官方 SLAM 精度：** 只对手机这条额外订阅支路限频解码和显示，不更改厂商雷达／IMU 发布器、SLAM 配置或输入。官方程序按原配置处理原始数据，最终地图也不由网页抽样点云生成。手机画面会更稀疏、更新较慢。

预览上限为每秒约 900 点、20 cm 体素、AGX 最多累计 30,000 点及 2,000 个近期位姿。完整地图以官方保存结果为准；在线累计图不会回溯修正全部回环历史点，不能直接用来导航。106 不运行 HTTP、网页渲染或额外 SLAM，只在有人查看时订阅并限频解码，AGX 负责累计。

源代码、部署分工和维护命令见 [手机建图工具](../tools/s10_mapping_web/README.md)。103 socket 和 AGX 用户服务已启用自启，golai linger 已开启；没有为验证自启而重启整机。此前临时加在 106 上的 `s10-mapping-web.service` 已停用并禁用，官方 `mapping`／`localization` 的启用状态未改变。

**验收范围：** 早期室外会话发生跳时、零回环和约103 m首尾偏差，不通过质量验收。时间保护部署后的20:34室内复测已确认8条回环约束并完成保存；这验证了本轮建图流程，不代替独立真值定位精度测试或下一次整机启动验收。

## 4. 当前输入和时钟情况

### 2026-09-11 15:42 后的当前结果

用户此前保存的地图位于 `s10_48_0911-test-20260314-033014`，跳时及错误地图证据保留在本地 `artifacts/s10-48-map-review-20260911/`。本轮连接时发现整机已于约 15:31 重启；**IMU 和异常 ODOM 在本轮修改前已经随重启恢复**，不把这一恢复归功于新增代码。

106 连续订阅 12 秒的结果：

| 话题 | 消息数／频率 | 结果 |
|---|---|---|
| `/LIDAR/POINTS` | 122 条，约 10.1 Hz | 测量时间距106系统时间117–281 ms；点内时间抽查覆盖约0.1秒扫描周期 |
| `/IMU` | 2403 条，约 200.3 Hz | 时间差9.4–34.0 ms；此前约181天的偏差已消失 |
| `/ODOM` | 120 条，约 10.0 Hz | 时间差26.6–33.1 ms；位置约`[0, -0.177, 0.018] m`，不再出现10^14米异常 |
| `/LIO_ODOM` | 有发布者，0 条消息 | 不能据话题名认定里程计可用 |
| `/SLAM_ODOM` | 无发布者，0 条消息 | 当前未启动 mapping，未验证建图输出 |
| `/LIDAR/POINTS_MERGED` | 122 条，约10.2 Hz | AGX本队合并点云，仍不是官方输入接口 |

各收到数据的话题均未见时间倒退。发现的 DDS GID 与106进程PID对应：IMU为`yesense_node`/`yesense.service`；ODOM为`localization_ddsnode`/`localization.service`；两个原始点云写者对应`rslidar`和`hsLidar`。后者仍需区分各自实际发包情况，不能将两个发布者解释为正确合并的双雷达。当前 Jazzy Python 回调只提供 DDS 发送时间和序列号，没有逐消息 publisher GID；检查结果统计收到的混合话题序列，并另列发现的端点GID。

### 已部署的启动保护

授时层级已实测确认：**103 PTP主时钟 → 106网卡PHC → 106系统时钟**。106的`portState=SLAVE`，主钟identity为`3ede53.fffe.78f3dd`，对应103的eth0；AGX使用原有NTP，没有改为PTP主钟。106无RTC，本次开机约15秒时仍执行过一次大幅校时；IMU约28秒启动。本次顺序碰巧正确，但原驱动和定位服务只有`After=network.target`，没有等待PTP稳定。

已为106的`yesense`、`rsdriver`、`hsLidar`、`localization`、`mapping`添加`48-time-ready.conf`：启动前只读检查PTP主钟、SLAVE状态、主从偏差以及PHC与系统时钟偏差。连续3次检查均在5 ms以内且系统时钟未跳变才放行，60秒未就绪则本次启动失败，日志给出`S10_PTP_NOT_READY`。容差可用`--max-offset-ms`调整；这是启动条件，不是SLAM精度指标。

源码：[PTP启动检查](../scripts/wait_s10_ptp.py)、[systemd配置模板](../tools/s10_mapping_web/106-time-ready.conf)、[传感器检查](../scripts/check_s10_slam.py)。`pmc GET`读取PTP状态的含义见[LinuxPTP官方说明](https://www.linuxptp.org/documentation/pmc/)。未更改厂商PTP主从设置、IMU采样时间来源、滤波算法或地图。

手机的测量时间门限为相对106系统时间`[-0.25, 1] s`；3秒启动采样还检查时间倒退与相对单调时钟大于0.5秒的跳变。它用于拦截旧时间戳和明显异常，不能代替严格的点云／IMU逐点同步或配准精度验收。

已通过真实PTP检查（一次验收主从偏差-560 ns、PHC与系统偏差-2039 ns）、systemd实际执行启动检查、单元校验和手机HTTP数据验证。仅重新加载106配置、重启AGX网页，未重启驱动、定位或运动控制。**新增启动顺序尚未通过整机重启验收，旧地图不会因此自动修复。**

### 再次出现时间异常时

先停稳、结束并保存当前建图，确认`mapping.service`已停止，再处理授时。103系统日期、106系统日期和消息测量时间都要检查；运行中不要执行`date -s`或强制跳时。若板间时钟已正确但IMU仍发布旧时间，106的实际源是`yesense.service`，应在上述前提下重启该驱动，检查输入通过后再重启`localization.service`；无需据此重启运动控制板。

在106手动启动官方`drmap`之前，可执行：

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/robot/fastdds.xml
python3 /home/user/check_s10_slam.py --seconds 6 --require-ready
# 退出码0后再开始新的建图会话；非0先查看输出中的输入异常。
```

最初是谁在走场中触发103的日期跳变尚未确认；103当前SSH入口是厂商容器，容器中失败的chrony不能当成宿主机授时服务失效的证据。新增保护不接管103的授时政策，也不会在运行中自动重启估计器。短距离闭环及下次正常开机仍需验收。证据位于`artifacts/s10-48-time-sync-20260911/`。

## 5. 底层算法

48 号安装文件确认的结构是：

```text
雷达点云 + IMU
    → 惯性预测、点云去畸变
    → 激光惯性里程计 LIO：局部点云配准与位姿估计
    → 关键帧、回环检测、位姿图优化 PGO
    → 全局点云、轨迹、二维栅格地图
```

**前端有 FAST-LIO／Faster-LIO 的实现来源线索，后端采用 GTSAM 图优化。** 48 号的 README、change_log、`use-ikfom.h`、`lio.h` 和默认参数与此前 50 号保存的对应文本一致，因此算法证据现在也已在 48 号安装文件中核实，而不只是相同版本号的推断。

- `use-ikfom.h` 引用 IKFoM，定义位姿、雷达／IMU 外参、速度、偏置和重力等状态；`imu_processing.h` 使用迭代误差状态滤波接口。
- FAST-LIO 将激光与 IMU 紧耦合，采用迭代扩展卡尔曼滤波；Faster-LIO 基于 FAST-LIO2，用增量体素结构加速查询。厂商旧注释接口保留 iVox 特征。[FAST-LIO](https://github.com/hku-mars/FAST_LIO)、[Faster-LIO](https://github.com/gaoxiang12/faster-lio)、[IKFoM](https://github.com/hku-mars/IKFoM)
- 厂商更新日志明确记载回环、全局优化和 GTSAM 构建依赖。回环约束可使保存轨迹不同于在线里程计。

当前实现隐藏于 `LidarOdometryImpl`，并使用 `voxel_block_map` 接口；不能把旧注释当成当前实现，也不能称其为原版 FAST-LIO、Faster-LIO 或 LIO-SAM。默认最大滤波迭代 3 次、点云降采样 0.15 m；用户栅格分辨率为 0.05 m。目前尚未通过闭环建图质量验收。

本地证据位于 `artifacts/s10-48-slam-20260911/`：`106-inventory.json`、`106-dds.json`、`106-cli-check.json` 和读取的厂商 CLI 源码，未保存密码。这些材料不随 Git 自动分发，可用检查脚本在板上重取。详细历史对照见 [50 号算法调查](S10_SLAM_106_RESEARCH_ZH.md)。
