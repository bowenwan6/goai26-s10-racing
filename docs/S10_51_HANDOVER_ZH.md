# 接手 51 号 S10：检查、补环境与复现实机测试

**历史计划，已被 48 号替代（2026-09-11）。当前按 [48 号安装记录](S10_48_SETUP_ZH.md) 和 [48 号 SLAM 指南](S10_48_SLAM_ZH.md) 操作。本文保留为换机检查清单，不代表正在使用 51 号。**

更新：2026-09-08。**51 号尚未连接或验收。** 本文用于从借用的 50 号迁移到工作人员拟安排的 51 号；不假定两台机器的账号、固件、代码和环境相同。

50 号的完整证据见 [真机上手与实测记录](S10_REAL_ROBOT_QUICKSTART_ZH.md)。最后一轮已完成右移、回零、趴下和进程退出，用户随后决定关机换电。后续文档整理没有再连接机器人或发送运动命令。

## 1. 哪些结论可以带走

| 项目 | 50 号的事实 | 51 号怎么处理 |
|---|---|---|
| Windows / Docker / WSL | Windows 用于 SSH；WSL 有 Jazzy；Docker 是 Isaac Lab 训练环境 | 本机环境可继续使用，不因换狗重装 |
| AGX 与本体 | 是两台设备，50 号分别为 `.102`、`.103` | 地址、账号和设备身份重新确认 |
| SDK / RL | SDK 允许外部关节控制；本体 RL 与 AGX ONNX 是两套控制流程 | 重新确认 SDK 开启、退出和手柄接管流程 |
| AGX 环境 | Ubuntu 24.04 ARM64、ROS Jazzy，已有 ARM C++ 部署与 ONNX Runtime | 先盘点，再补缺项；不假定已有或完全没有 |
| 模型来源 | AGX 文件与本地赛事仓库版本一致；本体权重来源未核实 | 向赛事方确认基线仓库、commit 和模型，不能只看文件名 |
| IMU | `/IMU_DATA` 为 Best Effort；姿态数值实测为弧度 | 分别检查 QoS 与单位，不能盲套 50 号补丁 |
| 传感器 | 双雷达可采样；相机原服务存在失效 IP 白名单问题 | 重新核对实际 IP、frame、驱动、服务和采样数据 |
| 部署方式 | 独立源码副本、独立输入话题，原目录保持不动 | 沿用这种做法，不覆盖前队伍代码/账号/服务 |
| 最小 SDK 体验 | AGX ONNX 完整起身、零速度支撑、趴下已通过 | 按同样顺序重新验收 |
| 右移 | 10% × 2 秒现场未见明显平移；100% × 1 秒现场确认稳定右移 | 只是 50 号结果，不是 51 号默认测试参数 |
| `/tmp` 文件 | 重启后临时包、编译副本会消失 | 工具和补丁保存在本机；重启后重建副本，不假定老路径有效 |

**数字特别容易混淆：**本体高层 RL 状态是 `17`；当前赛事部署程序内部 RL 状态是 `6`。不要混发。GUI 输入的百分比也不是 m/s；AGX 运行器当前将侧向比例乘以 `0.5`，所以 100% 对应策略侧向目标 0.5 m/s，不代表一定达到该实测速度。

## 2. 接机时请工作人员填写这些信息

| 项目 | 51 号记录 |
|---|---|
| 实物编号、型号、序列号 | 待确认 |
| Wi-Fi 的完整名称 | 待确认，不猜 `51` / `051` 的写法 |
| AGX IP / 登录账号 / SSH 指纹 | 待确认；密码另存 |
| 本体 IP / 登录账号 / SSH 指纹 | 待确认；Wi-Fi 密码不等于 SSH 密码 |
| 系统、ROS、固件版本 | 待确认 |
| 本队可使用的账号、工作目录、sudo 权限 | 待确认 |
| 官方仓库地址、commit、基线模型 | 待确认 |
| 哪些已有服务应保持运行、是否有人仍在使用 | 待确认 |
| SDK 授权状态、手柄开启/退出方式 | 待确认；不能仅凭“装过 SDK”判断 |
| SDK 切换前姿态要求、现场接管方式 | 待确认 |
| 雷达/相机型号、地址、安装位置与坐标系 | 待确认 |

不需要先获得本体 SSH 密码才能在 AGX 运行策略；但需要它才能读取本体上的模型文件/启动配置。50 号的本体 SSH 尚未登录成功，不能把本体权重说成已经拿到。

## 3. 先保证连的是 51 号

多台狗可能使用相同的内网 IP。**`.102` 通了，只能说明这个地址可达，不能证明连的是 51 号。**

1. 确认实物编号与工作人员提供的 SSID，断开上一台狗的 Wi-Fi，再连接指定的 51 号网络。
2. 在 Windows 执行 `netsh wlan show interfaces`，查看实际 SSID；检查是否还有旧狗的有线路径或静态路由。
3. 用工作人员确认的 AGX 账号/IP 登录，并使用独立 SSH 身份别名，保留 50 号的主机密钥记录。

PowerShell 示例（填入实际值后使用）：

```powershell
$agxIp = '<51号AGX的实际IP>'
$agxUser = '<51号AGX的实际账号>'
ssh -o HostKeyAlias=s10-51-agx "${agxUser}@${agxIp}"
```

首次出现指纹提示时，先与现场确认的设备指纹对照，再接受。不要遇到不同主机密钥就直接删除 50 号记录或关闭校验。换机后的通用启动工具会按这个已验证别名检查 SSH 身份。

50 号验证过的默认地址/账号只用于历史复查，不作为 51 号事实。当前旧 GUI 还包含 50 号专用的账号、sudo 用户和临时路径，不能只改 IP 就直接控制 51 号。

## 4. 只读盘点 AGX，缺什么再补什么

登录 AGX 后先检查：

```bash
id
hostname
uname -m
cat /etc/os-release
ip -br -4 address
df -h /
command -v g++ cmake colcon
ls /opt/ros
ps -eo user,pid,args | grep -E 'rl_deploy|mujoco|rslidar|realsense|imu_bridge'
```

在确认 Jazzy 已安装后，再检查 ROS：

```bash
source /opt/ros/jazzy/setup.bash
echo "$ROS_DOMAIN_ID"
echo "$RMW_IMPLEMENTATION"
echo "$FASTRTPS_DEFAULT_PROFILES_FILE"
echo "$FASTDDS_DEFAULT_PROFILES_FILE"
ros2 topic list -t --no-daemon
```

检查已发现的源码仓库时记录 `git remote -v`、`git rev-parse HEAD`、`git status --short`，以及模型路径和来源；不要执行 `git reset`、覆盖模型或清理他人的未跟踪文件。不要因看到未知节点就停掉它，先确认用途与归属。

| 实际缺项 | 最小处理 |
|---|---|
| 系统不是赛事要求的 Ubuntu 24.04 / ROS Jazzy | 先与赛事方确认支持的系统基线，不自行重装整台设备 |
| 缺少编译工具 | 按授权安装 `build-essential`、`cmake`、`git`、`python3-colcon-common-extensions` |
| 缺少 ROS / 消息包 | 按赛事仓库对应版本安装 Jazzy、构建 drdds；不要混用来源不同的同名消息定义 |
| 缺少部署源码/模型 | 在本队目录获取工作人员确认的基线版本；保留前队伍目录 |
| C++ ONNX Runtime 已随仓库提供 | 不用额外安装 Python `onnxruntime` 才能跑该 C++ 策略 |
| 传感器驱动缺失 | 按赛事 README 单独补雷达/相机驱动；传感器未启用不应靠启动运动程序来排查 |

真机 AGX 不需要为这次部署再装 MuJoCo、Isaac Lab 或训练环境。Windows 现有 Python/Tkinter/Paramiko 能继续使用；最终是否需要额外软件由盘点结果决定。

## 5. 核对模型与硬件接口，再准备副本

先得到实际部署源码包路径和消息包 install 路径。本文中的变量都要填为 **51 号自己的路径**：

```bash
SOURCE_PACKAGE=/absolute/path/to/S10_sdk_deploy
SOURCE_INSTALL=/absolute/path/to/message_workspace/install
```

需要逐项确认：

- 模型输入/输出名称、形状及预处理是否匹配运行器。50 号是 `obs [1,57] → actions [1,16]`，换模型必须重查。
- 关节顺序、方向、偏置、动作缩放、增益是否适用于这台硬件。不要用 50 号的标定覆盖 51 号标定。
- `/IMU_DATA`、`/JOINTS_DATA`、`/BATTERY_DATA`、`/HES_STATUS` 的消息类型、QoS、频率、时间戳是否匹配。能看到话题名不等于订阅能收到数据。
- IMU 欧拉角究竟是弧度还是度。可交叉对照标准 IMU 四元数及静止时重力方向；不要靠字段名称猜。
- 确认是否已有 AGX 关节控制程序，避免同时启动第二个控制器。
- SDK 开关是否确实打开、机器人是否按官方要求趴稳、现场手柄是否可接管。

50 号的已知修正是 IMU SensorDataQoS、直接使用弧度，以及让 `main(argc, argv)` 接收话题映射参数。它们与 51 号的源码是否相同要先比较；源实现不同就人工核对，不强行套补丁。

## 6. 可复用的本地工具与副本准备

已归档到仓库的工具：

| 文件 | 用途 | 是否能驱动电机 |
|---|---|---|
| [prepare_s10_sdk_copy.py](../scripts/prepare_s10_sdk_copy.py) | 在 AGX 创建独立源码副本并做受支持的接口修正 | 不会启动程序或控制电机 |
| [s10_sdk_trial.py](../scripts/s10_sdk_trial.py) | AGX 上的限时测试监控与模式/轴输入 | `--run` / 侧移模式会驱动；`--dry` 隔离关节输出 |
| [run_s10_sdk_trial.py](../scripts/run_s10_sdk_trial.py) | Windows SSH 启动、心跳与按设备归档日志 | 默认 dry；选择实机模式后会驱动 |
| [sdk-isolated-full.patch](../artifacts/s10-50-onboarding/sdk-isolated-full.patch) | 50 号副本的完整源码差异，供复核 | 补丁本身不会启动程序 |

把准备脚本复制到本队在 AGX 上的工具目录。仅在测量确认 IMU 是弧度后执行下面的 `rad` 示例；如果确认是度，使用 `deg`：

```bash
SDK_COPY_ROOT=$(python3 /path/to/prepare_s10_sdk_copy.py \
  --source "$SOURCE_PACKAGE" --imu-angle-unit rad)
echo "$SDK_COPY_ROOT"
```

脚本只向新建的 `/tmp/s10-sdk-isolated-*` 写入，模型复制后核对一致；third_party 只引用已有目录。对于不认识的主函数或 IMU 实现会拒绝继续，而不是猜着修改。它不替你验证模型来源、标定或 SDK 模式。

准备脚本报错时先停止处理，不执行后续构建；只有打印出有效的独立目录才继续。

随后构建独立副本：

```bash
source /opt/ros/jazzy/setup.bash
source "$SOURCE_INSTALL/setup.bash"
test -f "$SDK_COPY_ROOT/preparation.json" && \
cd "$SDK_COPY_ROOT" && \
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build \
  --packages-select s10_sdk_deploy --allow-overriding s10_sdk_deploy \
  --cmake-args -DBUILD_PLATFORM=arm
```

记录返回的绝对路径。`--allow-overriding` 是允许当前独立 overlay 使用同名包，不是覆盖原工作区的安装文件；构建命令必须在独立目录执行。

## 7. 按顺序复现，先隔离输出，再上电机

在本机 PowerShell 填好设备、路径变量：

```powershell
$agxIp = '<已核实的51号AGX IP>'
$agxUser = '<已核实的AGX账号>'
$sourceInstall = '/实际消息工作区/install'
$trialRoot = '/tmp/s10-sdk-isolated-准备脚本打印的后缀'

& 'D:/Anaconda/python.exe' scripts/run_s10_sdk_trial.py `
  --host $agxIp --user $agxUser --host-alias s10-51-agx `
  --source-install $sourceInstall --trial-root $trialRoot --mode dry
```

密码交互输入，不写入命令、文档或配置文件。当前工具使用 ROS_DOMAIN_ID=0 和 Fast DDS，这是 50 号测试基线；51 号必须先确认同样适用。如果不是这个分组/中间件，需要先适配，不能直接启动。

1. **dry 检查：**程序读取真实传感器，但所有关节输出转到独立测试话题；确认日志中的 IMU 与实际弧度反馈一致、初始化完成、退出成功。不要跳过这个步骤。
2. **零速度站立：**确认 SDK、姿态及现场条件后，才将 `--mode dry` 改为 `--mode stand`。约 4 秒起身、5 秒零输入 RL、约 4 秒趴下，最后进程退出。现场必须确认确实起身和趴下，不只看状态日志。
3. **小幅侧移：**零速度测试通过后，确认对应方向空旷，再选择 `--mode right` 或 `left`。当前实现比例 10%、持续 2 秒，按 50 号运行器映射为目标 0.05 m/s；实际响应仍需现场判断。
4. **更大输入：**`--mode right-full` 是 100% 右移、1 秒，仅记录为 50 号经现场授权后通过的模式。它不是 51 号初次验收参数，不要从这档开始。

测试使用独立按键/轴话题。回零是停止速度输入，不等于已退出 SDK；正常结束会请求趴下并终止本轮程序，SDK 开关需按官方流程操作。不要把旧本体 GUI 与 SDK 测试工具混用。

心跳断开、反馈超时、姿态/关节异常时，监控中止并请求阻尼；若本体、电源、控制器或网络出现故障，软件不能保证最后姿态，现场接管仍需保留。终端中断后不自动重试同一动作。

通用启动器会把日志保存在本机 `artifacts/s10-sdk/<SSH别名>/<本轮时间>/`，各轮分开。新的准备脚本和通用启动器已做本地检查，尚未在 51 号端到端验收；50 号实测采用相同监控逻辑的临时 SSH 启动方式。换机时务必从 dry 开始验证归档工具。

## 8. 雷达和深度相机的迁移验收

运动基线与传感器验收分开做。当前基线 ONNX 不接收雷达/相机，传感器画面正常也不等于策略已经使用感知。

- 先读现有服务的启动用户、工作目录、配置路径及运行状态。50 号雷达实际由其他账号的工作区启动，不能把该账号名写成通用前提。
- 核实有线网卡、AGX 地址、雷达设备地址、组播地址、接收端口与外参。50 号没有显式组播路由但实际收到了数据，不能仅凭配置表缺一行就修改网络。
- 分别确认前、后、合并点云有实际消息及合理频率。50 号使用 `/LIDAR/POINTS_MERGED`；不要为本地合并复用本体的 `/LIDAR/POINTS`。
- RViz Fixed Frame 使用点云实际 `frame_id`：50 号是 `base_link`，README 示例是 `lidar_link`，51 号以实际消息和外参为准。
- 相机先检查 USB 枚举和真实深度帧。50 号原相机 DDS 白名单指向已经不存在的 IP，造成服务运行但通信失败；51 号先核对自身接口，不能照抄那个文件。
- 不为了看画面直接停用未知服务。确需替换时在本队目录准备好配置，确认影响和归属后再操作。

详细驱动安装分支见主文档与赛事 README；已有可用环境优先验证，不重复安装两套驱动。

## 9. 关机、换电、重启后

1. 结束运动，确认趴稳、测试进程退出，再按官方流程退出 SDK、关机换电。
2. 不依赖本机断开 SSH 自动完成整台机器的关机；也不要把“SDK 已开”当成跨重启永久有效。
3. 重启后检查 Wi-Fi 是否重新连接正确设备、AGX 是否在线，以及 SDK 当前模式。
4. 检查 `/tmp` 中的接口包、副本是否仍存在；不存在就从本机归档脚本重新准备。日志、补丁和源码工具不能只留在 AGX `/tmp`。
5. 先只读检查，再 dry，再零速度验证；不把上次成功直接视为本轮可运动。

## 10. 51 号验收记录（目前全部待确认）

- [ ] 实物、SSID、IP、SSH 指纹对应正确。
- [ ] 账号、环境、固件和保留服务已记录。
- [ ] 官方仓库、commit、模型和标定来源明确。
- [ ] ROS 消息、QoS、单位和反馈新鲜度匹配。
- [ ] SDK 开启、退出、手柄接管流程确认。
- [ ] 独立副本构建通过，原目录未改变。
- [ ] dry 通过；程序姿态反馈正确。
- [ ] 零速度起身、稳定支撑、趴下得到现场确认。
- [ ] 方向性小动作、回零及终止验证通过，参数和观察有记录。
- [ ] 雷达/深度采样、frame 和实际网络配置核对。
- [ ] 文档、工具、日志归档到本机；重启后可重新准备。
