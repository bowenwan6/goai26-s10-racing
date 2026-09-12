# 48 号 golai 用户与真机环境安装记录

更新：2026-09-10。48 号 AGX 已完成本队账号创建、ROS 与驱动安装、ARM 构建、ONNX 离线推理和实际传感器采样。双雷达、点云合并器和深度相机已配置开机自启。没有启用 SDK、启动 `rl_deploy` 或执行机器人运动。

**2026-09-11 补充：当前使用 48 号，`xwy` 仅属于 50 号其他队伍。官方 SLAM 的当前状态与检查方式见 [48 号 SLAM 指南](S10_48_SLAM_ZH.md)。本日 AGX 的 SLAM 检查未发现点云；随后在 106 本机已确认官方雷达约 10 Hz、IMU 约 200 Hz。手机入口已部署到 103:8080，网页运行在 AGX，详见 SLAM 指南；完整走场保存尚未验收。下方新增本队独立 GUI 消息环境，编译和实机只读状态检查已通过。**

## 登录与工程位置

在本机 Windows 终端执行：

```powershell
ssh s10-48-golai
```

该本机别名使用 `golai@10.21.33.102`、主机身份别名 `s10-48-agx` 和专用 SSH 密钥。用户名为全小写 **golai**，UID/GID 为 2003。

2026-09-10 已将初建账号 `GOLAI`、同名主组和主目录改为 `golai`，保留 UID/GID、设备访问权限和 SSH 密钥。Bash 环境与安装／检查脚本已同步改名，五个 ROS 包已在新路径重新构建，ONNX 离线推理通过。旧构建目录和首次传感器检查保留在 `/home/golai/provisioning/before-lowercase-rename`，仅用于追溯，不要加载其中的旧 `install/setup.bash`。

| 项目 | 位置／配置 |
|---|---|
| 用户主目录 | `/home/golai` |
| 本队工作区 | `/home/golai/goai_embodied_future_material` |
| 安装脚本、日志、检查程序 | `/home/golai/provisioning` |
| DDS 配置 | `/home/golai/.ros/fastdds_ethernet.xml` |
| 本机 SSH 私钥 | `C:/Users/Lenovo/.ssh/s10_48_golai_ed25519`，保留在本机 |
| 登录方式 | 已配置本机密钥登录；2026-09-11 按用户要求设置系统登录密码，密码 SSH 已验证，密钥继续可用 |
| 权限 | `dialout`、`video`、`render`、`plugdev`；系统管理使用原有 `ysc` 账号 |

golai 的交互式 Bash 已自动加载 ROS Jazzy、已构建的工作区，以及 `ROS_DOMAIN_ID=0`、`RMW_IMPLEMENTATION=rmw_fastrtps_cpp` 和上述 DDS 配置。

## 安装依据与实际版本

使用用户提供的 [材料 README](../goai_embodied_future_material-main/goai_embodied_future_material-main/README.md) 和其完整源码，原材料 4,477 个文件已传入 golai 工作区。此本地 ZIP 解包目录没有 Git 提交元数据，因此不将它标为某个已核验 commit。

安装范围是 README 第 5 章的 ARM 真机部署，以及第 7、8 章的双雷达和深度相机。AGX 不需要本次仿真用的 MuJoCo 或训练环境。

| 组件 | 实际版本／状态 |
|---|---|
| 系统 | Ubuntu 24.04.4 LTS，aarch64，Tegra 内核 |
| ROS | Jazzy ROS Base、Fast DDS、RViz2 |
| 工具 | GCC/G++ 13、CMake 3.28.3、colcon、rosdep |
| C++ ONNX Runtime | 材料自带 ARM 1.22.0 |
| RoboSense SDK | v1.5.19，`78d2abb7f0c21f789a606d584475c631d5118dc3` |
| rs_driver 子模块 | `4eeadace465023db65a63fc8ef680dfaaca5812c` |
| rslidar_msg | `fe8a95cb242bd294cc3d5e3422f2093fb49a56ee` |
| RealSense ROS | 4.58.1 |
| Librealsense ROS 运行库 | 2.58.1 |
| OpenCV 开发包 | Ubuntu 4.6.0，与已有拆分开发包及 ROS 依赖配套 |

ROS 仓库通过官方 `ros2-apt-source` 1.2.0 配置。机器人直连 GitHub Release 和 ROS 主站下载不稳定，因此安装源包、固定版本雷达源码由 Windows 下载后传入；ROS apt 使用保留官方签名校验的 [清华镜像](https://mirrors.tuna.tsinghua.edu.cn/help/ros2/)。ROS 基础安装步骤依据 [官方 Jazzy 安装文档源码](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Installation/Ubuntu-Install-Debs.rst)。

本机 NVIDIA 源优先级较高，直接安装会选择另一版 OpenCV 并移除已有开发包。本次明确安装 Ubuntu `libopencv-dev=4.6.0+dfsg-13.1ubuntu1`，最终 apt 操作为 459 个新包、12 个升级、0 个移除，没有进行整机系统升级。

`rosdep update --rosdistro jazzy` 已完成；构建、构建工具和运行依赖检查通过。

## 构建与 README 的必要补充

五个 ROS 包构建成功：`drdds`、`s10_sdk_deploy`、`rslidar_msg`、`rslidar_sdk`、`dual_airy_merger`。

在 golai 登录会话中，可重新构建并执行离线模型检查：

```bash
bash ~/provisioning/build.sh
```

实际构建使用 `BUILD_PLATFORM=arm`。雷达另外开启 `ENABLE_TRANSFORM=ON` 和 `ENABLE_IMU_DATA_PARSE=ON`：v1.5.19 默认关闭这两项，而材料中的 YAML 外参及雷达 IMU 输出依赖它们。没有修改驱动源码来硬编码外参。

相机直接使用 ROS apt 安装的驱动和 Librealsense 运行库，已在本机 D435i 上验证深度数据，因此没有再执行 README 中额外的 `/usr/local` Librealsense 源码安装步骤。

材料 README 第 5.3 节称默认选用 Gamepad，但此份材料的 `main.cpp` 实际选择 `RemoteCommandType::kDDS`。后续运动控制须以实际源码和机器人 SDK 接口为准，先核对输入话题、IMU QoS／单位及现场接管流程；本次环境安装没有改动这些控制逻辑。

## 48 号网络配置

- `end0` 从 `10.21.33.102/28` 调整为 README 所需的 `10.21.33.102/24`。
- 持久增加 `224.10.10.0/24` 经 `end0` 的组播路由；通过 `nmcli device reapply end0` 生效。
- 保留原有 `10.21.41.0/24 via 10.21.33.103` 热点回程路由、Wi-Fi 默认路由及 `s10-agx-internet.service`。
- DDS 限定实际以太网地址 `10.21.33.102`；UDP 接收缓冲区按材料配置为 16 MiB。
- `.201`、`.202` 两台雷达均可达；原网络配置已记录在 `provisioning/wired-before.txt` 和 `routes-before.txt`。

## 已完成的验收

ARM 程序为 aarch64 ELF，动态库检查未见缺失。纯 ONNX Runtime 检查不创建 ROS／DDS 接口，实际完成 `obs float32 [1,57] → actions float32 [1,16]` 的合成输入推理，16 个输出均为有限值。

配置自启并启动服务后，执行约 35 秒的实际传感器检查，结果如下：

| 话题 | 收到消息数 | 接收窗口内频率 | 结果 |
|---|---:|---:|---|
| `/rslidar_front/points` | 350 | 10.0 Hz | 点云布局检查通过，`lidar_link` |
| `/rslidar_rear/points` | 350 | 10.0 Hz | 点云布局检查通过，`lidar_link` |
| `/LIDAR/POINTS_MERGED` | 350 | 10.0 Hz | 合并点云检查通过，`lidar_link` |
| `/camera/camera/depth/image_rect_raw` | 1041 | 29.8 Hz | 848×480、16UC1 深度图检查通过 |

三个传感器服务均以 `golai` 运行，检查结束后继续提供数据。消息与布局检查不替代物理外参标定或跨设备时间同步验收。

再次检查传感器时，在 golai 交互式登录会话执行：

```bash
python3 ~/provisioning/check_sensors.py
```

当前检查程序只订阅服务已发布的消息，不启动或停止驱动，不发送运动命令。不要再手动启动第二份雷达或相机驱动。

本地详细记录见 [安装与检查日志](../artifacts/s10-48-setup-20260910/)、[改名后传感器结果](../artifacts/s10-48-setup-20260910/lowercase-rename/sensor-check/result.json) 和 [改名后环境记录](../artifacts/s10-48-setup-20260910/lowercase-rename/verification.json)。原目录顶层的安装日志保留了初建 `GOLAI` 时的历史路径；改名验收记录位于 `lowercase-rename/`，最新自启验收记录位于 `autostart/`。这些 `artifacts` 日志保留在本机，不随 Git 自动分发。

## 传感器开机自启

2026-09-10 按用户要求安装以下 systemd 服务，已执行 `enable --now`。服务文件位于 `/etc/systemd/system/`，可复用副本位于 `/home/golai/provisioning/sensor-services/`。

| 服务 | 用途 | 已验证状态 |
|---|---|---|
| `rslidar-dual.service` | 前后两台 Airy 雷达 | enabled、active/running |
| `rslidar-merge.service` | 合并前后点云 | enabled、active/running |
| `s10-realsense.service` | RealSense 相机 | enabled、active/running |

沿用官方雷达服务模板，填入 `golai` 与实际工作区；三个服务显式使用 `ROS_DOMAIN_ID=0`、Fast DDS 和 `/home/golai/.ros/fastdds_ethernet.xml`，异常退出自动重启。没有改变点云 frame、QoS、配对容差，也没有增加 PTP、IMU 桥接或运动自启。

本轮验证了单元配置、开机启用链接、服务当前运行状态和实际消息；没有重启整机进行冷启动测试。记录见 [自启配置与验收](../artifacts/s10-48-setup-20260910/autostart/)。

查看状态（`golai` 可执行）：

```bash
systemctl status rslidar-dual rslidar-merge s10-realsense --no-pager
```

需要维护驱动时，在原有 `ysc` 管理账号会话执行：

```bash
sudo systemctl stop rslidar-merge rslidar-dual s10-realsense
# 完成维护后恢复
sudo systemctl start rslidar-dual rslidar-merge s10-realsense
```

重建这些服务的安装命令（已有服务时先确认副本是期望版本）：

```bash
sudo install -m 644 /home/golai/provisioning/sensor-services/*.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now rslidar-dual rslidar-merge s10-realsense
```

## 与 50 号的配置差异

对比基于 48 号当前实机、50 号 2026-09-10 的 [源码与配置备份](../artifacts/xwy-migration-050-20260910-174950/README.md)，以及此前的 [50 号实测记录](S10_REAL_ROBOT_QUICKSTART_ZH.md) 和 [手机采集部署记录](../tools/s10_gait_capture/DEPLOYMENT_050.md)。当前连接的是 48 号，未重新在线检查 50 号。下表列出部署相关差异，不代表整机固件或全部软件包完全一致。

| 项目 | 48 号当前状态 | 50 号已记录状态 |
|---|---|---|
| 基础环境 | Ubuntu 24.04.4 ARM64、ROS Jazzy、Fast DDS、ARM ONNX Runtime 1.22.0 | 同一基础软件系列，已有 ARM 部署环境 |
| 账号与工程 | 本队 `/home/golai/goai_embodied_future_material`，使用提供的官方材料 | 官方工程在 `ysc` 下；另一队的雷达驱动、工具在 `xwy` 下 |
| 双雷达参数 | 官方 YAML，前后点云 `frame_id=lidar_link` | XWY 运行 YAML，`frame_id=base_link`；逐项解析对比，两份 YAML 仅这两个 frame 字段不同，IP、端口、外参、距离范围与雷达时钟选项一致 |
| 合并器实现 | 官方版，Best Effort，前后帧最大时间差 100 ms，检查 `point_step` 和字段一致 | XWY 修改版，Reliable，最大时间差 60 ms，另有坐标系、字节序和点云缓冲区有效性检查 |
| 传感器启动 | 已配置双雷达、合并器、相机 systemd 自启，以 `golai` 运行并验证实际数据 | 雷达／合并器有 systemd 自启及 XWY 路径覆盖配置；相机曾存在服务 DDS 白名单问题，不能直接套用旧服务 |
| 时间同步、IMU 桥接 | 103→106原生PTP已核实，重启后IMU时间恢复；106已补PTP启动检查和手机测量时间检查，详见SLAM指南第4节；AGX仍用NTP，未部署IMU重打时间戳桥接 | 有 PTP／PHC 同步服务，以及 `/IMU` → `/S10_IMU`、`base_link` 的标准 IMU 桥接 |
| 运动部署适配 | 官方 SDK 源码；完成编译和离线推理，尚未启动 SDK 或做运动验收 | 之前的隔离测试副本应用了 IMU SensorDataQoS、弧度直用和 `main(argc, argv)` 话题映射适配；完成过起身、支撑、右移和趴下实测 |
| 手机采集与 Wi-Fi | AGX Wi-Fi 用于连接外网；保留 `s10-agx-internet.service`，尚未部署手机采集站 | `/home/ysc/s10_capture_session` 与 `s10-capture-session.service`；独立热点 `<CAPTURE_WIFI_SSID>`，网页 `10.42.50.1:8091` |

XWY 是其他队伍账号，因此上表中的 XWY 工程没有安装到 48 号。后续需要本队建图或运动部署时，应按实际用途核对坐标系／QoS、跨设备时间、IMU 消息单位及 SDK 输入映射；50 号的运动验证结果不能代替 48 号验收。本体固件、底层策略权重及物理外参的实际一致性，本次未确认。

## 本队控制 GUI 与缺失消息（2026-09-11）

原 GUI 通过 `ysc` 登录后 `sudo -u xwy`，并依赖 50 号重启即失效的临时工作区；这些前提在 48 号不成立。现已改用本机 SSH 配置中的 `s10-48-golai`、对应 `HostKeyAlias` 和专用密钥，直接以 `golai` 运行。默认 ROS 消息环境是持久目录 `/home/golai/s10_control_ws/install/setup.bash`。

48 号官方比赛消息源码缺少 `MotionInfo`、`MotionInfoValue`、`MotionState`、`MotionStateValue`、`GaitValue`。[准备脚本](../scripts/prepare_s10_control.py)从本机官方比赛 `drdds` 包复制到独立目录，补入这五个类型；定义依据已有厂商消息快照与官方手册中的 `MotionState`，已存在且冲突的定义会拒绝处理。原比赛工作区和系统传感器服务不受这份 overlay 影响。

本次已经创建和编译，不需要重复准备。首次在相同环境重建的步骤如下：

```powershell
# Windows 仓库根目录
scp scripts/prepare_s10_control.py s10-48-golai:provisioning/
ssh s10-48-golai
```

```bash
# AGX：仅在 ~/s10_control_ws 不存在时运行 prepare
python3 ~/provisioning/prepare_s10_control.py
source /opt/ros/jazzy/setup.bash
cd ~/s10_control_ws
colcon build --packages-select drdds --parallel-workers 2
source install/setup.bash
python3 -c 'from drdds.msg import MotionInfo, MotionState, BatteryData, Steer, StdMsgInt32'
```

已有目录需要重新编译时从 `source /opt/ros/jazzy/setup.bash` 开始，不删除目录。准备脚本可用 `--source` 和 `--output` 指定其他已核实的源码与新目录；它只准备消息，不构建 SDK、不发运动指令。

Windows 双击 `scripts/start_s10_gui.cmd`，默认保留 `s10-48-golai` 和 `golai`，**密码留空**即可使用已配置的密钥；点击“刷新状态”。可用 `S10_SSH_HOST`、`S10_SSH_USER`、`S10_GUI_ROS_SETUP` 覆盖连接和消息路径。GUI 的主机字段支持当前 SSH 别名中的 HostName、Port、IdentityFile 和 HostKeyAlias，不实现 ProxyJump。

本次构建成功约 36 秒；真实 GUI SSH 读状态返回 `error=null`、`state=0`、`zero_sent=false`，未发送运动命令。离线测试覆盖 SSH 别名／密钥、旧账号依赖移除、参数范围、反馈异常、断连／心跳超时回零。48 号的起身、平移、趴下与 SDK 运动仍未验收；这些按钮会发指令，不属于“刷新状态”。
