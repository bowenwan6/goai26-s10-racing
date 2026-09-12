# 50 号 S10：106 定位板、建图调用与 SLAM 算法调查（历史）

**2026-09-11 更正：本文调查对象是此前借用的 50 号，当前使用的是 48 号。`xwy` 是 50 号其他队伍的账号，48 号没有该账号，也没有这套 `s10_quick_mapping` 网页／后端。以下版本、路径和服务状态均为 50 号历史证据；48 号入口与实测结果见 [48 号官方 SLAM 指南](S10_48_SLAM_ZH.md)。**

调查日期：2026-09-09。实机访问路线为 Windows → AGX `10.21.33.102` → 定位板 `10.21.33.106`。本轮仅通过 SSH 读取安装文件、配置和系统状态，没有启动建图、修改配置或发送运动命令。

本文可直接在 GitHub 阅读。下文 `artifacts/...` 是另行共享的本地研究包路径，不是 Git 仓库附件；数据范围、地图预览和复现条件见 [研究交接](S10_DATA_RESEARCH_ZH.md)。

**50 号在调查当日的结论**

106 安装的是云深处维护的 `slam 3.5.1`，整体是 **激光惯性里程计（LIO）＋回环检测＋位姿图优化（PGO）＋地图输出**。现有头文件强烈指向 FAST-LIO／Faster-LIO 一系的前端实现来源，后端的 GTSAM 使用有厂商更新日志佐证。尚不能认定当前可执行文件等同于某个未经修改的开源版本，也不能确定具体上游提交。

| 判断 | 证据与确定程度 |
|---|---|
| 厂商程序身份与版本 | `dpkg-query -s`：包名 `slam`，版本 `3.5.1`，架构 `arm64`，维护者 `deeprobotics.cn`；配套 `slam-common-lib 1.1.3`。已确认。 |
| LIO＋位姿图优化 | 安装目录 README 直接描述为“激光里程计-位姿图优化建图”；头文件提供雷达、IMU 输入；配置分为 `lio` 和 `pgo`。已确认安装软件的结构。 |
| IKFoM 迭代误差状态滤波 | `use-ikfom.h` 引用 IKFoM；`ImuProcess::Process` 接口使用 `esekfom::esekf`。已确认随包头文件中的实现接口，未直接检查运行中的滤波器。 |
| FAST-LIO 系谱 | 状态定义、噪声定义、IMU 处理接口与官方 FAST-LIO 源码高度对应。强来源线索。 |
| Faster-LIO 系谱 | `lio.h` 中旧的、已注释代码包含 `IVoxType`、PHC/DEFAULT 分支、`ObsModel`、`MapIncremental` 及相同风格成员，对应 Faster-LIO 头文件。强历史线索；不能把注释认作当前运行代码。 |
| 回环与 GTSAM 后端 | 更新日志明确记录回环检测、全局优化、回环噪声和 GTSAM 构建依赖；系统安装 GTSAM 4.2.0。已确认包内后端设计；没有核实具体求解器配置或本次历史会话实际接受了哪些回环。 |
| 完整算法源码 | 当前 `LidarOdometry` 只暴露接口，实际实现藏在 `LidarOdometryImpl` 中；本轮所查安装文件没有给出对应完整实现。 |

**SLAM 是什么，和这几个程序有什么关系**

SLAM 是 Simultaneous Localization and Mapping，即同时定位与建图：机器人移动时，既估计自己的位置、姿态，也构建环境地图。它是一类技术；`slam_ddsnode` 是这台机器人上实现相关功能的具体程序。定位板是运行程序的计算机，地图和轨迹是输出，`s10_quick_mapping` 是操作工具。

LIO 指 LiDAR-Inertial Odometry，利用雷达与 IMU 估计连续运动；PGO 指 Pose Graph Optimization，利用关键帧之间的约束调整整体轨迹。检测到再次经过同一区域时，可以形成回环约束。厂商版本还提供 GPS/RTK、IMU 重力约束相关配置；有这些配置不代表每次采集都实际收到并使用了对应传感器。

```text
雷达点云 + IMU
      ↓
dr_lio：惯性预测、点云去畸变、局部配准与位姿估计
      ├─ 连续定位结果 /SLAM_ODOM
      ↓
关键帧、回环检测、位姿图优化
      ↓
保存轨迹、全局点云、二维栅格地图
```

这是根据安装接口、配置与更新日志整理的模块关系；`/SLAM_ODOM` 在所有模式下究竟发布优化前还是优化后的结果，仍需具体实现或运行日志确认。对已分析的高台录制，已有数值证据证明其位置与 `lio_odom.pose` 对应。

**106 如何连接，建图工具如何调用**

已经验证可通过 AGX 登录 106，登录用户为 `user`，主机名 `host`，CPU 架构 `aarch64`。本轮验证的是经 AGX 的路线，没有另外验证 Windows 直连 106。

AGX 的访问复用本机已有 SSH 密钥；106 的访问复用 AGX 上建图网页已有配置。密码仅在远端脚本内存中使用，没有输出或复制进本地研究材料。

1. 网页 `web_server.py` 或终端 `quick_mapping.py` 接受开始、标点、保存等操作。
2. `paramiko` 通过 SSH 登录 106，SFTP 上传 `robot_backend.py` 到 `/home/user/.s10_quick_mapping/`。
3. SSH 执行 Python 后端，先加载 `/opt/robot/scripts/setup_ros2.sh`。动作以一行 JSON 经标准输入传入，结果以 `S10_RESULT ` 开头的 JSON 经标准输出返回。
4. `start` 创建 `/home/user/s10_maps/<session>/`，先执行 `systemctl mask --now localization.service`，再由 `systemd-run` 启动本次建图服务。厂商程序通过 `drsec exec` 运行 `slam_ddsnode <session>/map indoor|outdoor`，并设置 DDS 配置与动态库路径。
5. `mark` 在 106 本机订阅 `/SLAM_ODOM`（`nav_msgs/msg/Odometry`），检查新鲜度后保存位置、四元数、航向、坐标系和时间戳；不控制机器人走向标记点。
6. `save` 调用厂商 `slam_command`，确认地图保存及后处理完成后停止本次服务，再经 SFTP 取回文件。工具运行于 AGX 时，下载目的地是 AGX；Windows 上的副本是我们后来再复制的。

源码入口：SSH 与 JSON 调用（`artifacts/xwy-inventory/source_snapshot/s10_quick_mapping/quick_mapping.py:55`）、106 启动逻辑（`artifacts/xwy-inventory/source_snapshot/s10_quick_mapping/robot_backend.py:170`）、位姿订阅（`artifacts/xwy-inventory/source_snapshot/s10_quick_mapping/robot_backend.py:73`）、保存逻辑（`artifacts/xwy-inventory/source_snapshot/s10_quick_mapping/robot_backend.py:236`）。

2026-09-09 本轮检查时，`localization.service` 为 `inactive`、`masked`，进程名检查未发现 slam/localization 匹配项。这是当时的状态快照。`s10_quick_mapping` 的保存流程没有解除原定位服务的屏蔽；不要与厂商另一个 `mapping_stop.sh` 脚本中的重启逻辑混为一谈。

**为什么判断与 FAST-LIO／Faster-LIO 有关**

直接证据位于下载的 use-ikfom.h（`artifacts/xwy-study/research/slam-106-evidence/include/dr_lio/use-ikfom.h:4`）、imu_processing.h（`artifacts/xwy-study/research/slam-106-evidence/include/dr_lio/imu_processing.h:39`） 和 lio.h（`artifacts/xwy-study/research/slam-106-evidence/include/dr_lio/lio.h:23`）。

前端的状态包括位置、旋转、雷达与 IMU 外参、速度、陀螺仪偏置、加速度计偏置、重力方向。状态排列与 FAST-LIO 的 `use-ikfom.hpp` 对应，连噪声初始化数值及部分注释也一致。IMU 接口包含初始化、滤波预测和点云去畸变。

更具体的线索是旧 `lio.h` 的 iVox 分支和 `ObsModel` 等接口，与 Faster-LIO 的 `laser_mapping.h` 对应。Faster-LIO 官方说明其基于 FAST-LIO2，采用增量体素结构。由此可以合理推断厂商前端有这一系实现的演化痕迹，但当前头文件已使用实现隐藏，并出现 `voxel_block_map` 接口，不能断言现版本仍完整使用原来的 iVox 实现。

官方对照来源，核对日期 2026-09-09：[FAST-LIO 状态定义](https://github.com/hku-mars/FAST_LIO/blob/main/include/use-ikfom.hpp)、[Faster-LIO 接口](https://github.com/gaoxiang12/faster-lio/blob/main/include/laser_mapping.h)、[Faster-LIO 项目说明](https://github.com/gaoxiang12/faster-lio)、[IKFoM 工具库](https://github.com/hku-mars/IKFoM)。这些是上游公开源码的当日网页对照，没有取得厂商与上游之间的提交对应关系。

**后端与配置中确认了什么**

厂商 change_log.md（`artifacts/xwy-study/research/slam-106-evidence/change_log.md`） 记录：3.2.0 加入回环和全局优化；3.2.1.1 调整回环配准与局部搜索；3.3.2 加入 GPS 和 IMU 重力约束；3.4.0 调整回环噪声类型；3.5.1 明确提到 GTSAM 和 `pose_graph_optimization` 的构建处理。

GTSAM 是图优化工具库，使用它不能单独证明系统就是 LIO-SAM。系统还安装了 `gtsam_points`、`fast_gicp`，但仅凭依赖存在，也不能认定主算法就是 GLIM 或某个 GICP 配准变体。[GTSAM 官方说明](https://github.com/borglab/gtsam)

当前默认配置：params.yaml（`artifacts/xwy-study/research/slam-106-evidence/conf/params.yaml`）。用户覆盖配置原路径是 `/var/opt/robot/conf/slam/params.yaml`，副本为 runtime/params.yaml（`artifacts/xwy-study/research/slam-106-evidence/runtime/params.yaml`）。3.5.1 日志说明采用默认→用户→机型三层合并，因此不能只看默认值。

| 项目 | 读取到的配置及含义 |
|---|---|
| 雷达输入 | 默认 `/LIDAR/POINTS`，`lidar_type: 1`（配置注释为速腾） |
| IMU 输入 | 默认及 CA9B/CA9C/CD1/CC3 配置为 `/IMU`；CR1 为 `/IMU_DATA_BODY`。本轮未确认当前机型选择项，不宣称最终生效话题已经核实。 |
| 定位输出 | `/SLAM_ODOM` |
| 滤波迭代 | `max_iteration: 3` |
| 点云降采样 | `leaf_size: 0.15`，机体输出点云 `leaf_size_body: 0.05` |
| 外参在线估计 | 默认 `extrinsic_est_en: false` |
| 栅格分辨率 | 默认 0.10 m，用户覆盖为 0.05 m；所读机型差异文件没有覆盖这一项 |
| 回环搜索配置 | `max_search_distance: 8.0`，`matching_error_threshold: 0.16`，`inlier_fraction_threshold: 0.95`；具体判断公式未取得 |
| 高度图 | 默认 `height_map.enable: false`；存在高度图接口不代表历史数据录制了高度图 |

安装文件的前四字节为 `DRDR`，`file` 将主程序识别为 `data`，`readelf` 报非 ELF。厂商 `config_dds.sh` 的注释明确说明 `drsec exec` 用于加密 ELF 的解密执行。因此本轮没有从主程序取得动态符号或直接调用链，也没有为探查而执行它。

**对地图重建与 RL 专家路径的意义**

现有高台会话已经能重建点云地图；SLAM 输出提供机身运动参考，关节录制提供腿轮运动。还需要统一时间与坐标系，并评估轨迹质量，才能用于 RL 的参考路径或跟踪奖励。

已有数值结果：高台 bag 与 `lio_odom.pose` 的 10 个重合时间样本位置在文本精度内一致；`lio_odom.pose` 与 `poses.txt` 各有 558 个相同时间戳样本，但单一刚体变换对齐后位置 RMSE 仍约 3.45 m。因此两者不是简单换一个固定坐标系就完全对应。

此次确认存在回环和全局优化，为“在线轨迹与保存轨迹为何不同”提供了合理机制，但尚未证明该会话差异全部由优化产生，或 `poses.txt` 就是可直接使用的准确真值。要进一步确认，需要该会话优化日志、轨迹输出实现或厂商说明。数值结论见 [研究交接](S10_DATA_RESEARCH_ZH.md)，详细原报告位于研究包 `artifacts/xwy-study/research/FINDINGS.md`。

**本地证据与复查**

已保存 29 份板上文本文件以及 7 组只读命令结果：inspection.json（`artifacts/xwy-study/research/slam-106-evidence/inspection.json`）。其中记录各命令的退出码和错误信息，个别目录查询非零不代表 SSH 登录失败。

历史复查脚本：inspect_slam_106.py（`artifacts/xwy-study/research/inspect_slam_106.py`），依赖 50 号原有连接配置，不用于当前 48 号。当前可分发的只读替代工具是 [check_s10_slam.py](../scripts/check_s10_slam.py)：在 AGX 采样话题，在已登录的 106 上用 `--inventory` 读取安装文件，不依赖 `xwy` 或本地研究包。
