# Devlog

## 记录约定

- 调用时机：开发过程中持续记录，在做出决策、完成尝试、遇到或绕过问题、改变依赖和路径时立即追加。
- 记录内容：按时间顺序写决策、尝试、踩坑、修复、小技巧、临时绕过、环境版本、代理、关键路径和验证证据。
- 维护方式：只追加事实；后续推翻旧结论时保留原记录并追加更正，不重写开发历史。

## 2026-08-14 — Jackdev 首次环境与控制链路交付

### 1. 基线检查与方案收敛

- 从干净的 `main` 创建并切换到 `Jackdev`；工作区根目录为 `D:\Desktop\Code\goai26-s10-racing`。
- 确认上游 SDK 已有基于终端 `termios` 的 `KeyboardInterface`，键位已经覆盖 Z 站起、C 进入 RL、W/A/S/D 移动、Q/E 转向，因此不新增键盘 ROS 节点或依赖。
- 决定只在上游 `main.cpp` 按 `S10_MANUAL` 环境变量选择 `kKeyBoard` 或 `kRosTopic`；未设置变量时保持现有自动比赛流程。
- 新增 `.gitattributes`，强制 `.sh` 和 `.py` 使用 LF，避免 WSL 下脚本因 CRLF 失败。

### 2. 代码实现

- `scripts/patch_upstream.py` 改为幂等地加入运行时输入源选择；连续应用两次后第二次为 no-op，`--check` 通过。
- `scripts/run_race.sh --manual` 延迟启动仿真、前台运行 `rl_deploy`、不启动 waypoint follower，并集中回收子进程。
- ROS Jazzy 的 setup 脚本不兼容 `set -u`，所以 `scripts/build.sh` 和 `scripts/run_race.sh` 只在 source 前临时关闭 nounset，source 后立即恢复。
- 给 `s10_auto_nav` 和 `s10_perception` 增加标准 `setup.cfg`，把 console scripts 安装到 `lib/<package>`；否则 `ros2 run` 只能看到 `bin/` 下的脚本并报 `No executable found`。
- 两个 Python 包声明 `tests_require=["pytest"]`，让 `colcon test` 使用 pytest 收集现有测试。
- `scripts/build.sh` 在 `.venv` 存在时使用 `.venv/bin/python -m colcon`，使生成的 ROS console script 绑定虚拟环境解释器，而不是看不到 MuJoCo 的 `/usr/bin/python3`。

### 3. WSL、ROS 与依赖安装

- 保留原有 C 盘 Ubuntu 20.04 和 Docker，不移动、不删除。
- 新建 WSL 2 发行版 `Ubuntu-24.04`，注册路径为 `D:\WSL\Ubuntu-24.04`，默认 UID 1000，默认用户 `goai2026`。
- 系统实测为 Ubuntu 24.04.4 LTS；ROS 环境为 Jazzy；项目使用 Python 3.12。
- 安装 ROS 基础开发环境、构建工具、`python3-venv`、OpenGL/GLFW、SciPy 和 CFFI 等运行依赖。
- 项目 `.venv` 使用 `--system-site-packages`，最终关键版本为 MuJoCo 3.11.0、NumPy 1.26.4、SciPy 1.11.4、Ruff 0.16.3，并可导入 `rclpy`。
- `.bashrc` 只自动 source `/opt/ros/jazzy/setup.bash`，不自动激活项目 `.venv`。

### 4. 下载与代理记录

- ROS 官方源下载过慢，在确认 apt 尚未进入 dpkg 写入阶段后中止，切换到中科大 ROS 2 镜像继续安装。
- rosdep 索引也切换到中科大镜像；镜像没有可用的 `deb-src` 索引，因此只保留二进制源。
- WSL NAT 下不能直接把 Windows `127.0.0.1` 当代理地址；上游 Git 拉取改由 Windows Git 使用 `http://127.0.0.1:7897`，约 5 秒完成。
- PyPI 大包在 `/mnt/d` 安装时有明显文件系统开销；优先复用已下载 wheel，系统可提供的 SciPy/CFFI 改用 apt，减少重复下载。

### 5. 联调踩坑与修复

- 第一次手动联调失败于 `ros2 run s10_perception sim_node` 找不到可执行文件，根因是两个 ament Python 包缺少 `setup.cfg`，不是 ROS topic 或键盘接口缺失。
- 修复入口后，console script 仍由系统 Python 生成并报 `No module named mujoco`；改用虚拟环境解释器执行 colcon 后解决。
- 上游仿真隐式导入 `scipy.spatial.transform.Rotation`，README 原依赖列表没有 SciPy，因此补充 SciPy 安装说明。
- MuJoCo 3.11 给 `mj_multiRay` 增加了 `normal` 参数，旧调用在第一帧感知发布时抛出 `TypeError`；传入 `normal=None` 后仿真稳定运行。
- SIGINT 时 `rclpy` 可能已由 ROS signal handler 关闭，再次 shutdown 会报错；在 `rclpy.ok()` 为真时才调用 shutdown。
- RL 控制器拥有工作线程，但上游 `QwStateMachine::Stop()` 没有调用当前状态的 `OnExit()`；补丁在销毁前回收该线程，消除 `terminate called without an active exception`。
- 清理 trap 最初会在 signal 和 EXIT 各执行一次；改为 signal 只退出、EXIT 单点清理，并用布尔保护保证幂等。
- 一次中止的 pip SciPy 安装已先完成依赖解析，悄悄把 NumPy 升到 2.5.2；随后固定回 1.26.4、卸载要求 NumPy 2 的 pip SciPy，使用 Ubuntu 的 SciPy 1.11.4，并以 `pip check` 确认无冲突。

### 6. 图形、控制与自动模式验证

- `/dev/dxg` 和 NVIDIA RTX 4060 可见；设置 `GALLIUM_DRIVER=d3d12` 与 `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` 后，`glxinfo -B` 显示 direct rendering 和 Accelerated 均为 yes。
- MuJoCo viewer 已通过 WSLg 打开；上述 GPU 环境变量写入 `goai2026` 的 `.bashrc`。
- 手动模式实际进入 `KeyboardInterface`，日志确认 `idle_state -> standup_state -> rl_control`；测试依次发送 W/A/S/D/Q/E，并在松键超时后采样里程计。
- 使用 SIGINT 模拟 Ctrl+C 后明确出现 `[KEYBOARD] Stopped.` 和单次 `Shutting down`，终端与仿真子进程正常清理。
- 自动模式同时出现 `/mujoco_simulation`、`/rl_deploy`、`/s10_cmd_bridge`、`/waypoint_follower`；`/cmd_vel` 有输出、`/robot_state` 为 6、里程计更新并到达第 1 个 waypoint。

### 7. 最终质量门

- `bash -n`、Ruff check、Ruff format check、补丁幂等检查、赛道配置检查全部通过。
- 完整 `colcon build` 成功构建 `drdds`、`s10_auto_nav`、`s10_perception`、`s10_bringup`、`s10_sdk_deploy` 共 5 包。
- 直接 Python 测试 35 项通过；`colcon test` 24 项通过，0 error、0 failure、0 skipped。
- `ros2 pkg executables` 可发现 `s10_perception sim_node` 和 `s10_auto_nav waypoint_follower`。
- 本次改动已暂存但未提交；分支为 `Jackdev`。
- 私密密码不能代填：`goai2026` 当前密码状态仍为 locked，已打开交互终端等待用户执行 `passwd`。

### 8. 文档维护机制

- 新建 Changelog、Devlog 和 Todo 三份本地 Markdown 文档；不引入生成脚本，后续按各文件顶部约定直接追加。

### 9. Ubuntu 账户收尾

- 用户在交互终端中完成 `goai2026` 私密密码设置；`passwd -S goai2026` 返回 `P`，此前的账户锁定事项已解除。
- 密码内容未被读取、记录或写入仓库；Ubuntu 24.04 WSL 2 环境交付完成。

### 10. 低帧率诊断

- WSL 交互环境已继承 `GALLIUM_DRIVER=d3d12` 和 `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`；`glxinfo -B` 确认 renderer 为 RTX 4060、direct rendering 与 Accelerated 均为 yes，不是 llvmpipe 软件渲染。
- WSL 可见 Ryzen 9 7945HX 的 32 个逻辑 CPU；当前没有运行中的仿真进程，因此未把诊断时的系统总 GPU 占用误当作 MuJoCo 独占数据。
- 上游主循环为 1 kHz 物理、每 10 步同步一次 viewer（目标 100 Hz）；感知层每 20 步计算 8×64 lidar 和 13×9 高度图（50 Hz）。
- 无窗口微基准：1000 次 `mj_step` 共 0.131 秒；512 射线 lidar 每次 0.894 ms；117 射线高度图每次 0.895 ms，折算每仿真秒约占 220 ms 单核时间，尚不足以单独解释个位数视觉 FPS。
- 结论：经典 MuJoCo 的 OpenGL 渲染使用 GPU，但本项目的物理、控制和 `mj_ray`/`mj_multiRay` 走 CPU；若画面明显低于正常刷新率，优先怀疑 WSLg viewer 的 100 Hz 同步或窗口显示链路。
- 官方存在 MJX 和 MuJoCo Warp GPU 后端，但它们面向大规模并行仿真，不是当前 ROS 2、交互 viewer、SDK 状态机和传感器代码的即插即用开关；暂不为单实例手动驾驶做高风险迁移。

### 11. 运行中实例现场采样

- 用户当前实际运行命令为 `scripts/run_race.sh --manual --headless`；`sim_node` 占用约 99% 单核，`rl_deploy` 约 47% 单核。
- `/ground_truth/odom` 实测约 45.25 Hz（设计 50 Hz），`/JOINTS_DATA` 实测约 181 Hz（设计 200 Hz），两者比例一致，折算仿真实时因子约 0.91。
- 无窗口模式用 topic 实际频率而非画面判断卡顿：接近 50/200 Hz 即接近实时，明显低于目标才是物理循环变慢。
- 手动键盘读取来自启动终端的 stdin；点击或拖动 MuJoCo viewer 后焦点离开终端，WASD 会被 viewer 接收而不是 `KeyboardInterface`。
- WASD 只在日志已经出现 `standup_state ------------> rl_control` 后生效；Z/C 只是请求状态转换，站立插值按仿真时间完成。

### 9. 密码初始化交互

- `passwd` 在 `New password:` 阶段关闭终端回显，输入时没有字符或星号是正常行为，并非无响应。
- 首次打开的交互进程没有收到用户侧 Ctrl+C，确认 PID 后以 SIGKILL 结束卡住的 `passwd goai2026`；密码状态仍为 `L`，未产生半完成配置。
- WSL 的 localhost 代理警告与密码流程无关；它只说明 NAT 模式不会把 Windows 的 `127.0.0.1` 代理自动映射进 WSL。

### 12. RL 模式 WASD 无响应修复

- 现场确认 `rl_deploy` 的 stdin 指向启动终端且终端已进入 `-icanon -echo` 原始输入模式，因此问题不是窗口焦点、管道或 `termios` 初始化失败。
- 用户确认状态机日志已到 `standup_state ------------> rl_control`；随后在 20 秒人工按 W/S 窗口内采集 1001 帧 `/JOINTS_CMD`，四个轮电机目标速度只有约 0.02 的漂移，证明按键没有进入策略输入。
- 终端版 `KeyboardInterface` 只有在 `MotionStateFeedback` 返回 RL 时才计算速度，但反馈字段由状态机线程写、键盘线程读且没有同步；状态机本身又会在非 RL 状态忽略速度，所以这层重复判断既无必要又会制造陈旧读取风险。
- 对照上游 `keyboard_interface_sim.hpp` 的做法，采用最小修复：键盘线程始终把 held-key 结果写入 `UserCommand`，不新增 ROS 节点、topic、依赖或另一套状态同步机制。
- 修复被加入 `scripts/patch_upstream.py` 的幂等补丁，首次应用、第二次 no-op 和 `--check` 全部通过，`s10_sdk_deploy` 增量构建成功。
- 使用独立 `ROS_DOMAIN_ID=23` 和伪终端启动 `--manual --headless`，自动完成 Z、C、持续 W、持续 S；W/S 分别采到 101/100 帧，四轮均值最大差为 31.731，确认相反方向命令已真正改变策略输出。
- 第一次自动测试被外层 10 秒超时中止并遗留独立进程组；按已核对的 PGID 只清理测试组后提高超时重跑，用户原有 PGID 597 全程未动。
- 用户当前运行的是修复前已加载的旧二进制进程；即使磁盘上的安装产物已经重编译，也必须先 Ctrl+C，再重新运行 `scripts/run_race.sh --manual` 才会加载修复。

### 13. viewer 与手动感知加速

- 沿 `run_race.sh -> s10_perception.sim_node ->` 上游 `MuJoCoSimulationNode.start()` 检查执行路径，确认 viewer 唯一同步点是 1 kHz 循环中的 `step % RENDER_INTERVAL`，lidar 与高度图唯一计算入口是 `_publish_perception()`。
- 采用最小改动：通过幂等上游补丁把 `RENDER_INTERVAL` 默认值从 10 改为 17，理论同步率从 100 Hz 降到约 58.8 Hz；保留 `S10_RENDER_INTERVAL` 环境变量作为不同显示链路的校准入口。
- `--manual` 只导出 `S10_USE_PERCEPTION=0`；`sim_node` 仍以 50 Hz 发布 ground-truth odom，但在调用 lidar 和高度图 sampler 前返回，自动模式未设置该变量，因此继续使用完整感知。
- 新增一条最小 pytest，直接调用感知发布路径，验证关闭感知时 odom 仍被调用且代码在访问 raycast 对象前返回；直接 pytest 与 `colcon test` 均为 1/1 通过。
- `s10_perception` 增量构建成功，Ruff、格式、Bash 语法、补丁重复应用、默认 interval 17 与覆盖 interval 33 检查全部通过。
- 第一次隔离运行只等待 5 秒，模型尚未完成初始化，故没有把 odom 警告误判为代码失败或成功；延长等待后确认 `/JOINTS_DATA` 与 `/ground_truth/odom` 均有消息，而 `/scan` 在连续 3 秒内无消息。
- 隔离验证使用 `ROS_DOMAIN_ID=24`；外层超时未自动回收 WSL 子进程后，按已核对的 PGID 8477 单独清理，用户原有 PGID 7602 未受影响。
- 可见窗口流畅度仍需用户重启后主观确认；当前运行中的旧进程不会热加载 Python 安装产物或上游模块常量。
- 首次从仓库根目录执行全量 pytest 时没有加入 `training`，收集阶段报 `s10_rl` 不可导入；随后两次尝试直接覆盖 `PYTHONPATH`，又分别遗漏 ROS 源码包和 `rclpy` 路径，均属于测试命令环境错误而非测试断言失败。
- 最终按两个既有运行环境拆分测试：ROS 源码套件沿用 workspace overlay，训练套件单独使用 `PYTHONPATH=training`；结果直接 Python 25+11 共 36 项通过，colcon 25 项通过且 0 error、0 failure、0 skipped。
- 完整增量构建再次成功构建 `drdds`、`s10_auto_nav`、`s10_perception`、`s10_bringup`、`s10_sdk_deploy` 共 5 包。

### 14. WSLg 窗口持续卡顿的 renderer 根因

- 用户反馈 60 Hz 与 30 Hz viewer 都卡且拖动窗口延迟；这种现象与同步频率无关，重新检查当前 WSL OpenGL 后发现 renderer 实际为 `llvmpipe (LLVM 20.1.2)`，即 CPU 软件渲染。
- `/dev/dxg`、WSLg 1.0.71、D3D12 库和 `d3d12_dri.so` 均存在；显式设置 `GALLIUM_DRIVER=d3d12` 与 `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` 后，renderer 立即恢复为 `D3D12 (NVIDIA GeForce RTX 4060 Laptop GPU)`。
- 两个变量此前写在 `/home/goai2026/.bashrc` 的交互路径中，`bash -lc` 等非交互启动不会可靠执行它们，因此把默认值移到项目 `run_race.sh`，同时保留用户通过外部环境覆盖适配器的能力。
- 有窗口启动现在先打印 renderer，便于以后第一眼区分硬件渲染和 `llvmpipe` 回退；没有引入额外检测服务或配置文件。
- RTX D3D12、约 59 Hz viewer 的隔离仿真中 `/JOINTS_DATA` 约 148 Hz，说明 WSLg viewer 仍会拖慢当前单线程主循环，但不再承担软件光栅化；后续若窗口仍不够流畅，应处理显示与物理耦合而不是继续降低同步频率。
- `vblank_mode=0` 对照约为 147 Hz，没有改善，因此不把该变量加入启动环境，避免无收益地提高渲染线程负载。
- 使用清空 GPU 变量的 `ROS_DOMAIN_ID=29` 启动验证，脚本仍打印 RTX 4060 D3D12；8 秒超时发生在模型初始化期间，产生预期的中断堆栈，但退出后确认没有残留测试进程。

### 15. 手动 viewer 与仿真进程解耦

- 用户确认恢复 RTX renderer 后窗口仍卡；正在运行的旧架构实例中 `sim_node` 约占 151% CPU，`/JOINTS_DATA` 实测约 165 Hz，说明剩余瓶颈是 viewer 与物理主循环耦合，而非软件光栅化。
- 新增 `s10_perception.viewer_node`，直接复用现有 `/ground_truth/odom`、`/JOINTS_DATA`、上游 MJCF 路径、关节方向与位置偏移，不增加 topic、第三方依赖或第二份物理仿真。
- viewer 进程只维护用于显示的 `MjData`：odom 更新基座位置和四元数，关节消息按上游 `raw = published * direction + offset` 还原 qpos，再以默认 60 Hz 执行 `mj_forward()` 和 `viewer.sync()`。
- `run_race.sh --manual` 现在强制实际 `sim_node` 使用 `S10_USE_VIEWER=0`，按需在后台启动 detached viewer，而 `rl_deploy` 继续占用前台终端读取 Z/C/WASD/QE；`--manual --headless` 不启动 viewer。
- 新增一条关节坐标转换测试；直接 pytest 与 colcon 均为 2/2 通过，`ros2 pkg executables s10_perception` 能发现 `sim_node` 和 `viewer_node`。
- 使用 `ROS_DOMAIN_ID=30` 同时运行独立 sim/viewer，ROS 图确认 `/mujoco_simulation`、`/mujoco_viewer` 及两条订阅连通；viewer 保持打开时 `/JOINTS_DATA` 平均约 187 Hz，高于旧架构现场约 165 Hz。
- 第一次清理脚本中的 `$alive` 被外层 PowerShell 展开为空，条件表达式报错且未执行删除；随后改用明确 PGID 11254/11319 的 SIGINT、短等待和 SIGTERM，仅清理测试域，用户原 PGID 10784 未动。
- 该方案优先保证物理和控制不被窗口拖慢；如果 detached viewer 自身仍因 WSLg 看起来掉帧，下一层才是 Windows 原生显示桥接，而不是再次修改控制循环。
- 最终质量门通过：5 个 ROS 包构建成功，Ruff 与格式检查通过，ROS 源码 26 项加训练 11 项共 37 项 Python 测试通过，colcon 26 项通过且 0 error、0 failure、0 skipped。

### 16. WSLg/MuJoCo 可见窗口掉帧的公开问题核验

- 用户确认 detached viewer 仍有明显掉帧，因此本轮只做公开资料检索和本机只读核验，不继续盲调物理步长、传感器或控制循环。
- 按 `yichen-unified-search` 路由尝试 AnySearch 时发现 `C:\Users\Lenovo\.agents\skills\anysearch\runtime.conf` 缺失；由于任务只涉及公共网页且用户明确要求联网，透明回退到内置公共搜索，并逐一核验原始 GitHub 文档、issue 和源码。
- WSLg 官方架构说明指出 OpenGL 加速由 Mesa D3D12 Gallium 提供；在独显上，渲染数据会从 VRAM 复制到系统内存，再在 Windows 侧上传到 GPU，性能损耗与画面呈现频率成比例：https://github.com/microsoft/wslg 。
- WSLg Discussion #146 有与本机症状相似的案例：`glxinfo` 已显示 NVIDIA D3D12 和 hardware accelerated，但 OpenGL 窗口只有约 20 FPS，甚至在部分负载下慢于 llvmpipe：https://github.com/microsoft/wslg/discussions/146 。
- WSLg Discussion #312 说明 `[WARN: COPY MODE]` 代表 RAIL 像素复制而不是 VAIL 共享内存，并记录了 `wsl --update`、`wsl --shutdown` 或重启后恢复的案例；本机 `/mnt/shared_memory` 不存在，与旧报告一致但不能仅凭这一点断言当前一定处于 COPY mode：https://github.com/microsoft/wslg/discussions/312 。
- WSLg Issue #272 记录了 NVIDIA D3D12 已加速时，XWayland GPU 窗口拖动/调整大小仍会冻结或停止刷新的问题，说明“GPU 生效”不能排除 WSLg 窗口层问题：https://github.com/microsoft/wslg/issues/272 。
- MuJoCo Python viewer 源码直接导入 GLFW，并在 viewer 侧运行 `render_loop()`；因此 GLFW 选择 X11 还是 Wayland 会实际影响本项目的可见窗口路径：https://github.com/google-deepmind/mujoco/blob/main/python/mujoco/viewer.py 。
- 本机 `DISPLAY=:0`、`WAYLAND_DISPLAY=wayland-0`；默认导入的 Python GLFW 报告 `3.4.0 X11 GLX Null EGL OSMesa monotonic shared`，说明 MuJoCo 当前确定走 X11/XWayland。
- `.venv/lib/python3.12/site-packages/glfw/` 同时包含 `x11` 与 `wayland` 两套动态库，Python wrapper 支持通过 `PYGLFW_LIBRARY_VARIANT` 选择；设置 `PYGLFW_LIBRARY_VARIANT=wayland` 后实测版本变为 `3.4.0 Wayland Null EGL OSMesa shared`，`glfw.init()` 返回 1 且平台确认为 Wayland。
- 最小下一步是不安装包、不改代码，先在停止旧实例后执行 `PYGLFW_LIBRARY_VARIANT=wayland scripts/run_race.sh --manual` 做主观流畅度对照；若失败，可立即删除该临时环境变量回到 X11。
- 本机屏幕为 2560×1600、240 Hz；基于 WSLg 官方“损耗与 presentation rate 成比例”的描述，降低 Windows 刷新率到 60/120 Hz 是有依据的后续诊断，但它属于推断，不能在 Wayland 对照前宣称一定有效。
- `wsl --shutdown` 会同时停止该用户的所有 WSL 发行版及 Docker 相关实例，虽然成本较低且有公开案例支持，但未在本轮擅自执行。

### 17. Windows 原生 MuJoCo viewer

- 用户完成 Wayland 对照后仍反馈卡顿，因此停止继续调 WSLg 参数，手动模式改为绕过 WSLg/XWayland；自动模式和 `--manual --headless` 的原路径不变。
- 采用最小架构：WSL 中现有 `viewer_node` 增加 UDP stream 模式，继续订阅 `/ground_truth/odom` 和 `/JOINTS_DATA`，每帧发送固定网络字节序的 23 个 double；Windows 脚本只加载同一 MJCF、接收最新 datagram、更新基座与 16 个关节并调用 `viewer.sync()`。
- 选择 UDP 而不是 TCP，是为了窗口偶发阻塞后直接丢弃旧姿态、只显示最新一帧，避免积压数秒的历史动画；固定帧只有 184 字节，不需要 JSON、消息代理或 ROS for Windows。
- 先用 PowerShell UDP listener 与 WSL `172.19.64.1` 网关实测，12 字节 `viewer-probe` 完整到达，确认当前 NAT 网络下 WSL 到 Windows 宿主机链路可用。
- Windows 原有 `D:\Anaconda\python.exe` 3.12 能创建 venv，但它随进程优先加载的 MSVC 14.29 覆盖系统 14.51，导致 MuJoCo 3.11 的 `mujoco.dll` 报 WinError 1114；没有替换 Anaconda 全局 DLL。
- 尝试机器已有的 MSYS2 UCRT Python 3.11 时，其 wheel tag 是 `mingw_x86_64_ucrt`，不能安装官方 `win_amd64` MuJoCo wheel，因此删除这两个临时无效环境，不为一个 viewer 引入源码编译。
- 复用机器已有 `uv 0.11.7`，设置 `UV_PYTHON_INSTALL_DIR=D:\DevTools\uv-python` 和 `UV_CACHE_DIR=D:\DevTools\uv-cache`，在 D 盘安装独立 CPython 3.12.13，并重建项目 `.venv-win`。
- `.venv-win` 固定安装 NumPy 1.26.4、MuJoCo 3.11.0 及其 wheel 依赖；赛道模型加载得到 `nq=30`，协议只更新前 23 个机器人 qpos，其余模型自由度保留本地默认值。
- Windows GLFW 报告 `3.4.0 Win32 WGL`；隐藏 OpenGL context 实测 renderer 为 `NVIDIA GeForce RTX 4060 Laptop GPU/PCIe/SSE2`、OpenGL 4.6.0 NVIDIA 580.88，确认显示完全绕过 WSLg。
- `run_race.sh --manual` 默认选择 Windows backend，通过 `ip route` 获取宿主网关并用 `wslpath` 传递脚本和 MJCF 的 Windows 路径；`S10_VIEWER_BACKEND=wsl` 保留为回退，`S10_VIEWER_PORT` 与 `S10_WINDOWS_HOST` 可在实际网络变化时校准。
- Windows viewer 对 UDP 包做固定长度和有限数值检查；收到首帧会打印 `Viewer stream connected`，流启动 30 秒仍无数据或已连接后中断 2 秒会退出，避免孤立静态窗口。
- 第一次测试误用系统 `/usr/bin/pytest`，因看不到 WSL `.venv` 的 MuJoCo 而在收集阶段失败；改用 `.venv/bin/python -m pytest/colcon` 后同一套测试全部通过，属于命令环境错误而非代码失败。
- 原生窗口 smoke test 使用端口 18781，确认 Windows listener 与 WSL sender 连通并在 12 秒超时后无残留 Windows Python；真实手动联调用独立 `ROS_DOMAIN_ID=43`、端口 18782，确认 `Viewer stream connected` 和 `idle_state -> standup_state`。
- 联调期间检测到用户原有 `run_race.sh --manual` 仍在运行，未终止或复用其进程；测试进程组以 SIGINT 清理并返回 130，用户需自行 Ctrl+C 后重新启动才能加载新 backend。
- 代码复查发现 stream loop 若每帧只 `spin_once()` 一次，会把 50 Hz odom 与 200 Hz joints 合计 250 Hz 的回调错误限流到 60 次/秒；改为像原 detached viewer 一样持续排空 ROS 回调，仅把 UDP 发送节流到 60 Hz。
- 第一次 UDP 帧数检查只给 3 秒，进程还在导入上游 MuJoCo/SciPy 模块就被 timeout，因而收到 0 帧；延长到 12 秒后 Windows 端连续收到 10 个长度正确的 184 字节 qpos datagram。
- 最终 Ruff、format、Bash 语法、补丁幂等检查通过；5 个 ROS 包构建成功，ROS 源码 27 项与训练 11 项共 38 项直接 Python 测试通过，colcon 27 项通过且 0 error、0 failure、0 skipped。
- 用户随后停止旧实例并实际运行默认手动命令，反馈 Windows 原生 viewer “现在很流畅了”；这完成了无法由自动化替代的可见窗口主观验收，也反证继续优化 WSLg 参数没有必要。


## 2026-09-09 — 历史数据与 SLAM 调查整理为团队交接

- 将此前 9 轮历史录制审查、关节映射、短时 MuJoCo 跟踪、假设楼梯匹配和地图重建结论汇总到 `docs/S10_DATA_RESEARCH_ZH.md`；保留上楼未通过、关节反馈不等于教师动作等限制。
- 经 AGX 只读登录 106，包元数据确认 `slam 3.5.1` / `slam-common-lib 1.1.3`；配置、头文件、更新日志支持 LIO＋回环＋GTSAM 位姿图优化。FAST-LIO／Faster-LIO 对应关系是来源推断，不能从旧注释证明现行二进制实现。
- 安装程序由 `drsec exec` 处理加密封装；本轮未运行它。此前在线与保存轨迹刚体对齐 RMSE 约 3.45 m，回环/优化是合理解释机制，但具体历史会话原因仍待证据。
- 将 SLAM 文档中的 Windows 绝对文件链接改为研究包相对路径说明，避免 GitHub 读者遇到仅本机有效的链接；地图预览 PNG 随文档保留。
- README 与采集工具文档指向当前 50 号独立采集站，旧 xwy 入口标记为历史；移除本地演示的过期 PID，修正失效的 demo 脚本路径。
- `artifacts/`、`tmp/`、`.playwright-cli/` 加入忽略规则；原始 bag、下载的第三方源码/模型/板上文件与完整分析输出单独共享。没有改写原始录制，没有修改机器人服务，没有暂存、提交或推送。


## 2026-09-10 — 按用户授权撤销 50 号独立采集部署

- 清理前采集 API 显示没有正在录制的会话，远端 data 中没有新 manifest 或 db3；本地已转存的 17 段数据保留。
- 经 AGX 以太网连接停用采集服务，确认进程退出后删除独立服务单元与部署目录；仅移除记录的热点 UUID，恢复无线关闭状态，其余 NetworkManager 连接保留。
- 复查目录不存在、服务 not-found/inactive、8091 无监听，并成功通过原以太网 SSH 重连；xwy 原目录和系统日志保留。
- 本机两份口令的批量清理命令被自动审批拒绝，返回 blocked by policy；改用逐个明确 LiteralPath 的删除后成功，并确认两文件不存在。
- Windows 与 AGX 的现有公钥都不能登录 103，旧中转目录没有核对或删除；部署文档及 TODO 明确保留该未完成项，没有声称全部清理完成。


### 103 中转清理补完

用户补充本体登录口令后，成功登录 103；口令仅用于内存中的 SSH 认证。目录内四个文件确认属于临时 8091 中转，没有运行进程或 8091 监听；核实精确路径、所有者和文件清单后删除目录。原 s10-gait-hotspot 与 s10-mapping-hotspot 服务保持 active，先前的 103 认证阻塞已解除，既定清理清单完成。原系统日志、本地录制及研究资料保留。


### 新增文件盘点后的 Tailscale 清理与说明

用户明确要求删除 Tailscale，其他项为影响咨询。先检查 SSH 回程与 apt 模拟，再只 purge tailscale 和 tailscale-archive-keyring，清理专属源与残余缓存；软件、状态目录、服务和虚拟网卡均已移除，其他包未 autoremove。公钥名单缺修改前副本，不能断言过去是否有其他条目；本轮未修改。106 后端与 AGX 源及两份本地快照完全一致，创建时间刷新不等于代码更新；零字节临时上传文件、其他缓存和日志均仅核对。

### 用户授权 9 月 7 日 15:00 后 Matplotlib 字体缓存清理

三板搜索后仅 102 发现字体缓存 `fontlist-v330.json`，创建于北京时间 2026-09-08 13:21:08。按新授权核对精确路径、inode/大小/修改时间未变、创建时间和 JSON 字体列表格式后删除 152,295 字节文件，确认路径不存在；103、106 搜索没有匹配文件，其他缓存未动。逐项记录位于 `artifacts/s10-050-cleanup-20260910/matplotlib-removal.json`。

### 用户授权 9 月 7 日 15:00 后日志清理

按北京时间 2026-09-07 15:00 筛选三板日志，删除 399 个已关闭文件并清空 18 个正在追加的文本日志，处理前逻辑大小合计 1,862,841,038 字节。按日期、记录时间和 journal 头时间判定；没有仅凭最近修改时间删除文件。journal 已同步和轮转，被占用的 journal、旧新混合记录、边界日期或时间不明的日志保留；未停止日志服务，复查时 106 已继续生成新记录。三板执行无错误，复查没有符合规则的已关闭文件剩余，程序、公钥、缓存和数据不在本次操作范围。详情见部署文档及本地 `artifacts/s10-050-cleanup-20260910/LOG_CLEANUP_RESULT.md`，不声称清除了全部日志痕迹。

## 2026-09-11 — 更正当前 48 号环境并补齐独立工具

用户明确当前使用 48 号，xwy 属于 50 号其他队伍。通过本机 s10-48-golai 别名成功登录，确认 golai UID 2003、xwy 不存在、三个传感器服务 active；106 可达，但 user 的现有公钥认证失败，未取得该板有效登录方式。

新增 check_s10_slam.py，替代依赖 50 号建图网页配置和本地 artifacts 的历史复查入口。实际 6 秒采样得到 ODOM 60 条约 10 Hz、IMU 1206 条约 200.8 Hz，消息时间戳落后 AGX 系统时间约 15,678,553 秒；LIO_ODOM 有发布者但无数据，SLAM_ODOM 无发布者，两个点云入口未发现发布者。记录是检查窗口结果，未据此认定所有厂商服务缺失或地图已可用。

修正 GUI 的 xwy sudo、50 号标题和临时 ROS 路径，支持当前 SSH 配置中的设备别名、专用密钥和独立 HostKeyAlias。准备脚本复制本机官方 drdds 源码到 /home/golai/s10_control_ws，补五个运动状态类型，36 秒完成 ARM 构建；后续实际 GUI read 返回 error=null、state=0、zero_sent=false。构建后单独 import 检查的第一次 shell 转义写错，后续真实 worker 成功导入并读到状态；没有启动 rl_deploy 或执行运动。

README、48 号指南与旧文档入口已更新。50 号算法来源保留证据边界，不改写为 48 号已验证版本。未改厂商定位／建图、雷达或相机服务；未复制另一队的网页、账号或系统配置。106 登录与真实建图保存验收仍待补齐。


### 48 号 106 登录核实与手机入口迁移

用户补充口令后，已登录 48 号 106，确认 slam 3.5.1、slam-common-lib 1.1.3、官方 drmap/map_manager 和可用雷达/IMU。106 本机 8 秒收包约 10.4/200.5 Hz；AGX 未发现雷达不代表 106 没有数据。官方 start/stop dry-run 通过；106 日期为 2026-03-14，与 AGX 不一致，未改时钟。

首次尝试把手机版放在 106；用户要求参照 xwy 本地副本并减轻 106 负载后，停用并禁用该服务。阅读最新备份 HOTSPOT_README.md、web_server.py、quick_mapping.py、robot_backend.py，确认旧部署为 103 转发、102 网页、106 LIO。48 号改成原生 socket-proxyd 的 103:8080 入口，网页部署在 102 golai，自启和 linger 已启用；106 只按需执行受限 SSH 后端。没有复制 xwy 账号或其永久 mask localization 行为。

数据订阅直接从 AGX 使用单独 DDS profile 的尝试仍未发现官方雷达，因此最终采用 SSH 按需读取。106 的 Python ROS 进程继承 SLAM 专用 LD_LIBRARY_PATH 时异常退出，改用系统 Python/ROS/Numpy，厂商库路径仅用于保存命令后通过。Windows 生成 bash 的 CRLF 导致首次启动失败，已统一 LF。修复 SSH 流 EOF 后的忙重试，异常时退避并明确显示离线。

通过 103 HTTP 登录后取得真实雷达约 890 点/帧和 IMU、地图列表；非法地图名与缺少 CSRF 被拒绝。点云大小端、行填充、截断/NaN 和保存失败不停止服务检查通过。将 raw=True 的订阅限频放在反序列化之前，106 未建图预览的 10 秒测量由单核 58.9% 降到 49.6%（8 核，RSS 约 181 MiB），不会改动官方 SLAM 输入频率。后台累计图在 AGX，106 无人查看约 30 秒后退出。用户询问精度，页面及指南已明确区分手机预览与官方地图。

本次未实际启动或保存新地图、未切换 active 地图、未发送运动指令；官方 localization 持续运行。浏览器自动化后续超时，真实手机显示及走场建图保存仍需现场验收；没有重启整机验证自启。代码和文档均留在工作区，未提交推送。

最终同步后经 103 再次确认真实雷达 891 点/帧、IMU、11 项地图列表和 CSRF 拒绝；首次状态读取早于 IMU 发现，验收脚本改为等待两路输入后通过。关闭查看后确认 106 读取进程退出、旧 HTTP 服务 inactive/disabled，旧 HTTP 源文件已撤下，官方 localization 保持 active。

### 48 号 golai 与网页登录密码更新

按用户明确要求设置 AGX golai 系统密码，并同步更新网页登录配置。已用新密码验证 golai SSH 登录及 103 网页登录，旧网页密码返回 401；既有 SSH 密钥仍可用。更新前确认无网页建图操作，随后仅重启 AGX 网页用户服务。凭据未写入源码或文档。

### 48号时间同步排查与启动保护（下午）

读取引用会话后实测发现整机已在约15:31重启，IMU旧时间与10^14米ODOM在本轮修改前已恢复。106连续12秒采样：IMU约200.3Hz、时间差9.4–34ms，点云约10.1Hz、117–281ms，ODOM约10Hz且位姿正常，无时间倒退。106的PTP跟随103，主钟identity为3ede53.fffe.78f3dd；AGX保留原NTP。按DDS GID与PID对应追到IMU源yesense_node、ODOM源localization_ddsnode，均位于106；两个点云端点对应rslidar和hsLidar，尚未取得逐消息写者统计。

106本次开机15秒时大幅校时，yesense约28秒启动；原服务只按network.target排序。新增只读wait_s10_ptp.py及五个启动前drop-in，连续3次确认103主钟、SLAVE和两级偏差在5ms以内才放行，超时不继续启动。保留厂商PTP方向、时钟来源与算法；未尝试进入103宿主机或修复其容器内chrony。最初是谁触发运行中日期跳变仍未确定，不能将启动保护等同于解决所有运行中校时来源。

修复网页只检查接收时间的误判，显示106测量延迟，并在厂商开始操作暂停定位之前重新采样3秒。检查脚本新增时间范围、倒退、相对单调时钟跳变、点内时间抽查、端点GID和--require-ready。离线回归、systemd单元校验、真实临时服务执行启动检查及103热点HTTP验证通过，最后一次PTP记录主从-560ns、PHC/系统-2039ns；原始地图保留。仅reload106、restart AGX网页，未重启驱动、定位或运动控制，新配置整机开机和短闭环留待现场验收。代码、部署前网页副本与采样证据见artifacts/s10-48-time-sync-20260911，未提交推送。


## 2026-09-12 — 当前研究分支解决 main 合并冲突

- 用户确认除 README、build.sh 外，其余七个冲突文件采用 main 当前版本；合并基线为 `fa49f57`，保留真机采集、复核、建图及 GUI 工具。
- 清理旧 replay_waypoint.sh、replay_velocity_profile.sh、test_sim_node.py，路点测试恢复主干版本；旧 WASD 和模型文档标明适用于合并前 `cc93d39`。
- build.sh 合并虚拟环境 colcon 选择及主干的构建输出目录、平台识别、运行版本检查，新增一个覆盖两种启动方式及含空格路径的回归测试。
- WSL 回归初次完成 419 项通过、1 项失败、9 项错误；唯一策略资产失败由 Windows CRLF 转换导致，固定该 JSON 的 LF 并恢复原始字节后单独复测通过，累计 420 项通过。
- 剩余 9 项错误共享 test_joint_owner.py 的 C++ 检查程序；在独立临时目录提取原始 main 的全部 integration 文件及对应测试后，复现同样的 Gate16 控制权检查失败。本次没有改变该主干实现或放宽测试。
- Shell 语法、修改部分 Ruff 检查通过，七个选定文件与 main 一致；本次不执行整场比赛或机器人部署。
