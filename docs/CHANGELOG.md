# Changelog

## 记录约定

- 调用时机：每次形成可交付结果时，与代码或环境交付同步更新。
- 记录内容：每个交付条目固定三句话，依次说明改了什么、为什么改、验证结果与影响。
- 维护方式：LLM 负责在交付时补充；用户可以直接修订任何历史描述。

## 2026-08-14 — Windows 原生 viewer 流畅度验收

1. 用户实际重启并运行 `scripts/run_race.sh --manual` 后，确认 Windows 原生 MuJoCo viewer 画面已经很流畅。
2. 这项验收证明持续掉帧来自 WSLg/XWayland 显示链路，而不是 MuJoCo 物理、ROS 状态流或 RTX 4060 算力不足。
3. 原生 viewer 方案正式保留为手动模式默认路径，WSLg backend 仅作回退，后续只剩完整键位与松键停止的人工确认。

## 2026-08-14 — 手动模式改用 Windows 原生 viewer

1. `run_race.sh --manual` 现在默认启动 Windows 原生 MuJoCo viewer，WSL 侧复用 `viewer_node` 以本机 UDP 只发送最新的 23 个机器人 qpos，并保留 `S10_VIEWER_BACKEND=wsl` 回退。
2. 这样修改是因为 Wayland 对照后窗口仍卡，瓶颈已收敛到 WSLg/XWayland 显示路径，而不是物理、RL 或 ROS topic 频率。
3. D 盘 Windows Python/MuJoCo 环境已安装，Win32 WGL 确认使用 RTX 4060；真实 ROS 状态流联调、5 包构建、38 项直接 Python 与 27 项 colcon 测试全部通过。

## 2026-08-14 — WSLg/MuJoCo 掉帧公开问题核验

1. 检索并核验 WSLg、GLFW 与 MuJoCo 的官方文档、源码和 issue，确认 D3D12 正常时仍存在独显显存往返、RAIL/COPY 传输及 X11 窗口卡顿的同类报告。
2. 这样核验是因为本机物理与显示已解耦但可见窗口仍掉帧，而本机 MuJoCo 实际加载 `GLFW 3.4.0 X11 GLX`，同一 wheel 内置的 Wayland variant 已能在 WSLg 成功初始化。
3. 此轮未修改运行代码，下一步已收敛为用 `PYGLFW_LIBRARY_VARIANT=wayland scripts/run_race.sh --manual` 做最小对照；若仍卡，再测试 Windows 刷新率或 Windows 原生 viewer。

## 2026-08-14 — 将手动 viewer 与物理控制解耦

1. 新增独立 `s10_perception viewer_node`，手动模式改为 headless `sim_node` 加订阅 odom/关节状态的显示进程，并保持键盘 RL 进程在前台。
2. 这样修改是因为即使 RTX D3D12 生效，原有 `viewer.sync()` 仍位于物理主循环中，WSLg 卡顿会直接降低仿真频率和放大控制延迟。
3. 5 包构建、37 项 Python 与 26 项 colcon 测试均通过，隔离运行在 viewer 打开时把 `/JOINTS_DATA` 从旧架构约 165 Hz 提升到约 187 Hz，且物理不再等待窗口。

## 2026-08-14 — 修复 WSLg 软件渲染回退

1. `run_race.sh` 现在自行设置 Mesa D3D12 与 NVIDIA 适配器默认值，并在有窗口启动前打印实际 OpenGL renderer。
2. 这样修改是因为 GPU 配置只位于交互式 `.bashrc`，从非交互 shell 启动时会静默回退到 CPU 的 `llvmpipe`，导致窗口拖动和仿真播放严重卡顿。
3. 清空原有 GPU 环境变量后运行启动脚本仍确认 renderer 为 RTX 4060 D3D12，且关闭 vblank 的对照没有性能收益，因此未保留无效开关。

## 2026-08-14 — 手动仿真显示与感知加速

1. 将 MuJoCo viewer 默认同步间隔从 10 个物理步调为 17 个以接近 60 Hz，并让 `--manual` 保留 odom 但跳过 lidar 与高度图射线计算。
2. 这样修改是为了减少 WSLg 的过高窗口同步和手动驾驶不使用的 CPU 感知开销，同时不改变自动模式的默认感知链路。
3. 补丁幂等、5 包构建、36 项 Python 与 25 项 colcon 测试全部通过，隔离运行确认关节状态与 odom 继续发布而 `/scan` 在 3 秒窗口内为 0 条。

## 2026-08-14 — 修复 RL 模式下 WASD 无响应

1. 修改上游 SDK 补丁，让终端 `KeyboardInterface` 始终发布当前按键速度，并继续由状态机决定何时使用速度命令。
2. 这样修改是因为键盘线程读取未同步的运动状态反馈时可能长期看到旧值，导致界面已进入 `rl_control` 但 W/A/S/D/Q/E 仍被持续清零。
3. 补丁重复应用和部署包编译均通过，隔离仿真实测 W、S 各约 100 帧的四轮命令最大均值差为 31.731，证明相反方向输入已真正进入 RL 策略。

## 2026-08-14 — 手动模式现场频率诊断

1. 对用户正在运行的 `--manual --headless` 实例读取进程占用和 ROS topic 频率，测得 odom 45.25 Hz、关节状态 181 Hz。
2. 这次诊断用于回答无窗口如何判断卡顿，并把 WASD 无响应区分为仿真速度、RL 状态和终端焦点三个独立问题。
3. 结果表明无窗口约为 0.91 倍实时速度，而可见窗口的明显拖动与低 FPS 更偏向 WSLg viewer；后续优化应优先降低 viewer 同步率并在手动模式跳过未使用感知。

## 2026-08-14 — 仿真低帧率诊断

1. 完成 WSLg 渲染器、CPU 配额和 MuJoCo 物理/lidar/高度图的只读性能诊断，确认窗口由 RTX 4060 的 D3D12 硬件加速渲染。
2. 诊断用于区分软件渲染、CPU 物理和 WSLg 显示链路，结果表明当前物理与射线计算仍走 CPU，而原始计算量不足以单独解释个位数画面帧率。
3. 微基准测得物理 0.131 ms/步、512 射线 lidar 0.894 ms/次、高度图 0.895 ms/次，建议先降低 viewer 同步频率或用 headless 验证，再考虑非即插即用的 MJX/Warp GPU 重构。

## 2026-08-14 — Ubuntu 用户密码设置完成

1. `goai2026` 的私密密码已由用户在交互终端中设置，Ubuntu 账户从锁定状态变为可用状态。
2. 这一步用于启用该默认用户的密码认证和 sudo 操作，同时确保密码没有进入聊天、命令历史或项目文件。
3. 已通过 `passwd -S goai2026` 验证状态为 `P`，WSL 2 环境安装交付至此完整结束。

## 2026-08-14 — 开发记录机制初始化

1. 新增 `docs/CHANGELOG.md`、`docs/DEVLOG.md` 和 `docs/TODO.md`，并为三个文件写明各自的调用时机、记录内容和维护责任。
2. 这样修改是为了让每次交付、开发过程和剩余待办分别落到稳定位置，避免关键决策、临时绕过和环境信息只存在于聊天记录中。
3. 已回填本次 Jackdev 分支的全部实现、安装、代理、踩坑、验证和密码待办，后续不需要额外生成器即可继续追加。

## 2026-08-14 — Jackdev：WSL 2 仿真与 WASD 双模式控制

1. 在 `Jackdev` 分支加入 `S10_MANUAL=1` 输入选择和 `scripts/run_race.sh --manual`，保留默认 `/cmd_vel` 自动模式，并补齐 ROS Python 可执行入口、LF 行尾、MuJoCo 3.11 兼容及退出清理。
2. 这样修改是为了复用上游 `KeyboardInterface` 完成 Z/C、W/A/S/D、Q/E 控制，同时解决 Ubuntu 20.04 不兼容 Jazzy、虚拟环境包不可见、ROS 可执行文件找不到和 Ctrl+C 线程异常等实际阻塞。
3. Ubuntu 24.04 WSL 2、ROS 2 Jazzy、MuJoCo 和项目依赖已安装在 D 盘环境并完成 5 包构建，Python 35 项与 colcon 24 项测试通过，手动和自动仿真链路均已联调。
