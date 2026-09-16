# S10 HIM ONNX 共同接口 v1

本文件是训练仓库与 goai26-s10-racing 接入任务共同遵守的接口。两个任务不得各自改变字段、排列或缩放；发现实际模型与契约不符时先报告，并协调两边更新。

## 1. 模型图与张量

- 导出完整确定性推理：estimator.encoder -> 3 维速度估计 + 16 维 latent 的原始 L2 normalize -> actor。保持原模型计算和 normalize 参数。
- 排除 critic、训练 target/prototype、优化器与探索噪声；比较对象是 HIMActorCritic.act_inference。
- 一个输入 obs，float32，固定 [1, 342]；一个输出 actions，float32，固定 [1, 16]。
- ONNX 输出为尚未执行动作裁剪/物理缩放的确定性网络动作。观测构造、裁剪、历史维护、动作裁剪和物理解码在 runner 中执行。
- 使用 FP32，不量化；无隐式跨调用状态。opset 默认 17；以实际 PyTorch/ONNX Runtime 导出并运行验证结果为准。
- 保留现有 TorchScript 一维调用约定，ONNX 包装保留 batch 维。

## 2. 单帧观测与历史

每帧 57 维：
| 切片 | 含义 |
|---|---|
| [0:3] | body-frame angular velocity * obs_scales.ang_vel |
| [3:6] | projected gravity，body frame 中的单位重力方向 |
| [6:9] | 物理单位速度指令 [vx, vy, wz] * commands_scale |
| [9:25] | (q - default_dof_pos) * obs_scales.dof_pos，wheel_indices 的位置误差固定为零 |
| [25:41] | dq * obs_scales.dof_vel |
| [41:57] | 上一次网络输出经 clip_actions 裁剪后的动作；尚未物理缩放，按 HIM 顺序 |

- 整帧使用 clip_observations 对称裁剪；部署不添加训练噪声。
- 六帧展平顺序：[frame_t, frame_t-1, ..., frame_t-5]。每个 frame 自身保持上述字段顺序。
- 每个策略步更新一次历史，当前 policy_dt=0.02 秒；指令仅写入最新帧，不能重写旧帧的指令。
- 进入 RL、切换策略、仿真复位时清空历史及动作缓存；第一次推理：[当前有效帧, 五帧零]，上一动作零。不能重复当前帧六次。
- /sim/reset 若可在 RL 中发生，必须让 runner 同步收到重置；也可通过明确的退出/重入 RL 流程实现。
- 输入始终来自同一套 SDK 机器人坐标与校准链，保留底层电机方向/零位转换。

## 3. HIM 关节顺序与动作

dof_names 的顺序固定为：
fl_hipx_joint, fl_hipy_joint, fl_knee_joint, fl_wheel_joint,
fr_hipx_joint, fr_hipy_joint, fr_knee_joint, fr_wheel_joint,
hl_hipx_joint, hl_hipy_joint, hl_knee_joint, hl_wheel_joint,
hr_hipx_joint, hr_hipy_joint, hr_knee_joint, hr_wheel_joint。

- wheel_indices = [3, 7, 11, 15]；观测中的 q/dq/last_action 及输出动作都采用该顺序。
- 当前等于 SDK robot_order；旧 57/174 维策略仍使用原 policy_order，不得全局改成 HIM 顺序。
- a = clip(raw_actions, -clip_actions, clip_actions)，保存 a 用于下次 last_action。
- 腿关节目标 q_target = default_dof_pos + a * action_scale，目标速度为零。
- 轮子使用速度控制：dq_target[wheel_indices] = a[wheel_indices] * vel_scale，Kp 为零；不是轮子位置控制。
- 网络输出不能再无条件套用旧策略的 policy2robot_idx。

## 4. 配套 JSON（规范字段）

ONNX 与 JSON 同目录、同 stem，例如 policy.onnx / policy.json 或 model_2000.onnx / model_2000.json。JSON 保留当前导出已有字段，增加以下固定字段：
- onnx_contract_version: 1
- policy_type: "s10_him"
- input_name: "obs"
- input_shape: [1, 342]
- output_name: "actions"
- output_shape: [1, 16]
- one_step_observation_dim: 57
- history_length: 6
- history_order: "newest_first"
- policy_dt: sim_dt * decimation（当前约 0.02，浮点比较用容差）

已有字段继续使用原名称：
robot="s10", interface_version=2, reset_history="zero",
dof_names, wheel_indices, default_dof_pos, p_gains, d_gains,
action_scale, vel_scale, torque_limits, dof_vel_limits,
commands_scale, obs_scales, clip_actions, clip_observations,
sim_dt, decimation, self_collisions, initial_position。

- 上述控制参数必须来自所选 checkpoint 对应的保存配置；不能用新的默认配置冒充旧模型配置。
- HIM 342 维模型要求有效的配套 JSON；缺失、形状不匹配或不支持的契约应明确报错。旧 57/174 维模型保留原有无此 sidecar 的加载能力。
- 当前参考 S10：commands_scale=[2,2,0.25]；obs ang_vel=.25、dof_pos=1、dof_vel=.05；clip_actions=clip_observations=100。
- 当前参考默认角：前两腿 [0,-.3,.6,0]，后两腿 [0,.3,-.6,0]。
- 当前参考 p_gains=[80,80,80,0]*4，d_gains=[2,2,2,.6]*4，action_scale=[.125,.25,.25,0]*4，vel_scale=5。
- 当前参考 torque_limits=[50,50,50,14]*4。参数应按 JSON 读取并在实际执行层核验。
- 2026-09-16 参考模型的 sim_dt=.0025、decimation=8。不能把训练 decimation=8 直接复制给基于不同主循环周期的 SDK。部署保持 policy_dt=.02。
- 比赛仿真 PD/物理子步与策略周期是不同概念；不因格式转换盲目更改比赛仿真步长。

## 5. 多模型与校验

- 支持现有 57/174 维策略及 HIM 342 维策略。每个 session 保留自身输入宽度/类型；切换时同步输入形状、观测、动作排列、所需控制参数，清零缓存。
- 一份可重置的 HIM 历史缓存即可；不需要保存每个模型的陈旧历史。
- 输入输出名称、float32 类型、可接受 batch=1 的形状、输出 16 维必须校验。新 HIM 图固定 [1,342]->[1,16]，已有模型保留合法动态 batch 兼容。
- 数值验证覆盖零输入、单个当前帧+零历史、六个不同帧、可获得的实际观测；与 act_inference 或已验证 JIT 比较，记录最大绝对/相对误差，不预先声称误差已达标。
- C++ 观测/历史/动作应与训练端 deploy/s10_mujoco.py 的参考函数一致；所有输出须有限。
- 先做导出/接口一致性和单模型闭环检查。性能 A/B 比较须另问用户；不新增训练、不操作真机。

## 6. 本次参考产物与交接

已有 checkpoint：
D:/Desktop/Code/HIMLoco-for-Go2W/artifacts/s10-obsfix-scratch2000-20260916/model_2000.pt

匹配训练配置：
D:/Desktop/Code/HIMLoco-for-Go2W/artifacts/s10-obsfix-scratch2000-20260916/config.json

已有经评估的 JIT 与参数：
D:/Desktop/Code/HIMLoco-for-Go2W/artifacts/s10-obsfix-scratch2000-20260916/assessments/checkpoint_2000/flat_gym/policy.pt
D:/Desktop/Code/HIMLoco-for-Go2W/artifacts/s10-obsfix-scratch2000-20260916/assessments/checkpoint_2000/flat_gym/policy.json

共同交接输出目录（训练任务写入，比赛任务读取）：
D:/Desktop/Code/HIMLoco-for-Go2W/artifacts/s10-onnx-handoff/model_2000/

交接文件：
- policy.onnx
- policy.json
- export_verification.json（记录所用 checkpoint、参考推理路径、实际误差和验证样本；不额外要求哈希）

代码修改分别在各任务工作区完成；原目录已有未提交修改必须保留。模型产物留在 artifacts/logs 等忽略目录。
