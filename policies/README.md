# S10 连续上楼 57D policy

> 历史版本说明：本文适用于 [cc93d39](https://github.com/bowenwan6/goai26-s10-racing/tree/cc93d39f8a939c88627f73534ca42c30d7705746)。当前分支已采用 main 的比赛实现，旧 WASD、键盘切换策略及 replay_waypoint.sh 入口已停用；当前操作见 [项目 README](../README.md)。

`s10_stairs_stable_up_57d_model499.onnx` 是当前 WP18→19 实测更稳定的连续上楼候选模型。它保持官方部署接口：57 维 observation、16 维 action，不读取高度图。

- SHA-256：`676D45A611A665799E6F41DA29B15C74D6E10E4D655CBE2FC04DE3254EA003F2`
- 训练命令速度：`vx=0.15～0.40 m/s`、`vy=±0.12 m/s`、`wz=±0.25 rad/s`
- 建议比赛地图测试速度：`0.35 m/s`
- WP18→19 同条件 A/B：10.4 s 到顶，无仿真数值异常；旧 `model1800` 在 5.28/8.14 m 后失稳

在 WSL 的仓库根目录测试 WP18→19：

```bash
export S10_POLICY_PATH="$PWD/policies/s10_stairs_stable_up_57d_model499.onnx"
scripts/replay_waypoint.sh 18 0.35 20
```

模型可以直接被现有 `rl_deploy` 加载，但当前 runner 一次只加载一个 ONNX。整场自动导航不应把它从起点一直用到终点；应由 router 在检测到真正的楼梯入口后切入，到达顶部后切回官方 policy。WP7 后还有一段小路，所以不能在 WP7 本身切换。

`s10_stairs_up_57d_model1800.onnx` 仍保留为规则楼梯矩阵回归基线。稳定版目前只有一次确定性 WP18 回归；正式替换前还应覆盖初始横向偏移、yaw 和摩擦变化。

## 直行窄走廊续训候选

`straight_up_compare/model_600.onnx`、`model_700.onnx`、`model_800.onnx` 和
`model_898.onnx` 是从 stable `model_499` 继续训练 101/201/301/399 次的实验导出，
均保持 57→16 接口。比赛地图复测中只有 `model_800` 到顶，但用时 17.1 s，横向
RMS/peak 为 0.280/0.540 m；当前 stable `model_499` 为 11.3 s 和
0.259/0.506 m。因此这些候选不替换默认上楼模型。

并排回放保存在 `artifacts/straight_up_compare/stable499_vs_model800_wp18.mp4`：左侧为
stable `model_499`，右侧为续训 `model_800`。`model_800.onnx` 的 SHA-256 为
`13B22645F6B8D5679EF1FDDFD3109057F4896849E2A572E33FED3F311EE3E65F`。
