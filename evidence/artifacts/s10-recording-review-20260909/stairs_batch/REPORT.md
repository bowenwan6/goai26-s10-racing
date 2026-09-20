# 剩余楼梯匹配结果

新增 5 个参考包，240.84 秒，27,675 个原关节样本，覆盖 7 个上/下楼候选阶段。

## 结果总览

| 参考包 | 源时间（秒） | 样本 | 轨迹质量掩码通过 | 楼梯拟合中位误差 |
|---|---:|---:|---:|---:|
| 145331_flight2 | 29–59 | 5980 | 100.0% | 3.3 cm |
| 145331_flight3 | 59–93.2 | 6800 | 100.0% | 3.7 cm |
| 152557_down | 0–40.396 | 8060 | 97.9% | 5.0 cm |
| 154654_up | 0–52.89 | 2640 | 93.2% | 3.2 cm |
| 154856_up_down | 0–84.12 | 4195 | 91.9% | 4.1 cm / 4.1 cm / 6.1 cm |

[集中看视频和下载 USD](index.html)。各目录保留原始点云缓存、配准轨迹、原关节参考、拟合地形、几何残差和复核视频；NPZ/视频/USD 为本地产物。

轨迹质量通过率不是成功通行率；配准中位残差约 1 cm 不代表楼梯边缘或轮端有同样精度。拟合楼梯的 P90 误差约 10–17 cm，轮端尚有偏差。本批不强制抬高机身消除穿插，不宣称碰撞或动力学通过。

## 排除区间

- `gait_20260909_145241_9cae0ba51169` 0–16.699 s：Active stair-attempt interval 1.5-7s is marked waste data in the latest human review; remainder is diagnostic context.
- `gait_20260909_152557_7ddbfc67c108` 40.396–120.004 s：Control exited; post-exit body movement is not expert stair motion.
- `gait_20260909_154654_65c5ae680a24` 52.89–111.551 s：Control exit and subsequent recovery/context; no additional identified stair passage.

## 使用入口

完整操作和训练用法见 [使用方法与经验文档](../../../docs/S10_STAIRS_MATCHING_GUIDE_ZH.md)。先看逐阶段 CSV 与回放，再使用 training_reference.npz 的逐通道权重；contact_label_valid 始终为 false。