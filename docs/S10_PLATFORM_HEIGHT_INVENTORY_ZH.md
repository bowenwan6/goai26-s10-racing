# 高台与台沿高差清单

日期：2026-09-19。只读检查既有文档、v3 保存点云与拟合模型；没有连接或控制机器人，没有修改地图/模型。这里只列目前资料能明确识别的对象，**不是全场所有高台/边缘的完整检测清单**。

## 大概有多高

| 范围 | 位置/对象 | 大概高差 | 数值含义与证据 |
|---|---|---:|---|
| v3 末段 | 草坡后的石笼高台台沿 | **约 30–36 cm** | 台沿两侧局部抽样；点云估计，未现场尺量，不是全台沿最大值 |
| v3 D 后 | 复杂楼梯缺中间踏面的一侧 | **约 29.5–30 cm** | 既有拟合模型中 level 2→4 的单次高差；与普通小台阶分开 |
| v3 D 后 | 复杂楼梯普通分级的一侧 | 约 **13–22 cm/级** | 所选 X=68…72 截面各边缘拟合值范围；不是新增独立高台 |
| v3 B | 楼梯顶部大平台相对底部 | **约 4.97 m** | 整段楼梯的累计地面高差；不能当成一跳 5 米的高台。约 31 个候选上升边界，单级约 14 cm 量级 |
| 旧 050 实录 | 9 月 9 日独立短高台 | **30 cm** | 历史实录说明/高度标签；不默认对应 v3 石笼高台 |
| 旧仿真赛道 | Gate16 高台 | **约 37.7 cm** | MJCF 几何记录：台面 Z≈0.4787248 m，下方地面 Z≈0.102 m。不是 v3 实测 |

对目前 v3 路线的 router 设计，优先单独处理前两项约 30 cm 级的台沿。仅凭高度不能判定原厂高台/楼梯模式能否通过，仍取决于台深、宽度、接近方向、摩擦和实际速度接口。

## 1. 末段石笼高台：本次点云抽样

来源：[保存帧派生点云 frames.npz](/Users/xxxwbwxxx/Documents/ChatGPT/GOAI/map-reviews/0914_fr_v3-20260914-142008/reconstruction/extension_final_garden_v1/sources/frames.npz)。这些点已由历史建模脚本应用保存优化位姿一次，本次没有再次配准或改变坐标。

对应范围由[末段说明](/Users/xxxwbwxxx/Documents/ChatGPT/GOAI/map-reviews/0914_fr_v3-20260914-142008/reconstruction/extension_final_garden_v1/README.md)与[原始截面图](/Users/xxxwbwxxx/Documents/ChatGPT/GOAI/map-reviews/0914_fr_v3-20260914-142008/reconstruction/extension_final_garden_v1/media/02_raw_sections.png)核对。单位 m，v3 map 坐标。

选取两个能辨认上下表面的截面，各取 Y±0.10 m，并远离主要竖直台沿回波：

| 截面 Y | 台上 X 窗口 | 台下 X 窗口 | 台上 Z 中位 | 台下 Z 中位 | 全点中位数之差 | 同帧配对高差中位数 |
|---:|---|---|---:|---:|---:|---:|
| −16.0 | [−39.55,−39.30] | [−38.90,−38.65] | 5.7043 | 5.4253 | 0.2791 | **0.3091**，3 对帧 |
| −15.5 | [−39.00,−38.80] | [−38.40,−38.15] | 5.7314 | 5.4119 | 0.3195 | **0.3591**，2 对帧 |

同帧配对方式：每帧在每个窗口至少有 3 点，分别取 Z 中位数，上减下，再对配对帧取中位数。第一截面台上/台下共 165/270 点；第二截面 180/625 点。全点统计容易被不同帧/采样密度影响，所以同时报告同帧结果。

同帧配对只有 3 对和 2 对，数量较少；窗口也非严格同一 XY，仍含局部坡度影响。约 30–36 cm 是本次抽样量级，**不是置信区间、全边缘范围或厘米级现场验收**。靠近另一截面 Y=−15.0 的下方窗口出现明显上层/植被混合回波（Z 达约 9.6 m），该窗口未用来给出高度结果，不能直接对全部回波取一个地面均值。

## 2. D 后复杂楼梯：一侧约 30 cm 的合并高阶

来源：[surface_fits.json](/Users/xxxwbwxxx/Documents/ChatGPT/GOAI/map-reviews/0914_fr_v3-20260914-142008/reconstruction/extension_to_asphalt_v1/sources/surface_fits.json) 与 [build_scene.py](/Users/xxxwbwxxx/Documents/ChatGPT/GOAI/map-reviews/0914_fr_v3-20260914-142008/reconstruction/extension_to_asphalt_v1/build_scene.py)。

现有模型有 5 个观测高度层，每层为独立平面 `z=a*x+b*y+c`；中间踏面 level 3 在 X≈69.605 m 处终止。X 较大的一侧由 level 2 直接接 level 4，产生一个约 30 cm 的高阶。该边缘位置：

```text
y = 4.87 + 0.098*(x-70)
Δz = z_level4(x,y) - z_level2(x,y)
```

- X=70.0，Y=4.870：Δz≈0.2947 m。
- X=72.0，Y=5.066：Δz≈0.2968 m。
- X=68…69 一侧保留中间踏面，相关两次上升约 13.2–16.6 cm；其他前两级在所选 X=68…72 截面约 15.2–22.0 cm。
- 该片复杂楼梯上下平台以 Y=9 和 Y=3 取样，X=68…72 时累计高差约 0.60–0.67 m；它也不是单级高差。

这是对既有点云拟合参数的计算，原报告 `field_acceptance=false`，未重新用独立实物尺寸验收。较大 X/较小 X 是 map 坐标方位，不直接叫“行进左/右侧”，以免接近朝向改变后混淆。

## 3. 其他资料，不能混作同一场地

- [B 尺寸复核](/Users/xxxwbwxxx/Documents/ChatGPT/GOAI/map-reviews/0914_fr_v3-20260914-142008/B_dimensions_20260917/README.md)：4.97 m 是底到顶整体高差，约 31 个候选级，不是独立高台。
- [旧 050 高台实录说明](/Users/xxxwbwxxx/Documents/ChatGPT/GOAI/s10-field-assistant/docs/S10_LEDGE_MATCHING_GUIDE_ZH.md)：注明 30 cm，高度来自原录制参数；实际台深未可靠量化，旧仿真 3 m 台深属于假设。
- [旧 Gate16 配置](/Users/xxxwbwxxx/Documents/ChatGPT/GOAI/s10-field-assistant/src/s10_bringup/config/strategy_gate16.yaml:45)：直接记载 rise 0.3769 m；台沿 XY≈(12.64593,32.49969)，属于旧仿真坐标与模型，不能放进 v3 地图直接导航。

后续现场重点量取：台沿不同横向位置的局部高差、接近地面坡度、台面有效深度/宽度，以及整机清空后的停稳区域。资料里尚没有可信尺寸的其他卵石/沟槽/边缘，本表不编造统一台高。
