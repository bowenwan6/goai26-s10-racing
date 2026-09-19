# GitHub 文件索引

整理日期：2026-09-18。仓库：`bowenwan6/goai26-s10-racing`，本次推送分支：`codex/field-assistant`。

2026-09-19 补充上传：[导航、建图、点云、IMU 与 Router 设计参考](S10_NAVIGATION_MAPPING_SENSOR_DESIGN_REFERENCE_ZH.md) · [详细文件索引](S10_NAVIGATION_SOURCE_INDEX_ZH.md)（附哈希清单 `S10_NAVIGATION_SOURCE_MANIFEST.json`）、[导航规划重设计](S10_NAVIGATION_PLANNING_REDESIGN_ZH.md)、[平台高度清单](S10_PLATFORM_HEIGHT_INVENTORY_ZH.md)，以及手机“采集助手”页面 `/teach`（[使用说明](../tools/s10_mapping_web/TEACH_GUIDE_ZH.md)）。下文“本次上传”仍是 2026-09-18 的范围。

## 本次上传

| 内容 | 仓库入口 | 范围 |
|---|---|---|
| 手机 app | [tools/s10_mapping_web](../tools/s10_mapping_web/README.md) | 建图、现场助手、航点复访、IMU 诊断与零参考、原生导航界面、服务脚本及 QA |
| v3 全场点云 | [full_cloud.pcd](../deliverables/S10_v3_Map_MuJoCo_20260916/maps/v3/full_cloud.pcd) | 3,346,032 点的原导出；坐标未变换 |
| MuJoCo 地形与机器人 | [场景包 README](../deliverables/S10_v3_Map_MuJoCo_20260916/README.md) | XML、OBJ/PLY、STL、碰撞层、测试证据、查看器及离线预览，保留包内完整依赖 |
| 可分享 ZIP | [S10_v3_Map_MuJoCo_20260916.zip](../deliverables/S10_v3_Map_MuJoCo_20260916.zip) | 原交付 ZIP 与 SHA-256 文件 |
| 原生策略接入 | [native_transfer](../native_transfer/README_ZH.md) | follower/router 接入、控制源管理、路线草稿、pending 配置及验收工具 |
| 离线真机适配 | [real_transfer](../real_transfer/README_ZH.md) | 点云/位姿输入、准入、影子计算与回放；历史测试报告保留 |
| 真机测试与计划 | [tests_real](../tests_real) · [Start/B 验收计划](NATIVE_START_B_ACCEPTANCE.md) | 输入异常、路由与几何回归，以及尚待完成的现场条件 |
| 远程连接工具 | [s10_remote_access](../tools/s10_remote_access/README_ZH.md) | 脚本、服务及说明；不含登录凭据或私钥 |
| 现场照片 | [waypoint-photos-20260914](../waypoint-photos-20260914/README_ZH.md) | HEIC 原图、JPG 预览、联系表与航点照片索引 |
| 雷达/IMU 录制索引 | [recordings/README_GITHUB_ZH.md](../recordings/README_GITHUB_ZH.md) | 录制说明、原始文件哈希清单、bag 元数据与审计证据；MCAP 正文留在本地 |
| 项目报告与海报 | [项目介绍](../PROJECT_INTRODUCTION_ZH.md) · [技术报告](../PROJECT_TECHNICAL_ZH.md) | 中文报告、核查/审计、PDF/海报，以及 academic/report/poster 制作源文件 |
| RL 训练规划 | [GPU 训练计划](../S10_GPU_RL_TRAINING_PLAN_ZH.md) · [NVIDIA 启动计划](../S10_NVIDIA_RL_TRAINING_START_ZH.md) | 当前方案与待验收条件，不包含新训练权重 |

原生接入还包含 `src/s10_auto_nav` 中已有的 XYZ 航点容差和 follower 接入更新。历史路由重建输入保留在 `map-reviews/0914_fr_v3-20260914-142008/reconstruction/` 对应路径；`native_transfer/prepare_routes.py` 按仓库根目录读取它们。

## 获取大文件

安装 Git LFS 后：

```bash
git clone --branch codex/field-assistant https://github.com/bowenwan6/goai26-s10-racing.git
cd goai26-s10-racing
git lfs pull
```

PCD、NPZ、STL、图片/PDF、场景视频、分享 ZIP 和 HEIC/JPG 照片使用 LFS。通过 `git lfs ls-files` 检查列表；普通 Git 的指针文件不是实际点云或 ZIP。分享场景时可把已下载的原 ZIP 完整发给队友。

## 本地保留范围

- `recordings/slam_test_20260917_170443/raw_bag/*.mcap`：同一次录制的 8 个原始分卷，约 3.97 GiB。
- `deliverables/S10_048_LiDAR_IMU_20260917_170443.zip`：约 2.31 GiB 的原始录制交付压缩包。
- 完整厂商地图 session、旧 v1/v2 数据、历史候选场景、编译后的 MJB、ROS/SDK 工作区及重复备份。
- `.venv`、运行时依赖、缓存、Finder 元数据、个人行程表和本地维护记录。

这些文件没有删除；本次推送不是整个 GOAI 工作区或原始采集的全量备份。原始录制的逐文件大小与 SHA-256 已随索引提交。无需复制历史缓存即可使用已交付的 v3 点云和场景。

## 证据与适用范围

复制文件列表和本次检查结果位于 [evidence/github-sync-20260918](../evidence/github-sync-20260918)。场景包原有的 117 项文件 SHA-256 在复制前后校验。

全场点云与局部精细网格的范围不同；当前 MuJoCo 模型覆盖 Start＋B＋B 后短平台。加载与离线测试不代表全场 mesh、策略整路线通过或真机验收。`real_transfer/results` 和各 QA 文档中的旧日期/旧结论是历史记录，本次检查结果另存，不覆盖旧证据。
