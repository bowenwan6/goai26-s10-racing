# 仓库说明：目录、分支、上传规则

2026-09-19 整理。policy 和 app 的总览见 [POLICIES_AND_APPS_ZH.md](POLICIES_AND_APPS_ZH.md)。

## 1. 目录

2026-09-20 整理过一次：文档收敛成 3 份，资料按用途归到 `data/`、`evidence/`、`reports/`、`vendor/`。代码目录没有动，避免破坏脚本、测试和其他会话的同步。

**代码**

| 目录 | 内容 |
|---|---|
| `src/` | ROS 2 包：`s10_auto_nav`（导航：follower、route_v2、`rl_nav`）、`s10_perception`、`s10_bringup` |
| `ros1_gateway/` | ROS 2 → ROS 1 网关、106 点云 tap、ROS 1 运控与导航运行时、一键运行脚本、MCAP 转换、x_nav 部署、现场文档 `docs/` |
| `integration/` | C++ SDK 接入：joint owner、Gate16 / HIM policy runner、起身状态机 |
| `sim_full_course/` | 运动学全程仿真 |
| `native_transfer/`、`real_transfer/`、`tests_real/` | 原生步态导航、真机适配（只读采集、影子计算、回放）及测试 |
| `tools/` | `s10_mapping_web`（手机页面）、`wp_match`、`s10_remote_access`、`s10_gait_capture`、`s10/` |
| `scripts/`、`docker/`、`compose.yaml`、`.github/` | 构建 / 运行 / SDK 脚本、开发容器、CI |
| `policy/`、`policies/`、`training/` | Ver1.0 部署用的 policy 包、8 月的 ONNX 模型集合、8 月训练代码。两个目录名字相近：`policy/` 是**部署包**（含 manifest 和校验），`policies/` 是**模型文件集合**；比赛配置和测试按这两个路径写死，暂不重命名 |

**资料**

| 目录 | 内容 |
|---|---|
| `docs/` | 3 份文档（见下）＋ `media/`（README 配图）＋ `reference/`（厂商手册 PDF） |
| `data/` | `deliverables/`（v3 地图与 MuJoCo 包）、`map-reviews/`、`waypoint-photos-20260914/`、`recordings/`（说明，原始数据不入库） |
| `evidence/` | `artifacts/`（现场证据）、`backups/`（已部署程序快照）、`output/`（页面截图）、`github-sync-20260918/`（推送核验） |
| `reports/` | 报告与海报：`academic_assets/`、`report_assets/`、`poster_assets/`、`posterly_rebuild/`、`submission/`，以及根目录搬来的报告 MD / PDF / ZIP。里面的构建脚本按 2026-09-20 之前的根目录路径写的，作为记录保留，不保证可直接重跑 |
| `vendor/contest_material/` | 主办方资料，原样不改 |

**文档只留 3 份**（2026-09-20 决定）：

| 文档 | 内容 |
|---|---|
| `docs/POLICIES_AND_APPS_ZH.md` | 有哪些 policy 和 app、各自状态、实测数字、缺口 |
| `docs/NAVIGATION_DESIGN_ZH.md` | route_v2 跟随、`rl_nav` 与鲁棒性方案、新 SLAM 接入计划（由三份文档合并） |
| `docs/REPO_GUIDE_ZH.md` | 本文件：目录、分支、大文件、第三方与许可、推送前检查 |

操作手册跟着代码走（`ros1_gateway/README_ZH.md`、`ros1_gateway/docs/`、`tools/s10_mapping_web/TEACH_GUIDE_ZH.md`、`sim_full_course/README_ZH.md` 等），不往 `docs/` 搬。

其余 36 份文档在 2026-09-20 合并或退役，用标签取回：

```bash
git show docs-archive-20260920:docs/TECHNICAL_DESIGN.md
git show docs-archive-20260920:docs/S10_REAL_ROBOT_QUICKSTART_ZH.md
```

新增文档前先问：能不能写进上面 3 份里？可以就不要新建文件。

## 2. 分支

### 2.1 规则

- **`main` 是唯一的最新整合版。** 其他分支都通过 PR 合进来。
- **开发分支按用途命名：**
  - `nav/*`：导航
  - `rl/*`：RL 导航
  - `codex/*`：现场 app
  - `ros1/*`：ROS 1 网关、新 SLAM 接入
  - `docs/*`：文档
  - `integration/*`：整合
  - 队友可以用个人分支（如 `Jackdev`）。
- **旧分支暂时都保留（2026-09-19 决定）。** 下表标出每个分支是否已并入 `main`。以后要清理时：
  - 已完全并入 `main` 的分支可以直接删，提交不会丢；
  - 有独有提交的分支先打 `archive/<分支名>` 标签再删。
- **8 月仿真赛版本用标签标记：**
  - `v1.0-sim-release` → `1e390f5`，"publish version 1 competition release"。
  - `sim-contest-submission` → `3660b81`，与提交镜像 `s10-racing:submission-3660b81-clean` 对应。

### 2.2 当前分支（2026-09-19）

相对 `main` 的状态由 GitHub compare 接口查得。

| 分支 | 最后提交 | 状态 |
|---|---|---|
| `main` | 09-19 | **最新整合版**（PR #4）：`rl/maneuver-router`、`nav/route-v2-integration`、`codex/field-assistant`、原 `main`、`ros1_gateway/` 及新文档 |
| `integration/2026-09-19` | 09-19 | PR #4 的源分支，已并入 `main` |
| `ros1/newdog-20260919` | 09-19 | PR #5（新狗 050 号实测结果、ROS 1 网关同步），已并入 `main` |
| `ros1/oldpages-check-20260919` | 09-19 | PR #6（旧页面连不上新狗的实测结论），已并入 `main` |
| `docs/readme-overhaul-20260920` | 09-20 | PR #8（英文 README 改版、配图），已并入 `main` |
| `ros1/nav-048-20260920` | 09-20 | PR #9（ROS 1 导航运行时、一键运行、开机自启、048 实验文档），已并入 `main` |
| `docs/gpu-stairs-20260920` | 09-20 | PR #10（楼梯 policy 训练结论），已并入 `main` |
| `docs/tidy-20260920` | 09-20 | PR #12（文档收敛成 3 份、目录整理），已并入 `main` |
| `rl/maneuver-router` | 09-19 | **开发中**（`rl_nav`）。截至 e1aaf73 已并入 `main`（PR #4、#7），之后的提交再通过 PR 合入 |
| `nav/route-v2-integration` | 09-19 | 已并入 `main` |
| `codex/field-assistant` | 09-19 | 已并入 `main`（现场 app、`/teach`） |
| `Jackdev` | 09-18 | 队友 Jack 的分支，与 `main` 分叉；有 1 个独有提交，保留 |
| `agent/stairs57-fusion-20260819` | 08-19 | Belsun 的 stairs57 接入；有 1 个独有提交，已停用 |
| 其余 22 个（`bw-test-*`、`bw-fix-*`、`agent/nav-commitment-v1`、`s10-recording-review-20260909`、`codex/backup-s10-app-20260913`） | 08-13 至 09-13 | 已完全并入 `main`，没有独有提交 |

## 3. 大文件与不进 git 的东西

- **Git LFS：** 点云、网格、图片、PDF、视频、ZIP 用 LFS。克隆后执行 `git lfs pull`。
- **不进 git：**
  - `ros1_gateway/nav/s10_auto_nav/`：由 `nav/sync_auto_nav.sh` 从 `src/s10_auto_nav` 生成，版本记在 `nav/COMMIT`
  - `ros1_gateway/vendor/agx_only/`：上游 `ros2/ros1_bridge@611755f` 的原样快照，需要时从上游取
  - 整理前的历史文档：留在 `docs-archive-20260920` 标签里，不在工作区
  - 原始录包（MCAP / bag）
  - x_nav 授权文件 `ssd_whitelist.conf.hash`
  - `.venv`、ROS 1 运行包 `bundle/`、`build/`
  - AGX 上的 SDK 副本
  - 所有账号、密码、私钥、授权码
- **只在本地的大件：**
  - `GOAI/S10_Nav_FullV3_Repro_20260919.zip`（97 MB 全程复现包）
  - 原始录制：采集工作站 `D:/S10Data/`；106 / AGX 上的录包

## 4. 第三方与许可

仓库按 [BSD-3-Clause](../LICENSE) 发布。合并前的完整披露见 `git show docs-archive-20260920:docs/THIRD_PARTY.md` 与 `…:docs/OPEN_SOURCE_PLAN.md`，要点：

| 来源 | 内容 | 边界 |
|---|---|---|
| 主办方 | `DeepRoboticsLab/goai_embodied_future_material`（锁定 `13dd084b`）：S10 SDK、机器人模型、MuJoCo 赛道与航点 | BSD-3-Clause，访问权由主办方授予；副本在 `vendor/contest_material/`，不随提交包分发 |
| 队友资产 | `policy/gate16/`（来自 `belsun/goai-s10-gate16-policy`）、`policy/stairs_stable/`（队友的 collision-ablation model599） | **集成时没有独立许可文件**，记为队内竞赛资产。对注册队伍和评委之外分发前，必须先拿到书面授权或补上许可与署名——这是目前唯一未解决的模型许可项 |
| 厂商 | S10 本体、SDK、原生步态、官方 SLAM、x_nav 容器 | Deep Robotics 所有；授权文件不入库 |
| 运行依赖 | Ubuntu 24.04、ROS 2 Jazzy、ROS-O（Noetic 系）、Python 3.12、NumPy、SciPy、MuJoCo、ONNX Runtime、rosbags | 各自上游许可；运行时不调用任何商业 API 或在线推理服务 |

`src/`、`ros1_gateway/`、`sim_full_course/`、`tools/`、`integration/`、`docs/` 里的内容是我们自己的，文件另有声明的除外。

## 5. 推送前检查

1. **跑测试：**
   - Python：`PYTHONPATH=.:src/s10_auto_nav:src/s10_perception python -m pytest -q src/s10_auto_nav/test sim_full_course/tests tests_real`
   - ROS 相关测试用 `s10-racing:dev` 镜像。
   - `ros1_gateway` 的测试见其 README。
2. **扫描密码和密钥：** 例如 `git diff --cached | grep -niE "passw|secret|token|BEGIN .*PRIVATE KEY"`。
3. **证据类文件逐字节保存：** 在 `.gitattributes` 里标 `-text`，不做换行转换。
4. **不提交无关的删除：** 本地清理脚本可能删掉工作区里的文件，提交前看清 `git status`。
