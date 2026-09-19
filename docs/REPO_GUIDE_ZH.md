# 仓库说明：目录、分支、上传规则

2026-09-19 整理。policy 和 app 的总览见 [POLICIES_AND_APPS_ZH.md](POLICIES_AND_APPS_ZH.md)。

## 1. 目录

| 目录 | 内容 |
|---|---|
| `src/` | ROS 2 包：`s10_auto_nav`（导航：follower、route_v2、`rl_nav`、strategy router）、`s10_perception`（仿真感知、高度图）、`s10_bringup`（launch 与配置） |
| `integration/` | C++ SDK 接入：joint owner、Gate16 / HIM policy runner、起身状态机 |
| `policy/` | Ver1.0 部署用的 policy 包（`gate16`、`stairs_stable`） |
| `policies/` | 8 月训练的 57D ONNX 模型 |
| `training/` | 8 月的 RL 训练代码 |
| `ros1_gateway/` | ROS 2 → ROS 1 网关、106 点云 tap、ROS 1 运控 SDK、MCAP 转换、x_nav 部署 |
| `tools/` | `s10_mapping_web`（手机页面）、`wp_match`（WP 与路线）、`s10_remote_access`、`s10_gait_capture`、`s10/`（高度图回放等） |
| `native_transfer/` | 原生步态（0x3002 / 0x3003）导航 |
| `real_transfer/`、`tests_real/` | 真机适配（只读采集、影子计算、回放）及其测试 |
| `sim_full_course/` | 运动学全程仿真 |
| `scripts/` | 构建、运行、SDK 副本、HIM、比赛脚本 |
| `docs/` | 文档。入口：`POLICIES_AND_APPS_ZH.md`、本文件、`GITHUB_FILE_INDEX_ZH.md` |
| `deliverables/`、`map-reviews/`、`waypoint-photos-20260914/` | v3 地图、MuJoCo 场景、地图审阅资料、WP 照片（大文件用 Git LFS） |
| `recordings/` | 录制说明（原始数据不在 git） |
| `artifacts/`、`evidence/`、`backups/` | 现场证据、推送核验、已部署程序的快照 |
| `academic_assets/`、`report_assets/`、`poster_assets/`、`posterly_rebuild/`，及根目录的报告 PDF / MD | 报告、海报及构建脚本。脚本按根目录路径引用这些文件，所以没有移动 |
| `goai_embodied_future_material-main/` | 比赛官方资料（上游原样） |
| `submission/`、`output/` | 8 月提交材料、页面截图 |
| `docker/`、`compose.yaml`、`.github/` | 开发容器、CI |

## 2. 分支

### 2.1 规则

- **`main` 是唯一的最新整合版。** 其他分支都通过 PR 合进来。
- **开发分支按用途命名：**
  - `nav/*`：导航
  - `rl/*`：RL 导航
  - `codex/*`：现场 app
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
| `rl/maneuver-router` | 09-19 | **开发中**（`rl_nav`）。截至 49fd514 已并入 `main`，之后的提交再通过 PR 合入 |
| `nav/route-v2-integration` | 09-19 | 已并入 `main` |
| `codex/field-assistant` | 09-19 | 已并入 `main`（现场 app、`/teach`） |
| `Jackdev` | 09-18 | 队友 Jack 的分支，与 `main` 分叉；有 1 个独有提交，保留 |
| `agent/stairs57-fusion-20260819` | 08-19 | Belsun 的 stairs57 接入；有 1 个独有提交，已停用 |
| 其余 22 个（`bw-test-*`、`bw-fix-*`、`agent/nav-commitment-v1`、`s10-recording-review-20260909`、`codex/backup-s10-app-20260913`） | 08-13 至 09-13 | 已完全并入 `main`，没有独有提交 |

## 3. 大文件与不进 git 的东西

- **Git LFS：** 点云、网格、图片、PDF、视频、ZIP 用 LFS。克隆后执行 `git lfs pull`。
- **不进 git：**
  - 原始录包（MCAP / bag）
  - x_nav 授权文件 `ssd_whitelist.conf.hash`
  - `.venv`、ROS 1 运行包 `bundle/`、`build/`
  - AGX 上的 SDK 副本
  - 所有账号、密码、私钥、授权码
- **只在本地的大件：**
  - `GOAI/S10_Nav_FullV3_Repro_20260919.zip`（97 MB 全程复现包）
  - 原始录制：采集工作站 `D:/S10Data/`；106 / AGX 上的录包

## 4. 推送前检查

1. **跑测试：**
   - Python：`PYTHONPATH=.:src/s10_auto_nav:src/s10_perception python -m pytest -q src/s10_auto_nav/test sim_full_course/tests tests_real`
   - ROS 相关测试用 `s10-racing:dev` 镜像。
   - `ros1_gateway` 的测试见其 README。
2. **扫描密码和密钥：** 例如 `git diff --cached | grep -niE "passw|secret|token|BEGIN .*PRIVATE KEY"`。
3. **证据类文件逐字节保存：** 在 `.gitattributes` 里标 `-text`，不做换行转换。
4. **不提交无关的删除：** 本地清理脚本可能删掉工作区里的文件，提交前看清 `git status`。
