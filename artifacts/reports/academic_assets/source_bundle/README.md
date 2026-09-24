# GOAI Lynx S10 — 技术报告源码与证据附件

正文：PROJECT_TECHNICAL_ZH.pdf（8页企业技术交流稿）；可编辑源稿：PROJECT_TECHNICAL_ZH.md。
FACT_CHECK_ZH.md 记录原稿表述、核查证据和改写结果。
ACADEMIC_REVIEW_ZH.md 说明面向企业合作方的最终结构与一致性检查；REPORT_PENDING_ITEMS_ZH.md 为企业会谈准备清单，正文没有待填空白。

## 目录与版本

- source/deployment/：部署版本 3660b81 的 ROS 包、导航/路由/感知、SDK 集成、配置与依赖锁。每个文件均与对应 Git 对象逐字节比对。
- source/training/：使用 git show 从发布版本 5ef14fa 导出的训练、评测、选模代码。目录保留原仓库 training/ 前缀，未使用更新后的工作区版本。
- evidence/：模型元数据与哈希、45-case 原始记录、选择记录、整圈日志与计时、历史测试和失败批次。
- evidence/external_industry_consultation_brief.md：用户提供的补充研究材料。该材料中的后续研究记录不归入 3660b81 的比赛结果，也未从现有原始数据独立重算。
- evidence/later_experiment/：后续 80 次迭代实验日志，只用于区分训练谱系，不是 speed_core iteration 120 的原始训练记录。
- evidence/recomputed_metrics.json：本轮从原始案例和计时日志重算的数值。
- evidence/implementation_detail_reference.md：精简前的详细技术稿，保留奖励归一化、配置与接口细目，用于追溯；当前正文及章节编号以 8 页企业技术交流稿为准。
- evidence/source_manifest.json：所有附带文件的来源、Git版本与 SHA-256。
- academic_assets/：PDF 排版、构建与文本完整性检查脚本。

## 使用范围

这是用于审阅实现与证据的文件选集。完整机器人运行依赖原始仓库、官方场景/SDK及模型权重；上述大型依赖不包含在此附件中。包定义、模型形状和选择记录可直接检查。以下命令在完整部署仓库中执行：

```bash
S10_UPSTREAM_OFFLINE=1 docker compose run --rm s10 scripts/setup_upstream.sh
docker compose run --rm s10 scripts/build.sh
docker compose run --rm s10 scripts/verify_install.sh
docker compose run --rm s10 scripts/run_race.sh --headless
```

部署源码保留原 LICENSE；模型和贡献归属以 evidence/THIRD_PARTY.md 及实际授权文件为准。本附件不更改现有权利归属。

## 重新生成报告

在支持中文字体的 macOS 环境安装 Pandoc、XeLaTeX（TeX Live），并提供 Times New Roman、Arial、Menlo、Songti SC、Heiti SC 后，执行 bash academic_assets/build_pdf.sh。PDF 检查脚本使用 pdfplumber、pypdf、Pillow 与 Poppler。

本轮工作仅修订报告、核对现存文件及重算统计，未重新训练、运行机器人或重跑历史测试。
