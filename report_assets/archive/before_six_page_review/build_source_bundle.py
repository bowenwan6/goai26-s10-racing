"""Assemble the report's selected source/evidence files without model weights."""
from pathlib import Path
import hashlib
import json
import re
import shutil
import statistics
import subprocess
import zipfile

ROOT = Path(__file__).resolve().parent.parent
BASE = Path('/Users/xxxwbwxxx/Documents/Projects/goai26')
DEPLOY = BASE / 'resources/submission_mac_retest_20260820/20260820_215224_main3660b81/source/goai26-s10-racing'
DEPLOY_GIT = BASE / 'goai26-s10-racing'
TRAIN = BASE / 'goai-s10-gate16-policy-v1-5'
EVID = BASE / 'resources/submission_update_20260820_392s/submission_20260820_392s_main3660b81'
RELEASE = BASE / 'gate16_front_tuck_release_20260817'
STAGE = ROOT / 'academic_assets/source_bundle'
DEPLOY_SHA = '3660b81e8244dfa238633c161673596fe91650f6'
TRAIN_SHA = '5ef14fadd313559e39b60ce6666122a8e9615c07'
UPSTREAM_SHA = '13dd084be6cb5e2514098bc87e586d00dfe580b2'
entries = []

def add_bytes(relative, payload, origin, revision=None, **extra):
    target = STAGE / relative
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes(payload)
    entries.append(dict(path=relative, original_path=str(origin), revision=revision,
                        sha256=hashlib.sha256(payload).hexdigest(), bytes=len(payload), **extra))

def add_file(relative, path, **kw):
    add_bytes(relative, path.read_bytes(), path, **kw)

def git_blob(repo, rev, path):
    return subprocess.check_output(['git', '-C', str(repo), 'show', f'{rev}:{path}'])

deploy_files = []
for package in ('s10_perception', 's10_auto_nav', 's10_bringup'):
    for path in (DEPLOY / 'src' / package).rglob('*'):
        if path.is_file() and '__pycache__' not in path.parts and 'test' not in path.parts:
            if path.suffix in {'.py', '.xml', '.yaml', '.cfg'} or path.name == 'CMakeLists.txt':
                deploy_files.append(path.relative_to(DEPLOY).as_posix())
deploy_files += [p.relative_to(DEPLOY).as_posix() for p in (DEPLOY / 'integration').glob('*.hpp')]
deploy_files += ['LICENSE', 'compose.yaml', 'docker/Dockerfile', 'docker/requirements.lock',
                 'scripts/run_race.sh', 'scripts/setup_upstream.sh', 'scripts/build.sh', 'scripts/verify_install.sh']
for path in sorted(set(deploy_files)):
    payload = (DEPLOY / path).read_bytes()
    expected = git_blob(DEPLOY_GIT, DEPLOY_SHA, path)
    if payload != expected:
        raise ValueError(f'Deployment snapshot differs from pinned Git: {path}')
    add_bytes('source/deployment/' + path, payload, DEPLOY / path, DEPLOY_SHA, git_blob_verified=True)

train_files = ['training/run_gate16_speed_first_pipeline.sh',
               'training/config/gate16_retention_generalization_v2.csv',
               'training/config/gate16_speed_boundary_v1.csv']
train_files += ['training/mujoco_s10/' + name + '.py' for name in (
    '__init__', 'config', 'env', 'network', 'train', 'train_official_policy_residual',
    'official_policy_env', 'official_residual_env', 'evaluate_official_track_skill',
    'collect_residual_demonstrations', 'gate16_entry_cases', 'speed_objective',
    'front_retention', 'rear_push', 'select_gate16_fastest', 'select_gate16_phase',
    'build_front_retention_dataset', 'distill_official_residual', 'distill_residual_dagger')]
train_files += ['training/s10_rl/' + name + '.py' for name in (
    'training_config', 'skill_gate', 'export_climb_bundle', 'observation', 'policy', 'checkpoint')]
for path in train_files:
    payload = git_blob(TRAIN, TRAIN_SHA, path)
    add_bytes('source/training/' + path, payload, TRAIN / path, TRAIN_SHA,
              git_blob_verified=True, extraction='git show; not current working tree')

evidence_files = {
    'external_industry_consultation_brief.md': BASE / 'resources/external_industry_consultation_brief.md',
    'previous_report.md': ROOT / 'report_assets/archive/PROJECT_TECHNICAL_ZH_18pages.md',
    'model_audit.json': ROOT / 'TECHNICAL_MODEL_AUDIT.json',
    'formal_summary.json': RELEASE / 'evaluation/formal_summary.json',
    'selection.json': RELEASE / 'provenance/selection.json',
    'climb_policy_manifest.json': RELEASE / 'deploy_policy/climb_policy_manifest.json',
    'later_experiment/corrected_front_tuck_train.csv': RELEASE / 'provenance/corrected_front_tuck_train.csv',
    'later_experiment/corrected_front_tuck_training_summary.json': RELEASE / 'provenance/corrected_front_tuck_training_summary.json',
    'accepted_run/00_32_seed8.json': EVID / '04_evidence/raw/00_32_seed8.json',
    'accepted_run/00_32_seed8.log': EVID / '04_evidence/raw/00_32_seed8.log',
    'accepted_run/RUN_SUMMARY.md': EVID / '04_evidence/RUN_SUMMARY.md',
    'validation_report.md': EVID / '04_evidence/validation_report.md',
    'TECHNICAL_DESIGN.md': EVID / '03_documents/TECHNICAL_DESIGN.md',
    'THIRD_PARTY.md': EVID / '03_documents/THIRD_PARTY.md',
    'independent_failed_batch.md': BASE / 'resources/submission_mac_retest_20260820/20260820_215224_main3660b81/REPORT.md',
}
for name, path in evidence_files.items():
    add_file('evidence/' + name, path)

formal = json.loads(evidence_files['formal_summary.json'].read_text())
rows = formal['results']
successful = [r for r in rows if r['success']]
timeout = [r for r in rows if not r['success'] and not r['fallen'] and r['steps'] == 850]
run = json.loads(evidence_files['accepted_run/00_32_seed8.json'].read_text())
log = evidence_files['accepted_run/00_32_seed8.log'].read_text()
events = [(int(i), float(t)) for i, t in re.findall(r'Reached waypoint (\d+), sim_time=([0-9.]+)s', log)]
start = re.search(r'Timer started at waypoint 0, sim_time=([0-9.]+)s', log)
assert start is not None and [i for i, _ in events] == list(range(1, 33))
events.insert(0, (0, float(start[1])))
timer = re.search(r'Timer stopped at sim_time=([0-9.]+)s, elapsed=([0-9.]+)s', log)
metrics = {
    'formal_cases': len(rows), 'successes': len(successful),
    'falls': sum(r['fallen'] for r in rows), 'timeouts': len(timeout),
    'drop_free_successes': sum(bool(r['success']) and bool(r['drop_free']) for r in rows),
    'success_rate': len(successful)/len(rows),
    'mean_success_seconds': statistics.mean(r['steps'] for r in successful)*.02,
    'std_success_seconds_population': statistics.pstdev(r['steps'] for r in successful)*.02,
    'mean_penalized_seconds': statistics.mean(r['steps'] if r['success'] else 850 for r in rows)*.02,
    'mean_front_to_rear_seconds': statistics.mean(r['front_to_rear_steps'] for r in successful)*.02,
    'formal_entry_center_y': sorted(set(r['entry_center_y'] for r in rows)),
    'formal_cmd_yaw': sorted(set(r['cmd_yaw'] for r in rows)),
    'official_waypoint_events': events,
    'official_final_sim_time': float(timer[1]), 'official_elapsed_seconds': float(timer[2]),
    'recorder_wall_time_seconds': run['elapsed_s'], 'actual_owner_at_stop': json.loads(run['router_status'])['joint_owner_actual'],
    'router_mode_at_stop': json.loads(run['router_status'])['mode'],
    'method': 'Recomputed from archived JSON rows and raw timer events; no new simulation or training.'}
assert (metrics['successes'], metrics['falls'], metrics['timeouts']) == (38, 2, 5)
assert abs(metrics['mean_penalized_seconds'] - 7.824) < 1e-10
assert len(events) == 33 and [i for i, _ in events] == list(range(33))
assert metrics['actual_owner_at_stop'] == 'official' and metrics['router_mode_at_stop'] == 'navigate'
add_bytes('evidence/recomputed_metrics.json', json.dumps(metrics, ensure_ascii=False, indent=2).encode(),
          ROOT / 'academic_assets/build_source_bundle.py', extraction='derived from archived evidence')

report = (ROOT / 'PROJECT_TECHNICAL_ZH.md').read_text()
cpp = re.search(r'```cpp\n(.*?)\n```', report, re.S)[1]
cpp_src = (STAGE / 'source/deployment/integration/gate16_policy_runner.hpp').read_text()
assert ''.join(cpp.split()) in ''.join(cpp_src.split()), 'C++ excerpt mismatch'
yaml = re.search(r'```yaml\n(.*?)\n```', report, re.S)[1]
yaml_src = (STAGE / 'source/deployment/src/s10_bringup/config/strategy_gate16.yaml').read_text()
assert all(line.strip() in yaml_src for line in yaml.splitlines()), 'YAML excerpt mismatch'

for name in ('PROJECT_TECHNICAL_ZH.md', 'PROJECT_TECHNICAL_ZH.pdf', 'FACT_CHECK_ZH.md', 'REPORT_PENDING_ITEMS_ZH.md'):
    path = ROOT / name
    if path.exists():
        payload = path.read_bytes()
        if path.suffix == '.md':
            payload = payload.replace(str(STAGE).encode() + b'/', b'')
        add_bytes(name, payload, path, extraction='report artifact; bundle links made relative')
for name in ('header.tex', 'layout.lua', 'build_pdf.sh', 'check_pdf.py', 'pdf_check.json', 'build_source_bundle.py'):
    add_file('academic_assets/' + name, ROOT / 'academic_assets' / name)

readme = '''# GOAI Lynx S10 — 技术报告源码与证据附件

正文：PROJECT_TECHNICAL_ZH.pdf（8页）；可编辑源稿：PROJECT_TECHNICAL_ZH.md。
FACT_CHECK_ZH.md 记录原稿表述、核查证据和改写结果。REPORT_PENDING_ITEMS_ZH.md 汇总 P1–P6 及新增 R1–R3，供团队填写。

## 目录与版本

- source/deployment/：部署版本 3660b81 的 ROS 包、导航/路由/感知、SDK 集成、配置与依赖锁。每个文件均与对应 Git 对象逐字节比对。
- source/training/：使用 git show 从发布版本 5ef14fa 导出的训练、评测、选模代码。目录保留原仓库 training/ 前缀，未使用更新后的工作区版本。
- evidence/：模型元数据与哈希、45-case 原始记录、选择记录、整圈日志与计时、历史测试和失败批次。
- evidence/external_industry_consultation_brief.md：用户提供的补充研究材料；187D Teacher 与 model_796/model_499 的新研究记录按该文档引用，原始产物待 R1 补齐。上述新研究不归入 3660b81 的比赛结果，也未从现有原始数据独立重算。
- evidence/later_experiment/：后续 80 次迭代实验日志，只用于区分训练谱系，不是 speed_core iteration 120 的原始训练记录。
- evidence/recomputed_metrics.json：本轮从原始案例和计时日志重算的数值。
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

部署源码保留原 LICENSE；模型和贡献归属保留 evidence/THIRD_PARTY.md 的说明。P6 是仍待团队补齐的模型署名/授权文件。本附件不更改现有权利归属。

## 重新生成报告

在支持中文字体的 macOS 环境安装 Pandoc、XeLaTeX（TeX Live），并提供 Times New Roman、Arial、Menlo、Songti SC、Heiti SC 后，执行 bash academic_assets/build_pdf.sh。PDF 检查脚本使用 PyMuPDF 和 Pillow。

本轮工作仅修订报告、核对现存文件及重算统计，未重新训练、运行机器人或重跑历史测试。
'''
add_bytes('README.md', readme.encode(), 'generated for report source bundle')
manifest = {
    'report_date': '2026-09-02',
    'deployment_commit': DEPLOY_SHA, 'training_commit': TRAIN_SHA, 'upstream_commit': UPSTREAM_SHA,
    'deployment_files_verified_against_git': len(set(deploy_files)),
    'training_files_extracted_from_git': len(train_files),
    'supplemental_research_evidence': {'source': 'evidence/external_industry_consultation_brief.md', 'status': 'user-provided narrative; original checkpoints and evaluation JSON pending R1'},
    'model_weights_included': False,
    'cpp_excerpt_verified_ignoring_whitespace': True,
    'yaml_excerpt_verified': True,
    'files': entries,
}
(STAGE / 'evidence/source_manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2))
zip_path = ROOT / 'PROJECT_REPORT_SOURCES.zip'
with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as archive:
    for path in sorted(STAGE.rglob('*')):
        if path.is_file():
            archive.write(path, 'PROJECT_REPORT_SOURCES/' + path.relative_to(STAGE).as_posix())
with zipfile.ZipFile(zip_path) as archive:
    assert archive.testzip() is None
    count = len(archive.namelist())
print(json.dumps({'files': count, 'zip_bytes': zip_path.stat().st_size,
                  'deployment_verified': len(set(deploy_files)), 'training_exported': len(train_files),
                  'metrics': {k:v for k,v in metrics.items() if k != 'official_waypoint_events'}}, ensure_ascii=False, indent=2))
