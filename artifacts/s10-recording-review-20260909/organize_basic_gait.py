"""Screen the 17 recordings for basic-gait motion references; no raw writes or training.

Run: python -s -B artifacts/s10-recording-review-20260909/organize_basic_gait.py
Uses existing decoded arrays/cloud samples. --selfcheck needs no recordings.
"""
import argparse
import base64
from collections import Counter
import html
import json
from pathlib import Path

import numpy as np
from plots import angles, plt

OUT = Path(__file__).resolve().parent
LABELS = {'level_candidate': '近水平地面候选', 'slope_candidate': '坡面候选',
          'uneven_or_edge': '起伏/边缘待分辨', 'unknown': '几何不确定',
          'ground_motion': '优先复核：地面运动', 'varied_ground': '扩展复核：坡面/起伏',
          'stationary': '站立/等待单列', 'obstacle_context': '障碍附近单列',
          'uncertain_ground': '地形证据不足', 'exclude': '暂不纳入',
          'moving_turn': '移动中转向候选', 'turn_or_adjust': '转向/调整候选',
          'reported_forward': '报告前向运动', 'reported_backward': '报告后向运动',
          'reported_sideways': '报告侧向运动', 'motion_unknown': '运动/调整，方向未定'}


def dump(path, obj):
    path.write_text(json.dumps(obj, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')


def level_rotation(quaternion):
    roll, pitch, _ = np.radians(angles(np.asarray([quaternion]))[0])
    cr, sr, cp, sp = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch)
    return np.array([[cp, sp*sr, sp*cr], [0, cr, -sr], [-sp, cp*sr, cp*cr]])


def fit_ground(points):
    """Local gravity-aligned plane hypothesis, not terrain/material ground truth."""
    v = points[(abs(points[:, 0]) < 2.5) & (abs(points[:, 1]) < 1.4)
               & (points[:, 2] > -1.5) & (points[:, 2] < -.15)
               & ((abs(points[:, 0]) > .65) | (abs(points[:, 1]) > .45))]
    if len(v) < 60:
        return None
    # Equal weight per 15 cm cell, so a dense near patch cannot dominate the plane.
    cells, groups = np.unique(np.floor(v[:, :2]/.15).astype(int), axis=0, return_inverse=True)
    v = np.array([np.median(v[groups == i], axis=0) for i in range(len(cells))])
    if len(v) < 30 or np.ptp(v[:, 0]) < .8 or np.ptp(v[:, 1]) < .5:
        return None
    a = np.c_[v[:, :2], np.ones(len(v))]
    rng = np.random.default_rng(1041)
    candidates = []
    for _ in range(48):
        ids = rng.choice(len(v), 3, replace=False)
        if abs(np.linalg.det(a[ids])) < .02:
            continue
        beta = np.linalg.solve(a[ids], v[ids, 2])
        if np.linalg.norm(beta[:2]) < np.tan(np.radians(30)) and -1.3 < beta[2] < -.15:
            candidates.append(beta)
    if not candidates:
        return None
    residual = abs(a @ np.array(candidates).T-v[:, 2, None])
    best = np.argmax(np.sum(residual < .045, axis=0))
    inliers = residual[:, best] < .045
    if inliers.sum() < 20:
        return None
    beta = np.linalg.lstsq(a[inliers], v[inliers, 2], rcond=None)[0]
    residual = abs(a @ beta-v[:, 2])
    return dict(tilt_deg=float(np.degrees(np.arctan(np.linalg.norm(beta[:2])))),
                forward_grade_deg=float(np.degrees(np.arctan(beta[0]))),
                cross_grade_deg=float(np.degrees(np.arctan(beta[1]))), height_m=float(beta[2]),
                support=float(np.mean(residual < .045)), residual_p80_m=float(np.percentile(residual, 80)),
                cells=len(v))


def cloud_planes(record, data):
    anchor = int(record['anchor_ns'])
    it = (data['IMU_src']-anchor)/1e9
    clouds = json.loads((OUT/'decoded'/f"{record['id']}.clouds.json").read_text(encoding='utf-8'))
    result = []
    for topic, frames in clouds.items():
        for frame in frames:
            t = frame['src']
            i = int(np.searchsorted(it, t).clip(0, len(it)-1))
            if i and abs(it[i-1]-t) < abs(it[i]-t):
                i -= 1
            q = data['IMU_v'][i, :4]
            if abs(it[i]-t) > .05 or not .99 < np.linalg.norm(q) < 1.01:
                continue
            xyz = np.frombuffer(base64.b64decode(frame['xyz_mm_b64']), dtype='<i2').reshape(-1, 3)/1000
            fit = fit_ground(xyz @ level_rotation(q).T)
            if fit:
                result.append(dict(t=t, side='front' if 'front' in topic else 'rear', **fit))
    return result


def geometry(rows):
    if not rows:
        return 'unknown', None
    sides = [[r for r in rows if r['side'] == side] for side in ('front', 'rear')]
    good = [r for r in rows if r['support'] >= .7 and r['residual_p80_m'] <= .06]
    tilt = float(np.median([r['tilt_deg'] for r in good])) if good else None
    if not all(sides) or len(rows) < 3:
        return 'unknown', tilt
    # ponytail: local planes only, extrinsic/timing errors and multiple surfaces still require review.
    consistent = abs(np.median([r['height_m'] for r in sides[0]])
                     -np.median([r['height_m'] for r in sides[1]])) <= .12
    gradients = [np.median([[r['forward_grade_deg'], r['cross_grade_deg']] for r in side], axis=0) for side in sides]
    consistent = consistent and np.linalg.norm(gradients[0]-gradients[1]) < 5
    if len(good)/len(rows) < .7 or not consistent:
        return 'uneven_or_edge', tilt
    spread = np.percentile([r['tilt_deg'] for r in good], 90)-np.percentile([r['tilt_deg'] for r in good], 10)
    if tilt < 4 and spread < 5:
        return 'level_candidate', tilt
    if 4 <= tilt <= 20 and spread < 6:
        return 'slope_candidate', tilt
    return 'uneven_or_edge', tilt


def motion_kind(joints, imu, motion):
    leg = np.sqrt(np.mean(joints[:, 16:32][:, [i for i in range(16) if i % 4 != 3]]**2))
    wheel = np.mean(abs(joints[:, [19, 23, 27, 31]]))
    yaw = np.median(abs(imu[:, 6]))
    vx, vy = np.median(motion[:, :2], axis=0)
    if leg < .2 and wheel < .35 and yaw < .1:
        name = 'stationary'
    elif yaw > .15:
        name = 'moving_turn' if max(abs(vx), abs(vy)) > .08 else 'turn_or_adjust'
    elif abs(vx) > .08 and abs(vx) >= abs(vy):
        name = 'reported_forward' if vx > 0 else 'reported_backward'
    elif abs(vy) > .08:
        name = 'reported_sideways'
    else:
        name = 'motion_unknown'
    return name, dict(leg_dq_rms=float(leg), wheel_abs_dq_mean=float(wheel),
                      gyro_z_abs_median=float(yaw), reported_vx_median=float(vx), reported_vy_median=float(vy))


def valid_window(times, values, start, end, tolerance):
    lo, hi = np.searchsorted(times, [start, end])
    v = values[lo:hi]
    ok = len(v) > 0 and np.isfinite(v).all()
    if ok:
        ok = times[lo]-start <= tolerance and end-times[hi-1] <= tolerance
        ok = ok and (len(v) < 2 or np.max(np.diff(times[lo:hi])) <= tolerance)
    return v, bool(ok)


def human_rows():
    path = OUT/'human_reviews.jsonl'
    latest = {}
    for line in path.read_text(encoding='utf-8').splitlines() if path.exists() else []:
        entry = json.loads(line)
        latest[entry['id']] = entry
    return [r for r in latest.values() if not r.get('retired')]


def screen(record, data, planes, annotations, notes):
    sid, duration, anchor = record['id'], record['duration_s'], int(record['anchor_ns'])
    changes = [r['start_s'] for r in record['control_timeline'][1:]]
    boundaries = sorted({0., duration, *np.arange(0, duration, 2.),
                         *[v for r in annotations for v in (r['start_s'], r['end_s'])],
                         *[max(0., min(duration, c+margin)) for c in changes for margin in (-.5, 0, .5)]})
    timelines = {k: (data[k+'_src']-anchor)/1e9 for k in ('IMU', 'JOINTS_DATA', 'MOTION_INFO')}
    assert all(np.all(np.diff(t) > 0) for t in timelines.values()), sid
    rows = []
    for a, b in zip(boundaries, boundaries[1:]):
        windows = {k: valid_window(t, data[k+'_v'], a, b, .12 if k == 'MOTION_INFO' else .06)
                   for k, t in timelines.items()}
        geo, tilt = geometry([r for r in planes if a <= r['t'] < b])
        manual = [r for r in annotations if r['start_s'] < b and r['end_s'] > a]
        row = dict(recording_id=sid, start_s=float(a), end_s=float(b), duration_s=float(b-a),
                   geometry_candidate=geo, plane_tilt_median_deg=tilt, surface_type='unknown',
                   terrain_human='unknown', human_notes=[r.get('notes', '') for r in manual if r.get('notes')],
                   human_review_ids=[r['id'] for r in manual], reference_use='pending',
                   motion_candidate='motion_unknown', category='exclude', reasons=[],
                   reference_ready=False, command_alignment='not_validated', velocity_validity='not_validated',
                   time_basis='source_stamp_minus_manifest_started_wall_ns', anchor_ns=str(anchor))
        if not all(ok for _, ok in windows.values()):
            row['reasons'].append('机身流缺失、边界不足或非有限数值')
        else:
            imu, joints, motion = (windows[k][0] for k in ('IMU', 'JOINTS_DATA', 'MOTION_INFO'))
            row['motion_candidate'], metrics = motion_kind(joints, imu, motion)
            row.update(metrics, max_body_tilt_deg=float(np.max(abs(angles(imu[:, :4])[:, :2]))),
                       state_codes=np.unique(motion[:, 4]).astype(int).tolist(),
                       gait_codes=np.unique(motion[:, 5]).astype(int).tolist())
            if not np.all(motion[:, 4] == 17):
                row['reasons'].append('非正常运动控制区间')
            elif not np.all(motion[:, 5] == 4097):
                row['reasons'].append('不是基础步态 0x1001')
            elif row['max_body_tilt_deg'] > 35:
                row['reasons'].append('大倾角，先检查失稳/干预/障碍')
            else:
                obstacle = record['manifest']['terrain'] != 'basic' or any(
                    o.get('terrain_candidate') in ('stairs_candidate', 'ledge_candidate') and o['start_s'] < b and o['end_s'] > a
                    for o in notes.get('observations', []))
                if any(a < c+.5 and b > c-.5 for c in changes):
                    row['category'] = 'obstacle_context'; row['reasons'].append('控制/步态切换前后 0.5 秒')
                elif obstacle:
                    row['category'] = 'obstacle_context'; row['reasons'].append('障碍录制或已有障碍候选；平台/接近段需确认')
                elif row['motion_candidate'] == 'stationary':
                    row['category'] = 'stationary'
                else:
                    row['category'] = {'level_candidate': 'ground_motion', 'slope_candidate': 'varied_ground',
                                       'uneven_or_edge': 'varied_ground', 'unknown': 'uncertain_ground'}[geo]
        if manual:
            terrains = {r.get('terrain', 'unknown') for r in manual if r.get('human_review_status') == 'confirmed'}-{'unknown'}
            if len(terrains) == 1:
                row['terrain_human'] = next(iter(terrains))
                if row['category'] != 'exclude':
                    if row['terrain_human'] in ('stairs', 'ledge'):
                        row['category'] = 'obstacle_context'
                    elif row['terrain_human'] in ('slope', 'uneven') and row['motion_candidate'] != 'stationary':
                        row['category'] = 'varied_ground'
                    elif row['terrain_human'] == 'flat' and row['motion_candidate'] != 'stationary':
                        row['category'] = 'ground_motion'
            if len(terrains) > 1:
                row['reasons'].append('人工地形标注冲突')
                if row['category'] != 'exclude': row['category'] = 'uncertain_ground'
            surfaces = {r.get('surface_type', 'unknown') for r in manual if r.get('human_review_status') == 'confirmed'}-{'unknown'}
            if len(surfaces) == 1:
                row['surface_type'] = surfaces.pop()
            if len(surfaces) > 1:
                row['reasons'].append('人工材质标注冲突')
                if row['category'] != 'exclude': row['category'] = 'uncertain_ground'
            if any(r.get('reference_use') == 'exclude' for r in manual):
                row['category'] = 'exclude'; row['reference_use'] = 'exclude'; row['reasons'].append('人工排除')
            elif any(r.get('human_review_status') == 'confirmed' and r.get('notes') and not r['notes'].replace('废数据', '').strip() for r in manual):
                row['category'] = 'exclude'; row['reasons'].append('已有人工作废备注；保留原日志')
            elif any(r.get('reference_use') == 'candidate' and r.get('human_review_status') == 'confirmed' for r in manual):
                row['reference_use'] = 'candidate'
        rows.append(row)
    return rows


def merge_windows(rows):
    clips = []
    fields = ('recording_id', 'category', 'geometry_candidate', 'motion_candidate', 'surface_type', 'terrain_human', 'reference_use')
    for row in rows:
        if clips and all(clips[-1][k] == row[k] for k in fields) and abs(clips[-1]['end_s']-row['start_s']) < 1e-8 and row['end_s']-clips[-1]['start_s'] <= 16:
            clip = clips[-1]; clip['end_s'] = row['end_s']; clip['duration_s'] = clip['end_s']-clip['start_s']
            clip['windows'].append(row)
        else:
            clips.append({**{k: row[k] for k in fields}, 'start_s': row['start_s'], 'end_s': row['end_s'],
                          'duration_s': row['duration_s'], 'windows': [row]})
    for i, clip in enumerate(clips, 1):
        clip['clip_id'] = f'B{i:03d}'
        clip['reference_ready'] = False
        anchor = int(clip['windows'][0]['anchor_ns'])
        clip['source_start_ns'] = str(anchor+round(clip['start_s']*1e9))
        clip['source_end_ns'] = str(anchor+round(clip['end_s']*1e9))
        clip['review_url'] = f"viewer.html?id={clip['recording_id']}&t={clip['start_s']:.9f}&end={clip['end_s']:.9f}&range=basic"
        clip['reasons'] = sorted({s for w in clip['windows'] for s in w['reasons']})
    return clips


def plot_record(record, data, planes, rows):
    anchor = int(record['anchor_ns']); sid = record['id']
    fig, axes = plt.subplots(3, 1, figsize=(13, 7), sharex=True)
    for side, color in (('front', '#1864ab'), ('rear', '#d9480f')):
        rr = [r for r in planes if r['side'] == side and r['support'] >= .7]
        axes[0].scatter([r['t'] for r in rr], [r['tilt_deg'] for r in rr], s=5, label=side, color=color)
    axes[0].axhline(4, ls='--', color='gray'); axes[0].set_ylabel('局部平面倾角 °'); axes[0].legend()
    axes[1].plot((data['IMU_src']-anchor)/1e9, angles(data['IMU_v'][:, :4])[:, :2], lw=.8)
    axes[1].set_ylabel('机身 roll / pitch °')
    categories = ['ground_motion', 'varied_ground', 'stationary', 'obstacle_context', 'uncertain_ground', 'exclude']
    for row in rows:
        i = categories.index(row['category']); axes[2].plot([row['start_s'], row['end_s']], [i, i], lw=5, color=['#247c57','#d98526','#8091a5','#855bbb','#767c83','#a83d40'][i])
    axes[2].set_yticks(range(len(categories)), [LABELS[k] for k in categories], fontsize=8)
    for ax in axes: ax.grid(alpha=.2); ax.set_xlim(0, record['duration_s'])
    axes[2].set_xlabel('原录制源秒（跨流时钟/外参未校准）')
    fig.suptitle(sid+' | 材质不能由平面拟合确定；坡度仅为候选')
    fig.tight_layout(); fig.savefig(OUT/'previews'/f'{sid}_basic_screen.png', dpi=110); plt.close(fig)


def write_report(records, clips, all_windows):
    totals = Counter()
    for clip in clips: totals[clip['category']] += clip['duration_s']
    summary = dict(recordings=len(records), duration_s=sum(r['duration_s'] for r in records),
                   clips=len(clips), category_duration_s=dict(totals), confirmed_training_seconds=0,
                   material_note='草地/硬地/碎石只能由人工材质标注提供，几何候选不推断草地。',
                   plane_assumptions='base_link 点云按近邻 IMU roll/pitch 旋转；无平移补偿、无配准、无独立外参验证。',
                   thresholds=dict(window_s=2, level_below_deg=4, plane_support=.7, plane_residual_p80_m=.06,
                                   front_rear_height_agreement_m=.12, front_rear_grade_agreement_deg=5,
                                   body_tilt_holdout_deg=35))
    dump(OUT/'basic_gait_summary.json', summary); dump(OUT/'basic_gait_clips.json', clips)
    dump(OUT/'basic_gait_windows.json', all_windows)
    intro = ['# 基础步态参考片段整理', '',
             '范围：新 17 段全部录制，按实际状态 17 / 基础步态 0x1001 筛选；不按整段 basic 标签直接放行。',
             '这是待人工确认的参考运动索引，没有导出已批准训练集、没有世界位置轨迹、没有开始训练。',
             '平面几何与表面材质分开：草地可以是平的或有坡度；起伏点云也可能是边缘、杂物或外参误差。未将任何未知材质自动标为草地。', '',
             '[打开筛选页](http://127.0.0.1:8767/basic_gait_review.html)；点击片段进入原 3D 动作/点云复核，填写地形、材质和参考用途后保存。',
             '人工保存后重跑 `python -s -B artifacts/s10-recording-review-20260909/organize_basic_gait.py` 更新索引；自动判断不会改写人工日志。', '',
             '| 分组 | 秒数 |', '|---|---:|']
    intro += [f'| {LABELS[k]} | {v:.3f} |' for k, v in totals.items()]
    intro += ['', '所有秒数互斥，合计覆盖整批；前两组也只是候选，不是训练可用时长。',
              '“报告前向/后向”等方向来自 MOTION_INFO，速度有效性与指令时钟尚未验证。停止段单列以便限制抽样占比。',
              '障碍附近即使处于基础步态且局部地面平整，也保留为上下文；可经人工确认后纳入平台/接近运动。', '',
              '局部平面使用约 2 Hz 抽样点云、15 cm 网格中值、RANSAC 和前后雷达一致性。4° 等阈值是筛查参数，不是测量精度或地形真值。基于 2 秒窗口提出边界，接触/转向细节应加载精细回放再调整。', '']
    details = []
    for record in records:
        sid = record['id']; cc = [c for c in clips if c['recording_id'] == sid]
        details += [f'## {sid}', '', f'[信号与地形候选图](previews/{sid}_basic_screen.png)', '',
                    '| 片段 | 源秒 | 分组 | 地形几何 | 动作 |', '|---|---|---|---|---|']
        details += [f"| [{c['clip_id']}]({c['review_url']}) | {c['start_s']:.2f}–{c['end_s']:.2f} | {LABELS[c['category']]} | {LABELS[c['geometry_candidate']]} | {LABELS[c['motion_candidate']]} |" for c in cc]
        details.append('')
    (OUT/'BASIC_GAIT_REVIEW.md').write_text('\n'.join(intro+details), encoding='utf-8')
    rows = []
    for c in clips:
        tilts = [w['plane_tilt_median_deg'] for w in c['windows'] if w['plane_tilt_median_deg'] is not None]
        slope = f'{np.median(tilts):.1f}°' if tilts else '—'
        surface = {'unknown':'待确认','hard':'硬地','grass':'草地','gravel':'碎石','mixed':'混合'}[c['surface_type']]
        human = '' if c['terrain_human']=='unknown' else ' / 人工：'+{'flat':'近水平','slope':'坡面','uneven':'起伏','stairs':'楼梯','platform':'平台','ledge':'高台'}[c['terrain_human']]
        rows.append(f'<tr data-category="{c["category"]}"><td><a href="{html.escape(c["review_url"], quote=True)}">{c["clip_id"]} · {c["recording_id"][14:20]}</a></td><td>{c["start_s"]:.2f}–{c["end_s"]:.2f}</td><td>{LABELS[c["category"]]}</td><td>{LABELS[c["geometry_candidate"]]} {slope}{human}</td><td>{LABELS[c["motion_candidate"]]}</td><td>{surface}</td><td>{html.escape("；".join(c["reasons"]))}</td></tr>')
    options = ''.join(f'<option value="{k}">{LABELS[k]} · {v:.1f}s</option>' for k, v in totals.items())
    page = '<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>基础步态参考片段</title><link rel="stylesheet" href="viewer.css"><header><a href="index.html">全部录制</a><h1>基础步态参考片段</h1><p>17 段逐时段筛选 · 地面形状与表面材质分别复核</p></header><main><p class="notice">所有结果都是候选，尚无批准训练集。局部点云平整不能证明是硬地；未自动识别草地。单段已限制在 16 秒以内，可进入机器人与点云复核，选择地形、材质、参考用途。</p><p><a href="BASIC_GAIT_REVIEW.md">整理报告</a> · <a href="basic_gait_clips.json">片段索引 JSON</a> · <a href="basic_gait_summary.json">统计与阈值</a></p><label>筛选分组 <select id="filter"><option value="all">全部</option>'+options+'</select></label><p id="count"></p><table><thead><tr><th>片段 / 录制</th><th>源秒</th><th>用途分组</th><th>几何 / 候选倾角</th><th>运动</th><th>材质</th><th>依据</th></tr></thead><tbody>'+''.join(rows)+'</tbody></table><script>const filter=document.getElementById("filter");function show(){let n=0;document.querySelectorAll("tr[data-category]").forEach(r=>{r.hidden=filter.value!=="all"&&r.dataset.category!==filter.value;if(!r.hidden)n++;});document.getElementById("count").textContent=`显示 ${n} 段；人工保存后需重跑整理脚本刷新本页统计。`;}filter.onchange=show;filter.value="ground_motion";show();</script></main></html>'
    (OUT/'basic_gait_review.html').write_text(page, encoding='utf-8')
    return summary


def selfcheck():
    x, y = np.meshgrid(np.linspace(-2, 2, 31), np.linspace(-1.2, 1.2, 19))
    flat = np.c_[x.ravel(), y.ravel(), np.full(x.size, -.45)]
    assert fit_ground(flat)['tilt_deg'] < .01
    slope = flat.copy(); slope[:, 2] += np.tan(np.radians(9))*slope[:, 0]
    assert abs(fit_ground(slope)['tilt_deg']-9) < .01
    assert fit_ground(flat[:5]) is None
    step = flat.copy(); step[step[:, 0] > 0, 2] += .22
    assert fit_ground(step)['support'] < .7
    assert np.allclose(level_rotation([0, 0, 0, 1]), np.eye(3))
    times = np.arange(0, 2, .02)
    assert valid_window(times, np.ones((len(times), 2)), 0, 2, .06)[1]
    assert not valid_window(times[:30], np.ones((30, 2)), 0, 2, .06)[1]
    assert geometry([])[0] == 'unknown'
    assert np.allclose(level_rotation([2**-.5, 0, 0, 2**-.5]) @ [0, 1, 0], [0, 0, 1])
    times = np.arange(0, 4, .01)
    data = {k+'_src': (times*1e9).astype(np.int64) for k in ('IMU', 'JOINTS_DATA', 'MOTION_INFO')}
    data['IMU_v'] = np.tile([0, 0, 0, 1, 0, 0, 0, 0, 0, 9.81], (len(times), 1))
    data['JOINTS_DATA_v'] = np.ones((len(times), 48))
    data['MOTION_INFO_v'] = np.tile([.3, 0, 0, .45, 17, 4097], (len(times), 1))
    data['MOTION_INFO_v'][times >= 2, 5] = 4099
    record = dict(id='test', duration_s=4, anchor_ns='0', manifest={'terrain': 'basic'},
                  control_timeline=[dict(start_s=0), dict(start_s=2)])
    annotations = [dict(id='test-note', start_s=.5, end_s=1, terrain='flat', surface_type='grass',
                        reference_use='exclude', human_review_status='confirmed')]
    windows = screen(record, data, [], annotations, {})
    assert all(r['category']=='exclude' for r in windows if r['start_s']>=2 or .5<=r['start_s']<1)
    assert next(r for r in windows if r['start_s']==.5)['surface_type']=='grass'
    assert abs(sum(r['duration_s'] for r in merge_windows(windows))-4)<1e-8
    assert not any(r['reference_ready'] for r in windows)
    print('PASS: flat/slope/step/sparse clouds; rotation; missing stream; gait switch and human exclusion; complete candidate coverage.')


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('--selfcheck', action='store_true'); args = parser.parse_args()
    selfcheck()
    if args.selfcheck: return
    records = json.loads((OUT/'inventory.json').read_text(encoding='utf-8'))
    notes = json.loads((OUT/'observations.json').read_text(encoding='utf-8'))
    annotations = human_rows(); all_windows = []; clips = []
    (OUT/'previews').mkdir(exist_ok=True)
    for r in records:
        with np.load(OUT/'decoded'/f"{r['id']}.npz") as z:
            data = dict(z)
        planes = cloud_planes(r, data)
        rows = screen(r, data, planes, [h for h in annotations if h['recording_id'] == r['id']], notes.get(r['id'][14:20], {}))
        all_windows.extend(rows); clips.extend(merge_windows(rows)); plot_record(r, data, planes, rows)
        assert abs(sum(w['duration_s'] for w in rows)-r['duration_s']) < 1e-7
        assert all(abs(a['end_s']-b['start_s']) < 1e-9 for a, b in zip(rows, rows[1:]))
        print(r['id'], len(planes), 'plane hypotheses;', len(rows), 'windows', flush=True)
    for i, c in enumerate(clips, 1): c['clip_id'] = f'B{i:03d}'
    summary = write_report(records, clips, all_windows)
    assert abs(sum(summary['category_duration_s'].values())-summary['duration_s']) < 1e-6
    assert not any(c['reference_ready'] for c in clips)
    print(json.dumps(summary, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
