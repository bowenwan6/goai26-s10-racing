"""Source-first ascent crops after the user identified a robot-length platform.

No simulation result participates in choosing a crop. Reuse existing decoded
reference/FK and the existing read-only PointCloud2 reader; preserve prior outputs.
"""
import argparse
import json
from pathlib import Path
import numpy as np

BASE = Path(__file__).resolve().parent
OUT = BASE / 'cropped_ascent'
RULE = dict(max_wheel_height_spread_m=.045, max_abs_pitch_roll_deg=10.,
            min_level_duration_s=.2, max_level_tail_s=.5, after_peak_s=.3)


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def dump(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')


def level_window(time, quaternion, wheels, peak):
    w, x, y, z = quaternion.T
    pitch = np.degrees(np.arcsin(np.clip(2*(w*y-z*x), -1, 1)))
    roll = np.degrees(np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y)))
    spread = np.ptp(wheels[:, :, 2], axis=1)
    good = ((time >= peak+RULE['after_peak_s']) &
            (spread < RULE['max_wheel_height_spread_m']) &
            (np.maximum(abs(pitch), abs(roll)) < RULE['max_abs_pitch_roll_deg']))
    runs = np.flatnonzero(np.diff(np.r_[False, good, False])).reshape(-1, 2)
    # ponytail: first near-level FK window is a crop candidate, not proof of world contact.
    # Upgrade the boundary to measured contact/world pose when those channels exist.
    for first, stop in runs:
        if time[stop-1]-time[first] >= RULE['min_level_duration_s']-1e-8:
            last = min(stop-1, np.searchsorted(time, time[first]+RULE['max_level_tail_s']+1e-8)-1)
            return dict(level_start_s=round(float(time[first]), 3),
                        level_run_end_s=round(float(time[stop-1]), 3),
                        end_s=round(float(time[last]), 3),
                        next_out_of_window_s=round(float(time[stop]), 3) if stop < len(time) else None,
                        retained_level_duration_s=round(float(time[last]-time[first]), 3),
                        end_pitch_deg=float(pitch[last]), end_roll_deg=float(roll[last]),
                        end_wheel_height_spread_m=float(spread[last]))
    raise ValueError('No 0.2 s near-level source window; leave this clip unresolved')


def selfcheck():
    t = np.arange(51)*.02
    q = np.tile([1., 0, 0, 0], (len(t), 1)); wheels = np.zeros((len(t), 4, 3))
    wheels[:20, 0, 2] = .1; wheels[40:, 0, 2] = .1
    a = level_window(t, q, wheels, 0)
    assert a['level_start_s'] == .4 and a['end_s'] == .78
    wheels[:] = 0
    assert level_window(t, q, wheels, 0)['end_s'] == .8
    wheels[:, 0, 2] = .1
    try:
        level_window(t, q, wheels, 0)
    except ValueError:
        pass
    else:
        raise AssertionError('An absent landing must not produce a crop')


def prepare():
    selfcheck(); OUT.mkdir(exist_ok=True); crops = []
    for old in read(BASE/'clips.json'):
        if old['mode'] == 'down':
            continue
        source = BASE/old['id']; previous = BASE/'ascent'/old['id']
        ref = dict(np.load(source/'reference.npz')); fk = dict(np.load(source/'recorded_kinematics.npz'))
        indices = np.round((fk['time_s']-ref['time_s'][0])/.005).astype(int)
        assert np.allclose(ref['time_s'][indices], fk['time_s'], atol=1e-7, rtol=0)
        window = level_window(fk['time_s'], ref['base_quaternion_wxyz'][indices], fk['positions'][:, [4, 8, 12, 16]], old['peak'])
        end = window['end_s']; keep = ref['time_s'] <= end+1e-7
        clipped = {k: (v.copy() if k == 'initial_gyro' else v[keep]) for k, v in ref.items()}
        assert all(np.isfinite(v).all() for v in clipped.values())
        assert abs(clipped['time_s'][-1]-end) < 1e-7
        assert np.allclose(np.diff(clipped['time_s']), .005, atol=1e-7, rtol=0)
        for k, v in clipped.items():
            assert np.array_equal(v, ref[k] if k == 'initial_gyro' else ref[k][:len(v)])
        folder = OUT/old['id']; folder.mkdir(exist_ok=True)
        np.savez_compressed(folder/'reference.npz', **clipped)
        old_config = read(source/'configuration.json'); old_result = read(previous/'selected.json')
        config = dict(id=old['id'], recording=old['recording'], source_recording_id=old_config['source_recording_id'],
                      start=old['start'], end=end, peak=old['peak'], height=old['height'], mode='up',
                      time_basis=old_config['time_basis'], reference_kind=old_config['reference_kind'],
                      parent_end=old['end'], previous_ascent_end=read(previous/'configuration.json')['end'],
                      sample_count=len(clipped['time_s']), source_rule=RULE, window=window,
                      crop_reason='First near-level recorded joint/IMU phase, before later motion; robot-length platform reported by user',
                      removed_source_interval_s=[round(end+.005, 3), old['end']],
                      terminal_wheel_velocity_rad_s=clipped['joint_velocity'][-1, [3, 7, 11, 15]].tolist(),
                      terminal_behavior='End reference/episode at end; terminal feedback is not a zero-velocity standing command',
                      platform_depth_m=None, platform_width_m=None,
                      dimensions_status='User reports platform just fits robot and grass beyond; numerical depth not measured',
                      source_contact_status='FK near-level candidate, not measured world contact',
                      human_review='candidate_not_confirmed', training_ready=False,
                      cropped_reference_dynamics_executed=False, isaac_sim_executed=False,
                      previous_simulation=dict(platform_depth_m=3., up_completed_source_s=old_result['up_completed_source_s'],
                                               status='Historical long-platform result; not validation of this crop'))
        dump(folder/'configuration.json', config); crops.append(config)
        print('CROP', old['id'], old['start'], end, 'removed from previous ascent', round(config['previous_ascent_end']-end, 3), flush=True)
    dump(OUT/'clips.json', crops)


def extract_clouds():
    import batch_match as b
    crops = read(OUT/'clips.json'); b.OUT = OUT
    # Existing helper uses start-.25 .. peak-.1 to bound read-only SQL queries.
    b.CLIPS = [dict(recording=c['recording'], start=c['end']-.5, peak=c['end']+.85) for c in crops]
    for short in dict.fromkeys(c['recording'] for c in crops):
        meta, _ = b.load_record(short)
        b.clouds_for(short, meta)


def plot():
    import plots as p
    crops = read(OUT/'clips.json'); raw_clouds = {}
    fig, axes = p.plt.subplots(5, 2, figsize=(16, 17))
    rows = []
    for c, overview in zip(crops, axes.flat):
        source = BASE/c['id']; folder = OUT/c['id']
        ref = np.load(source/'reference.npz'); fk = np.load(source/'recorded_kinematics.npz')
        t = ref['time_s']; pitch, roll = p.angles(ref['base_quaternion_wxyz'])
        wheels = fk['positions'][:, [4, 8, 12, 16]]
        spread = np.ptp(wheels[:, :, 2], axis=1)*100
        view_end = min(c['parent_end'], max(c['previous_ascent_end'], c['end']+1.))
        view_start = max(c['start'], c['peak']-1.)
        overview.plot(t, pitch, c='#284860', label='实录俯仰 / °')
        overview.plot(fk['time_s'], spread, c='#c58430', label='四轮高度差 / cm')
        overview.axvspan(c['window']['level_start_s'], c['end'], color='#4a9e86', alpha=.22, label='保留的首次回平窗口')
        overview.axvspan(c['end'], view_end, color='#bc6877', alpha=.14, label='移出上台参考')
        overview.axvline(c['end'], c='#227b61', lw=1.7)
        overview.axvline(c['previous_ascent_end'], c='#ab7180', ls='--', lw=1)
        overview.set(xlim=(view_start, view_end), ylim=(-55, 50), title=f"{c['id']}  保留 {c['start']:.2f}–{c['end']:.2f} s", xlabel='原始记录时间 / s')
        overview.grid(alpha=.15)
        if c == crops[0]: overview.legend(fontsize=8, loc='lower left', ncol=2)
        # Each lidar frame uses its own source timestamp for IMU levelling.
        if c['recording'] not in raw_clouds:
            raw_clouds[c['recording']] = dict(np.load(OUT/(c['recording']+'_clouds.npz')))
        clouds = raw_clouds[c['recording']]; evidence = []
        detail, axs = p.plt.subplots(2, 3, figsize=(15, 8), gridspec_kw={'height_ratios': [1.3, 1]})
        for j, target in enumerate([c['end']-.2, c['end'], c['end']+.5]):
            ax = axs[0, j]; nearest = np.argmin(abs(fk['time_s']-target)); bones = fk['positions'][nearest]
            timestamps = {}
            for topic, color in [('/rslidar_front/points', '#be892e'), ('/rslidar_rear/points', '#78a1b1')]:
                ix = np.flatnonzero(clouds['topic'] == topic)
                k = ix[np.argmin(abs(clouds['time_s'][ix]-target))]; actual = float(clouds['time_s'][k])
                assert abs(actual-target) < .12, (c['id'], target, actual)
                q = ref['base_quaternion_wxyz'][np.argmin(abs(t-actual))]
                points = clouds['xyz'][clouds['offsets'][k]:clouds['offsets'][k+1]] @ p.matrix(q).T
                points = points[(abs(points[:, 1]) < .55) & (abs(points[:, 0]) < 1.3) & (points[:, 2] > -1.) & (points[:, 2] < .4)]
                ax.scatter(points[:, 0], points[:, 2], s=.6, c=color, alpha=.35)
                timestamps[topic] = actual
            for leg, color in enumerate(p.COLORS):
                ix = np.arange(1+4*leg, 5+4*leg)
                ax.plot(bones[ix, 0], bones[ix, 2], '-o', c=color, lw=1.7, ms=3)
                ax.add_patch(p.plt.Circle(bones[ix[-1], [0, 2]], .081, fill=False, color=color))
            title = ['裁剪前', '参考终点', '被移除的后续'][j]
            ax.set(xlim=(-1.3, 1.3), ylim=(-1., .4), title=f'{title} {target:.2f} s', xlabel='相对机身 X / m', ylabel='相对机身 Z / m')
            ax.set_aspect('equal'); ax.grid(alpha=.15)
            evidence.append(dict(target_source_s=round(target, 3), skeleton_source_s=float(fk['time_s'][nearest]), lidar_source_s=timestamps))
        axs[1, 0].plot(t, pitch, label='俯仰'); axs[1, 0].plot(t, roll, label='侧倾'); axs[1, 0].set_ylabel('实录姿态 / °')
        for j, col in enumerate(p.COLORS):
            axs[1, 1].plot(fk['time_s'], wheels[:, j, 2], c=col, label=['左前', '右前', '左后', '右后'][j])
            axs[1, 2].plot(t, ref['joint_velocity'][:, 3+4*j], c=col, label=['左前', '右前', '左后', '右后'][j])
        axs[1, 1].set_ylabel('轮心相对机身 Z / m'); axs[1, 2].set_ylabel('映射后轮速 / rad/s')
        for ax in axs[1]:
            ax.axvspan(c['window']['level_start_s'], c['end'], color='#4a9e86', alpha=.2)
            ax.axvspan(c['end'], view_end, color='#bc6877', alpha=.12)
            ax.axvline(c['end'], c='#227b61'); ax.set_xlim(view_start, view_end); ax.set_xlabel('原始记录时间 / s')
            ax.grid(alpha=.15); ax.legend(fontsize=8, ncol=2)
        detail.suptitle(f"{c['id']} · 实录裁剪依据 · 保留 {c['start']:.2f}–{c['end']:.2f} s", fontsize=15)
        detail.text(.5, .02, '骨架：实录关节 + IMU，机身居中；橙/蓝：原始前/后雷达约 10 Hz。近回平不等于已测得四轮接触。', ha='center', fontsize=10)
        detail.tight_layout(rect=[0, .05, 1, .95]); detail.savefig(folder/'crop_evidence.png', dpi=130); p.plt.close(detail)
        dump(folder/'evidence_times.json', evidence)
        rows.append(f"| [{c['id']}]({c['id']}/crop_evidence.png) | {c['start']:.2f}–{c['end']:.2f} | {c['previous_ascent_end']:.2f} | {c['previous_ascent_end']-c['end']:.2f} | {c['window']['retained_level_duration_s']:.2f} |")
    fig.suptitle('上台参考裁剪 · 先按实录定边界，再做仿真匹配', fontsize=18)
    fig.tight_layout(rect=[0, 0, 1, .97]); fig.savefig(OUT/'overview.png', dpi=130); p.plt.close(fig)
    report(rows)


def report(rows):
    text = '''# 短高台：上台参考裁剪与边界证据

用户补充：实物高台沿行进方向只够容纳机器狗，再向前会进入草地。本次已为剩余四条录制的 **10 个上台候选导出更短的参考**，只保留接近、上台及首次回平短窗口。没有修改原始录制、此前参考和人工标注。

此前用深 3 m 的高台得到的 10/10 支撑结果只适用于那套测试场景，**不能作为真实短高台或本次裁剪的通过率**。本次按实录选择裁剪位置，不按仿真是否成功选择终点；没有执行新裁剪的动力学回放，也没有执行 Isaac Sim。

## 裁剪结果

所有时间均为原始 source 时间。点击片段查看终点前、终点、后续半秒的真实关节/IMU与前后点云，以及连续姿态、轮高和轮速。

| 片段 | 新参考 / s | 上轮上台终点 / s | 相比上轮减少 / s | 保留首次回平窗口 / s |
|---|---|---:|---:|---:|
'''+'\n'.join(rows)+'''

![裁剪总览](overview.png)

绿色区间是保留的首次回平窗口，红色区间从本次终点开始，不再交给上台参考；红虚线为上轮终点。红色表示“移出本次上台任务”，并非断言整段实录失败。

## 边界如何确定

使用实录 200 Hz 关节/IMU参考及 SDK 计算的 50 Hz 四轮 FK。在已标出的上台抬头峰值后 0.3 s 起，寻找首个连续至少 0.2 s 的候选窗口：俯仰和侧倾绝对值都低于 10°，四轮轮心相对机身的世界竖直方向高度极差小于 4.5 cm。最多保留该窗口的前 0.5 s；若更早越过条件边界，则在前一个有效 FK 样本结束。边界分辨率为 20 ms，不能解释成精确接触时刻。

这是一致的实录裁剪规则，阈值保存在 `configuration.json`；没有读取仿真结果来延长或缩短窗口。每段 `reference.npz` 均与此前完整参考对应前缀逐项相同，没有重定时、平滑、拼接、修改末帧或补造世界位移。

150705_C1/C2/C3 在首次回平后较快出现前后轮高度分离和低头，新的终点分别为 12.14、29.48、41.80 s，可去掉先前已包含的部分后续动作。结合用户对短台和草地的说明，这些后段应作为离开台面/接触草地的待复核阶段，不能继续统一视作上台控制失败。其他片段主要裁去台上继续运动或较长尾段；150146_C1 仅缩短 0.04 s，说明上一轮的边界已经接近这里的规则。

## 点云能说明什么

本轮重新从原始 bag 只读提取终点附近的约 10 Hz 前后点云。每帧使用自己的 source 时间匹配 IMU，图中目标时刻、骨架时刻、两个雷达时刻分别保存到 `evidence_times.json`。前后雷达帧并非严格同时，图为各自机身坐标下的近时刻对照，没有假装恢复世界平移。

后方低地与上台姿态变化可以互相参照；但前方草丛、遮挡及外参的不确定性，使远侧台沿仍不能稳定拟合为单一边界。本轮没有从“刚好容纳机器狗”编造一个精确深度，`platform_depth_m=null`。仅靠四轮高度接近，不能证明它们都接触同一实体台面。因此输出是“上台参考裁剪候选”，保留 `training_ready=false`。

## 使用方式与这次的经验

优先使用本目录每段的 `reference.npz` 和 `configuration.json`。参考仍是测量关节状态，不是厂家原动作。所有列的关节顺序、坐标和标定沿用 [使用方法](../../../docs/S10_LEDGE_MATCHING_GUIDE_ZH.md)。终点的轮速通常不为零：到终点应结束该参考片段/评估 episode，或切换已有的站立控制；不能把最后一行持续循环当作站稳命令，也不能把后段草地动作继续接到上台参考。

场景重建应分别记录近侧台沿距离、高度、远侧台沿/深度。近侧距离拟合得对，不表示远侧边界也对。获得实物深度后，需用有限深度碰撞体重新检查接触和姿态；此前 `ascent/selected_scene.xml` 与 `selected_terrain.usda` 仍是历史 3 m 深场景，本目录不复制它们作为已验证短台场景。

复现（仓库根目录，现有依赖）：

```powershell
python -s -B artifacts/s10-ledge-batch-20260910/crop_ascent.py
.venv-win/Scripts/python.exe -B artifacts/s10-ledge-batch-20260910/crop_ascent.py --clouds
python -s -B artifacts/s10-ledge-batch-20260910/crop_ascent.py --plot
```

第一步包含最小规则自检，验证短窗口截断、0.5 s 上限与无有效窗口时拒绝裁剪，并检查全部导出是原参考的连续完整前缀、200 Hz 时间连续且数据有限。第二步沿用现有 PointCloud2 解析器，不修改原 bag。第三步生成证据图和本文档。
'''
    (OUT/'REPORT.md').write_text(text, encoding='utf-8')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); group = parser.add_mutually_exclusive_group()
    group.add_argument('--clouds', action='store_true'); group.add_argument('--plot', action='store_true')
    args = parser.parse_args()
    if args.clouds:
        extract_clouds()
    elif args.plot:
        plot()
    else:
        prepare()
