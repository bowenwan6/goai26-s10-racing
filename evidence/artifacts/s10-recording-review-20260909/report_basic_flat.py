"""Build plots/HTML, or videos of logged Isaac states rendered with the SDK MuJoCo mesh."""
import argparse
import json
import shutil
import subprocess
import sys

import numpy as np
from match_basic_flat import HERE, REPO, OUT, JOBS, LEGS, WHEELS, dump, interpolate, model_xml, rotation


def yaw(q):
    r = rotation(q)
    return np.degrees(np.unwrap(np.arctan2(r[:, 1, 0], r[:, 0, 0])))


def check_asset():
    import mujoco
    sys.path.append(str(REPO/'tmp/s10-analysis-deps'))
    from pxr import Usd, UsdPhysics
    path = OUT/'isaac_flat_scene.usdc'
    stage = Usd.Stage.Open(str(path if path.exists() else OUT/'isaac_flat_scene.usda'))
    model = mujoco.MjModel.from_xml_path(str(model_xml()))
    masses = {p.GetName():float(UsdPhysics.MassAPI(p).GetMassAttr().Get()) for p in stage.Traverse()
              if p.HasAPI(UsdPhysics.RigidBodyAPI)}
    assert len(masses) == 17
    for name, mass in masses.items():
        assert abs(float(model.body(name).mass[0])-mass) < 1e-6, name
    joints = {p.GetName() for p in stage.Traverse() if p.IsA(UsdPhysics.RevoluteJoint)}
    ref = np.load(OUT/'forward/reference.npz')
    assert joints == set(ref['joint_names']), joints
    assert not any(p.IsA(UsdPhysics.FixedJoint) for p in stage.Traverse())
    if not path.exists():
        stage.Export(str(path))
    dump(OUT/'asset_check.json', dict(sdk_mass_match=True, joint_names_match=True,
         fixed_joint_count=0, mass_by_body_name=masses, usd_file=path.name, usd_bytes=path.stat().st_size))
    print('ASSET CHECK OK: 17 SDK masses, 16 named joints, no fixed root')


def plots():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    rows, cards = [], []
    for name, _, _, _ in JOBS:
        folder = OUT/name
        r = dict(np.load(folder/'reference.npz'))
        s = dict(np.load(folder/'isaac.npz'))
        m = json.loads((folder/'isaac.json').read_text())
        tilt = np.degrees(np.arccos(np.clip(rotation(s['root_pose'][:, 3:])[:, 2, 2], -1, 1)))
        fallen = np.flatnonzero((tilt > 60) | (s['root_pose'][:, 2] < .15))
        m['first_fall_s'] = float(s['time_s'][fallen[0]]) if len(fallen) else None
        dump(folder/'isaac.json', m)
        source = json.loads((folder/'source.json').read_text())
        fig, axes = plt.subplots(3, 2, figsize=(13, 9), constrained_layout=True)
        axes[0, 0].plot(r['time_s'], np.degrees(r['joint_position_rad'][:, 2]), label='Recorded FL knee')
        axes[0, 0].plot(s['time_s'], np.degrees(s['joint_position_rad'][:, 2]), label='Isaac FL knee')
        axes[0, 0].set_ylabel('Joint angle (deg)')
        axes[0, 1].plot(r['time_s'], r['joint_velocity_rad_s'][:, 3], label='Recorded FL wheel')
        axes[0, 1].plot(s['time_s'], s['joint_velocity_rad_s'][:, 3], label='Isaac FL wheel')
        axes[0, 1].set_ylabel('Wheel velocity (rad/s)')
        for col, title in [(0, 'vx'), (1, 'vy')]:
            axes[1, 0].plot(r['time_s'], r['reported_velocity'][:, col], label=f'Reported {title} (unvalidated)')
            axes[1, 0].plot(s['time_s'], s['linear_velocity_body'][:, col], '--', label=f'Isaac {title}')
        axes[1, 0].set_ylabel('Body velocity (m/s assumed for recorded)')
        axes[1, 1].plot(r['time_s'], r['angular_velocity_body_rad_s'][:, 2], label='Recorded IMU gyro z')
        axes[1, 1].plot(s['time_s'], s['angular_velocity_body'][:, 2], label='Isaac gyro z')
        axes[1, 1].set_ylabel('Yaw rate (rad/s)')
        axes[2, 0].plot(r['time_s'], yaw(r['root_quaternion_wxyz']), label='Recorded IMU yaw')
        axes[2, 0].plot(s['time_s'], yaw(s['root_pose'][:, 3:]), label='Isaac yaw')
        axes[2, 0].set_ylabel('Unwrapped yaw (deg)')
        q, _ = interpolate(r['time_s'], r['joint_position_rad'], s['time_s'], .01)
        err = np.degrees(np.sqrt(np.mean((q[:, LEGS]-s['joint_position_rad'][:, LEGS])**2, axis=1)))
        axes[2, 1].plot(s['time_s'], err, label='12-leg joint RMSE')
        axes[2, 1].set_ylabel('Joint error (deg)')
        for ax in axes.flat:
            ax.set_xlabel('Clip time (s)')
            ax.grid(alpha=.2)
            ax.legend(fontsize=8)
        fig.suptitle(f'{name}: measured state vs free-base Isaac physics; no measured world XYZ')
        fig.savefig(folder/'comparison.png', dpi=135)
        plt.close(fig)
        m.update(name=name, source=source,
                 suggested_use='diagnostic_only' if m['fell'] else 'soft_reference_trial_pending_review',
                 recorded_gyro_z_mean=float(r['angular_velocity_body_rad_s'][:, 2].mean()),
                 simulated_gyro_z_mean=float(s['angular_velocity_body'][:, 2].mean()),
                 recorded_reported_velocity_mean=r['reported_velocity'][:, :2].mean(0).tolist(),
                 simulated_velocity_body_mean=s['linear_velocity_body'].mean(0).tolist())
        rows.append(m)
        cards.append(f'''<article id="{name}"><h2>{name} · {source['start_s']:g}–{source['end_s']:g} 秒</h2>
<p>{'诊断保留：本轮侧翻，不加入首轮训练试验组' if m['fell'] else '首轮软参考试验候选：仍需人工复核'}</p><p>腿角 RMSE {m['leg_rmse_deg']:.2f}° · 轮速 RMSE {m['wheel_speed_rmse_rad_s']:.2f} rad/s ·
重力方向误差 {m['gravity_error_rms_deg']:.2f}° · 摔倒阈值：{'触发' if m['fell'] else '未触发'}</p>
<p><a href="{source['review_url']}">打开原始点云与关节复核</a> · <a href="{name}/reference.npz">参考 NPZ</a> ·
<a href="{name}/isaac.json">Isaac 指标</a> · <a href="{name}/mujoco.json">MuJoCo 对照</a></p>
<video controls preload="metadata" src="{name}/comparison.mp4"></video><img loading="lazy" src="{name}/comparison.png"></article>''')
    dump(OUT/'summary.json', rows)
    dump(OUT/'rl_selection.json', dict(
        soft_reference_trials=[r['name'] for r in rows if not r['fell']],
        diagnostic_only=[r['name'] for r in rows if r['fell']],
        human_confirmation='pending', rl_training_performed=False,
        note='Use explicit clip names, not a glob over all reference.npz. Trial selection is not training certification.'))
    table = '\n'.join(f"| {r['name']} | {r['duration_s']:g} | {r['leg_rmse_deg']:.2f} | {r['wheel_speed_rmse_rad_s']:.2f} | {r['gravity_error_rms_deg']:.2f} | {r['orientation_error_rms_deg']:.2f} | {'是' if r['fell'] else '否'} |" for r in rows)
    (OUT/'REPORT.md').write_text(f'''# 基础步态平面匹配结果

2026-09-10；5 段共 34 秒。已实际执行 Isaac Sim 5.1 / PhysX CPU 自由基座动力学，另有 MuJoCo 3.11 对照。最终本地静态薄板场景中，3 段未摔倒，转向与侧向 2 段侧翻。不是 RL 训练结果。图表、视频与原复核入口见 [index.html](index.html)。

| 片段 | 秒 | 腿角 RMSE ° | 轮速 RMSE rad/s | 重力方向 RMS ° | 完整姿态 RMS ° | 摔倒 |
|---|---:|---:|---:|---:|---:|---|
{table}

全部使用同一套 PD：腿 kp=80、kd=2，轮 kp=0、kd=0.6；腿/轮力矩限幅 50/14 Nm。没有基座位置或姿态驱动，只有初始化设置一次根状态。未将实测力矩用作前馈。

场景为 200×200×0.1 m 的静态薄板，上表面 Z=0。`rl_selection.json` 将 forward、backward、second_recording 列为待人工确认的首轮软参考试验候选；turn、sideways 只保留诊断。此选择不等于训练认证，不能用 `glob` 将所有参考包直接装入训练。

`summary.json` 另列逐段实录/仿真平均角速度、报告速度以及来源。转向误差必须结合未包裹 yaw 曲线判读；四元数最短角误差无法累计超过一圈的航向偏差。静态平面摩擦系数 1 是仿真假设，不代表草地接触。

视频左侧为实录关节 + IMU 的运动学示意，固定机身 XYZ，仅更新实录关节与姿态；右侧是 Isaac 动力学日志。两侧都用本地 MuJoCo 渲染器画 SDK 网格，**不是 Isaac RTX 截屏，也不是恢复的实录世界路径**。运动来自右侧已完成的 Isaac 计算，视频不再施加控制。

所有数据仍为候选，未改写人工标注。`reference_valid` 仅表示传感器/控制连续性，`training_ready=false` 表示尚无人工训练资格确认和 RL 验收。使用方式见 [方法与经验](../../../docs/S10_BASIC_FLAT_MATCHING_ZH.md)。
''', encoding='utf-8')
    (OUT/'index.html').write_text('''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>S10 基础步态平面匹配</title><style>body{font:16px/1.65 system-ui;margin:28px auto;padding:0 20px;max-width:1250px;color:#172c3e;background:#f5f7fa}article{background:white;padding:22px;margin:28px 0;border:1px solid #d9e1e8;border-radius:10px}img,video{width:100%;display:block;margin:16px 0}a{color:#096699}h1,h2{line-height:1.3}</style>
<h1>S10 基础步态 · 平面软参考验证</h1><p>5 段 · 34 秒 · Isaac Sim 自由机身动力学已执行 · 人工训练资格待确认</p>
<p>左侧视频：固定机身 XYZ 的实录关节/IMU 姿态示意。右侧：Isaac 自由动力学结果。网格由 MuJoCo 渲染。没有实测世界 XYZ；图中报告速度暂未校准。</p>
<p>最终本地静态平板场景：3 段未摔倒；转向、侧向 2 段侧翻，保留诊断。没有摔倒也不代表运动完全匹配。<a href="../basic_gait_review.html">返回基础步态复核区</a></p>'''+''.join(cards)+'</html>', encoding='utf-8')
    print('REPORT', OUT/'index.html')


def videos():
    import mujoco
    model = mujoco.MjModel.from_xml_path(str(model_xml()))
    # Two separate renderings of the same collision-verified SDK model, no new simulation.
    model.vis.global_.offwidth, model.vis.global_.offheight = 640, 480
    renderer = mujoco.Renderer(model, height=480, width=640)
    camera = mujoco.MjvCamera()
    camera.distance, camera.azimuth, camera.elevation = 2.2, 125, -23
    option = mujoco.MjvOption()
    option.geomgroup[1] = 0
    qa = model.jnt_qposadr[model.actuator_trnid[:, 0]]
    data = mujoco.MjData(model)
    try:
        for name, _, _, _ in JOBS:
            folder = OUT/name
            r, s = dict(np.load(folder/'reference.npz')), dict(np.load(folder/'isaac.npz'))
            font = 'C\\:/Windows/Fonts/arial.ttf' if sys.platform == 'win32' else '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
            cmd = [shutil.which('ffmpeg'), '-hide_banner', '-loglevel', 'error', '-y', '-f', 'rawvideo',
                   '-pix_fmt', 'rgb24', '-s', '1280x480', '-r', '20', '-i', '-', '-an',
                   '-vf', f"drawtext=fontfile='{font}':text='Recorded q + IMU (fixed XYZ illustration)':x=16:y=16:fontsize=19:fontcolor=white:box=1:boxcolor=black@0.7,drawtext=fontfile='{font}':text='Isaac free-base dynamics (MuJoCo rendering)':x=656:y=16:fontsize=19:fontcolor=white:box=1:boxcolor=black@0.7",
                   '-c:v', 'libx264', '-crf', '22', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(folder/'comparison.mp4')]
            proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
            try:
                for t in np.arange(0, s['time_s'][-1], .05):
                    i = int(np.argmin(abs(s['time_s']-t)))
                    j = int(np.argmin(abs(r['time_s']-s['time_s'][i])))
                    images = []
                    for pos, quat, q in [(r['initial_root_pose'][:3], r['root_quaternion_wxyz'][j], r['joint_position_rad'][j]),
                                        (s['root_pose'][i, :3], s['root_pose'][i, 3:], s['joint_position_rad'][i])]:
                        data.qpos[:3], data.qpos[3:7], data.qpos[qa] = pos, quat, q
                        mujoco.mj_forward(model, data)
                        camera.lookat[:] = data.qpos[:3]
                        camera.lookat[2] -= .1
                        renderer.update_scene(data, camera=camera, scene_option=option)
                        images.append(renderer.render().copy())
                    proc.stdin.write(np.concatenate(images, axis=1).tobytes())
            finally:
                proc.stdin.close()
                assert proc.wait(timeout=30) == 0
            print('VIDEO', name, flush=True)
    finally:
        renderer.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--video', action='store_true')
    parser.add_argument('--check-asset', action='store_true')
    args = parser.parse_args()
    if args.check_asset:
        check_asset()
    else:
        videos() if args.video else plots()
