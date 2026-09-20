"""Prepare measured-state references, run flat-ground MuJoCo checks, or report Isaac results.

See docs/S10_BASIC_FLAT_MATCHING_ZH.md. Source bags and human reviews are read-only.
"""
import argparse
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
OUT = HERE / 'basic_flat_match'
SDK = REPO / 'upstream/goai_embodied_future_material/src/S10_sdk_deploy'
LEGS = np.array([i for i in range(16) if i % 4 != 3])
WHEELS = np.array([3, 7, 11, 15])
JOBS = [
    ('forward', '144907', 12., 18.),
    ('backward', '144907', 18., 24.),
    ('turn', '144907', 26., 34.),
    ('sideways', '144907', 62., 68.),
    ('second_recording', '151931', 24., 32.),
]
DT = .001


def dump(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')


def rotation(q):
    """Body-to-world rotation, quaternion wxyz; works with batches."""
    w, x, y, z = np.moveaxis(q, -1, 0)
    return np.stack([1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w),
                     2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w),
                     2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)], axis=-1).reshape(q.shape[:-1]+(3, 3))


def interpolate(t, values, target, max_gap):
    if len(t) < 2 or not np.all(np.diff(t) > 0) or not np.isfinite(values).all():
        raise ValueError('Invalid or nonmonotonic source stream')
    right = np.searchsorted(t, target, side='right').clip(1, len(t)-1)
    left = right-1
    valid = (target >= t[0]) & (target <= t[-1]) & (t[right]-t[left] <= max_gap)
    result = np.stack([np.interp(target, t, col) for col in values.T], axis=1)
    return result, valid


def model_xml():
    """Same SDK model, portable mesh paths regenerated on the current host."""
    tree = ET.parse(SDK / 'S10_description/s10_mjcf/mjcf/S10.xml')
    tree.getroot().find('compiler').set('meshdir', str(SDK / 'S10_description/s10_mjcf/meshes'))
    option = tree.getroot().find('option')
    if option is None:
        option = ET.SubElement(tree.getroot(), 'option')
    option.set('timestep', str(DT))
    path = OUT / 'flat_model.xml'
    tree.write(path, encoding='utf-8')
    return path


def prepare():
    import mujoco
    sys.path.insert(0, str(REPO / 'artifacts/s10-expert-analysis'))
    from validate_motion import calibration, root_quaternions
    direction, offset = calibration()
    model = mujoco.MjModel.from_xml_path(str(model_xml()))
    joints = model.actuator_trnid[:, 0]
    qa, va = model.jnt_qposadr[joints], model.jnt_dofadr[joints]
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, int(j)) for j in joints]
    wheels = [model.body(n+'_wheel').id for n in ('fl', 'fr', 'hl', 'hr')]
    candidates = json.loads((HERE / 'basic_gait_clips.json').read_text(encoding='utf-8'))
    reviews = {}
    journal = HERE / 'human_reviews.jsonl'
    for line in journal.read_text(encoding='utf-8').splitlines() if journal.exists() else []:
        row = json.loads(line)
        reviews[row['id']] = row
    for name, short, start, end in JOBS:
        path = next((HERE / 'decoded').glob(f'gait_20260909_{short}_*.npz'))
        rid = path.stem
        selected = [r for r in candidates if r['recording_id'] == rid
                    and r['start_s'] < end and r['end_s'] > start]
        assert selected and all(r['category'] == 'ground_motion' and r['geometry_candidate'] == 'level_candidate'
                                and r['reference_use'] != 'exclude' for r in selected)
        assert abs(sum(min(end, r['end_s'])-max(start, r['start_s']) for r in selected)-(end-start)) < 1e-6
        # Recheck the live journal, since screening output may predate a human rejection.
        for r in reviews.values():
            if r.get('recording_id') == rid and not r.get('retired', False):
                if float(r['start_s']) < end and float(r['end_s']) > start:
                    assert r.get('reference_use') != 'exclude', 'New human rejection overlaps reference'
                    if r.get('human_review_status') == 'confirmed':
                        assert r.get('terrain') not in ('slope', 'uneven', 'stairs', 'ledge'), 'New terrain label requires rescreening'
                        assert not (r.get('notes') and not r['notes'].replace('废数据', '').strip()), 'Human-marked waste data'
        raw = dict(np.load(path))
        anchor = int(json.loads(path.with_suffix('.meta.json').read_text(encoding='utf-8'))['manifest']['started_wall_ns'])
        t = start + np.arange(round((end-start)*200))/200
        jt = (raw['JOINTS_DATA_src']-anchor)/1e9
        it = (raw['IMU_src']-anchor)/1e9
        mt = (raw['MOTION_INFO_src']-anchor)/1e9
        j, jvalid = interpolate(jt, raw['JOINTS_DATA_v'], t, .06)
        iv = raw['IMU_v'].copy()
        for i in range(1, len(iv)):
            if iv[i, :4] @ iv[i-1, :4] < 0:
                iv[i, :4] *= -1
        imu, ivalid = interpolate(it, iv, t, .06)
        imu[:, :4] /= np.linalg.norm(imu[:, :4], axis=1)[:, None]
        quat = root_quaternions(imu[:, :4])
        motion, mvalid = interpolate(mt, raw['MOTION_INFO_v'][:, :4], t, .12)
        mi = (np.searchsorted(mt, t, side='right')-1).clip(0, len(mt)-1)
        discrete = raw['MOTION_INFO_v'][mi, 4:6]
        valid = jvalid & ivalid & mvalid & (discrete[:, 0] == 17) & (discrete[:, 1] == 4097)
        assert valid.all(), f'{name}: missing data/control transition; choose a continuous valid window'
        q = j[:, :16]*direction+offset
        q[:, WHEELS] -= q[0, WHEELS]
        dq = j[:, 16:32]*direction
        limited = model.jnt_limited[joints].astype(bool)
        ranges = model.jnt_range[joints]
        excess = np.maximum(ranges[:, 0]-q, q-ranges[:, 1])
        assert np.max(excess[:, limited]) < .01, 'Reference exceeds SDK joint limits'
        data = mujoco.MjData(model)
        data.qpos[:3] = 0
        data.qpos[3:7] = quat[0]
        data.qpos[qa] = q[0]
        mujoco.mj_forward(model, data)
        initial = np.r_[0., 0., .083-data.xpos[wheels, 2].min(), quat[0]]
        # Reported vx/vy treated as body-frame m/s only for this explicit initialization hypothesis.
        initial_v = rotation(quat[0]) @ np.r_[motion[0, :2], 0.]
        folder = OUT / name
        folder.mkdir(exist_ok=True)
        native = (jt >= start) & (jt < end)
        arrays = dict(time_s=t-start, source_time_s=t, source_ns=anchor+np.rint(t*1e9).astype('i8'),
                      joint_names=np.array(names), joint_position_rad=q, joint_velocity_rad_s=dq,
                      measured_torque_nm=j[:, 32:48]*direction, root_quaternion_wxyz=quat,
                      projected_gravity=-rotation(quat)[:, 2, :], angular_velocity_body_rad_s=imu[:, 4:7],
                      reported_velocity=motion[:, :3], state_gait=discrete.astype('i4'),
                      reference_valid=valid, root_position_valid=np.zeros(len(t), bool),
                      command_valid=np.zeros(len(t), bool), reported_velocity_valid=np.zeros(len(t), bool),
                      contact_label_valid=np.zeros(len(t), bool), initial_root_pose=initial,
                      initial_root_linear_velocity=initial_v, initial_root_angular_velocity_body=imu[0, 4:7],
                      native_joint_source_ns=raw['JOINTS_DATA_src'][native],
                      native_joint_values=raw['JOINTS_DATA_v'][native],
                      leg_position_weight=valid.astype(float)*.5, wheel_velocity_weight=valid.astype(float)*.2,
                      gravity_weight=valid.astype(float)*.2, angular_velocity_weight=valid.astype(float)*.1)
        if 'STEER_src' in raw:
            steer_t = (raw['STEER_src']-anchor)/1e9
            take = (steer_t >= start-1) & (steer_t < end+1)
            arrays.update(steer_source_ns=raw['STEER_src'][take], steer_receive_ns=raw['STEER_rx'][take],
                          steer_normalized_raw=raw['STEER_v'][take])
        np.savez_compressed(folder / 'reference.npz', **arrays)
        dump(folder / 'source.json', dict(recording_id=rid, start_s=start, end_s=end,
             anchor_ns=str(anchor), sample_hz=200, joint_dir=direction.tolist(), offset_rad=offset.tolist(),
             candidate_ids=[r['clip_id'] for r in selected], human_confirmed=False, training_ready=False,
             initial_velocity_assumption='MOTION_INFO vx/vy interpreted as body m/s; not calibrated truth',
             reference_kind='measured state, not JOINTS_CMD',
             review_url=f'../viewer.html?id={rid}&t={start}&end={end}&range=basic'))
        print('PREPARED', name, len(t), flush=True)


def controller(q, dq, target_q, target_dq):
    kp, kd = np.full(16, 80.), np.full(16, 2.)
    kp[WHEELS], kd[WHEELS] = 0., .6
    requested = kp*(target_q-q)+kd*(target_dq-dq)
    limits = np.full(16, 50.)
    limits[WHEELS] = 14.
    return np.clip(requested, -limits, limits), abs(requested) > limits


def target(ref, t):
    i = min(int(t*200), len(ref['time_s'])-1)
    j = min(i+1, len(ref['time_s'])-1)
    a = np.clip((t-ref['time_s'][i])*200, 0, 1)
    return tuple(ref[k][i]*(1-a)+ref[k][j]*a for k in ('joint_position_rad', 'joint_velocity_rad_s'))


def soft_motion_reward(ref, index, q, dq, gravity_body, angular_velocity_body):
    """NumPy reward example; q/dq use ref joint_names, no wheel phase or world XYZ target.

    Scalars or a batch of indices are accepted. The PPO implementation can translate these
    four expressions to torch. Tolerances are starting hypotheses, not tuned RL parameters.
    """
    errors = [np.mean(((q[..., LEGS]-ref['joint_position_rad'][index][..., LEGS])/.25)**2, axis=-1),
              np.mean(((dq[..., WHEELS]-ref['joint_velocity_rad_s'][index][..., WHEELS])/5.)**2, axis=-1),
              np.mean(((gravity_body-ref['projected_gravity'][index])/.25)**2, axis=-1),
              np.mean(((angular_velocity_body-ref['angular_velocity_body_rad_s'][index])/1.)**2, axis=-1)]
    terms = ('leg_position_weight', 'wheel_velocity_weight', 'gravity_weight', 'angular_velocity_weight')
    return ref['reference_valid'][index]*sum(ref[k][index]*np.exp(-e) for k, e in zip(terms, errors))


def run_mujoco(only):
    import mujoco
    model = mujoco.MjModel.from_xml_path(str(model_xml()))
    qa = model.jnt_qposadr[model.actuator_trnid[:, 0]]
    va = model.jnt_dofadr[model.actuator_trnid[:, 0]]
    floor = model.geom('floor').id
    wheels = {model.body(n+'_wheel').id for n in ('fl', 'fr', 'hl', 'hr')}
    for name, _, start, end in JOBS:
        if only and name != only:
            continue
        ref = dict(np.load(OUT / name / 'reference.npz'))
        data = mujoco.MjData(model)
        data.qpos[:7] = ref['initial_root_pose']
        data.qvel[:3] = ref['initial_root_linear_velocity']
        data.qvel[3:6] = ref['initial_root_angular_velocity_body']
        data.qpos[qa] = ref['joint_position_rad'][0]
        data.qvel[va] = ref['joint_velocity_rad_s'][0]
        mujoco.mj_forward(model, data)
        rows = []
        saturation = np.zeros(16)
        nonwheel = 0
        for step in range(round((end-start)/DT)):
            t = step*DT
            q, dq = target(ref, t)
            torque, sat = controller(data.qpos[qa], data.qvel[va], q, dq)
            data.ctrl[:] = torque
            mujoco.mj_step(model, data)
            saturation += sat
            nonwheel += int(any(floor in (c.geom1, c.geom2) and
                int(model.geom_bodyid[c.geom2 if c.geom1 == floor else c.geom1]) not in wheels
                for c in data.contact[:data.ncon]))
            if (step+1) % 20 == 0:
                r = rotation(data.qpos[3:7])
                rows.append(np.r_[(step+1)*DT, data.qpos[:7], data.qpos[qa], data.qvel[va],
                                  r.T@data.qvel[:3], data.qvel[3:6], torque])
        save_run(name, 'mujoco', rows, saturation/(step+1), dict(nonwheel_contact_steps=nonwheel,
                 engine_version=mujoco.__version__, physics_dt_s=DT, controller_dt_s=DT))


def save_run(name, engine, rows, saturation, extra):
    a = np.asarray(rows)
    assert np.isfinite(a).all(), 'Non-finite simulation result'
    np.savez_compressed(OUT / name / f'{engine}.npz', time_s=a[:, 0], root_pose=a[:, 1:8],
        joint_position_rad=a[:, 8:24], joint_velocity_rad_s=a[:, 24:40],
        linear_velocity_body=a[:, 40:43], angular_velocity_body=a[:, 43:46], torque_nm=a[:, 46:62])
    ref = dict(np.load(OUT / name / 'reference.npz'))
    q, _ = interpolate(ref['time_s'], ref['joint_position_rad'], a[:, 0], .01)
    dq, _ = interpolate(ref['time_s'], ref['joint_velocity_rad_s'], a[:, 0], .01)
    g, _ = interpolate(ref['time_s'], ref['projected_gravity'], a[:, 0], .01)
    rq, _ = interpolate(ref['time_s'], ref['root_quaternion_wxyz'], a[:, 0], .01)
    rq /= np.linalg.norm(rq, axis=1)[:, None]
    actual_g = -rotation(a[:, 4:8])[:, 2, :]
    gravity_err = np.degrees(np.arccos(np.clip(np.sum(g*actual_g, axis=1), -1, 1)))
    tilt = np.degrees(np.arccos(np.clip(-actual_g[:, 2], -1, 1)))
    orientation_error = np.degrees(2*np.arccos(np.clip(abs(np.sum(rq*a[:, 4:8], axis=1)), 0, 1)))
    metrics = dict(engine=engine, duration_s=float(a[-1, 0]), free_base=True,
        leg_rmse_deg=float(np.degrees(np.sqrt(np.mean((a[:, 8:24][:, LEGS]-q[:, LEGS])**2)))),
        wheel_speed_rmse_rad_s=float(np.sqrt(np.mean((a[:, 24:40][:, WHEELS]-dq[:, WHEELS])**2))),
        gravity_error_rms_deg=float(np.sqrt(np.mean(gravity_err**2))),
        orientation_error_rms_deg=float(np.sqrt(np.mean(orientation_error**2))),
        tilt_max_deg=float(tilt.max()), root_height_min_m=float(a[:, 3].min()),
        simulated_displacement_xyz_m=(a[-1, 1:4]-ref['initial_root_pose'][:3]).tolist(),
        fell=bool(np.any((tilt > 60) | (a[:, 3] < .15))),
        saturation_fraction_per_joint=np.asarray(saturation).tolist(), training_ready=False, **extra)
    dump(OUT / name / f'{engine}.json', metrics)
    print(name, json.dumps(metrics), flush=True)


def selfcheck():
    v, mask = interpolate(np.array([0., .01, .3]), np.array([[0.], [1.], [2.]]),
                          np.array([-.1, .005, .15, .4]), .06)
    assert mask.tolist() == [False, True, False, False] and v[1, 0] == .5
    q = np.zeros(16)
    dq = np.zeros(16)
    dq[WHEELS] = 10
    torque, sat = controller(q, q, q, dq)
    assert np.allclose(torque[WHEELS], 6) and not sat.any()
    torque, sat = controller(q, q, np.ones(16)*10, dq*10)
    assert np.all(torque[LEGS] == 50) and np.all(torque[WHEELS] == 14) and sat.all()
    assert np.allclose(rotation(np.array([1., 0, 0, 0])), np.eye(3))
    ref = dict(joint_position_rad=np.zeros((2, 16)), joint_velocity_rad_s=np.zeros((2, 16)),
               projected_gravity=np.tile([0., 0., -1.], (2, 1)), angular_velocity_body_rad_s=np.zeros((2, 3)),
               reference_valid=np.array([True, False]))
    for key, weight in zip(('leg_position_weight', 'wheel_velocity_weight', 'gravity_weight', 'angular_velocity_weight'),
                           (.5, .2, .2, .1)):
        ref[key] = np.full(2, weight)
    shifted = ref['joint_position_rad'].copy()
    shifted[:, WHEELS] = 1000  # Symmetric wheel phase cannot affect the imitation reward.
    reward = soft_motion_reward(ref, np.arange(2), shifted, ref['joint_velocity_rad_s'],
                                 ref['projected_gravity'], ref['angular_velocity_body_rad_s'])
    assert np.allclose(reward, [1., 0.])
    print('SELF CHECK OK')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['prepare', 'mujoco', 'selfcheck'])
    parser.add_argument('--only', choices=[r[0] for r in JOBS])
    args = parser.parse_args()
    selfcheck()
    OUT.mkdir(exist_ok=True)
    if args.mode == 'prepare':
        prepare()
    elif args.mode == 'mujoco':
        run_mujoco(args.only)
