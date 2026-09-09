"""Try assumed human-scale stairs against recorded joint states; no robot connection.
Run: .venv-win/Scripts/python.exe artifacts/s10-expert-analysis/match_stairs.py
"""
import itertools
import json
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
import validate_motion as replay

OUT = replay.ROOT / 'stairs_match'
CLIPS = {'up': (7.0, 10.6), 'down': (13.4, 16.4),
         'up_repeat': (20.6, 24.0), 'down_repeat': (27.2, 30.2), 'up_extended': (7.0, 11.2)}


def terrain(height, depth, count, edge, descending):
    # Solid adjacent boxes, including a long landing; floor remains at z=0.
    spans = [(edge+i*depth, edge+(i+1)*depth, (count-i-1)*height if descending else (i+1)*height)
             for i in range(count)]
    if descending:
        spans.insert(0, (-10, edge, count*height))
    else:
        spans.append((edge+count*depth, edge+count*depth+10, count*height))
    return [(a, b, z) for a, b, z in spans if z > 0]


def model_for(params, front_x, name=None):
    height, depth, count, gap, descending = params
    edge = front_x + gap
    tree = ET.parse(replay.ROOT / 'simulation_review/free_base.xml')
    world = tree.getroot().find('worldbody')
    for i, (a, b, z) in enumerate(terrain(height, depth, count, edge, descending)):
        ET.SubElement(world, 'geom', name=f'stair_{i}', type='box',
                      pos=f'{(a+b)/2} 0 {z/2}', size=f'{(b-a)/2} 0.6 {z/2}',
                      rgba='0.48 0.59 0.68 1' if i % 2 else '0.63 0.72 0.78 1',
                      friction='1 0.01 0.01', contype='1', conaffinity='1')
    if name:
        tree.write(OUT / (name+'.xml'), encoding='utf-8')
    return mujoco.MjModel.from_xml_string(ET.tostring(tree.getroot(), encoding='unicode')), edge


def angles(quat):
    w, x, y, z = np.asarray(quat).T
    return np.degrees(np.array([np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
                               np.arcsin(np.clip(2*(w*y-z*x), -1, 1))])).T


def run(clip, params, record=None):
    start, end = CLIPS[clip]
    select = (D['time_s'] >= start-1e-8) & (D['time_s'] <= end+1e-8)
    q, dq = Q[select], DQ[select]
    quat = replay.root_quaternions(D['imu_orientation_xyzw'][select])
    times = D['time_s'][select] - start
    model, _ = model_for((0, .3, 0, 0, False), 0)
    data = mujoco.MjData(model)
    joints = model.actuator_trnid[:, 0]
    qa, va = model.jnt_qposadr[joints], model.jnt_dofadr[joints]
    wheels = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n)
              for n in ['fl_wheel', 'fr_wheel', 'hl_wheel', 'hr_wheel']]
    data.qpos[:3] = 0; data.qpos[3:7] = quat[0]; data.qpos[qa] = q[0]
    mujoco.mj_forward(model, data)
    # Gap is measured from the front wheel center, not the root.
    front_x = float(data.xpos[wheels, 0].max())
    initial_z = .081 - data.xpos[wheels, 2].min() + .002
    model, edge = model_for(params, front_x, record)
    data = mujoco.MjData(model)
    h, depth, count, gap, descending = params
    platform = h*count if descending else 0
    data.qpos[:3] = [0, 0, initial_z+platform]
    data.qpos[3:7] = quat[0]; data.qpos[qa] = q[0]; data.qvel[va] = dq[0]
    rotation = np.empty(9); mujoco.mju_quat2Mat(rotation, quat[0])
    motion = D['reported_motion_vx_vy_wz_height'][select][0]
    data.qvel[:3] = rotation.reshape(3, 3) @ [motion[0], motion[1], 0]
    data.qvel[3:6] = D['imu_angular_velocity'][select][0]
    mujoco.mj_forward(model, data)
    assert all(c.dist > -.003 for c in data.contact[:data.ncon]), 'Initial penetration'
    initial_position = data.qpos[:3].copy()
    kp = np.full(16, 80.); kd = np.full(16, 2.)
    kp[replay.WHEELS] = 0; kd[replay.WHEELS] = .6
    logs, errors, wheel_errors, poses = [], [], [], []
    first_fall = None; saturation = 0; stair_contact_frames = 0; body_contact_frames = 0
    video = replay.movie(model, record) if record else None
    try:
        for step in range(round((end-start)/.001)):
            t = step*.001
            a = min(int(t/.02), len(q)-2); blend = np.clip((t-times[a])/.02, 0, 1)
            target = q[a]*(1-blend)+q[a+1]*blend
            velocity = dq[a]*(1-blend)+dq[a+1]*blend
            torque = kp*(target-data.qpos[qa])+kd*(velocity-data.qvel[va])
            saturation += int(np.count_nonzero(abs(torque) > model.actuator_ctrlrange[:, 1]))
            data.ctrl[:] = np.clip(torque, model.actuator_ctrlrange[:, 0], model.actuator_ctrlrange[:, 1])
            mujoco.mj_step(model, data)
            if step % 20 == 0:
                mujoco.mj_forward(model, data)
                assert np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()
                rp = angles(data.qpos[3:7]); target_rp = angles(quat[a])
                body_hit = False; stair_hit = False
                for c in data.contact[:data.ncon]:
                    b1, b2 = int(model.geom_bodyid[c.geom1]), int(model.geom_bodyid[c.geom2])
                    if 0 in (b1, b2):
                        other = b2 if b1 == 0 else b1
                        ground_geom = c.geom1 if b1 == 0 else c.geom2
                        stair_hit |= (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, ground_geom) or '').startswith('stair_')
                        body_hit |= other not in wheels
                body_contact_frames += body_hit; stair_contact_frames += stair_hit
                tilt = np.degrees(np.arccos(np.clip(data.xmat[1, 8], -1, 1)))
                if tilt > 60 and first_fall is None: first_fall = t
                logs.append([start+t, *data.qpos[:3], *rp, *target_rp, body_hit, stair_hit])
                errors.append(data.qpos[qa]-target)
                wheel_errors.append(data.qvel[va[replay.WHEELS]]-velocity[replay.WHEELS])
                poses.append(data.qpos.copy())
            if video and step % 40 == 0: replay.frame(*video, model, data)
    finally:
        if video: replay.close_movie(video[0], video[2])
    log = np.asarray(logs); errors = np.asarray(errors)
    # Last riser is at edge+(count-1)*depth in either direction.
    cleared = bool(count and data.xpos[wheels, 0].min() > edge+(count-1)*depth+.081
                   and np.max(abs(data.xpos[wheels, 1])) < .6)
    desired_z = initial_z + (0 if descending else count*h)
    finished = bool(cleared and first_fall is None and abs(data.qpos[2]-desired_z) < .15)
    rp_error = (log[:, 4:6]-log[:, 6:8]+180) % 360-180
    pitch_rmse = float(np.sqrt(np.mean(rp_error[:, 1]**2)))
    leg_rmse = float(np.degrees(np.sqrt(np.mean(errors[:, replay.LEGS]**2))))
    result = dict(clip=clip, start_s=start, end_s=end, height_m=h, depth_m=depth, steps=count,
                  front_wheel_gap_m=gap, first_edge_x_m=edge, descending=descending,
                  pitch_rmse_deg=pitch_rmse, roll_pitch_rmse_deg=float(np.sqrt(np.mean(rp_error**2))),
                  leg_rmse_deg=leg_rmse, wheel_speed_rmse_rad_s=float(np.sqrt(np.mean(np.asarray(wheel_errors)**2))),
                  x_progress_m=float(data.qpos[0]), height_change_m=float(data.qpos[2]-initial_position[2]),
                  body_contact_fraction=body_contact_frames/len(log), stair_contact_fraction=stair_contact_frames/len(log),
                  first_tilt_over_60_s=first_fall, all_wheels_cleared=cleared, reached_final_landing=finished,
                  saturation_fraction=saturation/(round((end-start)/.001)*16),
                  score=pitch_rmse+.25*leg_rmse+30*(not finished)+60*(first_fall is not None)+5*body_contact_frames/len(log))
    if record:
        np.savez_compressed(OUT/(record+'.npz'), log=log, joint_error=errors, qpos=poses,
                            wheel_velocity_error=np.asarray(wheel_errors))
    return result


def selfcheck():
    for descending in (False, True):
        spans = terrain(.17, .28, 3, .5, descending)
        def z(x): return max([v for a, b, v in spans if a <= x < b]+[0])
        assert abs(z(.5-1e-5)-(.51 if descending else 0)) < 1e-9
        assert abs(z(.5+1e-5)-(.34 if descending else .17)) < 1e-9
        assert abs(z(.5+2*.28+1e-5)-(0 if descending else .51)) < 1e-9
    assert angles([np.cos(.1), 0, -np.sin(.1), 0])[1] < 0  # nose up, +x forward


if __name__ == '__main__':
    OUT.mkdir(exist_ok=True); selfcheck()
    D, Q, DQ, _, _, _ = replay.mapped_reference()
    replay.OUT = OUT
    results = []
    for clip in ('up', 'down'):
        descending = clip == 'down'
        results.append(run(clip, (0, .3, 0, 0, descending), clip+'_flat'))
        for h, depth, count, gap in itertools.product((.15, .17), (.28, .30), (2, 3, 4), (.15, .35, .55)):
            result = run(clip, (h, depth, count, gap, descending))
            results.append(result)
            print(json.dumps(result), flush=True)
        (OUT/'search.json').write_text(json.dumps(results, indent=2), encoding='utf-8')
    selected = []
    for clip in ('up', 'down'):
        best = min((r for r in results if r['clip'] == clip and r['steps']), key=lambda r:r['score'])
        params = tuple(best[k] for k in ('height_m','depth_m','steps','front_wheel_gap_m','descending'))
        selected.append(run(clip, params, clip+'_best'))
        selected.append(run(clip+'_repeat', params, clip+'_repeat'))
        if clip == 'up': selected.append(run('up_extended', params, 'up_extended'))
    (OUT/'selected.json').write_text(json.dumps(selected, indent=2), encoding='utf-8')
    print('SELECTED', json.dumps(selected, indent=2), flush=True)
