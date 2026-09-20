"""Offline lidar/ledge alignment; raw bags read-only, outputs beside this script.
Run: .venv-win/Scripts/python.exe -B artifacts/s10-ledge-match-20260909/match.py
"""
import base64
import csv
import json
from pathlib import Path
import sqlite3
import sys
import xml.etree.ElementTree as ET
import numpy as np
import mujoco

OUT = Path(__file__).resolve().parent
REPO = OUT.parents[1]
REVIEW = REPO / 'artifacts/s10-recording-review-20260909'
sys.path.insert(0, str(REVIEW))
import review
import validate_motion as replay

SID = 'gait_20260909_151135_f45a816f4062'


def rotation(xyzw):
    q = np.asarray(xyzw)[[3, 0, 1, 2]]
    w, x, y, z = q
    yaw = np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    q = replay.multiply([np.cos(yaw/2), 0, 0, -np.sin(yaw/2)], q)
    matrix = np.empty(9)
    mujoco.mju_quat2Mat(matrix, q)
    return matrix.reshape(3, 3)


def plane(points, vertical=False, max_vertical_z=.25):
    """Bounded RANSAC followed by orthogonal least-squares, in metres."""
    if len(points) < 40:
        return None
    # ponytail: local single-plane fit; use multi-surface segmentation for corners/curved obstacles.
    p = points[::max(1, len(points)//1800)]
    rng = np.random.default_rng(17)
    best = None
    for _ in range(240):
        a, b, c = p[rng.choice(len(p), 3, replace=False)]
        n = np.cross(b-a, c-a)
        norm = np.linalg.norm(n)
        if norm < 1e-8:
            continue
        n /= norm
        if vertical and (abs(n[2]) > max_vertical_z or abs(n[0]) < .85):
            continue
        if not vertical and abs(n[2]) < .94:
            continue
        mask = abs(p@n - a@n) < .018
        if mask.sum() < 35:
            continue
        span = np.ptp(p[mask], axis=0)
        if vertical and (span[2] < .18 or span[1] < .25):
            continue
        if best is None or mask.sum() > best.sum():
            best = mask
    if best is None:
        return None
    v = p[best]
    for _ in range(2):
        centre = v.mean(axis=0)
        n = np.linalg.svd(v-centre, full_matrices=False)[2][-1]
        if n[0 if vertical else 2] < 0:
            n = -n
        offset = float(centre@n)
        v = points[abs(points@n-offset) < .018]
    # Refinement may drift onto a neighbouring surface; retain the orientation constraint.
    if len(v)<35 or (vertical and (abs(n[2])>max_vertical_z or abs(n[0])<.85)) or (not vertical and abs(n[2])<.94):
        return None
    return dict(normal=n.tolist(), offset_m=offset, inliers=len(v),
                rms_m=float(np.sqrt(np.mean((v@n-offset)**2))),
                span_m=np.ptp(v, axis=0).tolist(),
                low_xyz=np.percentile(v, 2, axis=0).tolist(),
                high_xyz=np.percentile(v, 98, axis=0).tolist())


def fit(xyz, rot, max_vertical_z=.25):
    p = xyz @ rot.T
    local = p[(p[:,0] > .2) & (p[:,0] < 1.6) & (abs(p[:,1]) < .65)
              & (p[:,2] > -.8) & (p[:,2] < .18)]
    face = plane(local, vertical=True, max_vertical_z=max_vertical_z)
    if face is None:
        return None
    n = np.array(face['normal']); edge = face['offset_m']/np.linalg.norm(n[:2])
    # The lower support plane is fitted independently of the recorded height.
    ground_points = p[(p[:,0] > -2) & (p[:,0] < 1.5) & (abs(p[:,1]) < 1)
                      & (p[:,2] > -.9) & (p[:,2] < -.15)
                      & (p@n < face['offset_m']-.07)]
    ground = plane(ground_points)
    out = dict(edge_horizontal_distance_m=float(edge),
               face_yaw_deg=float(np.degrees(np.arctan2(n[1], n[0]))),
               face=face, ground=ground)
    if ground:
        # Wall's visible vertical extent is evidence, not a measured top plane.
        g = np.array(ground['normal'])
        x = face['offset_m']/n[0]
        gz = (ground['offset_m']-g[0]*x)/g[2]
        out['visible_face_height_m'] = float(face['high_xyz'][2]-gz)
        out['base_height_above_fitted_ground_m'] = float(-ground['offset_m']/g[2])
        out['ground_tilt_after_imu_deg'] = float(np.degrees(np.arccos(g[2])))
    return out


def extract():
    path = OUT/'full_clouds.npz'
    if path.exists():
        return np.load(path)
    folder = review.RAW/SID
    manifest = json.loads((folder/'manifest.json').read_text(encoding='utf-8'))
    store = review.store_types(); frames = []; times = []; topics = []
    for db in (folder/'bag').glob('*.db3'):
        with sqlite3.connect(db.resolve().as_uri()+'?mode=ro', uri=True) as conn:
            ids = dict(conn.execute("SELECT id,name FROM topics WHERE type='sensor_msgs/msg/PointCloud2'"))
            sql = 'SELECT topic_id,data FROM messages WHERE topic_id IN ('+','.join(map(str,ids))+') ORDER BY timestamp'
            for tid, raw in conn.execute(sql):
                msg = store.deserialize_cdr(raw, 'sensor_msgs/msg/PointCloud2')
                t = (msg.header.stamp.sec*10**9+msg.header.stamp.nanosec-manifest['started_wall_ns'])/1e9
                if not 0 <= t <= 17:
                    continue
                assert msg.header.frame_id == 'base_link'
                xyz = np.asarray(review.preview_points(msg, limit=msg.width*msg.height)['points'])
                xyz = xyz[(np.linalg.norm(xyz, axis=1) > .2) & (np.linalg.norm(xyz, axis=1) < 5)]
                frames.append(xyz.astype(np.float32)); times.append(t); topics.append(ids[tid])
    np.savez_compressed(path, xyz=np.concatenate(frames), offsets=np.r_[0,np.cumsum([len(v) for v in frames])],
                        time_s=times, topic=topics)
    return np.load(path)


def selfcheck():
    rng = np.random.default_rng(6)
    y = rng.uniform(-.6,.6,700); z = rng.uniform(-.4,-.02,700)
    wall = np.c_[.7-.1*y,y,z]+rng.normal(0,.002,(700,3))
    floor = np.c_[rng.uniform(-1,.55,900),rng.uniform(-.8,.8,900),np.full(900,-.4)]
    clutter = rng.uniform([.2,-.6,-.7],[1.5,.6,.15],(250,3))
    result = fit(np.r_[wall,floor,clutter], np.eye(3))
    assert abs(result['edge_horizontal_distance_m']-.7/np.sqrt(1.01)) < .008
    assert abs(result['visible_face_height_m']-.38) < .02
    assert abs(result['face_yaw_deg']-np.degrees(np.arctan(.1))) < 1
    assert plane(np.zeros((10,3)), True) is None


def survey():
    output=[]
    for path in sorted((REVIEW/'decoded').glob('*.meta.json')):
        meta=json.loads(path.read_text(encoding='utf-8'))
        if meta['manifest']['terrain']!='ledge':continue
        sid=meta['id'];d=np.load(REVIEW/'decoded'/f'{sid}.npz')
        it=(d['IMU_src']-meta['manifest']['started_wall_ns'])/1e9
        clouds=json.loads((REVIEW/'decoded'/f'{sid}.clouds.json').read_text())['/rslidar_front/points']
        samples=[]
        for t in [1.,5.,8.]:
            c=min(clouds,key=lambda v:abs(v['src']-t))
            xyz=np.frombuffer(base64.b64decode(c['xyz_mm_b64']),dtype='<i2').reshape(-1,3)/1000
            result=fit(xyz,rotation(d['IMU_v'][np.argmin(abs(it-c['src'])),:4]))
            samples.append(dict(time_s=c['src'],fit=result))
        output.append(dict(recording_id=sid,height_cm=meta['manifest']['parameters'].get('height_cm'),
            source='existing 2Hz/6000-point preview, screening only; selected clip uses full 10Hz raw clouds',samples=samples))
    review.dump(OUT/'other_recordings_survey.json',output)


def track(model, initial_qpos, initial_qvel, time, q, dq, normal, edge, height,
          delay=0., descending=False, kp_leg=80., kd_leg=2., kd_wheel=.6):
    """Same free-dynamics controller for baseline and distance/time experiments."""
    data=mujoco.MjData(model);data.qpos[:]=initial_qpos;data.qvel[:]=initial_qvel
    joints=model.actuator_trnid[:,0];qa=model.jnt_qposadr[joints];va=model.jnt_dofadr[joints]
    wheels=[model.body(n).id for n in ['fl_wheel','fr_wheel','hl_wheel','hr_wheel']]
    kp=np.full(16,kp_leg,dtype=float);kd=np.full(16,kd_leg,dtype=float);kp[replay.WHEELS]=0;kd[replay.WHEELS]=kd_wheel
    states=[];errors=[];logs=[];feet=[];top_contacts=[];first_fall=None;saturated=0;contacts={}
    steps=round((time[-1]-time[0])/.001)
    for step in range(steps):
        t=step*.001;reference_t=np.clip(t-delay,0,time[-1]-time[0])
        ix=min(int(reference_t/.005),len(time)-2);blend=(reference_t-ix*.005)/.005
        target=q[ix]*(1-blend)+q[ix+1]*blend;velocity=dq[ix]*(1-blend)+dq[ix+1]*blend
        torque=kp*(target-data.qpos[qa])+kd*(velocity-data.qvel[va])
        saturated+=np.count_nonzero((torque<model.actuator_ctrlrange[:,0])|(torque>model.actuator_ctrlrange[:,1]))
        data.ctrl[:]=np.clip(torque,model.actuator_ctrlrange[:,0],model.actuator_ctrlrange[:,1])
        mujoco.mj_step(model,data)
        if step%20==0:
            mujoco.mj_forward(model,data)
            assert np.isfinite(data.qpos).all()
            stamp=float(time[0]+t+.001)
            tilt=float(np.degrees(np.arccos(np.clip(data.xmat[1,8],-1,1))))
            if tilt>60 and first_fall is None:first_fall=stamp
            ledge_hit=False;wheel_top=np.zeros(4,dtype=bool)
            for c in data.contact[:data.ncon]:
                g1,g2=model.geom(c.geom1),model.geom(c.geom2)
                if g1.name!='matched_ledge' and g2.name!='matched_ledge':continue
                ledge_hit=True
                other=int(g2.bodyid[0] if g1.name=='matched_ledge' else g1.bodyid[0])
                name=model.body(other).name
                where='top' if abs(c.pos[2]-height)<.015 else 'face'
                contacts.setdefault(name+'_'+where,stamp)
                if other in wheels and abs(c.pos[2]-height)<.01 and abs(c.frame[2])>.95 and c.dist<=.001:
                    wheel_top[wheels.index(other)]=True
            states.append(data.qpos.copy());errors.append(data.qpos[qa]-target)
            feet.append(data.xpos[wheels].copy())
            top_contacts.append(wheel_top)
            logs.append([stamp,tilt,ledge_hit,float(np.min(data.xpos[wheels]@normal)),*data.qpos[:3]])
    mujoco.mj_forward(model,data)
    cleared=bool(np.min(data.xpos[wheels]@normal)>edge+.081)
    finished=bool(cleared and first_fall is None and abs(data.qpos[2]-(initial_qpos[2]+(-height if descending else height)))<.12
                  and np.max(abs(data.xpos[wheels]@np.array([-normal[1],normal[0],0])))<1-.081)
    result=dict(backend='MuJoCo, not Isaac Sim',free_dynamics=True,reached_top=finished and not descending,
        reached_destination=finished,descending=descending,
        all_wheels_cleared=cleared,first_tilt_over_60_source_s=first_fall,
        final_base_position_m=data.qpos[:3].tolist(),
        leg_tracking_rmse_deg=float(np.degrees(np.sqrt(np.mean(np.array(errors)[:,replay.LEGS]**2)))),
        actuator_saturation_fraction=float(saturated/(steps*16)),first_contacts_source_s=contacts,
        frames_with_ledge_contact=int(np.sum(np.array(logs)[:,2])),kp_leg=kp_leg,kd_leg=kd_leg,kd_wheel=kd_wheel)
    arrays=dict(qpos=np.array(states),log=np.array(logs),joint_error=np.array(errors),
                initial_qpos=initial_qpos,initial_qvel=initial_qvel,wheel_positions=np.array(feet),
                wheel_top_contact=np.array(top_contacts))
    return result,arrays


def export_and_track(rows, d, anchor, height):
    start, end = 9., 15.5
    time = np.arange(start, end+.0001, .005)
    jt = (d['JOINTS_DATA_src']-anchor)/1e9
    it = (d['IMU_src']-anchor)/1e9
    def interp(values, stamps):
        assert stamps[0] <= time[0] and stamps[-1] >= time[-1]
        return np.array([np.interp(time, stamps, column) for column in values.T]).T
    direction, offset = replay.calibration()
    q = interp(d['JOINTS_DATA_v'][:,:16], jt)*direction+offset
    dq = interp(d['JOINTS_DATA_v'][:,16:32], jt)*direction
    q[:,replay.WHEELS] -= q[0,replay.WHEELS]
    imu = d['IMU_v'][:,:4].copy()
    for i in range(1,len(imu)):
        if np.dot(imu[i-1],imu[i]) < 0:
            imu[i] *= -1
    quat = interp(imu,it); quat /= np.linalg.norm(quat,axis=1)[:,None]
    quat = replay.root_quaternions(quat)
    selected = [r for r in rows if abs(r['time_s']-start)<.21 and r['accepted_track']]
    edge = float(np.median([r['edge_horizontal_distance_m'] for r in selected]))
    yaw_deg = float(np.median([r['face_yaw_deg'] for r in selected]))
    yaw = np.deg2rad(yaw_deg); normal = np.array([np.cos(yaw),np.sin(yaw),0])
    tree = ET.parse(replay.ROOT/'simulation_review/free_base.xml')
    world = tree.getroot().find('worldbody')
    depth, width = 3., 2.
    centre = (edge+depth/2)*normal; centre[2]=height/2
    ET.SubElement(world,'geom',name='matched_ledge',type='box',
                  pos=' '.join(map(str,centre)),size=f'{depth/2} {width/2} {height/2}',
                  quat=f'{np.cos(yaw/2)} 0 0 {np.sin(yaw/2)}',
                  rgba='.48 .63 .75 1',friction='1 .01 .001',contype='1',conaffinity='1')
    model = mujoco.MjModel.from_xml_string(ET.tostring(tree.getroot(),encoding='unicode'))
    data = mujoco.MjData(model)
    joints = model.actuator_trnid[:,0]; qa=model.jnt_qposadr[joints]; va=model.jnt_dofadr[joints]
    names = [mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_JOINT,int(j)) for j in joints]
    wheels = [mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,n) for n in ['fl_wheel','fr_wheel','hl_wheel','hr_wheel']]
    data.qpos[:3]=0;data.qpos[3:7]=quat[0];data.qpos[qa]=q[0]
    mujoco.mj_forward(model,data)
    initial_z = float(.081-data.xpos[wheels,2].min()+.002)
    front_reach = float(np.max(data.xpos[wheels]@normal)+.081)
    initial = [0.,0.,initial_z]
    data.qpos[:3]=initial;data.qvel[va]=dq[0]
    mt=(d['MOTION_INFO_src']-anchor)/1e9
    motion=interp(d['MOTION_INFO_v'][:,:4],mt)
    matrix=np.empty(9);mujoco.mju_quat2Mat(matrix,quat[0])
    data.qvel[:3]=matrix.reshape(3,3)@[*motion[0,:2],0]
    data.qvel[3:6]=interp(d['IMU_v'][:,4:7],it)[0]
    mujoco.mj_forward(model,data)
    initial_qpos=data.qpos.copy()
    initial_penetration = float(min([c.dist for c in data.contact[:data.ncon]]+[0]))
    assert initial_penetration > -.003, 'Initial overlap: inspect calibration before tracking'
    limited=model.jnt_limited[joints].astype(bool); limits=model.jnt_range[joints]
    violations=np.maximum(limits[:,0]-q,q-limits[:,1]);violations[:,~limited]=0
    # Measured right-knee overshoot is ~0.003 rad; retain it and let physical limits act.
    assert violations.max()<.01, 'Joint mapping exceeds model limits by more than 0.01 rad'
    alignment=dict(recording_id=SID,start_s=start,end_s=end,height_m=height,
        height_source='manifest.parameters.height_cm; visible wall is not a complete independent height measurement',
        edge_horizontal_distance_m=edge,edge_normal_xy=normal[:2].tolist(),edge_yaw_deg=yaw_deg,
        front_wheel_outer_gap_m=edge-front_reach,initial_base_position_m=initial,
        initial_base_quaternion_wxyz=quat[0].tolist(),joint_names=names,
        platform_depth_m=depth,platform_width_m=width,
        assumed_geometry='3m depth/2m width are sufficient-size test geometry, not measured dimensions',
        coordinates='X forward at clip start; Z up; origin is initial base projected to floor; initial yaw removed',
        point_frame='declared base_link; no extra transform applied; physical lidar and IMU extrinsics unverified',
        initial_z_method='SDK kinematics puts lowest wheel 2mm above floor; lidar floor fit recorded separately',
        lidar_initial_base_height_m=float(np.median([r['base_height_above_fitted_ground_m'] for r in selected])),
        point_plane_residual_m=float(np.median([r['face']['rms_m'] for r in selected])),
        reference_kind='measured joint state, NOT original controller action; no reconstructed root translation',
        max_reference_joint_limit_excess_rad=float(max(0,violations.max())),
        isaac_sim_executed=False)
    review.dump(OUT/'alignment.json',alignment)
    np.savez_compressed(OUT/'expert_reference.npz',time_s=time,time_from_clip_start_s=time-start,
        joint_names=np.array(names),joint_position=q,joint_velocity=dq,base_quaternion_wxyz=quat,
        initial_base_position_m=initial,reported_motion=motion)
    keyframe=tree.getroot().find('keyframe')
    if keyframe is None:keyframe=ET.SubElement(tree.getroot(),'keyframe')
    ET.SubElement(keyframe,'key',name='matched_start',qpos=' '.join(map(str,initial_qpos)))
    tree.write(OUT/'matched_scene.xml',encoding='utf-8')
    def cube(name, pos, size, angle=0):
        return f'''    def Cube "{name}" (prepend apiSchemas = ["PhysicsCollisionAPI"]) {{
        double size = 1
        double3 xformOp:translate = ({', '.join(map(str,pos))})
        float xformOp:rotateZ = {angle}
        double3 xformOp:scale = ({', '.join(map(str,size))})
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateZ", "xformOp:scale"]
        color3f[] primvars:displayColor = [(0.48, 0.63, 0.75)]
    }}
'''
    usd = '#usda 1.0\n(defaultPrim = "World"\n metersPerUnit = 1\n upAxis = "Z")\ndef Xform "World" {\n    def PhysicsScene "PhysicsScene" {}\n'
    usd += cube('Ground',[0,0,-.05],[20,20,.1])+cube('MatchedLedge',centre,[depth,width,height],yaw_deg)+'}\n'
    (OUT/'matched_terrain.usda').write_text(usd,encoding='utf-8')
    result,arrays=track(model,initial_qpos,data.qvel.copy(),time,q,dq,normal,edge,height)
    result.update(initial_penetration_m=initial_penetration,
        initial_base_z_minus_lidar_ground_estimate_m=initial_z-alignment['lidar_initial_base_height_m'])
    states=arrays['qpos'];logs=arrays['log']
    review.dump(OUT/'tracking_result.json',result)
    np.savez_compressed(OUT/'tracking.npz',**arrays)
    print('TRACKING',json.dumps(result),flush=True)
    renderer=mujoco.Renderer(model,height=640,width=960)
    camera=mujoco.MjvCamera();camera.distance=2.5;camera.azimuth=120;camera.elevation=-18
    try:
        for k,t in enumerate([9.,11.,12.,12.5,13.,14.,15.4]):
            i=int(np.argmin(abs(np.array(logs)[:,0]-t)))
            data.qpos[:]=initial_qpos if k==0 else states[i];mujoco.mj_forward(model,data)
            camera.lookat[:]=[.6,0,.35]
            renderer.update_scene(data,camera=camera,scene_option=replay.VIEW_OPTIONS)
            rgb=renderer.render()
            (OUT/f'tracking_{k}.ppm').write_bytes(b'P6\n960 640\n255\n'+rgb.tobytes())
    finally:
        renderer.close()


def main():
    selfcheck()
    if '--survey' in sys.argv:
        survey()
    cloud = dict(extract()); meta = json.loads((REVIEW/'decoded'/f'{SID}.meta.json').read_text(encoding='utf-8'))
    d = np.load(REVIEW/'decoded'/f'{SID}.npz'); anchor = meta['manifest']['started_wall_ns']
    it = (d['IMU_src']-anchor)/1e9
    rows = json.loads((OUT/'fits.json').read_text()) if (OUT/'fits.json').exists() and '--refit' not in sys.argv else []
    for i, t in enumerate(cloud['time_s']) if not rows else []:
        if cloud['topic'][i] != '/rslidar_front/points':
            continue
        xyz = cloud['xyz'][cloud['offsets'][i]:cloud['offsets'][i+1]]
        rot = rotation(d['IMU_v'][np.argmin(abs(it-t)), :4])
        result = fit(xyz, rot)
        if result:
            rows.append(dict(time_s=float(t), **result))
    previous=None;ended=False
    for r in rows:
        if previous is not None and r['time_s']-previous['time_s']>.35:ended=True
        r['accepted_track']=not ended
        if not ended:previous=r
    review.dump(OUT/'fits.json', rows)
    with (OUT/'edge_distance.csv').open('w', newline='', encoding='utf-8') as f:
        fields = ['time_s','edge_horizontal_distance_m','face_yaw_deg','visible_face_height_m',
                  'base_height_above_fitted_ground_m','ground_tilt_after_imu_deg','accepted_track']
        w = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore'); w.writeheader(); w.writerows(rows)
    export_and_track(rows,d,anchor,meta['manifest']['parameters']['height_cm']/100)


if __name__ == '__main__':
    main()
