"""S10 recorded-state replay and bounded MuJoCo tracking checks.

Run with .venv-win/Scripts/python.exe artifacts/s10-expert-analysis/validate_motion.py
Does not connect to ROS or hardware. Outputs live in simulation_review/.
"""
import argparse
import ast
from collections import Counter
import json
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET
import numpy as np
import mujoco

ROOT=Path(__file__).resolve().parent
REPO=ROOT.parents[1]
SDK=REPO/'upstream/goai_embodied_future_material/src/S10_sdk_deploy'
OUT=ROOT/'simulation_review'
WHEELS=np.array([3,7,11,15])
LEGS=np.array([i for i in range(16) if i not in WHEELS])
SID='gait_20260906_134324_93fa2317c970'
VIEW_OPTIONS=mujoco.MjvOption()
VIEW_OPTIONS.geomgroup[1]=0  # Render visual meshes, retain collision geometry in physics.

def calibration():
    tree=ast.parse((SDK/'interface/robot/simulation/mujoco_simulation_ros2.py').read_text(encoding='utf-8'))
    values={}
    for node in tree.body:
        if isinstance(node,ast.Assign) and isinstance(node.targets[0],ast.Name):
            key=node.targets[0].id
            if key in ('JOINT_DIR','POS_OFFSET_DEG'):
                values[key]=np.asarray(ast.literal_eval(node.value.args[0]),dtype=float)
    return values['JOINT_DIR'],np.deg2rad(values['POS_OFFSET_DEG'])

def multiply(a,b):
    result=np.empty(4);mujoco.mju_mulQuat(result,a,b);return result

def root_quaternions(xyzw):
    q=xyzw[:,[3,0,1,2]].copy()
    for i in range(1,len(q)):
        if np.dot(q[i-1],q[i])<0:q[i]*=-1
    w,x,y,z=q[0]
    yaw=np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))
    align=np.array([np.cos(yaw/2),0,0,-np.sin(yaw/2)])
    return np.array([multiply(align,v) for v in q])

def make_model(fixed=False):
    xml=SDK/'S10_description/s10_mjcf/mjcf/S10.xml'
    tree=ET.parse(xml);root=tree.getroot()
    root.find('compiler').set('meshdir',str(xml.parent.parent/'meshes'))
    option=root.find('option')
    if option is None:option=ET.SubElement(root,'option')
    option.set('timestep','0.001')
    visual=root.find('visual')
    if visual is None:visual=ET.SubElement(root,'visual')
    global_view=visual.find('global')
    if global_view is None:global_view=ET.SubElement(visual,'global')
    global_view.set('offwidth','960');global_view.set('offheight','640')
    base=root.find("worldbody/body[@name='base_link']")
    if fixed:
        base.remove(base.find('freejoint'));base.set('pos','0 0 0.85')
    path=OUT/('fixed_base.xml' if fixed else 'free_base.xml')
    tree.write(path,encoding='utf-8')
    model=mujoco.MjModel.from_xml_path(str(path))
    return model

def movie(model,name):
    renderer=mujoco.Renderer(model,height=640,width=960)
    camera=mujoco.MjvCamera();camera.distance=2.3;camera.azimuth=130;camera.elevation=-17
    ffmpeg=shutil.which('ffmpeg')
    if not ffmpeg:raise RuntimeError('ffmpeg not found')
    command=[ffmpeg,'-hide_banner','-loglevel','error','-y','-f','rawvideo','-pix_fmt','rgb24',
             '-s','960x640','-r','25','-i','-','-an','-c:v','libx264','-preset','veryfast',
             '-crf','22','-pix_fmt','yuv420p','-movflags','+faststart',str(OUT/(name+'.mp4'))]
    proc=subprocess.Popen(command,stdin=subprocess.PIPE)
    return renderer,camera,proc

def frame(renderer,camera,proc,model,data):
    base=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'base_link')
    camera.lookat[:]=data.xpos[base];camera.lookat[2]-=.1
    renderer.update_scene(data,camera=camera,scene_option=VIEW_OPTIONS)
    rgb=renderer.render();proc.stdin.write(rgb.tobytes())
    return rgb

def close_movie(renderer,proc):
    renderer.close();proc.stdin.close()
    if proc.wait(timeout=30):raise RuntimeError('Video encoder failed')

def percentile(v):
    return {'median':float(np.median(v)),'p95':float(np.percentile(v,95)),'max':float(np.max(v))}

def mapped_reference():
    d=np.load(ROOT/'references'/(SID+'_50hz.npz'))
    direction,offset=calibration()
    q=d['joint_position']*direction+offset
    dq=d['joint_velocity']*direction
    # Wheel phases are arbitrary for symmetric wheels; retain recorded revolutions relative to first frame.
    q[:,WHEELS]-=q[0,WHEELS]
    quat=root_quaternions(d['imu_orientation_xyzw'])
    return d,q,dq,quat,direction,offset

def kinematic(d,q,dq,quat,direction,offset):
    model=make_model();data=mujoco.MjData(model)
    joints=model.actuator_trnid[:,0];qa=model.jnt_qposadr[joints];va=model.jnt_dofadr[joints]
    limited=model.jnt_limited[joints].astype(bool);ranges=model.jnt_range[joints]
    excess=np.maximum(ranges[:,0]-q,q-ranges[:,1]);excess[:,~limited]=0
    names=[mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_JOINT,int(j)) for j in joints]
    # The full SDK mapping is invertible for all 12 leg channels.
    reconstructed=(q[:,LEGS]-offset[LEGS])*direction[LEGS]
    assert np.allclose(reconstructed,d['joint_position'][:,LEGS],atol=1e-10)
    assert np.all(np.isfinite(q)) and np.allclose(np.linalg.norm(quat,axis=1),1,atol=1e-6)
    renderer,camera,proc=movie(model,'kinematic_stairs_32s')
    contacts=[];positions=[];feet=[]
    wheel_bodies=[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,n) for n in ['fl_wheel','fr_wheel','hl_wheel','hr_wheel']]
    try:
        for i in range(len(q)):
            data.qpos[:3]=[0,0,.85];data.qpos[3:7]=quat[i];data.qpos[qa]=q[i]
            mujoco.mj_forward(model,data)
            contacts.append(sum(c.dist<-.002 for c in data.contact[:data.ncon]))
            positions.append(data.qpos.copy());feet.append(data.xpos[wheel_bodies].copy()-data.qpos[:3])
            if i%2==0:
                rgb=frame(renderer,camera,proc,model,data)
                if i in (0,500,750):
                    (OUT/f'kinematic_{i:04d}.ppm').write_bytes(f'P6\n960 640\n255\n'.encode()+rgb.tobytes())
    finally:close_movie(renderer,proc)
    # Check IMU quaternion changes against the simultaneously recorded gyro in its own frame.
    qsensor=d['imu_orientation_xyzw'][:,[3,0,1,2]]
    angular=[]
    for a,b,dt in zip(qsensor[:-1],qsensor[1:],np.diff(d['time_s'])):
        if np.dot(a,b)<0:b=-b
        delta=multiply(a*np.array([1,-1,-1,-1]),b)
        vel=np.empty(3);mujoco.mju_quat2Vel(vel,delta,float(dt));angular.append(vel)
    gyro=(d['imu_angular_velocity'][:-1]+d['imu_angular_velocity'][1:])/2
    corr=[float(np.corrcoef(np.asarray(angular)[:,i],gyro[:,i])[0,1]) for i in range(3)]
    np.savez_compressed(OUT/'mapped_kinematic.npz',time_s=d['time_s'],qpos=np.array(positions),
                        joint_position=q,joint_velocity=dq,imu_aligned_quat_wxyz=quat,
                        wheel_centers_relative_world=np.asarray(feet),self_penetrating_contacts=np.asarray(contacts))
    return {'samples':len(q),'actuator_order':names,'leg_limit_violations':int(np.sum(excess>.0001)),
            'max_leg_limit_excess_rad':float(max(0,excess.max())),
            'frames_with_penetration_over_2mm':int(np.count_nonzero(contacts)),
            'imu_gyro_correlation_xyz':corr,'imu_gyro_difference_rad_s':percentile(np.linalg.norm(np.asarray(angular)-gyro,axis=1)),
            'root_position':'fixed at (0,0,0.85)m for display; not reconstructed world trajectory',
            'orientation':'IMU orientation with initial yaw removed; physical IMU-to-base extrinsic remains unverified',
            'wheel_phase':'subtract initial cumulative angle for visual rendering only'}

def physics(name,d,q,dq,quat,seconds,args,fixed=False,hold=False):
    model=make_model(fixed);data=mujoco.MjData(model)
    joints=model.actuator_trnid[:,0];qa=model.jnt_qposadr[joints];va=model.jnt_dofadr[joints]
    data.qpos[qa]=q[0];data.qvel[va]=0
    base=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'base_link')
    if not fixed:
        data.qpos[:3]=[0,0,0];data.qpos[3:7]=quat[0]
        mujoco.mj_forward(model,data)
        wheels=[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,n) for n in ['fl_wheel','fr_wheel','hl_wheel','hr_wheel']]
        data.qpos[2]=.081-float(data.xpos[wheels,2].min())+.002
    mujoco.mj_forward(model,data)
    kp=np.full(16,args.kp,dtype=float);kd=np.full(16,args.kd,dtype=float);kp[WHEELS]=0;kd[WHEELS]=args.wheel_kd
    assert np.all(kd[WHEELS]==args.wheel_kd), 'Wheel damping must retain fractional gains'
    renderer,camera,proc=movie(model,name)
    log=[];errors=[];saturation=[];ground_contacts=[];actual=[];desired=[];first_body_contact=None;first_fall=None
    contact_bodies=Counter();wheel_errors=[];peak_torque=np.zeros(16)
    floor=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,'floor')
    wheel_ids={mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,n) for n in ['fl_wheel','fr_wheel','hl_wheel','hr_wheel']}
    try:
        for step in range(round(seconds/model.opt.timestep)):
            t=step*model.opt.timestep
            if hold:target=q[0];velocity=np.zeros(16)
            else:
                a=min(int(t*50),len(q)-2);blend=np.clip((t-d['time_s'][a])/.02,0,1)
                target=q[a]*(1-blend)+q[a+1]*blend;velocity=dq[a]*(1-blend)+dq[a+1]*blend
            torque=kp*(target-data.qpos[qa])+kd*(velocity-data.qvel[va])
            data.ctrl[:]=np.clip(torque,model.actuator_ctrlrange[:,0],model.actuator_ctrlrange[:,1])
            peak_torque=np.maximum(peak_torque,np.abs(data.ctrl))
            mujoco.mj_step(model,data)
            if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():raise RuntimeError('Simulation diverged')
            saturation.append(np.abs(torque)>model.actuator_ctrlrange[:,1])
            if step%20==0:
                err=data.qpos[qa]-target;errors.append(err)
                body_floor=0
                for c in data.contact[:data.ncon]:
                    if floor in (c.geom1,c.geom2):
                        other=c.geom2 if c.geom1==floor else c.geom1
                        body=int(model.geom_bodyid[other])
                        if body not in wheel_ids:
                            body_floor+=1
                            contact_bodies[mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_BODY,body)]+=1
                if body_floor and first_body_contact is None:first_body_contact=t
                rotation=np.empty(9);mujoco.mju_quat2Mat(rotation,data.xquat[base]);tilt=float(np.degrees(np.arccos(np.clip(rotation[8],-1,1))))
                if (tilt>60 or data.xpos[base,2]<.15) and first_fall is None:first_fall=t
                log.append([t,*data.xpos[base],tilt,body_floor]);ground_contacts.append(body_floor)
                actual.append(data.qpos[qa].copy());desired.append(target.copy())
                wheel_errors.append(data.qvel[va[WHEELS]]-velocity[WHEELS])
            if step%40==0:frame(renderer,camera,proc,model,data)
    finally:close_movie(renderer,proc)
    errors=np.asarray(errors);sat=np.asarray(saturation);log=np.asarray(log)
    np.savez_compressed(OUT/(name+'.npz'),log=log,joint_error=errors,joint_actual=actual,joint_target=desired,wheel_velocity_error=np.asarray(wheel_errors))
    return {'duration_s':seconds,'fixed_base':fixed,'hold_first_pose':hold,'kp_leg':args.kp,'kd_leg':args.kd,'kd_wheel':args.wheel_kd,
            'leg_rmse_rad':float(np.sqrt(np.mean(errors[:,LEGS]**2))),
            'leg_abs_error_rad':percentile(abs(errors[:,LEGS]).ravel()),
            'saturation_fraction_per_actuator':sat.mean(axis=0).tolist(),
            'peak_applied_torque_nm_per_actuator':peak_torque.tolist(),
            'wheel_speed_rmse_rad_s':float(np.sqrt(np.mean(np.asarray(wheel_errors)**2))),
            'nonwheel_floor_contact_bodies':dict(contact_bodies),
            'root_height_min_m':float(log[:,3].min()),'tilt_max_deg':float(log[:,4].max()),
            'first_nonwheel_floor_contact_s':first_body_contact,'first_fall_threshold_s':first_fall,
            'finite_entire_run':True,'gravity_m_s2':model.opt.gravity.tolist(),
            'controller':'SDK gains, measured joint position/velocity as PD targets, no expert action or torque feedforward',
            'scene':'fixed suspended robot' if fixed else 'flat ground; not reconstructed stairs'}

def selfcheck():
    identity=np.array([1.,0,0,0]);a=np.array([np.cos(.2),0,0,np.sin(.2)])
    assert np.allclose(multiply(a,a*np.array([1,-1,-1,-1])),identity)
    q=root_quaternions(np.array([[0,0,np.sin(.2),np.cos(.2)]]))
    assert np.allclose(q[0],identity)

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--kp',type=float,default=80);parser.add_argument('--kd',type=float,default=2)
    parser.add_argument('--wheel-kd',type=float,default=.6)
    args=parser.parse_args()
    if not all(np.isfinite(v) and v>0 for v in (args.kp,args.kd,args.wheel_kd)):parser.error('Gains must be positive finite values')
    OUT.mkdir(exist_ok=True)
    selfcheck();d,q,dq,quat,direction,offset=mapped_reference()
    report={'session':SID,'joint_dir':direction.tolist(),'offset_rad':offset.tolist()}
    print('Kinematic replay',flush=True);report['kinematic']=kinematic(d,q,dq,quat,direction,offset)
    jobs=[('fixed_base_tracking',12,True,False),('flat_stance_hold',3,False,True),('flat_motion_tracking',12,False,False)]
    for name,seconds,fixed,hold in jobs:
        print(name,flush=True);report[name]=physics(name,d,q,dq,quat,seconds,args,fixed,hold)
        (OUT/'validation.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2),flush=True)
