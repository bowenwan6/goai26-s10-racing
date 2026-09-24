"""Batch local stair reconstruction. Run python -s batch_match_stairs.py --job NAME.
Retains original joint samples and source times. Does not claim contact accuracy.
"""
import argparse
import ast
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from scipy.ndimage import gaussian_filter1d
from reconstruct_stairs import raw_frames, voxel, target, icp, selfcheck
from fit_stairs import ROOT, load, rotation

BATCH=ROOT/'stairs_batch'
CONFIG=json.loads((ROOT/'stair_batch.json').read_text())

def export_motion(job,out,times,poses,stats):
    meta,d,_=load(job['recording_id']);anchor=meta['manifest']['started_wall_ns']
    jt=(d['JOINTS_DATA_src']-anchor)/1e9;it=(d['IMU_src']-anchor)/1e9
    keep=(jt>=max(times[0],it[0]))&(jt<=min(times[-1],it[-1]));tt=jt[keep];raw=d['JOINTS_DATA_v'][keep]
    sdk=ROOT.parents[1]/'upstream/goai_embodied_future_material/src/S10_sdk_deploy/interface/robot/simulation/mujoco_simulation_ros2.py'
    values={}
    for n in ast.walk(ast.parse(sdk.read_text(encoding='utf-8'))):
        if isinstance(n,ast.Assign) and isinstance(n.targets[0],ast.Name) and n.targets[0].id in ('JOINT_DIR','POS_OFFSET_DEG'):
            values[n.targets[0].id]=np.asarray(ast.literal_eval(n.value.args[0]))
    q=raw[:,:16]*values['JOINT_DIR']+np.deg2rad(values['POS_OFFSET_DEG']);dq=raw[:,16:32]*values['JOINT_DIR']
    r0=rotation(d['IMU_v'][0,:4]);align=Rotation.from_euler('z',-np.arctan2(r0[1,0],r0[0,0]))
    quat=(Slerp(times,Rotation.from_matrix(poses[:,:3,:3]))(tt)*align*Slerp(it,Rotation.from_quat(d['IMU_v'][:,:4]))(tt)).as_quat()[:,[3,0,1,2]]
    # Light smoothing is confined to estimated translation, never joint measurements.
    smooth=gaussian_filter1d(poses[:,:3,3],sigma=.7,axis=0,mode='nearest')
    xyz=np.column_stack([np.interp(tt,times,smooth[:,k]) for k in range(3)])
    idx=np.abs(times[:,None]-tt[None,:]).argmin(axis=0)
    median=np.array([r['median_plane_m'] for r in stats]);fraction=np.array([r['matched_fraction'] for r in stats])
    gaps=np.r_[False,np.diff(times)>.25];bad=(median>.05)|(fraction<.18)|gaps
    rollpitch=Rotation.from_quat(quat[:,[1,2,3,0]]).as_euler('xyz')[:,:2]
    usable=~bad[idx]&(np.max(abs(rollpitch),axis=1)<np.deg2rad(45))
    phase=np.full(len(tt),'unknown',dtype='U32')
    for a,b,label in job['phases']:phase[(tt>=a)&(tt<b)]=label
    usable&=phase!='interruption_context'
    weights=np.where(usable,.25,0.).astype('f4')
    np.savez_compressed(out/'reference_motion.npz',time_s=tt,joint_source_ns=d['JOINTS_DATA_src'][keep],
                        joint_position_rad=q,joint_velocity_rad_s=dq,raw_joint_position_rad=raw[:,:16],
                        root_position_m=xyz,root_quaternion_wxyz=quat,lidar_time_s=times,lidar_root_poses=poses,
                        reference_valid=usable,root_reference_weight=weights,phase_candidate=phase,
                        contact_label_valid=np.zeros(len(tt),dtype=bool))
    assert np.all(np.diff(tt)>0) and np.isfinite(q).all() and np.isfinite(xyz).all()
    summary=dict(**job,samples=len(tt),duration_s=float(tt[-1]-tt[0]),lidar_pairs=len(times),
                 reference_valid_fraction=float(np.mean(usable)),provisional_root_reference_weight=.25,
                 median_registration_residual_m=float(np.median(median)),p90_registration_residual_m=float(np.percentile(median,90)),
                 endpoint_displacement_m=poses[-1,:3,3].tolist(),minimum_source_gap_s=float(np.min(np.diff(times))),
                 maximum_source_gap_s=float(np.max(np.diff(times))),training_status='soft_reference_candidate_not_dynamics_validated',
                 contact_labels='unavailable',outcome='not_inferred_from_recording_label',root_coordinate_origin='first lidar frame body origin; recording-initial heading')
    (out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2));return summary

def run(job):
    out=BATCH/job['name'];out.mkdir(parents=True,exist_ok=True)
    times,frames=raw_frames(job['end_s'],sid=job['recording_id'],start=job['start_s'],out=out)
    path=out/'trajectory.npz'
    if path.exists():
        saved=np.load(path);assert np.array_equal(saved['time_s'],times);poses=saved['poses'];stats=json.loads((out/'registration_quality.json').read_text())
    else:
        poses=[np.eye(4)];stats=[dict(median_plane_m=0.,p90_plane_m=0.,matched_fraction=1.,pairs=len(frames[0]))]
        maps=[frames[0]];ref=target(frames[0])
        for i,p in enumerate(frames[1:],1):
            guess=poses[-1].copy()
            if i>1 and times[i]-times[i-1]<.25:
                guess[:3,3]+=(poses[-1][:3,3]-poses[-2][:3,3])*min(1.5,(times[i]-times[i-1])/(times[i-1]-times[i-2]))
            pose,q=icp(p,guess,ref);poses.append(pose);stats.append(q)
            if i%5==0:
                maps.append(p@pose[:3,:3].T+pose[:3,3]);maps=maps[-12:];ref=target(np.vstack(maps))
            if i%50==0:print(job['name'],i,round(times[i],2),pose[:3,3].round(2).tolist(),round(q['median_plane_m'],3),flush=True)
        poses=np.asarray(poses);np.savez_compressed(path,time_s=times,poses=poses)
        (out/'registration_quality.json').write_text(json.dumps(stats,indent=2))
    summary=export_motion(job,out,times,poses,stats)
    # Both odd/even near scans are retained for subsequent terrain fitting/validation.
    terrain=[[],[]]
    for i,(p,pose) in enumerate(zip(frames,poses)):
        p=p[(np.linalg.norm(p,axis=1)<3)&(p[:,2]<1)]
        terrain[i%2].append(p@pose[:3,:3].T+pose[:3,3])
    train,test=[voxel(np.vstack(p),.055) for p in terrain]
    np.savez_compressed(out/'registered_terrain.npz',train=train,test=test)
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(14,6))
    axes[0].scatter(train[:,0],train[:,1],c=train[:,2],s=.4,cmap='terrain');axes[0].plot(poses[:,0,3],poses[:,1,3],'r-');axes[0].set_aspect('equal')
    axes[0].set(xlabel='Local world X (m)',ylabel='Local world Y (m)',title=job['name'])
    for k,name in enumerate('XYZ'):axes[1].plot(times,poses[:,k,3],label=name)
    axes[1].legend();axes[1].grid(alpha=.2);axes[1].set(xlabel='Recording source time (s)',ylabel='Estimated root displacement (m)')
    fig.tight_layout();fig.savefig(out/'trajectory.png',dpi=130);plt.close(fig)
    print('DONE',json.dumps(summary),flush=True)

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--job',choices=[j['name'] for j in CONFIG['jobs']]);parser.add_argument('--selfcheck',action='store_true');a=parser.parse_args()
    if a.selfcheck:selfcheck();print('ICP selfcheck passed')
    elif a.job:run(next(j for j in CONFIG['jobs'] if j['name']==a.job))
    else:
        for job in CONFIG['jobs']:run(job)
