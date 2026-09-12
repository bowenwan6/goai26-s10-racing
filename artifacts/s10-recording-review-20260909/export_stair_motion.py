"""Export native-rate measured motion with estimated root trajectory; no policy actions."""
import ast
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from fit_stairs import ROOT, load, rotation
from reconstruct_stairs import OUT

def main():
    meta,d,_=load();anchor=meta['manifest']['started_wall_ns'];trajectory=np.load(OUT/'refined_trajectory.npz')
    ts=trajectory['time_s'];poses=trajectory['poses'];jt=(d['JOINTS_DATA_src']-anchor)/1e9
    keep=(jt>=ts[0])&(jt<=ts[-1]);tt=jt[keep];raw=d['JOINTS_DATA_v'][keep]
    sdk=ROOT.parents[1]/'upstream/goai_embodied_future_material/src/S10_sdk_deploy/interface/robot/simulation/mujoco_simulation_ros2.py'
    values={}
    for node in ast.walk(ast.parse(sdk.read_text(encoding='utf-8'))):
        if isinstance(node,ast.Assign) and isinstance(node.targets[0],ast.Name) and node.targets[0].id in ('JOINT_DIR','POS_OFFSET_DEG'):
            values[node.targets[0].id]=np.asarray(ast.literal_eval(node.value.args[0]))
    q=raw[:,:16]*values['JOINT_DIR']+np.deg2rad(values['POS_OFFSET_DEG'])
    dq=raw[:,16:32]*values['JOINT_DIR']
    it=(d['IMU_src']-anchor)/1e9;imu=Slerp(it,Rotation.from_quat(d['IMU_v'][:,:4]))(tt)
    first=rotation(d['IMU_v'][0,:4]);yaw=np.arctan2(first[1,0],first[0,0]);align=Rotation.from_euler('z',-yaw)
    correction=Slerp(ts,Rotation.from_matrix(poses[:,:3,:3]))(tt)
    quat=(correction*align*imu).as_quat()[:,[3,0,1,2]]
    # ponytail: 10Hz scan trajectory interpolated to native joint times; no claim of 200Hz measured root motion.
    xyz=np.column_stack([np.interp(tt,ts,poses[:,k,3]) for k in range(3)])
    np.savez_compressed(OUT/'matched_motion.npz',time_s=tt,root_position_m=xyz,root_quaternion_wxyz=quat,
                        joint_position_rad=q,joint_velocity_rad_s=dq,raw_joint_position_rad=raw[:,:16],
                        lidar_time_s=ts,lidar_root_poses=poses,joint_source_ns=d['JOINTS_DATA_src'][keep])
    assert np.isfinite(xyz).all() and np.all(np.diff(tt)>0) and q.shape==(len(tt),16)
    assert np.max(abs(np.linalg.norm(quat,axis=1)-1))<1e-7
    print('Exported',len(tt),'native joint samples; root estimated from',len(ts),'lidar pairs')

if __name__=='__main__':main()
