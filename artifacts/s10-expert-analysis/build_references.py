"""Export measured-state reference candidates and review plots.

Run with python -s to use the existing compatible NumPy/SciPy/Matplotlib set.
No extrapolation or interpolation through large gaps is allowed as valid data.
"""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'references';OUT.mkdir(exist_ok=True)
FIG=ROOT/'figures';FIG.mkdir(exist_ok=True)
WHEEL=np.array([3,7,11,15])
LEG=np.array([i for i in range(16) if i not in WHEEL])

def sample(t, values, query, max_gap):
    """Return linear samples and a mask for in-range, adequately sampled data."""
    assert len(t)>1 and np.all(np.diff(t)>0)
    assert values.shape[0]==len(t) and np.all(np.isfinite(values))
    index=np.searchsorted(t,query,side='right')-1
    index=np.clip(index,0,len(t)-2)
    valid=(query>=t[0])&(query<=t[-1])&((t[index+1]-t[index])<=max_gap)
    exact=np.isin(query,t)
    valid|=exact
    result=np.column_stack([np.interp(query,t,values[:,i]) for i in range(values.shape[1])])
    return result,valid

def test_sample():
    t=np.array([0.,.02,1.]);v=np.column_stack([t,t*2])
    got,valid=sample(t,v,np.array([-.1,0.,.01,.5,1.,1.1]),.05)
    assert valid.tolist()==[False,True,True,False,True,False]
    assert np.allclose(got[2],[.01,.02])
    q=Rotation.from_euler('z',[0,90],degrees=True)
    assert np.allclose(Slerp([0,1],q)([.5]).as_euler('xyz',degrees=True)[0],[0,0,45])

def export_reference(sid):
    d=np.load(ROOT/'decoded'/(sid+'.npz'))
    prefixes=['JOINTS_DATA','IMU','MOTION_INFO']
    origin=min(int(d[p+'_source_ns'][0]) for p in prefixes)
    time={p:(d[p+'_source_ns']-origin)/1e9 for p in prefixes}
    start=max(t[0] for t in time.values());end=min(t[-1] for t in time.values())
    # The long posture recording changes from 1 Hz to 50 Hz; retain only its dense tail.
    if sid.endswith('0250'):
        jt=time['JOINTS_DATA'];start=max(start,float(jt[np.flatnonzero(np.diff(jt)>.05)[-1]+1]))
    query=start+np.arange(int(np.floor((end-start)*50))+1)/50
    q,valid_q=sample(time['JOINTS_DATA'],d['JOINTS_DATA_values'],query,.05)
    imu,valid_i=sample(time['IMU'],d['IMU_values'],query,.02)
    imu[:,:4]=Slerp(time['IMU'],Rotation.from_quat(d['IMU_values'][:,:4]))(query).as_quat()
    motion,valid_m=sample(time['MOTION_INFO'],d['MOTION_INFO_values'][:,:4],query,.1)
    mi=np.searchsorted(time['MOTION_INFO'],query,side='right')-1
    codes=d['MOTION_INFO_values'][mi,4:6].astype(np.int32)
    valid=valid_q&valid_i&valid_m&(codes[:,0]==17)
    result={'time_s':query-query[0],'source_time_ns':origin+np.rint(query*1e9).astype(np.int64),
            'source_elapsed_s':query,'joint_position':q[:,:16],'joint_velocity':q[:,16:32],
            'imu_orientation_xyzw':imu[:,:4],'imu_angular_velocity':imu[:,4:7],
            'imu_linear_acceleration':imu[:,7:10],'reported_motion_vx_vy_wz_height':motion,
            'state_code':codes[:,0],'gait_code':codes[:,1],'valid_reference':valid,
            'vendor_channel_ids':np.arange(16)}
    assert len(query)>1 and all(len(v)==len(query) for k,v in result.items() if k!='vendor_channel_ids')
    assert all(np.all(np.isfinite(v)) for v in result.values())
    assert np.allclose(np.linalg.norm(result['imu_orientation_xyzw'],axis=1),1)
    np.savez_compressed(OUT/(sid+'_50hz.npz'),**result)
    columns=['time_s']+[f'q_vendor_{i}' for i in range(16)]+[f'dq_vendor_{i}' for i in range(16)]+['imu_qx','imu_qy','imu_qz','imu_qw','state','gait','valid_reference']
    csv=np.column_stack([result['time_s'],q[:,:32],imu[:,:4],codes,valid.astype(int)])
    np.savetxt(OUT/(sid+'_50hz.csv'),csv,delimiter=',',header=','.join(columns),comments='',fmt='%.9g')
    meta={'id':sid,'rate_hz':50,'samples':len(query),'duration_s':float(query[-1]-query[0]),
          'source_elapsed_start_s':float(query[0]),'valid_samples':int(valid.sum()),'invalid_samples':int((~valid).sum()),
          'status':'reference candidate; not validated in simulation','state_reference_not_teacher_action':True,
          'world_position_available':False,'joint_order':'numeric vendor channels 0..15; no simulator remapping performed',
          'units':'joint position/velocity preserved from vendor; rad and rad/s consistent with guide and derivative check; anatomical signs/zeros need mapping',
          'orientation':'IMU sensor orientation; not transformed to robot base or map frame',
          'synchronization':'common source timestamps; hardware synchronization and fixed sensor delay not calibrated',
          'wheel_handling':'channels 3/7/11/15 are wheel candidates per guide; preserve cumulative angle, use speed after channel mapping',
          'gaps':'valid_reference requires joint brackets <=50ms, IMU <=20ms, motion <=100ms, and state=17; success is not implied'}
    (OUT/(sid+'_50hz.json')).write_text(json.dumps(meta,ensure_ascii=False,indent=2),encoding='utf-8')
    return meta,result

def plots(references):
    plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
    sid='gait_20260906_141606_4ef6a913c02c';d=np.load(ROOT/'decoded'/(sid+'.npz'))
    origin=int(d['SLAM_ODOM_source_ns'][0]);t=(d['SLAM_ODOM_source_ns']-origin)/1e9;v=d['SLAM_ODOM_values'];xyz=v[:,:3]-v[0,:3]
    speed=np.linalg.norm(np.diff(v[:,:3],axis=0),axis=1)/np.diff(t)
    cols=['time_s','source_time_ns','map_x','map_y','map_z','qx','qy','qz','qw']
    # Keep timestamp integers exact in the NPZ; CSV contains seconds and raw pose only.
    np.savez_compressed(OUT/'ledge_odometry_native.npz',time_s=t,source_time_ns=d['SLAM_ODOM_source_ns'],map_position=v[:,:3],orientation_xyzw=v[:,3:7])
    np.savetxt(OUT/'ledge_odometry_native.csv',np.column_stack([t,v[:,:7]]),delimiter=',',header='time_s,map_x,map_y,map_z,qx,qy,qz,qw',comments='',fmt='%.12g')
    fig,ax=plt.subplots(2,2,figsize=(12,8),layout='constrained')
    dots=ax[0,0].scatter(xyz[:,0],xyz[:,1],c=t,s=7,cmap='viridis');ax[0,0].plot(0,0,'ko',label='Start');ax[0,0].plot(xyz[-1,0],xyz[-1,1],'r*',markersize=10,label='End')
    ax[0,0].set(xlabel='Map X relative to start (m)',ylabel='Map Y relative to start (m)',title='Recorded SLAM trajectory (not ground truth)');ax[0,0].axis('equal');ax[0,0].legend();fig.colorbar(dots,ax=ax[0,0],label='Time (s)')
    ax[0,1].plot(t,xyz[:,2]);ax[0,1].set(xlabel='Time (s)',ylabel='Relative map Z (m)',title='Two movement periods; long pause between')
    ax[1,0].plot(t[1:],speed,label='SLAM position difference / dt')
    mt=(d['MOTION_INFO_source_ns']-origin)/1e9;mv=d['MOTION_INFO_values']
    ax[1,0].plot(mt,np.linalg.norm(mv[:,:2],axis=1),label='MOTION_INFO planar speed');ax[1,0].set(xlabel='Time (s)',ylabel='Speed (m/s)',title='Reported speed becomes zero in ledge gait');ax[1,0].legend()
    jt=(d['JOINTS_DATA_source_ns']-origin)/1e9;jv=d['JOINTS_DATA_values']
    ax[1,1].plot(jt,jv[:,2],'.-',label='Vendor channel 2');ax[1,1].plot(jt,jv[:,6],'.-',label='Vendor channel 6');ax[1,1].set(xlabel='Time (s)',ylabel='Joint position (vendor rad)',title='Only 1 joint sample/s: fast motion is unresolved');ax[1,1].legend()
    fig.suptitle('120 s ledge recording | 1,201 SLAM poses, 120 joint samples',fontsize=15)
    fig.savefig(FIG/'ledge_trajectory.png',dpi=160);plt.close(fig)
    sid='gait_20260906_134324_93fa2317c970';r=references[sid];t=r['time_s']
    fig,ax=plt.subplots(3,1,figsize=(12,8),sharex=True,layout='constrained')
    for i in (1,2,5,6):ax[0].plot(t,r['joint_position'][:,i],label=f'Vendor {i}',linewidth=1)
    ax[0].set(ylabel='Joint angle (vendor rad)',title='50 Hz measured joint-state reference');ax[0].legend(ncol=4)
    rpy=Rotation.from_quat(r['imu_orientation_xyzw']).as_euler('xyz',degrees=True)
    ax[1].plot(t,rpy[:,:2]);ax[1].legend(['IMU roll','IMU pitch']);ax[1].set(ylabel='Sensor attitude (deg)')
    for i in WHEEL:ax[2].plot(t,r['joint_velocity'][:,i],label=f'Vendor {i}',linewidth=1)
    ax[2].set(xlabel='Reference time (s)',ylabel='Wheel-candidate speed (rad/s)');ax[2].legend(ncol=4)
    fig.suptitle('32.6 s stairs recording | reference candidate; outcome unlabelled',fontsize=15)
    fig.savefig(FIG/'stairs_reference.png',dpi=160);plt.close(fig)
    sid='gait_20260906_153020_96c782400250';d=np.load(ROOT/'decoded'/(sid+'.npz'));o=int(d['JOINTS_DATA_source_ns'][0]);t=(d['JOINTS_DATA_source_ns']-o)/1e9
    fig,ax=plt.subplots(2,1,figsize=(12,6),sharex=True,layout='constrained')
    ax[0].plot(t[1:],np.diff(t),'.',markersize=2);ax[0].set(ylabel='Joint sampling interval (s)',title='Sampling changes near 66.66 s: average 31.85 Hz hides 1 Hz section')
    mt=(d['MOTION_INFO_source_ns']-o)/1e9;ax[1].step(mt,d['MOTION_INFO_values'][:,4],where='post');ax[1].set(xlabel='Source elapsed time (s)',ylabel='Reported control state',yticks=[0,1,2,4,17])
    fig.savefig(FIG/'posture_sampling.png',dpi=160);plt.close(fig)

if __name__=='__main__':
    test_sample()
    ids=['gait_20260906_134245_67a76c09d415','gait_20260906_134324_93fa2317c970','gait_20260906_143150_2620f4f1e142','gait_20260906_153020_96c782400250']
    results={};metadata=[]
    for sid in ids:
        meta,result=export_reference(sid);metadata.append(meta);results[sid]=result
    plots(results)
    (OUT/'index.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(metadata,ensure_ascii=False,indent=2))
