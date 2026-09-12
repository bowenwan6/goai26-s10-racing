"""Compare bounded distance/time offsets without changing measured reference or height."""
import json
import itertools
import numpy as np
import mujoco
import match as m

OUT=m.OUT/'distance_search'
OUT.mkdir(exist_ok=True)
ref=dict(np.load(m.OUT/'expert_reference.npz'));base=dict(np.load(m.OUT/'tracking.npz'))
alignment=json.loads((m.OUT/'alignment.json').read_text())
normal=np.r_[alignment['edge_normal_xy'],0];edge=alignment['edge_horizontal_distance_m']
model=mujoco.MjModel.from_xml_path(str(m.OUT/'matched_scene.xml'))
ledge=model.geom('matched_ledge').id;original_pos=model.geom_pos[ledge].copy()
rows=[r for r in json.loads((m.OUT/'fits.json').read_text()) if r['accepted_track'] and r['time_s']>=9]
lidar_t=np.array([r['time_s'] for r in rows]);lidar_d=np.array([r['edge_horizontal_distance_m'] for r in rows])


def run(offset,delay,save=None):
    model.geom_pos[ledge]=original_pos+offset*normal
    result,arrays=m.track(model,base['initial_qpos'],base['initial_qvel'],ref['time_s'],
        ref['joint_position'],ref['joint_velocity'],normal,edge+offset,.38,delay=delay)
    log=arrays['log'];states=arrays['qpos']
    errors=np.interp(lidar_t,log[:,0],edge+offset-states[:,:3]@normal)-lidar_d
    idx=np.clip(np.round((log[:,0]-9)/.005).astype(int),0,len(ref['time_s'])-1)
    angular=np.degrees(2*np.arccos(np.clip(abs(np.sum(states[:,3:7]*ref['base_quaternion_wxyz'][idx],axis=1)),0,1)))
    onset=(log[:,0]>=12)&(log[:,0]<=14)
    result.update(offset_m=float(offset),delay_s=float(delay),initial_base_to_edge_m=float(edge+offset),
        approach_distance_rmse_m=float(np.sqrt(np.mean(errors**2))),
        ascent_orientation_rmse_deg=float(np.sqrt(np.mean(angular[onset]**2))))
    result['score']=result['ascent_orientation_rmse_deg']+200*result['approach_distance_rmse_m']
    if save:
        np.savez_compressed(OUT/(save+'.npz'),**arrays)
        m.review.dump(OUT/(save+'.json'),result)
    return result


def kinematics():
    data=mujoco.MjData(model);qa=model.jnt_qposadr[model.actuator_trnid[:,0]]
    names=['base_link']+[leg+'_'+joint for leg in ['fl','fr','hl','hr'] for joint in ['hipx','hipy','knee','wheel']]
    ids=[model.body(n).id for n in names];positions=[]
    for q,quat in zip(ref['joint_position'],ref['base_quaternion_wxyz']):
        data.qpos[:3]=0;data.qpos[3:7]=quat;data.qpos[qa]=q;mujoco.mj_forward(model,data)
        positions.append(data.xpos[ids].copy())
    positions=np.array(positions)
    np.savez_compressed(OUT/'recorded_kinematics.npz',time_s=ref['time_s'],body_names=names,positions=positions)
    w,x,y,z=ref['base_quaternion_wxyz'].T
    pitch=np.degrees(np.arcsin(np.clip(2*(w*y-z*x),-1,1)))
    events={'pitch_below_minus_10_source_s':float(ref['time_s'][np.flatnonzero(pitch<-10)[0]]),
            'peak_nose_up_source_s':float(ref['time_s'][np.argmin(pitch)]),'peak_pitch_deg':float(pitch.min()),
            'pitch_returns_above_minus_10_source_s':float(ref['time_s'][np.flatnonzero((np.arange(len(pitch))>np.argmin(pitch))&(pitch>-10))[0]])}
    wheel=positions[:,[4,8,12,16]]
    # Relative wheel heights mark pose changes, not unmeasured world contacts.
    front_lift=wheel[:,:2,2].mean(axis=1)-wheel[:,2:,2].mean(axis=1)
    events['front_minus_rear_wheel_height_peak_source_s']=float(ref['time_s'][np.argmax(front_lift)])
    events['front_minus_rear_wheel_height_peak_m']=float(front_lift.max())
    m.review.dump(OUT/'recorded_events.json',events)


if __name__=='__main__':
    # One regression check: extracted shared simulator must reproduce the prior baseline.
    zero=run(0,0,'baseline')
    assert np.allclose(np.load(OUT/'baseline.npz')['qpos'],base['qpos'],atol=1e-10)
    kinematics();results=[]
    for i,(offset,delay) in enumerate(itertools.product(np.linspace(-.15,.15,11),np.linspace(-.3,.3,7))):
        result=run(offset,delay);results.append(result)
        if i%11==0:print(i+1,'/77',result['reached_top'],round(result['score'],2),flush=True)
    m.review.dump(OUT/'search.json',results)
    best=min(results,key=lambda r:r['score'])
    success=[r for r in results if r['reached_top']]
    chosen=min(success,key=lambda r:r['score']) if success else best
    run(chosen['offset_m'],chosen['delay_s'],'selected')
    distance_only=[r for r in results if abs(r['delay_s'])<1e-6]
    summary=dict(trials=len(results),successes=len(success),baseline=zero,selected=chosen,
        best_distance_only=min(distance_only,key=lambda r:r['score']),
        successful_parameters=[{k:r[k] for k in ['offset_m','delay_s','approach_distance_rmse_m']} for r in success],
        ranges='distance -15..+15 cm in 3cm steps; action delay -0.3..+0.3s in 0.1s steps; +delay means later action',
        selection='Success first, then ascent orientation RMSE + 200*approach distance RMSE; result does not validate calibration')
    m.review.dump(OUT/'summary.json',summary);print(json.dumps(summary),flush=True)
    fine=[run(float(offset),0) for offset in np.arange(-.075,-.024,.005)]
    m.review.dump(OUT/'distance_fine.json',fine)
    closest=max((r for r in fine if r['reached_top']),key=lambda r:r['offset_m'])
    run(closest['offset_m'],0,'closest_success')
