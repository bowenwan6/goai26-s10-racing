"""Post-replay diagnostics only; never feeds geometry, labels, or future data into observations."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
from replay import (
    DEFAULT_REPO,
    HERE,
    NS,
    REGIONS,
    Kinematics,
    dump,
    metrics,
    plotting,
    stats,
)
from scipy.spatial.transform import Rotation


def runs(t,mask):
    start=np.flatnonzero(mask & ~np.r_[False,mask[:-1]])
    stop=np.flatnonzero(mask & ~np.r_[mask[1:],False])
    return [[float(t[x]),float(t[y])] for x,y in zip(start,stop)]


def analyze(out,repo):
    a=dict(np.load(out/'observations.npz'));f=np.load(out/'single_frames.npz');kin=Kinematics(repo)
    metrics(a,out)
    anchor=int(a['anchor_ns']);t=(a['time_ns']-anchor)/NS
    with (out/'frames.csv').open(encoding='utf-8') as stream:rows=list(csv.DictReader(stream))
    # Integers are parsed as integers: converting epoch ns through float would lose information.
    for r in rows:
        for k in ('frame','sensor','source_ns','receive_ns','arrival_ns','imu_source_ns','imu_receive_ns','joint_source_ns','joint_receive_ns','segment'):
            r[k]=int(r[k])
        for k in ('source_s','receive_s','p90','matched','condition','px','py','pz','revisit_p90','conflicts'):
            r[k]=float(r.get(k) or 'nan')
    valid=a['V'];now=a['time_ns'][:,None]
    assert np.all(a['measurement_ns'][valid]<=np.broadcast_to(now,valid.shape)[valid])
    assert np.all(a['arrival_ns'][valid]<=np.broadcast_to(now,valid.shape)[valid])
    assert np.all(a['H'][~valid]==0) and np.isinf(a['age'][~valid]).all()
    assert np.allclose(a['age'][valid],((now-a['measurement_ns'])/NS)[valid],atol=1e-6)
    assert np.all(np.diff(a['time_ns'])==20_000_000)
    assert np.all(a['imu_arrival_ns']<=a['time_ns']) and np.all(a['joint_arrival_ns']<=a['time_ns'])
    for r in rows:
        assert r['imu_receive_ns']<=r['arrival_ns'] and r['joint_receive_ns']<=r['arrival_ns']
        if r['imu_source_ns']:assert r['imu_source_ns']<=r['source_ns']
        if r['joint_source_ns']:assert r['joint_source_ns']<=r['source_ns']
    wheel_body=[];wheel_gravity=[]
    for state,pose in zip(a['joint_state'],a['pose']):
        if not np.isfinite(state).all():
            wheel_body.append(np.full((4,3),np.nan));wheel_gravity.append(np.full((4,3),np.nan));continue
        positions,_=kin.forward(state)
        w=np.array([positions[k+'_wheel'] for k in ('fl','fr','hl','hr')]);wheel_body.append(w)
        wheel_gravity.append(Rotation.from_quat(pose[3:]).apply(w))
    wb=np.array(wheel_body);wg=np.array(wheel_gravity)
    np.savez_compressed(out/'kinematic_diagnostics.npz',time_ns=a['time_ns'],joint_source_ns=a['joint_source_ns'],
                        wheel_body=wb,wheel_gravity=wg)
    euler=Rotation.from_quat(a['pose'][:,3:]).as_euler('xyz',degrees=True)
    short='151135' if '151135' in str(a['recording_id']) else '150146'
    windows=[(9.,15.5)] if short=='151135' else [(25.,30.5),(42.5,47.5)]
    phases=[];phase_events=[]
    for start,end in windows:
        mask=(t>=start)&(t<=end);inds=np.flatnonzero(mask)
        if not len(inds):continue
        baseline=(t>=start)&(t<start+.5)
        # FK lift relative to the body is a motion cue, not contact truth.
        front=wb[:,:2,2].max(axis=1);rear=wb[:,2:,2].max(axis=1)
        base_f=float(np.nanmedian(front[baseline]));base_r=float(np.nanmedian(rear[baseline]))
        # Four legs retract together during preparation: not a front-wheel lift.
        differential=front-rear
        base_diff=float(np.nanmedian(differential[baseline]))
        lift=runs(t,mask&(front>base_f+.04)&(differential>base_diff+.05))
        rear_lift=runs(t,mask&(rear>base_r+.04)&(differential<base_diff-.04))
        pitch=euler[:,1]
        peak_i=inds[np.argmax(abs(pitch[inds]))]
        start_i=next((i for i in inds if abs(pitch[i]-np.median(pitch[baseline]))>6),inds[0])
        after=np.flatnonzero(mask&(t>t[peak_i])&(abs(pitch)<10))
        settle=float(t[after[0]]) if len(after) else end
        lift_s=next((x for x,y in lift if y-x>=.04),float(t[start_i]))
        descending=bool(pitch[peak_i]>6)
        candidates=[('approach',start,lift_s),('front_descent' if descending else 'front_transition',lift_s,float(t[peak_i])),
                    ('rear_descent' if descending else 'rear_follow',float(t[peak_i]),settle),('level_or_later',settle,end)]
        phase_events.append(dict(window=[start,end],motion_candidate='down' if descending else 'up',
                                 front_transition_receive_s=lift_s,front_transition_source_s=float((a['joint_source_ns'][np.searchsorted(t,lift_s)]-anchor)/NS),
                                 pitch_peak_receive_s=float(t[peak_i]),pitch_peak_deg=float(pitch[peak_i]),
                                 rear_lift_intervals=rear_lift,first_pitch_under_10_after_peak_s=settle,
                                 caution='FK/pitch candidate boundaries; wheel contact and task success not independently observed'))
        for name,beg,stop in candidates:
            m=(t>=beg)&(t<stop)
            if not m.any():continue
            row=dict(window_start=start,phase=name,receive_start_s=beg,receive_end_s=stop,
                     pose_valid_fraction=float(a['pose_valid'][m].mean()))
            for region,r in REGIONS.items():
                v=valid[m][:,r];ages=a['age'][m][:,r][v]
                row[region+'_missing']=float(1-v.mean())
                row[region+'_single_missing']=float(1-a['single_V'][m][:,r].mean())
                row[region+'_age_p95']=float(np.percentile(ages,95)) if len(ages) else None
            row['front_sensor_coverage']=float(a['front_V'][m].mean())
            row['rear_sensor_coverage']=float(a['rear_V'][m].mean())
            row['exclusive_front']=float((a['front_V'][m]&~a['rear_V'][m]).mean())
            row['exclusive_rear']=float((a['rear_V'][m]&~a['front_V'][m]).mean())
            phases.append(row)
    diagnostic=[]
    for r in rows:
        key=f'surface_{r["frame"]}'
        if key not in f:continue
        scan_rotation=f[f'rot_{r["frame"]}']
        scan_yaw=Rotation.from_matrix(scan_rotation).as_euler('xyz')[2]
        yaw_matrix=Rotation.from_euler('z',scan_yaw).as_matrix()
        # Saved surfaces live in the gravity-aligned local axes. Diagnostic ROIs follow body yaw,
        # just like the policy grid; the second recording turns nearly 180 degrees before descent.
        p=f[key]@yaw_matrix;cloud=f[f'raw_{r["frame"]}']@scan_rotation.T@yaw_matrix
        near=p[(p[:,0]>.25)&(p[:,0]<1.2)&(abs(p[:,1])<.5)]
        # A near elevated patch requires 8 measured 5 cm cells and 20 cm lateral span.
        # H > -0.2 is only a body-relative visible-platform candidate; no recorded height enters.
        top=near[(near[:,2]>-.2)&(near[:,2]<.35)]
        low=near[near[:,2]<-.55]
        far=p[(p[:,0]>1.2)&(p[:,0]<2.)&(abs(p[:,1])<.5)&(p[:,2]>-.2)&(p[:,2]<.35)]
        slope=None
        if len(top)>=15 and np.ptp(top[:,0])>.25:
            beta=np.linalg.lstsq(np.c_[top[:,:2],np.ones(len(top))],top[:,2],rcond=None)[0]
            residual=top[:,2]-np.c_[top[:,:2],np.ones(len(top))]@beta
            if np.median(abs(residual))<.035:slope=float(np.degrees(np.arctan(beta[0])))
        # Near front face cluster (raw, no whole-map geometry). Excludes the known robot envelope.
        face=cloud[(cloud[:,0]>.26)&(cloud[:,0]<1.1)&(abs(cloud[:,1])<.5)&(cloud[:,2]>-.32)&(cloud[:,2]<-.08)]
        edge=None;edge_spread=None
        if len(face)>=30:
            bins=np.floor(face[:,0]/.025).astype(int);u,c=np.unique(bins,return_counts=True);b=u[np.argmax(c)]
            selected=face[abs(face[:,0]-(b+.5)*.025)<.045]
            if len(selected)>25 and np.ptp(selected[:,2])>.15:
                edge=float(np.median(selected[:,0]));edge_spread=float(np.percentile(selected[:,0],90)-np.percentile(selected[:,0],10))
        diagnostic.append(dict(frame=r['frame'],sensor=r['sensor'],source_s=r['source_s'],receive_s=r['receive_s'],
            near_top_cells=len(top),near_top_candidate=bool(len(top)>=8 and np.ptp(top[:,1])>.2),far_top_cells=len(far),
            low_surface_cells=len(low),low_surface_candidate=bool(len(low)>=8 and np.ptp(low[:,1])>.2),
            slope_deg=slope,edge_x_body=edge,edge_x_local=None if edge is None else np.cos(scan_yaw)*edge+r['px'],edge_spread=edge_spread,
            segment=r['segment'],revisit_p90=r['revisit_p90'],conflicts=r['conflicts']))
    first_near=next((r for r in diagnostic if r['sensor']==0 and r['near_top_candidate']),None)
    first_far=next((r for r in diagnostic if r['sensor']==0 and r['far_top_cells']>=8),None)
    failures=runs(t,~a['pose_valid'])
    summary=dict(recording_id=str(a['recording_id']),causal_checks='PASS',phase_events=phase_events,phases=phases,
        first_near_top_candidate=first_near,first_far_elevated_candidate=first_far,
        window_visibility=[dict(window=[b,e],
            first_near_top=next((r for r in diagnostic if r['sensor']==0 and b<=r['source_s']<=e and r['near_top_candidate']),None),
            first_lower_surface=next((r for r in diagnostic if r['sensor']==0 and b<=r['source_s']<=e and r['low_surface_candidate']),None)) for b,e in windows],
        pose_invalid_intervals_receive_s=failures,pose_valid_fraction=float(a['pose_valid'].mean()),
        overall_coverage=float(valid.mean()),rear_observed_age_s=stats(a['age'][:,REGIONS['rear_wheels']][valid[:,REGIONS['rear_wheels']]]),
        note='First visibility is algorithm/ROI dependent and uncertain by scan duration (~100 ms); far elevated surface identity unconfirmed. Phase analysis is post hoc and never enters replay.')
    # JSON null for missing scalar diagnostics, no fabricated zero accuracy.
    def finite(obj):
        if isinstance(obj,dict):return {k:finite(v) for k,v in obj.items()}
        if isinstance(obj,list):return [finite(v) for v in obj]
        if isinstance(obj,(float,np.floating)) and not np.isfinite(obj):return None
        return obj
    dump(out/'summary.json',finite(summary));dump(out/'surface_diagnostics.json',finite(diagnostic))
    with (out/'phase_metrics.csv').open('w',newline='',encoding='utf-8') as s:
        w=csv.DictWriter(s,fieldnames=list(phases[0]));w.writeheader();w.writerows(phases)
    plt=plotting();fig,ax=plt.subplots(4,1,figsize=(13,11),sharex=True)
    for i,n in enumerate(('FL','FR','HL','HR')):ax[0].plot(t,wb[:,i,2],label=n)
    ax[0].set_ylabel('wheel Z in body (m)');ax[0].legend(ncol=4)
    ax[1].plot(t,euler[:,:2]);ax[1].legend(['roll','pitch']);ax[1].set_ylabel('IMU degrees')
    front=[r for r in diagnostic if r['sensor']==0]
    ax[2].plot([r['source_s'] for r in front],[r['slope_deg'] if r['slope_deg'] is not None else np.nan for r in front],'.-',label='visible near surface tilt')
    ax[2].set_ylabel('surface tilt degrees');ax[2].legend()
    ax[3].plot([r['source_s'] for r in front],[r['edge_x_local'] if r['edge_x_local'] is not None else np.nan for r in front],'.',label='near face x in current local segment')
    ax[3].set_ylabel('edge local X (m)');ax[3].legend();ax[3].set_xlabel('time since recording start (s); cloud diagnostics use source, kinematics use receive')
    for axes in ax:axes.grid(alpha=.2)
    fig.tight_layout();fig.savefig(out/'consistency.png',dpi=140);plt.close(fig)
    print(json.dumps(finite({k:v for k,v in summary.items() if k not in ('phases','pose_invalid_intervals_receive_s')}),ensure_ascii=False,indent=2))
    return summary


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--recording',default='151135');p.add_argument('--repo',type=Path,default=DEFAULT_REPO)
    args=p.parse_args();out=next((HERE/'output').glob('gait_*'+args.recording+'*'));analyze(out,args.repo)
