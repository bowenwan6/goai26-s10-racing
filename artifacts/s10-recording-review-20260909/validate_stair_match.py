"""Check that contact-constrained root motion still agrees with native lidar scans."""
import json
import numpy as np
from reconstruct_stairs import OUT, raw_frames
from build_stair_scene import distance

def main():
    times,frames=raw_frames();poses=np.load(OUT/'refined_trajectory.npz')['poses'];a=np.load(OUT/'contact_matched_motion.npz')
    g=json.loads((OUT/'scene_geometry.json').read_text());v=[g[k] for k in ('edge_m','floor_z_m','height_m','depth_m','lateral_slope','curvature_per_m')]
    before=[];after=[];rows=[]
    for i,(t,p,pose) in enumerate(zip(times,frames,poses)):
        if i%2==0:continue
        p=p[(np.linalg.norm(p,axis=1)<2.7)&(abs(p[:,1])<1.3)]
        p=p@pose[:3,:3].T+pose[:3,3]
        k=np.clip(np.ceil((p[:,0]-.49-.04*p[:,1]**2)/.57),0,11)
        p=p[(p[:,0]>.15)&(p[:,0]<7.5)&(abs(p[:,1])<2)&(abs(p[:,2]-(-.45+k*.16))<.20)]
        if not len(p):continue
        delta=np.array([np.interp(t,a['time_s'],a['root_contact_correction_m'][:,k]) for k in range(3)])
        e0=distance(p,v);e1=distance(p+delta,v);before.extend(e0);after.extend(e1)
        rows.append(dict(t_s=float(t),points=len(p),median_before_m=float(np.median(e0)),median_after_m=float(np.median(e1)),correction_m=float(np.linalg.norm(delta))))
    gap=np.diff(a['root_position_m'],axis=0);dt=np.diff(a['time_s'])
    native=np.load(OUT/'matched_motion.npz')
    assert np.array_equal(a['joint_position_rad'],native['joint_position_rad'])
    assert np.array_equal(a['joint_velocity_rad_s'],native['joint_velocity_rad_s'])
    assert np.array_equal(a['root_quaternion_wxyz'],native['root_quaternion_wxyz'])
    assert np.isfinite(a['root_position_m']).all()
    quality=json.loads((OUT/'contact_alignment.json').read_text())
    assert quality['remaining_penetration_over_2mm']==0
    result=dict(held_out_scans=len(rows),held_out_points=len(after),
                median_before_m=float(np.median(before)),median_after_m=float(np.median(after)),
                p90_before_m=float(np.percentile(before,90)),p90_after_m=float(np.percentile(after,90)),
                max_native_position_increment_m=float(np.linalg.norm(gap,axis=1).max()),
                max_native_speed_m_s=float(np.max(np.linalg.norm(gap,axis=1)/dt)),
                measured_joints_unchanged=True,pre_contact_orientation_unchanged=True,frames=rows)
    (OUT/'final_validation.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='frames'},indent=2))
    assert result['median_after_m']<.04 and result['p90_after_m']<.10

if __name__=='__main__':main()
