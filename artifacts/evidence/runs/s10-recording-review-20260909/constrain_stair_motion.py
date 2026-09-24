"""Project uncertain root translation onto nonpenetration constraints; joints stay measured.
Run with .venv-win/Scripts/python.exe. The corrections are exported separately, never truth labels.
"""
import json
import sys
import numpy as np
import mujoco
from render_stair_motion import OUT, scene

def project(normals,limits,preferred):
    x=preferred.copy();duals=np.zeros((len(limits),3))
    for _ in range(60):
        old=x.copy()
        for i,(n,b) in enumerate(zip(normals,limits)):
            trial=x+duals[i];x=trial+max(0.,b-n@trial)*n/(n@n);duals[i]=trial-x
        if np.linalg.norm(x-old)<1e-7:break
    return x

def selfcheck():
    x=project(np.array([[1.,0,0],[0.,0,1],[1.,0,1]])/np.array([[1],[1],[2**.5]]),np.array([.03,.04,.07/2**.5]),np.zeros(3))
    assert np.allclose(x,[.03,0,.04],atol=1e-6)

def main():
    selfcheck();a=np.load(OUT/'matched_motion.npz');g=json.loads((OUT/'scene_geometry.json').read_text());model=scene(g);md=mujoco.MjData(model)
    qa=model.jnt_qposadr[model.actuator_trnid[:,0]];corrected=[];offsets=[];worst=[];previous=np.zeros(3)
    preferred=np.zeros_like(a['root_position_m'])
    if '--smooth' in sys.argv:
        previous_match=np.load(OUT/'contact_matched_motion.npz')['root_contact_correction_m']
        kernel=np.hanning(41);kernel/=kernel.sum()
        smooth=lambda v:np.convolve(np.pad(v,20,mode='edge'),kernel,mode='valid')
        for k in range(3):preferred[:,k]=smooth(a['root_position_m'][:,k])-a['root_position_m'][:,k]+smooth(previous_match[:,k])
        # Anticipate required vertical clearance over 0.2s instead of a one-frame root jump.
        peak=np.max(np.lib.stride_tricks.sliding_window_view(np.pad(previous_match[:,2],20,mode='edge'),41),axis=1)
        preferred[:,2]=smooth(a['root_position_m'][:,2])-a['root_position_m'][:,2]+smooth(peak)
    for i,t in enumerate(a['time_s']):
        origin=a['root_position_m'][i];md.qpos[3:7]=a['root_quaternion_wxyz'][i];md.qpos[qa]=a['joint_position_rad'][i]
        prior=preferred[i]*.8+previous*.2 if '--smooth' in sys.argv else previous*.7
        delta=prior.copy()
        for iteration in range(12):
            md.qpos[:3]=origin+delta;mujoco.mj_forward(model,md);normals=[];limits=[];depth=0.
            for c in md.contact[:md.ncon]:
                b1,b2=model.geom_bodyid[[c.geom1,c.geom2]]
                if 0 not in (b1,b2) or b1+b2==0:continue
                depth=min(depth,float(c.dist))
                if c.dist<.003:
                    n=c.frame[:3].copy()*(1 if b1==0 else -1)
                    normals.append(n);limits.append(float(n@delta-c.dist+.001))
            if not normals:break
            update=project(np.asarray(normals),np.asarray(limits),prior)
            if np.linalg.norm(update-delta)<1e-6 and depth>-.002:delta=update;break
            delta=update
        md.qpos[:3]=origin+delta;mujoco.mj_forward(model,md)
        distances=[float(c.dist) for c in md.contact[:md.ncon] if 0 in model.geom_bodyid[[c.geom1,c.geom2]] and np.sum(model.geom_bodyid[[c.geom1,c.geom2]])>0]
        worst.append(min(distances+[0.]));corrected.append(origin+delta);offsets.append(delta);previous=delta
        if i%1000==0:print(i,'root correction',delta.round(4).tolist(),'penetration',worst[-1],flush=True)
    offsets=np.array(offsets);norm=np.linalg.norm(offsets,axis=1)
    np.savez_compressed(OUT/'contact_matched_motion.npz',**{k:a[k] for k in a.files if k!='root_position_m'},
                        root_position_m=corrected,lidar_root_position_m=a['root_position_m'],root_contact_correction_m=offsets)
    result=dict(samples=len(norm),max_correction_m=float(norm.max()),median_correction_m=float(np.median(norm)),
                p95_correction_m=float(np.percentile(norm,95)),maximum_remaining_penetration_m=float(-min(worst)),
                correction_over_12cm=int(np.sum(norm>.12)),remaining_penetration_over_2mm=int(np.sum(np.array(worst)<-.002)),
                meaning='Root translation inferred from lidar and nonpenetration; joint measurements and orientation unchanged; not measured ground truth')
    (OUT/'contact_alignment.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2),flush=True)

if __name__=='__main__':main()
