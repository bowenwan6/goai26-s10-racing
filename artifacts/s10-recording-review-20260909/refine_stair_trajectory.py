"""Anchor the first flight to stationary endpoint scans and validate moving scans."""
import json
import numpy as np
from reconstruct_stairs import OUT, raw_frames, voxel, target, icp
import matplotlib.pyplot as plt

def main():
    times,frames=raw_frames();old=np.load(OUT/'trajectory.npz')['poses']
    initial=voxel(np.vstack([p for t,p in zip(times,frames) if t<3]),.06)
    end_ids=np.flatnonzero(times>26)
    endpoint=voxel(np.vstack([frames[i]@old[i][:3,:3].T+old[i][:3,3] for i in end_ids]),.06)
    correction,q=icp(endpoint,np.eye(4),target(initial));print('Endpoint anchor correction',correction.tolist(),q,flush=True)
    endpoint=endpoint@correction[:3,:3].T+correction[:3,3]
    static=voxel(np.vstack([initial,endpoint]),.06);ref=target(static)
    poses=[];quality=[]
    for i,p in enumerate(frames):
        guess=old[i].copy()
        alpha=np.clip((times[i]-11)/15,0,1);guess[:3,3]+=alpha*correction[:3,3]
        pose,q=icp(p,guess,ref);poses.append(pose);quality.append(q)
        if i%30==0:print(i,round(times[i],2),pose[:3,3].round(3).tolist(),q,flush=True)
    poses=np.array(poses)
    np.savez_compressed(OUT/'refined_trajectory.npz',time_s=times,poses=poses)
    np.savez_compressed(OUT/'static_map.npz',points=static)
    (OUT/'refined_quality.json').write_text(json.dumps(quality,indent=2))
    p=static[(abs(static[:,1]-.18*static[:,0])<.5)&(static[:,0]>-.5)&(static[:,0]<9)&(static[:,2]<2.5)]
    fig,ax=plt.subplots(figsize=(13,6));ax.scatter(p[:,0],p[:,2],s=1)
    ax.plot(poses[:,0,3],poses[:,2,3],color='red',label='Estimated base path');ax.legend()
    ax.set(xlabel='Initial world X (m)',ylabel='Initial world Z (m)');ax.grid(alpha=.2);fig.tight_layout();fig.savefig(OUT/'static_profile.png',dpi=160)

if __name__=='__main__':main()
