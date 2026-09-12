"""Reconstruct the first stair passage from native dual-lidar frames and IMU.
Run: python -s artifacts/s10-recording-review-20260909/reconstruct_stairs.py
"""
import json
import sqlite3
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp
from fit_stairs import ROOT, SID, load, rotation
from review import store_types

OUT=ROOT/'stairs_reconstruction'

def voxel(p,size=.08):
    return p[np.unique(np.floor(p/size).astype(np.int32),axis=0,return_index=True)[1]]

def raw_frames(end=29, *, sid=SID, start=0, out=OUT):
    cache=out/'raw_frames.npz'
    if cache.exists():
        a=np.load(cache)
        if 'recording_id' in a:
            assert str(a['recording_id'])==sid and float(a['start_s'])==start and float(a['end_s'])==end, 'Cache belongs to another interval'
        elif (sid,start,end,out)!=(SID,0,29,OUT):raise ValueError('Unidentified cache outside original first-flight interval')
        return a['times'],[a[f'p{i}'] for i in range(len(a['times']))]
    meta,data,_=load(sid);anchor=meta['manifest']['started_wall_ns'];store=store_types()
    it=(data['IMU_src']-anchor)/1e9
    orientation=Slerp(it,Rotation.from_quat(data['IMU_v'][:,:4]))
    first=rotation(data['IMU_v'][0,:4]);yaw=np.arctan2(first[1,0],first[0,0])
    align=Rotation.from_euler('z',-yaw)
    clouds={}
    db=Path(meta['raw_path'])/'bag/bag_0.db3'
    with sqlite3.connect(db.resolve().as_uri()+'?mode=ro',uri=True) as conn:
        for tid,name,kind in conn.execute('SELECT id,name,type FROM topics'):
            if name not in ('/rslidar_front/points','/rslidar_rear/points'):continue
            rows=[]
            for rx,blob in conn.execute('SELECT timestamp,data FROM messages WHERE topic_id=? AND timestamp>=? AND timestamp<? ORDER BY timestamp',(tid,anchor+int((start-.5)*1e9),anchor+int((end+.5)*1e9))):
                msg=store.deserialize_cdr(blob,kind);t=(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec-anchor)/1e9
                if not max(it[0],start)<=t<=min(it[-1],end):continue
                fields={f.name:f for f in msg.fields}
                assert all(fields[k].datatype==7 and fields[k].count==1 for k in ('x','y','z'))
                assert msg.row_step>=msg.point_step*msg.width and len(msg.data)>=msg.row_step*msg.height
                dtype=np.dtype(dict(names=['x','y','z'],formats=[('>' if msg.is_bigendian else '<')+'f4']*3,
                                    offsets=[fields[k].offset for k in ('x','y','z')],itemsize=msg.point_step))
                a=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,strides=(msg.row_step,msg.point_step))
                p=np.stack([a[k].ravel() for k in ('x','y','z')],axis=1)
                dist=np.linalg.norm(p,axis=1);p=p[np.isfinite(p).all(axis=1)&(dist>.65)&(dist<12)]
                p=(align*orientation(t)).apply(p)
                p=voxel(p,.07)
                rows.append((t,p));
            clouds[name]=rows
            print('decoded',name,len(rows),flush=True)
    rear=clouds['/rslidar_rear/points'];rt=np.array([x[0] for x in rear]);times=[];points=[]
    for t,p in clouds['/rslidar_front/points']:
        j=np.argmin(abs(rt-t));assert abs(rt[j]-t)<.025
        times.append(t);points.append(voxel(np.vstack([p,rear[j][1]]),.08))
    np.savez_compressed(cache,times=times,recording_id=sid,start_s=start,end_s=end,**{f'p{i}':p.astype('f4') for i,p in enumerate(points)})
    return np.array(times),points

def target(p):
    p=voxel(p,.10);tree=cKDTree(p);_,idx=tree.query(p,k=16,workers=1)
    centered=p[idx]-p[idx].mean(axis=1)[:,None,:]
    vals,vec=np.linalg.eigh(np.einsum('nki,nkj->nij',centered,centered))
    good=(vals[:,0]<.15*vals[:,1])&(vals[:,1]>.0003)
    return p,tree,vec[:,:,0],good

def icp(source,initial,ref):
    p,tree,normals,good=ref;pose=initial.copy()
    src=voxel(source,.12)
    for iteration in range(20):
        moved=src@pose[:3,:3].T+pose[:3,3]
        dist,idx=tree.query(moved,workers=1)
        keep=(dist<(.4 if iteration<4 else .20))&good[idx]
        a=moved[keep];n=normals[idx[keep]];res=np.sum((a-p[idx[keep]])*n,axis=1)
        if len(res)<150:raise ValueError(f'Insufficient ICP correspondences {len(res)}')
        weight=np.minimum(1,.025/np.maximum(abs(res),1e-9))
        matrix=np.c_[np.cross(a,n),n]
        step=np.linalg.lstsq(matrix*np.sqrt(weight[:,None]),-res*np.sqrt(weight),rcond=None)[0]
        delta=np.eye(4);delta[:3,:3]=Rotation.from_rotvec(step[:3]).as_matrix();delta[:3,3]=step[3:]
        pose=delta@pose
        if np.linalg.norm(step)<1e-5:break
    return pose,dict(median_plane_m=float(np.median(abs(res))),p90_plane_m=float(np.percentile(abs(res),90)),
                     matched_fraction=float(np.mean(keep)),pairs=len(res),iterations=iteration+1)

def selfcheck():
    rng=np.random.default_rng(42)
    p=np.vstack([np.c_[rng.uniform(-2,2,(1000,2)),np.zeros(1000)],
                 np.c_[np.zeros(1000),rng.uniform(-2,2,(1000,2))],
                 np.c_[rng.uniform(-2,2,1000),np.full(1000,2),rng.uniform(-2,2,1000)]])
    truth=np.eye(4);truth[:3,:3]=Rotation.from_euler('xyz',[.02,-.01,.03]).as_matrix();truth[:3,3]=[.08,-.06,.05]
    source=(p-truth[:3,3])@truth[:3,:3]
    found,_=icp(source,np.eye(4),target(p))
    assert np.max(abs(found-truth))<.004,found

def main():
    OUT.mkdir(exist_ok=True);selfcheck();times,frames=raw_frames()
    poses=[np.eye(4)];stats=[{}];maps=[frames[0]];ref=target(maps[0])
    for i,p in enumerate(frames[1:],1):
        guess=poses[-1].copy()
        if i>1:guess[:3,3]+=poses[-1][:3,3]-poses[-2][:3,3]
        pose,quality=icp(p,guess,ref);poses.append(pose);stats.append(quality)
        if i%5==0:
            maps.append(p@pose[:3,:3].T+pose[:3,3]);maps=maps[-12:]
            ref=target(np.vstack(maps))
        if i%20==0:print(i,round(times[i],2),pose[:3,3].round(3).tolist(),quality,flush=True)
    np.savez_compressed(OUT/'trajectory.npz',time_s=times,poses=poses)
    (OUT/'icp_quality.json').write_text(json.dumps(stats,indent=2))
    world=voxel(np.vstack([p@pose[:3,:3].T+pose[:3,3] for p,pose in zip(frames[::5],poses[::5])]),.05)
    np.savez_compressed(OUT/'map.npz',points=world)
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(1,2,figsize=(14,6));poses=np.array(poses)
    ax[0].scatter(world[:,0],world[:,1],c=world[:,2],s=.3,vmin=-.5,vmax=2)
    ax[0].plot(poses[:,0,3],poses[:,1,3],'r-');ax[0].set(xlim=(-3,10),ylim=(-6,6),xlabel='World X (m)',ylabel='World Y (m)');ax[0].set_aspect('equal')
    for k,name in enumerate('XYZ'):ax[1].plot(times,poses[:,k,3],label=name)
    ax[1].legend();ax[1].set(xlabel='Source time (s)',ylabel='Base displacement (m)');ax[1].grid(alpha=.2)
    fig.tight_layout();fig.savefig(OUT/'trajectory.png',dpi=150)
    print('DONE',len(times),flush=True)

if __name__=='__main__':main()
