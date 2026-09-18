"""Read only post-B saved frames; visualize bounded Start and post-B profiles."""
from pathlib import Path
import json,sys
import numpy as np
from scipy.stats import binned_statistic
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from inspect_start import read_frame,sha,world
from extract_features import plane_fit
ROOT=Path(__file__).resolve().parents[1];MAP=ROOT.parents[1];OUT=ROOT/'three_sources'

def main():
    OUT.mkdir(exist_ok=False)
    d=np.load(ROOT/'sources/Start_saved_frames.npz');poses=d['poses'];raw=MAP/'raw'/MAP.name
    scope=json.loads((ROOT/'scope.json').read_text());center=np.array(scope['post_B_proposed_centerline_map_xy']);a=center[0];b=center[-1]
    direction=(b-a)/np.linalg.norm(b-a);lateral=np.array([-direction[1],direction[0]])
    chosen=np.flatnonzero((np.linalg.norm(poses[:,1:3]-(a+b)/2,axis=1)<10)&(poses[:,3]>3.5)&(poses[:,3]<7))
    qs=[];fs=[];rs=[];manifest=[]
    for i in chosen:
        p=raw/f'.sessions/session_0/lidar_cloud/{i}.pcd';x,_=read_frame(p);rr=np.linalg.norm(x,axis=1)
        take=np.isfinite(x).all(1)&(rr>.6)&(rr<8);q=world(x[take],poses[i]);rng=rr[take]
        local=q[:,:2]-a;s=local@direction;t=local@lateral
        keep=(s> -2)&(s<6)&(abs(t)<3)&(q[:,2]>3.5)&(q[:,2]<7.5)
        qs.append(q[keep]);fs.append(np.full(keep.sum(),i,np.int32));rs.append(rng[keep])
        manifest.append(dict(path=str(p),sha256=sha(p),frame=int(i),retained_points=int(keep.sum())))
    q=np.vstack(qs);ids=np.concatenate(fs);local=q[:,:2]-a;s=local@direction;t=local@lateral
    np.savez_compressed(OUT/'post_B.npz',xyz=q,frame=ids,range_m=np.concatenate(rs),station=s,cross=t,origin=a,direction=direction,lateral=lateral,poses=poses)
    (OUT/'manifest.json').write_text(json.dumps(manifest,indent=2))
    fig,axs=plt.subplots(2,3,figsize=(17,8))
    start=d['xyz'];fid=d['frame']
    for ax,y in zip(axs[0],[0.,1.5,3.]):
        take=(abs(start[:,1]-y)<.06)&(start[:,0]>=-1)&(start[:,0]<=14)&(start[:,2]>-.8)&(start[:,2]<.3)
        ax.scatter(start[take,0],start[take,2],s=1,c=fid[take],cmap='viridis',vmin=0,vmax=1435)
        edges=np.arange(-1,14.001,.1);z=binned_statistic(start[take,0],start[take,2],'median',bins=edges).statistic
        ax.plot((edges[1:]+edges[:-1])/2,z,'r',lw=1);ax.set(title=f'Start Y={y}±0.06m',xlabel='map X/m',ylabel='Z/m',ylim=(-.8,.3))
    for ax,t0 in zip(axs[1],[-1.,0.,1.]):
        take=(abs(t-t0)<.06)&(q[:,2]<5.6)
        ax.scatter(s[take],q[take,2],s=1,c=ids[take],cmap='viridis')
        ax.set(title=f'Post-B lateral={t0}±0.06m',xlabel='Distance from B exit / m',ylabel='Z/m',xlim=(-1,5),ylim=(4,5.6))
    fig.tight_layout();fig.savefig(OUT/'profiles.png',dpi=160);plt.close(fig)
    info=dict(post_frames=len(chosen),post_points=len(q),post_extent_xyz=[q.min(0).tolist(),q.max(0).tolist()],
        post_origin=a.tolist(),post_direction=direction.tolist(),post_5m_scope_stop=b.tolist(),no_robot=True)
    (OUT/'summary.json').write_text(json.dumps(info,indent=2));print(json.dumps(info,indent=2),flush=True)
    # Independent local plane candidates; not selected or exported yet.
    regions=[('start_west',start,(start[:,0]>-1)&(start[:,0]<3)&(abs(start[:,1])<1)),
        ('start_middle',start,(start[:,0]>3)&(start[:,0]<8)&(start[:,1]>1.5)&(start[:,1]<3)),
        ('start_east',start,(start[:,0]>8)&(start[:,0]<12)&(start[:,1]>1)&(start[:,1]<3)),
        ('post_B',q,(s>.5)&(s<4.5)&(abs(t)<.8))]
    results=[]
    for name,p,mask in regions:
        mask&=((p[:,2]>-.8)&(p[:,2]<.05)) if name.startswith('start') else ((p[:,2]>4.25)&(p[:,2]<4.9))
        c=plane_fit(p[mask]);err=p[mask,2]-p[mask,:2]@c[:2]-c[2]
        results.append(dict(name=name,plane=c.tolist(),points=int(mask.sum()),p50_p95_max_abs_error_m=np.percentile(abs(err),[50,95,100]).tolist()))
    (OUT/'plane_candidates.json').write_text(json.dumps(results,indent=2));print(json.dumps(results,indent=2),flush=True)

if __name__=='__main__':main()
