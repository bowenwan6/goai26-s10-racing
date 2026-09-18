"""Read saved local frames, not the old downsample cache; never touch originals."""
from pathlib import Path
import sys,json,hashlib
import numpy as np
from scipy.stats import binned_statistic_2d
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager

ROOT=Path(__file__).resolve().parents[1]; MAP=ROOT.parents[1]
sys.path.insert(0,str(MAP)); from map_io import world
font_manager.fontManager.addfont('/System/Library/Fonts/Supplemental/Arial Unicode.ttf')
plt.rcParams.update({'font.family':'Arial Unicode MS','axes.unicode_minus':False})

def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()

def read_frame(p):
    header={}
    with p.open('rb') as f:
        while f.tell()<16384:
            words=f.readline().decode('ascii').split()
            if words and not words[0].startswith('#'):header[words[0]]=words[1:]
            if words and words[0]=='DATA':break
        offset=f.tell()
    assert header['DATA']==['binary']
    assert all(x=='1' for x in header['COUNT'])
    types={'F':'f','I':'i','U':'u'}
    dtype=np.dtype([(n,'<'+types[t]+s) for n,t,s in zip(header['FIELDS'],header['TYPE'],header['SIZE'])])
    n=int(header['POINTS'][0]);assert p.stat().st_size==offset+n*dtype.itemsize
    d=np.memmap(p,mode='r',dtype=dtype,offset=offset,shape=(n,))
    return np.c_[d['x'],d['y'],d['z']],np.array(d['intensity'])

def main():
    out=ROOT/'sources';out.mkdir(exist_ok=False);(ROOT/'inspection').mkdir(exist_ok=False)
    raw=MAP/'raw'/MAP.name;posefile=raw/'.sessions/session_0/poses.txt';poses=np.loadtxt(posefile)
    frameids=np.flatnonzero((poses[:,1]>-13)&(poses[:,1]<25)&(poses[:,2]>-14)&(poses[:,2]<16)&(poses[:,3]>-1)&(poses[:,3]<1.5))
    qs=[];ins=[];ids=[];ranges=[];manifest=[]
    for i in frameids:
        p=raw/f'.sessions/session_0/lidar_cloud/{i}.pcd';x,intensity=read_frame(p);rr=np.linalg.norm(x,axis=1)
        good=np.isfinite(x).all(1)&(rr>.6)&(rr<8);q=world(x[good],poses[i]);vv=intensity[good];rr=rr[good]
        keep=(q[:,0]>-5)&(q[:,0]<17)&(q[:,1]>-6)&(q[:,1]<8)&(q[:,2]>-1.5)&(q[:,2]<2.5)
        q=q[keep];qs.append(q);ins.append(vv[keep]);ids.append(np.full(len(q),i,np.int32));ranges.append(rr[keep])
        manifest.append(dict(path=str(p),sha256=sha(p),frame=int(i),source_points=len(x),retained_points=len(q)))
    q=np.vstack(qs);fid=np.concatenate(ids);inten=np.concatenate(ins);rr=np.concatenate(ranges)
    np.savez_compressed(out/'Start_saved_frames.npz',xyz=q.astype(np.float64),frame=fid,intensity=inten,range_m=rr,poses=poses)
    B=MAP/'reconstruction/B_structured_repair_v1/candidate_05'
    for p in [posefile,raw/'full_cloud.pcd',B/'geometry.npz',B/'assets/B_structured_map.obj',B/'repair_report.json',B/'B_structured_collision.xml']:
        manifest.append(dict(path=str(p),sha256=sha(p)))
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2))
    info=dict(read_frames=len(frameids),points=len(q),original_xyz_not_downsampled=True,xyz_crop=[-5,17,-6,8,-1.5,2.5],range_m=[.6,8],
        frame_selection='Saved poses within expanded XY box, pose Z -1 to 1.5 m. Original optimized transforms applied once. No timing/IMU or registration changes.',
        intensity_percentiles=np.percentile(inten,[0,10,50,90,100]).tolist(),frame_ids=frameids.tolist())
    (out/'summary.json').write_text(json.dumps(info,indent=2));print(json.dumps(info,indent=2),flush=True)
    ex=np.arange(-5,17.001,.1);ey=np.arange(-6,8.001,.1)
    floor=(q[:,2]>-.85)&(q[:,2]<.1)
    fields=[('低层候选中位高程（非地面标签）',q[floor],q[floor,2],'median',(-.65,-.15),'terrain'),
            ('低层候选强度（未标定，仅参考）',q[floor],inten[floor],'median',(0,80),'gray'),
            ('同一格全部回波Z的90%分位',q,q[:,2],lambda a:np.percentile(a,90),(0,1.5),'viridis'),
            ('低层回波厚度 P90-P10',q[floor],q[floor,2],lambda a:np.percentile(a,90)-np.percentile(a,10),(0,.25),'magma')]
    fig,axs=plt.subplots(2,2,figsize=(18,12))
    for ax,(title,points,values,stat,lim,cmap) in zip(axs.ravel(),fields):
        z=binned_statistic_2d(points[:,0],points[:,1],values,statistic=stat,bins=[ex,ey]).statistic
        im=ax.pcolormesh(ex,ey,z.T,cmap=cmap,vmin=lim[0],vmax=lim[1],rasterized=True);fig.colorbar(im,ax=ax,shrink=.7)
        good=(poses[:,3]<1.5)&(poses[:,1]>-5)&(poses[:,1]<17)&(poses[:,2]>-6)&(poses[:,2]<8)
        ax.plot(poses[good,1],poses[good,2],'.',color='#b7446a',ms=1)
        ax.set(aspect='equal',title=title,xlabel='map X/m',ylabel='map Y/m',xlim=(-5,17),ylim=(-6,8))
    fig.suptitle('Start 局部保存帧只读盘点｜10cm格仅用于观察，不作为新网格轮廓',fontsize=16)
    fig.tight_layout();fig.savefig(ROOT/'inspection/01_local_sources.png',dpi=140);plt.close(fig)
    # Literal longitudinal slices; retain all local returns inside fixed display range.
    fig,axs=plt.subplots(5,1,figsize=(17,12),sharex=True,sharey=True)
    for ax,y in zip(axs,[-2.,-1.,0.,1.,2.]):
        s=(abs(q[:,1]-y)<.06)&(q[:,2]>-.9)&(q[:,2]<.45)
        ax.scatter(q[s,0],q[s,2],c=fid[s],s=.5,cmap='viridis',vmin=0,vmax=1435)
        ax.set(title=f'Y={y} ± 0.06m；颜色为保存帧编号',ylim=(-.9,.45),ylabel='Z/m');ax.grid(alpha=.2)
    axs[-1].set_xlabel('map X/m');fig.tight_layout();fig.savefig(ROOT/'inspection/02_raw_slices.png',dpi=140);plt.close(fig)

if __name__=='__main__':main()
