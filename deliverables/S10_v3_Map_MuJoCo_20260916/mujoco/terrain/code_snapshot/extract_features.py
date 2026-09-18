"""Local source-grounded plane/curve hypotheses, not global mesh smoothing."""
from pathlib import Path
import sys,json
import numpy as np
from scipy.stats import binned_statistic
from scipy.interpolate import UnivariateSpline
from scipy.ndimage import median_filter
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
ROOT=Path(__file__).resolve().parents[1]
font_manager.fontManager.addfont('/System/Library/Fonts/Supplemental/Arial Unicode.ttf')
plt.rcParams.update({'font.family':'Arial Unicode MS','axes.unicode_minus':False})

def plane_fit(p):
    cells=np.floor(p[:,:2]/.035).astype(int);_,iv=np.unique(cells,axis=0,return_inverse=True)
    oo=np.argsort(iv);cut=np.r_[0,np.flatnonzero(np.diff(iv[oo]))+1,len(oo)]
    samples=np.array([np.median(p[oo[a:b]],axis=0) for a,b in zip(cut[:-1],cut[1:])])
    X=np.c_[samples[:,:2],np.ones(len(samples))];rng=np.random.default_rng(20260915)
    best=None;score=-1
    for _ in range(600):
        ix=rng.choice(len(X),3,replace=False)
        if abs(np.linalg.det(X[ix]))<.002:continue
        c=np.linalg.solve(X[ix],samples[ix,2])
        if np.linalg.norm(c[:2])>.12:continue
        n=int((abs(X@c-samples[:,2])<.025).sum())
        if n>score:best=c;score=n
    assert best is not None
    for _ in range(7):
        r=abs(X@best-samples[:,2]);w=np.minimum(1,.02/np.maximum(r,1e-9));w[r>.08]=0
        best=np.linalg.lstsq(X*w[:,None]**.5,samples[:,2]*w**.5,rcond=None)[0]
    return best

def main():
    out=ROOT/'features';out.mkdir(exist_ok=False)
    d=np.load(ROOT/'sources/Start_saved_frames.npz');q=d['xyz'];ids=d['frame']
    sel=(q[:,0]>=3)&(q[:,0]<=8)&(q[:,1]>=-3)&(q[:,1]<=.5)
    q=q[sel];ids=ids[sel]
    # Manual region seed locates the visible raised island, not its final outline.
    # Only outer/core pavement observations establish the plane.
    core=(q[:,2]>-.65)&(q[:,2]<-.27)&((q[:,1]>-.55)|(q[:,0]<3.8)|(q[:,0]>7.4))
    coef=plane_fit(q[core]);res=q[:,2]-q[:,:2]@coef[:2]-coef[2]
    edges=np.arange(-3,.526,.025);ym=(edges[:-1]+edges[1:])/2
    records=[];profiles=[]
    # Profile change points rather than a thresholded XY occupancy outline.
    for x in np.arange(3.95,7.31,.075):
        keep=(abs(q[:,0]-x)<.065)&(res>-.15)&(res<.32)
        val=binned_statistic(q[keep,1],res[keep],lambda a:np.percentile(a,35),bins=edges).statistic
        n=binned_statistic(q[keep,1],res[keep],'count',bins=edges).statistic
        ok=np.isfinite(val)&(n>=3)
        if ok.sum()<8:continue
        # Search only where local observations exist; no long-gap bridging.
        vv=np.where(ok,val,np.nan);sm=vv.copy()
        for j in range(len(vv)):
            win=vv[max(0,j-2):j+3];finite=win[np.isfinite(win)]
            if len(finite)>=3:sm[j]=np.median(finite)
        candidates=[]
        for j in np.flatnonzero((ym> -2.85)&(ym<-.6)&ok):
            # Lower/right of edge = pavement, inside/left = raised returns.
            before=sm[(ym>ym[j]+.075)&(ym<ym[j]+.3)]
            after=sm[(ym>ym[j]-.25)&(ym<ym[j]-.05)]
            if np.isfinite(before).sum()<4 or np.isfinite(after).sum()<4:continue
            lower=np.nanmedian(before);upper=np.nanmedian(after)
            if abs(lower)<.05 and upper>.075 and upper-lower>.07:
                cost=abs(lower)+abs(sm[j]-.035)
                candidates.append((cost,j,float(lower),float(upper)))
        if candidates:
            cost,j,lo,hi=min(candidates)
            yy=float(ym[j]);around=keep&(abs(q[:,1]-yy)<.22)
            frames=np.unique(ids[around]);cohorts=np.unique(np.where(frames<30,0,np.where(frames<119,1,np.where(frames<1400,2,3))))
            records.append(dict(x=float(x),y=yy,lower_residual_m=lo,inner_residual_m=hi,frames=frames.tolist(),cohorts=cohorts.tolist(),cost=float(cost)))
        profiles.append((float(x),ym.copy(),vv.copy(),sm.copy()))
    arr=np.array([[r['x'],r['y']] for r in records]);assert len(arr)>10
    # A small smoothing budget regularizes observations, not the road footprint.
    sp=UnivariateSpline(arr[:,0],arr[:,1],s=len(arr)*.025**2,k=3)
    xx=np.linspace(arr[:,0].min(),arr[:,0].max(),400);yy=sp(xx)
    detail=dict(pavement_plane=coef.tolist(),plane_slope_deg=float(np.degrees(np.arctan(np.linalg.norm(coef[:2])))),
        seed_only='Core outside x[3.8,7.4], y<-0.55 excludes elevated region. It is not the inferred boundary.',
        sample_box_xy=[3,8,-3,.5],plane_core_points=int(core.sum()),
        all_core_abs_vertical_residual_p50_p95_max_m=np.percentile(abs(res[core]),[50,95,100]).tolist(),
        boundary_candidates=records,boundary_curve_xy=np.c_[xx,yy].tolist(),
        boundary_curve_fit_y_error_p50_p95_max_m=np.percentile(abs(sp(arr[:,0])-arr[:,1]),[50,95,100]).tolist(),
        curve_is_geometric_transition_not_confirmed_concrete_kerb=True)
    (out/'extracted.json').write_text(json.dumps(detail,indent=2));np.savez_compressed(out/'sample_sources.npz',xyz=q,frame=ids,residual=res,core=core)
    fig,axs=plt.subplots(1,2,figsize=(15,6))
    low=(res>-.15)&(res<.32)
    axs[0].scatter(q[low,0],q[low,1],c=res[low],s=2,cmap='terrain',vmin=-.04,vmax=.22)
    axs[0].plot(arr[:,0],arr[:,1],'rx',ms=4,label='逐剖面变化候选');axs[0].plot(xx,yy,c='black',label='初始曲线拟合')
    axs[0].set(aspect='equal',xlabel='X/m',ylabel='Y/m',title='低位回波与几何边缘（不是叶片投影挖洞）');axs[0].legend()
    for x,y,raw,sm in profiles[::5]:axs[1].plot(y,sm,label=f'X={x:.2f}')
    axs[1].set(ylim=(-.15,.3),xlabel='Y/m',ylabel='相对路面Z/m',title='局部剖面的抬高过渡');axs[1].legend()
    fig.tight_layout();fig.savefig(ROOT/'inspection/06_feature_fit.png',dpi=160);plt.close(fig)
    print(json.dumps({k:v for k,v in detail.items() if k not in ['boundary_candidates','boundary_curve_xy']},indent=2),flush=True)
    print('boundaries',arr.round(3).tolist(),flush=True)

if __name__=='__main__':main()
