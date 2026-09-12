"""Fit the complete first flight to registered near-range scans and export scene geometry."""
import json
import numpy as np
from scipy.optimize import least_squares
from reconstruct_stairs import OUT, raw_frames, voxel
import matplotlib.pyplot as plt

def distance(p,v,count=11):
    edge,z,h,d,b,c=v
    x=p[:,0]-b*p[:,1]-c*p[:,1]**2;zz=p[:,2]
    distances=[]
    for k in range(count+1):
        a=-10 if k==0 else edge+(k-1)*d;bnd=edge+k*d if k<count else 15
        distances.append(np.hypot(x-np.clip(x,a,bnd),zz-z-k*h))
        if k<count:distances.append(np.hypot(x-bnd,zz-np.clip(zz,z+k*h,z+(k+1)*h)))
    return np.min(distances,axis=0)

def main():
    v=[.48,-.45,.15,.55,0,.04]
    pts=np.array([[.7,0,-.30],[1.3,0,-.15],[.52,1,-.38]])
    assert np.max(distance(pts,v))<1e-8
    times,frames=raw_frames();poses=np.load(OUT/'refined_trajectory.npz')['poses'];train=[];test=[]
    for i,(p,pose,t) in enumerate(zip(frames,poses,times)):
        p=p[(np.linalg.norm(p,axis=1)<2.7)&(abs(p[:,1])<1.3)]
        p=p@pose[:3,:3].T+pose[:3,3]
        p=p[(p[:,0]>.15)&(p[:,0]<7.5)&(abs(p[:,1])<2)&(p[:,2]<1.8)&(p[:,2]>-.55)]
        (train if i%2==0 else test).append(p)
    train=voxel(np.vstack(train),.035);test=voxel(np.vstack(test),.035)
    # Spatial terrain region excludes objects more than 20cm above/below the rough stair slope.
    def terrain(p):
        k=np.clip(np.ceil((p[:,0]-.49-.04*p[:,1]**2)/.57),0,11)
        return p[abs(p[:,2]-(-.45+k*.16))<.20]
    train=terrain(train);test=terrain(test)
    initial=[.48,-.45,.16,.57,.015,.048]
    fit=least_squares(lambda v:distance(train,v),initial,
                       bounds=([.3,-.55,.14,.52,-.10,0],[.7,-.35,.175,.62,.10,.10]),
                       loss='soft_l1',f_scale=.02,max_nfev=200)
    v=fit.x;err=distance(test,v)
    recorded=distance(test,[v[0],v[1],.15,.55,v[4],v[5]])
    result=dict(count=11,count_status='first-flight candidate checked against terminal landing',
                edge_m=v[0],floor_z_m=v[1],height_m=v[2],depth_m=v[3],lateral_slope=v[4],curvature_per_m=v[5],
                fitted_points=len(train),held_out_points=len(test),median_m=float(np.median(err)),p90_m=float(np.percentile(err,90)),
                within_5cm=float(np.mean(err<.05)),recorded_dimensions_median_m=float(np.median(recorded)),recorded_dimensions_p90_m=float(np.percentile(recorded,90)),
                validation='alternating native scans in the terrain ROI; registration shares stationary reference map')
    (OUT/'scene_geometry.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2),flush=True)
    np.savez_compressed(OUT/'near_terrain.npz',train=train,test=test)
    fig,axes=plt.subplots(1,2,figsize=(15,6))
    x=test[:,0]-v[4]*test[:,1]-v[5]*test[:,1]**2
    axes[0].scatter(x,test[:,2],s=.5,c='steelblue')
    xx=[0];zz=[v[1]]
    for k in range(11):xx.extend([v[0]+k*v[3]]*2);zz.extend([v[1]+k*v[2],v[1]+(k+1)*v[2]])
    xx.append(7.5);zz.append(zz[-1]);axes[0].plot(xx,zz,color='darkorange',lw=2)
    axes[0].set(xlabel='Curvature-corrected X (m)',ylabel='World Z (m)',title='Held-out near-range points / fitted stairs')
    axes[1].scatter(test[:,0],test[:,1],s=.5,c=test[:,2],cmap='viridis')
    y=np.linspace(-2,2,200)
    for k in range(11):axes[1].plot(v[0]+k*v[3]+v[4]*y+v[5]*y*y,y,color='darkorange',lw=1)
    axes[1].plot(poses[:,0,3],poses[:,1,3],color='red');axes[1].set(xlabel='World X (m)',ylabel='World Y (m)',title='Curved edges / estimated base path');axes[1].set_aspect('equal')
    for ax in axes:ax.grid(alpha=.2)
    fig.tight_layout();fig.savefig(OUT/'scene_fit.png',dpi=150)

if __name__=='__main__':main()
