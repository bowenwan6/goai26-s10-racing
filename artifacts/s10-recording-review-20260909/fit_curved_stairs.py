"""Fit common curved risers from the initial static scans; validate on later scans."""
import json
import numpy as np
from scipy.optimize import least_squares
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from reconstruct_stairs import OUT, raw_frames, voxel

def edge_points(p):
    # Isolate riser interiors, away from horizontal treads, for the first five edges.
    k=np.rint((p[:,0]-.49)/.56).astype(int)
    mid=-.45+(k+.5)*.16
    good=(k>=1)&(k<5)&(abs(p[:,1])<2.5)&(abs(p[:,0]-(.49+k*.56))<.3)&(abs(p[:,2]-mid)<.025)
    return p[good],k[good]

def solve(p,k,curved=True):
    design=np.c_[np.ones(len(p)),k,p[:,1],p[:,1]**2][:,:4 if curved else 3]
    initial=[.49,.56,0,.02][:design.shape[1]]
    fit=least_squares(lambda v:design@v-p[:,0],initial,loss='soft_l1',f_scale=.008)
    return fit.x,design@fit.x-p[:,0]

def selfcheck():
    rng=np.random.default_rng(3);k=rng.integers(1,5,600);y=rng.uniform(-2,2,600)
    truth=np.array([.48,.55,.01,.045]);x=np.c_[np.ones(len(k)),k,y,y*y]@truth
    found,_=solve(np.c_[x,y,np.zeros(len(y))],k)
    assert np.max(abs(found-truth))<1e-7

def main():
    selfcheck()
    times,frames=raw_frames();p=voxel(np.vstack([v for t,v in zip(times,frames) if t<3]),.015)
    p,k=edge_points(p);curve,err=solve(p,k);line,errline=solve(p,k,False)
    result=dict(model='x=edge+k*depth+slope*y+curvature*y^2 (local quadratic arc)',
                edge_m=curve[0],depth_m=curve[1],slope=curve[2],curvature_per_m=curve[3],
                approximate_radius_m=1/(2*abs(curve[3])),lateral_extent_m=2.5,
                points=len(p),curved_p90_m=float(np.percentile(abs(err),90)),straight_p90_m=float(np.percentile(abs(errline),90)),
                curved_median_m=float(np.median(abs(err))),straight_median_m=float(np.median(abs(errline))))
    (OUT/'curvature.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
    fig,ax=plt.subplots(figsize=(10,7));ax.scatter(p[:,0],p[:,1],c=k,s=2,cmap='viridis')
    yy=np.linspace(-2.5,2.5,200)
    for j in range(5):
        ax.plot(curve[0]+j*curve[1]+curve[2]*yy+curve[3]*yy**2,yy,'r-',lw=1)
        ax.plot(line[0]+j*line[1]+line[2]*yy,yy,color='gray',ls='--',lw=1)
    ax.set(xlabel='World X (m)',ylabel='World Y (m)',title='Static raw scans: curved risers (red), straight fit (gray)')
    ax.set_aspect('equal');ax.grid(alpha=.2);fig.tight_layout();fig.savefig(OUT/'curvature.png',dpi=160)

if __name__=='__main__':main()
