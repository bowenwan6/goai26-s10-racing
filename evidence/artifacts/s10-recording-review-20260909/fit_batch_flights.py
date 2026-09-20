"""Fit local analytic stair flights; do not confuse fitted geometry with measured contacts."""
import argparse
import json
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from batch_match_stairs import BATCH,CONFIG

def distance(p,v,count):
    edge,z,h,d,b,c=v;x=p[:,0]-b*p[:,1]-c*p[:,1]**2;zz=p[:,2];ds=[]
    for k in range(count+1):
        lo=-20 if k==0 else edge+(k-1)*d;hi=edge+k*d if k<count else 30
        ds.append(np.hypot(x-np.clip(x,lo,hi),zz-z-k*h))
        if k<count:ds.append(np.hypot(x-hi,zz-np.clip(zz,min(z+k*h,z+(k+1)*h),max(z+k*h,z+(k+1)*h))))
    return np.min(ds,axis=0)

def run(job):
    out=BATCH/job['name'];a=np.load(out/'reference_motion.npz');data=np.load(out/'registered_terrain.npz')
    fits=[]
    for start,end,label in job['phases']:
        if label not in ('stairs_up','stairs_down'):continue
        i=np.argmin(abs(a['time_s']-start));j=np.argmin(abs(a['time_s']-end));origin=a['root_position_m'][i]
        delta=a['root_position_m'][j]-origin;length=np.linalg.norm(delta[:2]);yaw=np.arctan2(delta[1],delta[0]);cy,sy=np.cos(yaw),np.sin(yaw)
        rot=np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
        localroot=(a['root_position_m'][i:j+1:10]-origin)@rot;track=cKDTree(localroot[:,:2])
        def crop(p):
            pp=(p-origin)@rot;dd,idx=track.query(pp[:,:2]);dz=pp[:,2]-localroot[idx,2]
            return pp[(pp[:,0]>-1.2)&(pp[:,0]<length+1.2)&(abs(pp[:,1])<1.3)&(dz>-.8)&(dz<-.1)&(dd<1.6)]
        train=crop(data['train']);test=crop(data['test']);sample=train[::max(1,len(train)//4500)]
        sign=1 if label=='stairs_up' else -1;estimate=max(2,round(abs(delta[2])/.15));candidates=[]
        for count in range(max(2,estimate-1),estimate+2):
            guess=[.25,-.43,sign*.15,np.clip(length/max(1,count-.5),.35,.7),0,.015]
            low=[-1.,-.65,.12 if sign>0 else -.18,.3,-.15,-.08];high=[1.,-.25,.18 if sign>0 else -.12,.75,.15,.08]
            # Dimension priors are weak (recorded dimensions are approximate and absent in three recordings).
            def residual(v):return np.r_[distance(sample,v,count),np.sqrt(len(sample))*.12*(abs(v[2])-.15),np.sqrt(len(sample))*.04*(v[3]-.55)]
            fit=least_squares(residual,guess,bounds=(low,high),loss='soft_l1',f_scale=.025,max_nfev=100)
            err=distance(test,fit.x,count);score=float(np.mean(np.minimum(err,.15)))
            candidates.append((score,count,fit.x,err))
        score,count,v,err=min(candidates,key=lambda row:row[0])
        result=dict(start_s=start,end_s=end,direction=label,origin_m=origin.tolist(),yaw_rad=float(yaw),length_m=float(length),
                    count=count,parameters=v.tolist(),train_points=len(train),test_points=len(test),median_m=float(np.median(err)),p90_m=float(np.percentile(err,90)),
                    candidates=[dict(count=n,score=s) for s,n,_,_ in candidates],geometry_status='fitted_local_hypothesis')
        fits.append(result);print(job['name'],label,start,'steps',count,'h/d',v[2:4].round(3),'median/p90',np.percentile(err,[50,90]).round(3),flush=True)
    # Reverse pass over the first climbed flight shares that same geometry, rather than overwriting it.
    if job['name']=='154856_up_down':fits[1]['shared_geometry_with']=0
    (out/'fitted_flights.json').write_text(json.dumps(fits,indent=2))
    base=np.load(out/'terrain_proxy.npz');h=base['height_m'].copy();valid=base['valid'].copy();originxy=base['origin_xy_m'];size=float(base['resolution_m'])
    ii,jj=np.indices(h.shape);xy=np.c_[originxy[0]+(ii.ravel()+.5)*size,originxy[1]+(jj.ravel()+.5)*size,np.zeros(h.size)]
    for f in fits:
        if 'shared_geometry_with' in f:continue
        angle=f['yaw_rad'];cy,sy=np.cos(angle),np.sin(angle);rot=np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
        pp=(xy-np.array(f['origin_m']))@rot;e,z,height,d,b,c=f['parameters'];x=pp[:,0]-b*pp[:,1]-c*pp[:,1]**2
        region=(x>-2.8)&(x<f['length_m']+1.2)&(abs(pp[:,1])<1.7)
        level=np.clip(np.floor((x-e)/d)+1,0,f['count']);worldz=f['origin_m'][2]+z+level*height
        h.ravel()[region]=worldz[region];valid.ravel()[region]=True
    # Grid is retained for approximate wheel checks; render analytic flights with crisp vertical risers.
    from build_batch_terrain import mesh
    vertices,faces=mesh(originxy,h,valid,size)
    np.savez_compressed(out/'terrain_fit.npz',origin_xy_m=originxy,resolution_m=size,height_m=h,observed=base['observed'],valid=valid,vertices=vertices,faces=faces)
    text=['#usda 1.0','(\n metersPerUnit = 1\n upAxis = "Z"\n)','def Xform "FittedTerrain" {']
    boxes=[]
    for n,f in enumerate(fits):
        if 'shared_geometry_with' in f:continue
        angle=f['yaw_rad'];cy,sy=np.cos(angle),np.sin(angle);e,z,height,d,b,c=f['parameters']
        for k in range(f['count']+1):
            for col,y in enumerate(np.arange(-1.7,1.7,.17)+.085):
                lo=-2.8 if k==0 else e+(k-1)*d+b*y+c*y*y
                hi=f['length_m']+1.2 if k==f['count'] else e+k*d+b*y+c*y*y
                if hi<=lo:continue
                top=f['origin_m'][2]+z+k*height;bottom=f['origin_m'][2]+z+min(0,f['count']*height)-.15
                pos=np.array(f['origin_m'])+np.array([cy*(lo+hi)/2-sy*y,sy*(lo+hi)/2+cy*y,0]);pos[2]=(top+bottom)/2
                scale=[hi-lo,.17,top-bottom];name=f'Flight{n}_Step{k}_{col}';boxes.append(dict(name=name,pos=pos.tolist(),size=scale,yaw=angle))
                text.extend([f' def Cube "{name}" {{','  double size = 1','  color3f[] primvars:displayColor = [(0.54,0.65,0.69)]',
                    '  double3 xformOp:translate = ('+','.join(map(str,pos))+')',f'  float xformOp:rotateZ = {np.degrees(angle)}',
                    '  double3 xformOp:scale = ('+','.join(map(str,scale))+')','  uniform token[] xformOpOrder = ["xformOp:translate","xformOp:rotateZ","xformOp:scale"]',' }'])
    text.append('}');(out/'terrain_fit.usda').write_text('\n'.join(text));(out/'terrain_boxes.json').write_text(json.dumps(boxes))

def selfcheck():
    for sign in (1,-1):
        v=[.5,-.4,sign*.15,.55,0,.04]
        assert distance(np.array([[.7,0,-.4+sign*.15],[.54,1,-.4+sign*.07]]),v,5).max()<1e-8

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--job',choices=[j['name'] for j in CONFIG['jobs']]);parser.add_argument('--selfcheck',action='store_true');a=parser.parse_args()
    if a.selfcheck:selfcheck();print('Bidirectional stair-distance selfcheck passed')
    elif a.job:run(next(j for j in CONFIG['jobs'] if j['name']==a.job))
    else:
        for job in CONFIG['jobs']:run(job)
