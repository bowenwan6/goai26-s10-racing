"""Local stair/profile matching; run with python -s fit_stairs.py (NumPy/Matplotlib).
Inputs are existing decoded caches. Never writes to raw recordings.
"""
import base64
import ast
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'stairs_alignment'
SID = 'gait_20260909_145331_616f9d8038d9'

def rotation(q):
    x,y,z,w = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def load(sid=SID):
    meta=json.loads((ROOT/'decoded'/f'{sid}.meta.json').read_text(encoding='utf-8'))
    data=np.load(ROOT/'decoded'/f'{sid}.npz')
    clouds=json.loads((ROOT/'decoded'/f'{sid}.clouds.json').read_text())
    return meta,data,clouds

def leveled(c,data,anchor):
    t=(data['IMU_src']-anchor)/1e9
    i=np.argmin(abs(t-c['src']))
    if abs(t[i]-c['src'])>.03: raise ValueError('No synchronized IMU')
    r=rotation(data['IMU_v'][i,:4])
    yaw=np.arctan2(r[1,0],r[0,0])
    cy,sy=np.cos(yaw),np.sin(yaw)
    r=np.array([[cy,sy,0],[-sy,cy,0],[0,0,1]])@r
    p=np.frombuffer(base64.b64decode(c['xyz_mm_b64']),dtype='<i2').reshape(-1,3)/1000
    return p@r.T

def profile_distance(p,params,h=.15,d=.55):
    edge,z,yaw=params
    x=p[:,0]*np.cos(yaw)+p[:,1]*np.sin(yaw)
    zz=p[:,2]
    distances=[]
    # ponytail: first flight only, eight model risers; upper landing/count are unmeasured.
    for k in range(9):
        a=-10 if k==0 else edge+(k-1)*d
        b=edge+k*d if k<8 else 10
        distances.append(np.hypot(x-np.clip(x,a,b),zz-(z+k*h)))
        if k<8:
            distances.append(np.hypot(x-b,zz-np.clip(zz,z+k*h,z+(k+1)*h)))
    return np.min(distances,axis=0)

def fit(p,guess,h=.15,d=.55,floor_bound=None):
    p=p[(abs(p[:,1])<.65)&(p[:,0]>.25)&(p[:,0]<3)&(p[:,2]<.65)&(p[:,2]>-1)]
    if len(p)<100:raise ValueError('Insufficient local stair points')
    low,high=([guess[0]-.4,guess[1]-.25,-.5],[guess[0]+.4,guess[1]+.25,.5])
    if floor_bound:low[1],high[1]=floor_bound
    result=least_squares(lambda v:profile_distance(p[::2],v,h,d),np.clip(guess,low,high),
                         bounds=(low,high),
                         loss='soft_l1',f_scale=.015,max_nfev=100)
    err=profile_distance(p[1::2],result.x,h,d)
    return result.x,dict(points=len(p),median_m=float(np.median(err)),p90_m=float(np.percentile(err,90)),
                        within_3cm=float(np.mean(err<.03)))

def selfcheck():
    rng=np.random.default_rng(4);p=[]
    for k in range(5):
        x=rng.uniform(.48+(k-1)*.55+.03,.48+k*.55-.03,100) if k else rng.uniform(.26,.45,100)
        p.extend(np.c_[x,rng.uniform(-.6,.6,100),np.full(100,-.45+k*.15)])
    p=np.array(p);yaw=.08;p[:,:2]=p[:,:2]@np.array([[np.cos(yaw),np.sin(yaw)],[-np.sin(yaw),np.cos(yaw)]])
    # Include riser samples to constrain edge rather than only height plateaus.
    for k in range(4):
        yy=rng.uniform(-.6,.6,80);xx=np.full(80,.48+k*.55)
        pp=np.c_[xx*np.cos(yaw)-yy*np.sin(yaw),xx*np.sin(yaw)+yy*np.cos(yaw),rng.uniform(-.45+k*.15,-.30+k*.15,80)]
        p=np.vstack([p,pp])
    found,_=fit(p,[.5,-.44,0])
    assert np.max(abs(found-[.48,-.45,.08]))<.005,found
    assert np.allclose(rotation(np.array([0.,0.,0.,1.])),np.eye(3))

def main():
    selfcheck()
    OUT.mkdir(exist_ok=True)
    meta,data,clouds=load()
    fig,axes=plt.subplots(2,3,figsize=(15,8),sharex=True,sharey=True)
    for ax,t in zip(axes.flat,[1,9,11,13,16,22]):
        c=min(clouds['/rslidar_front/points'],key=lambda c:abs(c['src']-t))
        p=leveled(c,data,meta['manifest']['started_wall_ns'])
        p=p[(abs(p[:,1])<.6)&(p[:,0]>.2)&(p[:,0]<5)]
        ax.scatter(p[:,0],p[:,2],s=2)
        ax.set(title=f"t={c['src']:.2f}s",xlabel='Forward from base (m)',ylabel='Leveled Z (m)',ylim=(-1,1.8),xlim=(0,5))
        ax.grid(alpha=.2)
    fig.tight_layout();fig.savefig(OUT/'profiles.png',dpi=150)
    rows=[];guess=np.array([.48,-.45,0.])
    frames=[c for c in clouds['/rslidar_front/points'] if 0<c['src']<25]
    for c in frames:
        p=leveled(c,data,meta['manifest']['started_wall_ns'])
        if c['src']<11.5:
            candidates=[fit(p,[e,-.45,0],floor_bound=(-.50,-.40)) for e in (.35,.65,.95)]
            params,quality=min(candidates,key=lambda item:item[1]['p90_m'])
        else:params,quality=fit(p,guess)
        rows.append(dict(t_s=c['src'],edge_from_base_m=float(params[0]),floor_from_base_m=float(params[1]),
                         stair_yaw_deg=float(np.degrees(params[2])),
                         alignment_usable=bool(c['src']<11.5 and quality['p90_m']<.04 and abs(params[2])<.2),**quality))
        guess=params
    # Check the recorded dimensions against an unconstrained local fit on a quiet frame.
    c=min(frames,key=lambda c:abs(c['src']-1));p=leveled(c,data,meta['manifest']['started_wall_ns'])
    p=p[(abs(p[:,1])<.65)&(p[:,0]>.25)&(p[:,0]<3)&(p[:,2]<.65)&(p[:,2]>-1)]
    free=least_squares(lambda v:profile_distance(p,v[:3],v[3],v[4]),[.48,-.45,0,.15,.55],
                       bounds=([.2,-.7,-.5,.1,.4],[.9,-.2,.5,.2,.7]),loss='soft_l1',f_scale=.015)
    result=dict(recording=SID,units='m, seconds, degrees',height_m=.15,depth_m=.55,
                frame_assumption='PointCloud2 base_link is already calibrated to IMU/body; source clocks assumed aligned',
                scope='Local first-flight longitudinal phase only; not a full XYZ trajectory or expert control actions',
                sample='Existing 2 Hz, max 6000 points/frame cache; held-out alternating points are not independent scans',
                free_geometry_at_1s=dict(edge_m=free.x[0],floor_m=free.x[1],yaw_deg=np.degrees(free.x[2]),height_m=free.x[3],depth_m=free.x[4]),
                frames=rows)
    (OUT/'alignment.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    # Export measured joint states beside cloud-frame alignment. No inferred low-level actions.
    sdk=ROOT.parents[1]/'upstream/goai_embodied_future_material/src/S10_sdk_deploy/interface/robot/simulation/mujoco_simulation_ros2.py'
    values={}
    for node in ast.walk(ast.parse(sdk.read_text(encoding='utf-8'))):
        if isinstance(node,ast.Assign) and isinstance(node.targets[0],ast.Name) and node.targets[0].id in ('JOINT_DIR','POS_OFFSET_DEG'):
            values[node.targets[0].id]=np.asarray(ast.literal_eval(node.value.args[0]))
    jt=(data['JOINTS_DATA_src']-meta['manifest']['started_wall_ns'])/1e9
    j=np.array([np.argmin(abs(jt-r['t_s'])) for r in rows]);raw=data['JOINTS_DATA_v'][j]
    it=(data['IMU_src']-meta['manifest']['started_wall_ns'])/1e9
    ii=np.array([np.argmin(abs(it-r['t_s'])) for r in rows])
    np.savez_compressed(OUT/'reference_at_cloud_frames.npz',time_s=np.array([r['t_s'] for r in rows]),
                        joint_source_time_s=jt[j],joint_position_sdk_rad=raw[:,:16]*values['JOINT_DIR']+np.deg2rad(values['POS_OFFSET_DEG']),
                        joint_velocity_sdk_rad_s=raw[:,16:32]*values['JOINT_DIR'],
                        imu_orientation_xyzw=data['IMU_v'][ii,:4],
                        first_edge_normal_distance_m=[r['edge_from_base_m'] for r in rows],
                        usable=[r['alignment_usable'] and abs(jt[i]-r['t_s'])<.03 for r,i in zip(rows,j)])
    fig,axes=plt.subplots(2,1,figsize=(11,7),sharex=True)
    tt=np.array([r['t_s'] for r in rows]);edge=np.array([r['edge_from_base_m'] for r in rows])
    usable=np.array([r['alignment_usable'] for r in rows])
    axes[0].plot(tt,edge,':',color='gray',label='Candidate only (stair-index ambiguity after 11.5s)')
    axes[0].scatter(tt[usable],edge[usable],s=16,label='Approach fit passing geometric checks')
    axes[0].set(ylabel='Distance from base (m)');axes[0].legend()
    axes[1].plot(tt,[r['p90_m']*100 for r in rows],label='90th percentile surface residual (cm)')
    axes[1].axhline(3,color='gray',ls='--');axes[1].set(xlabel='Recording source time (s)',ylabel='Residual (cm)');axes[1].legend()
    for ax in axes:ax.grid(alpha=.2)
    fig.tight_layout();fig.savefig(OUT/'alignment.png',dpi=150)
    # A portable static USD of the first four visible risers; unmeasured width explicitly assumed.
    usd=['#usda 1.0','(\n metersPerUnit = 1\n upAxis = "Z"\n)','def Xform "StairPreview" {']
    for k in range(4):
        usd.extend([f' def Cube "Step{k+1}" (prepend apiSchemas = ["PhysicsCollisionAPI"]) {{','  double size = 1',
                    '  bool physics:collisionEnabled = true',
                    f'  double3 xformOp:translate = ({(k+.5)*.55}, 0, {(k+1)*.15/2})',
                    f'  double3 xformOp:scale = (.55, 1.2, {(k+1)*.15})',
                    '  uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]',' }'])
    usd.append('}');(OUT/'first_four_steps.usda').write_text('\n'.join(usd))
    fig,ax=plt.subplots(figsize=(11,5))
    ax.scatter(p[:,0]*np.cos(free.x[2])+p[:,1]*np.sin(free.x[2]),p[:,2],s=3,label='Leveled lidar at 0.95s')
    fixed=next(r for r in rows if abs(r['t_s']-c['src'])<.01)
    x=[.25];z=[fixed['floor_from_base_m']]
    for k in range(5):
        e=fixed['edge_from_base_m']+k*.55
        x.extend([e,e]);z.extend([fixed['floor_from_base_m']+k*.15,fixed['floor_from_base_m']+(k+1)*.15])
    ax.plot(x,z,color='darkorange',lw=2,label='Recorded 15cm / 55cm stairs')
    ax.set(xlim=(.25,3),ylim=(-.55,.6),xlabel='Distance along stair normal from base (m)',ylabel='Height relative to base (m)')
    ax.legend();ax.grid(alpha=.2);fig.tight_layout();fig.savefig(OUT/'fit_overlay.png',dpi=150)
    print(json.dumps(dict(free_geometry=result['free_geometry_at_1s'],first=rows[0],last=rows[-1]),indent=2))

if __name__=='__main__':main()
