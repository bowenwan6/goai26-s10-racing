"""Offline arrival-ordered S10 heightmap replay. All writes stay beside this file/output."""
import argparse
import ast
import csv
import json
import sqlite3
import sys
import time
import xml.etree.ElementTree as ET
from collections import defaultdict, deque
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
DEFAULT_REPO = Path(__file__).resolve().parents[4]   # <repo>/robot/tools/s10/_heightmap/_replay/
RAW = Path('D:/S10Data/050/2026-09-09')
NS = 10**9
CFG = dict(cell=.05, query_half=.065, min_points=3, max_spread=.045,
           layer_gap=.07, normal_z=.88, plane_rms=.022, normal_radius=.11,
           max_range=8., terrain_range=3., body_margin=.01, leg_radius=.037,
           wheel_radius=.081, wheel_halfwidth=.025, ttl=1.5, fresh=.30,
           imu_max_age=.04, joints_max_age=.06, pose_max_age=.25,
           icp_voxel=.12, icp_distance=.20, icp_p90=.065, icp_match=.35,
           icp_condition=80., icp_max_speed=2., conflict=.08,
           min_confidence=.3, max_pose_sigma=.06, drift_per_second=.02)
GRID = np.array([(x,y) for x in np.linspace(-.6,1.2,13)
                 for y in np.linspace(-.6,.6,9)])
REGIONS = {'all':np.ones(117,bool), 'ahead':GRID[:,0]>.45+1e-8,
           'front_wheels':(GRID[:,0]>=.15-1e-8)&(GRID[:,0]<=.45+1e-8),
           'rear_wheels':GRID[:,0]<-.15+1e-8,
           'body':(abs(GRID[:,0])<=.15+1e-8)&(abs(GRID[:,1])<=.3+1e-8),
           'left':GRID[:,1]>.3, 'right':GRID[:,1]<-.3}


def dump(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')


def pure_functions(path, names, namespace):
    """Reuse reviewed functions without executing their module or entry point."""
    tree = ast.parse(path.read_text(encoding='utf-8'))
    nodes = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in names]
    assert {n.name for n in nodes} == set(names)
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(path), 'exec'), namespace)
    return [namespace[n] for n in names]


def decoder(repo):
    # Existing offline installation; append after NumPy/SciPy to avoid replacing their ABI.
    sys.path.append(str(repo/'tmp/s10-analysis-deps'))
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore
    store = get_typestore(Stores.ROS2_JAZZY)
    definitions = {}
    for p in (repo/'tools/s10_gait_capture/vendor_ws/src/drdds/msg').glob('*.msg'):
        definitions.update(get_types_from_msg(p.read_text(), 'drdds/msg/'+p.stem))
    store.register(definitions)
    return store


def xyz_points(msg):
    """Vectorized version of recorder preview_points; no sampling or mm rounding."""
    fields = {f.name:f for f in msg.fields}
    fmt = {7:'f4',8:'f8'}
    if msg.row_step < msg.width*msg.point_step or len(msg.data) < msg.row_step*msg.height:
        raise ValueError('Truncated PointCloud2 rows')
    for name in 'xyz':
        f = fields.get(name)
        if f is None or f.datatype not in fmt or f.count != 1:
            raise ValueError('PointCloud2 requires scalar float x/y/z')
        if not 0 <= f.offset <= msg.point_step-np.dtype(fmt[f.datatype]).itemsize:
            raise ValueError('PointCloud2 field offset outside record')
    endian = '>' if msg.is_bigendian else '<'
    dtype = np.dtype(dict(names=list('xyz'), formats=[endian+fmt[fields[k].datatype] for k in 'xyz'],
                         offsets=[fields[k].offset for k in 'xyz'], itemsize=msg.point_step))
    a = np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,strides=(msg.row_step,msg.point_step))
    p = np.stack([a[k].ravel() for k in 'xyz'],axis=1)
    return p[np.isfinite(p).all(axis=1)]


def stats(x):
    x = np.asarray(x); x = x[np.isfinite(x)]
    return dict(zip(('min','median','p95','max'),map(float,np.percentile(x,[0,50,95,100])))) if len(x) else None


def decode(folder, out, end, repo):
    cache = out/'decoded.npz'
    if cache.exists():
        d = dict(np.load(cache))
        assert float(d['end_s']) == end and str(d['recording_id']) == folder.name
        return d
    manifest = json.loads((folder/'manifest.json').read_text(encoding='utf-8'))
    anchor = manifest['started_wall_ns']; store = decoder(repo)
    events=[]; clouds=[]; imu=[]; joints=[]; audit=defaultdict(list); layouts={}; names=set(); topics_seen={}
    for di,db in enumerate(sorted((folder/'bag').glob('*.db3'))):
        with sqlite3.connect(db.resolve().as_uri()+'?mode=ro',uri=True) as conn:
            topics = {i:(n,k) for i,n,k in conn.execute('SELECT id,name,type FROM topics')}
            topics_seen.update({n:k for n,k in topics.values()})
            # id is recorder insertion order; do not sort by source time.
            for mid,tid,rx,blob in conn.execute('SELECT id,topic_id,timestamp,data FROM messages ORDER BY id'):
                if rx > anchor+round(end*NS):
                    continue
                name,kind = topics[tid]
                if name not in ('/rslidar_front/points','/rslidar_rear/points','/IMU','/JOINTS_DATA'):
                    continue
                m = store.deserialize_cdr(blob,kind)
                src = int(m.header.stamp.sec)*NS+int(m.header.stamp.nanosec)
                audit[name].append((rx,src))
                if kind == 'sensor_msgs/msg/PointCloud2':
                    if m.header.frame_id != 'base_link':
                        raise ValueError('Unconfigured cloud frame: '+m.header.frame_id)
                    layout = dict(frame_id=m.header.frame_id,width=m.width,height=m.height,
                                  point_step=m.point_step,row_step=m.row_step,bigendian=m.is_bigendian,
                                  fields=[dict(name=f.name,offset=f.offset,datatype=f.datatype,count=f.count) for f in m.fields])
                    layouts.setdefault(name,layout)
                    p=xyz_points(m); r=np.linalg.norm(p,axis=1)
                    # Cache all finite local raw returns, including self; no preview input.
                    p=p[(r>.08)&(r<CFG['max_range'])].astype('f4')
                    ix=len(clouds);clouds.append(p);code=0 if 'front' in name else 1
                elif name == '/IMU':
                    ix=len(imu);code=2
                    imu.append([*[getattr(m.orientation,k) for k in 'xyzw'],
                                *[getattr(m.angular_velocity,k) for k in 'xyz'],
                                *[getattr(m.linear_acceleration,k) for k in 'xyz']])
                else:
                    ix=len(joints);code=3
                    js=m.data.joints_data
                    names.add(tuple(bytes(j.name).split(b'\0')[0].decode() for j in js))
                    joints.append([float(getattr(j,k)) for k in ('position','velocity','torque') for j in js])
                events.append((rx,src,code,ix,di,mid))
    e=np.array(events,dtype='i8')
    # Bag insertion order is authoritative. If receive clock regresses, the scheduler uses a
    # running maximum without reordering messages; both original rx and effective arrival survive.
    arrival=np.maximum.accumulate(e[:,0])
    report=dict(recording_id=folder.name,anchor_ns=str(anchor),end_receive_s=end,
                topics=topics_seen,layouts=layouts,joint_name_orders=[list(x) for x in names],
                receive_regressions=int(np.sum(np.diff(e[:,0])<0)),streams={})
    for name, rows in audit.items():
        rx,src=np.array(rows,dtype='i8').T
        report['streams'][name]=dict(count=len(rx),source_s=stats((src-anchor)/NS),
            receive_s=stats((rx-anchor)/NS),rx_minus_src_s=stats((rx-src)/NS),
            source_dt_s=stats(np.diff(src)/NS),receive_dt_s=stats(np.diff(rx)/NS),
            nonincreasing_source=int(np.sum(np.diff(src)<=0)),missing_source=int(np.sum(src<=0)))
    d=dict(events=e,arrival_ns=arrival,points=np.concatenate(clouds),
           offsets=np.r_[0,np.cumsum([len(p) for p in clouds])],imu=np.array(imu),joints=np.array(joints),
           anchor_ns=np.int64(anchor),recording_id=folder.name,end_s=end)
    np.savez_compressed(cache,**d); dump(out/'audit.json',report)
    print('DECODE',folder.name,len(e),'events',len(clouds),'clouds',flush=True)
    return d


class Kinematics:
    """SDK body/joint transforms only: no dynamics, controls, or model modification."""
    def __init__(self,repo):
        sdk=repo/'upstream/goai_embodied_future_material/src/S10_sdk_deploy'
        tree=ast.parse((sdk/'interface/robot/simulation/mujoco_simulation_ros2.py').read_text(encoding='utf-8'))
        values={}
        for node in tree.body:
            if isinstance(node,ast.Assign) and isinstance(node.targets[0],ast.Name):
                key=node.targets[0].id
                if key in ('JOINT_DIR','POS_OFFSET_DEG'):
                    values[key]=np.asarray(ast.literal_eval(node.value.args[0]),dtype=float)
        self.direction=values['JOINT_DIR'];self.offset=np.deg2rad(values['POS_OFFSET_DEG'])
        self.root=ET.parse(sdk/'S10_description/s10_mjcf/mjcf/S10.xml').getroot().find("worldbody/body[@name='base_link']")
        self.joint_names=[n.attrib['name'] for n in self.root.iter('joint')]
        assert len(self.joint_names)==16

    def forward(self,raw):
        q=dict(zip(self.joint_names,raw[:16]*self.direction+self.offset));positions={};rotations={}
        def walk(node,pos,rot):
            if node is not self.root:
                offset=np.fromstring(node.get('pos','0 0 0'),sep=' ')
                pos=pos+rot@offset
                quat=np.fromstring(node.get('quat','1 0 0 0'),sep=' ')
                rot=rot@Rotation.from_quat(quat[[1,2,3,0]]).as_matrix()
                j=node.find('joint')
                if j is not None:
                    assert j.get('pos','0 0 0')=='0 0 0'
                    axis=np.fromstring(j.get('axis'),sep=' ')
                    rot=rot@Rotation.from_rotvec(axis*q[j.get('name')]).as_matrix()
            positions[node.get('name')]=pos;rotations[node.get('name')]=rot
            for child in node.findall('body'):walk(child,pos,rot)
        walk(self.root,np.zeros(3),np.eye(3))
        return positions,rotations

    def filter(self,p,raw):
        keep=~((abs(p[:,0])<.31)&(abs(p[:,1])<.114)&(abs(p[:,2])<.09))
        if raw is not None:
            pos,rot=self.forward(raw)
            for prefix in ('fl','fr','hl','hr'):
                for a,b in (('hipx','hipy'),('hipy','knee'),('knee','wheel')):
                    v=pos[prefix+'_'+a];delta=pos[prefix+'_'+b]-v
                    f=np.clip((p-v)@delta/(delta@delta),0,1)
                    keep &= np.linalg.norm(p-v-f[:,None]*delta,axis=1)>CFG['leg_radius']
                w=prefix+'_wheel';local=(p-pos[w])@rot[w]
                # No inflated sphere: avoid erasing the ground beneath a wheel.
                keep &= ~((local[:,0]**2+local[:,2]**2 < CFG['wheel_radius']**2)&(abs(local[:,1])<CFG['wheel_halfwidth']))
        else:
            # Unknown leg pose: uncertain leg envelope, not a 65 cm radial crop.
            keep &= ~((abs(p[:,0])<.45)&(abs(p[:,1])>.10)&(abs(p[:,1])<.32)&(p[:,2]>-.4)&(p[:,2]<.1))
        p=p[keep]
        if len(p)>8:
            dist,_=cKDTree(p).query(p,k=4,workers=1)
            p=p[dist[:,-1]<.20]
        return p


def voxel(p,size):
    return p[np.unique(np.floor(p/size).astype('i4'),axis=0,return_index=True)[1]] if len(p) else p


def surfaces(p):
    """5 cm cells. Reject multi-layer cells and unsupported/vertical local patches."""
    p=p[(np.linalg.norm(p[:,:2],axis=1)<CFG['terrain_range'])&(p[:,2]>-1.5)&(p[:,2]<.6)]
    if len(p)<8:return np.empty((0,3)),np.empty(0),np.empty((0,2),int)
    keys=np.floor(p[:,:2]/CFG['cell']).astype(int)
    unique,inv,counts=np.unique(keys,axis=0,return_inverse=True,return_counts=True)
    order=np.argsort(inv,kind='stable');starts=np.r_[0,np.cumsum(counts)]
    candidates=[];confidence=[];ambiguous=[]
    tree=cKDTree(p[:,:2])
    for i,key in enumerate(unique):
        cell=p[order[starts[i]:starts[i+1]]]
        if counts[i]<CFG['min_points']:continue
        zs=np.sort(cell[:,2]);spread=np.percentile(zs,90)-np.percentile(zs,10)
        if spread>CFG['max_spread'] or np.max(np.diff(zs),initial=0)>CFG['layer_gap']:
            ambiguous.append(key);continue
        c=np.median(cell,axis=0)
        ids=tree.query_ball_point(c[:2],CFG['normal_radius'])
        near=p[ids];near=near[abs(near[:,2]-c[2])<CFG['layer_gap']]
        if len(near)<6:continue
        centered=near-near.mean(axis=0)
        vals,vec=np.linalg.eigh(centered.T@centered/len(near))
        # At least a small 2D patch; a single scan line cannot establish a surface.
        if vals[1]<.00008 or abs(vec[2,0])<CFG['normal_z'] or np.sqrt(max(vals[0],0))>CFG['plane_rms']:
            continue
        candidates.append(c);confidence.append(min(1.,counts[i]/8)*max(.5,1-spread/.09))
    return np.asarray(candidates).reshape(-1,3),np.asarray(confidence),np.asarray(ambiguous,dtype=int).reshape(-1,2)


class HeightMap:
    def __init__(self):
        self.cells={};self.conflicts=0;self.revisits=[]

    def update(self,points,confidence,src,rx,sensor,frame,pose_sigma):
        keys=np.floor(points[:,:2]/CFG['cell']).astype(int)
        self.conflicts=0;self.revisits=[]
        for p,c,key in zip(points,confidence,keys):
            k=tuple(key);old=self.cells.get(k)
            if old is not None and src<old[3]:continue
            if old is not None and (src-old[3])/NS<CFG['ttl']:
                delta=abs(p[2]-old[0]);self.revisits.append(delta)
                if delta>CFG['conflict']:
                    self.conflicts+=1
                    # Invalidate on disagreement. A second consistent new measurement may restore it.
                    self.cells[k]=(p[2],0.,pose_sigma,src,rx,sensor,frame)
                    continue
            self.cells[k]=(p[2],float(c),pose_sigma,src,rx,sensor,frame)

    def invalidate(self,xy,src,rx,sensor,frame):
        for key in np.floor(xy/CFG['cell']).astype(int):
            k=tuple(key);old=self.cells.get(k)
            if old is None or old[3]<=src:
                self.cells[k]=(0.,0.,0.,src,rx,sensor,frame)

    def query(self,pose,yaw,now,pose_sigma=0.,grid=GRID):
        rz=Rotation.from_euler('z',yaw).as_matrix()[:2,:2]
        xy=grid@rz.T+pose[:2]
        H=np.zeros(len(grid),'f4');V=np.zeros(len(grid),bool);age=np.full(len(grid),np.inf,'f4')
        C=np.zeros(len(grid),'f4');S=np.full(len(grid),-1,'i1');T=np.zeros(len(grid),'i8');R=T.copy();F=np.full(len(grid),-1,'i4')
        active=[(k,v) for k,v in self.cells.items() if 0<=now-v[3]<=round(CFG['ttl']*NS) and v[4]<=now]
        if active:
            centers=(np.array([k for k,v in active])+.5)*CFG['cell']
            data=np.asarray([v for k,v in active],dtype=object)
            neighborhoods=cKDTree(centers).query_ball_point(xy,CFG['query_half'],p=np.inf)
        else:neighborhoods=[[] for _ in grid]
        for j,ids in enumerate(neighborhoods):
            if not ids:continue
            rows=data[ids]
            if np.any(rows[:,1]==0):continue  # Current contradictory/multi-layer evidence blocks memory.
            ages=(now-rows[:,3].astype('i8'))/NS
            ok=(rows[:,1].astype(float)>=CFG['min_confidence'])&(rows[:,2].astype(float)+pose_sigma+CFG['drift_per_second']*ages<=CFG['max_pose_sigma'])
            rows=rows[ok]
            if not len(rows):continue
            heights=rows[:,0].astype(float)
            if np.ptp(heights)>CFG['conflict']:continue
            # Closest-to-median measured surface, never average two levels.
            picked=rows[np.argmin(abs(heights-np.median(heights)))]
            H[j]=np.clip(float(picked[0])-pose[2],-1,1);V[j]=True
            # Conservative oldest contributing support, so a new neighbour cannot rejuvenate memory.
            T[j]=min(rows[:,3]);R[j]=max(rows[:,4]);age[j]=(now-T[j])/NS
            C[j]=min(rows[:,1]);S[j]=int(picked[5]);F[j]=int(picked[6])
        return dict(H=H,V=V,age=age,confidence=C,sensor=S,measurement_ns=T,arrival_ns=R,frame_id=F)

    def prune(self,now,pos):
        self.cells={k:v for k,v in self.cells.items() if (now-v[3])/NS<=CFG['ttl'] and np.linalg.norm((np.array(k)+.5)*CFG['cell']-pos[:2])<4}


def translation_icp(source,guess,ref):
    """Existing short-window point-to-plane method with IMU rotation held fixed."""
    p,tree,normals,good=ref;src=voxel(source,CFG['icp_voxel']);pos=guess.copy()
    for iteration in range(12):
        moved=src+pos;dist,idx=tree.query(moved,workers=1)
        keep=(dist<(CFG['icp_distance'] if iteration>2 else .30))&good[idx]
        if np.sum(keep)<100:raise ValueError('insufficient_pairs')
        n=normals[idx[keep]];res=np.sum((moved[keep]-p[idx[keep]])*n,axis=1)
        weight=np.minimum(1.,.025/np.maximum(abs(res),1e-9))
        A=n*np.sqrt(weight[:,None]);b=-res*np.sqrt(weight)
        eig=np.linalg.eigvalsh(A.T@A);condition=eig[-1]/max(eig[0],1e-9)
        step=np.linalg.lstsq(A,b,rcond=None)[0];pos+=step
        if np.linalg.norm(step)<1e-4:break
    # Evaluate the returned pose, not only the pre-step residual.
    moved=src+pos;dist,idx=tree.query(moved,workers=1);keep=(dist<CFG['icp_distance'])&good[idx]
    res=np.sum((moved[keep]-p[idx[keep]])*normals[idx[keep]],axis=1)
    q=dict(p90=float(np.percentile(abs(res),90)),median=float(np.median(abs(res))),
           matched=float(np.mean(dist<CFG['icp_distance'])),plane_fraction=float(np.mean(keep)),
           pairs=int(np.sum(keep)),condition=float(condition))
    return pos,q


def eligible(history,src,now,max_age):
    # History contains arrived messages only; no Slerp and no closest future sample.
    for stamp,rx,value in reversed(history):
        if stamp<=src and rx<=now:
            return (value,stamp,rx) if (src-stamp)/NS<=max_age else (None,stamp,rx)
    return None,0,0


def process(d,out,repo):
    start_wall=time.perf_counter();kin=Kinematics(repo)
    target,=pure_functions(repo/'artifacts/evidence/runs/s10-recording-review-20260909/reconstruct_stairs.py',
        ['target'],dict(np=np,cKDTree=cKDTree,voxel=voxel))
    events=d['events'];anchor=int(d['anchor_ns']);arrival=d['arrival_ns'];end=float(d['end_s'])
    imu=deque(maxlen=600);joints=deque(maxlen=600);maps=HeightMap();recent=deque(maxlen=8)
    history=deque(maxlen=6);single=[None,None];yaw0=None;pose=np.zeros(3);ref=None;segment=0
    latest_src=0;last_quality=False;last_q={};sigma=.0;frame=-1;frame_rows=[];seen_sensors=set()
    outputs=defaultdict(list);ei=0;cloud_results={};last_raw=[None,None];last_progress=-10
    for tick,now in enumerate(range(anchor,anchor+round(end*NS)+1,20_000_000)):
        while ei<len(events) and arrival[ei]<=now:
            rx,src,code,ix,db,mid=map(int,events[ei]);effective=int(arrival[ei]);ei+=1
            if code==2:
                val=d['imu'][ix]
                if np.isfinite(val).all() and .9<np.linalg.norm(val[:4])<1.1:
                    imu.append((src,effective,val))
                    if yaw0 is None:yaw0=Rotation.from_quat(val[:4]).as_euler('xyz')[2]
                continue
            if code==3:
                joints.append((src,effective,d['joints'][ix]));continue
            frame+=1;beg,stop=d['offsets'][ix:ix+2];raw=d['points'][beg:stop]
            attitude,its,irx=eligible(imu,src,effective,CFG['imu_max_age'])
            jv,jts,jrx=eligible(joints,src,effective,CFG['joints_max_age'])
            row=dict(frame=frame,sensor=code,source_ns=src,receive_ns=rx,arrival_ns=effective,
                     source_s=(src-anchor)/NS,receive_s=(rx-anchor)/NS,imu_source_ns=its,imu_receive_ns=irx,
                     joint_source_ns=jts,joint_receive_ns=jrx,raw_points=len(raw),reason='',segment=segment)
            if attitude is None:
                row.update(reason='imu_unavailable',accepted=False);frame_rows.append(row)
                maps.cells.clear();recent.clear();ref=None;last_quality=False;single=[None,None];seen_sensors.clear()
                continue
            rot=Rotation.from_euler('z',-yaw0)*Rotation.from_quat(attitude[:4])
            yaw=rot.as_euler('xyz')[2];p=kin.filter(raw,jv);leveled=rot.apply(p)
            surface,conf,ambig=surfaces(leveled)
            localmap=HeightMap();localmap.update(surface,conf,src,effective,code,frame,0)
            localmap.invalidate((ambig+.5)*CFG['cell'],src,effective,code,frame)
            # For single-frame query, the measurement has arrived at effective. Age remains rx-src.
            sq=localmap.query(np.zeros(3),yaw,effective)
            last_raw[code]=dict(raw=raw[::max(1,len(raw)//5500)],rot=rot.as_matrix(),src=src,rx=effective)
            single[code]=dict(map=localmap,pose=None,rot=rot,yaw=yaw,src=src,rx=effective,frame=frame)
            accepted=False;reason='';q=dict(p90=np.nan,median=np.nan,matched=np.nan,condition=np.nan,pairs=0)
            old_pose=pose.copy();_old_src=latest_src;guess=pose.copy()
            if len(history)>=2:
                dt=(history[-1][0]-history[0][0])/NS
                if dt>.05:
                    velocity=(history[-1][1]-history[0][1])/dt
                    guess+=velocity*np.clip((src-latest_src)/NS,-.05,.20)
            if ref is None:
                reason='bootstrap';pose=old_pose;segment+=1;history.clear();sigma=0.
            elif code not in seen_sensors and abs(src-latest_src)<25_000_000:
                # Two near-simultaneous sensors can initialize the same local gauge.
                # No translation quality is asserted until a later scan registers.
                reason='bootstrap_companion'
            else:
                try:
                    found,q=translation_icp(leveled,guess,ref)
                    dt=max(.04,abs(src-latest_src)/NS)
                    if q['condition']>CFG['icp_condition']:reason='degenerate'
                    elif q['p90']>CFG['icp_p90']:reason='residual'
                    elif q['matched']<CFG['icp_match']:reason='overlap'
                    elif np.linalg.norm(found-pose)>CFG['icp_max_speed']*dt+.035:reason='jump'
                    else:pose=found;accepted=True;sigma=min(.025,.004+q['p90']*.1)
                except ValueError as exc:reason=str(exc)
            if not accepted and reason not in ('bootstrap','bootstrap_companion'):
                # Failure severs memory/trajectory trust immediately. Bootstrap a fresh segment.
                maps.cells.clear();recent.clear();history.clear();seen_sensors.clear();segment+=1;sigma=0.;pose=old_pose
                single[1-code]=None
            seen_sensors.add(code)
            latest_src=src;last_quality=accepted;last_q=q
            if not history or src-history[-1][0]>60_000_000:history.append((src,pose.copy()))
            single[code]['pose']=pose.copy();single[code]['segment']=segment
            world=leveled+pose;recent.append(voxel(world,.10))
            ref=target(np.vstack(recent))
            # Fresh segment is a local gauge. Its single frame is usable; memory needs the next success.
            maps.update(surface+pose,conf,src,effective,code,frame,sigma)
            maps.invalidate((ambig+.5)*CFG['cell']+pose[:2],src,effective,code,frame)
            maps.prune(effective,pose)
            row.update(accepted=accepted,reason=reason,segment=segment,p90=q['p90'],matched=q['matched'],
                condition=q['condition'],plane_fraction=q.get('plane_fraction',np.nan),pairs=q['pairs'],px=pose[0],py=pose[1],pz=pose[2],
                roll_deg=rot.as_euler('xyz',degrees=True)[0],pitch_deg=rot.as_euler('xyz',degrees=True)[1],
                kept_points=len(p),surface_cells=len(surface),ambiguous_cells=len(ambig),
                single_coverage=float(sq['V'].mean()),conflicts=maps.conflicts,
                revisit_p90=float(np.percentile(maps.revisits,90)) if maps.revisits else np.nan)
            frame_rows.append(row)
            if reason and frame<12:print('REGISTER',frame,reason,q,flush=True)
            cloud_results[frame]=dict(H=sq['H'],V=sq['V'],age=sq['age'],surface=surface.astype('f4'),
                raw=last_raw[code]['raw'],rot=rot.as_matrix(),pose=pose.copy())
        attitude,its,irx=eligible(imu,now,now,CFG['pose_max_age'])
        jv,jts,jrx=eligible(joints,now,now,CFG['pose_max_age'])
        query_pose=pose.copy();pose_age=(now-latest_src)/NS if latest_src else np.inf
        extrap=np.zeros(3)
        if last_quality and len(history)>=2:
            dt=(history[-1][0]-history[0][0])/NS
            if dt>.05:extrap=(history[-1][1]-history[0][1])/dt*np.clip(pose_age,0,CFG['pose_max_age'])
        query_pose+=extrap
        query_sigma=sigma+np.linalg.norm(extrap)*.15
        valid_pose=attitude is not None and last_quality and 0<=pose_age<=CFG['pose_max_age'] and query_sigma<=CFG['max_pose_sigma']
        rot=Rotation.identity() if attitude is None else Rotation.from_euler('z',-(yaw0 or 0))*Rotation.from_quat(attitude[:4])
        yaw=rot.as_euler('xyz')[2]
        result=maps.query(query_pose,yaw,now,query_sigma) if valid_pose else HeightMap().query(query_pose,yaw,now)
        views=[]
        for s in single:
            # Current front/rear views are distinct last individual scans, each moved by its source pose.
            if s and now-s['src']<=round(CFG['fresh']*NS) and s.get('segment')==segment and s['pose'] is not None:
                local_pose=query_pose-s['pose'] if valid_pose else np.zeros(3)
                views.append(s['map'].query(local_pose,yaw if valid_pose else s['yaw'],now))
            else:views.append(HeightMap().query(query_pose,yaw,now))
        fusion=HeightMap()
        for s in single:
            if s and s.get('segment')==segment and s['pose'] is not None and now-s['src']<=round(CFG['fresh']*NS):
                for k,v in s['map'].cells.items():
                    point=np.r_[(np.array(k)+.5)*CFG['cell'],v[0]]
                    point+=s['pose']-query_pose if valid_pose else 0
                    fusion.update(point[None,:],np.array([v[1]]),v[3],v[4],v[5],v[6],0)
        fq=fusion.query(np.zeros(3),yaw,now)
        for key,value in result.items():outputs[key].append(value)
        outputs['time_ns'].append(now);outputs['pose'].append(np.r_[query_pose,rot.as_quat()])
        outputs['pose_valid'].append(valid_pose);outputs['pose_age'].append(pose_age);outputs['pose_sigma'].append(query_sigma)
        outputs['segment'].append(segment);outputs['imu_source_ns'].append(its);outputs['imu_arrival_ns'].append(irx)
        outputs['joint_source_ns'].append(jts);outputs['joint_arrival_ns'].append(jrx)
        outputs['joint_state'].append(np.full(48,np.nan) if jv is None else jv)
        outputs['front_H'].append(views[0]['H']);outputs['front_V'].append(views[0]['V'])
        outputs['rear_H'].append(views[1]['H']);outputs['rear_V'].append(views[1]['V'])
        outputs['single_H'].append(fq['H']);outputs['single_V'].append(fq['V'])
        outputs['single_in_current_frame'].append(valid_pose)
        outputs['icp_p90'].append(last_q.get('p90',np.nan));outputs['icp_matched'].append(last_q.get('matched',np.nan))
        outputs['latest_cloud_source_ns'].append(latest_src)
        outputs['latest_front_frame'].append(-1 if single[0] is None else single[0]['frame'])
        outputs['latest_rear_frame'].append(-1 if single[1] is None else single[1]['frame'])
        if (now-anchor)/NS-last_progress>=2:
            last_progress=(now-anchor)/NS
            print('REPLAY',round(last_progress,2),'coverage',round(float(result['V'].mean()),3),
                  'segment',segment,'xyz',query_pose.round(3),flush=True)
    arrays={k:np.asarray(v) for k,v in outputs.items()}
    arrays.update(anchor_ns=np.int64(anchor),grid_xy=GRID,config_json=json.dumps(CFG),recording_id=d['recording_id'])
    np.savez_compressed(out/'observations.npz',**arrays)
    fields=list(dict.fromkeys(k for row in frame_rows for k in row))
    with (out/'frames.csv').open('w',newline='',encoding='utf-8') as f:
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader();writer.writerows(frame_rows)
    cloud_data={}
    for f,r in cloud_results.items():
        for k,v in r.items():cloud_data[f'{k}_{f}']=v
    np.savez_compressed(out/'single_frames.npz',**cloud_data)
    metrics(arrays,out)
    dump(out/'run.json',dict(config=CFG,recording_id=str(d['recording_id']),end_receive_s=end,
         replay_wall_s=time.perf_counter()-start_wall,frames=len(frame_rows),
         accepted=sum(bool(r['accepted']) for r in frame_rows),segments=segment,
         failure_reasons={r:sum(v['reason']==r for v in frame_rows) for r in sorted({v['reason'] for v in frame_rows})},
         output_samples=len(arrays['time_ns']),mean_coverage=float(arrays['V'].mean())))
    return arrays


def metrics(a,out):
    rows=[];previous=np.zeros(117,'i8');last_update=np.zeros(117,'i8');previous_frames=set()
    for i,now in enumerate(a['time_ns']):
        row=dict(receive_s=(int(now)-int(a['anchor_ns']))/NS,time_ns=int(now),
                 cloud_source_s=(int(a['latest_cloud_source_ns'][i])-int(a['anchor_ns']))/NS,
                 pose_valid=int(a['pose_valid'][i]),segment=int(a['segment'][i]),
                 icp_p90=a['icp_p90'][i],icp_matched=a['icp_matched'][i])
        changed=a['V'][i]&(a['measurement_ns'][i]!=previous)
        latest={int(a[k][i]) for k in ('latest_front_frame','latest_rear_frame') if a[k][i]>=0}
        new_frames=latest-previous_frames;previous_frames=latest
        # Moving across remembered cells is a query support change, not a new lidar measurement.
        measured=changed&np.isin(a['frame_id'][i],list(new_frames))
        intervals=(now-last_update[measured & (last_update>0)])/NS
        row['support_changed_cells']=int(changed.sum())
        row['new_latest_lidar_frames']=len(new_frames)
        row['new_measurement_cells']=int(measured.sum())
        row['update_interval_median_s']=float(np.median(intervals)) if len(intervals) else np.nan
        last_update[measured]=now;previous[changed]=a['measurement_ns'][i,changed]
        for name,mask in REGIONS.items():
            good=a['V'][i]&mask;ages=a['age'][i,good]
            row[name+'_coverage']=float(a['V'][i,mask].mean())
            row[name+'_single']=float(a['single_V'][i,mask].mean())
            row[name+'_age_p95']=float(np.percentile(ages,95)) if len(ages) else np.nan
            row[name+'_memory']=float((a['V'][i,mask]&(a['age'][i,mask]>CFG['fresh'])).mean())
        rows.append(row)
    with (out/'metrics.csv').open('w',newline='',encoding='utf-8') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)


def plotting():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.size':9,'axes.grid':False,'figure.facecolor':'white'})
    return plt


def grid_image(ax,values,valid,title,kind='height'):
    from matplotlib.colors import ListedColormap
    if kind=='valid':
        cmap=ListedColormap(['#a8a8a8','#189c71']);vmin,vmax=0,1;data=values
    elif kind=='state':
        cmap=ListedColormap(['#a8a8a8','#24a87c','#dc942c']);vmin,vmax=0,2;data=values
    else:
        plt=plotting();cmap=plt.get_cmap('viridis' if kind=='age' else 'coolwarm').copy();cmap.set_bad('#a8a8a8')
        vmin,vmax=(0,CFG['ttl']) if kind=='age' else (-.65,.25)
        data=np.ma.array(values,mask=~valid)
    im=ax.imshow(np.asarray(data).reshape(13,9).T if kind in ('valid','state') else data.reshape(13,9).T,
        origin='lower',extent=(-.675,1.275,-.675,.675),aspect='equal',cmap=cmap,vmin=vmin,vmax=vmax,
        interpolation='nearest')
    ax.set(title=title,xlabel='+X forward (m)',ylabel='+Y left (m)')
    ax.plot([0,.25],[0,0],'k-',lw=1);ax.plot(.25,0,'k>',ms=4)
    return im


def stage_a(d,out,repo):
    plt=plotting();kin=Kinematics(repo);anchor=int(d['anchor_ns'])
    ih=deque(maxlen=600);jh=deque(maxlen=600);latest=[None,None];yaw0=None;done=set();saved={};records=[]
    targets=[1,8,9,10,11,12,13,14,15]
    for ei,(rx,src,code,ix,db,mid) in enumerate(d['events']):
        rx,src,code,ix=map(int,(rx,src,code,ix));arr=int(d['arrival_ns'][ei])
        if code==2:
            v=d['imu'][ix];ih.append((src,arr,v))
            if yaw0 is None:yaw0=Rotation.from_quat(v[:4]).as_euler('xyz')[2]
            continue
        if code==3:jh.append((src,arr,d['joints'][ix]));continue
        t=(src-anchor)/NS
        wanted=next((x for x in targets if x not in done and x<=t<x+.16),None)
        if wanted is None:continue
        att,its,irx=eligible(ih,src,arr,CFG['imu_max_age']);jv,jts,jrx=eligible(jh,src,arr,CFG['joints_max_age'])
        if att is None:continue
        rot=Rotation.from_euler('z',-yaw0)*Rotation.from_quat(att[:4]);yaw=rot.as_euler('xyz')[2]
        lo,hi=d['offsets'][ix:ix+2];raw=d['points'][lo:hi];p=rot.apply(kin.filter(raw,jv))
        latest[code]=(src,arr,rot.apply(raw),p,rot,jv)
        if any(x is None for x in latest) or abs(latest[0][0]-latest[1][0])>25_000_000:continue
        fig,axes=plt.subplots(3,3,figsize=(13,10));result={}
        for col,(name,pts) in enumerate([('Front',latest[0][3]),('Rear',latest[1][3]),('Fusion',np.vstack([x[3] for x in latest]))]):
            surf,conf,amb=surfaces(pts);hm=HeightMap();hm.update(surf,conf,min(x[0] for x in latest),arr,col,0,0)
            hm.invalidate((amb+.5)*CFG['cell'],min(x[0] for x in latest),arr,col,0)
            q=hm.query(np.zeros(3),yaw,arr)
            pr=latest[col][2] if col<2 else np.vstack([x[2] for x in latest])
            pr=pr[(abs(pr[:,0])<2)&(abs(pr[:,1])<1.2)&(abs(pr[:,2])<1.2)]
            axes[0,col].scatter(pr[:,0],pr[:,2],s=.35,c='#acacac',rasterized=True)
            axes[0,col].scatter(surf[:,0],surf[:,2],s=2,c=surf[:,2],cmap='coolwarm',vmin=-.65,vmax=.25)
            axes[0,col].set(xlim=(-1,2),ylim=(-1,.5),xlabel='X forward',ylabel='gravity Z',title=name+' raw gray / accepted surfaces colored')
            axes[0,col].axhline(0,color='k',lw=.5)
            axes[1,col].scatter(pr[:,0],pr[:,1],c=pr[:,2],s=.35,cmap='coolwarm',vmin=-.65,vmax=.25)
            axes[1,col].set(xlim=(-.8,1.8),ylim=(-.9,.9),xlabel='X',ylabel='Y left',aspect='equal',title='Raw top view')
            _im=grid_image(axes[2,col],q['H'],q['V'],f'{name} H: V={q["V"].mean():.0%}')
            result[name]=float(q['V'].mean())
            for k in ('H','V','age'):saved[f'{wanted}_{name}_{k}']=q[k]
        fig.suptitle(f'{d["recording_id"]} | src {t:.3f}s, arrival {(arr-anchor)/NS:.3f}s\n'
                     f'IMU roll/pitch {rot.as_euler("xyz",degrees=True)[:2].round(1)} deg; gray = unknown; header-start rigid scan')
        fig.tight_layout(rect=(0,0,1,.94));fig.savefig(out/f'A_{wanted:02d}s.png',dpi=125);plt.close(fig)
        records.append(dict(target_s=wanted,source_s=t,arrival_s=(arr-anchor)/NS,coverage=result,
                            source_pair_delta_ms=abs(latest[0][0]-latest[1][0])/1e6))
        done.add(wanted);print('A',t,result,flush=True)
    np.savez_compressed(out/'stage_A.npz',**saved);dump(out/'stage_A.json',records)


def render(a,out,repo,windows):
    import shutil
    import subprocess

    from matplotlib.animation import FFMpegWriter
    plt=plotting();kin=Kinematics(repo);frames=np.load(out/'single_frames.npz')
    times=(a['time_ns']-a['anchor_ns'])/NS
    # Support provenance distinguishes latest scans from remembered surface even between lidar arrivals.
    with (out/'frames.csv').open(encoding='utf-8') as stream:
        source_by_frame={int(r['frame']):int(r['source_ns']) for r in csv.DictReader(stream)}
    front_src=np.array([source_by_frame.get(int(f),0) for f in a['latest_front_frame']],dtype='i8')
    rear_src=np.array([source_by_frame.get(int(f),0) for f in a['latest_rear_frame']],dtype='i8')
    current=(a['measurement_ns']==front_src[:,None])|(a['measurement_ns']==rear_src[:,None])
    current &= a['V']
    state=np.where(a['V'],np.where(current,1,2),0)
    a['state']=state.astype('u1')
    np.savez_compressed(out/'observations.npz',**a)
    for begin,end in [tuple(map(float,w.split(':'))) for w in windows.split(',') if w.strip()]:
        # Include arrival latency so the last requested source frames can actually appear.
        indices=np.flatnonzero((times>=begin)&(times<=min(end+.25,times[-1])))
        indices=indices[::5]  # Video 10 Hz; NPZ queries are 50 Hz.
        fig,axs=plt.subplots(2,4,figsize=(16,8),dpi=90)
        video=out/f'replay_{begin:g}_{end:g}.mp4'
        writer=FFMpegWriter(fps=10,codec='libx264',extra_args=['-preset','veryfast','-crf','22','-pix_fmt','yuv420p','-movflags','+faststart'])
        with writer.saving(fig,str(video),90):
            for n,i in enumerate(indices):
                for ax in axs.ravel():ax.clear()
                rot=Rotation.from_quat(a['pose'][i,3:]);yaw=rot.as_euler('xyz')[2]
                for sensor,color,label in ((0,'#067eb5','front'),(1,'#e38318','rear')):
                    f=int(a['latest_front_frame' if sensor==0 else 'latest_rear_frame'][i])
                    if f<0 or f'raw_{f}' not in frames:continue
                    raw=frames[f'raw_{f}'];r=frames[f'rot_{f}'];p=raw@r.T
                    p=p+frames[f'pose_{f}']-a['pose'][i,:3]
                    p=p@Rotation.from_euler('z',-yaw).as_matrix().T
                    p=p[(abs(p[:,0])<2)&(abs(p[:,1])<1)&(abs(p[:,2])<1.2)]
                    axs[0,0].scatter(p[:,0],p[:,2],s=.6,c=color,label=label,alpha=.65)
                    axs[0,1].scatter(p[:,0],p[:,1],s=.6,c=color,alpha=.65)
                jv=a['joint_state'][i]
                if np.isfinite(jv).all():
                    pos,_=kin.forward(jv);r=Rotation.from_euler('z',-yaw)*rot
                    for leg in ('fl','fr','hl','hr'):
                        p=r.apply(np.array([pos[leg+'_'+x] for x in ('hipx','hipy','knee','wheel')]))
                        axs[0,0].plot(p[:,0],p[:,2],'k.-',lw=2,ms=3)
                        axs[0,0].add_patch(plt.Circle((p[-1,0],p[-1,2]),.081,fill=False,color='k',lw=1))
                        axs[0,1].plot(p[:,0],p[:,1],'k.-',lw=1,ms=2)
                axs[0,0].set(xlim=(-.9,1.8),ylim=(-1,.6),xlabel='X forward (m)',ylabel='gravity Z - base Z (m)',title='Raw returns + recorded joint FK')
                axs[0,0].legend(loc='upper right',fontsize=7);axs[0,0].grid(alpha=.2)
                axs[0,1].set(xlim=(-.7,1.3),ylim=(-.7,.7),aspect='equal',xlabel='X forward',ylabel='Y left',title='Dual raw clouds, top view')
                grid_image(axs[0,2],a['front_H'][i],a['front_V'][i],f'Front latest: {a["front_V"][i].mean():.0%}')
                grid_image(axs[0,3],a['rear_H'][i],a['rear_V'][i],f'Rear latest: {a["rear_V"][i].mean():.0%}')
                grid_image(axs[1,0],a['single_H'][i],a['single_V'][i],f'Latest fusion H: {a["single_V"][i].mean():.0%}')
                grid_image(axs[1,1],a['H'][i],a['V'][i],f'Rolling H: {a["V"][i].mean():.0%}; range [-.65,.25] m')
                grid_image(axs[1,2],state[i],a['V'][i],'V/state: gray unknown; green current; orange memory','state')
                grid_image(axs[1,3],a['age'][i],a['V'][i],f'Age 0-{CFG["ttl"]:g}s; gray invalid','age')
                source=(int(a['latest_cloud_source_ns'][i])-int(a['anchor_ns']))/NS
                fig.suptitle(f'{a["recording_id"]} | receive {times[i]:.2f}s | latest cloud source {source:.3f}s\n'
                    f'roll/pitch {rot.as_euler("xyz",degrees=True)[:2].round(1)} deg | pose valid={a["pose_valid"][i]} '
                    f'| ICP p90={a["icp_p90"][i]:.3f}m | segment {a["segment"][i]} | H=terrain Z-base Z; gray is NOT zero terrain')
                fig.tight_layout(rect=(0,0,1,.93));writer.grab_frame()
                if n in (0,len(indices)//3,2*len(indices)//3,len(indices)-1):
                    fig.savefig(out/f'key_{times[i]:05.2f}s.png',dpi=125)
                if n%30==0:print('VIDEO',video.name,n,'/',len(indices),flush=True)
        plt.close(fig)
        result=subprocess.run([shutil.which('ffprobe'),'-v','error','-select_streams','v:0','-show_entries',
            'stream=width,height,nb_frames,duration','-of','json',str(video)],capture_output=True,text=True,check=True)
        dump(video.with_suffix('.json'),json.loads(result.stdout))
    fig,axs=plt.subplots(4,1,figsize=(13,10),sharex=True)
    euler=Rotation.from_quat(a['pose'][:,3:]).as_euler('xyz',degrees=True)
    axs[0].plot(times,euler[:,:2]);axs[0].legend(['roll','pitch']);axs[0].set_ylabel('deg')
    for name in ('ahead','front_wheels','rear_wheels','body'):
        axs[1].plot(times,a['V'][:,REGIONS[name]].mean(axis=1),label=name)
    axs[1].legend(ncol=4);axs[1].set_ylabel('valid fraction')
    axs[2].plot(times,np.where(a['V'],a['age'],np.nan)[:,REGIONS['rear_wheels']],alpha=.15,color='#b86b00')
    axs[2].set_ylabel('rear age (s)')
    axs[3].plot(times,a['pose'][:,:3]);axs[3].legend(['X','Y','Z']);axs[3].set_ylabel('local pose (m)');axs[3].set_xlabel('receive time relative to recording start (s)')
    for ax in axs:ax.grid(alpha=.2)
    fig.tight_layout();fig.savefig(out/'timeline.png',dpi=140);plt.close(fig)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--recording',default='151135');p.add_argument('--end',type=float,default=17.)
    p.add_argument('--repo',type=Path,default=DEFAULT_REPO);p.add_argument('--raw',type=Path,default=RAW)
    p.add_argument('--decode-only',action='store_true');p.add_argument('--render-only',action='store_true')
    p.add_argument('--stage-a',action='store_true')
    p.add_argument('--video-windows',default='9:15.5');p.add_argument('--label',default='')
    args=p.parse_args();matches=list(args.raw.glob('gait_*'+args.recording+'*'))
    if len(matches)!=1:raise ValueError('Recording must select exactly one folder')
    out=HERE/'output'/(matches[0].name+args.label);out.mkdir(parents=True,exist_ok=True)
    if args.render_only:
        render(dict(np.load(out/'observations.npz')),out,args.repo,args.video_windows);return
    d=decode(matches[0],out,args.end,args.repo)
    if args.decode_only:return
    if args.stage_a:stage_a(d,out,args.repo);return
    a=process(d,out,args.repo)
    render(a,out,args.repo,args.video_windows)


if __name__=='__main__':main()
