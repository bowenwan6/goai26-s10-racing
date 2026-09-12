"""Batch remaining ledge recordings. Read-only source bags; reuse the validated controller.
Run: .venv-win/Scripts/python.exe -B artifacts/s10-ledge-batch-20260910/batch_match.py
"""
import argparse
import json
from pathlib import Path
import sqlite3
import struct
import sys
from types import SimpleNamespace
import xml.etree.ElementTree as ET
import numpy as np
import mujoco

OUT=Path(__file__).resolve().parent
sys.path.insert(0,str(OUT.parent/'s10-ledge-match-20260909'))
import match as m
CLIPS=json.loads((OUT/'clips.json').read_text(encoding='utf-8'))


def xyz_points(msg):
    fields={f.name:f for f in msg.fields};fmt={7:'f4',8:'f8'};endian='>' if msg.is_bigendian else '<'
    assert msg.row_step>=msg.width*msg.point_step and len(msg.data)>=msg.height*msg.row_step
    for key in ['x','y','z']:
        f=fields[key];assert f.datatype in fmt and f.count==1
        assert 0<=f.offset<=msg.point_step-np.dtype(fmt[f.datatype]).itemsize
    dt=np.dtype(dict(names=['x','y','z'],formats=[endian+fmt[fields[k].datatype] for k in ['x','y','z']],
                     offsets=[fields[k].offset for k in ['x','y','z']],itemsize=msg.point_step))
    array=np.ndarray((msg.height,msg.width),dtype=dt,buffer=msg.data,strides=(msg.row_step,msg.point_step))
    xyz=np.stack([array[k].ravel() for k in ['x','y','z']],axis=1)
    return xyz[np.isfinite(xyz).all(axis=1)]


def selfcheck():
    m.selfcheck()
    # Padded rows, padded records, big endian, and comparison with the existing reader.
    expected=np.arange(12,dtype=float).reshape(4,3);raw=bytearray(96)
    for i,v in enumerate(expected):struct.pack_into('>fff',raw,(i//2)*48+(i%2)*20,*v)
    msg=SimpleNamespace(width=2,height=2,point_step=20,row_step=48,is_bigendian=True,data=raw,
        fields=[SimpleNamespace(name=k,offset=j*4,datatype=7,count=1) for j,k in enumerate('xyz')],
        header=SimpleNamespace(frame_id='base_link',stamp=SimpleNamespace(sec=0,nanosec=0)))
    assert np.allclose(xyz_points(msg),expected)
    assert np.allclose(xyz_points(msg),m.review.preview_points(msg,limit=4)['points'])
    # Up / forward-down / up-then-reverse-down must have different completion rules.
    feet=np.zeros((30,4,3));feet[:,:,0]=1;feet[:,:,2]=.381
    assert completed(feet,np.zeros(30),np.arange(30)*.02,.5,.3,'up')['up_completed']
    feet[:,:,2]=.081
    assert completed(feet,np.zeros(30),np.arange(30)*.02,.5,.3,'down')['down_completed']
    assert not completed(feet,np.zeros(30),np.arange(30)*.02,.5,.3,'cycle')['completed']
    feet[:15,:,2]=.381;feet[15:,:,0]=0
    assert completed(feet,np.zeros(30),np.arange(30)*.02,.5,.3,'cycle',.3)['completed']
    tilted=np.zeros(30);tilted[2]=70
    assert not completed(feet,tilted,np.arange(30)*.02,.5,.3,'cycle',.3)['up_completed']
    # Near-edge support is a completed ascent even without a full wheel-radius margin.
    feet[:,:,0]=.505;feet[:,:,2]=.381
    assert not completed(feet,np.zeros(30),np.arange(30)*.02,.5,.3,'up')['completed']
    assert completed(feet,np.zeros(30),np.arange(30)*.02,.5,.3,'up',top_contacts=np.ones((30,4),bool))['completed']
    assert not completed(feet,np.zeros(30),np.arange(30)*.02,.5,.3,'up',top_contacts=np.zeros((30,4),bool))['completed']
    contacts=np.ones((30,4),bool);contacts[1:,3]=False
    assert completed(feet,np.zeros(30),np.arange(30)*.02,.5,.3,'up',top_contacts=contacts)['completed']
    contacts[:,3]=False
    assert not completed(feet,np.zeros(30),np.arange(30)*.02,.5,.3,'up',top_contacts=contacts)['completed']


def load_record(short):
    path=next((m.REVIEW/'decoded').glob('*'+short+'*.meta.json'))
    meta=json.loads(path.read_text(encoding='utf-8'))
    return meta,dict(np.load(m.REVIEW/'decoded'/(meta['id']+'.npz')))


def clouds_for(short,meta):
    cache=OUT/(short+'_clouds.npz')
    intervals=[(c['start']-.25,c['peak']-.1) for c in CLIPS if c['recording']==short]
    if cache.exists():
        cached=dict(np.load(cache))
        if 'intervals' in cached and np.array_equal(cached['intervals'],intervals):return cached
    anchor=meta['manifest']['started_wall_ns'];store=m.review.store_types();frames=[];times=[];topics=[]
    for db in (Path(meta['raw_path'])/'bag').glob('*.db3'):
        with sqlite3.connect(db.resolve().as_uri()+'?mode=ro',uri=True) as conn:
            ids=dict(conn.execute("SELECT id,name FROM topics WHERE type='sensor_msgs/msg/PointCloud2'"))
            conditions=' OR '.join('timestamp BETWEEN ? AND ?' for _ in intervals)
            values=[int(anchor+t*1e9) for a,b in intervals for t in (a-.3,b+.3)]
            query='SELECT topic_id,data FROM messages WHERE topic_id IN ('+','.join(map(str,ids))+') AND ('+conditions+') ORDER BY timestamp'
            for tid,raw in conn.execute(query,values):
                msg=store.deserialize_cdr(raw,'sensor_msgs/msg/PointCloud2')
                t=(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec-anchor)/1e9
                if not any(a<=t<=b for a,b in intervals):continue
                assert msg.header.frame_id=='base_link'
                p=xyz_points(msg);p=p[(np.linalg.norm(p,axis=1)>.15)&(np.linalg.norm(p,axis=1)<4)]
                frames.append(p.astype('f4'));times.append(t);topics.append(ids[tid])
    np.savez_compressed(cache,xyz=np.concatenate(frames),offsets=np.r_[0,np.cumsum([len(p) for p in frames])],time_s=times,topic=topics,intervals=intervals)
    print(short,'raw clouds',len(frames),flush=True)
    return dict(np.load(cache))


def down_edge(p):
    # A top-down lidar often cannot see the vertical face: bound the edge using two support levels.
    behind=p[(p[:,0]<-.15)&(p[:,0]>-1.8)&(abs(p[:,1])<.8)&(p[:,2]<-.1)&(p[:,2]>-.8)]
    top=m.plane(behind)
    if top is None:return None
    n=np.array(top['normal']);relative=p@n-top['offset_m']
    local=(p[:,0]>.1)&(p[:,0]<2)&(abs(p[:,1])<.65)
    upper=p[local&(abs(relative)<.025)];lower=p[local&(relative<-.15)&(relative>-.5)]
    bounds=[]
    for y in np.arange(-.6,.6,.12):
        u=upper[(upper[:,1]>=y)&(upper[:,1]<y+.12)]
        lo=lower[(lower[:,1]>=y)&(lower[:,1]<y+.12)]
        if len(u)<5 or len(lo)<5:continue
        near=float(np.percentile(u[:,0],98));far=float(np.percentile(lo[:,0],2))
        if .0<far-near<.65:bounds.append([y+.06,near,far])
    if len(bounds)<3:return None
    b=np.array(bounds);mid=b[:,1:3].mean(axis=1)
    slope,intercept=np.polyfit(b[:,0],mid,1);yaw=float(np.degrees(np.arctan(-slope)))
    if abs(yaw)>30:return None
    return dict(edge_horizontal_distance_m=float(intercept/np.sqrt(1+slope*slope)),face_yaw_deg=yaw,
        edge_bracket_width_m=float(np.median(b[:,2]-b[:,1])),method='top/lower-surface visibility bracket',
        bounds=b.tolist(),support_plane=top)


def prepare(c,meta,d,clouds):
    assert np.isclose(c['height'],meta['manifest']['parameters']['height_cm']/100)
    folder=OUT/c['id'];folder.mkdir(exist_ok=True)
    anchor=meta['manifest']['started_wall_ns'];time=np.arange(c['start'],c['end']+.0001,.005)
    jt=(d['JOINTS_DATA_src']-anchor)/1e9;it=(d['IMU_src']-anchor)/1e9
    def interp(v,t):
        assert t[0]<=time[0] and t[-1]>=time[-1] and np.all(np.diff(t)>0)
        return np.array([np.interp(time,t,x) for x in v.T]).T
    dr,off=m.replay.calibration();q=interp(d['JOINTS_DATA_v'][:,:16],jt)*dr+off
    dq=interp(d['JOINTS_DATA_v'][:,16:32],jt)*dr;q[:,m.replay.WHEELS]-=q[0,m.replay.WHEELS]
    imu=d['IMU_v'][:,:4].copy()
    for i in range(1,len(imu)):
        if np.dot(imu[i],imu[i-1])<0:imu[i]*=-1
    quat=interp(imu,it);quat/=np.linalg.norm(quat,axis=1)[:,None];quat=m.replay.root_quaternions(quat)
    mt=(d['MOTION_INFO_src']-anchor)/1e9
    motion=interp(d['MOTION_INFO_v'],mt)
    # Discrete controller state/gait use sample-and-hold, never linear interpolation.
    motion[:,4:]=d['MOTION_INFO_v'][np.clip(np.searchsorted(mt,time,side='right')-1,0,len(mt)-1),4:]
    assert (motion[:,4]==17).all(),'Control interruption inside selected interval'
    reference=dict(time_s=time,joint_position=q,joint_velocity=dq,base_quaternion_wxyz=quat,
        motion=motion,initial_gyro=interp(d['IMU_v'][:,4:7],it)[0])
    assert all(np.isfinite(v).all() for v in reference.values())
    np.savez_compressed(folder/'reference.npz',**reference)
    fits=[]
    for i,t in enumerate(clouds['time_s']):
        if clouds['topic'][i]!='/rslidar_front/points' or not c['start']-.2<=t<=c['peak']-.1:continue
        xyz=clouds['xyz'][clouds['offsets'][i]:clouds['offsets'][i+1]]
        rot=m.rotation(d['IMU_v'][np.argmin(abs(it-t)),:4])
        if c['mode']=='down':
            j=np.argmin(abs(clouds['time_s']-t)+np.where(clouds['topic']=='/rslidar_rear/points',0,1e6))
            rear=clouds['xyz'][clouds['offsets'][j]:clouds['offsets'][j+1]]
            result=down_edge(np.r_[xyz,rear]@rot.T)
        else:
            result=m.fit(xyz,rot,max_vertical_z=.45)
            if result and not (result.get('ground') and .12<result['visible_face_height_m']<c['height']+.10
                               and result['ground_tilt_after_imu_deg']<15):result=None
        if result:fits.append(dict(time_s=float(t),**result))
    # Once the face is occluded, a distant wall must not silently continue its trajectory.
    continuous=[]
    for row in fits:
        if continuous and (row['time_s']-continuous[-1]['time_s']>.35 or
                           abs(row['edge_horizontal_distance_m']-continuous[-1]['edge_horizontal_distance_m'])>.18):break
        continuous.append(row)
    fits=continuous
    initial=[r for r in fits if abs(r['time_s']-c['start'])<=.25]
    if len(initial)<2:
        geometry=dict(status='unresolved',fits=fits,reason='Fewer than two initial-frame edge estimates')
    else:
        geometry=dict(status='estimated',fits=fits,initial_distance_m=float(np.median([r['edge_horizontal_distance_m'] for r in initial])),
            yaw_deg=float(np.median([r['face_yaw_deg'] for r in initial])),
            initial_distance_spread_m=float(np.ptp([r['edge_horizontal_distance_m'] for r in initial])),
            method=initial[0].get('method','front vertical-plane fit'))
    m.review.dump(folder/'geometry.json',geometry)
    m.review.dump(folder/'configuration.json',{**c,'source_recording_id':meta['id'],'human_review':'candidate_not_confirmed',
        'time_basis':'source_stamp_ns minus manifest.started_wall_ns','reference_kind':'measured state, not original action'})
    return reference,geometry


def completed(feet,tilts,time,edge,height,mode,reverse_start=None,top_contacts=None):
    lateral=np.all(abs(feet[:,:,1])<1-.081,axis=1)
    on_top=np.all((feet[:,:,0]>edge+.081)&(feet[:,:,0]<edge+3-.081)&(abs(feet[:,:,2]-(height+.081))<.06),axis=1)&(tilts<25)&lateral
    rim_clear=on_top.copy()
    if top_contacts is not None:
        assert top_contacts.shape==(len(time),4)
        # A moving/adjusting quadruped can stand on three wheels: require evidence that
        # every wheel landed, all remain near the top, and at least three still support it.
        landed=np.logical_or.accumulate(top_contacts,axis=0).all(axis=1)
        near_top=np.all((feet[:,:,0]>edge-.03)&(feet[:,:,0]<edge+3+.03)&
                        (abs(feet[:,:,1])<1+.03)&(abs(feet[:,:,2]-(height+.081))<.04),axis=1)
        on_top=landed&near_top&(top_contacts.sum(axis=1)>=3)&(tilts<25)
    on_lower=np.all((feet[:,:,0]>edge+.081)&(abs(feet[:,:,2]-.081)<.06),axis=1)&(tilts<25)
    returned=np.all((feet[:,:,0]<edge-.081)&(abs(feet[:,:,2]-.081)<.06),axis=1)&(tilts<25)
    def sustained_end(mask):
        ix=np.flatnonzero(np.convolve(mask.astype(int),np.ones(10,dtype=int),mode='valid')==10) if len(mask)>=10 else []
        return int(ix[0]+9) if len(ix) else None
    ascent_window=np.ones(len(time),dtype=bool) if reverse_start is None else time<reverse_start
    up_end=sustained_end(on_top&ascent_window)
    up=up_end is not None and not np.any(tilts[:up_end+1]>60)
    # Landing must still hold at clip end; flying past the edge is not a completed descent.
    down=bool(np.all(on_lower[-10:])) if mode=='down' else bool(np.all(returned[-10:]))
    descent_window=np.ones(len(time),dtype=bool) if reverse_start is None else time>=reverse_start
    down=down and not np.any(tilts[descent_window]>60)
    full=down if mode=='down' else (up and down) if mode=='cycle' else bool(np.all(on_top[-10:]))
    return dict(completed=bool(full and not np.any(tilts>60)),up_completed=up if mode!='down' else None,
        down_completed=down if mode in ('down','cycle') else None,
        up_completed_source_s=float(time[up_end]) if up and mode!='down' else None,
        ascent_criterion='all wheels landed on top, near surface, at least three supporting' if top_contacts is not None else 'legacy full wheel-radius clearance',
        final_full_wheel_radius_clearance=bool(np.all(rim_clear[-10:])),
        final_tilt_deg=float(tilts[-1]),final_stable_under_20=bool(np.max(tilts[-10:])<20))


def scene(c,ref,geom,offset):
    tree=ET.parse(m.replay.ROOT/'simulation_review/free_base.xml');world=tree.getroot().find('worldbody')
    yaw=np.deg2rad(geom['yaw_deg']);normal=np.array([np.cos(yaw),np.sin(yaw),0])
    edge=geom['initial_distance_m']+offset;h=c['height'];desc=c['mode']=='down'
    centre=(edge+(-1.5 if desc else 1.5))*normal;centre[2]=h/2
    ET.SubElement(world,'geom',name='matched_ledge',type='box',pos=' '.join(map(str,centre)),
        size=f'1.5 1 {h/2}',quat=f'{np.cos(yaw/2)} 0 0 {np.sin(yaw/2)}',
        rgba='.48 .63 .75 1',friction='1 .01 .001',contype='1',conaffinity='1')
    model=mujoco.MjModel.from_xml_string(ET.tostring(tree.getroot(),encoding='unicode'));data=mujoco.MjData(model)
    joints=model.actuator_trnid[:,0];qa=model.jnt_qposadr[joints];va=model.jnt_dofadr[joints]
    data.qpos[:3]=0;data.qpos[3:7]=ref['base_quaternion_wxyz'][0];data.qpos[qa]=ref['joint_position'][0]
    mujoco.mj_forward(model,data);wheels=[model.body(n).id for n in ['fl_wheel','fr_wheel','hl_wheel','hr_wheel']]
    data.qpos[2]=.083-np.min(data.xpos[wheels,2])+(h if desc else 0)
    data.qvel[va]=ref['joint_velocity'][0];rot=np.empty(9);mujoco.mju_quat2Mat(rot,data.qpos[3:7])
    data.qvel[:3]=rot.reshape(3,3)@[*ref['motion'][0,:2],0];data.qvel[3:6]=ref['initial_gyro']
    mujoco.mj_forward(model,data)
    penetration=float(min([v.dist for v in data.contact[:data.ncon]]+[0]))
    return tree,model,data,normal,edge,penetration


def run(c,ref,geom,offset,delay,save=None):
    tree,model,data,normal,edge,penetration=scene(c,ref,geom,offset)
    if penetration<-.005:return dict(offset_m=offset,delay_s=delay,invalid_initial_penetration_m=penetration,completed=False,score=1e6)
    result,arrays=m.track(model,data.qpos.copy(),data.qvel.copy(),ref['time_s'],ref['joint_position'],
        ref['joint_velocity'],normal,edge,c['height'],delay=delay,descending=c['mode']=='down',**c.get('controller',{}))
    log=arrays['log'];feet=arrays['wheel_positions'].copy();feet[:,:,0]=arrays['wheel_positions']@normal
    feet[:,:,1]=arrays['wheel_positions']@np.array([-normal[1],normal[0],0])
    status=completed(feet,log[:,1],log[:,0],edge,c['height'],c['mode'],c.get('reverse_start'),arrays['wheel_top_contact'])
    ix=np.clip(np.round((log[:,0]-c['start'])/.005).astype(int),0,len(ref['time_s'])-1)
    angular=np.degrees(2*np.arccos(np.clip(abs(np.sum(arrays['qpos'][:,3:7]*ref['base_quaternion_wxyz'][ix],axis=1)),0,1)))
    fits=[r for r in geom['fits'] if c['start']<=r['time_s']<=c['peak']-.1]
    error=np.interp([r['time_s'] for r in fits],log[:,0],edge-arrays['qpos'][:,:3]@normal)-[r['edge_horizontal_distance_m'] for r in fits]
    rmse=float(np.sqrt(np.mean(error**2))) if len(error) else None
    # Score compares actual recorded orientation; do not assume every recording is a successful demonstration.
    result.update(status,offset_m=float(offset),delay_s=float(delay),initial_distance_m=edge,
        reference_orientation_rmse_deg=float(np.sqrt(np.mean(angular**2))),edge_distance_rmse_m=rmse,
        score=float(np.sqrt(np.mean(angular**2)))+200*(rmse if rmse is not None else abs(offset)),
        initial_penetration_m=penetration)
    if save:
        folder=OUT/c['id'];m.review.dump(folder/(save+'.json'),result);np.savez_compressed(folder/(save+'.npz'),**arrays)
        key=tree.getroot().find('keyframe')
        if key is None:key=ET.SubElement(tree.getroot(),'keyframe')
        ET.SubElement(key,'key',name='matched_start',qpos=' '.join(map(str,arrays['initial_qpos'])))
        tree.write(folder/(save+'_scene.xml'),encoding='utf-8')
    return result


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--only');parser.add_argument('--prepare-only',action='store_true')
    parser.add_argument('--wide',action='store_true',help='If distance alone fails, search all distance offsets at delays +/-0.1,0.2,0.3 s')
    args=parser.parse_args();selfcheck();records={};clouds={};summaries=[]
    for c in CLIPS:
        if args.only and c['id']!=args.only and c['recording']!=args.only:continue
        short=c['recording']
        if short not in records:
            records[short]=load_record(short);clouds[short]=clouds_for(short,records[short][0])
        folder=OUT/c['id'];print('PREPARE',c['id'],c['mode'],flush=True)
        ref,geom=prepare(c,*records[short],clouds[short])
        if args.prepare_only or geom['status']!='estimated':
            print(c['id'],geom['status'],flush=True);continue
        baseline=run(c,ref,geom,0,0,'baseline');results=[]
        for offset in np.linspace(-.18,.18,13):results.append(run(c,ref,geom,float(offset),0))
        success=[r for r in results if r['completed']]
        if not success and c.get('reference_outcome')!='interrupted':
            candidates=list(results) if args.wide else sorted(results,key=lambda r:r['score'])[:3]
            for candidate in candidates:
                for delay in ([-.3,-.2,-.1,.1,.2,.3] if args.wide else [-.15,.15]):
                    results.append(run(c,ref,geom,candidate['offset_m'],delay))
            success=[r for r in results if r['completed']]
        eligible=results if c.get('reference_outcome')=='interrupted' else success or results
        selected=min(eligible,key=lambda r:r['score']);run(c,ref,geom,selected['offset_m'],selected['delay_s'],'selected')
        m.review.dump(folder/'search.json',results)
        summary=dict(clip=c,geometry={k:v for k,v in geom.items() if k!='fits'},baseline=baseline,selected=selected,
            trial_count=len(results),successful_trials=len(success),human_review='candidate_not_confirmed',
            training_ready=False,reason='Simulation matching is not human demonstration qualification')
        m.review.dump(folder/'summary.json',summary);summaries.append(summary)
        print('RESULT',c['id'],selected['completed'],'up',selected.get('up_completed'),'down',selected.get('down_completed'),
            'offset',selected['offset_m'],'delay',selected['delay_s'],flush=True)
    m.review.dump(OUT/'last_run.json',summaries)


if __name__=='__main__':main()
