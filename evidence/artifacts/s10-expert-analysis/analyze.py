"""Offline audit and numeric extraction; raw bags are opened read-only.

Run: python artifacts/s10-expert-analysis/analyze.py
rosbags is installed only in tmp/s10-analysis-deps.
"""
import json
from pathlib import Path
import sqlite3
import sys
from collections import Counter
import numpy as np

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parents[1]
sys.path.append(str(REPO/'tmp/s10-analysis-deps'))
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

def distribution(values):
    a = np.asarray(values, dtype=float)
    if not len(a): return None
    return dict(zip(('min','median','p95','max'), map(float, np.nanpercentile(a, [0,50,95,100]))))

def vec(value, keys='xyz'):
    return [float(getattr(value,k)) for k in keys]

def audit(folder, store):
    manifest = json.loads((folder/'manifest.json').read_text(encoding='utf-8'))
    report = {'id':folder.name, 'terrain':manifest['terrain'], 'duration_s':manifest['duration_s'],
              'outcome':manifest['outcome'],'notes':manifest['notes'],'events':manifest['events'],
              'empty_metadata':(folder/'bag/metadata.yaml').stat().st_size==0,'streams':{},'sqlite':[]}
    streams={}; layouts={}; frames={}; names=set(); errors=Counter()
    for db in sorted((folder/'bag').glob('*.db3')):
        with sqlite3.connect(db.resolve().as_uri()+'?mode=ro',uri=True) as conn:
            report['sqlite'].append({'file':db.name,'quick_check':conn.execute('PRAGMA quick_check').fetchall()})
            catalog = {row[0]:(row[1],row[2]) for row in conn.execute('SELECT id,name,type FROM topics')}
            for tid, rx, raw in conn.execute('SELECT topic_id,timestamp,data FROM messages ORDER BY id'):
                topic,kind=catalog[tid]
                stream=streams.setdefault(topic,{'receive_ns':[],'source_ns':[],'values':[]})
                try: msg=store.deserialize_cdr(raw,kind)
                except Exception as exc:
                    errors[topic+': '+str(exc)]+=1; continue
                stamp=getattr(getattr(msg,'header',None),'stamp',None)
                source=stamp.sec*10**9+stamp.nanosec if stamp else 0
                stream['receive_ns'].append(rx);stream['source_ns'].append(source)
                frame=getattr(getattr(msg,'header',None),'frame_id',None)
                if isinstance(frame,str):frames.setdefault(topic,set()).add(frame)
                vals=[]
                if kind=='drdds/msg/JointsData':
                    js=msg.data.joints_data
                    names.add(tuple(bytes(j.name).split(b'\0')[0].decode('ascii') for j in js))
                    vals=[v for attr in ('position','velocity','torque') for v in [float(getattr(j,attr)) for j in js]]
                elif kind=='sensor_msgs/msg/Imu':
                    vals=vec(msg.orientation,'xyzw')+vec(msg.angular_velocity)+vec(msg.linear_acceleration)
                elif kind=='drdds/msg/MotionInfo':
                    d=msg.data;vals=[d.vel_x,d.vel_y,d.vel_yaw,d.height,d.motion_state.state,d.gait_state.gait]
                elif kind=='nav_msgs/msg/Odometry':
                    vals=vec(msg.pose.pose.position)+vec(msg.pose.pose.orientation,'xyzw')+vec(msg.twist.twist.linear)+vec(msg.twist.twist.angular)
                    frames.setdefault(topic+'/child',set()).add(msg.child_frame_id)
                elif kind=='drdds/msg/Steer':
                    vals=vec(msg.data,'xyz')+[msg.data.roll,msg.data.pitch,msg.data.yaw]
                elif kind=='drdds/msg/Gait':vals=[msg.data.gait]
                elif kind=='sensor_msgs/msg/PointCloud2':
                    if topic not in layouts:
                        layouts[topic]={'fields':[(f.name,f.offset,f.datatype,f.count) for f in msg.fields],
                                       'point_step':msg.point_step,'is_bigendian':msg.is_bigendian,
                                       'width':msg.width,'height':msg.height,'frame_id':msg.header.frame_id}
                    vals=[msg.width*msg.height,len(msg.data)]
                stream['values'].append(vals)
    arrays={}
    for topic,s in streams.items():
        rx=np.asarray(s['receive_ns'],dtype=np.int64);src=np.asarray(s['source_ns'],dtype=np.int64)
        values=np.asarray(s['values'],dtype=float);dt=np.diff(src)/1e9
        prefix=topic.strip('/').replace('/','_')
        arrays[prefix+'_receive_ns']=rx;arrays[prefix+'_source_ns']=src;arrays[prefix+'_values']=values
        expected=manifest['topics'].get(topic,{}).get('count')
        r={'count':len(src),'manifest_count':expected,'count_matches':len(src)==expected,
           'source_span_s':float((src[-1]-src[0])/1e9),'source_nonincreasing':int(np.sum(dt<=0)),
           'source_gap_s':distribution(dt),'receive_gap_s':distribution(np.diff(rx)/1e9),
           'receive_minus_source_s':distribution((rx-src)/1e9),'source_missing':int(np.sum(src<=0)),
           'measured_hz':float((len(src)-1)/((src[-1]-src[0])/1e9)) if len(src)>1 and src[-1]>src[0] else None,
           'nonfinite_values':int(np.sum(~np.isfinite(values))),'source_gaps_over_100ms':int(np.sum(dt>.1))}
        if topic.startswith('/JOINTS_DATA'):
            r['position_range']=np.ptp(values[:,:16],axis=0).tolist()
            r['velocity_abs_p95']=np.percentile(abs(values[:,16:32]),95,axis=0).tolist()
            r['velocity_abs_max']=abs(values[:,16:32]).max(axis=0).tolist()
            r['intervals_5ms_20ms_100ms_1s']={str(d):int(np.sum(abs(dt-d)<d*.15)) for d in (.005,.02,.1,1.)}
        elif topic=='/IMU':
            r['quaternion_norm']=distribution(np.linalg.norm(values[:,:4],axis=1))
            r['gyro_norm']=distribution(np.linalg.norm(values[:,4:7],axis=1))
        elif topic=='/MOTION_INFO':
            r['state_counts']=dict(Counter(map(int,values[:,4])))
            r['gait_counts']=dict(Counter(map(int,values[:,5])))
            r['velocity_xy_norm']=distribution(np.linalg.norm(values[:,:2],axis=1))
            r['height']=distribution(values[:,3])
            changes=np.r_[True,np.any(np.diff(values[:,4:6],axis=0)!=0,axis=1)]
            r['transitions']=[{'t_s':float((t-src[0])/1e9),'state':int(v[4]),'gait':int(v[5])} for t,v in zip(src[changes],values[changes])]
        elif topic=='/SLAM_ODOM':
            xyz=values[:,:3]; steps=np.linalg.norm(np.diff(xyz,axis=0),axis=1)
            r.update(position_start=xyz[0].tolist(),position_end=xyz[-1].tolist(),position_range=np.ptp(xyz,axis=0).tolist(),
                     accumulated_distance_m=float(steps.sum()),step_m=distribution(steps),
                     position_derived_speed=distribution(steps/dt),quaternion_norm=distribution(np.linalg.norm(values[:,3:7],axis=1)))
        report['streams'][topic]=r
    report['joint_name_orders']=[list(x) for x in names]
    report['frames']={k:sorted(v) for k,v in frames.items()}
    report['lidar_first_message_layouts']=layouts
    report['decode_errors']=dict(errors)
    output=ROOT/'decoded';output.mkdir(exist_ok=True)
    np.savez_compressed(output/(folder.name+'.npz'),**arrays)
    return report

def main():
    store=get_typestore(Stores.ROS2_JAZZY); definitions={}
    for p in (REPO/'tools/s10_gait_capture/vendor_ws/src/drdds/msg').glob('*.msg'):
        definitions.update(get_types_from_msg(p.read_text(),'drdds/msg/'+p.stem))
    store.register(definitions)
    reports=[]
    for manifest in sorted((ROOT/'raw').rglob('manifest.json')):
        print('AUDIT '+manifest.parent.name,flush=True)
        report=audit(manifest.parent,store);reports.append(report)
        (ROOT/'audit.json').write_text(json.dumps(reports,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps({'id':report['id'],'sqlite':report['sqlite'],'decode_errors':report['decode_errors'],
                          'joint_hz':report['streams'].get('/JOINTS_DATA',{}).get('measured_hz'),
                          'odom':report['streams'].get('/SLAM_ODOM')},ensure_ascii=False),flush=True)

if __name__=='__main__':main()
