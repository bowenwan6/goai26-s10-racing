"""Read closed rosbag SQLite files without publishing or modifying recordings."""
import json
import sqlite3
from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message


def distribution(values):
    a=np.asarray(values,dtype=float)
    if not len(a):return None
    return {'min':float(a.min()),'median':float(np.median(a)),
            'p95':float(np.percentile(a,95)),'max':float(a.max())}


def audit(folder):
    manifest=json.loads((folder/'manifest.json').read_text())
    if manifest['status']=='recording':return {'id':folder.name,'skipped':'active recording'}
    streams={};joints=[];joint_velocity=[];joint_torque=[];imu_norm=[];imu_acc=[];imu_gyro=[]
    gait_counts={};state_counts={};steer={};nonfinite={};joint_shapes=set()
    for db in sorted((folder/'bag').glob('*.db3')):
        with sqlite3.connect(db.as_uri()+'?mode=ro',uri=True) as conn:
            integrity=conn.execute('PRAGMA quick_check').fetchone()[0]
            if integrity!='ok':raise ValueError('SQLite integrity: '+integrity)
            catalog={row[0]:(row[1],row[2]) for row in conn.execute('SELECT id,name,type FROM topics')}
            classes={}
            for tid,receive,raw in conn.execute('SELECT topic_id,timestamp,data FROM messages ORDER BY timestamp,id'):
                topic,kind=catalog[tid]
                if kind not in classes:classes[kind]=get_message(kind)
                msg=deserialize_message(raw,classes[kind])
                header=getattr(msg,'header',None)
                stamp=getattr(header,'stamp',None)
                source=stamp.sec*10**9+stamp.nanosec if stamp else None
                stream=streams.setdefault(topic,{'receive':[],'source':[],'zero_source':0})
                stream['receive'].append(receive)
                stream['source'].append(source or 0)
                stream['zero_source']+=int(not source or source<=0)
                numeric=[]
                if topic=='/JOINTS_DATA':
                    js=msg.data.joints_data;joint_shapes.add(len(js))
                    p=[j.position for j in js];v=[j.velocity for j in js];t=[j.torque for j in js]
                    joints.append(p);joint_velocity.append(v);joint_torque.append(t);numeric=p+v+t
                elif topic=='/IMU':
                    q=[msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w]
                    a=[msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z]
                    g=[msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z]
                    imu_norm.append(float(np.linalg.norm(q)));imu_acc.append(float(np.linalg.norm(a)))
                    imu_gyro.append(float(np.linalg.norm(g)));numeric=q+a+g
                elif topic=='/MOTION_INFO':
                    code=msg.data.gait_state.gait;state=msg.data.motion_state.state
                    gait_counts[str(code)]=gait_counts.get(str(code),0)+1
                    state_counts[str(state)]=state_counts.get(str(state),0)+1
                    numeric=[msg.data.vel_x,msg.data.vel_y,msg.data.vel_yaw,msg.data.height]
                elif topic in ('/STEER','/HANDLE_STEER','/REAL_STEER'):
                    def visit(obj,path=''):
                        if isinstance(obj,dict):
                            for k,v in obj.items():visit(v,path+'.'+k)
                        elif type(obj) in (int,float) and not path.startswith('.header'):
                            steer.setdefault(topic,{}).setdefault(path,[]).append(obj)
                            numeric.append(obj)
                    visit(message_to_ordereddict(msg))
                nonfinite[topic]=nonfinite.get(topic,0)+int(not np.all(np.isfinite(numeric)))
    report={'id':folder.name,'terrain':manifest['terrain'],'duration_s':manifest['duration_s'],
            'outcome':manifest['outcome'],'parameters':manifest['parameters'],'events':manifest['events'],
            'sqlite_integrity':'ok','decoded_all_messages':True,'streams':{},'gait_counts':gait_counts,
            'motion_state_counts':state_counts,'joint_array_lengths':sorted(joint_shapes),
            'numeric_messages_with_nonfinite':nonfinite}
    for topic,s in streams.items():
        receive=np.asarray(s['receive'],dtype=np.int64);source=np.asarray(s['source'],dtype=np.int64)
        sr=np.diff(source)/1e9;rr=np.diff(receive)/1e9
        report['streams'][topic]={'count':len(receive),'average_hz':len(receive)/manifest['duration_s'],
            'source_gap_s':distribution(sr),'receive_gap_s':distribution(rr),
            'receive_minus_source_s':distribution((receive-source)/1e9),
            'source_nonincreasing':int(np.sum(sr<=0)),'zero_source':s['zero_source'],
            'matches_manifest_count':len(receive)==manifest['topics'][topic]['count']}
    if joints:
        report['joint_position_range_per_channel']=(np.max(joints,axis=0)-np.min(joints,axis=0)).tolist()
        report['joint_velocity_abs_max_per_channel']=np.max(np.abs(joint_velocity),axis=0).tolist()
        report['joint_torque_abs_max_per_channel']=np.max(np.abs(joint_torque),axis=0).tolist()
    report['imu_quaternion_norm']=distribution(imu_norm)
    report['imu_acceleration_norm']=distribution(imu_acc)
    report['imu_angular_velocity_norm']=distribution(imu_gyro)
    report['steer_fields']={topic:{name:distribution(values) for name,values in fields.items()} for topic,fields in steer.items()}
    report['missing_low_level_actions']='/JOINTS_CMD' not in streams
    report['scope']='All messages deserialized; numeric IMU/joints/motion/steer checked; lidar XYZ geometry not evaluated'
    return report


if __name__=='__main__':
    root=Path.home()/'s10_gait_data'
    print(json.dumps([audit(p) for p in sorted(root.glob('gait_*')) if p.is_dir() and (p/'manifest.json').exists()],ensure_ascii=False))
