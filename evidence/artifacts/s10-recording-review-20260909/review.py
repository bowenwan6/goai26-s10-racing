"""Offline recording inventory/review. Raw SQLite files are always opened mode=ro.
Run from repo: .venv-win/Scripts/python.exe -B artifacts/s10-recording-review-20260909/review.py
No ROS, networking, physics stepping, hashes, or writes to source recordings.
"""
import ast
import base64
from collections import Counter
import csv
import html
import json
import math
import re
from pathlib import Path
import sqlite3
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np

OUT = Path(__file__).resolve().parent
REPO = OUT.parents[1]
RAW = Path('D:/S10Data/050/2026-09-09')
sys.dont_write_bytecode = True
sys.path.insert(0, str(REPO/'artifacts/s10-expert-analysis'))
import analyze
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

# Reuse the recorder's tested, padding/endian-aware PointCloud2 preview reader.
tree = ast.parse((REPO/'tools/s10_gait_capture/server.py').read_text(encoding='utf-8'))
node = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'preview_points')
exec(compile(ast.Module(body=[node], type_ignores=[]), 'server.py:preview_points', 'exec'))

def dump(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')

def key(topic):
    return topic.strip('/').replace('/', '_')

def store_types():
    store = get_typestore(Stores.ROS2_JAZZY)
    definitions = {}
    for p in (REPO/'tools/s10_gait_capture/vendor_ws/src/drdds/msg').glob('*.msg'):
        definitions.update(get_types_from_msg(p.read_text(), 'drdds/msg/'+p.stem))
    store.register(definitions)
    return store

def extract(folder, store):
    manifest = json.loads((folder/'manifest.json').read_text(encoding='utf-8'))
    anchor = manifest['started_wall_ns']
    streams, clouds, frames, names, errors, dbcounts = {}, {}, {}, set(), Counter(), []
    for db in sorted((folder/'bag').glob('*.db3')):
        with sqlite3.connect(db.resolve().as_uri()+'?mode=ro', uri=True) as conn:
            topics = {r[0]: (r[1], r[2]) for r in conn.execute('SELECT id,name,type FROM topics')}
            counts = dict(conn.execute('SELECT topic_id,count(*) FROM messages GROUP BY topic_id'))
            dbcounts.append({'path': str(db), 'counts': {topics[k][0]: v for k,v in counts.items()}})
            for tid, rx, raw in conn.execute('SELECT topic_id,timestamp,data FROM messages ORDER BY id'):
                topic, kind = topics[tid]
                s = streams.setdefault(topic, {'rx': [], 'src': [], 'v': [], 'type': kind})
                try:
                    msg = store.deserialize_cdr(raw, kind)
                    stamp = getattr(getattr(msg, 'header', None), 'stamp', None)
                    src = stamp.sec*10**9+stamp.nanosec if stamp else 0
                    frame = getattr(getattr(msg, 'header', None), 'frame_id', None)
                    if isinstance(frame, str): frames.setdefault(topic, set()).add(frame)
                    vals = []
                    if kind == 'drdds/msg/JointsData':
                        js = msg.data.joints_data
                        names.add(tuple(bytes(j.name).split(b'\0')[0].decode('ascii') for j in js))
                        vals = [float(getattr(j,a)) for a in ('position','velocity','torque') for j in js]
                    elif kind == 'sensor_msgs/msg/Imu':
                        vals = analyze.vec(msg.orientation,'xyzw')+analyze.vec(msg.angular_velocity)+analyze.vec(msg.linear_acceleration)
                    elif kind == 'drdds/msg/MotionInfo':
                        d = msg.data
                        vals = [d.vel_x,d.vel_y,d.vel_yaw,d.height,d.motion_state.state,d.gait_state.gait]
                    elif kind == 'drdds/msg/Steer':
                        vals = analyze.vec(msg.data)+[msg.data.roll,msg.data.pitch,msg.data.yaw]
                    elif kind == 'drdds/msg/Gait': vals = [msg.data.gait]
                    elif kind == 'sensor_msgs/msg/PointCloud2':
                        vals = [msg.width*msg.height, len(msg.data)]
                        cc = clouds.setdefault(topic, [])
                        if not cc or rx-cc[-1]['rx_ns'] >= 490_000_000:
                            pc = preview_points(msg, limit=6000)
                            xyz = np.asarray(pc['points'], dtype=float).reshape(-1,3)
                            # ponytail: bounded local display sample; use raw bag for small/contact details.
                            xyz = xyz[(np.linalg.norm(xyz,axis=1)<12) & (np.linalg.norm(xyz,axis=1)>.15)]
                            packed = np.round(xyz*1000).astype('<i2').tobytes()
                            cc.append({'rx_ns':rx, 'src_ns':src, 'rx':(rx-anchor)/1e9, 'src':(src-anchor)/1e9,
                                       'frame':pc['frame_id'], 'total':pc['total'], 'n':len(xyz),
                                       'xyz_mm_b64':base64.b64encode(packed).decode('ascii')})
                    s['rx'].append(rx); s['src'].append(src); s['v'].append(vals)
                except Exception as exc:
                    errors[topic+': '+str(exc)] += 1
    arrays = {}
    for topic,s in streams.items():
        for name in ('rx','src','v'):
            arrays[key(topic)+'_'+name] = np.asarray(s[name],dtype=float if name=='v' else np.int64)
    np.savez_compressed(OUT/'decoded'/f'{folder.name}.npz', **arrays)
    # Timestamp integers remain strings in browser JSON to avoid JS precision loss.
    for cc in clouds.values():
        for c in cc:
            c['rx_ns']=str(c['rx_ns']); c['src_ns']=str(c['src_ns'])
    dump(OUT/'decoded'/f'{folder.name}.clouds.json', clouds)
    result = {'id':folder.name, 'manifest':manifest, 'raw_path':str(folder), 'databases':dbcounts,
              'frames':{k:sorted(v) for k,v in frames.items()}, 'joint_name_orders':[list(v) for v in names],
              'decode_errors':dict(errors)}
    dump(OUT/'decoded'/f'{folder.name}.meta.json',result)
    return result

def runs(times, values, end):
    if not len(times): return []
    changes = np.r_[0, np.flatnonzero(np.any(np.diff(values,axis=0)!=0,axis=1))+1]
    return [{'start_s':float(times[a]), 'end_s':float(times[b]) if b<len(times) else float(end),
             'values':values[a].astype(int).tolist()} for a,b in zip(changes,np.r_[changes[1:],len(times)])]

def audit(meta):
    d = np.load(OUT/'decoded'/f"{meta['id']}.npz")
    m = meta['manifest']; anchor=m['started_wall_ns']; duration=m['duration_s']
    report = {**meta, 'time_basis':'seconds relative to manifest.started_wall_ns; source and receive kept separately',
              'anchor_ns':str(anchor), 'duration_s':duration, 'streams':{}, 'issues':[]}
    for topic in sorted({t for db in meta['databases'] for t in db['counts']}):
        k=key(topic); rx=d[k+'_rx']; src=d[k+'_src']; v=d[k+'_v']
        dt=np.diff(src)/1e9; dr=np.diff(rx)/1e9
        tr=(rx-anchor)/1e9; ts=(src-anchor)/1e9
        positive=dt[dt>0]; median=float(np.median(positive)) if len(positive) else None
        gaplim=max(.1,5*median) if median is not None else .1
        valid=src>0
        s={'type':m['topics'].get(topic,{}).get('type'), 'count':len(rx),
           'manifest_count':m['topics'].get(topic,{}).get('count'),
           'source_start_s':float(ts[0]) if valid[0] else None,'source_end_s':float(ts[-1]) if valid[-1] else None,
           'receive_start_s':float(tr[0]),'receive_end_s':float(tr[-1]),
           'source_start_ns':str(int(src[0])),'source_end_ns':str(int(src[-1])),
           'receive_start_ns':str(int(rx[0])),'receive_end_ns':str(int(rx[-1])),
           'source_dt_s':analyze.distribution(dt),'receive_dt_s':analyze.distribution(dr),
           'receive_minus_source_s':analyze.distribution((rx[valid]-src[valid])/1e9),
           'source_nonincreasing':int(np.sum(dt<=0)), 'source_missing':int(np.sum(src<=0)),
           'nonfinite_values':int(np.sum(~np.isfinite(v))),
           'source_median_hz':1/median if median else None,
           'source_mean_hz':float((len(src)-1)/((src[-1]-src[0])/1e9)) if src[-1]>src[0] else None,
           'source_gap_threshold_s':gaplim,
           'source_gaps':[{'start_s':float(ts[i]),'end_s':float(ts[i+1]),'gap_s':float(dt[i]),
                           'receive_start_s':float(tr[i]),'receive_end_s':float(tr[i+1])} for i in np.flatnonzero(dt>gaplim)],
           'receive_stalls':[{'start_s':float(tr[i]),'end_s':float(tr[i+1]),'source_dt_s':float(dt[i])} for i in np.flatnonzero(dr>max(.2,gaplim))],
           'rate_windows':[]}
        for start in np.arange(0,duration,5):
            ii=(tr[:-1]>=start)&(tr[:-1]<start+5)&(dt>0)
            if np.any(ii):
                s['rate_windows'].append({'receive_start_s':float(start),'end_s':min(float(start+5),duration),
                    'source_median_hz':float(1/np.median(dt[ii])), 'source_dt_p95_s':float(np.percentile(dt[ii],95))})
        if topic in ('/IMU','/JOINTS_DATA','/MOTION_INFO','/rslidar_front/points','/rslidar_rear/points'):
            if duration-tr[-1]>.5: report['issues'].append(f'{topic} 接收流提前结束 {tr[-1]:.3f}s（距录制末尾 {duration-tr[-1]:.3f}s）')
            if s['source_gaps']: report['issues'].append(f'{topic} 有 {len(s["source_gaps"])} 个源时间长间隔，见 topics.csv / inventory.json')
        if s['source_nonincreasing'] or s['source_missing']: report['issues'].append(f'{topic} 源时间缺失/非递增')
        if s['count']!=s['manifest_count']: report['issues'].append(f'{topic} 数据库计数与 manifest 不一致')
        offset=s['receive_minus_source_s']['median'] if s['receive_minus_source_s'] else None
        if offset is not None and abs(offset)>.2: report['issues'].append(f'{topic} receive-source 中位数 {offset:+.3f}s；不得按源时间直接与机身话题混合')
        rr=[w['source_median_hz'] for w in s['rate_windows']]
        if rr and max(rr)>min(rr)*1.7: report['issues'].append(f'{topic} 5s 窗源时间采样率范围 {min(rr):.1f}–{max(rr):.1f}Hz（事件话题不视为掉帧）')
        report['streams'][topic]=s
    mt=(d['MOTION_INFO_src']-anchor)/1e9; mv=d['MOTION_INFO_v']
    report['control_timeline']=runs(mt,mv[:,4:6],float(mt[-1]))
    report['gait_commands']=[{'receive_s':float((rx-anchor)/1e9),'source_s':float((src-anchor)/1e9) if src>0 else None,
        'source_ns':str(int(src)),'receive_ns':str(int(rx)),'code':int(v[0])}
        for rx,src,v in zip(d['GAIT_rx'],d['GAIT_src'],d['GAIT_v'])] if 'GAIT_rx' in d else []
    for r in report['control_timeline']:
        if r['values'][0]!=17: report['issues'].append(f'实际运动状态 {r["values"][0]}：{r["start_s"]:.3f}–{r["end_s"]:.3f}s（源时间）')
    metadata=(Path(meta['raw_path'])/'bag/metadata.yaml').read_text(encoding='utf-8')
    total=re.search(r'^  message_count: (\d+)',metadata,re.M)
    files=metadata.split('  files:',1)[-1]
    filecounts=[int(v) for v in re.findall(r'^      message_count: (\d+)',files,re.M)]
    actual=sum(sum(db['counts'].values()) for db in meta['databases'])
    report['metadata_counts']={'database_actual':actual,'yaml_total':int(total[1]) if total else None,'yaml_file_counts':filecounts}
    if filecounts and sum(filecounts)!=actual:report['issues'].append(f'metadata.files 消息数 {sum(filecounts)} 与数据库实际 {actual} 不一致；按实际表计数')
    report['camera_evidence']={'camera_topics':[], 'media_files':[str(p) for p in Path(meta['raw_path']).rglob('*') if p.suffix.lower() in ('.mp4','.avi','.mkv','.mov','.webm')],
        'note':'本次实际话题类型仅六种，无 Image/CompressedImage/视频；原始目录无视频文件。'}
    return report

def main():
    (OUT/'decoded').mkdir(exist_ok=True)
    store=store_types(); reports=[]
    for folder in sorted(RAW.glob('gait_*')):
        cache=OUT/'decoded'/f'{folder.name}.meta.json'
        if not cache.exists():
            print('DECODE '+folder.name,flush=True); meta=extract(folder,store)
        else: meta=json.loads(cache.read_text(encoding='utf-8'))
        r=audit(meta); reports.append(r)
        print(json.dumps({'id':r['id'],'duration':r['duration_s'],'terrain':r['manifest']['terrain'],
            'control':r['control_timeline'],'issues':r['issues']},ensure_ascii=False),flush=True)
        dump(OUT/'inventory.json',reports)

if __name__=='__main__': main()
