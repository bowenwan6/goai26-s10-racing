"""Local review UI with an append-only annotation journal and native-rate detail windows.
Run: .venv-win/Scripts/python.exe -B artifacts/s10-recording-review-20260909/review_server.py
"""
import argparse
import base64
from datetime import datetime, timezone
from functools import lru_cache
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import sqlite3
import threading
from urllib.parse import urlsplit, parse_qs
import uuid

import numpy as np
import mujoco
import build as builder
from review import OUT, RAW, preview_points, store_types

CATALOG = {r['recording_id']: r for r in json.loads((OUT/'recordings.json').read_text(encoding='utf-8'))}
JOURNAL = OUT/'human_reviews.jsonl'
LOCK = threading.Lock()
DETAIL_LOCK = threading.Lock()


def validate(entry):
    if not isinstance(entry, dict): raise ValueError('标注必须是对象')
    sid = entry.get('recording_id')
    if sid not in CATALOG: raise ValueError('未知录制')
    uuid.UUID(entry['id'])
    a, b = entry['start_s'], entry['end_s']
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in (a,b)):
        raise ValueError('时间必须为有限数值')
    if not 0 <= a < b <= CATALOG[sid]['duration_s']+1e-8: raise ValueError('区间超出录制或起点不小于终点')
    choices = {'level': ('passage','segment'), 'terrain': ('unknown','flat','stairs','platform','ledge'),
               'direction': ('unknown','up','down','level'),
               'outcome': ('uncertain','completed','failure','interrupted'),
               'human_review_status': ('pending','confirmed','uncertain'),
               'action_phase': ('unknown','approach','adjust','ascend','descend','platform_walk','turn','wait','recover','interrupted')}
    for k, allowed in choices.items():
        if entry.get(k) not in allowed: raise ValueError('无效字段：'+k)
    result = {k:entry[k] for k in ('id','recording_id','start_s','end_s',*choices)}
    for k in ('notes','obstacle_id','group_id'):
        value = entry.get(k,'')
        if not isinstance(value,str) or len(value)>4000: raise ValueError('无效文本：'+k)
        result[k] = value
    result.update(time_basis='source_stamp_minus_manifest_started_wall_ns',
                  anchor_ns=str(json.loads((RAW/sid/'manifest.json').read_text(encoding='utf-8'))['started_wall_ns']),
                  saved_at=datetime.now(timezone.utc).isoformat())
    result['retired'] = entry.get('retired') is True
    return result


def latest(path=JOURNAL):
    rows = {}
    if path.exists():
        for line in path.read_text(encoding='utf-8').splitlines():
            if line.strip():
                e = json.loads(line); rows[e['id']] = e
    return [e for e in rows.values() if not e.get('retired')]


def append_batch(entries, path=JOURNAL):
    if not isinstance(entries,list) or not 1 <= len(entries) <= 100: raise ValueError('每次保存 1–100 条标注')
    checked = [validate(e) for e in entries]  # Validate the whole batch before any write.
    with LOCK:
        with path.open('a',encoding='utf-8') as f:
            f.write(''.join(json.dumps(e,ensure_ascii=False,allow_nan=False)+'\n' for e in checked))
            f.flush(); os.fsync(f.fileno())
    return checked


@lru_cache(maxsize=2)
def decoded(sid):
    with np.load(OUT/'decoded'/f'{sid}.npz') as z: return dict(z)


@lru_cache(maxsize=1)
def runtime():
    return mujoco.MjModel.from_xml_path(str(OUT/'data/free_base.xml')), store_types()


def detail(sid, start, end):
    if sid not in CATALOG or not 0<=start<end<=CATALOG[sid]['duration_s']+1e-8 or end-start>20.001:
        raise ValueError('精细回放每次最多 20 秒，且须位于录制内')
    with DETAIL_LOCK:
        d = decoded(sid)
        manifest = json.loads((RAW/sid/'manifest.json').read_text(encoding='utf-8'))
        anchor = manifest['started_wall_ns']; jt=(d['JOINTS_DATA_src']-anchor)/1e9
        actual = jt[(jt>=start)&(jt<=end)]
        # Native joint timestamps; a sparse guard grid makes missing-data windows visible.
        tt = np.unique(np.r_[actual,np.arange(start,end,.1),end])
        model, store = runtime()
        frames = builder.replay_data({'id':sid,'anchor_ns':str(anchor),'duration_s':manifest['duration_s']},
                                     d,model,tt,('src',))['src']
        clouds={}
        for which in ('front','rear'):
            topic='/rslidar_'+which+'/points'; k=topic.strip('/').replace('/','_')
            st=(d[k+'_src']-anchor)/1e9
            wanted=d[k+'_rx'][(st>=start-.15)&(st<=end+.15)]
            rows=[]
            if len(wanted):
                for db in (RAW/sid/'bag').glob('*.db3'):
                    with sqlite3.connect(db.resolve().as_uri()+'?mode=ro',uri=True) as conn:
                        tid,kind=conn.execute('select id,type from topics where name=?',(topic,)).fetchone()
                        for rx,raw in conn.execute('select timestamp,data from messages where topic_id=? and timestamp between ? and ? order by timestamp',
                                                   (tid,int(wanted.min()),int(wanted.max()))):
                            m=store.deserialize_cdr(raw,kind); stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
                            src=(stamp-anchor)/1e9
                            if not start-.15<=src<=end+.15:continue
                            pc=preview_points(m,limit=6000)
                            xyz=np.asarray(pc['points'],dtype=float).reshape(-1,3)
                            length=np.linalg.norm(xyz,axis=1);xyz=xyz[(length<12)&(length>.15)]
                            rows.append(dict(src=src,rx=(rx-anchor)/1e9,frame=pc['frame_id'],n=len(xyz),total=pc['total'],
                                             xyz_mm_b64=base64.b64encode(np.round(xyz*1000).astype('<i2').tobytes()).decode('ascii')))
            clouds[topic]=sorted(rows,key=lambda r:r['src'])
        dt=np.diff(actual); dt=dt[dt>0]
        return dict(start_s=start,end_s=end,frames=frames,clouds=clouds,native_joint_samples=len(actual),
                    joint_median_hz=float(1/np.median(dt)) if len(dt) else None,
                    note='原关节采样时刻，无补造测量；点云保留原帧率，每帧最多抽样6000点；无地图配准')


class Handler(SimpleHTTPRequestHandler):
    def __init__(self,*a,**kw): super().__init__(*a,directory=str(OUT),**kw)
    def send_json(self,value,status=200):
        raw=json.dumps(value,ensure_ascii=False,allow_nan=False).encode()
        self.send_response(status);self.send_header('Content-Type','application/json; charset=utf-8')
        self.send_header('Content-Length',str(len(raw)));self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(raw)
    def do_GET(self):
        u=urlsplit(self.path);q=parse_qs(u.query)
        try:
            if u.path=='/api/reviews':
                with LOCK: rows=latest(JOURNAL)
                return self.send_json({'reviews':rows,'file':str(JOURNAL)})
            if u.path=='/api/detail':return self.send_json(detail(q['id'][0],float(q['start'][0]),float(q['end'][0])))
            return super().do_GET()
        except (ValueError,KeyError,TypeError) as e:self.send_json({'error':str(e)},400)
        except Exception as e:self.send_json({'error':str(e)},500)
    def do_POST(self):
        if urlsplit(self.path).path!='/api/reviews':return self.send_json({'error':'未知接口'},404)
        origin=self.headers.get('Origin')
        if origin and origin not in ('http://'+self.headers.get('Host',''),):return self.send_json({'error':'只允许同源保存'},403)
        try:
            if not self.headers.get('Content-Type','').startswith('application/json'):raise ValueError('需要 JSON 请求')
            length=int(self.headers.get('Content-Length',0))
            if not 0<length<=1000000:raise ValueError('无效请求大小')
            self.send_json({'saved':append_batch(json.loads(self.rfile.read(length)),JOURNAL)})
        except (ValueError,KeyError,TypeError) as e:self.send_json({'error':str(e)},400)
        except Exception as e:self.send_json({'error':str(e)},500)


def selfcheck():
    global JOURNAL
    import tempfile
    import urllib.request
    r=next(iter(CATALOG)); entry=dict(id=str(uuid.uuid4()),recording_id=r,start_s=1.,end_s=2.,level='segment',
        terrain='unknown',direction='unknown',outcome='uncertain',human_review_status='pending',action_phase='unknown')
    with tempfile.TemporaryDirectory() as temp:
        path=Path(temp)/'test.jsonl';append_batch([entry],path)
        try:append_batch([entry,{**entry,'end_s':-1}],path)
        except ValueError:pass
        else:raise AssertionError('Invalid batch accepted')
        assert len(path.read_text().splitlines())==1
        append_batch([{**entry,'notes':'修订'}],path);assert len(latest(path))==1 and latest(path)[0]['notes']=='修订'
        append_batch([{**entry,'retired':True}],path);assert not latest(path)
        original=JOURNAL;JOURNAL=path
        server=ThreadingHTTPServer(('127.0.0.1',0),Handler);thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        try:
            url=f'http://127.0.0.1:{server.server_port}/api/reviews'
            request=urllib.request.Request(url,data=json.dumps([entry]).encode(),headers={'Content-Type':'application/json'})
            with urllib.request.urlopen(request) as response:assert len(json.load(response)['saved'])==1
            with urllib.request.urlopen(url) as response:assert json.load(response)['reviews'][0]['id']==entry['id']
        finally:server.shutdown();server.server_close();JOURNAL=original
    for suffix,hz in [('151135_f45a816f4062',200),('154856_5809460fe5ab',50)]:
        item=detail('gait_20260909_'+suffix,1,1.3)
        assert abs(item['joint_median_hz']-hz)<3 and any(f['pose'] for f in item['frames'])
        json.dumps(item,allow_nan=False)
    missing=detail('gait_20260909_150921_9ed8457a8250',70,70.3)
    assert all(f['pose'] is None for f in missing['frames']) and not any(missing['clouds'].values())
    print('PASS: HTTP save/read uses a temporary journal; invalid batches do not write; revisions and retirement; native 200/50Hz detail; stale samples hidden.')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--port',type=int,default=8767);parser.add_argument('--selfcheck',action='store_true')
    args=parser.parse_args()
    if args.selfcheck:selfcheck()
    else:
        print(f'Review: http://127.0.0.1:{args.port}/index.html',flush=True)
        ThreadingHTTPServer(('127.0.0.1',args.port),Handler).serve_forever()
