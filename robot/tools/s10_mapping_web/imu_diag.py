"""Read-only evidence recorder. No ROS publishers, calibration or robot commands.

Owned by the durable field worker, NOT an HTTP request/SSH connection. Only
already-published messages are accepted; source CDR is retained during captures.
"""
import base64
import csv
import hashlib
import html
import json
import math
import os
import queue
import re
import shutil
import threading
import time
import uuid
import zipfile
from collections import deque
from pathlib import Path

from field_core import FieldError


def finite(v):
    if isinstance(v, float): return v if math.isfinite(v) else None
    if isinstance(v, dict): return {k: finite(x) for k,x in v.items()}
    if isinstance(v, (tuple,list)): return [finite(x) for x in v]
    return v

def atomic(path, value):
    tmp=path.with_suffix('.partial')
    with tmp.open('w') as f:
        json.dump(finite(value),f,ensure_ascii=False,allow_nan=False);f.flush();os.fsync(f.fileno())
    tmp.replace(path)

def valid_id(value):
    if not isinstance(value,str) or not re.fullmatch('[a-f0-9]{32}',value):raise FieldError('诊断编号无效')
    return value

class Diagnostic:
    def __init__(self, root, nav=None, reserve=2*1024**3, start_thread=True):
        self.root=Path(root);self.root.mkdir(parents=True,exist_ok=True,mode=0o700)
        self.nav=nav;self.reserve=reserve;self.lock=threading.RLock();self.queue=queue.Queue(4096)
        self.epoch=uuid.uuid4().hex;self.seq=0;self.rows=deque(maxlen=3600);self.latest={};self.buckets={}
        self.counts={};self.dropped=0;self.error=None;self.watch_until=0.;self.active=None;self.file=None
        self.boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip() if Path('/proc/sys/kernel/random/boot_id').exists() else 'test-host'
        from imu_zero import ZeroReference
        self.zero=ZeroReference(self.root,self.epoch,self.boot,atomic)
        for path in self.root.glob('*/status.json'):
            if path.is_symlink():continue
            try:
                s=json.loads(path.read_text())
                if s['state']=='RUNNING':
                    s.update(state='INTERRUPTED',reason='采集进程重启；部分源数据保留，不冒充完成');atomic(path,s)
            except (ValueError,KeyError):pass
        self.shutdown=threading.Event()
        if start_thread:threading.Thread(target=self.loop,daemon=True,name='imu-diagnostic').start()

    def wants(self):return self.active is not None or self.zero.wants() or time.monotonic()<self.watch_until

    def ingest(self,row):
        if not self.wants():return
        row=finite(row)
        session=self.active  # Completion may clear active on another thread.
        row['session_id']=session['id'] if session is not None else None
        try:self.queue.put_nowait(row)
        except queue.Full:self.dropped+=1

    def consume(self,row):
        topic=row['topic'];t=row['received'];old=self.latest.get(topic)
        row['time_warning']=bool(old and ((row.get('stamp') is not None and old.get('stamp') is not None and
            (row['stamp']<=old['stamp'] or abs((row['stamp']-old['stamp'])-(t-old['received']))>.1)) or row.get('frame')!=old.get('frame')))
        self.latest[topic]={k:v for k,v in row.items() if k!='cdr_b64'};self.counts[topic]=self.counts.get(topic,0)+1
        self.zero.observe(row)
        if self.active and row.get('session_id')==self.active['id'] and row['received']<=self.active['started_mono']+self.active['seconds']:
            self.file.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n')
            self.active['samples']+=1
            if row['time_warning']:self.active['time_warnings']+=1
            if any(x is None for x in row.get('values',[])):self.active['invalid_values']+=1
        key=int(t*10);b=self.buckets.get(topic);v=row.get('values',[])
        if not b or b['key']!=key or len(v)!=len(b['min']):
            if b:
                self.seq+=1;b['seq']=self.seq;self.rows.append(b)
            b=dict(topic=topic,key=key,t=t,stamp=row.get('stamp'),min=list(v),max=list(v),mean=list(v),valid=[int(x is not None) for x in v],n=0)
            self.buckets[topic]=b
        else:
            for i,x in enumerate(v):
                if x is None:continue
                n=b['valid'][i];b['valid'][i]=n+1
                b['min'][i]=x if n==0 else min(b['min'][i],x);b['max'][i]=x if n==0 else max(b['max'][i],x)
                b['mean'][i]=x if n==0 else b['mean'][i]+(x-b['mean'][i])/(n+1)
        b['n']+=1
        b['discontinuity']=b.get('discontinuity',False) or row['time_warning']

    def tick(self):
        with self.lock:
            for _ in range(2000):
                try:self.consume(self.queue.get_nowait())
                except queue.Empty:break
            self.zero.tick(self.latest,time.monotonic(),self.dropped)
            if self.active:
                elapsed=time.monotonic()-self.active['started_mono'];self.active['elapsed_s']=elapsed
                self.active['queue_dropped']=self.dropped-self.active['drop_baseline']
                if elapsed>=self.active['seconds']:self.finish('duration_complete')
                elif self.file.tell()>256*1024**2:self.finish('size_limit')
                elif shutil.disk_usage(self.root).free<self.reserve:self.finish('disk_reserve')
                elif elapsed-self.active.get('flushed_at',0)>=1:
                    self.file.flush();os.fsync(self.file.fileno());self.active['flushed_at']=elapsed
                    atomic(self.root/self.active['id']/'status.json',self.active)

    def loop(self):
        last_nav=0.
        while not self.shutdown.wait(.05):
            try:
                if self.nav and self.wants() and time.monotonic()-last_nav>=1:
                    last_nav=time.monotonic();n=self.nav()
                    self.ingest(dict(topic='motion',received=time.monotonic(),received_wall=time.time(),stamp=n.get('stamp'),
                        values=(n.get('motion') or [None]*3)+(n.get('command') or [None]*3),navigation=n))
                self.tick()
            except Exception as exc:
                with self.lock:
                    self.error=str(exc)
                    if self.active:
                        s=dict(self.active,state='FAILED',reason='采集/写入异常：'+str(exc))
                        if self.file:self.file.close()
                        self.file=None;self.active=None
                        try:atomic(self.root/s['id']/'status.json',s)
                        except OSError:pass

    def finish(self,reason):
        if not self.active:return
        s=self.active;self.file.flush();os.fsync(self.file.fileno());self.file.close();self.file=None;self.active=None
        s.update(state='COMPLETED' if reason=='duration_complete' else 'PARTIAL',reason=reason,ended_wall=time.time(),queue_dropped=self.dropped-s['drop_baseline'])
        folder=self.root/s['id'];atomic(folder/'status.json',s)
        # Analysis uses every received sample, NOT decimated web points.
        windows={};topic_counts={};timing={};time_warnings=0;invalid=0
        with (folder/'samples.jsonl').open() as f,(folder/'samples.csv').open('x',newline='') as cf:
            writer=csv.writer(cf);writer.writerow(['topic','source_stamp_s','receive_monotonic_s','receive_wall_s','values_json','time_warning'])
            for line in f:
                row=json.loads(line);topic=row['topic'];topic_counts[topic]=topic_counts.get(topic,0)+1
                tt=timing.setdefault(topic,dict(first_receive=row['received'],last_receive=row['received'],max_receive_gap_s=0.,source_first=row.get('stamp'),source_last=row.get('stamp')))
                tt['max_receive_gap_s']=max(tt['max_receive_gap_s'],row['received']-tt['last_receive']);tt['last_receive']=row['received'];tt['source_last']=row.get('stamp')
                writer.writerow([topic,row.get('stamp'),row['received'],row.get('received_wall'),json.dumps(row.get('values',[])),row.get('time_warning')])
                time_warnings+=int(row.get('time_warning',False));invalid+=sum(x is None for x in row.get('values',[]))
                k=(topic,int((row['received']-s['started_mono'])//10));w=windows.setdefault(k,dict(topic=topic,window_s=k[1]*10,axes=[]))
                for i,x in enumerate(row.get('values',[])):
                    while len(w['axes'])<=i:w['axes'].append(dict(n=0,mean=0.,m2=0.,min=None,max=None))
                    if x is None:continue
                    a=w['axes'][i];a['n']+=1;d=x-a['mean'];a['mean']+=d/a['n'];a['m2']+=d*(x-a['mean'])
                    a['min']=x if a['min'] is None else min(a['min'],x);a['max']=x if a['max'] is None else max(a['max'],x)
        for w in windows.values():
            for a in w['axes']:a['std']=math.sqrt(max(0,a.pop('m2'))/max(1,a['n']-1));a['mean']=a['mean'] if a['n'] else None
        for topic,tt in timing.items():
            tt['start_missing_s']=max(0,tt['first_receive']-s['started_mono']);tt['end_missing_s']=max(0,min(s['seconds'],s['elapsed_s'])+s['started_mono']-tt['last_receive'])
        report=dict(session=s,topic_counts=topic_counts,timing=timing,missing_topics=[t for t in ['imu','odom','motion'] if not topic_counts.get(t)],windows=list(windows.values()),invalid_components=invalid,time_warnings=time_warnings,
            calibration='UNVERIFIED',stationarity='UNDETERMINED',hardware_raw_access_verified=False,
            conclusion='仅记录观测及10秒窗口统计；未判定IMU故障或完成校准，不放行原App门禁。',
            limits=['/IMU是驱动发布值，不证明未经标定的硬件原始值','没有硬件序号时不能精确估计传感器丢包','单站姿不能完成加速度标定','X/Y/yaw反馈单位和上游估计器尚待核实'])
        atomic(folder/'report.json',report)
        text=html.escape(json.dumps(report,ensure_ascii=False,indent=2));charts=[]
        # Offline curves show 10-second aggregates explicitly, not a fake raw replay.
        for topic,indices,title in [('imu',[0,1,2],'角速度：X/Y/Z'),('imu',[3,4,5],'加速度：X/Y/Z'),('motion',[0,1,2],'运动反馈：X/Y/转向（单位未核实）')]:
            ws=sorted([w for w in windows.values() if w['topic']==topic],key=lambda w:w['window_s'])
            values=[w['axes'][i]['mean'] for w in ws for i in indices if i<len(w['axes']) and w['axes'][i]['mean'] is not None]
            if not values:continue
            lo=min(0,min(values));hi=max(0,max(values));pad=max((hi-lo)*.1,1e-5);lo-=pad;hi+=pad
            lines=[]
            for i,color in zip(indices,['#c84639','#158160','#315de0']):
                points=[];segments=[];previous=None
                for w in ws:
                    if i<len(w['axes']) and w['axes'][i]['mean'] is not None:
                        if previous is not None and w['window_s']-previous>10:
                            segments.append(points);points=[]
                        x=60+640*max(0,w['window_s'])/max(10,s['seconds']);y=160-140*(w['axes'][i]['mean']-lo)/(hi-lo);points.append(f'{x:.2f},{y:.2f}')
                        previous=w['window_s']
                    else:
                        segments.append(points);points=[];previous=None
                segments.append(points)
                for segment in segments:
                    lines.append(f'<polyline fill="none" stroke="{color}" stroke-width="2" points="'+ ' '.join(segment)+'"/>')
            charts.append(f'<h2>{title}</h2><svg viewBox="0 0 740 200" role="img" aria-label="10秒窗口均值曲线"><text x="0" y="20">{hi:.5f}</text><text x="0" y="163">{lo:.5f}</text><text x="60" y="190">0秒</text><text x="670" y="190">{s["seconds"]}秒</text>'+''.join(lines)+'</svg>')
        (folder/'report.html').write_text('<!doctype html><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>IMU诊断报告</title><style>body{max-width:900px;margin:24px auto;padding:16px;font:16px/1.6 system-ui}svg{width:100%;background:#f4f7fa}pre{white-space:pre-wrap;overflow-wrap:anywhere}</style><h1>IMU只读诊断</h1><p>未校准，不控制机器人。下图为10秒窗口均值，不是原频率曲线；缺失窗口不得据此推断连续。完整采样见CSV/JSONL，标准差与时序见下方。</p>'+''.join(charts)+'<details><summary>完整诊断统计（JSON）</summary><pre>'+text+'</pre></details>')
        with zipfile.ZipFile(folder/'diagnostic.zip','x',zipfile.ZIP_DEFLATED) as z:
            for name in ['status.json','manifest.json','samples.jsonl','samples.csv','report.json','report.html']:z.write(folder/name,name)

    def rpc(self,request):
        with self.lock:
            action=request.get('action')
            allowed={'live':{'action','cursor'},'start':{'action','key','seconds','stationary','remote_ready'},'stop':{'action','id'},
                'event':{'action','id','label'},'list':{'action'},'report':{'action','id'},'artifact_info':{'action','artifact_id'},'artifact_read':{'action','artifact_id','offset','length'},
                'zero_start':{'action','key','epoch','stationary','remote_ready'},'zero_clear':{'action','id'}}
            if action not in allowed or set(request)-allowed[action]:raise FieldError('无效诊断操作/参数')
            if action=='live':
                cursor=request.get('cursor',0)
                if type(cursor) is not int or cursor<0:raise FieldError('游标无效')
                self.watch_until=time.monotonic()+30;now=time.monotonic()
                self.zero.tick(self.latest,now,self.dropped)
                return finite(dict(epoch=self.epoch,seq=self.seq,board_mono=now,rows=[b for b in self.rows if b['seq']>cursor][-600:],
                    latest={k:dict(v,age=now-v['received']) for k,v in self.latest.items()},counts=self.counts,queue_dropped=self.dropped,
                    active=self.active,error=self.error,calibration='未验证',robot_control=False,preview_hz=10,
                    zero=self.zero.snapshot(self.latest,now)))
            if action=='zero_start':
                key=valid_id(request.get('key'))
                if request.get('stationary') is not True or request.get('remote_ready') is not True:
                    raise FieldError('请重新确认本次停稳、外部策略退出和手柄可接管')
                if request.get('epoch')!=self.epoch:raise FieldError('采集服务会话已变化，请刷新后重新确认','conflict')
                if self.active:raise FieldError('限时检查正在采集，结束后才能建立新零点','busy')
                if self.error:raise FieldError('采集器异常，不能复零：'+self.error,'unavailable')
                if shutil.disk_usage(self.root).free<self.reserve:raise FieldError('磁盘空间不足，不能保存参考证据','unavailable')
                return self.zero.start(key,self.latest,time.monotonic(),self.dropped)
            if action=='zero_clear':
                sid=valid_id(request.get('id'))
                if sid!=self.zero.state.get('id'):raise FieldError('零点已被其他会话替换，请刷新','conflict')
                self.zero.state.update(state='CLEARED',reason='已清除参考；原始读数不变。下次停稳后重新勾选并复零')
                self.zero.save();return dict(self.zero.state)
            if action=='list':
                files=sorted(self.root.glob('*/status.json'),key=lambda p:p.stat().st_mtime,reverse=True)[:50]
                return dict(sessions=[json.loads(p.read_text()) for p in files if not p.is_symlink()])
            if action=='start':
                key=valid_id(request.get('key'));seconds=request.get('seconds')
                if type(seconds) is not int or seconds not in (60,180,300):raise FieldError('时长只能60/180/300秒')
                if request.get('stationary') is not True or request.get('remote_ready') is not True:raise FieldError('先确认人工停稳、外部策略退出和手柄可接管')
                folder=self.root/key
                if folder.is_symlink():raise FieldError('诊断目录不能是符号链接')
                if (folder/'status.json').exists():
                    old=json.loads((folder/'status.json').read_text())
                    if old['seconds']!=seconds:raise FieldError('同一请求编号不能改变时长','conflict')
                    return old
                if self.active:raise FieldError('已有诊断采样进行中','busy')
                if self.zero.state['state'] in ('SAMPLING','VALIDATING'):raise FieldError('静止复零正在采样／验证，请等待结束','busy')
                if self.error:raise FieldError('采集器异常，暂不能开始：'+self.error,'unavailable')
                if time.monotonic()-self.latest.get('imu',{}).get('received',-999)>1:raise FieldError('先查看实时数据，等待新鲜IMU消息','unavailable')
                if shutil.disk_usage(self.root).free<self.reserve:raise FieldError('磁盘空间不足','unavailable')
                folder.mkdir(mode=0o700);self.file=(folder/'samples.jsonl').open('x')
                self.zero.invalidate('开始新的限时检查；原零点失效，检查仍保存未复零的原始值')
                self.active=dict(id=key,state='RUNNING',seconds=seconds,started_mono=time.monotonic(),started_wall=time.time(),boot_id=self.boot,epoch=self.epoch,
                    elapsed_s=0,samples=0,time_warnings=0,invalid_values=0,queue_dropped=0,drop_baseline=self.dropped,events=[],stationary_human_label=True)
                atomic(folder/'manifest.json',dict(version=1,boot_id=self.boot,epoch=self.epoch,source='106 existing ROS subscriptions and planner monitor',
                    imu_fields=['wx','wy','wz','ax','ay','az'],imu_units=['rad/s']*3+['m/s^2']*3,units_driver_compliance='UNVERIFIED',
                    motion_fields=['x_velocity','y_velocity','yaw_velocity','cmd_x','cmd_y','cmd_yaw'],motion_units='vendor_unverified',
                    qos='qos_profile_sensor_data',cdr_encoding='base64 ROS serialized CDR; driver output, not necessarily hardware raw',calibration='UNVERIFIED',gravity_removed='UNVERIFIED'))
                atomic(folder/'status.json',self.active);return dict(self.active)
            sid=valid_id(request.get('artifact_id') if action.startswith('artifact_') else request.get('id'));folder=self.root/sid
            if folder.is_symlink() or not folder.is_dir():raise FieldError('没有该诊断记录','not_found')
            if action in ('stop','event'):
                if not self.active or self.active['id']!=sid:
                    if action=='stop':return json.loads((folder/'status.json').read_text())
                    raise FieldError('此诊断已结束','conflict')
                if action=='stop':self.tick();self.finish('operator_stop');return json.loads((folder/'status.json').read_text())
                label=request.get('label')
                if label not in ('停稳','移动'):raise FieldError('事件只能是停稳/移动')
                if label=='移动':self.zero.invalidate('人工标记移动；本次静止参考失效')
                if len(self.active['events'])>=100:raise FieldError('事件数量已达上限')
                self.active['events'].append(dict(label=label,mono=time.monotonic(),wall=time.time()));atomic(folder/'status.json',self.active);return dict(self.active)
            if action=='report':
                p=folder/'report.json';return json.loads(p.read_text()) if p.exists() else dict(session=json.loads((folder/'status.json').read_text()),conclusion='未生成完整报告；任务可能进行中或中断，部分源数据保留在106')
            p=folder/'diagnostic.zip'
            if not p.is_file() or p.is_symlink():raise FieldError('完整导出包尚未生成','not_found')
            if action=='artifact_info':
                with p.open('rb') as f:digest=hashlib.file_digest(f,'sha256').hexdigest()
                return dict(name='imu-'+sid+'.zip',size=p.stat().st_size,sha256=digest)
            offset,length=request.get('offset'),request.get('length')
            if type(offset) is not int or type(length) is not int or offset<0 or not 1<=length<=262144 or offset>p.stat().st_size:raise FieldError('读取范围无效')
            with p.open('rb') as f:f.seek(offset);raw=f.read(length)
            return dict(offset=offset,data=base64.b64encode(raw).decode())
