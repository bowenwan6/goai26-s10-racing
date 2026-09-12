"""Build conservative candidate indexes and a static local replay (no server required).
Run: .venv-win/Scripts/python.exe -B artifacts/s10-recording-review-20260909/build.py
"""
import csv
import html
import json
from pathlib import Path
import sys
import numpy as np
import mujoco
from review import OUT, REPO, RAW, dump, key
import validate_motion as replay

LABELS={0:'等待候选',1:'行走/调整候选',2:'转向候选',3:'姿态变化/越障候选',
        4:'大倾角/恢复待确认',5:'控制退出后的记录',6:'数据中断/不可判断'}

def angles(q):
    x,y,z,w=np.asarray(q).T
    return np.degrees(np.array([np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y)),
        np.arcsin(np.clip(2*(w*y-z*x),-1,1)),np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))])).T

def nearest(times, target):
    i=np.searchsorted(times,target).clip(0,len(times)-1); prev=np.maximum(i-1,0)
    return np.where(abs(times[prev]-target)<abs(times[i]-target),prev,i)

def blocks(values):
    edges=np.r_[0,np.flatnonzero(np.diff(values)!=0)+1,len(values)]
    return [(int(a),int(b),int(values[a])) for a,b in zip(edges[:-1],edges[1:])]

def features(r,d):
    anchor=int(r['anchor_ns']);duration=r['duration_s'];tt=np.arange(0,duration,.5)
    it=(d['IMU_src']-anchor)/1e9; jt=(d['JOINTS_DATA_src']-anchor)/1e9;mt=(d['MOTION_INFO_src']-anchor)/1e9
    rp=angles(d['IMU_v'][:,:4]);jv=d['JOINTS_DATA_v'];mv=d['MOTION_INFO_v']
    leg=[i for i in range(16) if i%4!=3];rows=[];codes=[]
    for a in tt:
        ii=(it>=a)&(it<a+.5); jj=(jt>=a)&(jt<a+.5); mm=(mt>=a)&(mt<a+.5)
        if not ii.any() or not jj.any() or not mm.any():
            rows.append({'t':float(a),'valid':False});codes.append(6);continue
        roll,pitch=np.median(rp[ii,:2],axis=0);tilt=float(np.max(abs(rp[ii,:2])))
        leg_rms=float(np.sqrt(np.mean(jv[jj,16:32][:,leg]**2)))
        wheel=float(np.mean(abs(jv[jj,16:32][:,[3,7,11,15]])))
        yaw=float(np.median(abs(d['IMU_v'][ii,6])))
        state=int(mv[np.flatnonzero(mm)[-1],4]);vx=float(np.median(mv[mm,0]))
        if state!=17: c=5
        elif tilt>45:c=4
        elif abs(pitch)>8 and (leg_rms>.3 or wheel>.6):c=3
        elif yaw>.35:c=2
        elif leg_rms<.2 and wheel<.6 and yaw<.1:c=0
        else:c=1
        rows.append({'t':float(a),'valid':True,'roll':float(roll),'pitch':float(pitch),'max_tilt':tilt,
            'leg_dq_rms':leg_rms,'wheel_dq_abs':wheel,'gyro_z_abs':yaw,'reported_vx':vx,'state':state})
        codes.append(c)
    codes=np.array(codes)
    # ponytail: 0.5s heuristic bins; exact contacts and human intervention need raw-data review.
    # Merge isolated quiet/motion/turn bins only; retain every tilt, dropout and control exit.
    for a,b,c in blocks(codes.copy()):
        if b-a==1 and c in (0,1,2) and a>0 and b<len(codes) and codes[a-1]==codes[b]:codes[a:b]=codes[a-1]
    return rows,codes

def load_notes():
    p=OUT/'observations.json'
    return json.loads(p.read_text(encoding='utf-8')) if p.exists() else {}

def index_record(r,rows,codes,notes):
    sid=r['id'];dur=r['duration_s'];note=notes.get(sid[14:20],{})
    # Keep stair flights, waiting and landing candidates connected. Gait changes never split a pass.
    splits=[0.0,dur]+note.get('passage_boundary_candidates_s',[])
    exits=[x['start_s'] for x in r['control_timeline'] if x['values'][0]!=17]
    if exits:splits.append(min(exits))
    resumes=[x['start_s'] for x in r['control_timeline'] if x['values'][0]==17 and exits and x['start_s']>min(exits)+1]
    if resumes:splits.append(min(resumes))
    splits=sorted(set(round(float(v),9) for v in splits if 0<=v<=dur))
    if splits[-1]!=dur:splits[-1]=dur
    passages=[];segments=[];events=[]
    for i,(a,b) in enumerate(zip(splits[:-1],splits[1:]),1):
        pid=f'{sid}-P{i:02d}'
        states=[x['values'][0] for x in r['control_timeline'] if x['start_s']<b and x['end_s']>a+1e-6]
        context=bool(states) and 17 not in states
        outcome='interrupted_control' if any(abs(b-e)<1e-6 for e in exits) else 'uncertain'
        stationary=bool(note.get('stationary_context'))
        passages.append({'recording_id':sid,'passage_id':pid,'start_s':a,'end_s':b,
            'time_basis':'source_stamp_minus_manifest_started_wall_ns','anchor_ns':r['anchor_ns'],
            'kind':'post_interruption_context' if context else 'stationary_context' if stationary else 'candidate_continuous_passage',
            'boundary_status':'candidate_not_human_confirmed','outcome':outcome,
            'obstacle_id':None,'repeat_group_id':None,'human_review_status':'pending',
            'evidence':'状态退出/恢复仅隔开控制中断上下文；其他通行分界为点云+姿态活动间歇提出的候选' if exits or len(splits)>2 else '录制内暂保留连续关系；实际通行数与起终点待复核',
            'notes':'此候选可能包含多个梯段、平台、往返或重试；未确认的重复障碍保持未关联。'})
    boundaries=sorted(set([0.,dur]+[min(a*.5,dur) for a,b,c in blocks(codes)][1:]+splits[1:-1]+note.get('segment_boundaries_s',[])))
    for i,(a,b) in enumerate(zip(boundaries[:-1],boundaries[1:]),1):
        if b-a<1e-7:continue
        c=int(codes[min(int(((a+b)/2)/.5),len(codes)-1)])
        pid=next(p['passage_id'] for p in passages if p['start_s']<b-1e-7 and p['end_s']>a+1e-7)
        rr=[v for v in rows if a<=v['t']<b and v['valid']]
        evidence={'automatic_class':LABELS[c], 'bin_s':.5}
        if rr:
            evidence.update(pitch_median_deg=round(float(np.median([v['pitch'] for v in rr])),2),
                max_abs_roll_pitch_deg=round(max(v['max_tilt'] for v in rr),2),
                leg_dq_rms_median=round(float(np.median([v['leg_dq_rms'] for v in rr])),3),
                wheel_dq_abs_median=round(float(np.median([v['wheel_dq_abs'] for v in rr])),3))
        terrain='unknown';phase=LABELS[c];direction='unknown';checked=[]
        for obs in note.get('observations',[]):
            if obs['start_s']<b and obs['end_s']>a:
                checked.append(obs['evidence'])
                if c in (0,1,2,3) and obs.get('terrain_candidate'):
                    terrain=obs['terrain_candidate']
                    if c==3:
                        phase=obs.get('action_candidate',phase)
                        direction=obs.get('direction_candidate','unknown')
                    phase=obs.get('phase_by_class',{}).get(str(c),phase)
        outcome='interrupted_data' if c==6 else 'uncertain'
        if any(a<e<=b+1e-7 for e in exits):outcome='interrupted_control'
        segments.append({'recording_id':sid,'raw_files':[db['path'] for db in r['databases']],
            'passage_id':pid,'segment_id':f'{sid}-S{i:03d}','start_s':a,'end_s':b,
            'time_basis':'source_stamp_minus_manifest_started_wall_ns','anchor_ns':r['anchor_ns'],
            'start_source_ns':str(int(r['anchor_ns'])+round(a*1e9)),'end_source_ns':str(int(r['anchor_ns'])+round(b*1e9)),
            'terrain':terrain,'action_phase':phase,'direction':direction,'obstacle_id':None,'flight_id':None,'platform_id':None,
            'outcome':outcome,'evidence':evidence,'visual_evidence':checked,
            'judgement_status':'model_visual_checked_candidate' if checked else 'automatic_candidate',
            'human_review_status':'pending','expert_sample_status':'not_assessed',
            'notes':'边界约 ±0.5–1s；未知地面可能为平地、平台或障碍上的停顿；整段 outcome 未传播。',
            'preview':f'viewer.html?id={sid}&t={a:.3f}&end={b:.3f}'})
    for s in segments[1:]:events.append({'t':s['start_s'],'kind':'phase_candidate','label':s['action_phase'],'segment_id':s['segment_id']})
    for c in r['control_timeline'][1:]:events.append({'t':c['start_s'],'kind':'actual_control','label':f'实际 state={c["values"][0]}, gait=0x{c["values"][1]:04X}'})
    for c in r['gait_commands']:events.append({'t':c['receive_s'],'kind':'gait_command','label':f'/GAIT 指令 0x{c["code"]:04X}（接收时刻；源戳无效时不使用）'})
    return passages,segments,sorted(events,key=lambda e:e['t'])

def replay_data(r,d,model,sample_times=None,modes=('src','rx')):
    cached=OUT/'data'/f'{r["id"]}.js'
    if sample_times is None and '--reuse-replay' in sys.argv and cached.exists():
        return json.loads(cached.read_text(encoding='utf-8').removeprefix('window.RECORDING=').removesuffix(';'))['frames']
    anchor=int(r['anchor_ns']);duration=r['duration_s'];tt=np.arange(0,duration,.1) if sample_times is None else sample_times
    direction,offset=replay.calibration();qa=model.jnt_qposadr[model.actuator_trnid[:,0]]
    md=mujoco.MjData(model);out={}
    for mode in modes:
        times={k:(d[k+'_'+mode]-anchor)/1e9 for k in ('IMU','JOINTS_DATA','MOTION_INFO')}
        ids={k:nearest(v,tt) for k,v in times.items()}
        qraw=d['JOINTS_DATA_v'][ids['JOINTS_DATA'],:16]
        q=qraw*direction+offset;q[:,replay.WHEELS]-=(d['JOINTS_DATA_v'][0,:16]*direction+offset)[replay.WHEELS]
        # Keep one heading origin for the whole recording, including detail windows.
        quat=replay.root_quaternions(np.vstack([d['IMU_v'][0,:4],d['IMU_v'][ids['IMU'],:4]]))[1:]
        rp=angles(d['IMU_v'][ids['IMU'],:4]);rows=[]
        for i,t in enumerate(tt):
            valid={k:abs(times[k][ids[k][i]]-t)<=(.075 if k=='MOTION_INFO' else .03) for k in ids}
            pose=None
            if valid['IMU'] and valid['JOINTS_DATA']:
                md.qpos[:3]=0;md.qpos[3:7]=quat[i];md.qpos[qa]=q[i]
                mujoco.mj_kinematics(model,md)
                pose=np.round(md.xpos[1:],4).tolist()
            row={'t':round(float(t),6),'pose':pose,'imu_quat_aligned':quat[i].tolist() if valid['IMU'] else None,'imu_rpy':np.round(rp[i],2).tolist() if valid['IMU'] else None,
                'joints':np.round(qraw[i],3).tolist() if valid['JOINTS_DATA'] else None,
                'motion':np.round(d['MOTION_INFO_v'][ids['MOTION_INFO'][i]],4).tolist() if valid['MOTION_INFO'] else None,
                'samples':{k:{'src':round(float((d[k+'_src'][ids[k][i]]-anchor)/1e9),6),
                              'rx':round(float((d[k+'_rx'][ids[k][i]]-anchor)/1e9),6),
                              'valid':bool(valid[k])} for k in ids}}
            rows.append(row)
        out[mode]=rows
    return out

def csvwrite(path,rows):
    if not rows:return
    with path.open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader()
        for row in rows:w.writerow({k:json.dumps(v,ensure_ascii=False) if isinstance(v,(dict,list)) else v for k,v in row.items()})

def selfcheck():
    assert nearest(np.array([0.,1.,2.]),np.array([-.5,.7,4.])).tolist()==[0,1,2]
    assert blocks(np.array([0,0,1,0]))==[(0,2,0),(2,3,1),(3,4,0)]
    assert np.allclose(angles(np.array([[0,0,0,1]])),0)

def main():
    selfcheck();(OUT/'data').mkdir(exist_ok=True)
    rs=json.loads((OUT/'inventory.json').read_text(encoding='utf-8'));notes=load_notes()
    replay.OUT=OUT/'data';model=replay.make_model()
    parents=model.body_parentid[1:]-1
    names=[mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_BODY,i) for i in range(1,model.nbody)]
    allp=[];alls=[];allq=[];catalog=[];topicrows=[];rate_rows=[];control_rows=[]
    for r in rs:
        sid=r['id'];d=np.load(OUT/'decoded'/f'{sid}.npz');rows,codes=features(r,d)
        ps,ss,events=index_record(r,rows,codes,notes);allp+=ps;alls+=ss
        for topic,s in r['streams'].items():
            topicrows.append({'recording_id':sid,'topic':topic,**{k:v for k,v in s.items() if k not in ('source_gaps','receive_stalls','rate_windows')},
                'source_long_gap_count':len(s['source_gaps']),'receive_stall_count':len(s['receive_stalls'])})
            rate_rows.extend({'recording_id':sid,'topic':topic,**w} for w in s['rate_windows'])
        control_rows.extend({'recording_id':sid,'time_basis':'source_stamp_minus_manifest_started_wall_ns',
            'start_s':c['start_s'],'end_s':c['end_s'],'state':c['values'][0],'gait':c['values'][1]} for c in r['control_timeline'])
        payload={'id':sid,'duration':r['duration_s'],'anchor_ns':r['anchor_ns'],'manifest':r['manifest'],
            'frames':replay_data(r,d,model),'clouds':json.loads((OUT/'decoded'/f'{sid}.clouds.json').read_text(encoding='utf-8')),
            'body_names':names,'body_parents':parents.tolist(),'segments':ss,'passages':ps,'events':events,
            'control':r['control_timeline'],'issues':r['issues'],'observations':notes.get(sid[14:20],{})}
        (OUT/'data'/f'{sid}.js').write_text('window.RECORDING='+json.dumps(payload,ensure_ascii=False,separators=(',',':'),allow_nan=False)+';',encoding='utf-8')
        note=notes.get(sid[14:20],{})
        for a,b,question in note.get('review_intervals',[]):
            b=min(b,r['duration_s']);a=max(a,0)
            allq.append({'recording_id':sid,'start_s':a,'end_s':b,'time_basis':'source_stamp_minus_manifest_started_wall_ns',
                'question':question,'human_review_status':'pending','preview':f'viewer.html?id={sid}&t={a:.3f}&end={b:.3f}',
                'signals':f'previews/{sid}_signals.png','cloud_sheet':f'previews/{sid}_clouds.jpg'})
        catalog.append({'recording_id':sid,'duration_s':r['duration_s'],'terrain_label':r['manifest']['terrain'],
            'parameters_recorded_unverified':r['manifest']['parameters'],'recording_outcome_label':r['manifest']['outcome'],
            'topic_count':len(r['streams']),'candidate_passages':sum(p['kind']=='candidate_continuous_passage' for p in ps),
            'context_blocks':sum(p['kind']!='candidate_continuous_passage' for p in ps),'segments':len(ss),
            'issues':r['issues'],'raw_path':r['raw_path'],'preview':f'viewer.html?id={sid}'})
        dump(OUT/'data'/f'{sid}.features.json',rows)
        print('BUILD '+sid+f' {len(ps)} groups / {len(ss)} segments',flush=True)
    for filename,records in [('recordings',catalog),('passages',allp),('segments',alls),('review_queue',allq)]:
        dump(OUT/f'{filename}.json',records);csvwrite(OUT/f'{filename}.csv',records)
    csvwrite(OUT/'topics.csv',topicrows);csvwrite(OUT/'sampling_windows.csv',rate_rows);csvwrite(OUT/'control_timeline.csv',control_rows)
    (OUT/'catalog.js').write_text('window.CATALOG='+json.dumps(catalog,ensure_ascii=False)+';',encoding='utf-8')
    for r in rs:
        ss=[s for s in alls if s['recording_id']==r['id']]
        assert ss[0]['start_s']==0 and abs(ss[-1]['end_s']-r['duration_s'])<1e-8
        assert all(abs(a['end_s']-b['start_s'])<1e-8 for a,b in zip(ss[:-1],ss[1:]))
        assert all(s['start_s']<s['end_s'] and s['human_review_status']=='pending' for s in ss)
        assert all(s['outcome'] not in ('success','failure') for s in ss)
    dump(OUT/'validation.json',{'recordings':len(rs),'duration_s':sum(r['duration_s'] for r in rs),
        'passage_groups':len(allp),'segments':len(alls),'questions':len(allq),
        'checks':['nearest sample boundary cases','identity quaternion','complete non-overlapping segment coverage',
                  'no manifest outcome propagation','all human review pending','all generated JSON finite'],
        'source_access':'SQLite mode=ro; prior verified.json reused; no source file writes or hashes'})

if __name__=='__main__':main()
