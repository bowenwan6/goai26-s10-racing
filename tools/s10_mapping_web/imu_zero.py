"""Ephemeral DISPLAY reference, not hardware calibration or a safety gate.

Consumes published data only. Never exports corrected values to ROS, navigation,
SLAM, waypoint checks or the controller. Original samples are never changed.
"""
import json
import math
import statistics
import time
from collections import deque
from pathlib import Path

from field_core import FieldError


def vector(v, n):
    return isinstance(v, (tuple, list)) and len(v) == n and all(type(x) in (int, float) and math.isfinite(x) for x in v)


def angle(a, b):
    if not vector(a, 4) or not vector(b, 4):
        return float('inf')
    na=math.sqrt(sum(x*x for x in a)); nb=math.sqrt(sum(x*x for x in b))
    if not .99 < na < 1.01 or not .99 < nb < 1.01:
        return float('inf')
    dot=abs(sum(x*y for x,y in zip(a,b))/(na*nb))
    return math.degrees(2*math.acos(min(1.,dot)))


class ZeroReference:
    FIT=10.; VERIFY=3.; TTL=120.

    def __init__(self, root, epoch, boot, atomic):
        self.root=Path(root)/'zero_references'; self.root.mkdir(exist_ok=True,mode=0o700)
        self.epoch=epoch; self.boot=boot; self.atomic=atomic
        self.state=dict(state='IDLE',reason='尚未建立本次静止参考',scope='DISPLAY_ONLY',hardware_applied=False,safety_gate_changed=False)
        self.samples={'imu':deque(maxlen=6000),'body_motion':deque(maxlen=1000)}
        self.previous={}; self.last_write=-1.; self.probe=None
        latest=self.root/'latest.json'
        if latest.exists():
            try:
                old=json.loads(latest.read_text())
                if not isinstance(old,dict) or 'state' not in old:raise ValueError('invalid state')
            except (ValueError,OSError):
                self.state.update(state='EXPIRED',reason='旧参考记录损坏，未恢复任何零点；可重新建立参考')
                return
            if old.get('state') in ('SAMPLING','VALIDATING','READY'):
                old.update(state='EXPIRED',reason='采集服务已重启，旧零点不再使用')
                self.state=old; self.save()
            else:self.state=old

    def wants(self):
        return self.state['state'] in ('SAMPLING','VALIDATING','READY')

    def save(self):
        self.atomic(self.root/'latest.json',self.state)
        if self.state.get('id'):
            self.atomic(self.root/(self.state['id']+'.json'),self.state)

    def invalidate(self, reason, failed=False):
        if self.wants():
            self.state.update(state='FAILED' if failed or self.state['state']!='READY' else 'EXPIRED',reason=reason)
            self.save()

    def guards(self, latest, now):
        imu=latest.get('imu',{})
        if not -.05 <= now-imu.get('received',-999) <= .7:
            return 'IMU 数据过期；等待实时数据恢复后重新复零'
        if not vector(imu.get('values'),6) or angle(imu.get('quaternion'),imu.get('quaternion')) > .1:
            return 'IMU 角速度、加速度或姿态无效，不能建立参考'
        if any(imu.get(k,[0])[0] == -1 for k in ('orientation_covariance','angular_velocity_covariance','linear_acceleration_covariance')):
            return '驱动标记 IMU 分量不可用，不能建立参考'
        if type(imu.get('stamp')) not in (int,float) or not -.1 <= imu.get('received_wall',0)-imu['stamp'] <= .5:
            return 'IMU 源时间异常／延迟过大，不能复零'
        motion=latest.get('motion',{}); nav=motion.get('navigation',{})
        if not -.05 <= now-motion.get('received',-999) <= 2.5 or not nav.get('fresh'):
            return '规划器状态过期；不能确认命令状态'
        if nav.get('goal_none') is not True or nav.get('planner_code') != 999:
            return '存在导航目标或规划器不空闲；先退出导航'
        if not vector(nav.get('command'),3) or any(x != 0 for x in nav['command']):
            return '规划器有非零命令，拒绝复零；此检查不覆盖全部外部控制通道'
        return None

    def start(self, key, latest, now, dropped):
        path=self.root/(key+'.json')
        if path.is_symlink():raise FieldError('复零记录路径无效')
        if path.exists():return json.loads(path.read_text())
        if self.state['state'] in ('SAMPLING','VALIDATING'):
            raise FieldError('正在复零，请等待 10 秒采样＋3 秒验证','busy')
        reason=self.guards(latest,now)
        if reason:raise FieldError(reason,'unavailable')
        if abs(max(latest['imu']['values'][:3],key=abs))>.04:
            raise FieldError('检测到明显角速度，请完全停稳后再试')
        # Superseding is explicit; a failed new attempt never resurrects an old zero.
        self.invalidate('已被新的静止复零请求替代')
        self.state=dict(id=key,state='SAMPLING',scope='DISPLAY_ONLY',hardware_applied=False,safety_gate_changed=False,
                        epoch=self.epoch,boot_id=self.boot,started_mono=now,started_wall=time.time(),elapsed_s=0.,
                        fit_s=self.FIT,validation_s=self.VERIFY,total_s=self.FIT+self.VERIFY,ttl_s=self.TTL,
                        reason='保持不动：采样 10 秒，再独立验证 3 秒',drop_baseline=dropped,
                        anchor_quaternion=latest['imu']['quaternion'],imu_frame=latest['imu'].get('frame'),
                        planner_invocation=latest['motion']['navigation'].get('invocation'))
        self.samples={'imu':deque(maxlen=6000),'body_motion':deque(maxlen=1000)}
        self.previous={}; self.probe=None; self.last_write=-1.; self.save()
        return dict(self.state)

    def observe(self, row):
        if not self.wants() or row['received'] < self.state['started_mono']:return
        topic=row['topic']
        if topic=='imu':
            v=row.get('values',[])
            if not vector(v,6) or angle(row.get('quaternion'),self.state['anchor_quaternion'])>1.:
                return self.invalidate('IMU 数据无效或姿态变化超过诊断参考范围；此次零点失效')
            if max(abs(x) for x in v[:3])>.04:
                return self.invalidate('检测到转动或过大角速度；请停稳后重新复零')
            if row.get('frame') != self.state['imu_frame']:
                return self.invalidate('IMU 坐标系发生变化；零点失效')
            old=self.previous.get(topic)
            if old:
                ds=row.get('stamp',0)-old.get('stamp',0);dt=row['received']-old['received']
                if ds<=0 or ds>.25 or dt<0 or abs(ds-dt)>.15:
                    return self.invalidate('IMU 时间重复、跳变或采样中断；请等待数据稳定')
            if self.state['state']=='READY':
                ref=self.state['gyro_bias']
                if max(abs(x-y) for x,y in zip(v[:3],ref))>.03:
                    return self.invalidate('角速度偏离本次静止参考；零点失效')
                if math.sqrt(sum((x-y)**2 for x,y in zip(v[3:],self.state['accel_reference'])))>.35:
                    return self.invalidate('加速度明显变化；请停稳后重新复零')
        elif topic=='body_motion' and self.state['state']=='READY':
            if self.state.get('body_state') is not None and row.get('motion_state')!=self.state['body_state']:
                return self.invalidate('本体控制状态变化／反馈无效；零点失效')
            if self.state.get('motion_bias') is None:return
            if row.get('time_warning'):return self.invalidate('本体速度源时间异常；零点失效')
            if not vector(row.get('values'),3):return self.invalidate('本体反馈无效；零点失效')
            if any(abs(x-y)>limit for x,y,limit in zip(row['values'],self.state['motion_bias'],[.03,.03,.03])):
                return self.invalidate('本体速度离开本次静止参考；请停稳后重新复零')
            if max(abs(x) for x in row['values'])>.0005:
                if self.probe and row['values']==self.probe[1]:
                    if row['received']-self.probe[0]>2.:
                        return self.invalidate('非零本体反馈持续不变，疑似冻结；不再显示复零值')
                else:self.probe=(row['received'],list(row['values']))
        self.previous[topic]=dict(row)
        if topic in self.samples and self.state['state'] in ('SAMPLING','VALIDATING'):
            self.samples[topic].append(dict(received=row['received'],stamp=row.get('stamp'),values=list(row['values']),
                                           motion_state=row.get('motion_state')))

    def tick(self, latest, now, dropped):
        if not self.wants():return
        reason=self.guards(latest,now)
        if reason:return self.invalidate(reason)
        if dropped != self.state['drop_baseline']:return self.invalidate('诊断队列丢弃了样本，不能建立或沿用零点')
        if latest['motion']['navigation'].get('invocation')!=self.state['planner_invocation']:
            return self.invalidate('规划器服务会话变化；零点失效')
        elapsed=now-self.state['started_mono'];self.state['elapsed_s']=min(elapsed,self.FIT+self.VERIFY)
        if self.state['state']=='READY':
            if now>=self.state['expires_mono']:return self.invalidate('本次零点已满 120 秒；每次重新停稳后再复零')
            if self.state.get('motion_bias') is not None:
                b=latest.get('body_motion',{})
                if not -.05 <= now-b.get('received',-999) <= .7:return self.invalidate('本体速度数据过期；零点失效')
            return
        self.state['imu_samples']=len(self.samples['imu'])
        if elapsed>=self.FIT:self.state['state']='VALIDATING';self.state['reason']='采样完成，正在用后 3 秒独立验证；继续保持不动'
        if elapsed>=self.FIT+self.VERIFY:return self.complete(now)
        if int(elapsed)>self.last_write:self.last_write=int(elapsed);self.save()

    def complete(self, now):
        start=self.state['started_mono']
        rows=[r for r in self.samples['imu'] if r['received']<=start+self.FIT+self.VERIFY]
        fit=[r for r in rows if r['received']<start+self.FIT];test=[r for r in rows if r['received']>=start+self.FIT]
        if len(fit)<500 or len(test)<150 or not rows or rows[0]['received']-start>.25 or now-rows[-1]['received']>.25:
            return self.invalidate('有效 IMU 样本或时间覆盖不足，请等待数据稳定后再试',True)
        mean=lambda rr,indices:[statistics.mean(r['values'][i] for r in rr) for i in indices]
        bias=mean(fit,range(3));hold=mean(test,range(3));std=[statistics.stdev(r['values'][i] for r in rows) for i in range(3)]
        if max(abs(x) for x in bias)>.01 or max(std)>.015 or max(abs(x-y) for x,y in zip(hold,bias))>.002:
            return self.invalidate('均值或波动不稳定，不能把这段运动／漂移当作静止零点',True)
        acc=mean(fit,range(3,6));acc_test=mean(test,range(3,6))
        if math.sqrt(sum((x-y)**2 for x,y in zip(acc,acc_test)))>.15:
            return self.invalidate('验证期加速度明显变化，请重新停稳',True)
        body=[r for r in self.samples['body_motion'] if start<=r['received']<=start+self.FIT+self.VERIFY]
        bodyfit=[r for r in body if r['received']<start+self.FIT];bodytest=[r for r in body if r['received']>=start+self.FIT]
        motion_bias=None;why='没有足够新鲜的本体 /MOTION_INFO；运动反馈不复零'
        if len(bodyfit)>=100 and len(bodytest)>=30:
            gaps=[b['received']-a['received'] for a,b in zip(body,body[1:])]
            valid_stamps=all(type(r.get('stamp')) in (int,float) and math.isfinite(r['stamp']) for r in body)
            stamps=[b['stamp']-a['stamp'] for a,b in zip(body,body[1:])] if valid_stamps else [0.]
            if any(r['motion_state']!=17 for r in body):why='本体不在 RL 站立控制态（state=17）；Idle/趴下可能保留旧反馈，运动反馈不复零'
            elif any(not vector(r['values'],3) for r in body):why='本体速度无效，运动反馈不复零'
            elif max(gaps,default=1)>.25 or min(stamps,default=0)<=0 or max(stamps,default=1)>.25 or now-body[-1]['received']>.25:
                why='本体速度源时间或覆盖异常，运动反馈不复零'
            elif len({tuple(r['values']) for r in body})<5:why='本体反馈缺少数值更新，疑似冻结；不对冻结值复零'
            elif any(abs(x)>limit for r in body for x,limit in zip(r['values'],[.05,.05,.04])):
                why='本体速度超出诊断参考范围，运动反馈不复零'
            else:
                candidate=mean(bodyfit,range(3));check=mean(bodytest,range(3))
                if any(abs(x-y)>limit for x,y,limit in zip(candidate,check,[.005,.005,.002])):
                    why='本体速度在独立验证期不稳定，运动反馈不复零'
                else:motion_bias=candidate;why='本次相对静止均值；不是已校准的真实速度'
        self.state.update(state='READY',ready_mono=now,expires_mono=now+self.TTL,gyro_bias=bias,
                          validation_residual=[x-y for x,y in zip(hold,bias)],gyro_std=std,accel_reference=acc,
                          motion_bias=motion_bias,motion_reason=why,body_state=body[-1]['motion_state'] if body else None,
                          reason='静止参考已建立；瞬时波动仍会存在。仅诊断显示，不代表硬件校准或安全放行')
        self.atomic(self.root/(self.state['id']+'.evidence.json'),dict(state=self.state,samples={k:list(v) for k,v in self.samples.items()}))
        self.save()

    def snapshot(self, latest, now):
        result=dict(self.state)
        result['current_epoch']=self.epoch
        result['guard_reason']=self.guards(latest,now)
        result['gyro_relative']=None;result['motion_relative']=None
        if self.state['state']=='READY' and not result['guard_reason'] and now<self.state['expires_mono']:
            result['remaining_s']=max(0,self.state['expires_mono']-now)
            result['gyro_relative']=[x-y for x,y in zip(latest['imu']['values'][:3],self.state['gyro_bias'])]
            b=latest.get('body_motion',{})
            if self.state['motion_bias'] is not None and -.05<=now-b.get('received',-999)<=.7 and vector(b.get('values'),3):
                result['motion_relative']=[x-y for x,y in zip(b['values'],self.state['motion_bias'])]
        return result
