"""Offline display-reference tests: no ROS or robot connection."""
import copy
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import uuid

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from imu_diag import Diagnostic, atomic
from imu_zero import ZeroReference
from field_core import FieldError
from field_robot import parse_navigation


class ZeroTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        self.now=100.;self.z=ZeroReference(self.root,'epoch','boot',atomic)
        self.latest={};self.feed(self.now)
    def tearDown(self):self.tmp.cleanup()
    def feed(self,t,state=17,frozen=False,command=0):
        self.now=t
        imu=dict(topic='imu',received=t,received_wall=1000+t,stamp=1000+t,frame='imu',
                 values=[.001+math.sin(t*17)*.001,-.002+math.cos(t*13)*.001,.0005+math.sin(t*21)*.001,0,0,9.81],quaternion=[0,0,0,1])
        motion=dict(topic='motion',received=t,stamp=1000+t,values=[.01,-.017,.0005,0,0,0],navigation=dict(fresh=True,goal_none=True,planner_code=999,command=[command,0,0],motion=[.01,-.017,.0005],idle=False,invocation='planner'))
        body=dict(topic='body_motion',received=t,stamp=1000+t,values=[.01,-.017,-.00242589 if frozen else .0005+.001*math.sin(t*23)],motion_state=state)
        for row in [imu,motion,body]:
            self.latest[row['topic']]=row;self.z.observe(row)
        self.z.tick(self.latest,t,0)
        return imu,motion,body
    def start(self):
        self.key=uuid.uuid4().hex;return self.z.start(self.key,self.latest,self.now,0)
    def complete(self,state=17,frozen=False):
        self.start();base=self.now
        for i in range(1,1302):self.feed(base+i*.01,state,frozen)
        return self.z.snapshot(self.latest,self.now)
    def test_reference_and_raw_preserved(self):
        d=self.complete();self.assertEqual(d['state'],'READY');self.assertIsNotNone(d['motion_relative'])
        before=copy.deepcopy(self.latest);self.z.snapshot(self.latest,self.now);self.assertEqual(self.latest,before)
        self.assertEqual(d['scope'],'DISPLAY_ONLY');self.assertFalse(d['hardware_applied']);self.assertFalse(d['safety_gate_changed'])
        self.assertLess(max(abs(x) for x in d['validation_residual']),.002)
        self.assertTrue((self.root/'zero_references'/(self.key+'.evidence.json')).is_file())
        self.assertEqual(self.z.start(self.key,self.latest,self.now,0)['id'],self.key)
    def test_idle_or_frozen_motion_not_zeroed(self):
        for state,frozen,word in [(0,False,'Idle'),(17,True,'冻结')]:
            with self.subTest(state=state):
                self.z=ZeroReference(self.root,str(state),'boot',atomic)
                d=self.complete(state,frozen);self.assertEqual(d['state'],'READY')
                self.assertIsNone(d['motion_relative']);self.assertIsNone(d['motion_bias']);self.assertIn(word,d['motion_reason'])
    def test_double_start_and_restart(self):
        d=self.start();self.assertEqual(self.z.start(d['id'],self.latest,self.now,0),d)
        with self.assertRaises(FieldError):self.z.start(uuid.uuid4().hex,self.latest,self.now,0)
        other=ZeroReference(self.root,'new','boot',atomic);self.assertEqual(other.state['state'],'EXPIRED')
        self.assertIsNone(other.snapshot(self.latest,self.now)['gyro_relative'])
    def test_block_command_and_stale_data(self):
        self.feed(100,command=.01)
        with self.assertRaises(FieldError):self.start()
        self.feed(100);self.latest['imu']['received']=98
        with self.assertRaises(FieldError):self.start()
    def test_time_reverse_and_queue_drop(self):
        self.start();self.feed(100.01);r=copy.deepcopy(self.latest['imu']);r['received']=100.02;r['stamp']=1099
        self.z.observe(r);self.assertEqual(self.z.state['state'],'FAILED')
        self.feed(101);self.start();self.z.tick(self.latest,101,1);self.assertEqual(self.z.state['state'],'FAILED')
    def test_ready_invalidation(self):
        for reason in ['motion','pose','stale','ttl','state','command','planner','body_clock']:
            with self.subTest(reason=reason):
                self.z=ZeroReference(self.root,'ep'+reason,'boot',atomic);self.feed(self.now+1);self.complete()
                if reason=='motion':
                    r=copy.deepcopy(self.latest['body_motion']);r['values']=[.2,0,0];self.z.observe(r)
                elif reason=='pose':
                    r=copy.deepcopy(self.latest['imu']);r['quaternion']=[0,0,.1,math.sqrt(.99)];self.z.observe(r)
                elif reason=='stale':self.z.tick(self.latest,self.now+3,0)
                elif reason=='ttl':
                    self.z.state['expires_mono']=self.now-1;self.z.tick(self.latest,self.now,0)
                elif reason=='state':
                    r=copy.deepcopy(self.latest['body_motion']);r['motion_state']=0;self.z.observe(r)
                elif reason=='command':self.feed(self.now+.01,command=.01)
                elif reason=='planner':
                    self.latest['motion']['navigation']['invocation']='restarted';self.z.tick(self.latest,self.now,0)
                else:
                    r=copy.deepcopy(self.latest['body_motion']);r['time_warning']=True;self.z.observe(r)
                self.assertEqual(self.z.state['state'],'EXPIRED')
                d=self.z.snapshot(self.latest,self.now);self.assertIsNone(d['gyro_relative']);self.assertIsNone(d['motion_relative'])
    def test_frozen_after_ready(self):
        self.complete();t=self.now
        for i in range(1,215):self.feed(t+i*.01,frozen=True)
        self.assertEqual(self.z.state['state'],'EXPIRED');self.assertIn('冻结',self.z.state['reason'])
    def test_no_motion_source_is_not_fabricated(self):
        self.start();t=self.now
        for i in range(1,1302):
            self.feed(t+i*.01);self.z.samples['body_motion'].clear()
        self.assertIsNone(self.z.state['motion_bias'])
    def test_synthetic_large_bias_rejected(self):
        self.start();t=self.now
        for i in range(1,1302):
            now=t+i*.01
            self.latest['imu']=dict(topic='imu',received=now,received_wall=1000+now,stamp=1000+now,frame='imu',values=[.02,0,0,0,0,9.81],quaternion=[0,0,0,1])
            self.latest['motion']['received']=now
            self.z.observe(self.latest['imu']);self.z.tick(self.latest,now,0)
        self.assertEqual(self.z.state['state'],'FAILED')


class RPC(unittest.TestCase):
    def test_contract_csrf_independent_recording_and_clear(self):
        with tempfile.TemporaryDirectory() as tmp,patch('imu_diag.time.monotonic',return_value=100):
            d=Diagnostic(tmp,reserve=0,start_thread=False)
            d.latest={'imu':dict(received=100,received_wall=1100,stamp=1100,values=[0,0,0,0,0,9.81],quaternion=[0,0,0,1]),
                      'motion':dict(received=100,navigation=dict(fresh=True,goal_none=True,planner_code=999,command=[0,0,0],invocation='a'))}
            key=uuid.uuid4().hex;req=dict(action='zero_start',key=key,epoch=d.epoch,stationary=True,remote_ready=True)
            for bad in [dict(req,stationary=False),dict(req,epoch='old'),dict(req,threshold=999),dict(req,key='../config')]:
                with self.assertRaises(FieldError):d.rpc(bad)
            s=d.rpc(req);self.assertEqual(d.rpc(req)['id'],key)
            with self.assertRaises(FieldError):d.rpc(dict(action='zero_clear',id=uuid.uuid4().hex))
            with self.assertRaises(FieldError):d.rpc(dict(action='start',key=uuid.uuid4().hex,seconds=60,stationary=True,remote_ready=True))
            d.rpc(dict(action='zero_clear',id=key));self.assertEqual(d.rpc(dict(action='live'))['zero']['state'],'CLEARED')
            self.assertIsNone(d.active);self.assertEqual(d.rpc(dict(action='list'))['sessions'],[])

if __name__=='__main__':unittest.main(verbosity=2)
