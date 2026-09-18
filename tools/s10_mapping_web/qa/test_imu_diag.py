"""Offline IMU integration checks; no network to a robot is permitted."""
import base64
import hashlib
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
from urllib.request import Request,build_opener,ProxyHandler,HTTPCookieProcessor
from urllib.error import HTTPError
import uuid
import zipfile
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from imu_diag import Diagnostic
from field_core import FieldError
import server as web
from field_robot import RobotAdapter

class Isolation(unittest.TestCase):
    def test_diagnostic_constructor_failure_keeps_original_adapter(self):
        with tempfile.TemporaryDirectory() as root, patch('imu_diag.Diagnostic', side_effect=OSError('disk denied')), patch('field_robot.threading.Thread.start'):
            adapter=RobotAdapter(root,Path(root)/'missing.json')
            self.assertIsNone(adapter.error)
            self.assertIsNone(adapter.imu_diag)
            self.assertIn('disk denied',adapter.imu_diag_error)
            self.assertFalse(adapter.diagnostic_wants())

    def test_diagnostic_callback_failure_isolated(self):
        adapter=RobotAdapter.__new__(RobotAdapter)
        adapter.error=None
        adapter.imu_diag=SimpleNamespace(error=None,dropped=0)
        adapter.offer_diagnostic('imu',b'',SimpleNamespace(),1,time.monotonic())
        self.assertIsNone(adapter.error)
        self.assertEqual(adapter.imu_diag.dropped,1)
        self.assertIn('诊断采集失败',adapter.imu_diag.error)

    def test_diagnostic_wants_failure_isolated(self):
        adapter=RobotAdapter.__new__(RobotAdapter)
        adapter.error=None
        adapter.imu_diag=SimpleNamespace(wants=lambda:1/0)
        self.assertFalse(adapter.diagnostic_wants())
        self.assertIsNone(adapter.error)

class Core(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.d=Diagnostic(Path(self.tmp.name),reserve=0,start_thread=False)
        self.d.rpc(dict(action='live'));self.feed()
    def tearDown(self):
        if self.d.active:self.d.finish('test_end')
        self.tmp.cleanup()
    def feed(self,values=None,t=None):
        t=time.monotonic() if t is None else t
        self.d.ingest(dict(topic='imu',received=t,received_wall=time.time(),stamp=t,values=values or [.001,.002,.003,0,0,9.81],cdr_b64=base64.b64encode(b'original').decode()))
        self.d.tick()
    def start(self,key=None):return self.d.rpc(dict(action='start',key=key or uuid.uuid4().hex,seconds=60,stationary=True,remote_ready=True))
    def test_idempotency_conflict_validation(self):
        s=self.start();self.assertEqual(self.start(s['id'])['id'],s['id'])
        with self.assertRaises(FieldError):self.start()
        for request in [dict(action='calibrate'),dict(action='live',shell='true'),dict(action='report',id='../passwd'),dict(action='live',cursor=-1)]:
            with self.assertRaises(FieldError):self.d.rpc(request)
        with self.assertRaises(FieldError):self.d.rpc(dict(action='start',key=uuid.uuid4().hex,seconds=60,stationary=True,remote_ready=False))
    def test_full_rate_record_and_nan_report_download(self):
        s=self.start();base=time.monotonic()
        for i in range(200):self.feed([.001,.002,float('nan') if i==4 else .003,0,0,9.81],base+i*.005)
        self.d.rpc(dict(action='event',id=s['id'],label='停稳'));self.d.rpc(dict(action='stop',id=s['id']))
        r=self.d.rpc(dict(action='report',id=s['id']));self.assertEqual(r['topic_counts']['imu'],200)
        self.assertEqual(r['invalid_components'],1);self.assertEqual(r['calibration'],'UNVERIFIED');self.assertEqual(r['stationarity'],'UNDETERMINED')
        info=self.d.rpc(dict(action='artifact_info',artifact_id=s['id']));self.assertGreater(info['size'],0)
        with zipfile.ZipFile(Path(self.tmp.name)/s['id']/'diagnostic.zip') as z:
            rows=z.read('samples.jsonl').decode().splitlines();self.assertEqual(len(rows),200)
            self.assertEqual(base64.b64decode(json.loads(rows[0])['cdr_b64']),b'original')
        with self.assertRaises(FieldError):self.d.rpc(dict(action='artifact_read',artifact_id=s['id'],offset=-1,length=1))
    def test_expiration_without_browser_and_restart(self):
        s=self.start();self.d.watch_until=0;self.assertTrue(self.d.wants());self.feed()
        self.d.active['started_mono']-=61;self.d.tick();self.assertIsNone(self.d.active)
        self.assertEqual(self.d.rpc(dict(action='report',id=s['id']))['session']['state'],'COMPLETED')
        self.d.rpc(dict(action='live'));self.feed();s=self.start();self.d.file.flush();self.d.file.close();self.d.active=None
        d2=Diagnostic(Path(self.tmp.name),reserve=0,start_thread=False)
        self.assertEqual(d2.rpc(dict(action='report',id=s['id']))['session']['state'],'INTERRUPTED')
    def test_stale_source_does_not_allow_start(self):
        self.d.latest['imu']['received']=time.monotonic()-5
        with self.assertRaises(FieldError):self.start()
    def test_envelope_preserves_spike_and_time_reverse(self):
        base=time.monotonic()+1
        for i in range(10):self.feed([1 if i==4 else 0]*6,base+i*.005)
        self.feed([0]*6,base+.2)
        rows=self.d.rpc(dict(action='live'))['rows'];self.assertTrue(any(max(b['max'])==1 for b in rows))
        self.feed([0]*6,base-.1);self.assertTrue(self.d.latest['imu']['time_warning'])
    def test_queue_overflow_is_visible(self):
        for _ in range(4200):self.d.ingest(dict(topic='imu',received=time.monotonic(),stamp=1,values=[0]*6))
        self.assertGreater(self.d.rpc(dict(action='live'))['queue_dropped'],0)

class HTTP(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.d=Diagnostic(Path(self.tmp.name),reserve=0,start_thread=False)
        self.d.rpc(dict(action='live'));self.d.ingest(dict(topic='imu',received=time.monotonic(),stamp=1,values=[0]*6));self.d.tick()
        self.patches=[patch.object(web,'config',dict(login_hash=hashlib.sha256(b'demo:test').hexdigest())),
            patch.object(web,'field_transport',lambda r:self.d.rpc(r['request'])),patch.object(web,'imu_thread',object()),
            patch.object(web,'imu_state',dict(self.d.rpc(dict(action='live')),online=True,received=time.monotonic())),
            patch.object(web,'connect',side_effect=AssertionError('No real SSH in tests'))]
        for p in self.patches:p.start()
        self.http=ThreadingHTTPServer(('127.0.0.1',0),web.Handler);self.t=threading.Thread(target=self.http.serve_forever,daemon=True);self.t.start()
        self.client=build_opener(ProxyHandler({}),HTTPCookieProcessor());self.base='http://127.0.0.1:'+str(self.http.server_port)
    def tearDown(self):
        self.http.shutdown();self.http.server_close();self.t.join()
        if self.d.active:self.d.finish('test_end')
        for p in reversed(self.patches):p.stop()
        self.tmp.cleanup()
    def req(self,path,body=None,csrf=False):
        headers={'Content-Type':'application/json'}
        if csrf:headers['X-CSRF-Token']=web.csrf
        try:r=self.client.open(Request(self.base+path,data=json.dumps(body).encode() if body is not None else None,headers=headers),timeout=5)
        except HTTPError as e:r=e
        with r:return r.code,r.read()
    def login(self):self.assertEqual(self.req('/phone/login',dict(username='demo',password='test'))[0],200)
    def test_entry_auth_csrf_bad_actions(self):
        self.assertIn(b'/imu-check',self.req('/')[1]);self.assertEqual(self.req('/phone/imu/live')[0],401)
        self.login();self.assertEqual(self.req('/imu-check')[0],200);self.assertEqual(self.req('/imu_diag.js')[0],200)
        self.assertEqual(self.req('/phone/imu/submit',dict(action='start'))[0],403)
        self.assertEqual(self.req('/phone/imu/submit',dict(action='calibrate'),True)[0],400)
        for path in ['/phone/imu/live?cursor=-1','/phone/imu/live?cmd=move','/phone/imu/report?id=../passwd','/phone/imu/list?x=1']:
            self.assertEqual(self.req(path)[0],400)
    def test_record_stop_report_export(self):
        self.login();key=uuid.uuid4().hex
        body=dict(action='start',key=key,seconds=60,stationary=True,remote_ready=True)
        self.assertEqual(self.req('/phone/imu/submit',body,True)[0],202)
        self.assertEqual(self.req('/phone/imu/submit',body,True)[0],202)
        self.assertEqual(self.req('/phone/imu/submit',dict(action='stop',id=key),True)[0],202)
        self.assertEqual(self.req('/phone/imu/report?id='+key)[0],200)
        status,data=self.req('/phone/imu/download?id='+key);self.assertEqual(status,200);self.assertTrue(data.startswith(b'PK'))
    def test_zero_reference_http_contract(self):
        self.login();key=uuid.uuid4().hex;now=time.monotonic()
        self.d.latest={'imu':dict(topic='imu',received=now,received_wall=time.time(),stamp=time.time(),values=[.001,0,0,0,0,9.81],quaternion=[0,0,0,1]),
                       'motion':dict(received=now,navigation=dict(fresh=True,goal_none=True,planner_code=999,command=[0,0,0],invocation='test'))}
        req=dict(action='zero_start',key=key,epoch=self.d.epoch,stationary=True,remote_ready=True)
        self.assertEqual(self.req('/phone/imu/submit',req)[0],403)
        self.assertEqual(self.req('/phone/imu/submit',dict(req,stationary=False),True)[0],400)
        self.assertEqual(self.req('/phone/imu/submit',dict(req,apply_to_robot=True),True)[0],400)
        self.assertEqual(self.req('/phone/imu/submit',req,True)[0],202)
        self.assertEqual(self.req('/phone/imu/submit',req,True)[0],202)
        self.assertEqual(self.req('/phone/imu/submit',dict(action='zero_clear',id=key),True)[0],202)
        self.assertEqual(self.d.zero.state['state'],'CLEARED')
        self.assertIsNone(self.d.active)

if __name__=='__main__':unittest.main(verbosity=2)
