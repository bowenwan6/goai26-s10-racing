"""Local only: real HTTP/auth and task ledger, synthetic telemetry, no robot I/O."""
import hashlib
import json
import sys
import tempfile
import threading
import time
import unittest
from http.server import ThreadingHTTPServer
from pathlib import Path
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import HTTPCookieProcessor, ProxyHandler, Request, build_opener

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[1]))
import native_nav as nav
import server as web

from native_transfer.app_control import AppControl

CODE = HERE.parents[1]


def snapshot():
    return dict(wall_time=time.time(), gait=4097, motion_state=17, hes=0,
                nav_cmd_publishers=0, nav_cmd_subscribers=1, gait_subscribers=1,
                stream_ages=dict(pose=.01, motion=.01, hes=.01), input_faults={},
                map_context=dict(global_mode=True), measured_velocity=[0, 0, 0])


class NativeTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.engine = nav.Engine(self.root/'runs', CODE, self.root/'shared.lock', field=lambda r: dict(active_job=None))

    def tearDown(self):
        self.tmp.cleanup()

    def seed(self, **updates):
        rid = 'c'*32
        (self.engine.root/rid).mkdir(exist_ok=True)
        snap = snapshot(); snap.update(updates)
        (self.engine.root/rid/'telemetry.jsonl').write_text(json.dumps(snap)+'\n')
        row = dict(id=rid, key='d'*32, kind='observe', state='completed', snapshot=snap, created=time.time())
        with self.engine.db() as db:
            db.execute('INSERT OR REPLACE INTO runs VALUES (?,?,?,?)', (rid, 'd'*32, '{}', json.dumps(row)))
        return row

    def request(self, kind='observe'):
        return dict(action='submit', kind=kind, key='a'*32, owner='b'*32,
                    onsite={} if kind=='observe' else dict(supervisor=True, clear=True, position=True))

    def test_pending_engineering_cannot_be_overridden_by_onsite(self):
        self.seed()
        with self.assertRaisesRegex(nav.FieldError, '实机验收'):
            self.engine.rpc(self.request('start_b'))
        self.assertIsNone(self.engine.active)

    def test_foreign_publisher_refuses_even_stationary_gait(self):
        self.seed(nav_cmd_publishers=2)
        with self.assertRaisesRegex(nav.FieldError, '其他速度指令'):
            self.engine.rpc(self.request('flat'))

    def test_stale_sensor_and_observation_refuse(self):
        self.seed(stream_ages=dict(pose=1, motion=.01, hes=.01))
        with self.assertRaisesRegex(nav.FieldError, '反馈缺失或过期'):
            self.engine.rpc(self.request('stairs'))
        self.seed(wall_time=time.time()-61)
        with self.assertRaisesRegex(nav.FieldError, '先运行一次'):
            self.engine.rpc(self.request('flat'))

    def test_unknown_map_context_is_blocked_without_crashing(self):
        self.seed(map_context=None)
        self.assertIn('全局定位', ' '.join(self.engine.status()['blockers']['start']))

    def test_request_cannot_inject_paths_or_commands(self):
        r = self.request(); r['config'] = '/etc/passwd'
        with self.assertRaises(nav.FieldError): self.engine.rpc(r)
        r = self.request(); r['kind'] = 'start; shutdown'
        with self.assertRaises(nav.FieldError): self.engine.rpc(r)
        with self.assertRaises(nav.FieldError): self.engine.rpc(dict(action='report', id='../secret'))

    def test_field_job_conflict(self):
        self.engine.field=lambda r:dict(active_job='other')
        with self.assertRaisesRegex(nav.FieldError, '现场助手正在'):
            self.engine.rpc(self.request())

    def test_idempotency_lock_cancel_and_ownership(self):
        gate = threading.Event()
        def hold(rid, reservation):
            gate.wait(3)
            reservation.__exit__(None,None,None)
        with patch.object(self.engine, 'run', side_effect=hold):
            first=self.engine.rpc(self.request())
            try:
                same=self.engine.rpc(self.request())
                self.assertEqual(first['id'],same['id'])
                with self.assertRaises(nav.FieldError):
                    with nav.exclusive(self.engine.lock_path): pass
                other=self.request();other['key']='e'*32
                with self.assertRaisesRegex(nav.FieldError,'已有测试'):
                    self.engine.rpc(other)
                other=self.request();other['kind']='flat'
                with self.assertRaisesRegex(nav.FieldError,'同一请求编号'):
                    self.engine.rpc(other)
                with self.assertRaisesRegex(nav.FieldError,'只有发起'):
                    self.engine.rpc(dict(action='heartbeat',id=first['id'],owner='f'*32))
                self.engine.rpc(dict(action='heartbeat',id=first['id'],owner='b'*32))
                self.assertGreater(self.engine.lease,time.monotonic())
                self.engine.rpc(dict(action='cancel',id=first['id']))
                self.assertTrue(self.engine.stop_requested)
            finally:
                gate.set();time.sleep(.05)

    def test_service_restart_marks_interrupted_without_resume(self):
        row=self.seed();self.engine.update(row['id'],state='running')
        restarted=nav.Engine(self.engine.root,CODE,self.root/'shared.lock',field=lambda r:{})
        self.assertEqual(restarted.row(row['id'])['state'],'interrupted')
        self.assertIsNone(restarted.active)

    def test_partial_json_does_not_replace_last_snapshot(self):
        p=self.root/'data.jsonl';p.write_text('{"wall_time":1}\n{"wall_time":')
        self.assertEqual(nav.last_record(p),{'wall_time':1})

    def test_app_lease_expires_and_cannot_rearm(self):
        p=self.root/'lease.json'
        nav.atomic(p,dict(command='wait',expires_monotonic=104))
        c=AppControl(p);self.assertEqual(c.poll(100),'wait')
        nav.atomic(p,dict(command='arm',expires_monotonic=104))
        self.assertEqual(c.poll(101),'arm');self.assertEqual(c.poll(102),'wait')
        self.assertEqual(c.poll(104),'cancel')
        nav.atomic(p,dict(command='arm',expires_monotonic=108))
        self.assertEqual(c.poll(105),'cancel')

    def test_app_missing_corrupt_or_unbounded_lease_stops(self):
        p=self.root/'lease.json'
        for value in [None, 'bad json', '{"expires_monotonic":9999,"command":"arm"}', '{"expires_monotonic":104,"command":"shell"}']:
            if value is not None:p.write_text(value)
            self.assertEqual(AppControl(p).poll(100),'cancel')

    def test_http_auth_csrf_no_get_motion_and_request_validation(self):
        web.config=dict(login_hash=hashlib.sha256(b'demo:native-local').hexdigest())
        web.native_transport=self.engine.rpc
        http=ThreadingHTTPServer(('127.0.0.1',0),web.Handler)
        threading.Thread(target=http.serve_forever,daemon=True).start()
        client=build_opener(ProxyHandler({}),HTTPCookieProcessor())
        def request(path,body=None,csrf=False):
            headers={'Content-Type':'application/json'}
            if csrf:headers['X-CSRF-Token']=web.csrf
            req=Request(f'http://127.0.0.1:{http.server_port}'+path,data=None if body is None else json.dumps(body).encode(),headers=headers)
            try:r=client.open(req,timeout=3)
            except HTTPError as e:r=e
            return r.code,r.read()
        try:
            with patch.object(web,'connect',side_effect=AssertionError('No real SSH')):
                self.assertEqual(request('/phone/native/status')[0],401)
                self.assertEqual(request('/native_nav.js')[0],401)
                self.assertIn(b'id="login"',request('/native-nav')[1])
                self.assertEqual(request('/phone/login',dict(username='demo',password='native-local'))[0],200)
                self.assertEqual(request('/phone/native/submit',self.request())[0],403)
                self.assertEqual(request('/phone/native/status')[0],200)
                self.assertIsNone(self.engine.active)
                self.assertEqual(request('/phone/native/submit',dict(action='arm'),True)[0],400)
                self.assertEqual(request('/phone/native/report?id=../../secret')[0],400)
                self.assertEqual(request('/native-nav')[0],200)
        finally:
            http.shutdown();http.server_close();web.native_transport=None


if __name__ == '__main__':unittest.main()
