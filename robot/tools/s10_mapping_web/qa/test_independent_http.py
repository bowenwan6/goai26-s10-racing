"""HTTP auth/CSRF/download tests with local fake Engine; production SSH forbidden."""
import hashlib
import json
import sys
import tempfile
import threading
import unittest
import uuid
from http.server import ThreadingHTTPServer
from pathlib import Path
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import HTTPCookieProcessor, ProxyHandler, Request, build_opener

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import server as web
from field_core import Store
from field_worker import Engine
from test_independent_worker import FakeEvidenceAdapter


class HTTPTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='s10-http-qa-'); root=Path(self.tmp.name)
        self.store=Store(root/'persist'); self.engine=Engine(self.store,FakeEvidenceAdapter(),root/'operation.lock')
        self.patches=[patch.object(web,'config',dict(login_hash=hashlib.sha256(b'demo:independent-http-only').hexdigest())),
                      patch.object(web,'field_transport',self.engine.rpc),
                      patch.object(web,'call',side_effect=AssertionError('No real SSH permitted in QA'))]
        for p in self.patches:p.start()
        self.http=ThreadingHTTPServer(('127.0.0.1',0),web.Handler)
        self.thread=threading.Thread(target=self.http.serve_forever,daemon=True);self.thread.start()
        self.base='http://127.0.0.1:'+str(self.http.server_port)
        self.client=build_opener(ProxyHandler({}),HTTPCookieProcessor())

    def tearDown(self):
        self.http.shutdown();self.http.server_close();self.thread.join(2)
        for p in reversed(self.patches):p.stop()
        self.tmp.cleanup()

    def request(self,path,body=None,headers=None):
        request=Request(self.base+path,data=json.dumps(body).encode() if body is not None else None,
                        headers={'Content-Type':'application/json',**(headers or {})})
        try:response=self.client.open(request,timeout=4)
        except HTTPError as exc:response=exc
        with response:return response.code,response.headers,response.read()

    def login(self):
        code,headers,_=self.request('/phone/login',dict(username='demo',password='independent-http-only'))
        self.assertEqual(code,200);self.assertIn('HttpOnly',headers['Set-Cookie']);self.assertIn('SameSite=Strict',headers['Set-Cookie'])

    def request_body(self):return dict(action='selfcheck',key=uuid.uuid4().hex,params=dict(target_map='test_map'))

    def test_auth_csrf_unknown_action_and_injection_rejected(self):
        for endpoint in ('/phone/field/health','/phone/field/live','/phone/field/list','/phone/field/dashboard','/phone/field/download?artifact_id='+'0'*32):
            self.assertEqual(self.request(endpoint)[0],401)
        self.login();body=self.request_body()
        self.assertEqual(self.request('/phone/field/submit',body)[0],403)
        csrf={'X-CSRF-Token':web.csrf}
        invalid=dict(body,action='run_shell');self.assertEqual(self.request('/phone/field/submit',invalid,csrf)[0],400)
        for endpoint in ('/phone/field/job?job_id=../secret','/phone/field/download?artifact_id=/etc/passwd',
                         '/phone/field/list?session_id=bad%27%20OR%201=1','/phone/field/live?command=ls'):
            self.assertEqual(self.request(endpoint)[0],400)
        self.assertEqual(self.request('/phone/field/job?job_id='+'0'*32)[0],404)
        self.assertEqual(len(self.store.jobs()),0)

    def test_http_idempotency_and_conflict_statuses(self):
        self.login();body=self.request_body();csrf={'X-CSRF-Token':web.csrf}
        code,_,raw=self.request('/phone/field/submit',body,csrf);self.assertEqual(code,202);first=json.loads(raw)
        code,_,raw=self.request('/phone/field/submit',body,csrf);self.assertEqual(code,202);self.assertEqual(json.loads(raw)['id'],first['id'])
        self.assertEqual(self.request('/phone/field/submit',dict(body,params=dict(target_map='other_map')),csrf)[0],409)
        self.assertEqual(self.request('/phone/field/submit',self.request_body(),csrf)[0],409)
        self.assertEqual(len(self.store.jobs()),1)

    def test_dashboard_auth_csrf_and_query_validation(self):
        self.login()
        code,_,raw=self.request('/phone/field/dashboard')
        self.assertEqual(code,200)
        self.assertEqual(json.loads(raw)['health']['csrf'],web.csrf)
        self.assertEqual(self.request('/phone/field/dashboard?session_id=bad')[0],400)
        self.assertEqual(self.request('/phone/field/dashboard?command=move')[0],400)
        self.assertEqual(len(self.store.jobs()),0)

    def test_authenticated_streaming_download_and_range(self):
        job=self.store.submit(self.request_body());root=self.store.root/'jobs'/job['id'];root.mkdir(parents=True)
        payload=b'0123456789'*70000;file=root/'report.bin';file.write_bytes(payload)
        aid=self.store.artifact(job['id'],file,name='报告";safe.bin');url='/phone/field/download?artifact_id='+aid
        self.assertEqual(self.request(url)[0],401);self.login()
        code,headers,raw=self.request(url);self.assertEqual(code,200);self.assertEqual(raw,payload)
        self.assertEqual(headers['ETag'],'"'+hashlib.sha256(payload).hexdigest()+'"')
        code,headers,raw=self.request(url,headers={'Range':'bytes=262142-262148'});self.assertEqual(code,206)
        self.assertEqual(raw,payload[262142:262149]);self.assertEqual(headers['Content-Range'],f'bytes 262142-262148/{len(payload)}')
        self.assertEqual(self.request(url,headers={'Range':'bytes=9999999-'})[0],416)
        self.assertEqual(self.request(url,headers={'Range':'bytes=1-2,4-5'})[0],416)

    def test_revisit_authenticated_api_export_and_no_side_effects(self):
        def perform(action,params,session_id=None):
            req=dict(action=action,key=uuid.uuid4().hex,params=params)
            if session_id:req['session_id']=session_id
            j=self.engine.rpc(dict(action='submit',request=req));return self.engine.execute(j['id'])
        sid=perform('selfcheck',dict(target_map='test_map'))['session_id']
        check=perform('localization_check',dict(stationary=True),sid)
        perform('confirm_overlay',dict(check_id=check['id'],confirmed=True),sid)
        point=perform('waypoint',dict(name='WP0',floor='1',draft=True,stationary=True),sid)
        body=dict(action='waypoint_revisit',key=uuid.uuid4().hex,session_id=sid,
                  params=dict(waypoint_id=point['id'],stationary=True,remote_ready=True,
                              physical_confirmed=True,reference_note='test-only physical marker'))
        # Existing POST contract combines login and CSRF rejection as 403.
        self.assertEqual(self.request('/phone/field/submit',body)[0],403)
        self.login();self.assertEqual(self.request('/phone/field/submit',body)[0],403)
        csrf={'X-CSRF-Token':web.csrf}
        invalid=dict(body,params=dict(body['params'],physical_confirmed=False))
        self.assertEqual(self.request('/phone/field/submit',invalid,csrf)[0],400)
        code,_,raw=self.request('/phone/field/submit',body,csrf);self.assertEqual(code,202)
        job=self.engine.execute(json.loads(raw)['id']);self.assertEqual(job['state'],'SUCCEEDED')
        art=next(a for a in job['artifacts'] if a['name']=='waypoint-revisit.json')
        code,_,raw=self.request('/phone/field/download?artifact_id='+art['id']);self.assertEqual(code,200)
        self.assertFalse(json.loads(raw)['absolute_accuracy_verified'])
        self.assertEqual(self.engine.adapter.effects,[])


if __name__=='__main__':unittest.main(verbosity=2)
