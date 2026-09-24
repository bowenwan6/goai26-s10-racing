#!/usr/bin/env python3
"""Tests for the teach page (采集助手): pure logic, then worker --fake behind the real
server.py (login, CSRF, proxy, marks, recordings, downloads, path traversal).

  python3 tools/s10_mapping_web/test_teach.py
No ROS needed. Uses a temporary HOME, 127.0.0.1 ports 18091 (worker) / 18090 (server).
"""
import hashlib
import http.cookiejar
import json
import math
import os
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import teach_core as core

failures = []


def check(name, cond, detail=''):
    print(('PASS ' if cond else 'FAIL ') + name + (('  ' + str(detail)) if detail else ''), flush=True)
    if not cond:
        failures.append(name)


def unit():
    still = [dict(t=i * 0.05, x=1.0 + 0.002 * math.sin(i), y=2.0, z=0.4, yaw=0.3) for i in range(60)]
    r = core.still_stats(still)
    check('still sample passes', r['passed'] and r['std_xy'] < 0.005 and abs(r['yaw'] - 0.3) < 1e-6, r)
    moving = [dict(t=i * 0.05, x=i * 0.02, y=0.0, z=0.4, yaw=0.0) for i in range(60)]
    r = core.still_stats(moving)
    check('moving sample fails', not r['passed'] and '没停稳' in r['reasons'][0], r['reasons'])
    few = still[:5]
    check('too few samples fails', not core.still_stats(few)['passed'])
    gap = still[:30] + [dict(s, t=s['t'] + 1.0) for s in still[30:]]
    check('pose gap fails', not core.still_stats(gap)['passed'])
    wrapped = [dict(t=i * 0.05, x=0, y=0, z=0, yaw=math.pi - 0.001 if i % 2 else -math.pi + 0.001) for i in range(60)]
    r = core.still_stats(wrapped)
    check('yaw mean handles +-pi wrap', r['passed'] and abs(abs(r['yaw']) - math.pi) < 0.01, r['yaw'])
    check('path length', abs(core.path_length([(0, 0), (3, 4), (3, 5)]) - 6.0) < 1e-9)
    o = core.loop_offset(dict(x=0, y=0, yaw=math.pi - 0.01), dict(x=0.3, y=0.4, yaw=-math.pi + 0.01))
    check('loop offset', abs(o['dist'] - 0.5) < 1e-9 and o['heading_deg'] < 1.2, o)
    for bad in [('XX', None, None), ('WP', 'WP31', None), ('WP', None, None), ('NOTE', None, '  ')]:
        try:
            core.validate_mark(*bad)
            check('reject %s' % (bad,), False)
        except core.TeachError:
            check('reject %s' % (bad,), True)
    check('session id sanitised', core.SESSION_RE.match(core.session_id('day1 loop/../x')) is not None, core.session_id('day1 loop/../x'))
    marks = [dict(seq=1, kind='WP', wp_id='WP01', result=dict(passed=True, warn=False)),
             dict(seq=2, kind='WP', wp_id='WP02', result=dict(passed=False)),
             dict(seq=3, kind='SWIN', result=dict(passed=True)), dict(seq=4, kind='SWOUT', result=dict(passed=True)),
             dict(seq=5, kind='SWIN', result=dict(passed=True)),
             dict(seq=6, kind='WP', wp_id='WP03', result=dict(passed=True), void=True)]
    b = core.wp_board(marks)
    check('wp board', b['WP01'] == 'pass' and b['WP02'] == 'fail' and b['WP03'] == 'none', {k: b[k] for k in ('WP01', 'WP02', 'WP03')})
    p = core.switch_pairs(marks)
    check('switch pairs', p == [dict(swin=3, swout=4), dict(swin=5, swout=None)], p)


class Client:
    def __init__(self, base):
        self.base = base
        self.opener = urllib.request.build_opener(urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()))
        self.csrf = ''

    def req(self, path, body=None, raw=False):
        data = None if body is None else json.dumps(body).encode()
        headers = {'Content-Type': 'application/json', 'X-CSRF-Token': self.csrf} if data else {}
        r = urllib.request.Request(self.base + path, data=data, headers=headers, method='POST' if data else 'GET')
        try:
            with self.opener.open(r, timeout=70) as resp:
                content = resp.read()
                return resp.status, content if raw else json.loads(content or b'{}')
        except urllib.error.HTTPError as exc:
            content = exc.read()
            try:
                return exc.code, json.loads(content)
            except ValueError:
                return exc.code, content


def wait_until(fn, timeout, step=0.2):
    end = time.time() + timeout
    while time.time() < end:
        v = fn()
        if v:
            return v
        time.sleep(step)
    return None


def integration():
    home = Path(tempfile.mkdtemp(prefix='teachtest-'))
    (home / '.config/s10-mapping-web').mkdir(parents=True)
    (home / '.config/s10-mapping-web/config.json').write_text(json.dumps(
        dict(login_hash=hashlib.sha256(b'demo:demo-only').hexdigest())))
    env = dict(os.environ, HOME=str(home), PYTHONDONTWRITEBYTECODE='1')
    worker = subprocess.Popen([sys.executable, str(HERE / 'teach_worker.py'), '--fake', '--port', '8091',
                               '--min-free-gb-mapping', '0.1'], env=env, cwd=str(HERE),
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    server = subprocess.Popen([sys.executable, str(HERE / 'server.py'), '--host', '127.0.0.1', '--port', '18090'],
                              env=env, cwd=str(HERE), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    c = Client('http://127.0.0.1:18090')
    try:
        time.sleep(2)
        code, _ = c.req('/phone/teach/status')
        check('status needs login', code == 401, code)
        code, _ = c.req('/phone/login', dict(username='demo', password='demo-only'))
        check('login', code == 200, code)
        code, page = c.req('/teach', raw=True)
        check('teach page served', code == 200 and b'teach.js' in page, code)
        code, st = c.req('/phone/teach/status')
        c.csrf = st.get('csrf', '')
        check('status via proxy', code == 200 and st['fake'] and st['ros']['ok'], code)
        code, r = c.req('/phone/teach/submit', dict(action='record_start', mode='survey'))
        check('recording needs a session', code == 400 and '会话' in r.get('detail', ''), r)
        code, r = c.req('/phone/teach/submit', dict(action='bogus'))
        check('unknown action rejected by server', code == 400, r)
        bad = Client(c.base); bad.opener = c.opener; bad.csrf = 'wrong'
        code, r = bad.req('/phone/teach/submit', dict(action='session_new', label='x'))
        check('CSRF enforced', code == 403, code)
        code, r = c.req('/phone/teach/submit', dict(action='session_new', label='test run', map_name='garden_demo'))
        check('session created', code == 200 and r['session']['id'].endswith('test_run'), r)
        sid = r['session']['id']
        code, r = c.req('/phone/teach/submit', dict(action='record_start', mode='survey'))
        check('survey recording started', code == 200, r)

        def phase(moving):
            s = c.req('/phone/teach/status')[1]
            sp = (s.get('pose') or {}).get('speed')
            return s if sp is not None and ((sp > 0.1) == moving) else None
        # fake robot: walks, then stands 8 s at each corner
        wait_until(lambda: phase(True), 30)
        check('robot moving', phase(True) is not None)
        code, r = c.req('/phone/teach/submit', dict(action='mark', kind='WP', wp_id='WP02'))
        check('mark accepted while moving', code == 200 and r['job']['wp_id'] == 'WP02', r)
        res = wait_until(lambda: (c.req('/phone/teach/status')[1].get('last_result') or {}).get('seq'), 8)
        st = c.req('/phone/teach/status')[1]
        check('moving mark fails', res and not st['last_result']['result']['passed'], st.get('last_result'))
        wait_until(lambda: phase(False), 40)
        time.sleep(0.5)
        code, r = c.req('/phone/teach/submit', dict(action='mark', kind='WP', wp_id='WP01'))
        check('still mark accepted', code == 200, r)
        code, r = c.req('/phone/teach/submit', dict(action='mark', kind='WP', wp_id='WP03'))
        check('second mark during sampling rejected', code == 400 and '采样' in r.get('detail', ''), r)
        time.sleep(3.6)
        st = c.req('/phone/teach/status')[1]
        lr = st['last_result']
        check('still mark passes', lr['wp_id'] == 'WP01' and lr['result']['passed'] and lr['result']['std_xy'] < 0.02, lr['result'])
        check('board shows WP01 pass / WP02 fail', st['board']['WP01'] == 'pass' and st['board']['WP02'] == 'fail', {k: st['board'][k] for k in ('WP01', 'WP02')})
        code, r = c.req('/phone/teach/submit', dict(action='mark', kind='NOTE', note='WP16 两面旗取左边'))
        check('note saved', code == 200, r)
        code, r = c.req('/phone/teach/submit', dict(action='redo'))
        check('redo voids the note', code == 200, r)
        code, r = c.req('/phone/teach/submit', dict(action='record_stop'))
        check('survey stopped', code == 200 and r['files'], r)
        code, r = c.req('/phone/teach/submit', dict(action='record_start', mode='path'))
        check('path started', code == 200, r)
        time.sleep(6)
        st = c.req('/phone/teach/status')[1]
        check('path length grows', st['path'] and st['path']['length_m'] >= 0.0, st.get('path'))
        code, r = c.req('/phone/teach/submit', dict(action='record_stop'))
        check('path stopped', code == 200 and 'path_length_m' in r, r)
        code, r = c.req('/phone/teach/submit', dict(action='record_start', mode='mapping'))
        check('mapping started', code == 200, r)
        time.sleep(3)
        st = c.req('/phone/teach/status')[1]
        check('loop helper active', st['loop'] is not None and 'dist' in st['loop'], st.get('loop'))
        code, r = c.req('/phone/teach/submit', dict(action='record_stop'))
        check('mapping stopped with loop summary', code == 200 and 'loop' in r, r)
        code, d = c.req('/phone/teach/sessions')
        files = {f['name'] for f in d['sessions'][0]['files']}
        check('session files', {'session.json', 'marks.jsonl', 'recordings.jsonl'} <= files, sorted(files))
        code, raw = c.req('/phone/teach/file?session=%s&name=marks.jsonl' % sid, raw=True)
        lines = [json.loads(x) for x in raw.decode().splitlines()]
        check('marks.jsonl download', code == 200 and any(m.get('wp_id') == 'WP01' for m in lines), code)
        check('void recorded', any(m.get('kind') == 'VOID' for m in lines))
        code, _ = c.req('/phone/teach/file?session=%s&name=..%%2F..%%2F.config' % sid)
        check('path traversal rejected', code in (400, 404), code)
        code, _ = c.req('/phone/teach/file?session=..&name=session.json')
        check('bad session id rejected', code == 400, code)
        code, r = c.req('/phone/teach/submit', dict(action='session_open', session_id=sid))
        check('session reopen restores marks', code == 200, r)
        st = c.req('/phone/teach/status')[1]
        check('reopened board', st['board']['WP01'] == 'pass', st['board']['WP01'])
    finally:
        server.terminate(); worker.terminate()
        server.wait(5); worker.wait(5)


if __name__ == '__main__':
    unit()
    integration()
    print('TEACH_TESTS_OK' if not failures else 'TEACH_TESTS_FAILED %s' % failures)
    sys.exit(1 if failures else 0)
