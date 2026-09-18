"""48 phone mapping server on AGX; 103 relays HTTP, 106 runs vendor SLAM."""
import argparse
import base64
from collections import deque, OrderedDict
import hashlib
import hmac
from http import cookies
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import secrets
import threading
import time
from urllib.parse import urlsplit, parse_qs, quote
from field_core import FieldError, ident

HERE = Path(__file__).resolve().parent
CONFIG = Path.home()/'.config/s10-mapping-web/config.json'
state = dict(online=False, active=False, streams={}, trail=[], error='正在连接定位板')
guard, operation = threading.Lock(), threading.Lock()
last_view = 0.0
last_height_view = 0.0
height_state = dict(online=False, streams={})
operation_name = ''
csrf = secrets.token_urlsafe(24)
login_cookie = secrets.token_urlsafe(32)
config = {}
field_transport = None
native_transport = None
imu_state = dict(online=False, error='等待连接IMU诊断采集器', rows=[])
imu_last_view = 0.
imu_thread = None


def imu_call(request):
    return field_call(dict(action='imu_diag', request=request))


def read_imu_stream():
    global imu_state
    history = deque(maxlen=3600)
    epoch = None
    while True:
        if time.monotonic()-imu_last_view > 30:
            time.sleep(.5); continue
        try:
            with connect() as client:
                stdin, stdout, stderr = client.exec_command('s10-mapping', timeout=12)
                stdin.write('{"action":"imu_diag_stream"}\n'); stdin.flush(); stdin.channel.shutdown_write()
                for line in stdout:
                    if time.monotonic()-imu_last_view > 30:break
                    if not line.startswith('S10_RESULT '):continue
                    row=json.loads(line[11:])
                    if epoch != row['epoch']:history.clear();epoch=row['epoch']
                    last_seq=history[-1]['seq'] if history else 0
                    history.extend(b for b in row['rows'] if b['seq']>last_seq)
                    with guard:imu_state=dict(row,rows=list(history),online=True,received=time.monotonic())
                if time.monotonic()-imu_last_view <= 30:raise RuntimeError('IMU诊断连接已退出')
        except Exception as exc:
            with guard:imu_state.update(online=False,error=str(exc))
            time.sleep(2)


def field_call(request):
    return field_transport(request) if field_transport is not None else call('field', request=request)


def native_call(request):
    return native_transport(request) if native_transport is not None else call('native_nav', request=request)


def error_status(exc):
    return {'invalid': 400, 'conflict': 409, 'busy': 409, 'not_found': 404}.get(getattr(exc, 'code', ''), 503)


def connect():
    import paramiko
    client = paramiko.SSHClient()
    client.load_host_keys(str(HERE/'known_hosts'))
    client.connect('10.21.33.106', username='user', key_filename=str(HERE/'backend_key'),
                   look_for_keys=False, allow_agent=False, timeout=8, banner_timeout=8, auth_timeout=8)
    client.get_transport().set_keepalive(15)
    return client


def call(action, **body):
    with connect() as client:
        stdin, stdout, stderr = client.exec_command('s10-mapping', timeout=240)
        stdin.write(json.dumps(dict(action=action, **body))+'\n'); stdin.flush()
        stdin.channel.shutdown_write()
        for line in stdout:
            if line.startswith('S10_RESULT '):
                response = json.loads(line[11:])
                if not response['ok']:
                    if 'code' in response:
                        raise FieldError(response['error'], response['code'])
                    raise RuntimeError(response['error'])
                return response['result']
        raise RuntimeError('定位板未返回操作结果；请先核对状态')


def read_stream():
    global state
    trail, cloud = deque(maxlen=2000), OrderedDict()
    session, frame = None, None
    while True:
        if time.monotonic()-last_view > 30:
            time.sleep(1)
            continue
        try:
            with connect() as client:
                stdin, stdout, stderr = client.exec_command('s10-mapping', timeout=12)
                stdin.write('{"action":"stream"}\n'); stdin.flush()
                stdin.channel.shutdown_write()
                for line in stdout:
                    if time.monotonic()-last_view > 30:
                        break
                    if not line.startswith('S10_RESULT '):
                        continue
                    row = json.loads(line[11:])
                    streams = row['streams']
                    points = streams.get('map', {})
                    identity = row.get('invocation') if row['active'] else None
                    if session != identity or frame != points.get('frame'):
                        cloud.clear(); trail.clear()
                        session, frame = identity, points.get('frame')
                    if row['active'] and points.get('age', 999) < 3:
                        # ponytail: 30k voxels / 20 cm is a bounded phone preview,
                        # not the vendor's complete, loop-optimized saved map.
                        for p in points.get('points', []):
                            key = tuple(round(v/.2) for v in p)
                            cloud[key] = p
                            cloud.move_to_end(key)
                            if len(cloud) > 30000:
                                cloud.popitem(last=False)
                        points['points'] = list(cloud.values())
                    pose = streams.get('pose', {})
                    if row['active'] and pose.get('age', 999) < 3:
                        trail.append(dict(position=pose['position'], frame=pose['frame']))
                    with guard:
                        state = dict(row, online=True, trail=list(trail), received=time.monotonic())
                if time.monotonic()-last_view <= 30:
                    raise RuntimeError('定位板数据流已退出：'+stderr.read(2048).decode(errors='replace'))
        except Exception as exc:
            with guard:
                state.update(online=False, error=str(exc))
            time.sleep(3)


def read_height_stream():
    global height_state
    while True:
        if time.monotonic()-last_height_view > 30:
            time.sleep(1)
            continue
        try:
            with connect() as client:
                stdin, stdout, stderr = client.exec_command('s10-mapping', timeout=12)
                stdin.write('{"action":"heightmap_stream"}\n'); stdin.flush()
                stdin.channel.shutdown_write()
                for line in stdout:
                    if time.monotonic()-last_height_view > 30:
                        break
                    if line.startswith('S10_RESULT '):
                        row = json.loads(line[11:])
                        with guard:
                            height_state = dict(row, online=True, received=time.monotonic())
                if time.monotonic()-last_height_view <= 30:
                    raise RuntimeError('高程图读取已退出：'+stderr.read(2048).decode(errors='replace'))
        except Exception as exc:
            with guard:
                height_state.update(online=False, error=str(exc))
            time.sleep(3)


class Handler(BaseHTTPRequestHandler):
    def reply(self, code, data, kind='application/json; charset=utf-8', cookie=None):
        raw = data if isinstance(data, bytes) else json.dumps(data, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header('Content-Type', kind)
        self.send_header('Content-Length', str(len(raw)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Content-Security-Policy', "frame-ancestors 'none'")
        if cookie:
            self.send_header('Set-Cookie', cookie)
        self.end_headers()
        try:
            self.wfile.write(raw)
        except (BrokenPipeError, ConnectionResetError):
            pass  # A closed phone page does not cancel an accepted mapping command.

    def authenticated(self):
        jar = cookies.SimpleCookie()
        try:
            jar.load(self.headers.get('Cookie', ''))
            return hmac.compare_digest(jar['s10'].value, login_cookie)
        except (KeyError, cookies.CookieError):
            return False

    def do_GET(self):
        global last_view, last_height_view
        if self.path == '/':
            return self.reply(200, (HERE/'index.html').read_bytes(), 'text/html; charset=utf-8')
        if self.path in ('/localization', '/heightmap', '/field', '/imu-check', '/native-nav'):
            page = HERE/({'/imu-check': 'imu_diag.html', '/native-nav': 'native_nav.html'}.get(self.path, self.path[1:]+'.html') if self.authenticated() else 'index.html')
            if not page.is_file():
                return self.reply(404, dict(detail='当前地图的定位页面尚未准备'))
            return self.reply(200, page.read_bytes(), 'text/html; charset=utf-8')
        if not self.authenticated():
            return self.reply(401, dict(detail='请登录'))
        if self.path == '/native_nav.js':
            return self.reply(200, (HERE/'native_nav.js').read_bytes(), 'text/javascript; charset=utf-8')
        if self.path.startswith('/phone/native/'):
            return self.native_get()
        if self.path == '/imu_diag.js':
            return self.reply(200, (HERE/'imu_diag.js').read_bytes(), 'text/javascript; charset=utf-8')
        if self.path.startswith('/phone/imu/'):
            return self.imu_get()
        if self.path == '/field.js':
            return self.reply(200, (HERE/'field.js').read_bytes(), 'text/javascript; charset=utf-8')
        if self.path.startswith('/phone/field/'):
            return self.field_get()
        if self.path == '/phone/heightmap':
            last_height_view = time.monotonic()
            with guard:
                result = dict(height_state)
                result['transport_age'] = last_height_view-result.get('received', 0)
                if result['transport_age'] > 5:
                    result['online'] = False
            return self.reply(200, result)
        if self.path == '/phone/state':
            last_view = time.monotonic()
            with guard:
                result = dict(state, operation=operation_name, csrf=csrf)
                if last_view-result.get('received', 0) > 5:
                    result['online'] = False
            return self.reply(200, result)
        if self.path == '/phone/maps':
            try:
                return self.reply(200, call('maps'))
            except Exception as exc:
                return self.reply(502, dict(detail=str(exc)))
        self.reply(404, dict(detail='没有这个页面'))

    def do_POST(self):
        global operation_name
        try:
            if self.headers.get('Content-Type', '').split(';')[0] != 'application/json':
                return self.reply(415, dict(detail='需要 JSON'))
            size = int(self.headers.get('Content-Length', '0'))
            if not 0 < size <= 4096:
                return self.reply(400, dict(detail='请求大小无效'))
            body = json.loads(self.rfile.read(size))
            if not isinstance(body, dict):
                raise ValueError('请求格式无效')
            if self.path == '/phone/login':
                digest = hashlib.sha256((str(body.get('username'))+':'+str(body.get('password'))).encode()).hexdigest()
                if not hmac.compare_digest(digest, config['login_hash']):
                    time.sleep(1)
                    return self.reply(401, dict(detail='用户名或密码错误'))
                return self.reply(200, dict(message='已登录'), cookie=f's10={login_cookie}; HttpOnly; SameSite=Strict; Path=/')
            if not self.authenticated() or not hmac.compare_digest(self.headers.get('X-CSRF-Token', ''), csrf):
                return self.reply(403, dict(detail='请刷新页面重新登录'))
            if self.path == '/phone/native/submit':
                try:
                    if body.get('action') not in ('submit', 'cancel', 'heartbeat'):
                        raise FieldError('无效导航操作')
                    return self.reply(202, native_call(body))
                except Exception as exc:
                    return self.reply(error_status(exc), dict(detail=str(exc), code=getattr(exc, 'code', 'unavailable')))
            if self.path == '/phone/imu/submit':
                try:
                    if body.get('action') not in ('start','stop','event','zero_start','zero_clear'):
                        raise FieldError('此入口仅允许诊断采集、标记事件或本次显示参考复零；不允许硬件校准')
                    return self.reply(202, imu_call(body))
                except Exception as exc:
                    return self.reply(error_status(exc), dict(detail=str(exc)))
            if self.path == '/phone/field/submit':
                try:
                    return self.reply(202, field_call(dict(action='submit', request=body)))
                except Exception as exc:
                    return self.reply(error_status(exc), dict(detail=str(exc), code=getattr(exc, 'code', 'unavailable')))
            if self.path not in ('/phone/start', '/phone/save'):
                return self.reply(404, dict(detail='无效操作'))
            if not operation.acquire(blocking=False):
                return self.reply(409, dict(detail='正在处理建图操作'))
            try:
                with guard:
                    if not state.get('online') or time.monotonic()-state.get('received', 0) > 5:
                        raise ValueError('定位板连接尚未就绪')
                    if self.path.endswith('start'):
                        if state['active'] or any(state['streams'].get(k, {}).get('age', 999) > 3 or state['streams'].get(k, {}).get('error') for k in ('raw', 'imu')):
                            raise ValueError('已有会话或雷达、IMU 未就绪')
                        if any(not -.25 <= state['streams'][k].get('stamp_age_s', float('inf')) <= 1 for k in ('raw', 'imu')):
                            raise ValueError('雷达或 IMU 测量时间异常，不能开始建图')
                    elif not state.get('owned'):
                        raise ValueError('当前建图不属于手机会话')
                operation_name = 'starting' if self.path.endswith('start') else 'saving'
                result = call('start', name=body.get('name'), mode=body.get('mode')) if self.path.endswith('start') else call('save')
                return self.reply(200, result)
            finally:
                operation_name = ''
                operation.release()
        except Exception as exc:
            self.reply(400, dict(detail=str(exc)))

    def log_message(self, format, *args):
        pass

    def native_get(self):
        try:
            parsed = urlsplit(self.path)
            params = parse_qs(parsed.query, strict_parsing=True)
            if parsed.path == '/phone/native/status' and not params:
                result = native_call(dict(action='status'))
                return self.reply(200, dict(result, csrf=csrf))
            if parsed.path == '/phone/native/report' and set(params) == {'id'} and len(params['id']) == 1:
                from re import fullmatch
                if not fullmatch('[a-f0-9]{32}', params['id'][0]):
                    raise FieldError('测试编号无效')
                return self.reply(200, native_call(dict(action='report', id=params['id'][0])))
            raise FieldError('无效导航查询')
        except Exception as exc:
            return self.reply(error_status(exc), dict(detail=str(exc), code=getattr(exc, 'code', 'unavailable')))

    def field_get(self):
        try:
            parsed = urlsplit(self.path)
            path, params = parsed.path, parse_qs(parsed.query, strict_parsing=True)
            allowed = {'/phone/field/list': ('list', 'session_id'),
                       '/phone/field/dashboard': ('dashboard', 'session_id'),
                       '/phone/field/job': ('job', 'job_id'),
                       '/phone/field/session': ('session', 'session_id'),
                       '/phone/field/preview': ('preview', 'session_id')}
            if path in ('/phone/field/health', '/phone/field/live'):
                if params:
                    raise FieldError('无效查询参数')
                result = field_call(dict(action=path.rsplit('/', 1)[1]))
                if path.endswith('health'):
                    result['csrf'] = csrf
                return self.reply(200, result)
            if path in allowed:
                action, key = allowed[path]
                if set(params)-{key} or any(len(v) != 1 for v in params.values()):
                    raise FieldError('无效查询参数')
                request = dict(action=action)
                if key in params:
                    request[key] = ident(params[key][0])
                elif action not in ('list', 'dashboard'):
                    raise FieldError('缺少查询编号')
                result = field_call(request)
                if action == 'dashboard':
                    result['health']['csrf'] = csrf
                return self.reply(200, result)
            if path == '/phone/field/download':
                if set(params) != {'artifact_id'} or len(params['artifact_id']) != 1:
                    raise FieldError('产物编号无效')
                return self.download(ident(params['artifact_id'][0]))
            return self.reply(404, dict(detail='不存在这个现场接口'))
        except Exception as exc:
            return self.reply(error_status(exc), dict(detail=str(exc), code=getattr(exc, 'code', 'unavailable')))

    def imu_get(self):
        global imu_last_view, imu_thread
        try:
            parsed=urlsplit(self.path);params=parse_qs(parsed.query,strict_parsing=True)
            if any(len(v)!=1 for v in params.values()):raise FieldError('重复参数')
            if parsed.path=='/phone/imu/live':
                if set(params)-{'cursor'}:raise FieldError('未知参数')
                value=params.get('cursor',['0'])[0]
                if not value.isdigit() or len(value)>12:raise FieldError('游标无效')
                imu_last_view=time.monotonic()
                with guard:
                    if imu_thread is None:
                        imu_thread=threading.Thread(target=read_imu_stream,daemon=True);imu_thread.start()
                    row=dict(imu_state,csrf=csrf,transport_age=imu_last_view-imu_state.get('received',0))
                    row['rows']=[b for b in imu_state.get('rows',[]) if b['seq']>int(value)][-600:]
                    if row['transport_age']>2:row['online']=False
                return self.reply(200,row)
            if parsed.path=='/phone/imu/list':
                if params:raise FieldError('未知参数')
                return self.reply(200,imu_call(dict(action='list')))
            if parsed.path in ('/phone/imu/report','/phone/imu/download'):
                if set(params)!={'id'}:raise FieldError('需要诊断编号')
                sid=ident(params['id'][0])
                if parsed.path.endswith('download'):return self.download(sid,provider=imu_call)
                return self.reply(200,imu_call(dict(action='report',id=sid)))
            return self.reply(404,dict(detail='没有这个诊断接口'))
        except Exception as exc:return self.reply(error_status(exc),dict(detail=str(exc)))

    def download(self, artifact_id, provider=field_call):
        import re
        info = provider(dict(action='artifact_info', artifact_id=artifact_id))
        size = info['size']
        start, end, partial = 0, size-1, False
        value = self.headers.get('Range')
        if value:
            match = re.fullmatch(r'bytes=(\d+)-(\d*)', value)
            if not match:
                return self.reply(416, dict(detail='仅支持单个 bytes=start-end 范围'))
            start = int(match[1]); end = min(int(match[2]), size-1) if match[2] else size-1
            if start > end or start >= size:
                return self.reply(416, dict(detail='下载范围超出文件'))
            partial = True
        self.send_response(206 if partial else 200)
        self.send_header('Content-Type', 'application/octet-stream')
        self.send_header('Content-Length', str(max(0, end-start+1)))
        self.send_header('Accept-Ranges', 'bytes')
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('ETag', '"'+info['sha256']+'"')
        self.send_header('Content-Disposition', "attachment; filename*=UTF-8''"+quote(info['name'], safe=''))
        if partial:
            self.send_header('Content-Range', f'bytes {start}-{end}/{size}')
        self.end_headers()
        try:
            offset = start
            while offset <= end:
                length = min(262144, end-offset+1)
                chunk = provider(dict(action='artifact_read', artifact_id=artifact_id, offset=offset, length=length))
                raw = base64.b64decode(chunk['data'], validate=True)
                if chunk['offset'] != offset or len(raw) != length:
                    raise RuntimeError('下载数据范围与请求不符')
                self.wfile.write(raw)
                offset += len(raw)
        except Exception:
            # Once headers were sent, abort rather than send a second HTTP reply
            # or claim a truncated body is a complete download.
            self.close_connection = True


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='10.21.33.102')
    parser.add_argument('--port', type=int, default=8080)
    args = parser.parse_args()
    config = json.loads(CONFIG.read_text())
    threading.Thread(target=read_stream, daemon=True).start()
    threading.Thread(target=read_height_stream, daemon=True).start()
    ThreadingHTTPServer((args.host, args.port), Handler).serve_forever()
