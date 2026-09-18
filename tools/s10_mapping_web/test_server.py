"""Offline parser and save-order checks; no robot or ROS imports."""
import json
from pathlib import Path
import struct
import sys
import tempfile
from types import SimpleNamespace as NS
from unittest.mock import patch

import robot_backend as server


def check():
    import time
    stamp = time.mktime(time.strptime('2026-09-11 20:50:10', '%Y-%m-%d %H:%M:%S'))
    log = '[2026-09-11 20:50:10.123.456] [INFO ] [monitor] 上报状态=0(正常), 可用=0, 运行状态=全局'
    assert server.parse_localization_status(log, stamp+.2)['code'] == 0
    assert not server.parse_localization_status(log, stamp+10)['fresh']
    assert not server.parse_localization_status(log, stamp+.2, stamp+1)['fresh']
    lost = log.replace('0(正常)', '3(定位丢失)').replace('全局', '局部')
    assert server.parse_localization_status(log+'\n'+lost, stamp+.2)['code'] == 3
    assert 'error' not in server.measurement_time(100, 10, 100.01)
    assert 'error' in server.measurement_time(100, 10, 100+15638400)
    assert 'error' in server.measurement_time(200, 11, 200.01, dict(stamp=100, received=10))
    vendor = NS(start_mapping=lambda *a, **k: (_ for _ in ()).throw(AssertionError('Unsafe start')))
    with patch.dict(sys.modules, {'map_manager.services.mapping': NS(MappingService=vendor)}), \
         patch.object(server, 'service', return_value=dict(ActiveState='inactive')), \
         patch.object(server.subprocess, 'run', return_value=NS(returncode=1)), \
         patch.object(server, 'run', side_effect=RuntimeError('IMU clock is old')):
        try:
            server.dispatch(dict(action='start', name='test', mode='indoor'))
        except ValueError as exc:
            assert '测量时间' in str(exc)
        else:
            raise AssertionError('Preflight failure did not block vendor start')
    for big in (False, True):
        # Organized cloud: two rows, padding after each row, one NaN point.
        endian = '>' if big else '<'
        data = struct.pack(endian+'fff', 1, 2, 3)+b'PAD!'
        data += struct.pack(endian+'fff', float('nan'), 2, 3)+b'PAD!'
        msg = NS(fields=[NS(name=k, datatype=7, offset=i*4) for i, k in enumerate('xyz')],
                 width=1, height=2, point_step=12, row_step=16, data=data, is_bigendian=big)
        assert server.cloud_points(msg) == [[1.0, 2.0, 3.0]]
        msg.data = data[:-1]
        try:
            server.cloud_points(msg)
        except ValueError:
            pass
        else:
            raise AssertionError('Truncated cloud accepted')
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        for name in ('full_cloud.pcd', 'occ_grid.yaml', 'occ_grid.pgm'):
            (root/name).write_text('data')
        info = dict(ActiveState='active', InvocationID='a'*32)
        (root/'session.json').write_text(json.dumps(dict(invocation='a'*32, map_name=root.name)))
        calls = []

        def run(argv, **kwargs):
            calls.append(argv)
            if argv[0] == 'journalctl':
                return '地图后处理完成' if any(c[0] == 'drsec' for c in calls) else ''
            return ''

        with patch.object(server, 'HERE', root), \
             patch.object(server, 'service', return_value=info), \
             patch.object(server, 'map_directory', return_value=root), \
             patch.object(server, 'run', side_effect=run):
            result = server.save_mapping()
        assert result['map_name'] == root.name
        assert [c[0] for c in calls] == ['journalctl', 'drsec', 'journalctl', 'systemctl', 'systemctl']

        calls.clear()
        (root/'session.json').write_text(json.dumps(dict(invocation='a'*32, map_name=root.name)))

        def fail_save(argv, **kwargs):
            calls.append(argv)
            if argv[0] == 'drsec':
                raise RuntimeError('Save failed')
            return ''

        with patch.object(server, 'HERE', root), \
             patch.object(server, 'service', return_value=info), \
             patch.object(server, 'map_directory', return_value=root), \
             patch.object(server, 'run', side_effect=fail_save):
            try:
                server.save_mapping()
            except RuntimeError:
                pass
            else:
                raise AssertionError('Save error hidden')
        assert not any(c[0] == 'systemctl' for c in calls), 'Failed save must keep mapping alive'
    # Exercise the HTTP trust boundary without contacting or starting the robot.
    import hashlib
    import threading
    from urllib.request import Request, build_opener, HTTPCookieProcessor, ProxyHandler
    from urllib.error import HTTPError
    from http.server import ThreadingHTTPServer
    import server as web
    web.config = dict(login_hash=hashlib.sha256(b'golai:test').hexdigest())
    http = ThreadingHTTPServer(('127.0.0.1', 0), web.Handler)
    threading.Thread(target=http.serve_forever, daemon=True).start()
    client = build_opener(ProxyHandler({}), HTTPCookieProcessor())
    base = 'http://127.0.0.1:'+str(http.server_port)

    def request(path, body=None, headers=None):
        req = Request(base+path, data=json.dumps(body).encode() if body is not None else None,
                      headers={'Content-Type': 'application/json', **(headers or {})})
        try:
            response = client.open(req, timeout=3)
        except HTTPError as exc:
            response = exc
        with response:
            return response.code, json.loads(response.read())

    try:
        with patch.object(web, 'call') as remote:
            assert request('/phone/state')[0] == 401
            assert request('/phone/heightmap')[0] == 401
            assert request('/phone/login', dict(username='golai', password='test'))[0] == 200
            assert request('/phone/heightmap')[1]['online'] is False
            assert request('/phone/start', dict(name='test', mode='indoor'))[0] == 403
            assert request('/phone/state')[0] == 200
            # Even with valid CSRF, stale telemetry must not invoke the robot.
            assert request('/phone/start', dict(name='test', mode='indoor'), {'X-CSRF-Token': web.csrf})[0] == 400
            remote.assert_not_called()
            web.state = dict(online=True, received=web.time.monotonic(), active=False,
                             streams={k: dict(age=0, stamp_age_s=15638400) for k in ('raw', 'imu')})
            assert request('/phone/start', dict(name='test', mode='indoor'), {'X-CSRF-Token': web.csrf})[0] == 400
            remote.assert_not_called()
            for stream in web.state['streams'].values():
                stream['stamp_age_s'] = .01
            remote.return_value = {}
            assert request('/phone/start', dict(name='test', mode='indoor'), {'X-CSRF-Token': web.csrf})[0] == 200
            remote.assert_called_once_with('start', name='test', mode='indoor')
    finally:
        http.shutdown(); http.server_close()
    print('MAPPING_WEB_CHECK_OK')


if __name__ == '__main__':
    check()
