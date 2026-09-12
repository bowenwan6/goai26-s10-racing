"""Run with python tools/s10_gait_capture/test_snapshot.py; no ROS needed."""
from pathlib import Path
import tempfile
import threading
from unittest.mock import patch

from server import Recorder, atomic_json, create_app


def check():
    with tempfile.TemporaryDirectory() as root:
        recorder = Recorder(root, demo=True)
        folders = []
        for i in range(105):
            folder = Path(root) / f'gait_20260101_000000_{i:012x}'
            folder.mkdir()
            atomic_json(folder/'manifest.json', {'id': folder.name, 'status': 'stopped', 'events': []})
            folders.append(folder)
        # A partial directory and ZIP must not displace real history entries.
        (Path(root)/'gait_99999999_000000_000000000000').mkdir()
        (Path(root)/'gait_99999999_000000_000000000001.zip').touch()
        first = recorder.snapshot()
        expected = [p.name for p in reversed(folders[-100:])]
        assert [r['id'] for r in first['history']] == expected
        assert len(recorder._history_cache) == 100
        first['history'][0]['events'].append('must not leak')
        # Warm polling reads no manifest contents; replies remain independent.
        with patch.object(Path, 'read_text', side_effect=AssertionError('unexpected reread')):
            assert recorder.snapshot()['history'][0]['events'] == []
        latest = folders[-1]/'manifest.json'
        atomic_json(latest, {'id': folders[-1].name, 'status': 'failed', 'events': ['changed']})
        assert recorder.snapshot()['history'][0]['events'] == ['changed']
        latest.unlink()
        assert recorder.snapshot()['history'][0]['id'] == folders[-2].name
        assert folders[-1].name not in recorder._history_cache

        client = create_app(recorder, token='test').test_client()
        assert client.get('/api/state').status_code == 401
        csrf = client.post('/api/login', json={'token': 'test'}).json['csrf']
        def action(**payload):
            response = client.post('/api/action', json={'request_id': 'request-'+payload['action'], **payload},
                                   headers={'X-CSRF-Token': csrf})
            assert response.status_code == 200, response.json
            return response.json
        sid = action(action='start', terrain='basic', allow_partial=True)['session_id']
        assert client.get('/api/state').json['active']['id'] == sid
        action(action='mark', session_id=sid, label='slip')
        state = client.get('/api/state').json
        assert next(r for r in state['history'] if r['id'] == sid)['events'][0]['label'] == 'slip'
        action(action='stop', session_id=sid, outcome='success')
        state = client.get('/api/state').json
        assert state['active'] is None
        assert next(r for r in state['history'] if r['id'] == sid)['status'] == 'stopped'

        # A stalled filesystem read in a status request must not stall ingest.
        entered, release, ingested = threading.Event(), threading.Event(), threading.Event()
        original = Path.read_text
        errors = []
        def slow_read(path, *args, **kwargs):
            entered.set()
            assert release.wait(5), 'test did not release reader'
            return original(path, *args, **kwargs)
        def snapshot():
            try:
                recorder.snapshot()
            except BaseException as exc:
                errors.append(exc)
        def ingest():
            recorder.ingest('/IMU', 'demo/Synthetic', b'', 123)
            ingested.set()
        recorder._history_cache = {}
        with patch.object(Path, 'read_text', slow_read):
            reader = threading.Thread(target=snapshot)
            reader.start()
            try:
                assert entered.wait(2), 'reader did not start'
                writer = threading.Thread(target=ingest)
                writer.start()
                assert ingested.wait(1), 'history read blocked ingestion'
            finally:
                release.set()
                reader.join(5)
                if 'writer' in locals():
                    writer.join(5)
        assert not reader.is_alive() and not errors, errors
        assert recorder.stats['/IMU']['count'] == 1
    print('Snapshot cache, API lifecycle and ingestion concurrency checks passed')


if __name__ == '__main__':
    check()
