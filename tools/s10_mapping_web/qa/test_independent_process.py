"""Restart/client-loss tests of a separate local fake worker, not robot services."""
import json
import os
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest
import uuid
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from field_core import Store
from field_worker import Engine, RPCServer, rpc_call
from test_independent_worker import FakeEvidenceAdapter


class WorkerProcessTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='s10-qa-')
        self.root = Path(self.tmp.name)
        self.socket = self.root/'qa.sock'
        self.process = None
        self.start()

    def tearDown(self):
        self.stop()
        self.tmp.cleanup()

    def start(self):
        self.process = subprocess.Popen([sys.executable, '-B', str(Path(__file__).with_name('worker_fixture.py')), str(self.root)],
                                        stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        deadline = time.monotonic()+5
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                self.fail('local worker exited: '+self.process.stderr.read().decode())
            try:
                if self.call('health')['demo'] is True:
                    return
            except (OSError, ValueError):
                time.sleep(.05)
        self.fail('local worker startup timed out')

    def stop(self):
        if self.process:
            self.process.terminate()
            try: self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill(); self.process.wait(timeout=3)
            self.process.stderr.close()
            self.process = None

    def call(self, action, **kwargs):
        return rpc_call(dict(action=action, **kwargs), self.socket)

    def await_state(self, job_id, accepted):
        deadline = time.monotonic()+5
        while time.monotonic() < deadline:
            job = self.call('job', job_id=job_id)
            if job['state'] in accepted: return job
            time.sleep(.05)
        self.fail('expected state '+str(accepted)+', last '+str(job))

    def selfcheck(self):
        job = self.call('submit', request=dict(action='selfcheck', key=uuid.uuid4().hex, params=dict(target_map='test_map')))
        return self.await_state(job['id'], {'SUCCEEDED'})

    def test_client_closes_after_acceptance_does_not_cancel_or_duplicate(self):
        request = dict(action='selfcheck', key=uuid.uuid4().hex, params=dict(target_map='test_map'))
        with socket.socket(socket.AF_UNIX) as client:
            client.connect(str(self.socket))
            client.sendall((json.dumps(dict(action='submit', request=request))+'\n').encode())
        deadline = time.monotonic()+5
        while time.monotonic() < deadline:
            jobs = self.call('list')['jobs']
            if jobs: break
            time.sleep(.05)
        self.assertEqual(len(jobs), 1)
        job = self.await_state(jobs[0]['id'], {'SUCCEEDED'})
        self.assertEqual(self.call('submit', request=request)['id'], job['id'])
        self.assertEqual(len(self.call('list')['jobs']), 1)

    def test_worker_restart_marks_running_interrupted_not_replayed(self):
        first = self.selfcheck()
        job = self.call('submit', request=dict(action='localization_check', key=uuid.uuid4().hex,
                        session_id=first['session_id'], params=dict(stationary=True)))
        self.await_state(job['id'], {'RUNNING'})
        self.stop(); self.start()
        restored = self.call('job', job_id=job['id'])
        self.assertEqual(restored['state'], 'INTERRUPTED')
        time.sleep(.3)
        self.assertEqual(self.call('job', job_id=job['id'])['state'], 'INTERRUPTED')
        self.assertEqual(len(self.call('list')['jobs']), 2)
        self.assertEqual(os.stat(self.socket).st_mode & 0o777, 0o600)


class PreviewTransportTests(unittest.TestCase):
    def test_realistic_thirty_thousand_point_preview_is_not_truncated(self):
        with tempfile.TemporaryDirectory(prefix='s10-rpc-qa-') as temp:
            root = Path(temp); store = Store(root/'persist'); adapter = FakeEvidenceAdapter()
            adapter.preview = lambda target: dict(map_name=target, map_identity='fake', frame='map',
                            points=[[123.456, 789.012, -123.456] for _ in range(29897)])
            engine = Engine(store, adapter, root/'operation.lock')
            job = store.submit(dict(action='selfcheck', key=uuid.uuid4().hex, params=dict(target_map='test_map')))
            socket_path = root/'qa.sock'
            with RPCServer(str(socket_path), __import__('field_worker').RPCHandler) as server:
                server.engine = engine
                thread = threading.Thread(target=server.serve_forever, daemon=True); thread.start()
                try:
                    result = rpc_call(dict(action='preview', session_id=job['session_id']), socket_path)
                    self.assertEqual(len(result['points']), 29897)
                finally:
                    server.shutdown(); thread.join(2)


if __name__ == '__main__':
    unittest.main(verbosity=2)
