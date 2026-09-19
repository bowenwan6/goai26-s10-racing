"""Loopback-only E2E fixture. No production config, robot adapter or SSH."""
import hashlib
import argparse
from http.server import ThreadingHTTPServer
import json
import multiprocessing
from pathlib import Path
import sys
import tempfile
import time

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from field_fake import FakeAdapter
from field_worker import run, rpc_call

class LostAdapter(FakeAdapter):
    def snapshot(self):
        snap=super().snapshot()
        snap['status'].update(code=3,mode='局部',label='定位丢失')
        return snap

def fake_worker(root,sock,lost=False):
    adapter=LostAdapter() if lost else FakeAdapter()
    adapter.map_name='0914_fr_v3-20260914-142008'
    run(Path(root),Path(sock),adapter,Path(root)/'operation.lock')

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--lost',action='store_true');args=parser.parse_args()
    with tempfile.TemporaryDirectory(prefix='s10-revisit-demo-') as directory:
        root=Path(directory);sock=root/'worker.sock'
        process=multiprocessing.Process(target=fake_worker,args=(str(root),str(sock),args.lost));process.start()
        try:
            deadline=time.monotonic()+10
            while not sock.exists() and time.monotonic()<deadline:time.sleep(.05)
            import server
            server.field_transport=lambda req:rpc_call(req,sock)
            def forbidden(*args,**kwargs):raise AssertionError('Robot/SSH forbidden in local QA')
            server.call=forbidden
            server.config=dict(login_hash=hashlib.sha256(b'demo:revisit-demo-only').hexdigest())
            with ThreadingHTTPServer(('127.0.0.1',0),server.Handler) as http:
                print(json.dumps(dict(port=http.server_port,demo=True)),flush=True)
                http.serve_forever()
        finally:
            process.terminate();process.join(10)
