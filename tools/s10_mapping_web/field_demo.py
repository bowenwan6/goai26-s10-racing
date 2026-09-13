"""Loopback-only demo with a separate fake worker; never imports robot adapter."""
import argparse
import hashlib
import multiprocessing
from pathlib import Path
import secrets
import tempfile
import time
from http.server import ThreadingHTTPServer


def fake_worker(root, socket_path):
    from field_fake import FakeAdapter
    from field_worker import run
    run(Path(root), Path(socket_path), FakeAdapter(), Path(root)/'operation.lock')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=18080)
    parser.add_argument('--password', help='DEMO only; omitted generates a one-time demo password')
    args = parser.parse_args()
    password = args.password or secrets.token_urlsafe(12)
    with tempfile.TemporaryDirectory(prefix='s10-field-demo-') as directory:
        root = Path(directory)
        socket_path = root/'worker.sock'
        process = multiprocessing.Process(target=fake_worker, args=(str(root), str(socket_path)))
        process.start()
        deadline = time.monotonic()+10
        while not socket_path.exists() and time.monotonic() < deadline:
            time.sleep(.05)
        import server
        from field_worker import rpc_call
        server.field_transport = lambda request: rpc_call(request, socket_path)
        def no_real_backend(*args, **kwargs):
            raise RuntimeError('演示模式禁止访问真实SSH/旧建图后台')
        server.call = no_real_backend
        server.config = dict(login_hash=hashlib.sha256(('demo:'+password).encode()).hexdigest())
        # No real SSH stream thread, no production configuration, no wildcard bind.
        print(f'DEMO ONLY http://127.0.0.1:{args.port}/field username=demo password={password}', flush=True)
        try:
            with ThreadingHTTPServer(('127.0.0.1', args.port), server.Handler) as http:
                http.serve_forever()
        finally:
            process.terminate(); process.join(10)
