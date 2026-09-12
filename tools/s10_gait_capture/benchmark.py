"""ROS-free benchmark: python tools/s10_gait_capture/benchmark.py --output result.json.

Use --server /path/to/server.py to compare the same workload against another copy.
Synthetic history, Flask test client (no network), warm OS cache; no robot traffic.
"""
import argparse
import cProfile
import importlib.util
import io
import json
from pathlib import Path
import platform
import pstats
import statistics
import struct
import tempfile
import threading
import time
from types import SimpleNamespace as NS
from importlib.metadata import version


class TimedLock:
    """Measure the outermost acquisition's hold time without changing locking."""
    def __init__(self):
        self.lock = threading.RLock()
        self.depth = 0
        self.holds = []

    def __enter__(self):
        self.lock.acquire()
        if self.depth == 0:
            self.started = time.perf_counter_ns()
        self.depth += 1
        return self

    def __exit__(self, *args):
        self.depth -= 1
        if self.depth == 0:
            self.holds.append((time.perf_counter_ns() - self.started) / 1e6)
        self.lock.release()


def summary(samples):
    return {'median_ms': statistics.median(samples),
            'p95_ms': sorted(samples)[max(0, int(len(samples) * .95 + .999) - 1)]}


def measure(fn, repeats):
    for _ in range(5):
        fn()
    samples = []
    for _ in range(repeats):
        start = time.perf_counter_ns()
        fn()
        samples.append((time.perf_counter_ns() - start) / 1e6)
    return summary(samples)


def run(server, repeats):
    result = {}
    with tempfile.TemporaryDirectory(prefix='s10-benchmark-') as root:
        recorder = server.Recorder(root, demo=True)
        recorder.lock = TimedLock()
        client = server.create_app(recorder, token='benchmark').test_client()
        assert client.post('/api/login', json={'token': 'benchmark'}).status_code == 200

        def state():
            response = client.get('/api/state')
            assert response.status_code == 200
            return response

        result['state_empty'] = measure(state, repeats)
        # Build real manifest structure through the application, then fixed history.
        recorder.action({'action': 'start', 'request_id': 'benchmark-start',
                         'terrain': 'basic', 'allow_partial': True, 'duration_s': 1800})
        for topic in server.TOPICS:
            recorder.ingest(topic, 'demo/Synthetic', b'', 123)
        recorder.observe_motion({'gait': 0x1001, 'state': 17}, 123)
        record = recorder.active
        recorder._stop('stopped', '')
        record.update(started_wall_ns=1000000000, stopped_wall_ns=13000000000, duration_s=12.0)
        for topic in record['topics'].values():
            topic['last_receive_monotonic'] = 1.0
        event = {**record['motion_feedback_events'][0], 'elapsed_s': 1.0, 'receive_wall_ns': 1000000000}
        record['motion_feedback_events'] = [event] * 30
        original = Path(root) / record['id']
        for path in original.iterdir():
            path.unlink()
        original.rmdir()
        for i in range(100):
            record['id'] = f'gait_20260909_120000_{i:012x}'
            folder = Path(root) / record['id']
            folder.mkdir()
            server.atomic_json(folder / 'manifest.json', record)
        result['history_bytes'] = sum(p.stat().st_size for p in Path(root).glob('*/manifest.json'))
        result['state_100'] = measure(state, repeats)
        result['state_response_bytes'] = len(state().data)
        result['snapshot_100'] = measure(recorder.snapshot, repeats)
        recorder.lock.holds.clear()
        for _ in range(repeats):
            recorder.snapshot()
        result['snapshot_lock_hold'] = summary(recorder.lock.holds)

        profile = cProfile.Profile()
        profile.runcall(lambda: [recorder.snapshot() for _ in range(20)])
        output = io.StringIO()
        pstats.Stats(profile, stream=output).strip_dirs().sort_stats('cumulative').print_stats(15)
        result['snapshot_profile'] = output.getvalue()

        recorder.action({'action': 'start', 'request_id': 'benchmark-start-2',
                         'terrain': 'basic', 'allow_partial': True, 'duration_s': 1800})
        def ingest_tick():
            for _ in range(100):
                recorder.ingest('/IMU', 'demo/Synthetic', b'', time.time_ns())
                recorder.tick()
        result['ingest_tick_100'] = measure(ingest_tick, repeats)
        recorder.lock.holds.clear()
        result['state_100_recording'] = measure(state, repeats)
        result['state_recording_lock_hold'] = summary(recorder.lock.holds)
        recorder._stop('stopped', '')

        cloud = NS(width=100000, height=1, point_step=16, row_step=1600000,
                   data=struct.pack('<ffff', 1.125, 2.25, 3.5, 0) * 100000,
                   is_bigendian=False,
                   fields=[NS(name=n, offset=i*4, datatype=7, count=1)
                           for i, n in enumerate(('x', 'y', 'z'))],
                   header=NS(frame_id='lidar', stamp=NS(sec=1, nanosec=2)))
        result['preview_100k'] = measure(lambda: server.preview_points(cloud), repeats)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--server', type=Path, default=Path(__file__).with_name('server.py'))
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--repeats', type=int, default=50)
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error('--repeats must be positive')
    spec = importlib.util.spec_from_file_location('capture_server', args.server.resolve())
    server = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(server)
    results = {'environment': {'platform': platform.platform(), 'python': platform.python_version(),
                              'flask': version('flask'), 'repeats': args.repeats},
               **run(server, args.repeats)}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(results, indent=2), encoding='utf-8')
    print(json.dumps(results, indent=2))
