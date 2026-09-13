"""LOCAL TEST ONLY: simulated evidence adapter, Unix socket, no robot/ROS/SSH."""
import argparse
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from field_worker import run
from test_independent_worker import FakeEvidenceAdapter


class SlowFakeAdapter(FakeEvidenceAdapter):
    def selfcheck(self, target):
        time.sleep(.5)
        return super().selfcheck(target)

    def sample(self, duration, progress):
        deadline = time.monotonic()+duration
        while time.monotonic() < deadline:
            progress('LOCAL QA simulated sampling; no robot connected')
            time.sleep(.2)
        return super().sample(duration, progress)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('root', type=Path)
    p.add_argument('--socket-under-root', action='store_true', help='QA layout matching persistent production socket')
    args = p.parse_args()
    socket_path = args.root/'persist'/'worker.sock' if args.socket_under_root else args.root/'qa.sock'
    run(args.root/'persist', socket_path, SlowFakeAdapter(), args.root/'operation.lock')
