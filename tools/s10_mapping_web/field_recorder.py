"""Private child guardian: bounds raw recorder independent of HTTP/worker life."""
import argparse
import json
import os
import select
import shutil
import signal
import subprocess
import time
from pathlib import Path


def supervise(argv, root, seconds, parent_fd, max_bytes=512*1024**2, reserve=2*1024**3):
    if not 10 <= seconds <= 30:
        raise ValueError('Recorder duration must be 10–30 seconds')
    root = Path(root).resolve(strict=True)
    started = time.monotonic()
    child = subprocess.Popen(argv, stdin=subprocess.DEVNULL, start_new_session=True)
    reason, stopped = 'unknown', False
    def on_signal(signum, frame):
        nonlocal stopped
        stopped = True
    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)
    try:
        while child.poll() is None:
            if stopped:
                reason = 'guardian_interrupted'; break
            if select.select([parent_fd], [], [], 0)[0] and not os.read(parent_fd, 1):
                reason = 'worker_disconnected'; break
            if time.monotonic()-started >= seconds:
                reason = 'duration_complete'; break
            size = sum(p.stat().st_size for p in root.rglob('*') if p.is_file() and not p.is_symlink())
            if size > max_bytes:
                reason = 'size_limit'; break
            if shutil.disk_usage(root).free < reserve:
                reason = 'disk_reserve'; break
            time.sleep(.1)
        else:
            reason = 'recorder_exited_early'
    finally:
        if child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            try:
                child.wait(timeout=15)
            except subprocess.TimeoutExpired:
                reason = 'finalization_timeout'
                os.killpg(child.pid, signal.SIGTERM)
                try:
                    child.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL); child.wait(timeout=3)
        value = dict(reason=reason, returncode=child.returncode, elapsed_s=time.monotonic()-started,
                     max_bytes=max_bytes, disk_reserve=reserve,
                     note='部分文件保留；停止录制不停车')
        partial = root/'recorder-result.partial'
        with partial.open('x') as f:
            json.dump(value, f); f.flush(); os.fsync(f.fileno())
        partial.replace(root/'recorder-result.json')
    return 0 if reason == 'duration_complete' and child.returncode in (0, -signal.SIGINT) else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--seconds', type=int, required=True)
    parser.add_argument('--parent-fd', type=int, required=True)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    # This is not an RPC endpoint; only the worker constructs this process argv.
    if not command or command[:3] != ['ros2', 'bag', 'record']:
        parser.error('Only ros2 bag record is supported')
    raise SystemExit(supervise(command, args.root, args.seconds, args.parent_fd))
