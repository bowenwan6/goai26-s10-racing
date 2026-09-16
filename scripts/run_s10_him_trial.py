"""Windows SSH launcher; defaults to dry mode. Live modes actuate the selected robot."""
import argparse
import json
import os
from pathlib import Path, PurePosixPath
import re
import shlex
import subprocess
import sys
import threading
import time
from collections import deque

from s10_him_trial import manual_request, handset_request


def paths(root, source_install):
    if not re.fullmatch(r'/(?:tmp|home/[A-Za-z0-9_-]+)/s10-sdk-isolated-[A-Za-z0-9_-]+', root):
        raise ValueError('Use the isolated root printed by prepare_s10_sdk_copy.py')
    if not source_install.startswith('/') or '..' in PurePosixPath(source_install).parts:
        raise ValueError('source-install must be a verified absolute AGX path')
    return PurePosixPath(root), PurePosixPath(source_install)


def check():
    assert str(paths('/tmp/s10-sdk-isolated-abc_123', '/home/team/ws/install')[0]) == '/tmp/s10-sdk-isolated-abc_123'
    assert str(paths('/home/golai/s10-sdk-isolated-abc', '/home/golai/ws/install')[0]).startswith('/home/golai/')
    for root, source in [('/tmp/other', '/home/team/install'),
                         ('/tmp/s10-sdk-isolated-../escape', '/home/team/install'),
                         ('/tmp/s10-sdk-isolated-abc', '/home/team/../other'),
                         ('/home/../s10-sdk-isolated-abc', '/home/team/install')]:
        try:
            paths(root, source)
        except ValueError:
            pass
        else:
            raise AssertionError('Unexpected remote path accepted')
    quoted = shlex.quote('/home/team/a b/install/setup.bash')
    assert shlex.split('source ' + quoted) == ['source', '/home/team/a b/install/setup.bash']
    print('SDK_LAUNCHER_CHECK_OK')


def run(args, cancel=None, alive=None, report=print, command_source=None):
    root, source_install = paths(args.trial_root, args.source_install)
    handset = args.mode in ('handset', 'handset-dry')
    manual = handset or args.mode in ('manual', 'manual-dry')
    if manual and command_source is None:
        raise ValueError('Manual mode requires a control source; open start_s10_him1500_handset.cmd')
    for value in (args.host, args.user, args.host_alias):
        if value and not re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.-]*', value):
            raise ValueError('Invalid SSH host, user, or host alias')
    options = ['-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes', '-o', 'ConnectTimeout=8',
               '-o', 'ServerAliveInterval=3', '-o', 'ServerAliveCountMax=2']
    if args.host_alias:
        options += ['-o', 'HostKeyAlias=' + args.host_alias]
    target = (args.user + '@' if args.user else '') + args.host
    process_options = {'creationflags': subprocess.CREATE_NO_WINDOW} if os.name == 'nt' else {}
    done = threading.Event()
    proc = None
    run_id = str(time.time_ns())
    remote_log = root / 'runs' / run_id
    local_log = Path(__file__).resolve().parents[1] / 'results/s10-sdk' / (args.host_alias or args.host) / run_id
    try:
        subprocess.run(['scp', *options, str(Path(__file__).with_name('s10_him_trial.py')),
                        target + ':' + str(root / 'supervisor.py')], check=True, **process_options)
        if cancel is not None and cancel.is_set():
            return
        mode = {'dry': '--him-dry', 'stand': '--him-stand', 'left': '--left', 'right': '--right',
                'right-full': '--right-full', 'manual': '--manual', 'manual-dry': '--manual-dry',
                'handset': '--handset', 'handset-dry': '--handset-dry'}[args.mode]
        command = ' && '.join([
            'set -o pipefail',
            'source /opt/ros/jazzy/setup.bash',
            'source ' + shlex.quote(str(source_install / 'setup.bash')),
            'source ' + shlex.quote(str(root / 'install/setup.bash')),
            'mkdir -p ' + shlex.quote(str(remote_log)),
            'unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE',
            'export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp',
            'export S10_TRIAL_OUTPUT_DIR=' + shlex.quote(str(remote_log)),
            'python3 -u ' + shlex.quote(str(root / 'supervisor.py')) + ' ' + shlex.quote(str(root))
            + ' ' + mode + ' 2>&1 | tee ' + shlex.quote(str(remote_log / 'supervisor.log'))])
        proc = subprocess.Popen(['ssh', *options, target, 'bash -c ' + shlex.quote(command)],
                                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, bufsize=1, **process_options)

        def heartbeat():
            try:
                while not done.is_set():
                    if (cancel is not None and cancel.is_set()) or (alive is not None and time.monotonic() - alive[0] > 1):
                        proc.stdin.write('stop\n')
                        proc.stdin.flush()
                        return
                    line = 'ping'
                    if manual:
                        packet = command_source()
                        if alive is not None and time.monotonic() - alive[0] > .3:
                            packet = {'action': 'zero'}
                        line = json.dumps(packet)
                        (handset_request if handset else manual_request)(line)
                    proc.stdin.write(line + '\n')
                    proc.stdin.flush()
                    done.wait(.1 if manual else .2)
            except (OSError, ValueError):
                pass
        threading.Thread(target=heartbeat, daemon=True).start()
        report('Running ' + args.mode + ' on ' + target + '; Ctrl-C / stop interrupts this run.')
        lines = deque(maxlen=200)
        for line in proc.stdout:
            lines.append(line)
            report(line.rstrip())
        error_output = ''.join(lines)
        proc.wait(timeout=5)
        done.set()
        if proc.returncode:
            raise RuntimeError(error_output or 'SSH job failed')
        suffix = 'dry' if args.mode in ('dry', 'manual-dry', 'handset-dry') else 'live'
        local_log.mkdir(parents=True, exist_ok=True)
        for name in ('supervisor.log', 'result-' + suffix + '.json', 'deploy-' + suffix + '.log', 'policy_trace.jsonl'):
            # Preflight can fail before a deployment log exists; always retrieve its result.
            subprocess.run(['scp', *options, target + ':' + str(remote_log / name), str(local_log / name)],
                           check=name.startswith(('supervisor', 'result-')), **process_options)
        result = json.loads((local_log / ('result-' + suffix + '.json')).read_text())
        report('Logs: ' + str(local_log))
        if not result['completed']:
            raise RuntimeError(result.get('error') or error_output or 'Trial incomplete')
        return result
    finally:
        done.set()
        if proc is not None:
            try:
                proc.stdin.close()  # EOF / heartbeat timeout tells the AGX supervisor to stop.
            except OSError:
                pass
            if proc.poll() is None:
                proc.terminate()
                proc.wait(timeout=5)


if __name__ == '__main__':
    if sys.argv[1:] == ['--check']:
        check()
        raise SystemExit(0)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='s10-48-golai', help='Verified OpenSSH config alias')
    parser.add_argument('--user', help='Defaults to the SSH config user')
    parser.add_argument('--host-alias', help='Optional known_hosts identity override')
    parser.add_argument('--source-install', default='/home/golai/s10_control_ws/install')
    parser.add_argument('--trial-root', required=True)
    parser.add_argument('--mode', choices=('dry', 'stand', 'left', 'right', 'right-full'), default='dry')
    try:
        run(parser.parse_args())
    except KeyboardInterrupt:
        raise SystemExit('Connection interrupted. Use the handset; do not assume the final posture.')
    except (RuntimeError, ValueError, subprocess.CalledProcessError) as exc:
        raise SystemExit(str(exc))
