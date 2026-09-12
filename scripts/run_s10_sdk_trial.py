"""Windows SSH launcher; defaults to dry mode. Live modes actuate the selected robot."""
import argparse
import getpass
import json
from pathlib import Path, PurePosixPath
import re
import shlex
import sys
import threading
import time


def paths(root, source_install):
    if not re.fullmatch(r'/tmp/s10-sdk-isolated-[A-Za-z0-9_-]+', root):
        raise ValueError('Use the isolated root printed by prepare_s10_sdk_copy.py')
    if not source_install.startswith('/') or '..' in PurePosixPath(source_install).parts:
        raise ValueError('source-install must be a verified absolute AGX path')
    return PurePosixPath(root), PurePosixPath(source_install)


def check():
    assert str(paths('/tmp/s10-sdk-isolated-abc_123', '/home/team/ws/install')[0]) == '/tmp/s10-sdk-isolated-abc_123'
    for root, source in [('/tmp/other', '/home/team/install'),
                         ('/tmp/s10-sdk-isolated-../escape', '/home/team/install'),
                         ('/tmp/s10-sdk-isolated-abc', '/home/team/../other')]:
        try:
            paths(root, source)
        except ValueError:
            pass
        else:
            raise AssertionError('Unexpected remote path accepted')
    quoted = shlex.quote('/home/team/a b/install/setup.bash')
    assert shlex.split('source ' + quoted) == ['source', '/home/team/a b/install/setup.bash']
    print('SDK_LAUNCHER_CHECK_OK')


def run(args):
    import paramiko
    root, source_install = paths(args.trial_root, args.source_install)
    if not re.fullmatch(r'[A-Za-z0-9_.-]+', args.host_alias):
        raise ValueError('Invalid host alias')
    known = paramiko.HostKeys(str(Path.home() / '.ssh/known_hosts'))
    trusted = known.lookup(args.host_alias)
    if not trusted:
        raise ValueError('Verify this robot first using ssh -o HostKeyAlias=' + args.host_alias)
    client = paramiko.SSHClient()
    for kind, key in trusted.items():
        client.get_host_keys().add(args.host, kind, key)
    done = threading.Event()
    run_id = str(time.time_ns())
    remote_log = root / 'runs' / run_id
    local_log = Path(__file__).resolve().parents[1] / 'artifacts/s10-sdk' / args.host_alias / run_id
    try:
        client.connect(args.host, username=args.user, password=getpass.getpass('AGX SSH password: '),
                       timeout=8, auth_timeout=8, banner_timeout=8, allow_agent=False, look_for_keys=False)
        sftp = client.open_sftp()
        sftp.put(str(Path(__file__).with_name('s10_sdk_trial.py')), str(root / 'supervisor.py'))
        sftp.close()
        mode = {'dry': '--dry', 'stand': '--run', 'left': '--left', 'right': '--right', 'right-full': '--right-full'}[args.mode]
        command = ' && '.join([
            'source /opt/ros/jazzy/setup.bash',
            'source ' + shlex.quote(str(source_install / 'setup.bash')),
            'source ' + shlex.quote(str(root / 'install/setup.bash')),
            'mkdir -p ' + shlex.quote(str(remote_log)),
            'unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE',
            'export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp',
            'export S10_TRIAL_OUTPUT_DIR=' + shlex.quote(str(remote_log)),
            'python3 ' + shlex.quote(str(root / 'supervisor.py')) + ' ' + shlex.quote(str(root))
            + ' ' + mode + ' > ' + shlex.quote(str(remote_log / 'supervisor.log')) + ' 2>&1'])
        stdin, stdout, stderr = client.exec_command('bash -c ' + shlex.quote(command), timeout=65)

        def heartbeat():
            try:
                while not done.is_set():
                    stdin.write('ping\n')
                    stdin.flush()
                    done.wait(.2)
            except (OSError, EOFError, paramiko.SSHException):
                pass
        threading.Thread(target=heartbeat, daemon=True).start()
        print('Running', args.mode, 'on', args.host_alias, '; Ctrl-C interrupts this run.')
        stdout.read()
        error_output = stderr.read().decode()
        done.set()
        suffix = 'dry' if args.mode == 'dry' else 'live'
        local_log.mkdir(parents=True, exist_ok=True)
        sftp = client.open_sftp()
        for name in ('supervisor.log', 'result-' + suffix + '.json', 'deploy-' + suffix + '.log'):
            sftp.get(str(remote_log / name), str(local_log / name))
        sftp.close()
        result = json.loads((local_log / ('result-' + suffix + '.json')).read_text())
        print((local_log / 'supervisor.log').read_text(encoding='utf-8'))
        print('Logs:', local_log)
        if stdout.channel.recv_exit_status() or not result['completed']:
            raise RuntimeError(result.get('error') or error_output or 'Trial incomplete')
    finally:
        done.set()
        client.close()  # EOF / heartbeat timeout tells the AGX supervisor to stop.


if __name__ == '__main__':
    if sys.argv[1:] == ['--check']:
        check()
        raise SystemExit(0)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', required=True)
    parser.add_argument('--user', required=True)
    parser.add_argument('--host-alias', required=True, help='Previously verified known_hosts identity')
    parser.add_argument('--source-install', required=True)
    parser.add_argument('--trial-root', required=True)
    parser.add_argument('--mode', choices=('dry', 'stand', 'left', 'right', 'right-full'), default='dry')
    try:
        run(parser.parse_args())
    except KeyboardInterrupt:
        raise SystemExit('Connection interrupted. Use the handset; do not assume the final posture.')
