"""Read-only startup gate for 48's existing 103 -> 106 PTP chain. Never sets time."""
import argparse
import os
import re
import subprocess
import time


def offsets(status, comparison):
    def value(name):
        match = re.search(r'^\s*'+name+r'\s+(\S+)', status, re.M)
        if not match:
            raise ValueError('Missing PTP field: '+name)
        return match[1]

    if value('portState') != 'SLAVE' or value('gmPresent') != 'true':
        raise ValueError('PTP is not following a grandmaster')
    if value('gmIdentity') != '3ede53.fffe.78f3dd':
        raise ValueError('PTP grandmaster is not the verified 48 board 103')
    match = re.search(r'offset from CLOCK_REALTIME is\s+([-+\d.]+)ns', comparison)
    if not match:
        raise ValueError('Cannot compare PHC with system clock')
    return int(value('master_offset')), float(match[1])


def wait_ready(timeout, max_offset_ms):
    deadline, stable, previous = time.monotonic()+timeout, 0, None
    reason = 'No PTP measurement'
    while time.monotonic() < deadline:
        try:
            def read(argv):
                return subprocess.run(argv, check=True, capture_output=True, text=True,
                    timeout=min(4, max(.1, deadline-time.monotonic())),
                    env=dict(os.environ, LC_ALL='C')).stdout

            status = read(['/usr/sbin/pmc', '-u', '-b', '0', '-s', '/var/run/ptp/ptp4l_eth0',
                           'GET TIME_STATUS_NP', 'GET PORT_DATA_SET'])
            master, system = offsets(status, read(['/usr/sbin/phc_ctl', 'eth0', 'cmp']))
            now = (time.monotonic(), time.time())
            jumped = previous and abs((now[1]-previous[1])-(now[0]-previous[0])) > max_offset_ms/1000
            previous = now
            if jumped or max(abs(master), abs(system)) > max_offset_ms*1e6:
                raise ValueError(f'Clock settling: master={master} ns, system={system} ns')
            stable += 1
            if stable >= 3:
                print(f'S10_PTP_READY master_offset_ns={master} system_offset_ns={system}', flush=True)
                return
        except (OSError, ValueError, subprocess.SubprocessError) as exc:
            stable, reason = 0, str(exc)
        time.sleep(min(.5, max(0, deadline-time.monotonic())))
    raise SystemExit('S10_PTP_NOT_READY: '+reason)


def check():
    status = 'master_offset -120\ngmPresent true\ngmIdentity 3ede53.fffe.78f3dd\nportState SLAVE'
    assert offsets(status, 'phc_ctl[1]: offset from CLOCK_REALTIME is -54ns') == (-120, -54.)
    for bad in (status.replace('SLAVE', 'LISTENING'), status.replace('true', 'false'),
                status.replace('3ede53', 'ffffff'), ''):
        try:
            offsets(bad, 'offset from CLOCK_REALTIME is 0ns')
        except ValueError:
            pass
        else:
            raise AssertionError('Unready PTP accepted')
    from types import SimpleNamespace
    from unittest.mock import patch
    with patch('subprocess.run', side_effect=lambda argv, **kw: SimpleNamespace(stdout=
               status if argv[0].endswith('pmc') else 'offset from CLOCK_REALTIME is 1ns')) as run, \
         patch('time.sleep'):
        wait_ready(1, 5)
        assert run.call_count == 6, 'Require three successful pairs of clock checks'
        try:
            wait_ready(.01, .0000001)
        except SystemExit as exc:
            assert 'S10_PTP_NOT_READY' in str(exc)
        else:
            raise AssertionError('Offset above tolerance accepted')
    print('PTP_GATE_CHECK_OK')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=60)
    parser.add_argument('--max-offset-ms', type=float, default=5)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    if args.check:
        check()
    elif not 0 < args.timeout <= 120 or not 0 < args.max_offset_ms <= 100:
        parser.error('timeout must be 0..120 s and offset tolerance 0..100 ms')
    else:
        wait_ready(args.timeout, args.max_offset_ms)
