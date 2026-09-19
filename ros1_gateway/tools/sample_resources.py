#!/usr/bin/env python3
"""Sample CPU, RSS and NIC traffic once per second from /proc (read-only, no root).

  sample_resources.py --pid 1234 --pid 5678 --iface end0 --seconds 600 --out res.csv
Prints a summary (mean/p95/max CPU %, max RSS, mean Mbit/s) at the end.
"""
import argparse
import os
import statistics
import time

TICK = os.sysconf('SC_CLK_TCK')


def proc_cpu_rss(pid):
    with open(f'/proc/{pid}/stat') as f:
        parts = f.read().rsplit(')', 1)[1].split()
    ticks = int(parts[11]) + int(parts[12])  # utime + stime
    with open(f'/proc/{pid}/status') as f:
        rss = next(int(l.split()[1]) for l in f if l.startswith('VmRSS:'))
    return ticks / TICK, rss / 1024.0


def iface_bytes(iface):
    with open('/proc/net/dev') as f:
        for line in f:
            if line.strip().startswith(iface + ':'):
                v = line.split(':', 1)[1].split()
                return int(v[0]), int(v[8])
    return 0, 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pid', type=int, action='append', default=[])
    ap.add_argument('--iface', default='end0')
    ap.add_argument('--seconds', type=float, default=60)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    prev = {p: proc_cpu_rss(p) for p in args.pid}
    prx, ptx = iface_bytes(args.iface)
    t_prev = time.monotonic()
    rows = []
    with open(args.out, 'x') as out:
        out.write('t,' + ','.join(f'cpu_{p},rss_mib_{p}' for p in args.pid) + ',rx_mbit_s,tx_mbit_s\n')
        end = t_prev + args.seconds
        while time.monotonic() < end:
            time.sleep(1.0)
            now = time.monotonic()
            dt = now - t_prev
            vals = []
            for p in args.pid:
                try:
                    cpu, rss = proc_cpu_rss(p)
                    vals.append((100.0 * (cpu - prev[p][0]) / dt, rss))
                    prev[p] = (cpu, rss)
                except FileNotFoundError:
                    vals.append((float('nan'), float('nan')))
            rx, tx = iface_bytes(args.iface)
            rxm, txm = (rx - prx) * 8 / dt / 1e6, (tx - ptx) * 8 / dt / 1e6
            prx, ptx, t_prev = rx, tx, now
            rows.append((vals, rxm, txm))
            out.write(f'{now:.1f},' + ','.join(f'{c:.2f},{r:.1f}' for c, r in vals) + f',{rxm:.2f},{txm:.2f}\n')
            out.flush()
    for i, p in enumerate(args.pid):
        cpus = sorted(r[0][i][0] for r in rows if r[0][i][0] == r[0][i][0])
        rss = [r[0][i][1] for r in rows if r[0][i][1] == r[0][i][1]]
        if cpus:
            print(f'pid {p}: cpu mean {statistics.mean(cpus):.1f}% p95 {cpus[int(0.95 * (len(cpus) - 1))]:.1f}% '
                  f'max {cpus[-1]:.1f}% (100% = one core), rss max {max(rss):.0f} MiB, samples {len(cpus)}')
    if rows:
        print(f'{args.iface}: rx mean {statistics.mean(r[1] for r in rows):.1f} Mbit/s, '
              f'tx mean {statistics.mean(r[2] for r in rows):.1f} Mbit/s')


if __name__ == '__main__':
    main()
