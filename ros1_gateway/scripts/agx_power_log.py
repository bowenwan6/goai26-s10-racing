#!/usr/bin/env python3
"""Black-box logger for the AGX's unexplained resets: input voltage/current of every INA3221
rail, temperatures, load and memory, twice a second, each line fsync'ed so the last samples
before a power loss survive. One CSV per boot in ~/ros1_gateway/logs/power/ (kept 30 files)."""
import glob
import os
import time

OUT = os.path.expanduser('~/ros1_gateway/logs/power')
os.makedirs(OUT, exist_ok=True)
for old in sorted(glob.glob(OUT + '/power-*.csv'))[:-30]:
    os.remove(old)
rails = []
for d in glob.glob('/sys/bus/i2c/drivers/ina3221/*/hwmon/hwmon*'):
    for i in (1, 2, 3):
        try:
            rails.append((open(f'{d}/in{i}_label').read().strip(), f'{d}/in{i}_input', f'{d}/curr{i}_input'))
        except OSError:
            pass
zones = [(open(z + '/type').read().strip(), z + '/temp') for z in sorted(glob.glob('/sys/class/thermal/thermal_zone*'))]


def rd(p):
    try:
        return int(open(p).read())
    except (OSError, ValueError):
        return -1


name = f"{OUT}/power-{time.strftime('%Y%m%d-%H%M%S')}-up{int(float(open('/proc/uptime').read().split()[0]))}s.csv"
with open(name, 'w') as f:
    f.write('wall,uptime_s,' + ','.join(f'{l}_mV,{l}_mA' for l, _, _ in rails) + ',' + ','.join(t for t, _ in zones) + ',load1,mem_avail_MB\n')
    while True:
        up = open('/proc/uptime').read().split()[0]
        mem = next(int(l.split()[1]) // 1024 for l in open('/proc/meminfo') if l.startswith('MemAvailable'))
        row = [f'{time.time():.2f}', up] + [str(v) for _, pv, pc in rails for v in (rd(pv), rd(pc))] + \
              [str(rd(p) // 1000) for _, p in zones] + [open('/proc/loadavg').read().split()[0], str(mem)]
        f.write(','.join(row) + '\n')
        f.flush()
        os.fsync(f.fileno())
        time.sleep(0.5)
