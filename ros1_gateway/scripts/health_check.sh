#!/usr/bin/env bash
# Read-only health check of the gateway on the AGX.
#   bash ~/ros1_gateway/scripts/health_check.sh          # status summary
#   bash ~/ros1_gateway/scripts/health_check.sh --hz 10  # plus ROS 1 rostopic hz for 10 s
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN="$ROOT/run"
rc=0
for name in roscore gateway; do
  if [ -f "$RUN/$name.pid" ] && kill -0 "$(cat "$RUN/$name.pid")" 2>/dev/null; then
    pid="$(cat "$RUN/$name.pid")"
    echo "$name: running pid $pid  $(ps -o %cpu=,rss= -p "$pid" | awk '{printf "cpu %s%%  rss %.0f MiB", $1, $2/1024}')"
  else
    echo "$name: NOT running"; [ "$name" = gateway ] && rc=1
  fi
done
if [ -f "$RUN/status.json" ]; then
  python3 - "$RUN/status.json" <<'PY' || rc=1
import json, sys, time
import os
OPTIONAL = set(filter(None, os.environ.get('S10_OPTIONAL_TOPICS', '/ODOM').split(',')))
s = json.load(open(sys.argv[1]))
age = (time.time_ns() - s['wall_ns']) / 1e9
print(f"status age {age:.1f}s, uptime {s['uptime_s']:.0f}s, master {s['ros_master_uri']}")
bad = age > 5
for r in s['routes']:
    lag = r['stamp_minus_local_clock_s']
    print(f"  {r['source']:3} {r['ros2_topic']:15} -> {r['ros1_topic']:15} {r['rate_hz'] or 0:7.2f} Hz  "
          f"published {r['published']:8}  errors {r['convert_errors']}  backwards {r['stamp_backwards']}  "
          f"stamp-localclock {lag['last'] if lag['last'] is not None else float('nan'):+.3f}s  "
          f"subs {r['ros1_subscribers']}  frame '{r['last_frame_id']}'")
    optional = r['ros2_topic'] in OPTIONAL
    if r['convert_errors'] or r['stamp_backwards']:
        bad = True
    elif r['published'] == 0 or (r['last_rx_age_s'] or 99) > 2:
        if optional:
            print(f"    note: {r['ros2_topic']} has no data (optional: 106 official localization is off; x_nav uses its own SLAM)")
        else:
            bad = True
t = s['tap']
if t.get('enabled'):
    print(f"  tap: {t['state']} peer {t['peer'] or '-'} frames {t['frames']} "
          f"errors {t['protocol_errors']} rejected {t['rejected_connections']} reported {t['tap_reported']}")
print('HEALTH_OK' if not bad else 'HEALTH_DEGRADED (no data, stale data, errors or stamps going backwards)')
sys.exit(1 if bad else 0)
PY
else
  echo "no status file yet"; rc=1
fi
if [ "$1" = "--hz" ]; then
  secs="${2:-10}"
  ( source "$ROOT/ros1/ros1_env.sh"; export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
    topics=$(rostopic list 2>/dev/null | grep -E '^/(LIDAR/POINTS|IMU|ODOM|lidar_points|imu/data|odom)$' | tr '\n' ' ')
    echo "rostopic hz $topics (${secs}s)"; timeout -s INT "$secs" rostopic hz $topics 2>&1 | tail -12 )
fi
exit $rc
