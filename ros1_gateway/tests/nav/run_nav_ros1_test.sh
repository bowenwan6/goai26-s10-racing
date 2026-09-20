#!/usr/bin/env bash
# Isolated ROS 1 test of nav/s10_rl_nav_ros1.py against fake_xnav_ros1.py (no robot):
#   docker run --rm --network none -v "$PWD":/src:ro -v /tmp/out:/out s10-ros1-nav-test \
#     bash /src/tests/nav/run_nav_ros1_test.sh
# On the AGX: ROS1_SETUP=~/ros1_gateway/ros1/ros1_env.sh OUT_BASE=/tmp/navtest bash tests/nav/run_nav_ros1_test.sh
# A straight 3 m route is made with tools/teach_to_route.py --straight; the node must drive the
# fake robot to WP02 (progress 1.0, finished) with a forward speed <= 0.30 m/s.
set -eo pipefail
ROS1_SETUP="${ROS1_SETUP:-/opt/ros/one/setup.bash}"
SRC="${SRC:-/src}"; OUT="${OUT_BASE:-/out}/nav"
mkdir -p "$OUT"; rm -f "$OUT"/*.jsonl "$OUT"/*.log
export ROS_MASTER_URI=http://127.0.0.1:11312 ROS_IP=127.0.0.1
pids=()
cleanup() { for p in "${pids[@]}"; do kill -INT "$p" 2>/dev/null || true; done; wait 2>/dev/null || true; }
trap cleanup EXIT
python3 "$SRC/tools/teach_to_route.py" --straight 1.0 2.0 0.27 0.5 3.0 --map-id test --out "$OUT/route" > "$OUT/route.log" 2>&1 \
  || { cat "$OUT/route.log"; echo ROUTE_FAILED; exit 1; }
( source "$ROS1_SETUP" && exec roscore -p 11312 ) > "$OUT/roscore.log" 2>&1 & pids+=($!)
sleep 4
( source "$ROS1_SETUP" && exec python3 -u "$SRC/tests/nav/fake_xnav_ros1.py" --log "$OUT/fake.jsonl" --x 1.0 --y 2.0 --yaw 0.5 ) > "$OUT/fake.log" 2>&1 & pids+=($!)
sleep 2
( source "$ROS1_SETUP" && exec python3 -u "$SRC/nav/s10_rl_nav_ros1.py" --config "$SRC/config/nav.yaml" --route-dir "$OUT/route" ) > "$OUT/nav.log" 2>&1 & pids+=($!)
sleep 6
( source "$ROS1_SETUP"
  rostopic echo -n 1 /rl_nav/status > "$OUT/status_paused.txt" 2>&1 || true
  rostopic pub -1 /rl_nav/cmd std_msgs/String "data: start" > /dev/null 2>&1
  end=$((SECONDS + 60)); fin=""
  while [ $SECONDS -lt $end ]; do
    fin=$(timeout 3 rostopic echo -n 1 /nav/finished 2>/dev/null | grep -o 'True' || true)
    [ -n "$fin" ] && break
    sleep 1
  done
  rostopic echo -n 1 /rl_nav/status > "$OUT/status_end.txt" 2>&1 || true
  echo "finished=$fin" )
python3 - "$OUT" <<'PY'
import json, sys, glob
out = sys.argv[1]
rows = [json.loads(l) for l in open(out + "/fake.jsonl") if l.strip()]
poses = [r for r in rows if "x" in r]
last = poses[-1]
paused = open(out + "/status_paused.txt").read()
end = open(out + "/status_end.txt").read()
fails = []
if "paused" not in paused: fails.append("node did not wait for start")
if last["n_cmd"] < 50: fails.append(f"few commands {last['n_cmd']}")
if last["max_v"] > 0.30 + 1e-6: fails.append(f"forward {last['max_v']} > 0.30")
import math
d = math.hypot(last["x"] - (1.0 + 3.0 * math.cos(0.5)), last["y"] - (2.0 + 3.0 * math.sin(0.5)))
if d > 0.35: fails.append(f"ended {d:.2f} m from WP02")
if '"reached": 2' not in end and 'DONE' not in end: fails.append("status does not show DONE / reached 2")
print(f"end pose ({last['x']:.2f},{last['y']:.2f}) {d:.2f} m from WP02; cmds {last['n_cmd']}; max v {last['max_v']}")
print("NAV_ROS1_TEST_OK" if not fails else "NAV_ROS1_TEST_FAILED " + "; ".join(fails))
sys.exit(1 if fails else 0)
PY
