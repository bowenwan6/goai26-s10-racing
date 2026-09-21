#!/usr/bin/env bash
# Start the ROS 1 motion interface (s10_ros1_control) on the AGX, in the background.
# Requires the ROS 1 master (start_gateway.sh starts one).
#
#   bash ~/ros1_gateway/scripts/start_control.sh                  # DRY RUN: reads feedback, sends nothing
#   bash ~/ros1_gateway/scripts/start_control.sh --enable-motion  # really commands the robot
#   bash ~/ros1_gateway/scripts/start_control.sh --enable-motion --stage zero|probe|flat
#
# --stage is for OUR navigation runs. It writes run/control-<stage>.yaml from config/control.yaml with
#   zero  : limits 0 / 0 / 0        (everything armed, robot cannot move: checks the whole chain)
#   probe : limits 0.10 / 0 / 0.30  (first motion)
#   flat  : limits 0.20 / 0.10 / 0.50
#   <v>   : a number (m/s, up to 1.67) or a multiplier of the robot maximum ("0.7x" = 1.17 m/s):
#           forward limit v, lateral min(0.40, v/2), yaw 1.0 rad/s
# and in all three: rl_nav is the ONLY velocity source and commands are read from the private
# topic /s10_control/web_cmd, so the x_nav web page (/web_cmd, /cmd_vel) cannot move the robot.
#
# Only use --enable-motion with an operator holding the remote, a clear area,
# and no other navigation/control program running (the node refuses to arm if
# another /NAV_CMD publisher exists).
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN="$ROOT/run"; LOGS="$ROOT/logs"
mkdir -p "$RUN" "$LOGS"
FLAG=""
STAGE=""
CONFIG="$ROOT/config/control.yaml"
ADVERTISE="${S10_ROS1_ADVERTISE:-10.21.33.102}"
PORT="${S10_ROS1_MASTER_PORT:-11311}"
while [ $# -gt 0 ]; do
  case "$1" in
    --enable-motion) FLAG="--enable-motion"; shift ;;
    --stage) STAGE="$2"; shift 2 ;;
    --config) CONFIG="$2"; shift 2 ;;
    --advertise) ADVERTISE="$2"; shift 2 ;;
    --master-port) PORT="$2"; shift 2 ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac
done
if [ -f "$RUN/control.pid" ] && kill -0 "$(cat "$RUN/control.pid")" 2>/dev/null; then
  echo "control already running (pid $(cat "$RUN/control.pid")); stop it first"; exit 1
fi
(exec 3<>"/dev/tcp/127.0.0.1/$PORT") 2>/dev/null || { echo "no ROS 1 master on port $PORT; run start_gateway.sh first"; exit 1; }
STAMP="$(date +%Y%m%d-%H%M%S)"
if [ -n "$STAGE" ]; then
  CONFIG="$(python3 - "$ROOT/config/control.yaml" "$RUN/control-$STAGE.yaml" "$STAGE" <<'PY'
import sys, yaml
src, dst, stage = sys.argv[1:4]
limits = {"zero": (0.0, 0.0, 0.0), "probe": (0.10, 0.0, 0.30), "flat": (0.20, 0.10, 0.50)}
if stage not in limits:
    try:
        v = float(stage[:-1]) * 1.67 if stage.endswith("x") else float(stage)   # "0.7x" = 0.7 of the robot maximum
    except ValueError:
        sys.exit("unknown --stage %r (zero|probe|flat|<m/s>|<k>x)" % stage)
    if not 0.0 < v <= 1.67:
        sys.exit("--stage speed must be in (0, 1.67] m/s (robot command range)")
    v = round(v, 3)
    limits[stage] = (v, min(0.40, round(v / 2, 2)), 1.0)
c = yaml.safe_load(open(src))
c["limits"] = dict(zip(("max_vx", "max_vy", "max_wz"), limits[stage]))
import os
if os.environ.get("S10_FLAT_GAIT"):                  # experiment: another gait in place of the navigation flat gait
    g = int(os.environ["S10_FLAT_GAIT"], 0)
    c.setdefault("gait_switch", {})["flat_gait"] = g
    c.setdefault("stand", {})["nav_gait"] = g
c["cmd_sources"] = {"rl_nav": c["cmd_sources"]["rl_nav"]}
c["cmd_source"] = "rl_nav"
c["web_cmd_topic"] = "/s10_control/web_cmd"
c["stand"]["nav_gait"] = hex(int(c["stand"]["nav_gait"]))
yaml.safe_dump(c, open(dst, "w"), sort_keys=False)
print(dst)
PY
)"
fi
if [[ "$ADVERTISE" =~ ^[0-9.]+$ ]]; then export ROS_IP="$ADVERTISE"; unset ROS_HOSTNAME
else export ROS_HOSTNAME="$ADVERTISE"; unset ROS_IP; fi
export ROS_MASTER_URI="http://127.0.0.1:$PORT"
( source "$ROOT/ros1/ros1_env.sh"
  source /opt/ros/jazzy/setup.bash
  source "$ROOT/ws/install/setup.bash"
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTRTPS_DEFAULT_PROFILES_FILE="${S10_FASTDDS_PROFILE:-$HOME/.ros/fastdds_ethernet.xml}"
  exec setsid "$ROOT/ws/install/s10_ros1_control/lib/s10_ros1_control/s10_ros1_control" \
    --config "$CONFIG" --event-log "$LOGS/control-events-$STAMP.jsonl" $FLAG
) > "$LOGS/control-$STAMP.log" 2>&1 < /dev/null &
echo $! > "$RUN/control.pid"
# ready = the node logged its start event, and with --enable-motion its "armed" event (was a fixed 4 s wait)
WANT="event start"; [ -n "$FLAG" ] && WANT="event armed"
for i in $(seq 1 40); do
  sleep 0.25
  kill -0 "$(cat "$RUN/control.pid")" 2>/dev/null || break
  grep -q "$WANT\|event fault" "$LOGS/control-$STAMP.log" 2>/dev/null && break
done
if ! kill -0 "$(cat "$RUN/control.pid")" 2>/dev/null; then
  echo "control exited; see $LOGS/control-$STAMP.log"; tail -20 "$LOGS/control-$STAMP.log"; exit 1
fi
echo "control started (pid $(cat "$RUN/control.pid"), ${FLAG:-DRY RUN}, config $CONFIG, log $LOGS/control-$STAMP.log)"
grep -E 'MOTION ENABLED|fault|dry_run' "$LOGS/control-$STAMP.log" | tail -3 || true
