#!/usr/bin/env bash
# Start the ROS 1 navigation node (our route runner) on the AGX, in the background.
#   bash ~/ros1_gateway/scripts/start_nav.sh --route-dir ~/routes/<name> [--shadow] [--autostart]
# --shadow: status only, never publishes /rl_nav/cmd_vel (drive with the remote, watch progress).
# Without --autostart it waits for  rostopic pub -1 /rl_nav/cmd std_msgs/String "data: start".
# Needs the ROS 1 master (start_gateway.sh) and, for motion, s10_ros1_control with
# cmd_source rl_nav and --enable-motion.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN="$ROOT/run"; LOGS="$ROOT/logs"
mkdir -p "$RUN" "$LOGS"
# all parameters live in config/s10_params.yaml; the file read below is regenerated from it first (tools/params.py)
python3 "$ROOT/tools/params.py" sync --quiet || { echo "params sync failed: config/s10_params.yaml is broken"; exit 1; }
PORT="${S10_ROS1_MASTER_PORT:-11311}"
ARGS=()
CONFIG="$ROOT/config/nav.yaml"
while [ $# -gt 0 ]; do
  case "$1" in
    --route-dir) ARGS+=(--route-dir "$2"); shift 2 ;;
    --config) CONFIG="$2"; shift 2 ;;
    --transform) ARGS+=(--transform "$2"); shift 2 ;;
    --shadow|--autostart) ARGS+=("$1"); shift ;;
    --master-port) PORT="$2"; shift 2 ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac
done
if [ -f "$RUN/nav.pid" ] && kill -0 "$(cat "$RUN/nav.pid")" 2>/dev/null; then
  echo "nav already running (pid $(cat "$RUN/nav.pid")); stop it first (stop_nav.sh)"; exit 1
fi
(exec 3<>"/dev/tcp/127.0.0.1/$PORT") 2>/dev/null || { echo "no ROS 1 master on port $PORT; run start_gateway.sh first"; exit 1; }
STAMP="$(date +%Y%m%d-%H%M%S)"
export ROS_MASTER_URI="http://127.0.0.1:$PORT" ROS_IP=127.0.0.1
( source "$ROOT/ros1/ros1_env.sh"
  exec setsid python3 -u "$ROOT/nav/s10_rl_nav_ros1.py" --config "$CONFIG" "${ARGS[@]}" \
) > "$LOGS/nav-$STAMP.log" 2>&1 < /dev/null &
echo $! > "$RUN/nav.pid"
# ready = the node printed its route summary (was a fixed 3 s wait)
for i in $(seq 1 60); do
  sleep 0.25
  kill -0 "$(cat "$RUN/nav.pid")" 2>/dev/null || break
  grep -q "rl_nav ros1:" "$LOGS/nav-$STAMP.log" 2>/dev/null && break
done
if kill -0 "$(cat "$RUN/nav.pid")" 2>/dev/null; then
  echo "nav started (pid $(cat "$RUN/nav.pid"), log $LOGS/nav-$STAMP.log)"; tail -5 "$LOGS/nav-$STAMP.log"
else
  echo "nav exited:"; tail -20 "$LOGS/nav-$STAMP.log"; rm -f "$RUN/nav.pid"; exit 1
fi
