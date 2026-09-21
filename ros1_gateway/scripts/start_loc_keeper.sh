#!/usr/bin/env bash
# Start (or restart) the localisation keeper on the AGX: remembers the last pose, restores map + pose
# after x_nav restarts. See loc_keeper.py.   bash ~/ros1_gateway/scripts/start_loc_keeper.sh [--no-auto]
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN="$ROOT/run"; LOGS="$ROOT/logs"; mkdir -p "$RUN" "$LOGS"
AUTO="--auto-restore"; [ "$1" = "--no-auto" ] && AUTO=""
if [ -f "$RUN/loc_keeper.pid" ] && kill -0 "$(cat "$RUN/loc_keeper.pid")" 2>/dev/null; then
  kill -INT "$(cat "$RUN/loc_keeper.pid")"; sleep 1
fi
( source "$ROOT/ros1/ros1_env.sh"
  export ROS_MASTER_URI="http://127.0.0.1:${S10_ROS1_MASTER_PORT:-11311}" ROS_IP=127.0.0.1
  exec setsid python3 -u "$ROOT/scripts/loc_keeper.py" $AUTO
) > "$LOGS/loc_keeper-$(date +%Y%m%d-%H%M%S).log" 2>&1 < /dev/null &
echo $! > "$RUN/loc_keeper.pid"
sleep 2
kill -0 "$(cat "$RUN/loc_keeper.pid")" 2>/dev/null && echo "loc keeper running (pid $(cat "$RUN/loc_keeper.pid"))" || { echo "loc keeper exited"; tail -5 "$LOGS"/loc_keeper-*.log | tail -5; }
