#!/usr/bin/env bash
# Stop s10_ros1_control. On SIGINT it sends zero velocity before exiting.
#   bash ~/ros1_gateway/scripts/stop_control.sh
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FILE="$ROOT/run/control.pid"
[ -f "$FILE" ] || { echo "control: not running (no pid file)"; exit 0; }
PID="$(cat "$FILE")"
if kill -0 "$PID" 2>/dev/null; then
  kill -INT "$PID"
  for _ in $(seq 1 20); do kill -0 "$PID" 2>/dev/null || break; sleep 0.5; done
  kill -0 "$PID" 2>/dev/null && { kill -TERM "$PID"; sleep 2; }
  kill -0 "$PID" 2>/dev/null && kill -KILL "$PID"
  echo "control: stopped (pid $PID)"
else
  echo "control: pid $PID was not running"
fi
rm -f "$FILE"
