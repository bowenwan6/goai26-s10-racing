#!/usr/bin/env bash
# Stop the gateway and the roscore that start_gateway.sh launched (a master
# that was already running before is left alone).
#   bash ~/ros1_gateway/scripts/stop_gateway.sh
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN="$ROOT/run"
stop_pid() {
  local name="$1" file="$RUN/$1.pid"
  [ -f "$file" ] || { echo "$name: not running (no pid file)"; return; }
  local pid; pid="$(cat "$file")"
  if kill -0 "$pid" 2>/dev/null; then
    kill -INT "$pid"
    for _ in $(seq 1 20); do kill -0 "$pid" 2>/dev/null || break; sleep 0.5; done
    kill -0 "$pid" 2>/dev/null && { kill -TERM "$pid"; sleep 2; }
    kill -0 "$pid" 2>/dev/null && { echo "$name: pid $pid still alive, sending SIGKILL"; kill -KILL "$pid"; }
    echo "$name: stopped (pid $pid)"
  else
    echo "$name: pid $pid was not running"
  fi
  rm -f "$file"
}
stop_pid gateway
stop_pid roscore
rm -f "$RUN/status.json"
