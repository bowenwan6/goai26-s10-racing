#!/usr/bin/env bash
# Stop the ROS 1 navigation node started by start_nav.sh.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN="$ROOT/run"
if [ -f "$RUN/nav.pid" ]; then
  pid="$(cat "$RUN/nav.pid")"
  if kill -0 "$pid" 2>/dev/null; then
    kill -INT "$pid"; for i in 1 2 3 4 5 6; do kill -0 "$pid" 2>/dev/null || break; sleep 0.5; done
    kill -0 "$pid" 2>/dev/null && kill -KILL "$pid"
    echo "nav stopped (pid $pid)"
  else
    echo "nav not running"
  fi
  rm -f "$RUN/nav.pid"
else
  echo "nav not running"
fi
