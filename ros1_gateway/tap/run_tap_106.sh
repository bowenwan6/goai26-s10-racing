#!/usr/bin/env bash
# Start / stop / status of the read-only lidar tap on 106, as the normal `user`.
# Installs nothing; everything lives in the directory of this script.
#   bash ~/ros1_gateway_tap/run_tap_106.sh start [--gateway 10.21.33.102:47631]
#   bash ~/ros1_gateway_tap/run_tap_106.sh stop
#   bash ~/ros1_gateway_tap/run_tap_106.sh status
# nice 19 on CPU cores 0-3: the vendor drivers are pinned to cores 4-7.
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PIDF="$DIR/tap.pid"
mkdir -p "$DIR/logs"
case "${1:-status}" in
  start)
    shift
    if [ -f "$PIDF" ] && kill -0 "$(cat "$PIDF")" 2>/dev/null; then echo "tap already running (pid $(cat "$PIDF"))"; exit 0; fi
    LOG="$DIR/logs/tap-$(date +%Y%m%d-%H%M%S).log"
    ( source /opt/ros/jazzy/setup.bash
      export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTRTPS_DEFAULT_PROFILES_FILE=/opt/robot/fastdds.xml
      exec setsid nice -n 19 taskset -c 0-3 python3 -u "$DIR/s10_lidar_tap.py" "$@" ) > "$LOG" 2>&1 < /dev/null &
    echo $! > "$PIDF"
    sleep 2
    kill -0 "$(cat "$PIDF")" 2>/dev/null && echo "tap started (pid $(cat "$PIDF"), log $LOG)" || { echo "tap exited"; cat "$LOG"; exit 1; }
    ;;
  stop)
    [ -f "$PIDF" ] || { echo "tap not running"; exit 0; }
    PID="$(cat "$PIDF")"
    kill -INT "$PID" 2>/dev/null
    for _ in $(seq 1 20); do kill -0 "$PID" 2>/dev/null || break; sleep 0.5; done
    kill -0 "$PID" 2>/dev/null && kill -TERM "$PID"
    rm -f "$PIDF"; echo "tap stopped (pid $PID)"
    ;;
  status)
    if [ -f "$PIDF" ] && kill -0 "$(cat "$PIDF")" 2>/dev/null; then
      PID="$(cat "$PIDF")"; echo "tap running pid $PID $(ps -o %cpu=,rss= -p "$PID" | awk '{printf "cpu %s%% rss %.0f MiB", $1, $2/1024}')"
      tail -2 "$(ls -t "$DIR"/logs/tap-*.log | head -1)"
    else echo "tap not running"; fi
    ;;
  *) echo "usage: $0 start|stop|status" >&2; exit 2 ;;
esac
