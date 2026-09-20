#!/usr/bin/env bash
# Start / stop / status of the teach worker (采集助手后台) on the AGX, as golai.
#   bash ~/s10_mapping_web/teach-worker.sh start [--pose-topic /base_link/odom]
#   bash ~/s10_mapping_web/teach-worker.sh start --fake     # demo robot, no ROS
#   bash ~/s10_mapping_web/teach-worker.sh stop | status
# Needs the ROS 1 master from ~/ros1_gateway/scripts/start_gateway.sh (not for --fake).
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PIDF="$HOME/teach/teach-worker.pid"
ROS1_ENV="${S10_ROS1_ENV:-$HOME/ros1_gateway/ros1/ros1_env.sh}"
mkdir -p "$HOME/teach/logs"
case "${1:-status}" in
  start)
    shift
    if [ -f "$PIDF" ] && kill -0 "$(cat "$PIDF")" 2>/dev/null; then echo "teach worker already running (pid $(cat "$PIDF"))"; exit 0; fi
    LOG="$HOME/teach/logs/teach-worker-$(date +%Y%m%d-%H%M%S).log"
    (
      [ -f "$ROS1_ENV" ] && source "$ROS1_ENV"
      export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}" ROS_IP="${ROS_IP:-127.0.0.1}"
      cd "$DIR"
      # Data goes straight to the external SSD when it is mounted (fstab: /mnt/s10ssd, exFAT,
      # automount); otherwise to the AGX's own storage, and the page says so in red.
      SSD="${S10_TEACH_SSD:-/mnt/s10ssd}"
      case " $* " in *" --data-dir "*) ;; *)
        # "mountpoint -q" also answers yes for the bare autofs point, so the mkdir/touch is the
        # real test; touching the path is also what triggers the automount. Retry, because after
        # a reboot USB enumeration can lag behind s10-stack.service.
        for _ in $(seq "${S10_TEACH_SSD_WAIT:-10}"); do
          mkdir -p "$SSD/s10_teach" 2>/dev/null && touch "$SSD/s10_teach/.write-test" 2>/dev/null && break
          sleep 1
        done
        if rm -f "$SSD/s10_teach/.write-test" 2>/dev/null && [ -d "$SSD/s10_teach" ]; then
          set -- --data-dir "$SSD/s10_teach" "$@"
        else
          echo "WARNING: external SSD $SSD is not usable - recording to the AGX eMMC (~/teach), only ~20 GB free"
        fi ;;
      esac
      echo "teach data dir args: $*"
      # exit code 3 = the ROS master restarted: restart the worker so it re-registers
      while true; do python3 -u teach_worker.py "$@"; code=$?; [ "$code" = 3 ] || exit "$code"; sleep 1; done
    ) > "$LOG" 2>&1 < /dev/null &
    echo $! > "$PIDF"
    sleep 2
    if kill -0 "$(cat "$PIDF")" 2>/dev/null; then echo "teach worker started (pid $(cat "$PIDF"), log $LOG)"; else echo "teach worker exited:"; cat "$LOG"; exit 1; fi
    ;;
  stop)
    [ -f "$PIDF" ] || { echo "teach worker not running"; exit 0; }
    PID="$(cat "$PIDF")"
    pkill -TERM -P "$PID" 2>/dev/null   # the python worker stops an active recording cleanly
    for _ in $(seq 1 60); do pgrep -P "$PID" >/dev/null || break; sleep 1; done
    kill "$PID" 2>/dev/null; rm -f "$PIDF"; echo "teach worker stopped"
    ;;
  status)
    if [ -f "$PIDF" ] && kill -0 "$(cat "$PIDF")" 2>/dev/null; then echo "teach worker running (pid $(cat "$PIDF"))"
      curl -s --max-time 3 http://127.0.0.1:8091/status | python3 -c 'import json,sys; s=json.load(sys.stdin); print("ros", s["ros"], "| pose", s["topics"]["pose"], "| lidar", s["topics"]["lidar"], "| imu", s["topics"]["imu"], "| recording", s["recording"], "| data", s["config"]["data_dir"])' 2>/dev/null
    else echo "teach worker not running"; fi
    ;;
  migrate)   # move any sessions left on the AGX eMMC onto the SSD (stop the worker first)
    SSD="${S10_TEACH_SSD:-/mnt/s10ssd}"; SRC="$HOME/teach/sessions"; DST="$SSD/s10_teach/sessions"
    mkdir -p "$DST" 2>/dev/null || { echo "SSD $SSD not mounted"; exit 1; }
    [ -d "$SRC" ] || { echo "nothing to migrate"; exit 0; }
    for S in "$SRC"/*/; do
      [ -d "$S" ] || continue
      N="$(basename "$S")"
      [ -e "$DST/$N" ] && { echo "skip $N (already on the SSD)"; continue; }
      echo "moving $N ..."
      rsync -a --info=progress2 "$S" "$DST/$N/" && rm -rf "$S" && echo "  moved $N"
    done
    du -sh "$DST" 2>/dev/null
    ;;
  *) echo "usage: $0 start|stop|status|migrate" >&2; exit 2 ;;
esac
