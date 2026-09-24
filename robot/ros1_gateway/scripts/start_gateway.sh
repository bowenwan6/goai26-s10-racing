#!/usr/bin/env bash
# Start the ROS 1 master (if none is running on the port) and the gateway on the
# AGX, in the background, as the current user. Idempotent: refuses to start a
# second gateway.
#
#   bash ~/ros1_gateway/scripts/start_gateway.sh [--names keep|ros1] [--advertise IP_OR_HOST]
#                                                [--master-port 11311]
#   --names keep  ROS 1 topics keep the robot names /LIDAR/POINTS /IMU /ODOM (default)
#   --names ros1  ROS 1 topics /lidar_points /imu/data /odom
#   --advertise   address ROS 1 clients use to reach this AGX (ROS_IP/ROS_HOSTNAME).
#                 Default 10.21.33.102 (robot network). Use the Wi-Fi address when
#                 the ROS 1 client is on that network.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NAMES=keep
ADVERTISE="${S10_ROS1_ADVERTISE:-10.21.33.102}"
PORT="${S10_ROS1_MASTER_PORT:-11311}"
while [ $# -gt 0 ]; do
  case "$1" in
    --names) NAMES="$2"; shift 2 ;;
    --advertise) ADVERTISE="$2"; shift 2 ;;
    --master-port) PORT="$2"; shift 2 ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac
done
case "$NAMES" in
  keep) TOPICS="$ROOT/config/topics_keep_ros2_names.yaml" ;;
  ros1) TOPICS="$ROOT/config/topics.yaml" ;;
  *) echo "--names must be keep or ros1" >&2; exit 2 ;;
esac
RUN="$ROOT/run"; LOGS="$ROOT/logs"
mkdir -p "$RUN" "$LOGS"
if [ -f "$RUN/gateway.pid" ] && kill -0 "$(cat "$RUN/gateway.pid")" 2>/dev/null; then
  echo "gateway already running (pid $(cat "$RUN/gateway.pid"))"; exit 0
fi
STAMP="$(date +%Y%m%d-%H%M%S)"

if [[ "$ADVERTISE" =~ ^[0-9.]+$ ]]; then export ROS_IP="$ADVERTISE"; unset ROS_HOSTNAME
else export ROS_HOSTNAME="$ADVERTISE"; unset ROS_IP; fi
export ROS_MASTER_URI="http://127.0.0.1:$PORT"

if (exec 3<>"/dev/tcp/127.0.0.1/$PORT") 2>/dev/null; then
  echo "ROS 1 master already listening on port $PORT; using it"
else
  ( source "$ROOT/ros1/ros1_env.sh"
    export ROS_MASTER_URI="http://127.0.0.1:$PORT"
    exec setsid roscore -p "$PORT" ) > "$LOGS/roscore-$STAMP.log" 2>&1 < /dev/null &
  echo $! > "$RUN/roscore.pid"
  for _ in $(seq 1 30); do (exec 3<>"/dev/tcp/127.0.0.1/$PORT") 2>/dev/null && break; sleep 0.5; done
  echo "roscore started (pid $(cat "$RUN/roscore.pid"), log $LOGS/roscore-$STAMP.log)"
fi

( source "$ROOT/ros1/ros1_env.sh"
  source /opt/ros/jazzy/setup.bash
  source "$ROOT/ws/install/setup.bash"
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTRTPS_DEFAULT_PROFILES_FILE="${S10_FASTDDS_PROFILE:-$HOME/.ros/fastdds_ethernet.xml}"
  exec setsid "$ROOT/ws/install/s10_ros1_gateway/lib/s10_ros1_gateway/s10_ros1_gateway" \
    --config "$ROOT/config/gateway.yaml" --topics "$TOPICS" ) > "$LOGS/gateway-$STAMP.log" 2>&1 < /dev/null &
echo $! > "$RUN/gateway.pid"
ln -sfn "gateway-$STAMP.log" "$LOGS/gateway-latest.log"
echo "gateway started (pid $(cat "$RUN/gateway.pid"), topics $NAMES, advertise $ADVERTISE, log $LOGS/gateway-$STAMP.log)"
sleep 3
kill -0 "$(cat "$RUN/gateway.pid")" 2>/dev/null || { echo "gateway exited; see log"; tail -20 "$LOGS/gateway-$STAMP.log"; exit 1; }
