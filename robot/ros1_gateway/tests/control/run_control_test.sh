#!/usr/bin/env bash
# Isolated test of s10_ros1_control against mock_s10_robot.py (no robot, no network):
#   docker run --rm --network none -v "$PWD":/src:ro -v /tmp/out:/out s10-ros1-gateway-test \
#     bash /src/tests/control/run_control_test.sh enabled|dry_run
# REBUILD=1 first rebuilds s10_ros1_control from /src inside the container (the image's
# /ws/install may be older than the source).
set -eo pipefail  # no -u: ROS setup.bash reads unset variables
ROS1_SETUP="${ROS1_SETUP:-/opt/ros/one/setup.bash}"  # AGX: ~/ros1_gateway/ros1/ros1_env.sh
WS="${WS:-/ws/install}"                               # AGX: ~/ros1_gateway/ws/install
SCENARIO="${1:-enabled}"
SRC="${SRC:-/src}"; OUT="${OUT_BASE:-/out}/$SCENARIO"
mkdir -p "$OUT"; rm -f "$OUT"/*.jsonl
if [ "${REBUILD:-0}" = 1 ]; then
  ( source "$ROS1_SETUP" && source /opt/ros/jazzy/setup.bash && source "$WS/setup.bash" \
    && rm -rf /ws/src/s10_ros1_control && cp -r "$SRC/src/s10_ros1_control" /ws/src/ && cd /ws \
    && colcon build --packages-select s10_ros1_control --cmake-args -DCMAKE_BUILD_TYPE=Release ) > "$OUT/build.log" 2>&1 \
    || { tail -30 "$OUT/build.log"; echo BUILD_FAILED; exit 1; }
  echo "rebuilt s10_ros1_control ($(grep -c warning "$OUT/build.log" || true) warnings)"
fi
PORT="${ROS1_PORT:-11311}"   # AGX: use another port, the gateway owns 11311
export ROS_DOMAIN_ID="${TEST_DOMAIN:-78}" ROS_MASTER_URI="http://127.0.0.1:$PORT" ROS_IP=127.0.0.1
pids=()
cleanup() { for p in "${pids[@]}"; do kill -INT "$p" 2>/dev/null || true; done; wait 2>/dev/null || true; }
trap cleanup EXIT
( source "$ROS1_SETUP" && exec roscore -p "$PORT" ) > "$OUT/roscore.log" 2>&1 & pids+=($!)
sleep 4
( source /opt/ros/jazzy/setup.bash && source "$WS/setup.bash" \
  && exec python3 -u "$SRC/tests/control/mock_s10_robot.py" --log "$OUT/mock.jsonl" --silent-nav-publisher ) > "$OUT/mock.log" 2>&1 & pids+=($!)
FLAG=""; [ "$SCENARIO" = enabled ] && FLAG="--enable-motion"
( source "$ROS1_SETUP" && source /opt/ros/jazzy/setup.bash && source "$WS/setup.bash" \
  && exec $WS/s10_ros1_control/lib/s10_ros1_control/s10_ros1_control --config "$SRC/config/control.yaml" \
     --event-log "$OUT/events.jsonl" $FLAG ) > "$OUT/control.log" 2>&1 & pids+=($!)
sleep 5
touch "$OUT/mock.jsonl"
( source "$ROS1_SETUP" && exec python3 -u "$SRC/tests/control/drive_ros1.py" "$OUT/mock.jsonl" "$SCENARIO" ) \
  > "$OUT/driver.log" 2>&1 & drv=$!
# Start a competing /NAV_CMD publisher when the driver asks for it.
while kill -0 "$drv" 2>/dev/null; do
  if grep -q STARTING_SECOND_PUBLISHER "$OUT/driver.log" 2>/dev/null; then
    ( source /opt/ros/jazzy/setup.bash && source "$WS/setup.bash" \
      && exec ros2 topic pub -r 1 /NAV_CMD drdds/msg/NavCmd "{}" ) > "$OUT/second.log" 2>&1 & pids+=($!)
    break
  fi
  sleep 0.2
done
wait "$drv" || true
cat "$OUT/driver.log"
