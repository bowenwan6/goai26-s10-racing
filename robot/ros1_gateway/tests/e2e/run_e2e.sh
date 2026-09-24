#!/usr/bin/env bash
# End-to-end test inside the builder image, isolated from any network:
#   docker run --rm --network none -v "$PWD":/src:ro -v /tmp/out:/out s10-ros1-gateway-test \
#     bash /src/tests/e2e/run_e2e.sh
# synthetic ROS 2 publisher -> (lidar: tap over TCP 127.0.0.1) / (IMU, ODOM: DDS)
#   -> s10_ros1_gateway -> ROS 1 rosbag record -> verify_ros1_bag.py
set -eo pipefail  # no -u: ROS setup.bash reads unset variables
ROS1_SETUP="${ROS1_SETUP:-/opt/ros/one/setup.bash}"  # AGX: ~/ros1_gateway/ros1/ros1_env.sh
WS="${WS:-/ws/install}"                               # AGX: ~/ros1_gateway/ws/install
SRC=${SRC:-/src}
OUT=${OUT:-/out}
SECONDS_RUN=${SECONDS_RUN:-20}
mkdir -p "$OUT/run"
export ROS_DOMAIN_ID=77 ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTRTPS_DEFAULT_PROFILES_FILE="$SRC/tests/e2e/fastdds_shm_like_106.xml"
case "$OUT" in /*) ;; *) OUT="$PWD/$OUT" ;; esac
pids=()
cleanup() { for p in "${pids[@]}"; do kill -INT "$p" 2>/dev/null || true; done; wait 2>/dev/null || true; }
trap cleanup EXIT

cat > "$OUT/gateway.test.yaml" <<EOF
status_topic: /s10_ros1_gateway/status
status_file: $OUT/run/status.json
status_period_s: 1.0
tap: {enabled: true, listen_address: 127.0.0.1, listen_port: 47631, allowed_peers: [127.0.0.1], idle_timeout_s: 5.0}
EOF

( source "$ROS1_SETUP" && exec roscore -p 11311 ) > "$OUT/roscore.log" 2>&1 & pids+=($!)
sleep 4
( source "$ROS1_SETUP" && source /opt/ros/jazzy/setup.bash && source "$WS/setup.bash" \
  && exec $WS/s10_ros1_gateway/lib/s10_ros1_gateway/s10_ros1_gateway \
       --config "$OUT/gateway.test.yaml" --topics "$SRC/config/topics.yaml" ) > "$OUT/gateway.log" 2>&1 & pids+=($!)
sleep 3
( source /opt/ros/jazzy/setup.bash && exec python3 -u "$SRC/tap/s10_lidar_tap.py" --gateway 127.0.0.1:47631 ) \
  > "$OUT/tap.log" 2>&1 & pids+=($!)
( source "$ROS1_SETUP" && exec rosbag record -O "$OUT/e2e.bag" /lidar_points /imu/data /odom \
  /s10_ros1_gateway/status ) > "$OUT/rosbag.log" 2>&1 & rec=$!; pids+=($rec)
sleep 3
( source /opt/ros/jazzy/setup.bash && python3 "$SRC/tests/e2e/synthetic_ros2_pub.py" --seconds "$SECONDS_RUN" \
  --log "$OUT/sent.jsonl" ) > "$OUT/publisher.log" 2>&1
sleep 2
kill -INT "$rec"; wait "$rec" 2>/dev/null || true
cp "$OUT/run/status.json" "$OUT/status.final.json" 2>/dev/null || true
( source "$ROS1_SETUP" && rosbag info "$OUT/e2e.bag" && python3 "$SRC/tests/e2e/verify_ros1_bag.py" \
  "$OUT/e2e.bag" "$OUT/sent.jsonl" ) | tee "$OUT/verify.log"
