#!/usr/bin/env bash
# Audit a converted ROS 1 bag against its ROS 2 source with the official decoders:
#   ROS 2 side: rosbag2_py + rclpy in the Jazzy builder image
#   ROS 1 side: rosbag info + rosbag/genpy in the official ros:noetic image
# Both containers run with --network none and read-only data mounts.
#
#   tools/audit_offline.sh <ros2_bag_dir> <converted.bag> <report_dir> [topics.yaml]
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_BAG="$(cd "$1" && pwd)"
ROS1_BAG="$(cd "$(dirname "$2")" && pwd)/$(basename "$2")"
OUT="$3"
TOPICS="${4:-$HERE/../config/topics.yaml}"
TOPICS="$(cd "$(dirname "$TOPICS")" && pwd)/$(basename "$TOPICS")"
JAZZY_IMAGE="${JAZZY_IMAGE:-s10-ros1-gateway-test}"
NOETIC_IMAGE="${NOETIC_IMAGE:-ros:noetic-ros-base@sha256:72b8bc59035dc0a5b8e07aae28c16caa84192971d72d207c72ed734fb1d5e97d}"
mkdir -p "$OUT"
OUT="$(cd "$OUT" && pwd)"
[ -e "$OUT/ros2.jsonl" ] || [ -e "$OUT/ros1.jsonl" ] && { echo "refusing to overwrite digests in $OUT" >&2; exit 1; }

echo "== ROS 2 digest (Jazzy, rosbag2_py)"
docker run --rm --network none -v "$SRC_BAG":/bag:ro -v "$HERE":/tools:ro -v "$OUT":/out "$JAZZY_IMAGE" \
  bash -c 'source /opt/ros/jazzy/setup.bash && ros2 bag info /bag > /out/ros2_bag_info.txt && python3 /tools/dump_ros2_digest.py bag /bag /out/ros2.jsonl'

echo "== ROS 1 rosbag info + digest (official Noetic image)"
docker run --rm --network none -v "$ROS1_BAG":/in/converted.bag:ro -v "$HERE":/tools:ro -v "$TOPICS":/in/topics.yaml:ro \
  -v "$OUT":/out "$NOETIC_IMAGE" \
  bash -c 'source /opt/ros/noetic/setup.bash && rosversion -d > /out/noetic_version.txt && rosbag info /in/converted.bag | tee /out/rosbag_info.txt && python3 /tools/dump_ros1_digest.py /in/converted.bag /out/ros1.jsonl --topics /in/topics.yaml'

echo "== compare"
python3 "$HERE/compare_digests.py" "$OUT/ros2.jsonl" "$OUT/ros1.jsonl" --mode offline --report "$OUT/offline_audit.json"
