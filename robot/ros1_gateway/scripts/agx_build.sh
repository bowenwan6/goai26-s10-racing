#!/usr/bin/env bash
# Build ros1_bridge + s10_ros1_gateway natively on the AGX against the AGX's own
# ROS 2 Jazzy and the user-space ROS 1 bundle. No sudo, no system changes.
#   bash ~/ros1_gateway/scripts/agx_build.sh
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
JOBS="${JOBS:-4}"
source "$ROOT/ros1/ros1_env.sh"
source /opt/ros/jazzy/setup.bash
cd "$ROOT/ws"
# ros1_bridge generates converters for every message package present in both
# ROS 1 and ROS 2. Only /opt/ros/jazzy is sourced on the ROS 2 side.
MAKEFLAGS="-j$JOBS" nice -n 10 colcon build --base-paths "$ROOT/src" \
  --packages-select ros1_bridge s10_ros1_gateway drdds s10_ros1_control \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  --event-handlers console_cohesion+
source "$ROOT/ws/install/setup.bash"
"$ROOT/ws/install/s10_ros1_gateway/lib/s10_ros1_gateway/s10_ros1_gateway" \
  --config "$ROOT/config/gateway.yaml" --topics "$ROOT/config/topics_keep_ros2_names.yaml" --check
