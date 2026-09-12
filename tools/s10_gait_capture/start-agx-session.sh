#!/usr/bin/env bash
set -eo pipefail
ROOT=/home/ysc/s10_capture_session
cd "$ROOT"
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONPATH="$ROOT/webdeps:$ROOT/vendor/drdds/lib/python3.12/site-packages:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$ROOT/vendor/drdds/lib:${LD_LIBRARY_PATH:-}"
export AMENT_PREFIX_PATH="$ROOT/vendor/drdds:${AMENT_PREFIX_PATH:-}"
export FASTRTPS_DEFAULT_PROFILES_FILE="$ROOT/fastdds.xml"
exec python3 -u server.py --host 0.0.0.0 --port 8091 --output "$ROOT/data"
