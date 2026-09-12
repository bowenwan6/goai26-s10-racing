#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
source /opt/ros/jazzy/setup.bash
if [[ -n "${S10_GAIT_ROS_OVERLAY:-}" ]]; then
    source "$S10_GAIT_ROS_OVERLAY"
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
exec .venv/bin/python -u server.py --host 127.0.0.1 --port 8090 --output "$HOME/s10_gait_data" --auth-config "$HOME/.config/s10-mapping-web/config.json"
