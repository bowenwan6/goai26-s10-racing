#!/usr/bin/env bash
# One command for a whole navigation run (on the AGX). See nav_run.py for the steps.
#   bash ~/ros1_gateway/scripts/nav_run.sh [--speed 0.5] [--route short|full|<dir>] [--at start|end] [--shadow]
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/ros1/ros1_env.sh"
export ROS_MASTER_URI="http://127.0.0.1:${S10_ROS1_MASTER_PORT:-11311}" ROS_IP=127.0.0.1
exec python3 -u "$ROOT/scripts/nav_run.py" "$@"
