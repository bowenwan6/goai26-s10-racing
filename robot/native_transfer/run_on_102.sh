#!/usr/bin/env bash
set -eo pipefail
native_code_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$native_code_dir/../native_ws/install/setup.bash"
set -u
export PYTHONPATH="$native_code_dir:$native_code_dir/src/s10_auto_nav${PYTHONPATH:+:$PYTHONPATH}"
export ROS_DOMAIN_ID=0
cd "$native_code_dir"
exec python3 -m native_transfer.runtime "$@"
