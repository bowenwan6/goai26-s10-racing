#!/usr/bin/env bash
#
# Source the workspace and run one segment on the production stack.
#
#   docker/run.sh scripts/run_segment.sh --start 23 --end 24 --seeds 10 --out /res/fullstack
#
# The Python driver is the interesting half; this exists because `ros2 run` and `ros2 launch`
# need the overlay on the environment, and `docker/run.sh` hands its command straight to the
# container without one.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"
export S10_USE_VIEWER="${S10_USE_VIEWER:-0}"

S10_INSTALL_BASE="${S10_INSTALL_BASE:-${REPO_ROOT}/install}"
if [[ ! -f "${S10_INSTALL_BASE}/setup.bash" ]]; then
  echo "error: workspace not built. Run scripts/build.sh first." >&2
  exit 1
fi

# ROS setup scripts read variables that may be unset, so -u is lifted across the source.
set +u
# shellcheck disable=SC1091
source "${S10_INSTALL_BASE}/setup.bash"
set -u

exec python3 -u "${REPO_ROOT}/robot/scripts/run_segment.py" "$@"
