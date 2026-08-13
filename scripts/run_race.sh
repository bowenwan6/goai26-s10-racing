#!/usr/bin/env bash
#
# Run one autonomous lap: locomotion policy, perception simulator, waypoint follower.
#
# The policy binary and the simulator are separate processes joined over DDS, so all three
# are started here and torn down together. The elapsed lap time is printed by the
# simulator when the final waypoint is reached.
#
#   scripts/run_race.sh                       # default course and tuning
#   scripts/run_race.sh --headless            # no viewer, for batch evaluation
#   S10_MUJOCO_XML=/path/model.xml scripts/run_race.sh

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"

if [[ ! -f "${REPO_ROOT}/install/setup.bash" ]]; then
  echo "error: workspace not built. Run scripts/build.sh first." >&2
  exit 1
fi

# ROS setup scripts read variables that may be unset, so -u is lifted across the source.
set +u
# shellcheck disable=SC1091
source "${REPO_ROOT}/install/setup.bash"
set -u

LAUNCH_ARGS=()
for arg in "$@"; do
  case "${arg}" in
    --headless) export S10_USE_VIEWER=0 ;;
    *) LAUNCH_ARGS+=("${arg}") ;;
  esac
done

pids=()
cleanup() {
  echo
  echo "==> Shutting down"
  for pid in "${pids[@]}"; do
    kill "${pid}" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "==> Starting locomotion policy (rl_deploy)"
ros2 run s10_sdk_deploy rl_deploy &
pids+=($!)

# The policy needs its ONNX session up before the simulator starts stepping physics.
sleep 2

echo "==> Starting simulator and waypoint follower"
ros2 launch s10_bringup race.launch.py "${LAUNCH_ARGS[@]}" &
pids+=($!)

wait -n "${pids[@]}"
