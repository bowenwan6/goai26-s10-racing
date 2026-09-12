#!/usr/bin/env bash
# Record a deterministic in-place-turn and straight-speed profile for one policy.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
POLICY="${1:?policy.onnx is required}"
OUTPUT="${2:?output.npy is required}"
DOMAIN="${3:-47}"
LOG_DIR="${S10_PROFILE_LOG_DIR:-/tmp/s10_velocity_profile_${DOMAIN}}"

set +u
# shellcheck disable=SC1091
source "${REPO_ROOT}/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${DOMAIN}"
export S10_POLICY_PATH="${POLICY}"
export S10_START_WAYPOINT=2
export S10_USE_VIEWER=0
export S10_USE_LIDAR=0
mkdir -p "${LOG_DIR}" "$(dirname "${OUTPUT}")"

pids=()
cleanup() {
  for pid in "${pids[@]}"; do
    kill -- "-${pid}" 2>/dev/null || kill "${pid}" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

setsid ros2 run s10_sdk_deploy rl_deploy >"${LOG_DIR}/policy.log" 2>&1 &
pids+=($!)
sleep 2
setsid ros2 run s10_perception sim_node >"${LOG_DIR}/sim.log" 2>&1 &
pids+=($!)
sleep 10

setsid ros2 run s10_perception viewer_node --record-path "${OUTPUT}" --duration 22 --hz 20 \
  >"${LOG_DIR}/recorder.log" 2>&1 &
recorder_pid=$!
pids+=("${recorder_pid}")
for _ in {1..60}; do
  ros2 node list 2>/dev/null | grep -q '/mujoco_viewer' && break
  sleep 0.5
done
ros2 node list 2>/dev/null | grep -q '/mujoco_viewer'
"${REPO_ROOT}/.venv/bin/python" "${REPO_ROOT}/scripts/profile_commands.py" >"${LOG_DIR}/commands.log" 2>&1
wait "${recorder_pid}"
echo "saved ${OUTPUT}"
