#!/usr/bin/env bash
# Replay a perceptive ONNX policy from one waypoint on the contest MuJoCo map.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WAYPOINT="${1:-16}"
SPEED="${2:-0.60}"
DURATION="${3:-20}"
LOG_DIR="${S10_REPLAY_LOG_DIR:-/tmp/s10_replay_waypoint_${WAYPOINT}}"
COURSE_FILE="${S10_COURSE_FILE:-${REPO_ROOT}/src/s10_bringup/config/course.yaml}"
COURSE_WAYPOINT="${S10_COURSE_START_WAYPOINT:-${WAYPOINT}}"

: "${S10_POLICY_PATH:?Set S10_POLICY_PATH to an exported 57- or 174-dimensional ONNX policy}"

if [[ ! -f "${REPO_ROOT}/install/setup.bash" ]]; then
  echo "error: workspace not built; run scripts/build.sh first" >&2
  exit 1
fi
if [[ ! -f "${S10_POLICY_PATH}" ]]; then
  echo "error: policy not found: ${S10_POLICY_PATH}" >&2
  exit 1
fi
if [[ ! -f "${COURSE_FILE}" ]]; then
  echo "error: course not found: ${COURSE_FILE}" >&2
  exit 1
fi

set +u
# shellcheck disable=SC1091
source "${REPO_ROOT}/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-26}"
export S10_START_WAYPOINT="${S10_SIM_START_WAYPOINT:-${WAYPOINT}}"
export S10_USE_VIEWER="${S10_USE_VIEWER:-0}"
export S10_USE_LIDAR="${S10_USE_LIDAR:-0}"
mkdir -p "${LOG_DIR}"

pids=()
recorder_pid=""
cleanup() {
  for pid in "${pids[@]}"; do
    kill -- "-${pid}" 2>/dev/null || kill "${pid}" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if [[ -n "${S10_RECORD_QPOS:-}" ]]; then
  # Start the slow-importing recorder before physics so the robot does not sit
  # uncontrolled for 12 s and keep it alive through the full replay duration.
  setsid ros2 run s10_perception viewer_node --record-path "${S10_RECORD_QPOS}" \
    --duration "$((DURATION + 16))" --hz 10 >"${LOG_DIR}/recorder.log" 2>&1 &
  recorder_pid=$!
  pids+=("${recorder_pid}")
  # Importing MuJoCo takes several seconds on WSL; let it subscribe before publishers start.
  sleep 12
fi
setsid ros2 run s10_sdk_deploy rl_deploy >"${LOG_DIR}/policy.log" 2>&1 &
pids+=($!)
sleep 2
setsid ros2 run s10_perception sim_node >"${LOG_DIR}/sim.log" 2>&1 &
pids+=($!)
sleep 2
setsid ros2 run s10_auto_nav waypoint_follower --ros-args \
  --params-file "${REPO_ROOT}/src/s10_bringup/config/nav.yaml" \
  -p course_file:="${COURSE_FILE}" \
  -p start_waypoint:="${COURSE_WAYPOINT}" \
  -p max_forward:="${SPEED}" >"${LOG_DIR}/navigation.log" 2>&1 &
pids+=($!)

sleep "${DURATION}"
timeout 5 ros2 topic echo /ground_truth/odom --once >"${LOG_DIR}/final_odom.txt" 2>&1 || true
if [[ -n "${recorder_pid}" ]]; then
  wait "${recorder_pid}" || true
fi

echo "==> Simulator"
tail -80 "${LOG_DIR}/sim.log"
echo "==> Policy"
tail -80 "${LOG_DIR}/policy.log"
echo "==> Final odometry"
cat "${LOG_DIR}/final_odom.txt"
if [[ -f "${S10_RECORD_QPOS:-}" ]]; then
  echo "==> Qpos recording: ${S10_RECORD_QPOS}"
fi
echo "==> Logs: ${LOG_DIR}"
