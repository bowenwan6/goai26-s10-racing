#!/usr/bin/env bash
# Explicit HIM simulation entry; hardware uses start_s10_him1500_handset.cmd.
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
: "${S10_POLICY_PATH:?Set S10_POLICY_PATH to a HIM ONNX model with its JSON sidecar}"
export S10_POLICY_PATH
export S10_SECOND_POLICY_PATH="${S10_SECOND_POLICY_PATH-}"
export S10_DOWN_POLICY_PATH="${S10_DOWN_POLICY_PATH-}"
export S10_SPEEDTURN_POLICY_PATH="${S10_SPEEDTURN_POLICY_PATH-}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"
for policy in "$S10_POLICY_PATH" "$S10_SECOND_POLICY_PATH" "$S10_DOWN_POLICY_PATH" "$S10_SPEEDTURN_POLICY_PATH"; do
  [[ -z "$policy" ]] && continue
  [[ -f "$policy" ]] || { echo "error: policy not found: $policy" >&2; exit 1; }
done
python3 "$REPO_ROOT/robot/scripts/patch_him_upstream.py" --check
S10_INSTALL_BASE="${S10_INSTALL_BASE:-$REPO_ROOT/install}"
python3 "$REPO_ROOT/robot/scripts/runtime_fingerprint.py" check --install-base "$S10_INSTALL_BASE" --controller him
set +u
source "$S10_INSTALL_BASE/setup.bash"
set -u
pids=()
cleanup() {
  for pid in "${pids[@]}"; do kill "$pid" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM
ros2 run s10_sdk_deploy rl_deploy &
pids+=($!)
sleep 2
ros2 launch s10_bringup race.launch.py "$@" strategy_router:=false &
pids+=($!)
wait -n "${pids[@]}"
