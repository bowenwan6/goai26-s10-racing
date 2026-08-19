#!/usr/bin/env bash
#
# Run one autonomous lap: locomotion policy, perception simulator, waypoint follower.
#
# The policy binary and the simulator are separate processes joined over DDS, so all three
# are started here and torn down together. The elapsed lap time is printed by the
# simulator when the final waypoint is reached.
#
#   scripts/run_race.sh                       # verified competition configuration
#   scripts/run_race.sh --headless            # no viewer, for batch evaluation
#   S10_STRATEGY_ROUTER=0 scripts/run_race.sh # official policy only (diagnostic)
#   S10_MUJOCO_XML=/path/model.xml scripts/run_race.sh

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"

# Must match whatever scripts/build.sh installed into; see the note there on why this
# is redirectable.
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

LAUNCH_ARGS=()
HAS_ROUTER_ARG=0
HAS_ROUTER_PARAMS_ARG=0
STRATEGY_ROUTER="${S10_STRATEGY_ROUTER:-1}"
for arg in "$@"; do
  case "${arg}" in
    --headless) export S10_USE_VIEWER=0 ;;
    strategy_router:=true) HAS_ROUTER_ARG=1; STRATEGY_ROUTER=1; LAUNCH_ARGS+=("${arg}") ;;
    strategy_router:=false) HAS_ROUTER_ARG=1; STRATEGY_ROUTER=0; LAUNCH_ARGS+=("${arg}") ;;
    strategy_router:=*)
      echo "error: strategy_router launch argument must be true or false" >&2
      exit 2
      ;;
    router_params:=*) HAS_ROUTER_PARAMS_ARG=1; LAUNCH_ARGS+=("${arg}") ;;
    *) LAUNCH_ARGS+=("${arg}") ;;
  esac
done

# The validated competition path uses the Gate 16 router for WP15->WP16 and gives control
# back to the official follower everywhere else.  Keep race.launch.py conservative for
# developers who invoke it directly, but make this documented one-shot entry point reproduce
# the tested stack.  Explicit launch arguments still win, and the environment switch provides
# a simple official-policy-only diagnostic.
[[ "${STRATEGY_ROUTER}" == "0" || "${STRATEGY_ROUTER}" == "1" ]] || {
  echo "error: S10_STRATEGY_ROUTER must be 0 or 1" >&2
  exit 2
}
if ((HAS_ROUTER_ARG == 0)); then
  if [[ "${STRATEGY_ROUTER}" == "1" ]]; then
    LAUNCH_ARGS+=("strategy_router:=true")
  else
    LAUNCH_ARGS+=("strategy_router:=false")
  fi
fi
if ((HAS_ROUTER_PARAMS_ARG == 0)) && [[ "${STRATEGY_ROUTER}" == "1" ]]; then
  LAUNCH_ARGS+=("router_params:=${REPO_ROOT}/src/s10_bringup/config/strategy_gate16.yaml")
fi

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
if [[ " ${LAUNCH_ARGS[*]} " == *" strategy_router:=true "* ]]; then
  echo "==> Gate 16 router enabled; official policy remains active outside WP15->WP16"
fi
ros2 launch s10_bringup race.launch.py "${LAUNCH_ARGS[@]}" &
pids+=($!)

wait -n "${pids[@]}"
