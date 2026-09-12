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
#   scripts/run_race.sh --manual              # terminal or focused MuJoCo WASD
#   S10_MUJOCO_XML=/path/model.xml scripts/run_race.sh

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"
export GALLIUM_DRIVER="${GALLIUM_DRIVER:-d3d12}"
export MESA_D3D12_DEFAULT_ADAPTER_NAME="${MESA_D3D12_DEFAULT_ADAPTER_NAME:-NVIDIA}"
export S10_SECOND_POLICY_PATH="${S10_SECOND_POLICY_PATH:-${REPO_ROOT}/policies/s10_stairs_stable_up_57d_model499.onnx}"
export S10_DOWN_POLICY_PATH="${S10_DOWN_POLICY_PATH:-${REPO_ROOT}/policies/stable_down_compare/model300.onnx}"
export S10_SPEEDTURN_POLICY_PATH="${S10_SPEEDTURN_POLICY_PATH:-${REPO_ROOT}/policies/speedturn_sweep/model_2000.onnx}"

for policy in "${S10_SECOND_POLICY_PATH}" "${S10_DOWN_POLICY_PATH}" "${S10_SPEEDTURN_POLICY_PATH}"; do
  [[ -f "${policy}" ]] || { echo "error: policy not found: ${policy}" >&2; exit 1; }
done

if [[ ! -f "${REPO_ROOT}/install/setup.bash" ]]; then
  echo "error: workspace not built. Run scripts/build.sh first." >&2
  exit 1
fi

# ROS setup files are not nounset-safe.
set +u
# shellcheck disable=SC1091
source "${REPO_ROOT}/install/setup.bash"
set -u

LAUNCH_ARGS=()
MANUAL=false
for arg in "$@"; do
  case "${arg}" in
    --headless) export S10_USE_VIEWER=0 ;;
    --manual) MANUAL=true ;;
    *) LAUNCH_ARGS+=("${arg}") ;;
  esac
done

if [[ "${S10_USE_VIEWER:-1}" != "0" ]] && command -v glxinfo >/dev/null; then
  if [[ "${MANUAL}" != true ]] || [[ "${S10_VIEWER_BACKEND:-windows}" == "wsl" ]]; then
    glxinfo -B 2>/dev/null | grep "OpenGL renderer string" || true
  fi
fi

pids=()
cleaned=false
cleanup() {
  if [[ "${cleaned}" == true ]]; then
    return
  fi
  cleaned=true
  echo
  echo "==> Shutting down"
  for pid in "${pids[@]}"; do
    kill "${pid}" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM

if [[ "${MANUAL}" == true ]]; then
  if ((${#LAUNCH_ARGS[@]})); then
    echo "error: manual mode accepts only --headless; use S10_MUJOCO_XML for a custom scene." >&2
    exit 2
  fi

  export S10_USE_PERCEPTION=0
  echo "==> Starting manual control"
  echo "    Focus the terminal or MuJoCo: Z/C, WASD/QE, V/M, H, P/L, 0"
  echo "    Obstacle tuning: 7 slower, 8 faster (1 rad/s per press)"
  echo "    MuJoCo viewer: Ctrl+Q or close the window to exit"
  (
    sleep 2
    echo "==> Starting headless simulator"
    export S10_USE_VIEWER=0
    exec ros2 run s10_perception sim_node
  ) &
  pids+=($!)

  if [[ "${S10_USE_VIEWER:-1}" != "0" ]]; then
    if [[ "${S10_VIEWER_BACKEND:-windows}" == "windows" ]]; then
      WIN_PYTHON="${REPO_ROOT}/.venv-win/Scripts/python.exe"
      WIN_PYTHON_PATH="$(wslpath -m "${WIN_PYTHON}")"
      WIN_VIEWER="$(wslpath -m "${REPO_ROOT}/scripts/windows_viewer.py")"
      WIN_VIEWER_LAUNCHER="$(wslpath -m "${REPO_ROOT}/scripts/start_windows_viewer.ps1")"
      MODEL_PATH="${S10_MUJOCO_XML:-${REPO_ROOT}/upstream/goai_embodied_future_material/src/S10_sdk_deploy/S10_description/s10_mjcf/mjcf/S10_track.xml}"
      if [[ ! -f "${MODEL_PATH}" ]]; then
        echo "error: MuJoCo model not found: ${MODEL_PATH}" >&2
        exit 1
      fi
      WIN_MODEL_PATH="$(wslpath -m "${MODEL_PATH}")"
      VIEWER_PORT="${S10_VIEWER_PORT:-18777}"
      WINDOWS_HOST="${S10_WINDOWS_HOST:-$(ip route show default | awk '/default/ {print $3; exit}')}"
      if [[ ! -x "${WIN_PYTHON}" ]]; then
        echo "error: Windows viewer environment missing. Create .venv-win and install mujoco." >&2
        exit 1
      fi

      (
        sleep 4
        echo "==> Starting WSL-to-Windows viewer stream"
        exec ros2 run s10_perception viewer_node \
          --stream-host "${WINDOWS_HOST}" --stream-port "${VIEWER_PORT}"
      ) &
      pids+=($!)
      echo "==> Starting native Windows viewer"
      powershell.exe -NoProfile -ExecutionPolicy Bypass -File "${WIN_VIEWER_LAUNCHER}" \
        -Python "${WIN_PYTHON_PATH}" -Viewer "${WIN_VIEWER}" \
        -Model "${WIN_MODEL_PATH}" -Port "${VIEWER_PORT}"
    else
      (
        sleep 4
        echo "==> Starting detached WSLg viewer"
        exec ros2 run s10_perception viewer_node
      ) &
      pids+=($!)
    fi
  fi

  S10_MANUAL=1 ros2 run s10_sdk_deploy rl_deploy
  exit
fi

echo "==> Starting locomotion policy (rl_deploy)"
ros2 run s10_sdk_deploy rl_deploy &
pids+=($!)

# The policy needs its ONNX session up before the simulator starts stepping physics.
sleep 2

echo "==> Starting simulator and waypoint follower"
ros2 launch s10_bringup race.launch.py "${LAUNCH_ARGS[@]}" &
pids+=($!)

wait -n "${pids[@]}"
