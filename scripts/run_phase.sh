#!/usr/bin/env bash
# Run either 59D flat phase checkpoint through the existing MuJoCo/Windows viewer.
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
GROUP="${1:-A}"
if (($#)); then shift; fi
case "${GROUP}" in
  A|B) ;;
  *) echo "Usage: bash scripts/run_phase.sh A|B [--headless]" >&2; exit 2 ;;
esac
export S10_POLICY_PATH="${REPO_ROOT}/policies/phase_20260917/${GROUP}500.onnx"
[[ -f "${S10_POLICY_PATH}" ]] || { echo "Missing ${S10_POLICY_PATH}" >&2; exit 1; }
export S10_POLICY_SLOT=0
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-59}"
export ROS_LOCALHOST_ONLY=1
echo "Phase ${GROUP}500: 59D, sin/cos, 0.6 s cycle, 50 Hz. Z: stand; C: RL; WASD/QE: commands."
exec bash "${REPO_ROOT}/scripts/run_race.sh" --manual "$@"
