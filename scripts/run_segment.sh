#!/usr/bin/env bash
# Run a reproducible waypoint segment matrix and save machine-readable results.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
START=""
END=""
SEEDS=1
SEED_FROM=42
MAX_TIME=60
SPEED=0.35
OUT="${REPO_ROOT}/artifacts/segment_eval"
TAG="segment"
POLICY="${S10_POLICY_PATH:-}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --start) START="$2"; shift 2 ;;
    --end) END="$2"; shift 2 ;;
    --seeds) SEEDS="$2"; shift 2 ;;
    --seed-from) SEED_FROM="$2"; shift 2 ;;
    --max-time) MAX_TIME="$2"; shift 2 ;;
    --speed) SPEED="$2"; shift 2 ;;
    --out) OUT="$2"; shift 2 ;;
    --tag) TAG="$2"; shift 2 ;;
    --policy) POLICY="$2"; shift 2 ;;
    *) echo "error: unknown argument: $1" >&2; exit 2 ;;
  esac
done

if [[ -z "${START}" || -z "${END}" || -z "${POLICY}" ]]; then
  echo "usage: $0 --start N --end N --policy FILE [--seeds N --seed-from N --max-time S --speed MPS --out DIR --tag NAME]" >&2
  exit 2
fi
if (( START < 1 || END <= START || END > 33 || SEEDS < 1 )); then
  echo "error: require 1 <= start < end <= 33 and seeds >= 1" >&2
  exit 2
fi
if [[ ! -f "${POLICY}" ]]; then
  echo "error: policy not found: ${POLICY}" >&2
  exit 2
fi

mkdir -p "${OUT}"
target_log_index=$((END - 1))

for ((offset = 0; offset < SEEDS; offset++)); do
  seed=$((SEED_FROM + offset))
  yaw_values=(-3 0 3)
  lateral_values=(-0.08 0.00 0.08)
  forward_values=(-0.05 0.00 0.05)
  friction_values=(0.60 0.80 1.00)
  yaw="${yaw_values[$((seed % 3))]}"
  lateral="${lateral_values[$(((seed * 2 + 1) % 3))]}"
  forward="${forward_values[$(((seed * 5 + 2) % 3))]}"
  friction="${friction_values[$((seed % 3))]}"
  run_dir="${OUT}/${TAG}_seed${seed}"
  mkdir -p "${run_dir}"

  set +e
  S10_POLICY_PATH="${POLICY}" \
  S10_REPLAY_LOG_DIR="${run_dir}" \
  S10_START_YAW_OFFSET_DEG="${yaw}" \
  S10_START_FORWARD_OFFSET="${forward}" \
  S10_START_LATERAL_OFFSET="${lateral}" \
  S10_WHEEL_FRICTION="${friction}" \
    "${REPO_ROOT}/scripts/replay_waypoint.sh" "${START}" "${SPEED}" "${MAX_TIME}" \
      >"${run_dir}/replay.log" 2>&1
  replay_rc=$?
  set -e

  success=false
  if grep -Eq "Waypoint ${target_log_index} reached" "${run_dir}/navigation.log" 2>/dev/null; then
    success=true
  fi
  process_ok=true
  if (( replay_rc != 0 )) || grep -Eq "Traceback|what\(\):|Segmentation fault" "${run_dir}/sim.log" "${run_dir}/policy.log" 2>/dev/null; then
    process_ok=false
  fi
  reached_index=$(sed -n 's/.*Waypoint \([0-9][0-9]*\) reached.*/\1/p' "${run_dir}/navigation.log" 2>/dev/null | tail -1)
  reached_waypoint=$(( ${reached_index:--1} + 1 ))

  python3 - "${run_dir}/result.json" "${run_dir}/final_odom.txt" \
    "${REPO_ROOT}/src/s10_bringup/config/course.yaml" \
    "${TAG}" "${POLICY}" "${START}" "${END}" "${seed}" \
    "${MAX_TIME}" "${SPEED}" "${yaw}" "${forward}" "${lateral}" "${friction}" \
    "${success}" "${process_ok}" "${replay_rc}" "${reached_waypoint}" <<'PY'
import json
import math
import os
import re
import sys

import yaml

(
    path, odom_path, course_path, tag, policy, start, end, seed, max_time, speed, yaw, forward,
    lateral, friction, success, process_ok, replay_rc, reached_waypoint,
) = sys.argv[1:]
odom = open(odom_path, encoding="utf-8").read() if os.path.exists(odom_path) else ""
match = re.search(
    r"position:\s*\n\s*x:\s*([-+0-9.eE]+)\s*\n\s*y:\s*([-+0-9.eE]+)\s*\n\s*z:\s*([-+0-9.eE]+)",
    odom,
)
final_pose = [float(value) for value in match.groups()] if match else None
course = yaml.safe_load(open(course_path, encoding="utf-8"))["waypoints"]
target = course[int(end) - 1]["position"]
target_distance = (
    math.hypot(final_pose[0] - target[0], final_pose[1] - target[1])
    if final_pose is not None
    else None
)
payload = {
    "tag": tag,
    "policy": policy,
    "start_waypoint": int(start),
    "end_waypoint": int(end),
    "seed": int(seed),
    "max_time_s": float(max_time),
    "speed_m_s": float(speed),
    "perturbation": {
        "yaw_offset_deg": float(yaw),
        "forward_offset_m": float(forward),
        "lateral_offset_m": float(lateral),
        "wheel_friction": float(friction),
    },
    "success": success == "true",
    "process_ok": process_ok == "true",
    "replay_exit_code": int(replay_rc),
    "last_reached_waypoint": int(reached_waypoint),
    "final_position": final_pose,
    "target_distance_xy_m": target_distance,
}
with open(path, "w", encoding="utf-8") as stream:
    json.dump(payload, stream, indent=2)
PY
  echo "${TAG} seed=${seed} success=${success} process_ok=${process_ok} reached=${reached_waypoint}/${END}"
done

python3 - "${OUT}/${TAG}_summary.json" "${OUT}" "${TAG}" <<'PY'
import glob
import json
import os
import sys

path, directory, tag = sys.argv[1:]
results = []
for candidate in sorted(glob.glob(os.path.join(directory, f"{tag}_seed*", "result.json"))):
    with open(candidate, encoding="utf-8") as stream:
        results.append(json.load(stream))
payload = {
    "tag": tag,
    "runs": len(results),
    "successes": sum(result["success"] for result in results),
    "success_rate": sum(result["success"] for result in results) / len(results),
    "process_failures": sum(not result["process_ok"] for result in results),
    "results": results,
}
with open(path, "w", encoding="utf-8") as stream:
    json.dump(payload, stream, indent=2)
print(f"saved {path}: {payload['successes']}/{payload['runs']} successes")
PY
