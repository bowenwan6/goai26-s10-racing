#!/usr/bin/env bash
set -euo pipefail

task_stage=bootstrap
task_started=$(date -Iseconds)
task_exit_report() {
  task_code=$?
  printf 'pipeline_exit code=%s stage=%s started=%s finished=%s\n' \
    "$task_code" "$task_stage" "$task_started" "$(date -Iseconds)"
}
trap task_exit_report EXIT

policy_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
work_root=${S10_WORK_ROOT:-/home/bella/goai/goai26-s10-racing}
task_python=${S10_POLICY_PYTHON:-/home/bella/goai/venv-s10-policy/bin/python}
task_track_xml="$work_root/upstream/goai_embodied_future_material/src/S10_sdk_deploy/S10_description/s10_mjcf/mjcf/S10_track.xml"
task_base="$work_root/training/assets/s10_29cm_stable.pt"
task_previous_run=${1:?verified baseline Gate-16 run is required}
task_run=${2:?output directory is required}
task_formal_matrix="$policy_root/training/config/gate16_retention_generalization_v2.csv"
task_speed_matrix="$policy_root/training/config/gate16_speed_boundary_v1.csv"
task_video_matrix="$policy_root/training/config/gate16_speed_video_v1.csv"
task_initial="$task_previous_run/selected/best_checkpoint.pt"
task_previous_selection="$task_previous_run/selected/selection.json"

for task_required in \
  "$task_python" "$task_track_xml" "$task_base" "$task_initial" \
  "$task_previous_selection" "$task_formal_matrix" "$task_speed_matrix" \
  "$task_video_matrix"; do
  if [[ ! -e "$task_required" ]]; then
    echo "missing required path: $task_required" >&2
    exit 2
  fi
done
mkdir -p "$task_run"
cd "$policy_root/training"

# Reuse the completed baseline formal matrix.  Only the new paired speed matrix
# needs one baseline run.
task_baseline_formal=$("$task_python" - "$task_previous_selection" <<'PY'
import json, sys
payload = json.load(open(sys.argv[1]))
selected = payload["selected"]
print(next(row["summary"] for row in payload["candidates"] if row["name"] == selected))
PY
)
if [[ ! -f "$task_baseline_formal" ]]; then
  echo "baseline formal summary is missing: $task_baseline_formal" >&2
  exit 3
fi

evaluate_candidate() {
  local task_name=$1
  local task_checkpoint=$2
  local task_cases=$3
  local task_output=$4
  task_stage="evaluate_$task_name"
  set +e
  PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. "$task_python" \
    -m mujoco_s10.collect_residual_demonstrations \
    --xml "$task_track_xml" \
    --base-checkpoint "$task_base" \
    --residual-checkpoint "$task_checkpoint" \
    --output "$task_output" \
    --entry-cases-csv "$task_cases" \
    --activation-mode heightmap --correction-limit 4.0 \
    --seed 377510
  local task_code=$?
  set -e
  if [[ $task_code -ne 0 && $task_code -ne 2 ]]; then
    return "$task_code"
  fi
  test -f "$task_output/summary.json"
}

task_stage=evaluate_baseline_speed
task_baseline_speed="$task_run/evaluations/baseline_speed"
evaluate_candidate baseline_speed "$task_initial" \
  "$task_speed_matrix" "$task_baseline_speed"

task_candidate_args=(
  --candidate baseline "$task_initial" "$task_baseline_formal" \
  "$task_baseline_speed/summary.json"
)

select_current() {
  local task_name=$1
  local task_output="$task_run/selections/$task_name"
  task_stage="select_$task_name"
  PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. "$task_python" \
    -m mujoco_s10.select_gate16_fastest \
    "${task_candidate_args[@]}" --baseline-name baseline \
    --output "$task_output" >/dev/null
  task_selected="$task_output/best_checkpoint.pt"
}

run_speed_stage() {
  local task_name=$1
  local task_input=$2
  local task_speed_min=$3
  local task_speed_max=$4
  local task_iterations=$5
  local task_seed=$6
  local task_output="$task_run/$task_name"

  task_stage="train_$task_name"
  OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 \
  PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. "$task_python" \
    -m mujoco_s10.train_official_policy_residual \
    --terrain-mode official_track --objective-mode speed \
    --xml "$task_track_xml" --base-checkpoint "$task_base" \
    --initial-residual-checkpoint "$task_input" \
    --output "$task_output" --device cuda \
    --actor-hidden 512 512 256 128 \
    --envs 12 --env-step-workers 1 --steps-per-env 32 \
    --iterations "$task_iterations" \
    --learning-rate 1e-5 --actor-learning-rate 2e-8 \
    --critic-warmup-iterations 40 \
    --learning-epochs 1 --mini-batches 4 --clip-param 0.05 \
    --entropy-coef 0 --initial-log-std -3.5 --zero-anchor-coef 2.0 \
    --eval-interval 40 --eval-distance-grid 3 --eval-speed-grid 3 \
    --eval-yaw-degrees -8 0 8 --save-interval 40 --early-stop-evals 3 \
    --activation-mode heightmap --correction-limit 4.0 \
    --distance-min 0.55 --distance-max 0.65 \
    --approach-speed 0.20 \
    --speed-min "$task_speed_min" --speed-max "$task_speed_max" \
    --lateral-range 0.02 --yaw-range 0.1396263 --height-noise 0 \
    --training-hard-distances 0.55 0.60 0.65 \
    --training-hard-probability 0.85 \
    --speed-step-penalty 0.05 \
    --speed-success-time-scale 0.50 \
    --speed-fall-penalty 250.0 \
    --seed "$task_seed"

  local task_checkpoint="$task_output/best_checkpoint.pt"
  local task_formal="$task_run/evaluations/${task_name}_formal"
  local task_speed="$task_run/evaluations/${task_name}_speed"
  evaluate_candidate "${task_name}_formal" "$task_checkpoint" \
    "$task_formal_matrix" "$task_formal"
  evaluate_candidate "${task_name}_speed" "$task_checkpoint" \
    "$task_speed_matrix" "$task_speed"
  task_candidate_args+=(
    --candidate "$task_name" "$task_checkpoint" \
    "$task_formal/summary.json" "$task_speed/summary.json"
  )
  select_current "$task_name"
}

# Core branch: deployment-aligned +/-8 degrees and up to 0.40 m/s.
run_speed_stage speed_core "$task_initial" 0.18 0.40 200 377511

# Push branch: increase entry speed, but always starts from the fastest candidate
# that already passed both paired baseline success gates.
run_speed_stage speed_push "$task_selected" 0.25 0.50 160 377512

task_stage=final_selection
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. "$task_python" \
  -m mujoco_s10.select_gate16_fastest \
  "${task_candidate_args[@]}" --baseline-name baseline \
  --output "$task_run/selected"

task_stage=export_deploy_bundle
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. "$task_python" \
  -m s10_rl.export_climb_bundle \
  --base-checkpoint "$task_base" \
  --residual-checkpoint "$task_run/selected/best_checkpoint.pt" \
  --output "$task_run/deploy_policy"

task_stage=render_fast_success
set +e
MUJOCO_GL=egl PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. "$task_python" \
  -m mujoco_s10.render_official_residual \
  --xml "$task_track_xml" \
  --base-checkpoint "$task_base" \
  --residual-checkpoint "$task_run/selected/best_checkpoint.pt" \
  --output "$task_run/gate16_fastest_success.mp4" \
  --summary "$task_run/gate16_fastest_video.json" \
  --entry-cases-csv "$task_video_matrix"
task_video_code=$?
set -e
if [[ $task_video_code -ne 0 ]]; then
  echo "no speed-matrix success video rendered; matrix evidence remains" >&2
fi

task_stage=all_complete
echo "run=$task_run"
echo "selection=$task_run/selected/selection.json"
echo "deploy=$task_run/deploy_policy"
echo "video=$task_run/gate16_fastest_success.mp4"
