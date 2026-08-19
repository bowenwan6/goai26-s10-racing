#!/usr/bin/env bash
# Run one continuous MuJoCo experiment, capture lightweight replay state, render at 1080p,
# and encode a true wall-clock-timed MP4. Raw evidence and video stay outside the repository.

set -euo pipefail

usage() {
  echo "usage: $0 OUTPUT_DIR [START=0] [END=32] [SEED=6] [MAX_TIME=2400]" >&2
  exit 2
}

[[ $# -ge 1 && $# -le 5 ]] || usage

OUTPUT_DIR="$1"
START="${2:-0}"
END="${3:-32}"
SEED="${4:-6}"
MAX_TIME="${5:-2400}"
TIMING="${S10_VIDEO_TIMING:-wall}"

[[ "${TIMING}" == "wall" || "${TIMING}" == "simulation" ]] || {
  echo "error: S10_VIDEO_TIMING must be 'wall' or 'simulation'" >&2
  exit 2
}

[[ "${OUTPUT_DIR}" = /* ]] || {
  echo "error: OUTPUT_DIR must be absolute" >&2
  exit 2
}
command -v docker >/dev/null || { echo "error: docker is required" >&2; exit 2; }
command -v ffmpeg >/dev/null || { echo "error: ffmpeg is required on the host" >&2; exit 2; }
OVERLAY_FONT="${S10_VIDEO_FONT:-/System/Library/Fonts/Supplemental/Arial.ttf}"
[[ -f "${OVERLAY_FONT}" ]] || {
  echo "error: overlay font not found: ${OVERLAY_FONT} (set S10_VIDEO_FONT)" >&2
  exit 2
}
[[ "${OVERLAY_FONT}" != *"'"* && "${OVERLAY_FONT}" != *$'\n'* ]] || {
  echo "error: S10_VIDEO_FONT must not contain a quote or newline" >&2
  exit 2
}

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${IMAGE:-s10-racing:dev}"
BUILD_VOLUME="${BUILD_VOLUME:-s10-racing-build}"
RUN_NAME="$(printf '%02d_%02d_seed%s' "${START}" "${END}" "${SEED}")"
RAW_DIR="${OUTPUT_DIR}/raw"
FRAMES_NAME="frames_1080p_${TIMING}"
FRAMES_DIR="${OUTPUT_DIR}/video/${FRAMES_NAME}"
MP4="${OUTPUT_DIR}/video/wp${START}_to_wp${END}_seed${SEED}_1080p_${TIMING}.mp4"

mkdir -p "${RAW_DIR}" "${FRAMES_DIR}"

if [[ "${S10_SKIP_CAPTURE:-0}" != "1" ]]; then
  docker rm --force s10-realtime-capture >/dev/null 2>&1 || true
  docker run --rm -i --name s10-realtime-capture --network host \
    -v "${REPO_ROOT}:/ws" \
    -v "${BUILD_VOLUME}:/opt/s10-build" \
    -v "${OUTPUT_DIR}:/evidence" \
    -w /ws \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}" \
    -e S10_USE_VIEWER=0 \
    -e S10_BUILD_BASE=/opt/s10-build/build \
    -e S10_INSTALL_BASE=/opt/s10-build/install \
    -e S10_LOG_BASE=/opt/s10-build/log \
    -e S10_SEGMENT_VIDEO_MODE=replay \
    -e S10_SEGMENT_VIDEO_WIDTH=1920 \
    -e S10_SEGMENT_VIDEO_HEIGHT=1080 \
    "${IMAGE}" bash -lc \
    "source /opt/ros/jazzy/setup.bash && source /opt/s10-build/install/setup.bash && \
     scripts/run_segment.py --start ${START} --end ${END} --seeds 1 --seed-from ${SEED} \
     --max-time ${MAX_TIME} --out /evidence/raw --router \
     --router-params /ws/src/s10_bringup/config/strategy_gate16.yaml --video --video-hz 10"
fi

REPLAY="${RAW_DIR}/${RUN_NAME}_frames/replay.npz"
[[ -f "${REPLAY}" ]] || { echo "error: replay not produced at ${REPLAY}" >&2; exit 1; }

# Four OSMesa workers fit the standard 8 GiB Docker Desktop VM. Override this on
# larger or smaller machines; every worker renders a disjoint deterministic shard.
RENDER_JOBS="${S10_RENDER_JOBS:-4}"
[[ "${RENDER_JOBS}" =~ ^[1-9][0-9]*$ ]] || {
  echo "error: S10_RENDER_JOBS must be a positive integer" >&2
  exit 2
}

# Generate the shared timeline once, then render disjoint frame ranges concurrently. Each
# worker owns different PNG names, so the result is byte-for-byte equivalent to serial order
# while making a 15-minute 1080p replay practical on a multicore workstation.
docker rm --force s10-realtime-manifest >/dev/null 2>&1 || true
FRAME_COUNT="$(docker run --rm -i --name s10-realtime-manifest \
  -v "${REPO_ROOT}:/ws" \
  -v "${BUILD_VOLUME}:/opt/s10-build" \
  -v "${OUTPUT_DIR}:/evidence" \
  -w /ws -e MUJOCO_GL=osmesa \
  "${IMAGE}" bash -lc \
  "source /opt/ros/jazzy/setup.bash && source /opt/s10-build/install/setup.bash && \
   python3 -c \"import numpy as np; print(len(np.load('/evidence/raw/${RUN_NAME}_frames/replay.npz')['qpos']))\"")"
[[ "${FRAME_COUNT}" =~ ^[1-9][0-9]*$ ]] || {
  echo "error: could not determine replay frame count: ${FRAME_COUNT}" >&2
  exit 1
}
docker run --rm -i --name s10-realtime-manifest \
  -v "${REPO_ROOT}:/ws" \
  -v "${BUILD_VOLUME}:/opt/s10-build" \
  -v "${OUTPUT_DIR}:/evidence" \
  -w /ws -e MUJOCO_GL=osmesa \
  "${IMAGE}" bash -lc \
  "source /opt/ros/jazzy/setup.bash && source /opt/s10-build/install/setup.bash && \
   scripts/render_replay_3d.py /evidence/raw/${RUN_NAME}_frames/replay.npz \
   --out /evidence/video/${FRAMES_NAME} --width 1920 --height 1080 \
   --timing ${TIMING} --overlay-font '${OVERLAY_FONT}' --manifest-only"

render_pids=()
for ((worker = 0; worker < RENDER_JOBS; worker++)); do
  start=$((worker * FRAME_COUNT / RENDER_JOBS))
  stop=$(((worker + 1) * FRAME_COUNT / RENDER_JOBS))
  ((start < stop)) || continue
  name="s10-realtime-render-${worker}"
  docker rm --force "${name}" >/dev/null 2>&1 || true
  docker run --rm -i --name "${name}" \
    -v "${REPO_ROOT}:/ws" \
    -v "${BUILD_VOLUME}:/opt/s10-build" \
    -v "${OUTPUT_DIR}:/evidence" \
    -w /ws -e MUJOCO_GL=osmesa \
    "${IMAGE}" bash -lc \
    "source /opt/ros/jazzy/setup.bash && source /opt/s10-build/install/setup.bash && \
     scripts/render_replay_3d.py /evidence/raw/${RUN_NAME}_frames/replay.npz \
     --out /evidence/video/${FRAMES_NAME} --width 1920 --height 1080 \
     --frame-start ${start} --frame-stop ${stop} --no-manifest" &
  render_pids+=("$!")
done

render_status=0
for pid in "${render_pids[@]}"; do
  wait "${pid}" || render_status=1
done
((render_status == 0)) || {
  echo "error: one or more replay render workers failed" >&2
  exit 1
}

# Validate that every manifest entry now has a frame before encoding.
missing=0
for ((frame = 0; frame < FRAME_COUNT; frame++)); do
  path="${FRAMES_DIR}/$(printf '%05d.png' "${frame}")"
  [[ -f "${path}" ]] || { echo "missing frame: ${path}" >&2; missing=1; }
done
((missing == 0)) || exit 1

trim_args=()
if [[ "${TIMING}" == "simulation" ]]; then
  RUN_LOG="${RAW_DIR}/${RUN_NAME}.log"
  OFFICIAL_DURATION="$(
    sed -nE 's/.*Final waypoint reached.*elapsed=([0-9.]+)s.*/\1/p' "${RUN_LOG}" | tail -1
  )"
  [[ "${OFFICIAL_DURATION}" =~ ^[0-9]+([.][0-9]+)?$ ]] || {
    echo "error: official simulator elapsed time not found in ${RUN_LOG}" >&2
    exit 1
  }
  trim_args=(-t "${OFFICIAL_DURATION}")
fi

ffmpeg -y -f concat -safe 0 -i "${FRAMES_DIR}/frames.ffconcat" \
  -filter_script:v "${FRAMES_DIR}/overlay_filters.txt" \
  -c:v libx264 -preset slow -crf 15 -pix_fmt yuv420p -r 30 \
  -vsync cfr "${trim_args[@]}" -movflags +faststart "${MP4}"

echo "video: ${MP4}"
