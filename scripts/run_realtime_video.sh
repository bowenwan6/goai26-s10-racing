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

[[ "${OUTPUT_DIR}" = /* ]] || {
  echo "error: OUTPUT_DIR must be absolute" >&2
  exit 2
}
command -v docker >/dev/null || { echo "error: docker is required" >&2; exit 2; }
command -v ffmpeg >/dev/null || { echo "error: ffmpeg is required on the host" >&2; exit 2; }

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${IMAGE:-s10-racing:dev}"
BUILD_VOLUME="${BUILD_VOLUME:-s10-racing-build}"
RUN_NAME="$(printf '%02d_%02d_seed%s' "${START}" "${END}" "${SEED}")"
RAW_DIR="${OUTPUT_DIR}/raw"
FRAMES_DIR="${OUTPUT_DIR}/video/frames_1080p_realtime"
MP4="${OUTPUT_DIR}/video/wp${START}_to_wp${END}_seed${SEED}_1080p_realtime.mp4"

mkdir -p "${RAW_DIR}" "${FRAMES_DIR}"

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

REPLAY="${RAW_DIR}/${RUN_NAME}_frames/replay.npz"
[[ -f "${REPLAY}" ]] || { echo "error: replay not produced at ${REPLAY}" >&2; exit 1; }

docker rm --force s10-realtime-render >/dev/null 2>&1 || true
docker run --rm -i --name s10-realtime-render \
  -v "${REPO_ROOT}:/ws" \
  -v "${BUILD_VOLUME}:/opt/s10-build" \
  -v "${OUTPUT_DIR}:/evidence" \
  -w /ws -e MUJOCO_GL=osmesa \
  "${IMAGE}" bash -lc \
  "source /opt/ros/jazzy/setup.bash && source /opt/s10-build/install/setup.bash && \
   scripts/render_replay_3d.py /evidence/raw/${RUN_NAME}_frames/replay.npz \
   --out /evidence/video/frames_1080p_realtime --width 1920 --height 1080 \
   --timing wall"

ffmpeg -y -f concat -safe 0 -i "${FRAMES_DIR}/frames.ffconcat" \
  -c:v libx264 -preset slow -crf 15 -pix_fmt yuv420p -r 30 \
  -vsync cfr -movflags +faststart "${MP4}"

echo "video: ${MP4}"
