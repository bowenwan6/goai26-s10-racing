#!/usr/bin/env bash
#
# Build the contest SDK and our packages into a single overlay workspace.
#
# The SDK lives in upstream/ and our packages in src/. colcon is pointed at both so a
# single install/ tree contains everything, and there is only one setup.bash to source.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UPSTREAM_SRC="${REPO_ROOT}/upstream/goai_embodied_future_material/src"
BUILD_PLATFORM="${BUILD_PLATFORM:-x86}"
ROS_DISTRO_SETUP="${ROS_DISTRO_SETUP:-/opt/ros/jazzy/setup.bash}"

if [[ ! -d "${UPSTREAM_SRC}" ]]; then
  echo "error: ${UPSTREAM_SRC} not found. Run scripts/setup_upstream.sh first." >&2
  exit 1
fi

if [[ ! -f "${ROS_DISTRO_SETUP}" ]]; then
  echo "error: ${ROS_DISTRO_SETUP} not found. Install ROS 2 Jazzy or set ROS_DISTRO_SETUP." >&2
  exit 1
fi

# ROS setup files are not nounset-safe.
set +u
# shellcheck disable=SC1090
source "${ROS_DISTRO_SETUP}"
set -u

cd "${REPO_ROOT}"
echo "==> Building for platform: ${BUILD_PLATFORM}"

COLCON=(colcon)
if [[ -x "${REPO_ROOT}/.venv/bin/python" ]]; then
  COLCON=("${REPO_ROOT}/.venv/bin/python" -m colcon)
fi

"${COLCON[@]}" build \
  --base-paths src "${UPSTREAM_SRC}" \
  --symlink-install \
  --cmake-args "-DBUILD_PLATFORM=${BUILD_PLATFORM}" \
  "$@"

cat <<EOF

Build complete. Source the workspace with:

    source ${REPO_ROOT}/install/setup.bash

EOF
