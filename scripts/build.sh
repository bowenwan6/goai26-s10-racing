#!/usr/bin/env bash
#
# Build the contest SDK and our packages into a single overlay workspace.
#
# The SDK lives in upstream/ and our packages in src/. colcon is pointed at both so a
# single install/ tree contains everything, and there is only one setup.bash to source.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UPSTREAM_SRC="${REPO_ROOT}/upstream/goai_embodied_future_material/src"
# The SDK vendors ONNX Runtime as prebuilt binaries under third_party/onnxruntime/{x86,arm}
# and selects between them with BUILD_PLATFORM. Its own default is x86, which links the
# wrong object format on Apple silicon and fails late, at rl_deploy, with "file in wrong
# format". Pick from the host instead; override for a cross build.
case "${BUILD_PLATFORM:-$(uname -m)}" in
  aarch64|arm64|arm) BUILD_PLATFORM=arm ;;
  x86_64|amd64|x86)  BUILD_PLATFORM=x86 ;;
  *) echo "error: unknown build platform '${BUILD_PLATFORM:-$(uname -m)}'" >&2; exit 1 ;;
esac
ROS_DISTRO_SETUP="${ROS_DISTRO_SETUP:-/opt/ros/jazzy/setup.bash}"

# Build output defaults to the workspace, but can be redirected. Point it outside the
# repo when the checkout lives on a synchronising filesystem (iCloud Drive, Dropbox):
# those services race with colcon and silently fork conflicted copies such as
# "install/drdds 2", which then break `source install/setup.bash`.
S10_BUILD_BASE="${S10_BUILD_BASE:-${REPO_ROOT}/build}"
S10_INSTALL_BASE="${S10_INSTALL_BASE:-${REPO_ROOT}/install}"
S10_LOG_BASE="${S10_LOG_BASE:-${REPO_ROOT}/log}"

if [[ ! -d "${UPSTREAM_SRC}" ]]; then
  echo "error: ${UPSTREAM_SRC} not found. Run scripts/setup_upstream.sh first." >&2
  exit 1
fi

if [[ ! -f "${ROS_DISTRO_SETUP}" ]]; then
  echo "error: ${ROS_DISTRO_SETUP} not found. Install ROS 2 Jazzy or set ROS_DISTRO_SETUP." >&2
  exit 1
fi

# ROS setup scripts read variables that may be unset, so -u is lifted across the source.
set +u
# shellcheck disable=SC1090
source "${ROS_DISTRO_SETUP}"
set -u

cd "${REPO_ROOT}"
echo "==> Building for platform: ${BUILD_PLATFORM}"

# --log-base is a colcon global option and has to precede the verb.
COLCON=(colcon)
if [[ -x "${REPO_ROOT}/.venv/bin/python" ]]; then
  COLCON=("${REPO_ROOT}/.venv/bin/python" -m colcon)
fi

"${COLCON[@]}" --log-base "${S10_LOG_BASE}" build \
  --base-paths src "${UPSTREAM_SRC}" \
  --build-base "${S10_BUILD_BASE}" \
  --install-base "${S10_INSTALL_BASE}" \
  --symlink-install \
  --cmake-args "-DBUILD_PLATFORM=${BUILD_PLATFORM}" \
  "$@"

python3 "${REPO_ROOT}/scripts/runtime_fingerprint.py" write \
  --install-base "${S10_INSTALL_BASE}"

cat <<EOF

Build complete. Source the workspace with:

    source ${S10_INSTALL_BASE}/setup.bash

EOF
