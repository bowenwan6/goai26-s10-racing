#!/usr/bin/env bash
#
# Fetch the contest material and wire our command bridge into it.
#
# The material is a private repository issued to registered teams, so it is not vendored
# here. It is cloned into upstream/ (gitignored) and patched in place; upstream/ can be
# deleted and recreated at any time without touching our code.
#
# Requires git credentials with access to the contest organisation for the first checkout.
# The exact SDK revision used for the validated run is pinned below.  A prepared checkout can
# be reused without credentials inside Docker by setting S10_UPSTREAM_OFFLINE=1.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
UPSTREAM_URL="${S10_UPSTREAM_URL:-https://github.com/DeepRoboticsLab/goai_embodied_future_material.git}"
UPSTREAM_DIR="${REPO_ROOT}/upstream/goai_embodied_future_material"
UPSTREAM_REF="${S10_UPSTREAM_REF:-13dd084be6cb5e2514098bc87e586d00dfe580b2}"
OFFLINE="${S10_UPSTREAM_OFFLINE:-0}"

[[ "${OFFLINE}" == "0" || "${OFFLINE}" == "1" ]] || {
  echo "error: S10_UPSTREAM_OFFLINE must be 0 or 1" >&2
  exit 2
}

mkdir -p "${REPO_ROOT}/upstream"

if [[ -d "${UPSTREAM_DIR}/.git" ]]; then
  echo "==> Preparing existing checkout at ${UPSTREAM_DIR}"
  # Drop our patch before pulling so the merge is against a pristine tree.
  python3 "${REPO_ROOT}/robot/scripts/patch_upstream.py" --revert >/dev/null 2>&1 || true
  if [[ "${OFFLINE}" == "0" ]]; then
    git -c safe.directory="${UPSTREAM_DIR}" -C "${UPSTREAM_DIR}" fetch --depth 1 origin "${UPSTREAM_REF}"
  fi
else
  [[ "${OFFLINE}" == "0" ]] || {
    echo "error: offline mode requires an existing checkout at ${UPSTREAM_DIR}" >&2
    exit 2
  }
  echo "==> Cloning ${UPSTREAM_URL}"
  git clone --no-checkout "${UPSTREAM_URL}" "${UPSTREAM_DIR}"
fi

if ! git -c safe.directory="${UPSTREAM_DIR}" -C "${UPSTREAM_DIR}" cat-file -e "${UPSTREAM_REF}^{commit}" 2>/dev/null; then
  echo "error: pinned upstream commit ${UPSTREAM_REF} is unavailable" >&2
  echo "       rerun with network access or provide a checkout containing that commit" >&2
  exit 2
fi

# Do not force this checkout: unrelated local edits in the official SDK must make setup stop
# visibly instead of being discarded. patch_upstream.py --revert above touches only our known
# integration targets.
git -c safe.directory="${UPSTREAM_DIR}" -C "${UPSTREAM_DIR}" checkout --detach "${UPSTREAM_REF}"
ACTUAL_REF="$(git -c safe.directory="${UPSTREAM_DIR}" -C "${UPSTREAM_DIR}" rev-parse HEAD)"
[[ "${ACTUAL_REF}" == "${UPSTREAM_REF}" ]] || {
  echo "error: expected upstream ${UPSTREAM_REF}, got ${ACTUAL_REF}" >&2
  exit 2
}
echo "==> Upstream pinned at ${ACTUAL_REF}"

echo "==> Applying the ROS command bridge"
python3 "${REPO_ROOT}/robot/scripts/patch_upstream.py"

echo "==> Regenerating the racing course from the track overlay"
python3 "${REPO_ROOT}/robot/scripts/extract_waypoints.py"

cat <<'EOF'

Upstream is ready. Next:

    scripts/build.sh      # build the SDK and our packages
    scripts/run_race.sh   # simulator + policy + autonomous follower

EOF
