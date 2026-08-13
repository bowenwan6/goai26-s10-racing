#!/usr/bin/env bash
#
# Fetch the contest material and wire our command bridge into it.
#
# The material is a private repository issued to registered teams, so it is not vendored
# here. It is cloned into upstream/ (gitignored) and patched in place; upstream/ can be
# deleted and recreated at any time without touching our code.
#
# Requires git credentials with access to the contest organisation.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UPSTREAM_URL="${S10_UPSTREAM_URL:-https://github.com/DeepRoboticsLab/goai_embodied_future_material.git}"
UPSTREAM_DIR="${REPO_ROOT}/upstream/goai_embodied_future_material"

mkdir -p "${REPO_ROOT}/upstream"

if [[ -d "${UPSTREAM_DIR}/.git" ]]; then
  echo "==> Updating existing checkout at ${UPSTREAM_DIR}"
  # Drop our patch before pulling so the merge is against a pristine tree.
  python3 "${REPO_ROOT}/scripts/patch_upstream.py" --revert >/dev/null 2>&1 || true
  git -C "${UPSTREAM_DIR}" pull --ff-only
else
  echo "==> Cloning ${UPSTREAM_URL}"
  git clone --depth 1 "${UPSTREAM_URL}" "${UPSTREAM_DIR}"
fi

echo "==> Applying the ROS command bridge"
python3 "${REPO_ROOT}/scripts/patch_upstream.py"

echo "==> Regenerating the racing course from the track overlay"
python3 "${REPO_ROOT}/scripts/extract_waypoints.py"

cat <<'EOF'

Upstream is ready. Next:

    scripts/build.sh      # build the SDK and our packages
    scripts/run_race.sh   # simulator + policy + autonomous follower

EOF
