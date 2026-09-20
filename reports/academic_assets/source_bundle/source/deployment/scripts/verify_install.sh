#!/usr/bin/env bash
# Verify that a checkout contains the exact policy assets, upstream revision and built ROS
# packages required by the competition entry point. Run inside the supplied container after
# scripts/setup_upstream.sh and scripts/build.sh.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UPSTREAM_DIR="${REPO_ROOT}/upstream/goai_embodied_future_material"
UPSTREAM_REF="${S10_UPSTREAM_REF:-13dd084be6cb5e2514098bc87e586d00dfe580b2}"
S10_INSTALL_BASE="${S10_INSTALL_BASE:-${REPO_ROOT}/install}"

[[ -d "${UPSTREAM_DIR}/.git" ]] || {
  echo "error: upstream checkout is missing; run scripts/setup_upstream.sh" >&2
  exit 1
}
ACTUAL_REF="$(git -C "${UPSTREAM_DIR}" rev-parse HEAD)"
[[ "${ACTUAL_REF}" == "${UPSTREAM_REF}" ]] || {
  echo "error: upstream revision ${ACTUAL_REF}; expected ${UPSTREAM_REF}" >&2
  exit 1
}

python3 "${REPO_ROOT}/scripts/patch_upstream.py" --check
python3 "${REPO_ROOT}/scripts/extract_waypoints.py" --check

python3 - "${REPO_ROOT}" <<'PY'
import hashlib
import json
import sys
from pathlib import Path

root = Path(sys.argv[1])
bundles = (
    (root / "policy/gate16/climb_policy_manifest.json", ("base_onnx", "base_sha256")),
    (root / "policy/gate16/climb_policy_manifest.json", ("residual_onnx", "residual_sha256")),
    (root / "policy/stairs57/policy_manifest.json", ("onnx", "sha256")),
)
for manifest_path, (asset_key, checksum_key) in bundles:
    manifest = json.loads(manifest_path.read_text())
    asset = manifest_path.parent / manifest[asset_key]
    actual = hashlib.sha256(asset.read_bytes()).hexdigest()
    expected = manifest[checksum_key]
    if actual != expected:
        raise SystemExit(f"error: checksum mismatch for {asset}: {actual} != {expected}")
    print(f"ok: {asset.relative_to(root)} sha256={actual}")
PY

[[ -f "${S10_INSTALL_BASE}/setup.bash" ]] || {
  echo "error: build overlay is missing at ${S10_INSTALL_BASE}; run scripts/build.sh" >&2
  exit 1
}

set +u
# shellcheck disable=SC1090
source "${S10_INSTALL_BASE}/setup.bash"
set -u

for package in s10_sdk_deploy s10_perception s10_auto_nav s10_bringup; do
  ros2 pkg prefix "${package}" >/dev/null
  echo "ok: ROS package ${package}"
done

echo "ready: verified competition environment"
