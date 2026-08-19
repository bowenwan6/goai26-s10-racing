#!/usr/bin/env bash
# Create the exact code ZIP submitted to GOAI from a clean, committed main revision.
# Logs, videos, upstream SDK material and build output are ignored by Git and therefore cannot
# enter this archive. The output directory should live outside the repository.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTPUT_DIR="${1:-}"

if [[ -z "${OUTPUT_DIR}" ]]; then
  echo "usage: scripts/package_submission.sh /absolute/output/directory" >&2
  exit 2
fi
mkdir -p "${OUTPUT_DIR}"
OUTPUT_DIR="$(cd "${OUTPUT_DIR}" && pwd)"

case "${OUTPUT_DIR}/" in
  "${REPO_ROOT}/"*)
    echo "error: output directory must be outside the competition repository" >&2
    exit 1
    ;;
esac

cd "${REPO_ROOT}"

if [[ -n "$(git status --porcelain --untracked-files=no)" ]]; then
  echo "error: tracked files are dirty; commit or restore them before packaging" >&2
  exit 1
fi

BRANCH="$(git branch --show-current)"
if [[ "${BRANCH}" != "main" && "${S10_ALLOW_NON_MAIN_PACKAGE:-0}" != "1" ]]; then
  echo "error: submission packages must be made from main (current: ${BRANCH})" >&2
  echo "       set S10_ALLOW_NON_MAIN_PACKAGE=1 only for a diagnostic archive" >&2
  exit 1
fi

FULL_COMMIT="$(git rev-parse HEAD)"
SHORT_COMMIT="$(git rev-parse --short=12 HEAD)"
PREFIX="goai26-s10-racing/"
ARCHIVE="${OUTPUT_DIR}/goai26-track4-challenge2-ver1-${SHORT_COMMIT}.zip"
MANIFEST="${ARCHIVE%.zip}.manifest.txt"
CHECKSUM="${ARCHIVE}.sha256"

for forbidden in '*.mp4' '*.log' 'results/*' 'upstream/*' 'build/*' 'install/*'; do
  if git ls-files "${forbidden}" | grep -q .; then
    echo "error: forbidden generated/private material is tracked: ${forbidden}" >&2
    exit 1
  fi
done

git archive --format=zip --prefix="${PREFIX}" --output="${ARCHIVE}" HEAD
unzip -tq "${ARCHIVE}" >/dev/null

CONTENTS="$(mktemp)"
trap 'rm -f "${CONTENTS}"' EXIT
unzip -Z1 "${ARCHIVE}" >"${CONTENTS}"
required=(
  README.md
  LICENSE
  compose.yaml
  docker/Dockerfile
  docker/requirements.lock
  docs/SUBMISSION.md
  docs/TECHNICAL_DESIGN.md
  docs/THIRD_PARTY.md
  docs/OPEN_SOURCE_PLAN.md
  scripts/setup_upstream.sh
  scripts/build.sh
  scripts/verify_install.sh
  scripts/run_race.sh
  src/s10_bringup/config/nav.yaml
  src/s10_bringup/config/strategy_gate16.yaml
  src/s10_bringup/rviz/perception.rviz
  policy/gate16/policy.onnx
  policy/gate16/climb_residual.onnx
  policy/gate16/climb_policy_manifest.json
)
for path in "${required[@]}"; do
  if ! grep -Fxq "${PREFIX}${path}" "${CONTENTS}"; then
    echo "error: required submission file missing from archive: ${path}" >&2
    exit 1
  fi
done

if command -v sha256sum >/dev/null 2>&1; then
  ARCHIVE_SHA="$(sha256sum "${ARCHIVE}" | awk '{print $1}')"
elif command -v shasum >/dev/null 2>&1; then
  ARCHIVE_SHA="$(shasum -a 256 "${ARCHIVE}" | awk '{print $1}')"
else
  echo "error: sha256sum or shasum is required" >&2
  exit 1
fi

printf '%s  %s\n' "${ARCHIVE_SHA}" "$(basename "${ARCHIVE}")" >"${CHECKSUM}"
{
  printf 'project=Terrain-Aware Autonomous Navigation for the Lynx S10\n'
  printf 'track=GOAI 2026 Track 4 Challenge 2\n'
  printf 'git_branch=%s\n' "${BRANCH}"
  printf 'git_commit=%s\n' "${FULL_COMMIT}"
  printf 'archive=%s\n' "$(basename "${ARCHIVE}")"
  printf 'archive_sha256=%s\n' "${ARCHIVE_SHA}"
  printf 'archive_bytes=%s\n' "$(wc -c <"${ARCHIVE}" | tr -d ' ')"
  printf 'official_self_test_seconds=436.058\n'
  printf 'self_test_video=wp0_to_wp32_seed6_720p_official.mp4\n'
  printf 'self_test_video_sha256=291cbfdbc6f6b0100f42d304ffdd7b976d2d15557ae956cbfbfbdf9acaabba81\n'
  printf 'created_utc=%s\n' "$(date -u +'%Y-%m-%dT%H:%M:%SZ')"
} >"${MANIFEST}"

echo "ready: ${ARCHIVE}"
echo "sha256: ${ARCHIVE_SHA}"
echo "manifest: ${MANIFEST}"
