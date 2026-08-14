#!/usr/bin/env bash
#
# Open a shell in the development container with the workspace mounted.
#
# The repository is bind-mounted rather than copied, so edits on the host take effect
# immediately and build artefacts persist between container runs.
#
#   docker/run.sh                    # interactive shell
#   docker/run.sh scripts/build.sh   # run one command and exit
#   REBUILD=1 docker/run.sh          # force an image rebuild first
#   CONTAINER_NAME=s10-race docker/run.sh scripts/run_race.sh   # replaces any prior run

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${IMAGE:-s10-racing:dev}"

if [[ "${REBUILD:-0}" == "1" ]] || ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  echo "==> Building ${IMAGE}"
  docker build -t "${IMAGE}" "${REPO_ROOT}/docker"
fi

# Allocate a TTY only when there is one to allocate, so the script also works from CI
# and from non-interactive tooling.
tty_flags=(-i)
[[ -t 0 && -t 1 ]] && tty_flags+=(-t)

# Build output goes to a named volume rather than into the bind-mounted repo. Bind mounts
# inherit the host filesystem's behaviour, and a checkout under iCloud Drive or Dropbox
# gets its build tree forked into conflicted copies ("install/drdds 2") mid-build. Sources
# stay bind-mounted so host edits still apply immediately.
BUILD_VOLUME="${BUILD_VOLUME:-s10-racing-build}"

# Naming the container makes a run replaceable. Killing the host-side `docker run` client
# does not stop the container it started, so an abandoned race keeps its simulator, policy
# and follower alive -- and because every stack shares --network host and one
# ROS_DOMAIN_ID, the orphan's follower goes on publishing /cmd_vel into the next run's
# simulator. Two stacks fighting over one robot cost an afternoon of debugging terrain that
# was never the problem. A fixed name plus this pre-emptive kill makes that unrepeatable;
# the default stays unique so ordinary shells do not evict each other.
CONTAINER_NAME="${CONTAINER_NAME:-s10-dev-$$}"
docker rm --force "${CONTAINER_NAME}" >/dev/null 2>&1 || true

# --network host keeps DDS discovery simple; without it the simulator and the controller
# land in different network namespaces and never find each other.
exec docker run --rm "${tty_flags[@]}" \
  --name "${CONTAINER_NAME}" \
  --network host \
  --volume "${REPO_ROOT}:/ws" \
  --volume "${BUILD_VOLUME}:/opt/s10-build" \
  --workdir /ws \
  --env "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-1}" \
  --env "S10_USE_VIEWER=${S10_USE_VIEWER:-0}" \
  --env "S10_BUILD_BASE=/opt/s10-build/build" \
  --env "S10_INSTALL_BASE=/opt/s10-build/install" \
  --env "S10_LOG_BASE=/opt/s10-build/log" \
  "${IMAGE}" \
  "${@:-/bin/bash}"
