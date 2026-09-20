#!/usr/bin/env bash
# Deploy the navigation stack to the AGX and run its offline checks there (no robot motion):
#   bash scripts/deploy_nav.sh            # sync + build control + python deps + tests
#   bash scripts/deploy_nav.sh --no-test
# Runs on the Mac. Syncs nav/, tools/, config/, tests/, src/, scripts/ to ~/ros1_gateway on the
# AGX, rebuilds s10_ros1_control, installs scipy/yaml for the golai user if missing (pip --user,
# no sudo), then runs the control mock test on a private ROS master/domain and the nav ROS 1
# test. The running gateway/roscore on 11311 are untouched.
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
AGX="${S10_AGX_SSH:-s10-48-remote}"
TEST=1; [ "$1" = --no-test ] && TEST=0
echo "== sync to $AGX:~/ros1_gateway"
# No --delete on src/: the AGX also keeps src/ros1_bridge, which is not in this tree.
rsync -a --exclude '__pycache__' --exclude 'run/' --exclude 'logs/' --exclude 'ws/' --exclude 'ros1/' \
  --exclude '*.bag' --exclude 'vendor/x_nav/ssd_whitelist.conf.hash' \
  "$HERE/nav" "$HERE/tools" "$HERE/config" "$HERE/tests" "$HERE/src" "$HERE/scripts" "$HERE/tap" "$HERE/README_ZH.md" \
  "$AGX:ros1_gateway/"
echo "== python deps (golai user)"
ssh "$AGX" 'python3 -c "import scipy, yaml, numpy; print(\"scipy\", scipy.__version__, \"numpy\", numpy.__version__)" 2>/dev/null \
  || { echo "installing scipy/pyyaml with pip --user"; python3 -m pip install --user --break-system-packages scipy pyyaml 2>&1 | tail -2; \
       python3 -c "import scipy, yaml; print(\"scipy\", scipy.__version__)"; }'
echo "== build s10_ros1_control"
ssh "$AGX" 'cd ~/ros1_gateway && source ros1/ros1_env.sh && source /opt/ros/jazzy/setup.bash && source ws/install/setup.bash \
  && cd ws && colcon build --base-paths ~/ros1_gateway/src --packages-select s10_ros1_control \
     --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF 2>&1 | tail -3'
[ "$TEST" = 1 ] || exit 0
echo "== control mock test on the AGX (private master :11399, domain 78)"
ssh "$AGX" 'cd ~/ros1_gateway && ROS1_SETUP=$HOME/ros1_gateway/ros1/ros1_env.sh WS=$HOME/ros1_gateway/ws/install SRC=$HOME/ros1_gateway \
  OUT_BASE=/tmp/s10_ctl_test ROS1_PORT=11399 FASTRTPS_DEFAULT_PROFILES_FILE= bash tests/control/run_control_test.sh enabled 2>&1 | grep -E "^(PASS|FAIL|CONTROL_TEST)" | tail -40'
echo "== nav ROS 1 test on the AGX (private master :11312)"
ssh "$AGX" 'cd ~/ros1_gateway && ROS1_SETUP=$HOME/ros1_gateway/ros1/ros1_env.sh SRC=$HOME/ros1_gateway OUT_BASE=/tmp/s10_nav_test \
  bash tests/nav/run_nav_ros1_test.sh 2>&1 | tail -4'
