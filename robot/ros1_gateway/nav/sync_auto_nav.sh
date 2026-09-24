#!/usr/bin/env bash
# Copy the pure-Python navigation core (s10_auto_nav minus its ROS 2 nodes, plus the
# real_transfer geometry helpers) from a goai26-s10-racing checkout into nav/ so the
# AGX runs exactly one recorded commit of it under ROS 1.
#   bash nav/sync_auto_nav.sh /path/to/wt-rl-nav
set -eo pipefail
SRC="${1:?path to the goai26-s10-racing checkout (rl/maneuver-router)}"
DST="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG="$SRC/src/s10_auto_nav/s10_auto_nav"
[ -d "$PKG" ] || { echo "no s10_auto_nav package under $SRC" >&2; exit 2; }
rm -rf "$DST/s10_auto_nav"
mkdir -p "$DST/s10_auto_nav"
# ROS 2 nodes are left out (they import rclpy); everything else is numpy/scipy only.
( cd "$PKG" && find . -name '*.py' -o -name '*.json' -o -name '*.yaml' | grep -v __pycache__ \
  | grep -v -E '/(follower_node|pitfail_recorder|rl_nav_node|segment_recorder|strategy_router_node)\.py$' \
  | cpio -pdm --quiet "$DST/s10_auto_nav" )
cp "$SRC/real_transfer/geometry.py" "$DST/geometry.py"
{
  echo "source=$SRC"
  echo "commit=$(git -C "$SRC" rev-parse HEAD)"
  echo "branch=$(git -C "$SRC" branch --show-current)"
  echo "synced=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
} > "$DST/COMMIT"
if grep -rl "rclpy\|rcl_interfaces" "$DST/s10_auto_nav" >/dev/null; then
  echo "ERROR: ROS 2 imports left in the copy:" >&2; grep -rl "rclpy" "$DST/s10_auto_nav" >&2; exit 1
fi
cat "$DST/COMMIT"; find "$DST/s10_auto_nav" -name '*.py' | wc -l | xargs echo "python files:"
