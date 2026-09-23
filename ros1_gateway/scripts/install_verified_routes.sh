#!/usr/bin/env bash
# Replace the v7_o routes on the AGX with the ones that passed tools/verify_route_clearance.py (2026-09-21).
# The three earlier v7_o routes fail that check (robot body on map obstacles) and are moved out of ~/routes.
set -eo pipefail
AGX="${S10_AGX_SSH:-s10-48-remote}"
SRC="<workspace>/s10-real-readiness/field_data/20260920-v7_o/routes"
for r in v7_o-teachline v7_o-line-short; do grep -q '"hard_violating_cells": 0' "$SRC/$r/clearance.json" || { echo "$r has no clean clearance.json: not installing"; exit 1; }; done
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
rsync -a "$HERE/nav" "$HERE/tools" "$HERE/tests" "$HERE/docs" "$HERE/scripts" "$AGX:ros1_gateway/"
ssh "$AGX" 'mkdir -p ~/routes_unverified; for r in 20260920-223720-v7_o-short 20260921-v7_o-line-short 20260921-v7_o-line-full; do [ -d ~/routes/$r ] && mv ~/routes/$r ~/routes_unverified/ && echo "withdrawn: $r"; done; rm -rf ~/ros1_gateway/run/web-route; true'
rsync -a --delete "$SRC/v7_o-teachline/" "$AGX:routes/20260921-v7_o-safe-full/"
rsync -a --delete "$SRC/v7_o-line-short/" "$AGX:routes/20260921-v7_o-safe-short/"
ssh "$AGX" 'cd ~/ros1_gateway && for r in 20260921-v7_o-safe-short 20260921-v7_o-safe-full; do python3 tests/nav/sim_full_route.py ~/routes/$r 0.8 0.5 | tail -1; done; ls ~/routes'
