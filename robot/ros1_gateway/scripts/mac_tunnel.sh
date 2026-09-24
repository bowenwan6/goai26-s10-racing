#!/usr/bin/env bash
# Keep the Mac -> AGX tunnel open (reconnects after AGX reboots / network changes):
#   http://localhost:18080/  采集助手      http://localhost:18000/  x_nav
#   bash scripts/mac_tunnel.sh &      stop: pkill -f mac_tunnel.sh; pkill -f "L 18080:10.21.33.102"
AGX="${S10_AGX_SSH:-s10-48-remote}"
while true; do
  ssh -N -o ExitOnForwardFailure=yes -o ServerAliveInterval=10 -o ServerAliveCountMax=3 -o ConnectTimeout=15 \
    -L 18080:10.21.33.102:8080 -L 18000:127.0.0.1:8000 -L 9000:127.0.0.1:9000 -L 8765:127.0.0.1:8765 "$AGX"
  sleep 5
done
