#!/usr/bin/env bash
# One-time root setup on the 48 AGX for the vendor x_nav container.
# Run as the admin account (ysc) with sudo, from the directory holding this file:
#   sudo bash agx_root_setup.sh
#
# 1. Backs up firewall/forwarding state.
# 2. Writes /etc/docker/daemon.json BEFORE installing Docker so Docker never
#    rewrites iptables (the AGX forwards traffic; x_nav uses host networking).
# 3. Installs Ubuntu's docker.io + docker-compose-v2, then checks iptables are unchanged.
# 4. Creates /opt/data/{nav_map,config,compose} and installs the vendor files
#    (never overwrites an existing file; existing ones are backed up and left).
# It does NOT start any container, add users to the docker group or enable autostart
# of x_nav (the compose file uses restart: "no").
set -euo pipefail
[ "$(id -u)" = 0 ] || { echo "run with sudo" >&2; exit 1; }
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STAMP="$(date +%Y%m%d-%H%M%S)"
BK="/var/backups/s10-docker-$STAMP"
mkdir -p "$BK"
iptables-save > "$BK/iptables-before.txt" 2>/dev/null || true
ip6tables-save > "$BK/ip6tables-before.txt" 2>/dev/null || true
sysctl -n net.ipv4.ip_forward > "$BK/ip_forward-before.txt"
echo "backups: $BK"

mkdir -p /etc/docker
if [ -e /etc/docker/daemon.json ]; then
  cp -a /etc/docker/daemon.json "$BK/daemon.json.before"
  echo "NOTE: /etc/docker/daemon.json already exists, left unchanged:"; cat /etc/docker/daemon.json
else
  printf '{\n  "iptables": false,\n  "ip6tables": false\n}\n' > /etc/docker/daemon.json
  echo "wrote /etc/docker/daemon.json (iptables management off)"
fi

apt-get update || echo "WARNING: apt-get update reported errors; installing from current package lists"
DEBIAN_FRONTEND=noninteractive apt-get install -y docker.io docker-compose-v2
systemctl is-active --quiet docker || systemctl start docker
docker --version
docker compose version

iptables-save > "$BK/iptables-after.txt" 2>/dev/null || true
sysctl -n net.ipv4.ip_forward > "$BK/ip_forward-after.txt"
if diff -q <(grep -v '^#' "$BK/iptables-before.txt") <(grep -v '^#' "$BK/iptables-after.txt") >/dev/null; then
  echo "iptables rules unchanged"
else
  echo "WARNING: iptables rules changed; diff:"; diff <(grep -v '^#' "$BK/iptables-before.txt") <(grep -v '^#' "$BK/iptables-after.txt") || true
fi
diff -q "$BK/ip_forward-before.txt" "$BK/ip_forward-after.txt" >/dev/null && echo "ip_forward unchanged ($(cat "$BK/ip_forward-after.txt"))"

mkdir -p /opt/data/nav_map /opt/data/config /opt/data/compose
place() {
  local src="$1" dst="$2"
  if [ -e "$dst" ]; then
    cp -a "$dst" "$BK/$(basename "$dst").existing"
    if cmp -s "$src" "$dst"; then echo "$dst already identical"; else echo "NOTE: $dst exists and differs; left unchanged (copy in $BK)"; fi
  else
    install -m 644 "$src" "$dst"; echo "installed $dst"
  fi
}
place "$HERE/x_nav.yaml" /opt/data/nav_map/x_nav.yaml
place "$HERE/docker-compose.yml" /opt/data/compose/docker-compose.yml
echo "ROOT_SETUP_DONE"
