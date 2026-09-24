#!/usr/bin/env bash
# Root step 2 on the 48 AGX (run with sudo from this directory):
#   switch compose to the ARM image, install the vendor license file.
set -euo pipefail
[ "$(id -u)" = 0 ] || { echo "run with sudo" >&2; exit 1; }
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BK="/var/backups/s10-docker-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$BK"
[ -e /opt/data/compose/docker-compose.yml ] && cp -a /opt/data/compose/docker-compose.yml "$BK/docker-compose.yml.before"
[ -e /opt/data/config/ssd_whitelist.conf.hash ] && cp -a /opt/data/config/ssd_whitelist.conf.hash "$BK/ssd_whitelist.conf.hash.before"
install -m 644 "$HERE/docker-compose.yml" /opt/data/compose/docker-compose.yml
install -m 644 "$HERE/ssd_whitelist.conf.hash" /opt/data/config/ssd_whitelist.conf.hash
echo "installed compose (ARM image) and license file; backups in $BK"
grep -n 'image:' /opt/data/compose/docker-compose.yml
sha256sum /opt/data/config/ssd_whitelist.conf.hash
