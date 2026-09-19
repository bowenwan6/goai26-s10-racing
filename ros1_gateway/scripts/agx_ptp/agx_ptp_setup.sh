#!/usr/bin/env bash
# Make the AGX system clock follow the robot's PTP time (run with sudo on the AGX).
# Rollback: sudo bash agx_ptp_rollback.sh
set -euo pipefail
[ "$(id -u)" = 0 ] || { echo "run with sudo" >&2; exit 1; }
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "before: $(date -Is)  ntp=$(timedatectl show -p NTP --value)"
command -v ptp4l >/dev/null || { apt-get update || true; DEBIAN_FRONTEND=noninteractive apt-get install -y linuxptp; }
install -D -m 644 "$HERE/s10-agx-ptp4l.conf" /etc/linuxptp/s10-agx-ptp4l.conf
install -m 644 "$HERE/s10-ptp4l.service" "$HERE/s10-phc2sys.service" /etc/systemd/system/
timedatectl set-ntp false          # stop internet NTP so it does not fight PTP
systemctl daemon-reload
systemctl enable --now s10-ptp4l.service
sleep 5
systemctl enable --now s10-phc2sys.service
sleep 20
systemctl is-active s10-ptp4l s10-phc2sys | paste -sd" "
journalctl -u s10-phc2sys -n 5 --no-pager -o cat || true
echo "after: $(date -Is)"
echo PTP_SETUP_DONE
