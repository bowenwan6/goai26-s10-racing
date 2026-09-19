#!/usr/bin/env bash
# Undo agx_ptp_setup.sh: stop PTP following, return to internet NTP (run with sudo).
set -uo pipefail
[ "$(id -u)" = 0 ] || { echo "run with sudo" >&2; exit 1; }
systemctl disable --now s10-phc2sys.service s10-ptp4l.service
rm -f /etc/systemd/system/s10-phc2sys.service /etc/systemd/system/s10-ptp4l.service /etc/linuxptp/s10-agx-ptp4l.conf
systemctl daemon-reload
timedatectl set-ntp true
echo "PTP following removed; internet NTP re-enabled (linuxptp package left installed)"
