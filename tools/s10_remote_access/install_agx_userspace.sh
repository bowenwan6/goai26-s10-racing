#!/usr/bin/env bash
# User-owned installation only; no sudo or network reconfiguration.
# Upload the reviewed service template first as the sole argument.
set -euo pipefail
umask 077
[[ $# == 1 && -f "$1" ]] || { printf 'Usage: bash install_agx_userspace.sh /path/to/reviewed.service\n' >&2; exit 2; }
[[ $(id -un) == golai && $(uname -m) == aarch64 ]] || { printf 'Unexpected robot account/architecture\n' >&2; exit 3; }
root=/home/golai/.local/lib/s10-tailscale
state=/home/golai/.local/state/s10-tailscale
unit=/home/golai/.config/systemd/user/s10-tailscale.service
archive=tailscale_1.102.4_arm64.tgz
expected=9dd1e6a592a014bbaea0103167ffe299adeda4ba14e078ce9c2895364f6c4c3f
[[ ! -e "$root" && ! -e "$state" && ! -e "$unit" ]] || {
    printf 'Existing S10 Tailscale files found; inspect instead of overwriting.\n' >&2; exit 4;
}
mkdir -p /home/golai/.local/lib /home/golai/.local/state /home/golai/.config/systemd/user
mkdir -m 700 "$root" "$state"
printf 'Downloading verified official ARM64 release...\n'
curl -q --noproxy '*' --fail --silent --show-error --connect-timeout 8 --max-time 180 \
    --output "$root/$archive" "https://pkgs.tailscale.com/stable/$archive"
printf '%s  %s\n' "$expected" "$root/$archive" | sha256sum --check -
tar -xzf "$root/$archive" --no-same-owner -C "$root"
"$root/tailscale_1.102.4_arm64/tailscale" version
install -m 600 "$1" "$unit"
systemd-analyze --user verify "$unit"
systemctl --user daemon-reload
systemctl --user enable --now s10-tailscale.service
printf 'Daemon installed; NOT logged in, not yet reachable over Tailscale.\n'
printf 'Rollback: systemctl --user disable --now s10-tailscale.service\n'
