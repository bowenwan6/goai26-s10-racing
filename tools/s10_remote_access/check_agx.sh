#!/usr/bin/env bash
# Read-only Linux/AGX inventory. Designed to be streamed into bash over SSH.
# No sudo, installs, configuration writes, service changes, or motion commands.
set -uo pipefail
export LC_ALL=C SYSTEMD_PAGER=cat PAGER=cat

usage() {
    printf '%s\n' 'Usage: bash check_agx.sh [--no-probes] [--help]' \
        'Default: local inventory plus bounded DNS/ICMP/HTTPS/Tailscale probes.' \
        '--no-probes: local inventory only; no explicit outbound probes.' \
        'Report goes to stdout. No report file is written on the robot.' \
        'Exit 0 = collection finished, NOT proof that remote access works.' \
        'Exit 2 = bad arguments; 3 = missing platform prerequisite; 4 = budget exhausted.'
}

# Every optional command has a timeout. Failed/missing commands do not abort collection.
check() {
    local title="$1" limit="$2" result rc remaining
    shift 2
    printf '\n### %s\n' "$title"
    remaining=$((180 - SECONDS + START_SECONDS))
    if ((remaining <= 0)); then
        printf '[SKIP] Total collection budget exhausted.\n'
        BUDGET_EXHAUSTED=1
        return 0
    fi
    if ! command -v "$1" >/dev/null 2>&1; then
        printf '[SKIP] Command unavailable: %s\n' "$1"
        SKIPPED=$((SKIPPED + 1))
        return 0
    fi
    ((limit > remaining)) && limit=$remaining
    result=$(timeout --kill-after=1s "${limit}s" "$@" 2>&1)
    rc=$?
    # Consume all captured output, but limit printed lines/line length and control codes.
    printf '%s\n' "$result" | tr -d '\000-\010\013-\037\177' | \
        awk 'NR <= 100 { print substr($0, 1, 1000) } NR == 101 { print "[output truncated]" }'
    if ((rc == 0)); then
        printf '[OK] command_exit=0 (interpret the evidence above)\n'
        PASSED=$((PASSED + 1))
    else
        printf '[WARN] command_exit=%s (124/137 usually indicate timeout)\n' "$rc"
        WARNINGS=$((WARNINGS + 1))
    fi
    return 0
}

https_check() {
    local label="$1" url="$2"
    # -q MUST be first: ignore ~/.curlrc. Never output response bodies, cookies or headers.
    # Keep TLS verification; bypass proxy environment variables; do not follow redirects.
    check "$label" 12 curl -q --noproxy '*' --silent --show-error \
        --connect-timeout 4 --max-time 10 --head --output /dev/null \
        --write-out 'http=%{http_code} remote_ip=%{remote_ip} dns_s=%{time_namelookup} connect_s=%{time_connect} tls_s=%{time_appconnect} total_s=%{time_total}\n' \
        "$url"
}

main() {
    local do_probes=1 arg net_path iface proxy_var tool
    for arg in "$@"; do
        case "$arg" in
            --help|-h) usage; return 0 ;;
            --no-probes) do_probes=0 ;;
            *) printf 'Unknown option: %s\n' "$arg" >&2; return 2 ;;
        esac
    done
    if [[ $(uname -s) != Linux ]]; then
        printf 'Run this collector on the Linux AGX, not on the Mac.\n' >&2
        return 3
    fi
    if ! command -v timeout >/dev/null 2>&1; then
        printf 'GNU timeout is required; refusing unbounded collection. Nothing installed.\n' >&2
        return 3
    fi
    PASSED=0 WARNINGS=0 SKIPPED=0 BUDGET_EXHAUSTED=0 START_SECONDS=$SECONDS
    printf '%s\n' 'S10_AGX_REMOTE_ACCESS_REPORT_V1' \
        'Scope: current AGX only. No login to 103/106. No network configuration changes.' \
        'Sensitive network metadata (SSID/IP/hostname) is included; keep report private.' \
        'No passwords/private keys/auth keys, saved Wi-Fi profiles or raw environment dumps.' \
        'Missing permissions mean UNKNOWN, not an absent firewall or a healthy interface.'

    check 'UTC time' 3 date -u '+%Y-%m-%dT%H:%M:%SZ'
    check 'Host identity (must still verify physical robot 048 and trusted SSH host key)' 3 hostname
    check 'Kernel and architecture' 3 uname -srmo
    check 'OS release: selected fields only' 3 awk \
        '/^(NAME|VERSION_ID|VERSION_CODENAME|ID|PRETTY_NAME)=/' /etc/os-release
    check 'Account and groups (administrator capability not tested; no sudo)' 3 id
    check 'Uptime' 3 uptime
    check 'Root filesystem capacity' 3 df -h /
    check 'Time synchronisation state (no time changes)' 5 timedatectl show \
        -p NTPSynchronized -p NTP -p TimeUSec

    check 'Addresses' 5 ip -brief address show
    check 'IPv4 routes: all tables' 5 ip -4 route show table all
    check 'IPv6 routes: all tables' 5 ip -6 route show table all
    check 'Routing policy' 5 ip rule show
    check 'External IPv4 route choice (lookup only, not a packet)' 3 ip -4 route get 1.1.1.1
    check 'Hotspot return route (lookup only)' 3 ip -4 route get 10.21.41.19
    check 'Link counters snapshot A' 5 ip -s link show
    check 'DNS resolver state' 5 resolvectl status
    check 'Fallback resolver nameservers only' 3 awk '$1 == "nameserver" { print }' /etc/resolv.conf
    check 'NetworkManager device state' 5 nmcli --wait 3 \
        --fields DEVICE,TYPE,STATE,CONNECTION device status
    check 'Active NetworkManager connections only (no saved secrets)' 5 nmcli --wait 3 \
        --fields NAME,TYPE,DEVICE connection show --active
    check 'Current device addresses/gateways/DNS (no Wi-Fi scan)' 5 nmcli --wait 3 \
        --fields GENERAL.DEVICE,GENERAL.TYPE,GENERAL.STATE,GENERAL.CONNECTION,IP4,IP6 device show
    check 'Wireless interface inventory (no scan)' 4 iw dev

    for net_path in /sys/class/net/*; do
        [[ -d "$net_path" ]] || continue
        iface=${net_path##*/}
        # Physical devices only. No ethtool tests, set operations or firmware reloads.
        if [[ -e "$net_path/device" ]]; then
            check "Driver path: $iface" 3 readlink -f "$net_path/device/driver"
            check "Driver/version: $iface" 4 ethtool -i "$iface"
            check "Link negotiation: $iface" 4 ethtool "$iface"
        fi
        if [[ -d "$net_path/wireless" || -e "$net_path/phy80211" ]]; then
            check "Connected Wi-Fi SSID/signal/rate: $iface" 4 iw dev "$iface" link
            check "Current Wi-Fi power saving: $iface" 4 iw dev "$iface" get power_save
        fi
    done

    check 'Only SSH/web TCP listeners (no process arguments)' 4 ss -H -ltn \
        '( sport = :22 or sport = :8080 )'
    check 'Relevant system services: state only' 5 systemctl show \
        -p Id -p LoadState -p ActiveState -p SubState -p UnitFileState \
        ssh.service ssh.socket tailscaled.service s10-agx-internet.service
    check 'Mapping web user service: state only' 4 systemctl --user show \
        -p Id -p LoadState -p ActiveState -p SubState -p UnitFileState s10-mapping-web.service
    check 'TUN device permissions (no device creation)' 3 ls -l /dev/net/tun
    check 'IPv4 forwarding flag (read only)' 3 cat /proc/sys/net/ipv4/ip_forward
    check 'nftables rules (current account only; no sudo)' 5 nft list ruleset
    check 'iptables filter rules (current account only; no sudo)' 5 iptables -S
    check 'iptables NAT rules (current account only; no sudo)' 5 iptables -t nat -S
    printf '\n### Existing SSH public-key setup\n'
    if [[ -s "${HOME}/.ssh/authorized_keys" ]]; then
        printf 'Current account authorized_keys exists and is nonempty; contents not read.\n'
    else
        printf 'No readable/nonempty authorized_keys confirmed at the default path.\n'
    fi
    printf 'Effective sshd authentication settings/Match rules and admin access: not verified.\n'
    printf '\n### Proxy environment: names only, never values\n'
    for proxy_var in http_proxy https_proxy all_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY; do
        [[ -n "${!proxy_var:-}" ]] && printf '%s is set; value omitted\n' "$proxy_var"
    done
    printf '\n### Available installation/diagnostic tools (nothing installed)\n'
    for tool in apt-get dpkg curl python3 sudo tailscale tailscaled; do
        command -v "$tool" || true
    done

    if command -v tailscale >/dev/null 2>&1; then
        check 'Installed Tailscale version' 4 tailscale version
        # Filter locally before any output: do not collect peers, user accounts or AuthURL.
        if command -v python3 >/dev/null 2>&1; then
            check 'Tailscale local-node state only' 8 bash -o pipefail -c '
                tailscale status --json 2>/dev/null | python3 -c '\''
import json, sys
try:
    data = json.load(sys.stdin)
    node = data.get("Self") or {}
    print(json.dumps({"BackendState": data.get("BackendState"),
                      "TUN": data.get("TUN"),
                      "Self": {k: node.get(k) for k in
                               ("HostName", "Online", "TailscaleIPs", "KeyExpiry")}},
                     ensure_ascii=True))
except (ValueError, TypeError, AttributeError):
    print("Tailscale status unavailable or unrecognised; raw output withheld.")
    sys.exit(1)
'\'''
        else
            printf '[SKIP] Tailscale state: python3 unavailable; raw status withheld.\n'
        fi
    else
        printf '\nTailscale is not installed/in PATH; no attempt to install or log in.\n'
    fi

    if ((do_probes)); then
        check 'DNS: ordinary HTTPS endpoint' 6 getent ahosts www.baidu.com
        check 'DNS: Tailscale login endpoint' 6 getent ahosts login.tailscale.com
        check 'External IP reachability (ICMP blocked does NOT prove offline)' 6 ping -n -c 2 -W 2 1.1.1.1
        https_check 'Ordinary HTTPS, sample 1' https://www.baidu.com/
        https_check 'Tailscale site HTTPS' https://tailscale.com/
        https_check 'Tailscale login HTTPS (not a login or control-channel test)' https://login.tailscale.com/
        https_check 'Tailscale package endpoint HTTPS (no download/install)' https://pkgs.tailscale.com/
        if command -v tailscale >/dev/null 2>&1; then
            check 'Tailscale NAT/UDP/relay reachability probes' 15 tailscale netcheck
        fi
        https_check 'Ordinary HTTPS, sample 2 (short snapshot, not a stability certificate)' https://www.baidu.com/
    else
        printf '\nOutbound DNS/ICMP/HTTPS/netcheck probes disabled.\n'
    fi
    check 'Link counters snapshot B' 5 ip -s link show
    printf '\nCollection complete: successful_commands=%s warnings=%s missing_commands=%s budget_exhausted=%s\n' \
        "$PASSED" "$WARNINGS" "$SKIPPED" "$BUDGET_EXHAUSTED"
    printf '%s\n' 'HTTP 3xx/4xx/5xx still need interpretation; command success is not service readiness.' \
        'No network/SSH/ROS settings changed. No software installed. No movement commands.' \
        'NOT VERIFIED: long-term stability, tailnet ACLs, cross-network SSH, or safe robot control.' \
        'S10_AGX_REMOTE_ACCESS_REPORT_END'
    ((BUDGET_EXHAUSTED == 0)) || return 4
    return 0
}

# Allow sourcing for offline tests; also run when delivered via SSH bash -s.
if [[ -z "${BASH_SOURCE[0]:-}" || "${BASH_SOURCE[0]}" == "$0" ]]; then
    main "$@"
fi
