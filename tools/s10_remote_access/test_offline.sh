#!/usr/bin/env bash
# Offline tests only: timeout is replaced below; no SSH or external probes execute.
set -uo pipefail
TEST_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$TEST_DIR/check_agx.sh"

assert_contains() {
    [[ "$1" == *"$2"* ]] || { printf 'FAIL: missing %s\n' "$2" >&2; exit 1; }
}
assert_not_contains() {
    [[ "$1" != *"$2"* ]] || { printf 'FAIL: unexpected %s\n' "$2" >&2; exit 1; }
}

# These real invocations only parse/help/dry-run; no confirmation flag is supplied.
for script in check_agx.sh collect_from_mac.sh test_offline.sh; do
    bash -n "$TEST_DIR/$script" || exit 1
done
output=$(bash "$TEST_DIR/collect_from_mac.sh") || exit 1
assert_contains "$output" 'DRY RUN: no SSH connection'
output=$(bash "$TEST_DIR/check_agx.sh" --help) || exit 1
assert_contains "$output" 'Exit 0 = collection finished'
if bash "$TEST_DIR/check_agx.sh" --unexpected >/dev/null 2>&1; then
    printf 'FAIL: bad collector option accepted\n' >&2; exit 1
fi
if bash "$TEST_DIR/collect_from_mac.sh" --identity >/dev/null 2>&1; then
    printf 'FAIL: missing identity accepted\n' >&2; exit 1
fi
if bash "$TEST_DIR/collect_from_mac.sh" --host-key-name '-oBad' >/dev/null 2>&1; then
    printf 'FAIL: invalid host-key name accepted\n' >&2; exit 1
fi
output=$(bash "$TEST_DIR/collect_from_mac.sh" --host-key-name 10.21.33.102) || exit 1
assert_contains "$output" 'DRY RUN: no SSH connection'
printf 'PASS: syntax, help, invalid arguments and local dry-run\n'

TEST_PLATFORM=Linux
TEST_MISSING=''
TEST_FAIL=''
uname() { printf '%s\n' "$TEST_PLATFORM"; }
# Fake command discovery for optional Linux utilities. No actual invocation follows.
command() {
    if [[ "${1:-}" == -v ]]; then
        [[ "${2:-}" != "$TEST_MISSING" ]]
    else
        builtin command "$@"
    fi
}
tailscale() {
    printf '%s\n' '{"BackendState":"Running","TUN":true,"AuthURL":"SECRET_LOGIN_URL","Peer":{"other":"SECRET_PEER"},"User":{"other":"SECRET_USER"},"Self":{"HostName":"mock-048","Online":true,"TailscaleIPs":["100.100.1.1"],"KeyExpiry":null}}'
}
export -f tailscale
timeout() {
    shift 2 # --kill-after and duration
    if [[ "$1" == "$TEST_FAIL" ]]; then
        printf 'mock command timed out\n'; return 124
    fi
    if [[ "$1" == bash ]]; then
        # The single embedded status filter runs for real with the fake tailscale function.
        /bin/bash "${@:2}"
    else
        printf 'MOCK:'
        printf ' <%s>' "$@"
        printf '\n'
    fi
}

output=$(main --no-probes) || exit 1
assert_contains "$output" 'S10_AGX_REMOTE_ACCESS_REPORT_END'
assert_contains "$output" 'Outbound DNS/ICMP/HTTPS/netcheck probes disabled.'
assert_contains "$output" '"HostName": "mock-048"'
assert_not_contains "$output" 'SECRET_LOGIN_URL'
assert_not_contains "$output" 'SECRET_PEER'
assert_not_contains "$output" 'SECRET_USER'
assert_not_contains "$output" 'MOCK: <curl>'
assert_not_contains "$output" 'MOCK: <ping>'
assert_not_contains "$output" 'MOCK: <getent>'
assert_not_contains "$output" 'MOCK: <tailscale> <netcheck>'
printf 'PASS: no-probes and Tailscale state privacy filter\n'

output=$(main) || exit 1
assert_contains "$output" 'MOCK: <curl> <-q> <--noproxy> <*>'
assert_contains "$output" '<--head> <--output> </dev/null>'
assert_contains "$output" 'MOCK: <tailscale> <netcheck>'
assert_contains "$output" 'S10_AGX_REMOTE_ACCESS_REPORT_END'
assert_not_contains "$output" '<--insecure>'
assert_not_contains "$output" '<--location>'
assert_not_contains "$output" '<--show-secrets>'
printf 'PASS: bounded probe invocation options, no TLS bypass or redirects\n'

TEST_FAIL=ip
output=$(main --no-probes) || exit 1
assert_contains "$output" '[WARN] command_exit=124'
assert_contains "$output" 'S10_AGX_REMOTE_ACCESS_REPORT_END'
TEST_FAIL=''
TEST_MISSING=tailscale
output=$(main --no-probes) || exit 1
assert_contains "$output" 'Tailscale is not installed/in PATH'
TEST_MISSING=nmcli
output=$(main --no-probes) || exit 1
assert_contains "$output" '[SKIP] Command unavailable: nmcli'
printf 'PASS: timeout and missing-tool failures do not abort collection\n'

TEST_MISSING=timeout
main --no-probes >/dev/null 2>&1
[[ $? == 3 ]] || exit 1
TEST_MISSING=''
TEST_PLATFORM=Darwin
main --no-probes >/dev/null 2>&1
[[ $? == 3 ]] || exit 1
TEST_PLATFORM=Linux
START_SECONDS=$((SECONDS - 181))
BUDGET_EXHAUSTED=0
check 'budget test' 3 hostname >/dev/null
[[ "$BUDGET_EXHAUSTED" == 1 ]] || exit 1
printf 'PASS: platform prerequisites and total-budget exhaustion\n'
printf 'All offline test groups passed. No robot or network access occurred.\n'
