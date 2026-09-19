#!/usr/bin/env bash
# Local runner: stream collector to AGX; keep report only on this computer.
set -euo pipefail
umask 077
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

usage() {
    printf '%s\n' 'Usage: bash collect_from_mac.sh [--confirm-048] [--no-probes] [--identity /path/to/key] [--host-key-name name]' \
        'Default is a dry run: prints the target and makes NO connection.' \
        '--confirm-048: operator has confirmed physical robot 048 and S10 PRO-048-5G Wi-Fi.' \
        'Target: golai@10.21.33.102; default trusted host-key alias: s10-48-agx.' \
        '--host-key-name: use an already verified known_hosts name, e.g. 10.21.33.102.' \
        'Unknown/changed SSH host keys are rejected, not auto-accepted.'
}

confirmed=0
host_key_name=s10-48-agx
probe_args=()
identity_args=()
while (($#)); do
    case "$1" in
        --help|-h) usage; exit 0 ;;
        --confirm-048) confirmed=1; shift ;;
        --no-probes) probe_args=(--no-probes); shift ;;
        --host-key-name)
            [[ $# -ge 2 && "$2" =~ ^[a-zA-Z0-9][a-zA-Z0-9._-]*$ ]] || {
                printf 'A valid, already trusted host-key name is required.\n' >&2; exit 2;
            }
            host_key_name="$2"; shift 2 ;;
        --identity)
            [[ $# -ge 2 && -f "$2" ]] || { printf 'A local SSH key file is required.\n' >&2; exit 2; }
            identity_args=(-o IdentitiesOnly=yes -i "$2"); shift 2 ;;
        *) printf 'Unknown option: %s\n' "$1" >&2; exit 2 ;;
    esac
done
if ((confirmed == 0)); then
    usage
    printf '\nDRY RUN: no SSH connection, no report directory created.\n'
    exit 0
fi

# Existing ignored artifact area. mktemp prevents overwriting an older/partial report.
report_root="$SCRIPT_DIR/../../artifacts/s10-remote-access"
mkdir -p "$report_root"
report_dir=$(mktemp -d "$report_root/$(date -u +%Y%m%dT%H%M%SZ).XXXXXX")
report_file="$report_dir/agx-report.txt"
printf 'Collecting from 048 AGX; report: %s\n' "$report_file"
printf 'An SSH password, if needed, is entered in the terminal, never in a script or report.\n'

# Clear inherited forwards; no local/remote listeners or shared-master session.
# No PTY on robot. SSH can still prompt for a password on the local controlling terminal.
set +e
ssh -T -o "HostKeyAlias=$host_key_name" -o StrictHostKeyChecking=yes -o UpdateHostKeys=no \
    -o ConnectTimeout=8 -o ConnectionAttempts=1 \
    -o ServerAliveInterval=5 -o ServerAliveCountMax=3 \
    -o NumberOfPasswordPrompts=1 -o ClearAllForwardings=yes \
    -o ForwardAgent=no -o ForwardX11=no -o PermitLocalCommand=no \
    -o ControlMaster=no -o ControlPath=none -o LogLevel=ERROR \
    ${identity_args[@]+"${identity_args[@]}"} golai@10.21.33.102 \
    'env -u BASH_ENV bash --noprofile --norc -s --' ${probe_args[@]+"${probe_args[@]}"} \
    < "$SCRIPT_DIR/check_agx.sh" | tee "$report_file"
pipeline_status=("${PIPESTATUS[@]}")
set -e
ssh_rc=${pipeline_status[0]}
tee_rc=${pipeline_status[1]}
printf '\nSSH exit: %s; local report write exit: %s\n' "$ssh_rc" "$tee_rc"
printf 'Report kept locally (possibly partial): %s\n' "$report_file"
if ((ssh_rc != 0)); then
    printf 'Collection incomplete. Preserve host-key checks; do not delete known_hosts to bypass a mismatch.\n' >&2
    exit "$ssh_rc"
fi
((tee_rc == 0)) || exit "$tee_rc"
if ! grep -qx 'S10_AGX_REMOTE_ACCESS_REPORT_END' "$report_file"; then
    printf 'Report completion marker missing; treat as incomplete.\n' >&2
    exit 4
fi
printf 'Report collection finished. This does not mean remote access is configured.\n'
