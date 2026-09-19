#!/usr/bin/env bash
# One-command start/restore of everything we put on the shared robot (103/106).
# Runs on the operator's Mac. The AGX is ours and is not restored by "down"
# unless --agx is given.
#
#   scripts/robot_session.sh up       # deploy + start the 106 lidar tap, start the AGX gateway
#   scripts/robot_session.sh down     # stop the tap and delete everything we created on 106
#   scripts/robot_session.sh status
#   scripts/robot_session.sh down --agx   # also stop the gateway on the AGX
#
# Footprint on 106 (all of it, removed by "down"):
#   /home/user/ros1_gateway_tap/   tap script, launcher, logs, pid file
#   one user-level python3 process (nice 19, cores 0-3) while running
# Nothing on 103. No vendor files, services, systemd units or configs are touched.
#
# 106 login: reuses the SSH master ~/.ssh/sockets/s10-106 if open, otherwise asks
# for the 106 password (typed by the operator, never stored).
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
AGX="${S10_AGX_SSH:-s10-48-remote}"
TAP_DIR=/home/user/ros1_gateway_tap
S106=(ssh -o ControlMaster=auto -o "ControlPath=$HOME/.ssh/sockets/s10-106" -o ControlPersist=8h
      -o StrictHostKeyChecking=yes -J "$AGX" user@10.21.33.106)
A=(ssh -o BatchMode=yes "$AGX")

identity() {
  echo "== 106 identity"
  "${S106[@]}" 'hostname; cat /var/opt/robot/conf/robot_manufacturing_info.toml 2>/dev/null | tr "\n" " "; echo; grep -E "^model" /var/opt/robot/conf/robot_hardware_info.toml 2>/dev/null'
}

tap_status() {
  "${S106[@]}" "[ -f $TAP_DIR/run_tap_106.sh ] && bash $TAP_DIR/run_tap_106.sh status || echo 'tap not deployed'; pgrep -fa '^python3 -u $TAP_DIR/s10_lidar_tap.py' || true"
}

case "${1:-status}" in
  up)
    identity
    echo "== deploy tap to 106:$TAP_DIR"
    "${S106[@]}" "mkdir -p $TAP_DIR/logs && chmod 700 $TAP_DIR"
    "${S106[@]}" "cat > $TAP_DIR/s10_lidar_tap.py" < "$HERE/tap/s10_lidar_tap.py"
    "${S106[@]}" "cat > $TAP_DIR/run_tap_106.sh" < "$HERE/tap/run_tap_106.sh"
    "${S106[@]}" "cd $TAP_DIR && sha256sum s10_lidar_tap.py run_tap_106.sh"
    echo "== start gateway on AGX (idempotent)"
    "${A[@]}" 'bash ~/ros1_gateway/scripts/start_gateway.sh'
    echo "== start tap on 106"
    "${S106[@]}" "bash $TAP_DIR/run_tap_106.sh start"
    sleep 5
    tap_status
    "${A[@]}" 'bash ~/ros1_gateway/scripts/health_check.sh' || true
    ;;
  down)
    echo "== stop tap and remove $TAP_DIR on 106"
    "${S106[@]}" "[ -f $TAP_DIR/run_tap_106.sh ] && bash $TAP_DIR/run_tap_106.sh stop || true; \
      pkill -INT -f '$TAP_DIR/s10_lidar_tap.py' 2>/dev/null || true; sleep 1; \
      rm -rf $TAP_DIR; \
      if pgrep -f '^python3 -u $TAP_DIR/s10_lidar_tap.py' >/dev/null; then echo 'WARNING: tap process still running'; else echo '106: no tap process'; fi; \
      [ -e $TAP_DIR ] && echo 'WARNING: $TAP_DIR still exists' || echo '106: $TAP_DIR removed'"
    if [ "$2" = "--agx" ]; then "${A[@]}" 'bash ~/ros1_gateway/scripts/stop_gateway.sh'; fi
    echo "106/103 restored (nothing of ours left on 106; nothing was ever changed on 103)"
    ;;
  status)
    identity
    tap_status
    "${A[@]}" 'bash ~/ros1_gateway/scripts/health_check.sh' || true
    ;;
  *) echo "usage: $0 up|down [--agx]|status" >&2; exit 2 ;;
esac
