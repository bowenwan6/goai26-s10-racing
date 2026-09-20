#!/usr/bin/env bash
# One-command start/restore of everything we put on the shared robot (103/106).
# Runs on the operator's Mac. The AGX is ours and is not restored by "down"
# unless --agx is given.
#
#   scripts/robot_session.sh up       # deploy + start the 106 lidar tap, start the AGX gateway
#   scripts/robot_session.sh down     # stop the tap and delete everything we created on 106
#   scripts/robot_session.sh status
#   scripts/robot_session.sh down --agx   # also stop the gateway on the AGX
#   scripts/robot_session.sh keys     # ONE TIME: install this Mac's SSH key on 106, 103 and ysc@AGX
#                                     # (operator types each password once; afterwards no passwords)
#   scripts/robot_session.sh unkeys   # remove our keys (Mac and AGX) from 106/103 again (full restore)
#   scripts/robot_session.sh nav [--speed 0.5] [--route short|full] [--at start|end] [--shadow]
#                                     # ONE COMMAND navigation run from the Mac (runs nav_run.sh on the AGX):
#                                     # map select, initial pose, forward/reverse route, arm, nav mode, go, restore.
#   scripts/robot_session.sh navstop  # stop a run from another terminal
#   scripts/robot_session.sh tls status|off|restore
#                                     # 103 robot_server ASDU encryption (Network.toml enableTls). Factory = true.
#                                     # "off" is needed for the use-mode switch (nav mode); takes effect after the
#                                     # dog is power-cycled. "restore" puts true back: run it before hand-back.
#   scripts/robot_session.sh autostart  # ONE TIME: AGX boots the whole stack by itself (systemd user unit);
#                                     # the 106 tap only while a session is active (between up and down)
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
J=(ssh -o StrictHostKeyChecking=yes -o ControlPath=none -J "$AGX")
KEY_TAG=s10-ros1-gateway-mac
PUB="$HOME/.ssh/id_ed25519.pub"

# Appends one tagged line to ~/.ssh/authorized_keys; remembers if the file did not exist before
# so "unkeys" restores the exact previous state. Only the public key is sent.
KEY_ADD="umask 077; mkdir -p ~/.ssh; [ -e ~/.ssh/authorized_keys ] || touch ~/.ssh/.$KEY_TAG.created ~/.ssh/authorized_keys; \
  grep -q ' $KEY_TAG\$' ~/.ssh/authorized_keys || echo \"\$(cat)\" >> ~/.ssh/authorized_keys; echo \"\$(whoami)@\$(hostname): key installed\""
KEY_DEL="if [ -f ~/.ssh/authorized_keys ]; then grep -v -e ' $KEY_TAG\$' -e ' s10-ros1-gateway-agx\$' ~/.ssh/authorized_keys > ~/.ssh/.ak.tmp; cat ~/.ssh/.ak.tmp > ~/.ssh/authorized_keys; rm -f ~/.ssh/.ak.tmp; fi; \
  if [ -e ~/.ssh/.$KEY_TAG.created ] && [ ! -s ~/.ssh/authorized_keys ]; then rm -f ~/.ssh/authorized_keys; fi; rm -f ~/.ssh/.$KEY_TAG.created; \
  echo \"\$(whoami)@\$(hostname): key removed\""
keyline() { echo "$(awk '{print $1" "$2}' "$PUB") $KEY_TAG"; }

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
    "${A[@]}" 'mkdir -p ~/ros1_gateway/run && touch ~/ros1_gateway/run/session_active'
    echo "== AGX: x_nav container first (its roscore owns :11311), then gateway, control (dry run), teach worker"
    "${A[@]}" 'docker start nav >/dev/null 2>&1 || (cd /opt/data/compose && docker compose up -d) >/dev/null 2>&1 || echo "WARNING: could not start the x_nav container (run: cd /opt/data/compose && sudo docker compose up -d)"
      for i in $(seq 1 30); do (exec 3<>/dev/tcp/127.0.0.1/11311) 2>/dev/null && break; sleep 1; done
      bash ~/ros1_gateway/scripts/start_gateway.sh
      [ -f ~/ros1_gateway/run/control.pid ] && kill -0 "$(cat ~/ros1_gateway/run/control.pid)" 2>/dev/null || bash ~/ros1_gateway/scripts/start_control.sh | tail -1
      rm -f ~/teach/teach-worker.pid.stale; setsid nohup bash ~/s10_mapping_web/teach-worker.sh start > /tmp/teach-start.log 2>&1 < /dev/null; tail -1 /tmp/teach-start.log'
    echo "== start tap on 106"
    "${S106[@]}" "bash $TAP_DIR/run_tap_106.sh start"
    sleep 5
    tap_status
    "${A[@]}" 'bash ~/ros1_gateway/scripts/health_check.sh; bash ~/s10_mapping_web/teach-worker.sh status; docker ps --format "x_nav container: {{.Names}} {{.Status}}"' || true
    ;;
  down)
    "${A[@]}" 'rm -f ~/ros1_gateway/run/session_active' || true   # the AGX no longer starts the tap at boot
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
  keys)
    echo "== 106 (user)";      keyline | "${S106[@]}" "$KEY_ADD"
    echo "== 103 (user)";      keyline | "${J[@]}" user@10.21.33.103 "$KEY_ADD"
    echo "== AGX (ysc)";       keyline | "${J[@]}" ysc@10.21.33.102 "$KEY_ADD"
    echo "== check (no password allowed)"
    for h in user@10.21.33.106 user@10.21.33.103 ysc@10.21.33.102; do
      "${J[@]}" -o BatchMode=yes "$h" true && echo "$h: key login OK" || echo "$h: key login FAILED"
    done
    ;;
  autostart)
    echo "== AGX key for the 106 tap (public key only leaves the AGX)"
    "${A[@]}" 'umask 077; mkdir -p ~/.ssh; [ -f ~/.ssh/s10_tap_ed25519 ] || ssh-keygen -q -t ed25519 -N "" -C s10-ros1-gateway-agx -f ~/.ssh/s10_tap_ed25519'
    "${A[@]}" 'awk "{print \$1\" \"\$2\" s10-ros1-gateway-agx\"}" ~/.ssh/s10_tap_ed25519.pub' | \
      "${S106[@]}" "umask 077; mkdir -p ~/.ssh; touch ~/.ssh/authorized_keys; grep -q ' s10-ros1-gateway-agx\$' ~/.ssh/authorized_keys || cat >> ~/.ssh/authorized_keys; echo 106: AGX key installed"
    # pin 106's host key on the AGX from this Mac's verified known_hosts (strict checking stays on)
    ssh-keygen -F 10.21.33.106 | grep -v '^#' | "${A[@]}" 'while read -r l; do grep -qxF "$l" ~/.ssh/known_hosts 2>/dev/null || echo "$l" >> ~/.ssh/known_hosts; done'
    "${A[@]}" 'ssh -i ~/.ssh/s10_tap_ed25519 -o BatchMode=yes -o StrictHostKeyChecking=yes -o ConnectTimeout=8 user@10.21.33.106 "echo AGX to 106 key login OK"'
    echo "== systemd user unit"
    "${A[@]}" 'mkdir -p ~/.config/systemd/user && cp ~/ros1_gateway/scripts/s10-stack.service ~/.config/systemd/user/ && systemctl --user daemon-reload && systemctl --user enable s10-stack.service && loginctl show-user "$USER" -p Linger'
    ;;
  nav)     shift; exec ssh -t "$AGX" "bash ~/ros1_gateway/scripts/nav_run.sh $*" ;;
  navstop) exec ssh "$AGX" 'bash ~/ros1_gateway/scripts/nav_session.sh stop' ;;
  tls)
    F=/var/opt/robot/conf/robot_server/Network.toml
    case "${2:-status}" in
      status)  "${J[@]}" -o BatchMode=yes user@10.21.33.103 "grep -n enableTls $F; D=\$(ls -d /var/opt/robot/log/2026_* | tail -1); grep -a -h -E '明文|udp server init' \$D/robot_server*.log | tail -2" ;;
      off)     "${J[@]}" -o BatchMode=yes user@10.21.33.103 "[ -e $F.s10-orig ] || cp -p $F $F.s10-orig; sed -i 's/^enableTls *= *true/enableTls = false/' $F; grep -n enableTls $F; echo 'power-cycle the dog to apply'" ;;
      restore) "${J[@]}" -o BatchMode=yes user@10.21.33.103 "sed -i 's/^enableTls *= *false/enableTls = true/' $F; rm -f $F.s10-orig; grep -n enableTls $F; echo 'factory value restored; applies at the next dog power-cycle'" ;;
      *) echo "tls status|off|restore"; exit 2 ;;
    esac
    ;;
  unkeys)
    for h in user@10.21.33.106 user@10.21.33.103; do "${J[@]}" -o BatchMode=yes "$h" "$KEY_DEL" || echo "$h: not reachable"; done
    ;;
  *) echo "usage: $0 up|down [--agx]|status|keys|unkeys|autostart|tls" >&2; exit 2 ;;
esac
