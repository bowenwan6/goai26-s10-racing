#!/usr/bin/env bash
# Boot-time bring-up of our stack on the AGX (run by the systemd user unit s10-stack.service,
# or by hand). Order matters: the x_nav container's roscore owns :11311, the gateway joins it.
#   x_nav container -> gateway -> s10_ros1_control (DRY RUN, never --enable-motion) -> teach worker
#   -> 106 lidar tap, ONLY while a session is active (run/session_active, set by
#      "robot_session.sh up", cleared by "down"): a returned shared dog is never touched at boot.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$ROOT/run" "$ROOT/logs"
exec >> "$ROOT/logs/boot-$(date +%Y%m%d).log" 2>&1
echo "=== agx_boot $(date '+%F %T') uptime $(cut -d' ' -f1 /proc/uptime)s"
pgrep -f agx_power_log.py >/dev/null || setsid nohup python3 "$ROOT/scripts/agx_power_log.py" >/dev/null 2>&1 < /dev/null &
for i in $(seq 1 60); do docker info >/dev/null 2>&1 && break; sleep 2; done
docker start nav >/dev/null 2>&1 || (cd /opt/data/compose && docker compose up -d) || echo "WARNING: x_nav container not started"
for i in $(seq 1 60); do (exec 3<>/dev/tcp/127.0.0.1/11311) 2>/dev/null && break; sleep 1; done
bash "$ROOT/scripts/start_gateway.sh" | tail -2
if ! { [ -f "$ROOT/run/control.pid" ] && kill -0 "$(cat "$ROOT/run/control.pid")" 2>/dev/null; }; then
  bash "$ROOT/scripts/start_control.sh" | tail -1          # dry run only
fi
setsid nohup bash "$HOME/s10_mapping_web/teach-worker.sh" start > /tmp/teach-start.log 2>&1 < /dev/null; tail -1 /tmp/teach-start.log
if [ -f "$ROOT/run/session_active" ] && [ -f "$HOME/.ssh/s10_tap_ed25519" ]; then
  S=(ssh -i "$HOME/.ssh/s10_tap_ed25519" -o BatchMode=yes -o StrictHostKeyChecking=yes -o ConnectTimeout=8 user@10.21.33.106)
  T=/home/user/ros1_gateway_tap
  for i in $(seq 1 40); do "${S[@]}" true 2>/dev/null && break; sleep 5; done
  if "${S[@]}" "mkdir -p $T/logs && chmod 700 $T"; then
    "${S[@]}" "cat > $T/s10_lidar_tap.py" < "$ROOT/tap/s10_lidar_tap.py"
    "${S[@]}" "cat > $T/run_tap_106.sh" < "$ROOT/tap/run_tap_106.sh"
    "${S[@]}" "bash $T/run_tap_106.sh start" | tail -1
  else
    echo "WARNING: 106 not reachable with the AGX key; tap not started"
  fi
else
  echo "no active session (or no AGX tap key): 106 not touched"
fi
sleep 6; bash "$ROOT/scripts/health_check.sh" | tail -3
