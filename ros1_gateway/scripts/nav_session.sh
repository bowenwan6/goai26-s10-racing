#!/usr/bin/env bash
# One place for a navigation run on the AGX (run as golai on the AGX):
#
#   nav_session.sh route [<session dir>|latest] --map-id <x_nav map> [teach_to_route options]
#                                   # session -> ~/routes/<session id>/ (route_v2.json, maneuvers.json, report.json)
#   nav_session.sh shadow [<route dir>|latest]   # nav node publishes status only; drive with the remote
#   nav_session.sh arm zero|probe|flat|<v> [<route dir>|latest] [<climb v>]
#                                   # <v> = THE forward speed: m/s (e.g. 0.8, up to 1.67) or "0.7x" = 0.7 of the robot maximum: control clamp, runner walking speed and
#                                   # the route's flat limits all follow it. <climb v> = same for the stairs part.
#                                   # control with --enable-motion at that stage's limits + nav node, PAUSED
#   nav_session.sh go               # start / continue the route
#   nav_session.sh pause            # zero velocity, keep position on the route
#   nav_session.sh loc [status|restore|wp <WPxx> [route]]   # map + pose back WITHOUT the x_nav page
#   nav_session.sh usemode [status|nav|normal]   # ON SITE ONLY for nav/normal. Robot "use mode" over ASDU
#                                   # (developer guide 1.2.2 / 2.3.1): /NAV_CMD only works in 导航模式 (1).
#                                   # status is read-only. "stop" switches back to 常规模式 (0).
#   nav_session.sh stand|navmode    # ON SITE ONLY: "cmd4" stand -> RL -> flat nav gait / "cmd1" RL -> flat nav gait
#   nav_session.sh watch            # one status line per second
#   nav_session.sh stop             # stop nav, stop armed control, control back to DRY RUN
#
# arm never moves the robot by itself: the nav node waits for "go". The operator keeps the remote
# in hand; taking over with the remote is always allowed. "stop" is the normal end of every run.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROUTES="$HOME/routes"
PORT="${S10_ROS1_MASTER_PORT:-11311}"
# all parameters live in config/s10_params.yaml; the file read below is regenerated from it first (tools/params.py)
python3 "$ROOT/tools/params.py" sync --quiet || { echo "params sync failed: config/s10_params.yaml is broken"; exit 1; }
ros() { ( source "$ROOT/ros1/ros1_env.sh"; export ROS_MASTER_URI="http://127.0.0.1:$PORT" ROS_IP=127.0.0.1; "$@" ); }
latest_in() { ls -1dt "$@" 2>/dev/null | head -1 || true; }
route_dir() {
  if [ -z "$1" ] || [ "$1" = latest ]; then latest_in "$ROUTES"/*/; else echo "$1"; fi
}

case "${1:-}" in
  route)
    shift; S="${1:-latest}"; [ $# -gt 0 ] && shift
    if [ "$S" = latest ]; then
      S="$(latest_in /mnt/s10ssd/s10_teach/sessions/*/ "$HOME"/teach/sessions/*/)"
      [ -n "$S" ] || { echo "no teach session found"; exit 1; }
    fi
    S="${S%/}"; OUT="$ROUTES/$(basename "$S")"
    case " $* " in *" --last-wp "*) OUT="$OUT-short" ;; esac
    echo "session: $S"; echo "marks: $(grep -c . "$S/marks.jsonl" 2>/dev/null || echo 0) rows; trails: $(ls "$S"/*.trail.csv 2>/dev/null | wc -l)"
    python3 "$ROOT/tools/teach_to_route.py" "$S" --out "$OUT" "$@"
    echo "route dir: $OUT"
    ;;
  shadow)
    R="$(route_dir "$2")"; [ -d "$R" ] || { echo "no route dir"; exit 1; }
    bash "$ROOT/scripts/stop_nav.sh" >/dev/null 2>&1 || true
    bash "$ROOT/scripts/start_nav.sh" --route-dir "$R" --shadow --autostart
    ;;
  arm)
    STAGE="$2"; R="$(route_dir "$3")"
    NAVCFG="$ROOT/config/nav.yaml"
    case "$STAGE" in
      zero|probe|flat) ;;
      [0-9]*)         # a number = forward speed limit (m/s) for BOTH the control clamp and the runner's walking speed
        NAVCFG="$ROOT/run/nav-$STAGE.yaml"
        python3 - "$ROOT/config/nav.yaml" "$NAVCFG" "$STAGE" "${4:-}" <<'PY'
import sys, yaml
src, dst = sys.argv[1], sys.argv[2]
v = float(sys.argv[3][:-1]) * 1.67 if sys.argv[3].endswith("x") else float(sys.argv[3])   # "0.7x" = 0.7 of the robot maximum
c = yaml.safe_load(open(src))
climb = float(sys.argv[4]) if len(sys.argv) > 4 and sys.argv[4] else 0.0
c["gains"]["max_yaw_rate"] = max(float(c["gains"]["max_yaw_rate"]), 0.8 if v > 0.5 else 0.6)
c["runner_params"]["walk_v"] = v
c["gains"]["max_forward"] = max(float(c["gains"]["max_forward"]), v)
c["flat_speed_override"] = v                 # the route file's flat limits no longer matter
for k in ("approach_v", "detour_v", "recover_v"):
    c["runner_params"][k] = max(float(c["runner_params"][k]), min(v, 0.5))
if climb > 0.0:                              # stairs: climb speed, also ONE number
    c["stairs_speed_override"] = climb
    import os
    steep = float(os.environ.get("S10_STEEP_V") or (c.get("zone_speed") or {}).get("steep", 0.45))
    c["zone_speed"] = dict(c.get("zone_speed") or {}, cruise=climb, steep=(min(climb, steep) if steep > 0.0 else 0.0))
    c["runner_params"]["climb_v"] = climb
    c["runner_params"]["climb_turn_v"] = round(climb * 0.66, 2)
import os as _os
if _os.environ.get("S10_WALK_V"):
    c.setdefault("zone_speed", {})["walk"] = float(_os.environ["S10_WALK_V"])
yaml.safe_dump(c, open(dst, "w"), sort_keys=False)
PY
        ;;
      *) echo "arm zero|probe|flat|<max forward m/s> [route dir]"; exit 2 ;;
    esac
    [ -d "$R" ] || { echo "no route dir"; exit 1; }
    bash "$ROOT/scripts/stop_nav.sh" >/dev/null 2>&1 || true
    bash "$ROOT/scripts/stop_control.sh" >/dev/null 2>&1 || true
    sleep 1
    bash "$ROOT/scripts/start_control.sh" --enable-motion --stage "$STAGE"
    bash "$ROOT/scripts/start_nav.sh" --route-dir "$R" --config "$NAVCFG"
    echo "ARMED at stage $STAGE, nav PAUSED. Route: $R"
    echo "Next: robot standing (state 17) -> nav_session.sh usemode nav -> nav_session.sh go"
    ;;
  go)    ros rostopic pub -1 /rl_nav/cmd std_msgs/String "data: start" ;;
  pause) ros rostopic pub -1 /rl_nav/cmd std_msgs/String "data: pause" ;;
  loc)
    # Localisation without the x_nav page (scripts/loc_keeper.py):
    #   loc status | loc restore | loc wp <WPxx> [route dir|latest]  (robot standing ON that waypoint, facing as marked)
    case "${2:-status}" in
      status)  ros rostopic echo -n1 /s10_loc/state | head -4 ;;
      restore) ros rostopic pub -1 /s10_loc/cmd std_msgs/String "data: restore" >/dev/null; sleep 1; echo "asked; watch: nav_session.sh loc status" ;;
      wp)
        R="$(route_dir "${4:-}")"; [ -f "$R/route_v2.json" ] || { echo "no route"; exit 1; }
        CMD="$(python3 - "$R/route_v2.json" "$3" <<'PY'
import json, math, sys
d = json.load(open(sys.argv[1])); w = next((w for w in d["waypoints"] if w["id"] == sys.argv[2]), None)
if w is None: sys.exit("no such waypoint in this route: " + sys.argv[2])
print("at %s %.3f %.3f %.1f %.2f" % (d["map_id"], w["position"][0], w["position"][1], math.degrees(w["yaw"]), w["position"][2] + 0.41))
PY
)" || exit 1
        echo "$CMD"; ros rostopic pub -1 /s10_loc/cmd std_msgs/String "data: $CMD" >/dev/null; echo "asked; watch: nav_session.sh loc status" ;;
      *) echo "loc status|restore|wp <WPxx> [route]"; exit 2 ;;
    esac
    ;;
  usemode)
    case "${2:-status}" in
      status) python3 "$ROOT/tools/asdu_mode.py" status --seconds 4 ;;
      nav)    python3 "$ROOT/tools/asdu_mode.py" set-mode 1 --i-am-on-site ;;
      normal) python3 "$ROOT/tools/asdu_mode.py" set-mode 0 --i-am-on-site ;;
      *) echo "usemode status|nav|normal"; exit 2 ;;
    esac
    ;;
  stand)   ros rostopic pub -1 /s10_control/web_cmd std_msgs/String "data: cmd4" ;;
  navmode) ros rostopic pub -1 /s10_control/web_cmd std_msgs/String "data: cmd1" ;;
  watch)
    ros python3 -u - <<'PY'
import json, rospy
from std_msgs.msg import String
last = {"c": {}}
def on_c(m):
    try: last["c"] = json.loads(m.data)
    except ValueError: pass
def on_s(m):
    try: s = json.loads(m.data)
    except ValueError: return
    c = last["c"]
    fb = c.get("feedback") or {}
    print("%-8s %-28s wp %s/%s target %s s=%s d=%s cmd=%s gait %s/%s | ctl motion=%s fault=%s latched=%s robot_state=%s" % (
        s.get("mode"), (s.get("reason") or "")[:28], s.get("reached"), s.get("total"), s.get("target"), s.get("s"),
        s.get("d"), s.get("cmd"), s.get("gait_request"), s.get("gait_reported"),
        c.get("enable_motion"), c.get("fault"), c.get("latched_stop"), fb.get("state")), flush=True)
rospy.init_node("nav_watch", anonymous=True)
rospy.Subscriber("/s10_control/state", String, on_c, queue_size=1)
rospy.Subscriber("/rl_nav/status", String, lambda m: None, queue_size=1)
r = rospy.Rate(1)
while not rospy.is_shutdown():
    try: on_s(rospy.wait_for_message("/rl_nav/status", String, timeout=2))
    except rospy.ROSException: print("no /rl_nav/status", flush=True)
    r.sleep()
PY
    ;;
  stop)
    ros rostopic pub -1 /rl_nav/cmd std_msgs/String "data: pause" >/dev/null 2>&1 || true
    bash "$ROOT/scripts/stop_nav.sh" || true
    bash "$ROOT/scripts/stop_control.sh" || true
    # give the robot back to the remote: use mode 常规 (0). No effect if it already is, or if ASDU is unreachable.
    python3 "$ROOT/tools/asdu_mode.py" set-mode 0 --i-am-on-site --seconds 3 2>&1 | tail -1 || true
    # ... and its remote gait: a run ends in a navigation gait, in which the remote page cannot drive or take over.
    python3 "$ROOT/tools/asdu_mode.py" remote-gait --i-am-on-site --seconds 3 2>&1 | tail -1 || true
    sleep 1
    bash "$ROOT/scripts/start_control.sh" | tail -1
    ;;
  *) sed -n '2,22p' "${BASH_SOURCE[0]}"; exit 2 ;;
esac
