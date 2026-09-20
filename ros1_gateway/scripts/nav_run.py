#!/usr/bin/env python3
"""One command for a navigation run on the AGX (run as golai, ROS 1 env sourced by nav_run.sh):

  nav_run.sh [--speed 0.5] [--route short|full|<dir>] [--map v6_room] [--at start|end] [--shadow]

What it does, in order, stopping with a clear message at the first thing that is not right:
  1. sensor gateway healthy
  2. x_nav localising on --map: if not, selects the map (/node_cmd launch_navigation#<map>)
  3. pose arriving: if not, sends /initialpose = the route's first waypoint (--at end: the last one).
     Put the dog on that waypoint, facing along the route, before starting. An already running
     localisation is never touched.
  4. where is the dog? within 1.2 m of the route start -> forward route; within 1.2 m of the route
     end -> the reverse route (flat routes only); anywhere else -> stop and say so
  5. dog standing (state 17); waits up to 90 s for the operator to stand it with the remote
  6. arm at --speed, robot use mode -> navigation, go, one status line per second
  7. ALWAYS at the end (done, hold, fault, Ctrl-C): stop nav, disarm, use mode back to normal (remote)

Ctrl-C = pause and stop. The remote's emergency stop stays the real emergency stop.
"""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ROUTES = os.path.expanduser("~/routes")
NAV = os.path.join(ROOT, "scripts", "nav_session.sh")


def sh(*args, quiet=False):
    r = subprocess.run(list(args), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    if not quiet and r.stdout.strip():
        print("    " + r.stdout.strip().splitlines()[-1][:200])
    return r.returncode, r.stdout


def say(msg):
    print(time.strftime("%H:%M:%S ") + msg, flush=True)


def die(msg):
    say("STOP: " + msg)
    sys.exit(1)


def latest_route(short):
    ds = sorted((d for d in os.listdir(ROUTES) if d.endswith("-short") == short and not d.endswith("-rev")),
                key=lambda d: os.path.getmtime(os.path.join(ROUTES, d)))
    if not ds:
        die("no route in ~/routes (build one: nav_session.sh route latest --map-id <map>)")
    return os.path.join(ROUTES, ds[-1])


def reverse_route(route_dir):
    """<route>-rev: same taught line walked the other way. Flat routes only."""
    src = json.load(open(os.path.join(route_dir, "route_v2.json")))
    man = os.path.join(route_dir, "maneuvers.json")
    if os.path.exists(man) and (json.load(open(man)).get("maneuvers") or json.load(open(man)).get("zones")):
        die("the dog is at the END of a route with a stairs zone: the way back (descending) is not supported yet. "
            "Drive it back to the start with the remote.")
    if any(s.get("gait") != "flat" for s in src["segments"]):
        die("reverse is only built for flat routes; drive the dog back to the start with the remote")
    out = dict(src)
    out["status"] = "REVERSED " + src.get("status", "")
    wps = []
    for w in reversed(src["waypoints"]):
        w = dict(w)
        w["yaw"] = math.atan2(math.sin(w["yaw"] + math.pi), math.cos(w["yaw"] + math.pi))
        wps.append(w)
    segs = []
    for s in reversed(src["segments"]):
        s = dict(s)
        s["from"], s["to"] = s["to"], s["from"]
        s["id"] = f'{s["from"]}-{s["to"]}'
        s["centerline"] = list(reversed(s["centerline"]))
        segs.append(s)
    out["waypoints"], out["segments"] = wps, segs
    dst = route_dir.rstrip("/") + "-rev"
    os.makedirs(dst, exist_ok=True)
    json.dump(out, open(os.path.join(dst, "route_v2.json"), "w"), indent=1)
    return dst


class Run:
    def __init__(self):
        self.pose = None
        self.pose_t = 0.0
        self.ctl = {}
        self.nav = {}
        self.nav_t = 0.0
        rospy.Subscriber("/base_link/odom", Odometry, self.on_pose, queue_size=1)
        rospy.Subscriber("/s10_control/state", String, lambda m: self._json(m, "ctl"), queue_size=1)
        rospy.Subscriber("/rl_nav/status", String, lambda m: self._json(m, "nav"), queue_size=1)

    def _json(self, m, name):
        try:
            setattr(self, name, json.loads(m.data))
            if name == "nav":
                self.nav_t = time.monotonic()
        except ValueError:
            pass

    def on_pose(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.pose = (p.x, p.y, 2 * math.atan2(q.z, q.w))
        self.pose_t = time.monotonic()

    def pose_fresh(self):
        return self.pose is not None and time.monotonic() - self.pose_t < 1.0

    def wait(self, cond, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end and not rospy.is_shutdown():
            if cond():
                return True
            time.sleep(0.2)
        return cond()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--speed", default="0.3", help="forward limit m/s (0.1-1.0), or zero|probe|flat")
    ap.add_argument("--climb-speed", default="", help="stairs part m/s (default: nav.yaml climb_v 0.15)")
    ap.add_argument("--route", default="short", help="short | full | a route dir")
    ap.add_argument("--map", default="", help="x_nav map name (default: the route's map_id)")
    ap.add_argument("--at", choices=["start", "end"], default="start",
                    help="only used when localisation has to be initialised: which end of the route the dog stands on")
    ap.add_argument("--shadow", action="store_true", help="no motion: status only, drive with the remote")
    a = ap.parse_args()

    route = a.route if os.path.isdir(os.path.expanduser(a.route)) else latest_route(a.route == "short")
    route = os.path.expanduser(route).rstrip("/")
    doc = json.load(open(os.path.join(route, "route_v2.json")))
    map_id = a.map or doc["map_id"]
    first, last = doc["waypoints"][0], doc["waypoints"][-1]
    say(f"route {route} ({len(doc['waypoints'])} WPs, map {map_id})")

    # 1. sensors
    rc, out = sh("bash", os.path.join(ROOT, "scripts", "health_check.sh"), quiet=True)
    if "/LIDAR/POINTS" not in out or " 0.00 Hz" in [l for l in out.splitlines() if "/LIDAR/POINTS" in l][0]:
        die("no lidar on the AGX: run robot_session.sh up on the Mac (106 tap) and try again")
    say("1/6 sensors ok")

    rospy.init_node("nav_run", anonymous=True, disable_signals=True)
    run = Run()

    # 2. x_nav localising on this map
    rc, out = sh("docker", "exec", "nav", "pgrep", "-fa", "x_slam", quiet=True)
    if f"x_slam localization /nav_map/{map_id}" not in out:
        if not os.path.isdir(f"/opt/data/nav_map/{map_id}"):
            die(f"map {map_id} is not saved in x_nav (/opt/data/nav_map)")
        say(f"2/6 selecting map {map_id} in x_nav ...")
        pub = rospy.Publisher("/node_cmd", String, queue_size=1, latch=True)
        time.sleep(1.0)
        pub.publish(String(f"launch_navigation#{map_id}"))
        if not run.wait(lambda: f"localization /nav_map/{map_id}" in sh("docker", "exec", "nav", "pgrep", "-fa", "x_slam", quiet=True)[1], 25):
            die("x_nav did not start localisation; select the map in the x_nav page (:8000)")
        time.sleep(6.0)
    say("2/6 x_nav localisation running")

    # 3. pose
    time.sleep(1.5)
    if not run.pose_fresh():
        w = first if a.at == "start" else last
        say(f"3/6 no pose yet: sending the initial pose = {w['id']} ({w['position'][0]:.2f}, {w['position'][1]:.2f}, "
            f"{math.degrees(w['yaw']):.0f} deg). The dog must be standing THERE, facing along the route.")
        pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1, latch=True)
        m = PoseWithCovarianceStamped()
        m.header.frame_id = "map"
        m.pose.pose.position.x, m.pose.pose.position.y = float(w["position"][0]), float(w["position"][1])
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(w["yaw"] / 2), math.cos(w["yaw"] / 2)
        m.pose.covariance[0] = m.pose.covariance[7] = 0.25
        m.pose.covariance[35] = 0.07
        time.sleep(1.0)
        m.header.stamp = rospy.Time.now()
        pub.publish(m)
        if not run.wait(run.pose_fresh, 20):
            die("x_nav gives no pose after the initial pose; set it in the x_nav page (设置位姿)")
        time.sleep(3.0)
    x, y, yaw = run.pose
    say(f"3/6 pose ok: x {x:.2f} y {y:.2f} yaw {math.degrees(yaw):.0f} deg")

    # 4. which way
    d0 = math.hypot(x - first["position"][0], y - first["position"][1])
    d1 = math.hypot(x - last["position"][0], y - last["position"][1])
    if d0 <= 1.2:
        say(f"4/6 dog at the route START ({d0:.2f} m from {first['id']}): forward route")
    elif d1 <= 1.2:
        route = reverse_route(route)
        say(f"4/6 dog at the route END ({d1:.2f} m from {last['id']}): reverse route {route}")
    else:
        die(f"the dog is {d0:.1f} m from {first['id']} and {d1:.1f} m from {last['id']}. Drive it to one end with the remote "
            "(or the localisation is wrong: check red/green clouds in the x_nav page).")

    if a.shadow:
        sh("bash", NAV, "shadow", route)
        say("shadow mode running (no motion). Watch: nav_session.sh watch")
        return

    # 5. standing
    def standing():
        fb = run.ctl.get("feedback") or {}
        return fb.get("fresh") and fb.get("state") == 17
    if not standing():
        say("5/6 stand the dog up with the remote (waiting up to 90 s) ...")
        if not run.wait(standing, 90):
            die("the dog is not standing in RL control (state 17)")
    say("5/6 dog standing")

    # 6. arm, nav mode, go
    stopping = {"done": False}

    def finish(*_):
        if stopping["done"]:
            return
        stopping["done"] = True
        say("stopping: nav off, disarm, use mode back to normal ...")
        sh("bash", NAV, "stop")
        say("finished. The remote has control again.")

    signal.signal(signal.SIGINT, lambda *_: (finish(), sys.exit(130)))
    signal.signal(signal.SIGTERM, lambda *_: (finish(), sys.exit(143)))
    try:
        rc, out = sh("bash", NAV, "arm", a.speed, route, a.climb_speed, quiet=True)
        if "ARMED" not in out:
            print(out[-600:])
            die("arming failed")
        rc, out = sh("bash", NAV, "usemode", "nav", quiet=True)
        if "MODE_OK 1" not in out and "already in mode 1" not in out:
            print(out[-400:])
            die("the robot did not enter navigation use mode (ASDU). Is encryption off? robot_session.sh tls status")
        say(f"6/6 armed at {a.speed}, navigation mode on. GO in 3 s  (Ctrl-C = stop)")
        time.sleep(3.0)
        sh("bash", NAV, "go", quiet=True)
        t0, last_line = time.monotonic(), ""
        while not rospy.is_shutdown():
            time.sleep(1.0)
            s, c = run.nav, run.ctl
            fb = c.get("feedback") or {}
            line = "%-6s wp %s/%s s=%s d=%s cmd=%s gait %s | %s" % (
                s.get("mode"), s.get("reached"), s.get("total"), s.get("s"), s.get("d"), s.get("cmd"),
                s.get("gait_reported"), (s.get("reason") or "")[:50])
            if line != last_line:
                say(line)
                last_line = line
            if c.get("fault"):
                say("CONTROL FAULT: " + str(c.get("fault")))
                break
            if s.get("mode") == "DONE":
                say("route finished in %.0f s" % (time.monotonic() - t0))
                break
            if s.get("mode") == "HOLD" and time.monotonic() - t0 > 3:
                say("HOLD: the navigation stopped by itself (%s / %s)" % (s.get("follower"), s.get("follower_reason")))
                break
            if fb.get("state") not in (17, None):
                say("the dog left RL control (state %s)" % fb.get("state"))
                break
            if time.monotonic() - run.nav_t > 3:
                say("no status from the navigation node")
                break
    finally:
        finish()


if __name__ == "__main__":
    main()
