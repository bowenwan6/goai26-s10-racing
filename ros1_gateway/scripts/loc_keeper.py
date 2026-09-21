#!/usr/bin/env python3
"""Keeps x_nav's localisation usable without its web page (AGX, ROS 1).

x_nav forgets everything at every restart: somebody has to pick the map and set the pose in its UI.
This node remembers the last good pose on disk and puts it back:

  * while x_slam localises on a saved map and /base_link/odom is fresh, the pose is written to
    run/last_pose.json once a second (atomic, fsynced: survives the AGX power cuts);
  * at start-up with --auto-restore, and whenever asked on /s10_loc/cmd, it selects the map
    (/node_cmd "launch_navigation#<map>", what the x_nav page sends) and publishes /initialpose.

/s10_loc/cmd   std_msgs/String
    restore                      last saved map + pose
    restore <map>                last saved pose, only if it was saved on <map>
    at <map> <x> <y> <yaw_deg> [z]   explicit pose (e.g. a waypoint the robot stands on)
/s10_loc/state std_msgs/String JSON, 1 Hz: map, localising, pose_fresh, pose, saved, phase, message

A restored pose is only right if the robot was not moved while x_nav was down. Nothing here moves the
robot; the app shows the pose and the operator confirms it before any navigation starts.
"""
import argparse
import json
import math
import os
import subprocess
import threading
import time

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SAVE = os.path.join(ROOT, "run", "last_pose.json")
MAPS = "/opt/data/nav_map"


def active_map():
    """Map x_slam is LOCALISING on, or None (mapping mode and 'no map' both give None)."""
    try:
        out = subprocess.run(["docker", "exec", "nav", "pgrep", "-fa", "x_slam"], capture_output=True, text=True, timeout=4).stdout
    except (OSError, subprocess.TimeoutExpired):
        return None
    for line in out.splitlines():
        parts = line.split()
        if "localization" in parts:
            return os.path.basename(parts[parts.index("localization") + 1].rstrip("/"))
    return None


class Keeper:
    def __init__(self, auto_restore):
        self.lock = threading.Lock()
        self.pose = None
        self.pose_t = 0.0
        self.map = None
        self.map_t = 0.0
        self.phase, self.message = "idle", ""
        self.busy = False
        self.saved = self.load()
        self.node_cmd = rospy.Publisher("/node_cmd", String, queue_size=1)
        self.initial = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
        self.state_pub = rospy.Publisher("/s10_loc/state", String, queue_size=1, latch=True)
        rospy.Subscriber("/base_link/odom", Odometry, self.on_pose, queue_size=1)
        rospy.Subscriber("/s10_loc/cmd", String, self.on_cmd, queue_size=5)
        if auto_restore:
            threading.Thread(target=self.auto, daemon=True).start()

    # ------------------------------------------------------------------ persistence
    @staticmethod
    def load():
        try:
            return json.load(open(SAVE))
        except (OSError, ValueError):
            return None

    def save(self):
        with self.lock:
            fresh = self.pose is not None and time.monotonic() - self.pose_t < 1.0
            pose, m = self.pose, self.map
        if not fresh or not m or self.busy:
            return
        doc = dict(map=m, x=round(pose[0], 3), y=round(pose[1], 3), z=round(pose[2], 3), yaw=round(pose[3], 4), wall=round(time.time(), 1))
        tmp = SAVE + ".tmp"
        os.makedirs(os.path.dirname(SAVE), exist_ok=True)
        with open(tmp, "w") as f:
            json.dump(doc, f)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, SAVE)
        self.saved = doc

    # ------------------------------------------------------------------ inputs
    def on_pose(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        with self.lock:
            self.pose, self.pose_t = (p.x, p.y, p.z, yaw), time.monotonic()

    def on_cmd(self, msg):
        parts = msg.data.split()
        if not parts or self.busy:
            return
        try:
            if parts[0] == "restore":
                s = self.saved
                if not s:
                    return self.set("fault", "no saved pose yet")
                if len(parts) > 1 and parts[1] != s["map"]:
                    return self.set("fault", f"the saved pose is on map {s['map']}, not {parts[1]}")
                target = (s["map"], s["x"], s["y"], s["yaw"], s.get("z", 0.0))
            elif parts[0] == "at" and len(parts) >= 5:
                target = (parts[1], float(parts[2]), float(parts[3]), math.radians(float(parts[4])), float(parts[5]) if len(parts) > 5 else 0.0)
            else:
                return self.set("fault", "unknown command: " + msg.data[:60])
        except ValueError:
            return self.set("fault", "bad numbers in: " + msg.data[:60])
        threading.Thread(target=self.restore, args=target, daemon=True).start()

    # ------------------------------------------------------------------ actions
    def set(self, phase, message):
        self.phase, self.message = phase, message
        rospy.loginfo("loc_keeper %s: %s", phase, message)

    def auto(self):
        time.sleep(8.0)                       # let x_nav's controller come up after boot
        for _ in range(30):                   # wait for the container's ROS master side to be ready
            if rospy.is_shutdown():
                return
            if active_map():
                return self.set("idle", "x_nav is already localising; left alone")
            if self.saved and self.node_cmd.get_num_connections() > 0:
                s = self.saved
                return self.restore(s["map"], s["x"], s["y"], s["yaw"], s.get("z", 0.0))
            time.sleep(2.0)
        self.set("idle", "nothing to restore" if not self.saved else "x_nav controller not reachable")

    def restore(self, map_id, x, y, yaw, z=0.0):
        if self.busy:
            return
        self.busy = True
        try:
            if not os.path.isdir(os.path.join(MAPS, map_id)):
                return self.set("fault", f"map {map_id} is not saved in x_nav")
            if active_map() != map_id:
                self.set("selecting", f"selecting map {map_id}")
                self.node_cmd.publish(String(f"launch_navigation#{map_id}"))
                end = time.monotonic() + 40
                while time.monotonic() < end and active_map() != map_id:
                    time.sleep(1.0)
                if active_map() != map_id:
                    return self.set("fault", "x_nav did not start localisation on " + map_id)
                time.sleep(6.0)               # x_slam loads the map before it accepts a pose
            self.set("posing", f"initial pose {x:.2f} {y:.2f} {math.degrees(yaw):.0f} deg on {map_id}")
            m = PoseWithCovarianceStamped()
            m.header.frame_id = "map"
            m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = x, y, z
            m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            m.pose.covariance[0] = m.pose.covariance[7] = 0.25
            m.pose.covariance[35] = 0.07
            for attempt in range(3):
                sent = time.monotonic()
                m.header.stamp = rospy.Time.now()
                self.initial.publish(m)
                end = time.monotonic() + 12
                while time.monotonic() < end:
                    with self.lock:
                        ok = self.pose is not None and self.pose_t > sent + 1.0
                        pose = self.pose
                    if ok:
                        d = math.hypot(pose[0] - x, pose[1] - y)
                        if d < 1.5:
                            return self.set("restored", f"localising on {map_id}, {d:.2f} m from the restored pose: CHECK it before navigating")
                        return self.set("fault", f"localisation answered {d:.1f} m away from the requested pose")
                    time.sleep(0.3)
            self.set("fault", "no pose from x_nav after the initial pose")
        finally:
            self.busy = False

    def spin(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            now = time.monotonic()
            if now - self.map_t > 5.0 and not self.busy:
                m = active_map()
                with self.lock:
                    self.map, self.map_t = m, now
            self.save()
            with self.lock:
                fresh = self.pose is not None and now - self.pose_t < 1.0
                pose = [round(v, 3) for v in self.pose] if self.pose else None
            self.state_pub.publish(String(json.dumps(dict(map=self.map, localising=bool(self.map), pose_fresh=fresh, pose=pose, saved=self.saved,
                                                          phase=self.phase, message=self.message))))
            rate.sleep()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--auto-restore", action="store_true", help="at start-up, if x_nav has no map, restore the saved map and pose")
    a, _ = ap.parse_known_args()
    rospy.init_node("s10_loc_keeper")
    Keeper(a.auto_restore).spin()


if __name__ == "__main__":
    main()
