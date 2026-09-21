#!/usr/bin/env python3
"""ROS 1 navigation node for the S10 on the AGX: the team's route runner (s10_auto_nav.rl_nav) fed
by x_nav's pose and the gateway's lidar, driving the robot through s10_ros1_control.

  python3 nav/s10_rl_nav_ros1.py --config config/nav.yaml --route-dir ~/routes/<route>  [--shadow] [--autostart]

Subscribed (ROS 1)
  pose_topic            nav_msgs/Odometry | geometry_msgs/PoseStamped | PoseWithCovarianceStamped (x_nav)
  lidar_topic           sensor_msgs/PointCloud2 (gateway /LIDAR/POINTS, lidar frame)
  attitude_topic        sensor_msgs/Imu (robot /IMU): roll and pitch for levelling the cloud and for the
                        runner. x_nav's roll/pitch are NOT used when this is fresh: on dog 048 x_nav
                        reported pitch -3.5 deg while the robot stood level (IMU 0, ground fit 0.5 deg).
  /s10_control/gait     std_msgs/String  "flat" | "stairs" | "switching" | "none"  (confirmed gait)
  /s10_control/state    std_msgs/String  JSON (fault / latched stop -> the runner is told to hold)
  /rl_nav/cmd           std_msgs/String  "start" | "pause" | "reset"
Published
  /rl_nav/cmd_vel       geometry_msgs/Twist   body velocity for s10_ros1_control (source rl_nav)
  /rl_nav/gait_request  std_msgs/String       "flat" | "stairs", every tick
  /rl_nav/status        std_msgs/String       JSON, every tick
  /nav/progress         std_msgs/Float32 (latched), /nav/finished std_msgs/Bool (latched)

--shadow publishes status only (no /rl_nav/cmd_vel, no gait requests): drive with the remote and
watch progress, target and mode. Without --autostart the node waits for "start" on /rl_nav/cmd.
It commands zero velocity whenever the pose is older than pose_timeout, the control node reports
a fault or a latched stop, or it is paused.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import threading
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import nav_core as core  # noqa: E402

import rospy  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from sensor_msgs.msg import Imu, PointCloud2  # noqa: E402
from std_msgs.msg import Bool, Float32, String  # noqa: E402

POSE_TYPES = {
    "nav_msgs/Odometry": ("nav_msgs.msg", "Odometry"),
    "geometry_msgs/PoseStamped": ("geometry_msgs.msg", "PoseStamped"),
    "geometry_msgs/PoseWithCovarianceStamped": ("geometry_msgs.msg", "PoseWithCovarianceStamped"),
}


class Node:
    def __init__(self, args):
        cfg = core.load_yaml(args.config)
        self.cfg = cfg
        self.rate_hz = float(cfg.get("control_rate", 20.0))
        self.pose_timeout = float(cfg.get("pose_timeout", 0.3))
        self.obs_timeout = float(cfg.get("obs_timeout", 0.5))
        self.pose_jump_m = float(cfg.get("pose_jump_m", 0.5))      # between two pose messages
        self.pose_jump_deg = float(cfg.get("pose_jump_deg", 30.0))
        self.jump = None                                           # latched until "start"
        self.att = None                                            # (roll, pitch) from the robot IMU
        self.att_t = -1e9
        self.att_n = 0
        self.att_timeout = float(cfg.get("attitude_timeout", 0.3))
        self.shadow = bool(args.shadow)
        bundle = core.RouteBundle.from_dir(args.route_dir)
        self.core = core.NavCore(bundle, cfg)
        self.transform = core.FrameTransform.load(cfg.get("route_from_xnav_file") or args.transform)
        ext = cfg.get("lidar_extrinsics") or {}
        self.perception = core.Perception(core.transform_from_dict(ext), int(cfg.get("max_points", 60000)))
        self.perception.blind_radius = float(cfg.get("blind_radius", 0.0))
        self.perception.self_box = tuple(float(v) for v in cfg.get("self_box", (0.0, 0.0, -0.30)))
        self.perception.self_clear_radius = float(cfg.get("self_clear_radius", 0.0))
        self.lock = threading.Lock()
        self.pose = None
        self.pose_t = -1e9
        self.pose_type = None
        self.obs = None
        self.obs_t = -1e9
        self.obs_ms = 0.0
        self.gait_reported = None
        self.gait_t = -1e9
        self.control = {}
        self.control_t = -1e9
        self.running = bool(args.autostart)
        self.finished = False
        self.t0 = time.monotonic()
        self.tick_n = 0
        self.last_log = 0.0
        self.last_mode = None

        self.cmd_pub = rospy.Publisher("/rl_nav/cmd_vel", Twist, queue_size=1)
        self.gait_pub = rospy.Publisher("/rl_nav/gait_request", String, queue_size=1)
        self.status_pub = rospy.Publisher("/rl_nav/status", String, queue_size=1)
        self.progress_pub = rospy.Publisher("/nav/progress", Float32, queue_size=1, latch=True)
        self.finished_pub = rospy.Publisher("/nav/finished", Bool, queue_size=1, latch=True)
        rospy.Subscriber(cfg.get("pose_topic", "/base_link/odom"), rospy.AnyMsg, self.on_pose, queue_size=5)
        rospy.Subscriber(cfg.get("lidar_topic", "/LIDAR/POINTS"), PointCloud2, self.on_cloud, queue_size=1,
                         buff_size=16 * 1024 * 1024)
        if cfg.get("attitude_topic"):
            rospy.Subscriber(cfg["attitude_topic"], rospy.AnyMsg, self.on_attitude, queue_size=1)
        rospy.Subscriber(cfg.get("gait_topic", "/s10_control/gait"), String, self.on_gait, queue_size=5)
        rospy.Subscriber(cfg.get("control_state_topic", "/s10_control/state"), String, self.on_control, queue_size=5)
        rospy.Subscriber("/rl_nav/cmd", String, self.on_cmd, queue_size=5)
        self.progress_pub.publish(Float32(0.0))
        self.finished_pub.publish(Bool(False))
        d = self.core.describe()
        rospy.loginfo("rl_nav ros1: %s; shadow=%s autostart=%s transform=%s", json.dumps(d), self.shadow,
                      self.running, "identity" if self.transform.identity else "file")
        for w in self.core.warnings:
            rospy.logwarn("%s", w)

    def now(self):
        return time.monotonic() - self.t0

    # ----------------------------------------------------------------- inputs
    def on_pose(self, msg):
        ttype = msg._connection_header["type"]
        if ttype not in POSE_TYPES:
            rospy.logwarn_throttle(10, "pose topic type %s not supported", ttype)
            return
        if self.pose_type != ttype:
            mod, name = POSE_TYPES[ttype]
            self.pose_cls = getattr(__import__(mod, fromlist=[name]), name)
            self.pose_type = ttype
        m = self.pose_cls()
        m.deserialize(msg._buff)
        if ttype == "nav_msgs/Odometry":
            p, q, tw = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist
            v_fwd, yaw_rate = float(tw.linear.x), float(tw.angular.z)
        elif ttype == "geometry_msgs/PoseStamped":
            p, q, v_fwd, yaw_rate = m.pose.position, m.pose.orientation, None, None
        else:
            p, q, v_fwd, yaw_rate = m.pose.pose.position, m.pose.pose.orientation, None, None
        roll, pitch, yaw = core.rpy_from_quat(q.x, q.y, q.z, q.w)
        x, y, z, roll, pitch, yaw = self.transform.pose(float(p.x), float(p.y), float(p.z), roll, pitch, yaw)
        t = self.now()
        with self.lock:
            if self.att is not None and t - self.att_t < self.att_timeout:
                roll, pitch = self.att      # gravity-referenced, from the robot; yaw stays x_nav's
            if self.pose is not None and t - self.pose_t < 1.0:
                dist = math.hypot(x - self.pose[0], y - self.pose[1])
                dyaw = abs(math.degrees(math.atan2(math.sin(yaw - self.pose[5]), math.cos(yaw - self.pose[5]))))
                if (dist > self.pose_jump_m or dyaw > self.pose_jump_deg) and self.jump is None:
                    # localisation jumped (relocalisation / map switch): stop and wait for the operator
                    self.jump = "pose jump %.2f m / %.0f deg" % (dist, dyaw)
                    self.running = False
            if v_fwd is None:
                # No twist in the message: finite-difference the pose (x_nav's PoseStamped case).
                if self.pose is not None and t - self.pose_t > 1e-3:
                    dt = t - self.pose_t
                    dx, dy = x - self.pose[0], y - self.pose[1]
                    v_fwd = (dx * math.cos(yaw) + dy * math.sin(yaw)) / dt
                    yaw_rate = math.atan2(math.sin(yaw - self.pose[5]), math.cos(yaw - self.pose[5])) / dt
                else:
                    v_fwd, yaw_rate = 0.0, 0.0
            self.pose = (x, y, z, roll, pitch, yaw, v_fwd, yaw_rate)
            self.pose_t = t

    def on_attitude(self, msg):
        self.att_n += 1
        if self.att_n % 5:          # 200 Hz topic: 40 Hz is plenty
            return
        m = Imu()
        m.deserialize(msg._buff)
        q = m.orientation
        roll, pitch, _ = core.rpy_from_quat(q.x, q.y, q.z, q.w)
        with self.lock:
            self.att, self.att_t = (roll, pitch), self.now()

    def on_cloud(self, msg):
        with self.lock:
            pose = self.pose
            pose_age = self.now() - self.pose_t
        if pose is None or pose_age > self.pose_timeout:
            return
        t0 = time.monotonic()
        try:
            cloud = core.decode_pointcloud2(msg)
            obs = self.perception.observe(cloud, pose[:6])
        except Exception as e:  # a bad cloud must not stop the node
            rospy.logwarn_throttle(5, "cloud ignored: %s", e)
            return
        with self.lock:
            self.obs, self.obs_t, self.obs_ms = obs, self.now(), (time.monotonic() - t0) * 1e3

    def on_gait(self, msg):
        with self.lock:
            self.gait_reported = msg.data if msg.data in ("flat", "stairs", "fast", "platform") else None
            self.gait_t = self.now()

    def on_control(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            self.control, self.control_t = d, self.now()

    def on_cmd(self, msg):
        c = msg.data.strip().lower()
        with self.lock:
            if c == "start":
                self.running = True
                self.jump = None
            elif c == "pause":
                self.running = False
            elif c == "reset":
                self.running = False
                self.finished = False
                self.jump = None
                bundle = core.RouteBundle.from_dir(self.args_route_dir)
                self.core = core.NavCore(bundle, self.cfg)
                self.progress_pub.publish(Float32(0.0))
                self.finished_pub.publish(Bool(False))
        rospy.loginfo("rl_nav cmd %s -> running=%s", c, self.running)

    # ----------------------------------------------------------------- tick
    def publish(self, cmd, gait_request, status):
        if rospy.is_shutdown():
            return
        try:
            self._publish(cmd, gait_request, status)
        except rospy.ROSException:
            pass  # topics close while the last timer tick is still running

    def _publish(self, cmd, gait_request, status):
        if not self.shadow:
            tw = Twist()
            tw.linear.x, tw.linear.y, tw.angular.z = (float(v) for v in cmd)
            self.cmd_pub.publish(tw)
        if gait_request and not self.shadow:
            self.gait_pub.publish(String(gait_request))
        self.status_pub.publish(String(json.dumps(status)))

    def tick(self, _evt=None):
        try:
            self._tick()
        except Exception as e:  # a bug in a tick must stop the robot, not the node
            with self.lock:
                self.running = False
            rospy.logerr_throttle(2, "tick failed, paused: %r", e)
            self.publish((0.0, 0.0, 0.0), None, dict(t=round(self.now(), 2), mode="HOLD", reason="tick error: %r" % (e,),
                                                      running=False, shadow=self.shadow))

    def _tick(self):
        t = self.now()
        with self.lock:
            pose, pose_age = self.pose, t - self.pose_t
            obs, obs_age = self.obs, t - self.obs_t
            gait = self.gait_reported if t - self.gait_t < 1.0 else None
            ctrl, ctrl_age = self.control, t - self.control_t
            running = self.running and not self.finished
            jump = self.jump
        self.tick_n += 1
        hold = None
        if pose is None or pose_age > self.pose_timeout:
            hold = "no pose"
        elif ctrl and ctrl_age < 2.0 and (ctrl.get("fault") or ctrl.get("latched_stop")):
            hold = "control: " + (ctrl.get("fault") or "latched stop")
        elif jump:
            hold = jump + ' (send "start" to continue)'
        elif not running:
            hold = "finished" if self.finished else "paused"
        base = dict(t=round(t, 2), shadow=self.shadow, running=running, pose_age=round(pose_age, 2) if pose else None,
                    obs_age=round(obs_age, 2) if obs is not None else None, obs_ms=round(self.obs_ms, 1),
                    att="imu" if self.att is not None and t - self.att_t < self.att_timeout else "pose")
        if hold is not None:
            r = self.core.zero(hold, gait)
            st = dict(base, **r.status)
            st.update(mode=r.mode, reached=self.core.reached, total=self.core.n_wp)
            self.publish(r.command, r.gait_request, st)
            self.log_change(hold)
            return
        r = self.core.step(t, pose, obs, gait, obs_age)
        st = dict(base, **r.status)
        self.publish(r.command, r.gait_request, st)
        if t - self.last_log >= 1.0:
            self.last_log = t
            rospy.loginfo("run %s cmd %s s=%s d=%s follower=%s/%s seg_limit=%s plan_scale=%s gait=%s lane=%s%s", r.mode, st.get("cmd"),
                          st.get("s"), st.get("d"), st.get("follower"), st.get("follower_reason"), st.get("speed_limit"),
                          st.get("plan_scale"), gait, st.get("lane"), " blocked=%s" % st.get("blocked") if st.get("blocked") else "")
        if r.reached:
            self.progress_pub.publish(Float32(self.core.reached / self.core.n_wp))
            rospy.loginfo("%s reached (%d/%d)", ", ".join(r.reached), self.core.reached, self.core.n_wp)
        if r.mode != self.last_mode:
            rospy.loginfo("%s -> %s: %s", self.last_mode, r.mode, r.status.get("why", ""))
            self.last_mode = r.mode
        if r.finished and not self.finished:
            self.finished = True
            self.finished_pub.publish(Bool(True))
            rospy.loginfo("route finished")

    def log_change(self, hold):
        if hold != self.last_mode:
            rospy.loginfo("hold: %s", hold)
            self.last_mode = hold


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", required=True)
    ap.add_argument("--route-dir", required=True, help="route_rl.json|route_v2.json [+ maneuvers.json, map_surface.npz]")
    ap.add_argument("--transform", default="", help="JSON with route_from_xnav (overrides nav.yaml)")
    ap.add_argument("--shadow", action="store_true", help="never publish /rl_nav/cmd_vel")
    ap.add_argument("--autostart", action="store_true")
    args, _ = ap.parse_known_args(rospy.myargv(sys.argv)[1:])
    rospy.init_node("rl_nav", disable_signals=False)
    node = Node(args)
    node.args_route_dir = args.route_dir
    timer = rospy.Timer(rospy.Duration(1.0 / node.rate_hz), node.tick)
    rospy.on_shutdown(lambda: node.cmd_pub.publish(Twist()) if not node.shadow else None)
    rospy.spin()
    timer.shutdown()


if __name__ == "__main__":
    main()
