#!/usr/bin/env python3
"""Short single-model SDK/MuJoCo wiring check, including a reset while RL is active.

Run after `source install/setup.bash`, using a Python with MuJoCo and ROS bindings.
This is an interface check, not a policy performance evaluation.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=Path("results/him-sim"))
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    model = args.model.resolve()
    config = json.loads(model.with_suffix(".json").read_text())
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    # Keep the check isolated from hardware and unrelated ROS sessions.
    os.environ.update(ROS_DOMAIN_ID="84", ROS_LOCALHOST_ONLY="1", S10_POLICY_PATH=str(model),
                      S10_SECOND_POLICY_PATH="", S10_DOWN_POLICY_PATH="", S10_SPEEDTURN_POLICY_PATH="",
                      S10_POLICY_SLOT="0", S10_USE_VIEWER="0", S10_USE_PERCEPTION="0",
                      S10_USE_LIDAR="0", S10_MANUAL_PORT="18784", S10_HIGH_SPEED="0",
                      S10_POLICY_TRACE_PATH=str(output / "policy_trace.jsonl"), S10_POLICY_TRACE_STEPS="1500")
    os.environ.pop("S10_MANUAL", None)
    os.environ["S10_UPSTREAM_DIR"] = str(root / "upstream/goai_embodied_future_material")
    import rclpy
    from drdds.msg import JointsDataCmd
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Empty, UInt8

    rclpy.init()
    node = rclpy.create_node("him_sim_check")
    state = {"mode": 0, "matching_pd_commands": 0, "odom_messages": 0, "reset_done": 0}

    def joints(msg):
        values = msg.data.joints_data
        assert len(values) == 16
        assert all(math.isfinite(v) for j in values for v in (j.position, j.velocity, j.kp, j.kd, j.torque))
        if state["mode"] == 6 and all(abs(j.kp - config["p_gains"][i]) < 1e-5 and
                                        abs(j.kd - config["d_gains"][i]) < 1e-5 for i, j in enumerate(values)):
            state["matching_pd_commands"] += 1

    def odom(msg):
        pos = msg.pose.pose.position
        assert all(math.isfinite(v) for v in (pos.x, pos.y, pos.z))
        state["odom_messages"] += 1
        state["last_position"] = [pos.x, pos.y, pos.z]

    node.create_subscription(UInt8, "/robot_state", lambda msg: state.update(mode=msg.data), 10)
    node.create_subscription(JointsDataCmd, "/JOINTS_CMD", joints, 10)
    node.create_subscription(Odometry, "/ground_truth/odom", odom, 10)
    node.create_subscription(Empty, "/sim/reset_done", lambda msg: state.update(reset_done=state["reset_done"] + 1), 10)
    command_pub = node.create_publisher(Twist, "/cmd_vel", 10)
    reset_pub = node.create_publisher(Empty, "/sim/reset", 10)
    processes, logs = [], []
    try:
        for name, cmd in (
            ("policy", [root / "install/s10_sdk_deploy/lib/s10_sdk_deploy/rl_deploy"]),
            ("sim", [sys.executable, "-m", "s10_perception.sim_node"]),
        ):
            log = (output / f"{name}.log").open("w")
            logs.append(log)
            processes.append(subprocess.Popen([str(x) for x in cmd], cwd=root, stdout=log,
                                              stderr=subprocess.STDOUT, start_new_session=True))
        started = time.monotonic()
        reset_at = None
        while time.monotonic() - started < 45:
            assert all(p.poll() is None for p in processes), "SDK/simulator exited; inspect output logs"
            rclpy.spin_once(node, timeout_sec=.02)
            command = Twist()
            command.linear.x = .1
            command_pub.publish(command)
            if state["matching_pd_commands"] >= 80 and reset_at is None:
                reset_pub.publish(Empty())
                reset_at = time.monotonic()
            if reset_at and time.monotonic() - reset_at > 3 and state["matching_pd_commands"] >= 180:
                break
        assert state["matching_pd_commands"] >= 180 and state["odom_messages"] > 0
        assert state["reset_done"] == 1, "sim reset acknowledgement missing"
    finally:
        for p in processes:
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
        for p in processes:
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
                p.wait()
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.shutdown()

    records = [json.loads(line) for line in (output / "policy_trace.jsonl").read_text().splitlines()]
    fresh = []
    for i, record in enumerate(records):
        obs = record["observation"]
        assert len(obs) == 342 and all(math.isfinite(x) for x in obs)
        if all(x == 0 for x in obs[57:]):
            assert all(x == 0 for x in obs[41:57])
            fresh.append(i)
        elif i:
            assert obs[57:] == records[i - 1]["observation"][:285]
            assert obs[41:57] == records[i - 1]["clipped_action_policy"]
    assert len(fresh) >= 2, "entry and in-RL reset must both produce zero old history"
    state.update(passed=True, policy_frames=len(records), zero_history_frames=fresh,
                 purpose="interface/closed-loop smoke; no performance A/B", model=str(model))
    (output / "verification.json").write_text(json.dumps(state, indent=2) + "\n")
    print(json.dumps(state, indent=2))


if __name__ == "__main__":
    main()
