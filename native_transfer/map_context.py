"""106-only read-only map/session observer. Publishes diagnostic metadata only."""

import json
import re
import subprocess
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def context():
    active = Path("/var/opt/robot/data/maps/active")
    date = datetime.now().strftime("%Y_%m%d")
    log = Path(f"/var/opt/robot/log/{date}/localization.{date}.log")
    result = subprocess.run(
        ["systemctl", "show", "localization", "--property=InvocationID,ActiveState"],
        capture_output=True,
        text=True,
        timeout=2,
        check=True,
    )
    props = dict(line.split("=", 1) for line in result.stdout.splitlines() if "=" in line)
    last = ""
    if log.exists():
        with log.open("rb") as handle:
            handle.seek(max(0, log.stat().st_size - 65536))
            lines = handle.read().decode(errors="replace").splitlines()
        last = next((line for line in reversed(lines) if "运行状态=" in line), "")
    match = re.match(r"\[(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2})", last)
    age = (
        None
        if not match
        else time.time() - datetime.strptime(match[1], "%Y-%m-%d %H:%M:%S").timestamp()
    )
    return {
        "wall_time": time.time(),
        "map_id": active.resolve().name if active.exists() else None,
        "session_id": props.get("InvocationID"),
        "active": props.get("ActiveState"),
        "global_mode": (
            props.get("ActiveState") == "active"
            and age is not None
            and 0 <= age < 3
            and "运行状态=全局" in last
        ),
        "localization_log_age": age,
        "localization_log": last[:1200],
    }


def main():
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=30)
    args = parser.parse_args()
    rclpy.init(args=[])
    node = Node("native_map_context", enable_rosout=False, start_parameter_services=False)
    pub = node.create_publisher(String, "/native_start_b/map_context", 1)
    end = time.monotonic() + min(1200, max(1, args.duration))
    try:
        while time.monotonic() < end:
            try:
                data = context()
            except (OSError, ValueError, subprocess.SubprocessError) as exc:
                data = {"wall_time": time.time(), "global_mode": False, "error": str(exc)}
            text = json.dumps(data)
            pub.publish(String(data=text))
            print(text, flush=True)
            rclpy.spin_once(node, timeout_sec=1.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
