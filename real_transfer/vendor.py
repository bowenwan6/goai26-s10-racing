"""Read-only vendor localization status, on the localization board itself."""

from __future__ import annotations

import re
import subprocess
import time
from pathlib import Path

STATUS = re.compile(
    r"^\[([\d-]{10} [\d:]{8})\.(\d{3})\.(\d{3})\].*\[monitor\].*"
    r"上报状态=(\d+)\(([^)]+)\).*运行状态=(全局|局部)"
)


def parse_status(text, now, started_at, session_id, map_id, received):
    """Retain source time; never convert a stale log line to a current heartbeat."""
    for line in reversed(text.splitlines()):
        match = STATUS.search(line)
        if match:
            stamp = time.mktime(time.strptime(match[1], "%Y-%m-%d %H:%M:%S"))
            stamp += int(match[2]) / 1000 + int(match[3]) / 1e6
            if stamp < started_at or not -0.02 <= now - stamp <= 2.0:
                raise ValueError("localization log is stale or predates service")
            return {
                "stamp": stamp,
                "received": received,
                "code": int(match[4]),
                "global": match[6] == "全局",
                "session_id": session_id,
                "map_id": map_id,
            }
    raise ValueError("no localization status")


def read_status():
    """Only `systemctl show`, symlink resolution and bounded log reading; no writes."""
    result = subprocess.run(
        [
            "systemctl",
            "show",
            "localization.service",
            "-p",
            "ActiveState,InvocationID,ExecMainStartTimestampMonotonic",
        ],
        capture_output=True,
        text=True,
        timeout=1,
        check=True,
    )
    fields = dict(line.split("=", 1) for line in result.stdout.splitlines() if "=" in line)
    if fields.get("ActiveState") != "active" or not fields.get("InvocationID"):
        raise ValueError("localization service not active")
    root = Path("/var/opt/robot/data/maps").resolve(strict=True)
    active = (root / "active").resolve(strict=True)
    if active.parent != root or not active.is_dir():
        raise ValueError("active map is outside expected map directory")
    day = time.strftime("%Y_%m%d")
    log = Path("/var/opt/robot/log") / day / f"localization.{day}.log"
    with log.open("rb") as stream:
        stream.seek(max(0, stream.seek(0, 2) - 65536))
        text = stream.read().decode("utf-8", errors="replace")
    wall, mono = time.time(), time.monotonic()
    started = wall - (mono - int(fields["ExecMainStartTimestampMonotonic"]) / 1e6)
    return parse_status(text, wall, started, fields["InvocationID"], active.name, mono)
