"""Local app lease: loss of the phone/manager never implicitly re-arms a run."""
import json
import math
from pathlib import Path


class AppControl:
    def __init__(self, path):
        self.path = Path(path)
        self.armed = False
        self.stopped = False

    def poll(self, now):
        if self.stopped:
            return "cancel"
        try:
            value = json.loads(self.path.read_text())
            expiry = value["expires_monotonic"]
            if not isinstance(expiry, (int, float)) or not math.isfinite(expiry) or not 0 < expiry - now <= 6:
                raise ValueError("lease expired or invalid")
            command = value["command"]
            if command not in ("wait", "arm", "cancel"):
                raise ValueError("invalid app command")
        except (OSError, ValueError, KeyError, TypeError):
            command = "cancel"
        if command == "cancel":
            self.stopped = True
            return "cancel"
        if command == "arm" and not self.armed:
            self.armed = True
            return "arm"
        return "wait"
