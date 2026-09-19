"""Harness adapter for the real route_v2 follower core (s10_auto_nav.route_follower).

    python -m sim_full_course.harness --controller sim_full_course.route_v2_controller:RouteV2Controller

The harness stands in for NativeGaitRouter: it only executes a gait request while stopped, and
reports the confirmed gait through `on_feedback`. The core returns WAIT_GAIT with a zero command
until the confirmed gait matches its segment, so the settle -> switch -> confirm order is kept.
"""

from __future__ import annotations

import json
import sys
import tempfile
from pathlib import Path

_REPO = Path(__file__).resolve().parents[1]
for _p in (_REPO, _REPO / "src" / "s10_auto_nav"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from s10_auto_nav.route_follower import RouteFollowerCore  # noqa: E402
from s10_auto_nav.route_planner import LocalGridConfig  # noqa: E402

BODY_Z_OFFSET = 0.43  # matches sim_full_course.robot default base height above ground


class RouteV2Controller:
    #: Frames fused into the local grid (0.5 s at 10 Hz). Single frames leave scattered
    #: invalid height cells that block; see docs/ROUTE_V2_PLANNER_ZH.md.
    FUSE_FRAMES = 5

    def __init__(self, body_z_offset: float = BODY_Z_OFFSET, **config_kwargs):
        self.body_z_offset = body_z_offset
        config_kwargs.setdefault("grid", LocalGridConfig(fuse_frames=self.FUSE_FRAMES, max_age=1.0))
        self.config_kwargs = config_kwargs
        self.core: RouteFollowerCore | None = None
        self.fb = {"gait": "flat", "gait_switching": False}

    def reset(self, route) -> None:
        # The harness may hand us a sub-route (scenario windows); write it out so the core loads
        # exactly what the harness scores against.
        with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as f:
            json.dump(route.data, f)
            path = f.name
        self.core = RouteFollowerCore.from_file(
            path, body_z_offset=self.body_z_offset, native=True, **self.config_kwargs
        )

    def on_feedback(self, fb: dict) -> None:
        self.fb = fb

    def step(self, t, pose_xyzyaw, height_grid, valid_mask, scan):
        gait = None if self.fb.get("gait_switching") else self.fb.get("gait")
        out = self.core.step(t, tuple(pose_xyzyaw), height_grid, valid_mask, scan,
                             current_gait=gait)
        vx, vy, wz = out.command
        status = out.status if out.status in ("DONE",) else f"{out.status}:{out.reason or ''}"
        return float(vx), float(vy), float(wz), out.gait_request, status
