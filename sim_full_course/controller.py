"""Controller interface used by the harness + a trivial pure-pursuit reference.

Interface (duck-typed; subclassing `Controller` is optional):

    controller.reset(route: Route) -> None                       # before the run
    controller.on_feedback(fb: dict) -> None                     # OPTIONAL, every tick first
        fb = {"gait": "flat"|"stairs", "gait_switching": bool}   # native gait receipt only
    controller.step(t, pose_xyzyaw, height_grid, valid_mask, scan)
        -> (vx, vy, yaw_rate, gait_request, status)

      t            float s since start (sim clock, 10 Hz ticks by default)
      pose_xyzyaw  (x, y, z, yaw) map frame; z = BASE height (ground + body_z_offset); may
                   carry configured pose noise
      height_grid  (13, 9) float, real_transfer.geometry.height_grid values: highest return
                   per 0.15 m cell, robot yaw frame, X -0.6..1.2 (rows), Y -0.6..0.6 (cols),
                   z relative to the BASE origin (flat ground ~ -0.43); -1 where invalid
      valid_mask   (13, 9) bool (False = unknown / mixed level / clipped)
      scan         (72,) float ranges, native_transfer.contracts.conservative_scan: bin i
                   covers angle [-pi + i*5deg, -pi + (i+1)*5deg) in the yaw frame
                   (bin 36 starts at 0 = straight ahead); NaN = unknown
      vx, vy, yaw_rate  body-frame command; the harness clamps to native limits + slew
      gait_request "flat" | "stairs" | None (None = keep). A change is executed only while
                   stationary (the harness zeroes motion until the switch completes)
      status       free string; "DONE" or "ABORT" ends the run, e.g. "RUNNING", "BLOCKED"
"""

from __future__ import annotations

import math

import numpy as np

from sim_full_course.route import Route


class Controller:
    def reset(self, route: Route) -> None:
        self.route = route

    def on_feedback(self, fb: dict) -> None:
        self.fb = fb

    def step(self, t, pose_xyzyaw, height_grid, valid_mask, scan):
        raise NotImplementedError


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class PurePursuitController(Controller):
    """Reference only: follows the route centerline, no detours.

    - lookahead 0.5 m; pivot in place when |heading error| > 10 deg; speed falls off to 0 at 30
      deg (sec. 8.5 follower settings)
    - stops at a gait boundary (the segment's `from` WP), requests the new gait, waits for the
      receipt, then continues
    - flat gait: stops with status BLOCKED if a known scan range ends inside the 0.52 m wide
      lane within stop_range ahead (it does NOT detour; that's the planner's job); in stairs
      gait the scan is ignored (steps themselves fall in the -0.25..0.75 m obstacle band)
    """

    def __init__(self, lookahead=0.5, pivot_deg=10.0, falloff_deg=30.0, k_yaw=1.2,
                 k_lat=0.4, stop_range=0.75, lane_half_width=0.26, finish_tol=0.15):
        self.lookahead = lookahead
        self.pivot = math.radians(pivot_deg)
        self.falloff = math.radians(falloff_deg)
        self.k_yaw = k_yaw
        self.k_lat = k_lat
        self.stop_range = stop_range
        self.lane_half_width = lane_half_width
        self.finish_tol = finish_tol
        self.fb = {"gait": "flat", "gait_switching": False}

    def reset(self, route: Route) -> None:
        self.route = route
        self.s = None
        self.boundaries = route.gait_boundaries()
        self.done_boundaries = set()
        self.gait = route.segments[0]["gait"]

    def step(self, t, pose_xyzyaw, height_grid, valid_mask, scan):
        r = self.route
        x, y, _, yaw = pose_xyzyaw
        self.s, lat, _ = r.project((x, y), self.s, back=0.5, ahead=2.0)
        s = self.s
        # Finish.
        end_p, _ = r.point_at(r.length)
        if r.length - s < self.finish_tol + 0.1 and math.hypot(x - end_p[0], y - end_p[1]) < \
                self.finish_tol:
            return 0.0, 0.0, 0.0, None, "DONE"
        # Gait boundary: stop at the from-WP and switch.
        for i, (sb, wid, gait) in enumerate(self.boundaries):
            if i in self.done_boundaries:
                continue
            if s >= sb - 0.05:
                if self.fb.get("gait") == gait and not self.fb.get("gait_switching"):
                    self.done_boundaries.add(i)
                    self.gait = gait
                    continue
                return 0.0, 0.0, 0.0, gait, f"SWITCH_GAIT@{wid}"
            break
        seg = r.segment_at(s + 0.01)
        vmax = float(seg["speed_limit"])
        # Next boundary: do not run past it before switching.
        nxt = next((sb for i, (sb, _, _) in enumerate(self.boundaries)
                    if i not in self.done_boundaries), None)
        s_target = s + self.lookahead
        if nxt is not None:
            s_target = min(s_target, nxt)
            vmax = min(vmax, max(0.03, 0.8 * (nxt - s)))
        tp, _ = r.point_at(s_target)
        err = _wrap(math.atan2(tp[1] - y, tp[0] - x) - yaw)
        if math.hypot(tp[0] - x, tp[1] - y) < 0.03:
            err = 0.0
        if self.fb.get("gait", "flat") == "flat" and scan is not None:
            # Scan bins whose range endpoint lies inside the body-width lane ahead.
            ang = -math.pi + (np.arange(72) + 0.5) * (2 * math.pi / 72)
            fwd, side = scan * np.cos(ang), scan * np.sin(ang)
            lane = np.isfinite(scan) & (fwd > 0) & (fwd < self.stop_range) & (
                np.abs(side) < self.lane_half_width)
            if lane.any():
                return 0.0, 0.0, 0.0, None, "BLOCKED"
        if abs(err) > self.pivot:
            return 0.0, 0.0, float(np.sign(err)) * 0.2, None, "PIVOT"
        vx = vmax * max(0.0, 1.0 - abs(err) / self.falloff)
        wz = self.k_yaw * err
        vy = -self.k_lat * lat
        return vx, vy, wz, None, "RUNNING"
