"""Who drives, and with what command, along an RL route: the walking actor by default, the stairs
actor through the manoeuvres, every switch gated on what the robot measures there and then.

Inputs are only what the robot has: localisation pose, IMU attitude and yaw rate, measured forward
speed, the 13x9 height grid and LiDAR the follower gets, the follower's own output and local grid,
and the route. The map enters through the manoeuvre list (``maneuvers.py``: where along the route a
climb is expected, and a prior for its first edge) and, optionally, its surface (``map_check``: to
tell terrain the map knows from something new). Output is a body-velocity command and the
joint-owner request the SDK understands (``official`` = walking actor, ``stairs_stable`` = stairs
actor, ``stop``).

The nominal behaviour is the first (simulation) version's: follow the route, hand over to the stairs
actor at each manoeuvre, hand back after it. Everything else only acts when something is off.

Modes::

    NAVIGATE      The route follower drives the walking actor; the forward command is held at
                  walk_floor or more (it stalls on any bump below ~0.5 m/s). If the follower
                  refuses (BLOCKED / OFF_CORRIDOR): FOLLOW_ROUTE when on the route and nothing new
                  is in the way, else REJOIN.
    FOLLOW_ROUTE  Pure pursuit on the route itself at walking speed, lateral error strafed back --
                  what the first version did whenever the planner refused (its grid reads rough
                  ground as blocked; the route is where the mapping robot drove). Left for REJOIN
                  when something new is in the way; never forward onto a drop or a hole.
    REJOIN        A* on the follower's local grid back onto the route ahead. When nothing seen
                  leads back: turn to look, then pursue the route if nothing is in the way; an
                  obstacle with no way round: wait for it to move, then HOLD.
    APPROACH      A manoeuvre is near: the follower drives, capped; if it refuses, A* to the entry
                  point, else the route itself.
    ALIGN         The walking actor turns to the route's heading at the entry -- refined by the
                  MEASURED edge when that agrees with the plan (squares up to the riser actually
                  there, cancels a localisation yaw error) -- strafes back onto the route and creeps
                  into the entry band. The switch needs, held for ready_dwell: the measured edge in
                  align_band, heading within align_heading_tol (align_heading_soft after a while),
                  yaw rate and speed low. The SDK takes the moving handover in one tick. A slope
                  (nothing to measure) is climbed from where the robot is when ALIGN times out.
    CLIMB         The stairs actor follows the route (a carrot on it, as in the first version),
                  never more than max_skew_hard off the risers it measures, biased to the wider side
                  near a drop, slowed near a waypoint and when faster than asked, stopped short of a
                  drop or a hole. Wedged: back off and retry, leaning towards the route. A waypoint
                  run past (by the robot's own projection) is left for RECOVER_GATE, never turned
                  back to on the steps. Leaves for VERIFY past the last expected edge once no edge
                  is measured under the body.
    DESCEND       A ledge to step off: walking actor, square to the measured edge, walking speed.
    VERIFY        Level (pitch, roll < 5 deg), no edge under the body, held 0.4 s.
    HANDBACK      Request the walking actor; the SDK holds the joints 0.25 s and resets it.
    RECOVER_GATE  The stairs actor ran past a waypoint it did not score: go back for it walking.
    HOLD          Stopped, reason given: the route needs an operator (or, for stale input, data).
    ABORT         Tilt beyond fall_tilt: owner ``stop``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum

import numpy as np

from s10_auto_nav.rl_nav import local_astar
from s10_auto_nav.rl_nav.edge_tracker import EdgeTracker, detect_edge, side_margins, wrap
from s10_auto_nav.rl_nav.map_check import unexpected_ahead

MOVING = {"RUNNING", "DETOUR", "ASTAR", "GATE"}


class Mode(Enum):
    NAVIGATE = "NAVIGATE"
    FOLLOW_ROUTE = "FOLLOW_ROUTE"
    REJOIN = "REJOIN"
    APPROACH = "APPROACH"
    ALIGN = "ALIGN"
    CLIMB = "CLIMB"
    DESCEND = "DESCEND"
    VERIFY = "VERIFY"
    HANDBACK = "HANDBACK"
    RECOVER_GATE = "RECOVER_GATE"
    HOLD = "HOLD"
    ABORT = "ABORT"
    DONE = "DONE"


@dataclass
class RouterParams:
    walk_v: float = 0.6
    walk_floor: float = 0.5
    #: Approach: from approach_dist before the first mark, the follower drives at up to approach_v.
    #: The first version approached over 1.2 m; longer only costs time (ALIGN needs the edge
    #: within ~1 m anyway), and under 0.5 m/s the walking actor stalls on bumps.
    approach_v: float = 0.5
    approach_dist: float = 1.6
    align_enter: float = 1.0
    #: Distance from the base to the measured edge at which the stairs actor may take over. The near
    #: end is the front wheels touching the riser (~0.42 m), which is a fine place to start climbing
    #: from.
    align_band: tuple = (0.35, 0.70)
    align_heading_tol: float = math.radians(8.0)
    #: After align_soft_after s in ALIGN the heading tolerance relaxes to this: on a slope the
    #: walking actor barely pivots, and a few degrees matter less than standing there.
    align_heading_soft: float = math.radians(15.0)
    align_soft_after: float = 6.0
    align_pivot: float = math.radians(12.0)
    align_yaw_rate: float = 0.15
    align_speed: float = 0.30
    align_creep_v: float = 0.20
    align_back_v: float = 0.12
    align_through_v: float = 0.12
    align_min_yaw_rate: float = 0.3
    ready_dwell: float = 0.3
    align_timeout: float = 15.0
    climb_v: float = 0.30
    climb_turn_v: float = 0.20
    climb_slow_v: float = 0.15
    climb_w: float = 0.35
    #: How far the climbing heading may leave the route's direction to steer back onto it (the
    #: first version's 12 deg).
    climb_correction: float = math.radians(12.0)
    #: Heading control on the stairs actor. Damping the yaw rate (climb_kd) cut an overshoot on
    #: WP11's rock crest but weakened the corrections it needs against its left drift on the B
    #: flights, where it then slid along a riser and off an edge; off by default.
    climb_kp: float = 2.0
    climb_kd: float = 0.0
    #: A drop this close beside the body while climbing: stop, turn away from it. The stairs actor
    #: slides sideways along a riser it pushes against at an angle, which the look-ahead cannot see.
    side_stop: float = 0.12
    #: The skew the route preparation plans with (policy profile); informational here.
    max_skew: float = math.radians(25.0)
    #: Hard limit on the angle to the measured risers while aligning and climbing: the route sets
    #: the heading, this only stops the robot going side-on to the steps. The first version's lower
    #: B flight is crossed at up to ~50 deg by a line the stairs actor holds.
    max_skew_hard: float = math.radians(55.0)
    #: A measured edge within this of the plan refines the entry heading; beyond it the route
    #: decides.
    agree_tol: float = math.radians(12.0)
    #: ALIGN strafes back onto the route (walking actor) while it creeps, at most this fast.
    align_strafe: float = 0.12
    climb_gate_slow: float = 1.2
    climb_timeout: float = 90.0
    climb_stall: float = 15.0
    #: Wedged (a knee on a rock, a wheel against a riser): no progress for climb_stuck s -> back off
    #: for backoff_time s, then carry on with the heading biased towards the route by retry_bias for
    #: retry_reach m; climb_retries times, then HOLD.
    climb_stuck: float = 4.0
    backoff_v: float = 0.12
    backoff_time: float = 1.5
    retry_bias: float = math.radians(12.0)
    retry_reach: float = 1.5
    climb_retries: int = 3
    #: Sliding: pressed against a riser at an angle, the stairs actor can slide sideways along it
    #: instead of climbing it (and off the end of the tread). Moving sideways faster than slide_v
    #: while making less than slide_fwd forward, for slide_time s: stop pushing and square up to
    #: the riser; if it will not turn, back off first. Counts as a retry.
    slide_v: float = 0.08
    slide_fwd: float = 0.05
    slide_time: float = 1.0
    square_time: float = 2.5
    centre_trigger: float = 0.25
    centre_bias: float = math.radians(5.0)
    centre_strafe: float = 0.15
    exit_past_last: float = 0.3
    verify_hold: float = 0.4
    verify_level: float = math.radians(5.0)
    handback_hold: float = 0.5
    rejoin_after: float = 1.0
    rejoin_v: float = 0.45
    rejoin_replan: float = 1.0
    rejoin_timeout: float = 25.0
    #: FOLLOW_ROUTE: only from within this of the route, with the scan clear this far along it.
    trust_offset: float = 0.4
    trust_reach: float = 1.6
    follow_lateral: float = 0.25
    follow_min_progress: float = 1.0
    #: An obstacle on the route that A* cannot pass: wait this long (people move), then HOLD.
    obstacle_wait: float = 10.0
    missed_gate_margin: float = 0.3
    drop_depth: float = 0.3
    drop_cells: int = 3
    #: A drop or a hole ahead of the stairs actor: stop, keep turning towards the aim, HOLD if it
    #: stays.
    drop_hold: float = 4.0
    fall_tilt: float = math.radians(45.0)
    grid_stale: float = 0.5
    edge_gate_distance: float = 0.5
    edge_gate_angle: float = math.radians(20.0)
    #: Manoeuvres whose first edge the map could not fit: the measured edge is still required, gated
    #: against the route itself (its point at the first edge, its tangent) with this much slack.
    route_gate_distance: float = 0.8
    #: ALIGN does not start before the last bend sharper than this ahead of the entry.
    align_bend: float = math.radians(20.0)
    #: An edge more than this after a manoeuvre's first mark does not gate its entry.
    edge_start_gap: float = 0.8
    route_gate_angle: float = math.radians(65.0)

    @classmethod
    def from_profile(cls, profile, **overrides):
        """Speeds and skews from the policy profile (capability.PolicyProfile); rest as given."""
        kw = {
            "walk_floor": profile.walk.min_speed or cls.walk_floor,
            "max_skew": math.radians(profile.climb.max_entry_skew_deg),
        }
        kw.update(overrides)
        return cls(**kw)


@dataclass
class NavInput:
    t: float
    x: float
    y: float
    z: float
    yaw: float
    pitch: float
    roll: float
    yaw_rate: float
    v_forward: float
    grid: np.ndarray | None
    valid: np.ndarray | None
    grid_time: float
    follower: object  # RouteFollowerCore.step() output
    local_grid: object | None = None  # follower.last_grid
    owner_feedback: str | None = None  # SDK /joints/owner
    scan: np.ndarray | None = None  # 72-bin conservative scan (robot yaw frame), NaN = no returns
    points: np.ndarray | None = None  # LiDAR points, robot yaw frame relative to the base (N x 3)


@dataclass
class NavOutput:
    command: tuple
    owner: str
    mode: Mode
    reason: str = ""
    info: dict = field(default_factory=dict)


class ManeuverRouter:
    def __init__(self, path, maneuvers, params: RouterParams | None = None, map_surface=None):
        self.path = path
        self.p = params or RouterParams()
        #: The map's surface (map_check.MapSurface): tells terrain the map knows from new obstacles.
        self.map_surface = map_surface
        self.mans = sorted(maneuvers, key=lambda m: m.s0)
        self.k = 0
        self.mode, self.mode_t = Mode.NAVIGATE, 0.0
        self.log: list[dict] = []
        self.tracker: EdgeTracker | None = None
        self.ready_since = None
        self.refuse_since = None
        self.astar_path, self.astar_t = None, -math.inf
        self.climb_normal = None
        self.climb_start_s, self.climb_best_s, self.climb_best_t = 0.0, 0.0, 0.0
        self.retries, self.backoff_until, self.retry_bias, self.retry_until_s = (
            0,
            None,
            0.0,
            -math.inf,
        )
        self.track: list = []  # (t, x, y) while climbing, for the slide check
        self.slide_since, self.square_until = None, None
        self.verify_since = None
        self.pending_gate = None
        self.missed: list[dict] = []
        self.last_s = 0.0
        self.proj_s = None  # the robot's own projection on the route (runs backwards too)
        self.drop_since = None
        self.yaw_rate_f = 0.0  # low-passed: a walking gait stepping in place wobbles the body
        self.blocked_since = None
        self.follow_from_s = None
        self.align_from_s = -math.inf

    # ------------------------------------------------------------------ helpers
    def _set(self, mode, t, s, why=""):
        self.log.append(
            {
                "t": round(t, 2),
                "s": round(s, 2),
                "from": self.mode.value,
                "to": mode.value,
                "maneuver": self.mans[self.k].id if self.k < len(self.mans) else None,
                "why": why,
            }
        )
        self.mode, self.mode_t = mode, t
        if mode == Mode.FOLLOW_ROUTE:
            self.follow_from_s = None
        self.ready_since = None
        self.verify_since = None

    def _man(self):
        return self.mans[self.k] if self.k < len(self.mans) else None

    def _tangent(self, s):
        return float(np.atleast_1d(self.path.tangent_yaw_at(min(max(s, 0.0), self.path.length)))[0])

    def _edge_frame(self, inp, m):
        """(distance, lateral, heading error, normal_yaw, measured) for the manoeuvre's first
        edge."""
        if self.tracker is not None:
            est = self.tracker.update(inp.t, (inp.x, inp.y), inp.yaw, inp.grid, inp.valid)
            d, lat, head = EdgeTracker.coordinates(est, (inp.x, inp.y), inp.yaw)
            return d, lat, head, est.normal_yaw, est.measured
        # Nothing to measure (a slope, a side slope): distance along the route from the robot's own
        # projection -- not the follower's progress, which never runs backwards.
        normal = self._tangent(m.s_first + 0.3)
        pr = self.path.project((inp.x, inp.y), s_hint=self.last_s, window=(2.0, 2.0))
        return m.s_first - pr.s, pr.d, wrap(inp.yaw - normal), normal, False

    def _slide_check(self, inp, m, s):
        """While climbing: sliding sideways along a riser (sideways motion, no forward motion,
        over the last second) -> stop pushing and square up to the riser; if it will not turn,
        back off first. Returns the command to use instead of climbing, or None."""
        p = self.p
        self.track = [q for q in self.track if inp.t - q[0] <= 1.0] + [(inp.t, inp.x, inp.y)]
        if len(self.track) >= 3 and self.track[-1][0] - self.track[0][0] >= 0.6:
            t0, x0, y0 = self.track[0]
            dt_ = max(inp.t - t0, 1e-3)
            dx, dy = inp.x - x0, inp.y - y0
            v_fwd = (dx * math.cos(inp.yaw) + dy * math.sin(inp.yaw)) / dt_
            v_lat = (-dx * math.sin(inp.yaw) + dy * math.cos(inp.yaw)) / dt_
            sliding = abs(v_lat) > p.slide_v and v_fwd < p.slide_fwd
            if not sliding:
                self.slide_since = None
            elif self.slide_since is None:
                self.slide_since = inp.t
        if (
            self.slide_since is not None
            and inp.t - self.slide_since >= p.slide_time
            and self.square_until is None
        ):
            self.retries += 1
            self.square_until = inp.t + p.square_time
            self.slide_since = None
            self.log.append(
                {
                    "t": round(inp.t, 2),
                    "s": round(s, 2),
                    "from": "CLIMB",
                    "to": "CLIMB",
                    "maneuver": m.id,
                    "why": f"sliding along a riser: squaring up, retry {self.retries}",
                }
            )
        if self.square_until is None:
            return None
        err = wrap(self.climb_normal - inp.yaw)
        if abs(err) < math.radians(8.0):
            self.square_until = None
            self.climb_best_t = inp.t
            return None
        if inp.t < self.square_until:
            wz = float(np.clip(2.0 * err, -p.climb_w, p.climb_w))
            return NavOutput((0.0, 0.0, wz), "stairs_stable", self.mode, "squaring up")
        # It will not turn while pressed on the riser: back off, then square up again.
        self.square_until = None
        self.backoff_until = inp.t + p.backoff_time
        return NavOutput((-p.backoff_v, 0.0, 0.0), "stairs_stable", self.mode, "backing off")

    def _last_bend_before(self, m):
        """Arc length from which ALIGN may start: past the last bend (over align_bend) of the route
        before the manoeuvre's entry. ALIGN creeps straight at the edge; before a corner that would
        cut it -- onto the part of the edge the route was drawn to avoid. Never later than the far
        end of the entry band: a bend right at the edge (turn on the boulder top, then step off) is
        ALIGN's own turn on the spot."""
        entry = self._tangent(m.s_first + 0.3)
        s_from = m.s_first - self.p.approach_dist
        for sv in np.arange(m.s_first, max(0.0, m.s_first - self.p.approach_dist), -0.1):
            if abs(wrap(self._tangent(sv) - entry)) > self.p.align_bend:
                s_from = sv + 0.3
                break
        return min(s_from, m.s_first - self.p.align_band[1] - 0.1)

    def _make_tracker(self, m):
        want = "down" if m.policy == "walk_descend" else "up"
        s_edge = getattr(m, "s_edge", None)
        if s_edge is not None and s_edge - m.s_first > self.p.edge_start_gap:
            # It starts on a slope or side slope and the edge comes later: hand over at the start,
            # on the route, as for a slope (the walking actor slides on side slopes).
            return None
        if m.has_edge:
            return EdgeTracker(
                m.prior_point,
                m.prior_normal_yaw,
                m.prior_height,
                self.p.edge_gate_distance,
                self.p.edge_gate_angle,
                want=want,
            )
        if ("edge_down" if want == "down" else "edge_up") in m.kinds:
            # The map saw an edge here but could not fit a line to it: gate on the route instead.
            point = np.asarray(self.path.point_at(m.s_first))[:2]
            return EdgeTracker(
                point,
                self._tangent(m.s_first),
                0.0,
                self.p.route_gate_distance,
                self.p.route_gate_angle,
                want=want,
            )
        return None

    def _shaped_walk(self, f):
        vx, vy, wz = f.command
        if f.status not in MOVING:
            return (0.0, 0.0, 0.0)
        if vx > 0.25:
            vx = max(vx, self.p.walk_floor)
        return (min(vx, self.p.walk_v), vy, wz)

    def _drop_ahead(self, inp, s):
        """A drop or a hole in the body's path 0.45-1.05 m ahead. A hole is a column of the grid
        unknown for two rows running (a riser blanks one row, not two), counted only where the route
        is not going down (going down, the treads below are hidden anyway)."""
        if inp.grid is None:
            return False
        g = np.asarray(inp.grid, float).reshape(13, 9)
        v = np.asarray(inp.valid, bool).reshape(13, 9)
        under = g[3:6, 2:7][v[3:6, 2:7]]
        ref = float(np.median(under)) if under.size else -0.42
        zs = self.path.points[:, 2]
        expected = float(np.interp(s + 0.75, self.path.s, zs) - np.interp(s, self.path.s, zs))
        low = (g[7:12, 2:7] < ref + min(expected, 0.0) - self.p.drop_depth) & v[7:12, 2:7]
        n = int(low.sum())
        if expected > -0.05:
            hole = ~v[7:12, 2:7]
            n += int((hole[1:] & hole[:-1]).sum())
        return n >= self.p.drop_cells

    def _scan_blocks(self, inp, s_from, reach=None, half_width=0.3):
        """Distance to something in the route's corridor within ``reach`` ahead of ``s_from`` that
        the robot should not drive into, or None.

        With the map surface and the LiDAR points: only returns standing more than 0.2 m above the
        mapped surface count (map_check) -- rising terrain the map knows does not. Without them: the
        first body-height return of the conservative scan inside the corridor, which also counts
        rising terrain (conservative). Bins with no returns do not block: the height grid's drop
        check covers holes and drops."""
        reach = self.p.trust_reach if reach is None else reach
        if self.map_surface is not None and inp.points is not None:
            return unexpected_ahead(
                inp.points,
                (inp.x, inp.y, inp.z, inp.yaw),
                self.path,
                s_from,
                self.map_surface,
                reach=reach,
                half_width=half_width + 0.05,
            )
        if inp.scan is None:
            return None
        r = np.asarray(inp.scan, float)
        n = len(r)
        c, s_ = math.cos(inp.yaw), math.sin(inp.yaw)
        for ds in np.arange(0.3, reach + 1e-9, 0.2):
            q = np.asarray(self.path.point_at(min(self.path.length, s_from + ds)))[:2]
            dx, dy = q[0] - inp.x, q[1] - inp.y
            fx, fy = c * dx + s_ * dy, -s_ * dx + c * dy
            dist = math.hypot(fx, fy)
            if dist < 0.3:
                continue
            bearing = math.atan2(fy, fx)
            spread = math.atan2(half_width, dist)
            lo = math.floor((bearing - spread + math.pi) / (2 * math.pi) * n)
            hi = math.floor((bearing + spread + math.pi) / (2 * math.pi) * n)
            for i in range(lo, hi + 1):
                ri = r[i % n]
                if np.isfinite(ri) and ri < dist - 0.15:
                    return float(ri)
        return None

    def _pursue_route(self, inp, s_here, s_gate, v):
        """The first version's pursuit on the route: carrot 1 m ahead, never past the unscored
        waypoint; turn in place beyond 35 deg; lateral error strafed back."""
        carrot = np.asarray(
            self.path.point_at(min(s_here + 1.0, self.path.length, max(s_gate, s_here + 0.3)))
        )
        err = wrap(math.atan2(carrot[1] - inp.y, carrot[0] - inp.x) - inp.yaw)
        wz = float(np.clip(1.5 * err, -0.6, 0.6))
        if abs(err) > math.radians(35):
            return (0.0, 0.0, wz)
        pr = self.path.project((inp.x, inp.y), s_hint=s_here, window=(1.0, 1.0))
        vy = float(np.clip(-1.0 * pr.d, -self.p.follow_lateral, self.p.follow_lateral))
        # No slowing for the heading error: under walk_floor the walking actor stalls on any bump.
        return (v, vy, wz)

    def _astar_cmd(self, inp, s, goal_s=None, v=None):
        v = self.p.rejoin_v if v is None else v
        if inp.local_grid is None:
            return (0.0, 0.0, 0.0), "no local grid"
        if self.astar_path is None or inp.t - self.astar_t > self.p.rejoin_replan:
            ahead = (
                (0.8, 3.0) if goal_s is None else (max(0.0, goal_s - s), max(0.0, goal_s - s) + 0.3)
            )
            self.astar_path = local_astar.plan(
                inp.local_grid, (inp.x, inp.y, inp.yaw), self.path, s, ahead
            )
            self.astar_t = inp.t
        if self.astar_path is None:
            # Nothing seen leads back -- typically the route ahead lies outside the grid (it covers
            # 1.2 m ahead and 0.6 m to each side): turn on the spot towards the route until it is in
            # view.
            base = self.proj_s if self.proj_s is not None else s
            tgt = np.asarray(self.path.point_at(min(self.path.length, base + 1.0)))[:2]
            err = wrap(math.atan2(tgt[1] - inp.y, tgt[0] - inp.x) - inp.yaw)
            if abs(err) > math.radians(10.0):
                wz = float(np.sign(err) * max(self.p.align_min_yaw_rate, min(0.5, 1.5 * abs(err))))
                return (0.0, 0.0, wz), "A* found no way back: turning to look"
            return (0.0, 0.0, 0.0), "A* found no way back"
        return local_astar.pursue_path(self.astar_path, (inp.x, inp.y, inp.yaw), v), "A*"

    # ------------------------------------------------------------------ tick
    def step(self, inp: NavInput) -> NavOutput:
        p, f = self.p, inp.follower
        s = float(f.s) if math.isfinite(f.s) else self.last_s
        self.last_s = s
        # The follower's progress stops at a waypoint it has not scored; this one does not, so a
        # waypoint run past is noticed and the carrot is not pinned behind the robot.
        hint = s if self.proj_s is None else self.proj_s
        self.proj_s = float(self.path.project((inp.x, inp.y), s_hint=hint, window=(1.0, 2.0)).s)
        self.yaw_rate_f += 0.3 * (inp.yaw_rate - self.yaw_rate_f)
        s_gate = (
            float(self.path.waypoint_s[min(f.target_index, len(self.path.waypoint_s) - 1)])
            if not f.finished
            else self.path.length
        )
        tilt = math.acos(max(-1.0, min(1.0, math.cos(inp.pitch) * math.cos(inp.roll))))
        owner_climb = self.mode in (Mode.CLIMB, Mode.VERIFY)

        if self.mode == Mode.ABORT or tilt > p.fall_tilt:
            if self.mode != Mode.ABORT:
                self._set(Mode.ABORT, inp.t, s, f"tilt {math.degrees(tilt):.0f} deg")
            return NavOutput((0.0, 0.0, 0.0), "stop", self.mode, "abort")
        if f.finished:
            if self.mode != Mode.DONE:
                self._set(Mode.DONE, inp.t, s, "all waypoints reached")
            return NavOutput((0.0, 0.0, 0.0), "official", self.mode, "done")
        if inp.grid is None or inp.t - inp.grid_time > p.grid_stale:
            return NavOutput(
                (0.0, 0.0, 0.0),
                "stairs_stable" if owner_climb else "official",
                self.mode,
                "height grid stale",
            )
        if self.mode == Mode.HOLD:
            return NavOutput(
                (0.0, 0.0, 0.0), "official", self.mode, self.log[-1]["why"] if self.log else "hold"
            )

        m = self._man()
        while (
            m is not None
            and self.mode in (Mode.NAVIGATE, Mode.FOLLOW_ROUTE, Mode.REJOIN)
            and s > m.s1
        ):
            self.k += 1
            m = self._man()

        # --- NAVIGATE / FOLLOW_ROUTE / REJOIN
        # ---------------------------------------------------------
        walking = (Mode.NAVIGATE, Mode.FOLLOW_ROUTE, Mode.REJOIN)
        if self.mode in walking and m is not None and s >= m.s_first - p.approach_dist:
            self.tracker = self._make_tracker(m)
            self.align_from_s = self._last_bend_before(m)
            self._set(Mode.APPROACH, inp.t, s, f"{m.id} {', '.join(m.kinds)}")
        if self.mode in walking:
            s_here = max(s, self.proj_s)
            refusing = f.status in ("BLOCKED", "OFF_CORRIDOR")
            self.refuse_since = (
                (self.refuse_since if self.refuse_since is not None else inp.t)
                if refusing
                else None
            )
            on_route = abs(f.d) < p.trust_offset if math.isfinite(f.d) else False
            obstacle = self._scan_blocks(inp, s_here)
            if self.mode == Mode.NAVIGATE:
                if self.refuse_since is not None and inp.t - self.refuse_since >= p.rejoin_after:
                    self.astar_path = None
                    if on_route and obstacle is None:
                        self._set(Mode.FOLLOW_ROUTE, inp.t, s, f"{f.status} {f.reason}; scan clear")
                    else:
                        self._set(Mode.REJOIN, inp.t, s, f"{f.status} {f.reason}")
                else:
                    return NavOutput(self._shaped_walk(f), "official", self.mode)
            if self.mode == Mode.FOLLOW_ROUTE:
                if self.follow_from_s is None:
                    self.follow_from_s = s_here
                # Hand back only after real progress: the follower tends to accept a step and refuse
                # the next one on the same rough patch, and flipping between them stalls the walking
                # actor.
                if (
                    f.status in MOVING
                    and inp.t - self.mode_t > 1.5
                    and s_here - self.follow_from_s >= p.follow_min_progress
                ):
                    self.follow_from_s = None
                    self._set(Mode.NAVIGATE, inp.t, s, "follower driving again")
                    return NavOutput(self._shaped_walk(f), "official", self.mode)
                if obstacle is not None:
                    self.astar_path = None
                    self._set(
                        Mode.REJOIN, inp.t, s, f"something {obstacle:.1f} m ahead on the route"
                    )
                elif self._drop_ahead(inp, s_here):
                    self.drop_since = self.drop_since if self.drop_since is not None else inp.t
                    if inp.t - self.drop_since > p.drop_hold:
                        self.astar_path = None
                        self._set(Mode.REJOIN, inp.t, s, "drop ahead on the route")
                    else:
                        # Never forward onto it; turning on the spot is still fine.
                        cmd = self._pursue_route(inp, s_here, s_gate, p.walk_floor)
                        return NavOutput((0.0, 0.0, cmd[2]), "official", self.mode, "drop ahead")
                else:
                    self.drop_since = None
                    cmd = self._pursue_route(inp, s_here, s_gate, p.walk_floor)
                    return NavOutput(cmd, "official", self.mode, "route")
            if self.mode == Mode.REJOIN:
                if f.status in MOVING and inp.t - self.mode_t > 1.5:
                    self._set(Mode.NAVIGATE, inp.t, s, "follower driving again")
                    return NavOutput(self._shaped_walk(f), "official", self.mode)
                if inp.t - self.mode_t > p.rejoin_timeout:
                    self._set(Mode.HOLD, inp.t, s, "could not get back onto the route")
                    return NavOutput((0.0, 0.0, 0.0), "official", self.mode, "hold")
                cmd, why = self._astar_cmd(inp, s)
                if why == "A* found no way back":
                    # Facing the route and nothing seen leads there. Scan clear: back onto the route
                    # as the first version did. Something in the way: wait for it to move, then give
                    # up.
                    if obstacle is None and not self._drop_ahead(inp, s_here):
                        if on_route:
                            self._set(Mode.FOLLOW_ROUTE, inp.t, s, "A* has no way; scan clear")
                        cmd, why = (
                            self._pursue_route(inp, s_here, s_gate, p.rejoin_v),
                            "pursuing the route",
                        )
                    elif inp.t - self.mode_t > p.obstacle_wait:
                        self._set(Mode.HOLD, inp.t, s, "route blocked and no way round")
                        return NavOutput((0.0, 0.0, 0.0), "official", self.mode, "hold")
                    else:
                        why = "waiting: route blocked"
                return NavOutput(cmd, "official", self.mode, why)

        # --- APPROACH ----------------------------------------------------------------------------
        if self.mode == Mode.APPROACH:
            d, _lat, _head, normal, measured = self._edge_frame(inp, m)
            s_here = max(s, self.proj_s)
            past_bend = s_here >= self.align_from_s
            if past_bend and ((measured and d <= p.align_enter) or s >= m.s_first - 0.9):
                source = "measured" if measured else "from the map"
                self._set(Mode.ALIGN, inp.t, s, f"edge {source} {d:.2f} m ahead")
            else:
                if f.status in MOVING:
                    vx, vy, wz = self._shaped_walk(f)
                    return NavOutput(
                        (min(vx, p.approach_v), vy, wz), "official", self.mode, "approach"
                    )
                # The follower refuses (risers in its horizon read as unknown). On the route with
                # nothing new in the way: the route itself, as in NAVIGATE. Otherwise A* to a point
                # past the entry's last bend; if nothing seen leads there, the route when clear.
                s_here = max(s, self.proj_s)
                clear = self._scan_blocks(inp, s_here) is None and not self._drop_ahead(inp, s_here)
                on_route = math.isfinite(f.d) and abs(f.d) < p.trust_offset
                if on_route and clear:
                    cmd = self._pursue_route(inp, s_here, s_gate, p.approach_v)
                    return NavOutput(cmd, "official", self.mode, "route")
                goal_s = max(m.s_first - 0.6, self.align_from_s + 0.4)
                cmd, why = self._astar_cmd(inp, s, goal_s=goal_s, v=p.approach_v)
                if why == "A* found no way back" and clear:
                    cmd, why = (
                        self._pursue_route(inp, s_here, s_gate, p.approach_v),
                        "pursuing the route",
                    )
                return NavOutput(cmd, "official", self.mode, why)

        # --- ALIGN -------------------------------------------------------------------------------
        if self.mode == Mode.ALIGN:
            d, _lat, head, normal, measured = self._edge_frame(inp, m)
            tangent = self._tangent(m.s_first + 0.3)
            # The route's heading, as in the first version. A measured edge that agrees with the
            # plan (the route's angle to it on the map, re-applied to it as measured, lands within
            # agree_tol of the route) refines it: square-on means square to the riser actually
            # there, and a localisation yaw error cancels. A measured edge that does not agree (a
            # natural bank's ragged edge) is not followed; it only keeps the robot from going
            # side-on.
            entry = tangent
            if measured and m.has_edge:
                planned = wrap(tangent - m.prior_normal_yaw)
                if abs(planned) <= p.agree_tol:
                    planned = 0.0
                candidate = normal + planned
                if abs(wrap(candidate - tangent)) <= p.agree_tol:
                    entry = candidate
            if measured:
                entry = normal + float(
                    np.clip(wrap(entry - normal), -p.max_skew_hard, p.max_skew_hard)
                )
            head = wrap(inp.yaw - entry)
            left, right = side_margins(inp.grid, inp.valid)
            pr = self.path.project((inp.x, inp.y), s_hint=self.proj_s, window=(1.0, 1.0))
            vy = float(np.clip(-0.8 * pr.d, -p.align_strafe, p.align_strafe))
            if left is not None and right is not None and min(left, right) < p.centre_trigger:
                vy = float(np.clip(0.8 * (left - right) / 2, -p.centre_strafe, p.centre_strafe))
            elif left is not None and left < p.centre_trigger:
                vy = -p.centre_strafe / 2
            elif right is not None and right < p.centre_trigger:
                vy = p.centre_strafe / 2
            lo, hi = p.align_band
            if abs(head) > p.align_pivot:
                # Well off: turn in place, at no less than align_min_yaw_rate (the walking actor
                # ignores yaw commands below ~0.15 rad/s).
                cmd = (
                    0.0,
                    vy,
                    float(-np.sign(head) * max(p.align_min_yaw_rate, min(0.5, 1.5 * abs(head)))),
                )
            else:
                # Close: steer while rolling -- the walking actor turns far better moving than on
                # the spot -- and keep rolling through the band, since the SDK hands over to the
                # stairs actor moving.
                wz = float(np.clip(-2.0 * head, -0.4, 0.4))
                if d > hi - 0.05:
                    vx = p.align_creep_v
                elif d > lo + 0.06:
                    vx = p.align_through_v
                elif d >= lo:
                    vx = 0.0  # at the near end: do not push the wheels into the riser
                else:
                    vx = -p.align_back_v
                cmd = (vx, vy, wz)
            checks = {
                "band": lo <= d <= hi,
                "heading": abs(head)
                <= (
                    p.align_heading_tol
                    if inp.t - self.mode_t < p.align_soft_after
                    else p.align_heading_soft
                ),
                "yaw_rate": abs(self.yaw_rate_f) <= p.align_yaw_rate,
                "speed": abs(inp.v_forward) <= p.align_speed,
                "edge": measured or self.tracker is None,
            }
            ready = all(checks.values())
            self.ready_since = (
                (self.ready_since if self.ready_since is not None else inp.t) if ready else None
            )
            info = {
                "edge_d": round(d, 3),
                "edge_head_deg": round(math.degrees(head), 1),
                "measured": measured,
                "not_ready": [k for k, ok in checks.items() if not ok],
            }
            if self.ready_since is not None and inp.t - self.ready_since >= p.ready_dwell:
                if m.policy == "walk_descend":
                    self._set(
                        Mode.DESCEND, inp.t, s, f"aligned {math.degrees(head):.1f} deg at {d:.2f} m"
                    )
                else:
                    self.climb_normal = normal if measured else entry
                    self.climb_start_s = self.climb_best_s = s
                    self.climb_best_t = inp.t
                    self.retries, self.backoff_until, self.retry_bias = 0, None, 0.0
                    self.track, self.slide_since, self.square_until = [], None, None
                    source = "measured" if measured else "no edge in this manoeuvre"
                    self._set(
                        Mode.CLIMB,
                        inp.t,
                        s,
                        f"aligned {math.degrees(head):.1f} deg at {d:.2f} m, {source}",
                    )
            elif (
                inp.t - self.mode_t > p.align_timeout
                and self.tracker is None
                and m.policy != "walk_descend"
            ):
                # Nothing to measure (a slope): as the first version did, climb from where it is.
                self.climb_normal = entry
                self.climb_start_s = self.climb_best_s = s
                self.climb_best_t = inp.t
                self.retries, self.backoff_until, self.retry_bias = 0, None, 0.0
                self._set(
                    Mode.CLIMB,
                    inp.t,
                    s,
                    f"align timed out {math.degrees(head):.1f} deg off; a slope, climbing",
                )
            elif inp.t - self.mode_t > p.align_timeout:
                self._set(
                    Mode.HOLD,
                    inp.t,
                    s,
                    "edge never measured where the map puts it"
                    if not measured
                    else "could not settle into the entry band",
                )
                return NavOutput((0.0, 0.0, 0.0), "official", self.mode, "hold", info)
            else:
                return NavOutput(cmd, "official", self.mode, "align", info)

        # --- DESCEND -----------------------------------------------------------------------------
        if self.mode == Mode.DESCEND:
            if s >= m.s_last + 0.7:
                self.k += 1
                self._set(Mode.NAVIGATE, inp.t, s, "off the ledge")
                return NavOutput(self._shaped_walk(f), "official", self.mode)
            _d, _lat, head, _normal, _measured = self._edge_frame(inp, m)
            return NavOutput(
                (p.walk_floor, 0.0, float(np.clip(-1.5 * head, -0.3, 0.3))),
                "official",
                self.mode,
                "descend",
            )

        # --- CLIMB -------------------------------------------------------------------------------
        if self.mode == Mode.CLIMB:
            # Progress on the robot's own projection: the follower's stops at a waypoint not yet
            # scored.
            s_prog = max(s, self.proj_s)
            if s_prog > self.climb_best_s + 0.05:
                self.climb_best_s, self.climb_best_t = s_prog, inp.t
            stalled = inp.t - self.climb_best_t
            if (
                inp.t - self.mode_t > p.climb_timeout
                or (
                    stalled > p.climb_stuck
                    and self.retries >= p.climb_retries
                    and self.backoff_until is None
                )
                or stalled > p.climb_stall + p.climb_retries * (p.climb_stuck + p.backoff_time)
            ):
                self._set(Mode.HOLD, inp.t, s, f"climb made no progress ({self.retries} retries)")
                return NavOutput((0.0, 0.0, 0.0), "stairs_stable", self.mode, "hold")
            if (
                stalled > p.climb_stuck
                and self.backoff_until is None
                and self.retries < p.climb_retries
            ):
                # Wedged: back off, then retry leaning towards the route (the planned line was
                # clear).
                self.retries += 1
                self.backoff_until = inp.t + p.backoff_time
                pr = self.path.project((inp.x, inp.y), s_hint=s_prog, window=(1.0, 1.0))
                self.retry_bias = p.retry_bias * (1.0 if pr.d < 0 else -1.0)
                self.retry_until_s = s_prog + p.retry_reach
                self.log.append(
                    {
                        "t": round(inp.t, 2),
                        "s": round(s, 2),
                        "from": "CLIMB",
                        "to": "CLIMB",
                        "maneuver": m.id,
                        "why": f"wedged: backing off, retry {self.retries}",
                    }
                )
            if self.backoff_until is not None:
                if inp.t < self.backoff_until:
                    return NavOutput(
                        (-p.backoff_v, 0.0, 0.0), "stairs_stable", self.mode, "backing off"
                    )
                self.backoff_until = None
                self.climb_best_t = inp.t
            slide = self._slide_check(inp, m, s)
            if slide is not None:
                return slide
            if self.proj_s > s_gate + p.missed_gate_margin and self.pending_gate is None:
                self.pending_gate = s_gate
                gate_xy = np.asarray(self.path.point_at(s_gate))[:2]
                self.missed.append(
                    {
                        "t": round(inp.t, 2),
                        "s_gate": round(s_gate, 2),
                        "target": f.target_id,
                        "passed_at_m": round(
                            float(np.hypot(gate_xy[0] - inp.x, gate_xy[1] - inp.y)), 2
                        ),
                    }
                )
            fix = detect_edge(inp.grid, inp.valid, x_min=0.2, x_max=1.2)
            if (
                fix is not None
                and abs(wrap(inp.yaw + fix.angle - self.climb_normal)) < p.edge_gate_angle
            ):
                a = 0.3
                n_new = inp.yaw + fix.angle
                self.climb_normal = math.atan2(
                    (1 - a) * math.sin(self.climb_normal) + a * math.sin(n_new),
                    (1 - a) * math.cos(self.climb_normal) + a * math.cos(n_new),
                )
            under = detect_edge(inp.grid, inp.valid, x_min=-0.45, x_max=0.45, want="up")
            if s_prog >= m.s_last + p.exit_past_last and under is None:
                self._set(Mode.VERIFY, inp.t, s, "past the last expected edge")
            else:
                s_here = max(s, self.proj_s)
                cap = self.path.length if self.pending_gate is not None else s_gate
                carrot = self.path.point_at(
                    min(s_here + 0.8, self.path.length, max(cap, s_here + 0.3))
                )
                bearing = math.atan2(carrot[1] - inp.y, carrot[0] - inp.x)
                # The first version's law: the route's own direction, corrected towards the carrot
                # by at most climb_correction. On a flight the steps push the motion up the fall
                # line while the body turns; a pursuit free to turn further to correct the drift
                # ends up side-on to the risers, or asks for a turn the stairs actor answers by
                # toppling.
                tangent = self._tangent(s_here + 0.4)
                aim = tangent + float(
                    np.clip(wrap(bearing - tangent), -p.climb_correction, p.climb_correction)
                )
                # Safety only: never more than max_skew_hard off the risers as measured.
                aim = self.climb_normal + float(
                    np.clip(wrap(aim - self.climb_normal), -p.max_skew_hard, p.max_skew_hard)
                )
                left, right = side_margins(inp.grid, inp.valid)
                if left is not None and left < p.centre_trigger and (right is None or right > left):
                    aim -= p.centre_bias
                elif (
                    right is not None
                    and right < p.centre_trigger
                    and (left is None or left > right)
                ):
                    aim += p.centre_bias
                if s_prog < self.retry_until_s:
                    aim += self.retry_bias
                err = wrap(aim - inp.yaw)
                near_gate = self.pending_gate is None and 0.0 <= s_gate - s_here < p.climb_gate_slow
                tight = any(m_ is not None and m_ < p.centre_trigger for m_ in (left, right))
                v = p.climb_turn_v if (abs(err) > math.radians(8) or near_gate) else p.climb_v
                if abs(err) > math.radians(15) or tight:
                    # It turns at about the same rate whatever its speed: slower means more turn per
                    # metre.
                    v = p.climb_slow_v
                v = max(0.1, min(v, v - max(0.0, inp.v_forward - v)))
                wz = float(
                    np.clip(p.climb_kp * err - p.climb_kd * self.yaw_rate_f, -p.climb_w, p.climb_w)
                )
                side = None
                if right is not None and right < p.side_stop and (left is None or left > right):
                    side = "right"
                elif left is not None and left < p.side_stop and (right is None or right > left):
                    side = "left"
                if side is not None:
                    self.drop_since = self.drop_since if self.drop_since is not None else inp.t
                    if inp.t - self.drop_since > p.drop_hold:
                        self._set(Mode.HOLD, inp.t, s, f"drop beside the stairs actor ({side})")
                        return NavOutput((0.0, 0.0, 0.0), "stairs_stable", self.mode, "hold")
                    wz_away = p.climb_w if side == "right" else -p.climb_w
                    return NavOutput(
                        (0.0, 0.0, wz_away),
                        "stairs_stable",
                        self.mode,
                        f"drop {side}",
                        {"aim_deg": round(math.degrees(aim), 1)},
                    )
                if self._drop_ahead(inp, s_here):
                    # Never onto a drop: stop, keep turning towards the aim (or towards the roomier
                    # side).
                    self.drop_since = self.drop_since if self.drop_since is not None else inp.t
                    if inp.t - self.drop_since > p.drop_hold:
                        self._set(Mode.HOLD, inp.t, s, "drop ahead of the stairs actor")
                        return NavOutput((0.0, 0.0, 0.0), "stairs_stable", self.mode, "hold")
                    if abs(err) < math.radians(5) and left is not None and right is not None:
                        wz = p.climb_w * (1.0 if left > right else -1.0)
                    return NavOutput(
                        (0.0, 0.0, wz),
                        "stairs_stable",
                        self.mode,
                        "drop ahead",
                        {"aim_deg": round(math.degrees(aim), 1)},
                    )
                self.drop_since = None
                return NavOutput(
                    (v, 0.0, wz),
                    "stairs_stable",
                    self.mode,
                    "climb",
                    {"aim_deg": round(math.degrees(aim), 1)},
                )

        # --- VERIFY / HANDBACK -------------------------------------------------------------------
        if self.mode == Mode.VERIFY:
            level = abs(inp.pitch) < p.verify_level and abs(inp.roll) < p.verify_level
            under = detect_edge(inp.grid, inp.valid, x_min=-0.45, x_max=0.45)
            ok = level and under is None
            self.verify_since = (
                (self.verify_since if self.verify_since is not None else inp.t) if ok else None
            )
            if self.verify_since is not None and inp.t - self.verify_since >= p.verify_hold:
                self._set(Mode.HANDBACK, inp.t, s, "level, no edge under the body")
                return NavOutput((0.0, 0.0, 0.0), "official", self.mode, "handback")
            if inp.t - self.mode_t > 8.0:
                self._set(Mode.HOLD, inp.t, s, "never verified clear of the climb")
                return NavOutput((0.0, 0.0, 0.0), "stairs_stable", self.mode, "hold")
            return NavOutput((0.15, 0.0, 0.0), "stairs_stable", self.mode, "verify")
        if self.mode == Mode.HANDBACK:
            held = inp.t - self.mode_t
            if (inp.owner_feedback == "official" and held > 0.3) or held > p.handback_hold:
                self.k += 1
                if self.pending_gate is not None and s_gate <= self.pending_gate + 1e-6:
                    self._set(Mode.RECOVER_GATE, inp.t, s, "a waypoint was run past on the climb")
                else:
                    self.pending_gate = None
                    self._set(Mode.NAVIGATE, inp.t, s, "walking actor back")
            return NavOutput((0.0, 0.0, 0.0), "official", self.mode, "handback")

        # --- RECOVER_GATE ------------------------------------------------------------------------
        if self.mode == Mode.RECOVER_GATE:
            if s_gate > self.pending_gate + 1e-6:
                self.pending_gate = None
                # Going back for it may have brought the robot back into (or before) a manoeuvre it
                # had finished -- down the flight it climbed: re-arm that manoeuvre.
                s_here = min(s, self.proj_s)
                for j in range(self.k):
                    if self.mans[j].s0 - 0.3 <= s_here < self.mans[j].s1:
                        self.k = j
                        break
                self._set(Mode.NAVIGATE, inp.t, s, "waypoint scored")
                return NavOutput(self._shaped_walk(f), "official", self.mode)
            gate = self.path.point_at(s_gate)
            err = wrap(math.atan2(gate[1] - inp.y, gate[0] - inp.x) - inp.yaw)
            dist = math.hypot(gate[0] - inp.x, gate[1] - inp.y)
            if abs(err) > math.radians(25):
                return NavOutput(
                    (0.0, 0.0, float(np.clip(1.5 * err, -0.5, 0.5))),
                    "official",
                    self.mode,
                    "turn back",
                )
            return NavOutput(
                (
                    float(np.clip(dist, 0.15, p.approach_v)),
                    0.0,
                    float(np.clip(1.5 * err, -0.5, 0.5)),
                ),
                "official",
                self.mode,
                "back to the waypoint",
            )
        return NavOutput((0.0, 0.0, 0.0), "official", self.mode, "no branch")
