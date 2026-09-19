"""Who drives, and with what command, along an RL route on the real robot.

The nominal controller is the first (simulation) version's -- the one that ran the whole 30-waypoint
course end to end -- with its modes and numbers unchanged: that is what makes this at least as
smooth. What the first version lacked was any answer to the robot not being where the route says:
pushed off it, the route blocked by something the map does not have, a slide, a stall. That is
added as recovery that engages only on a deviation the robot measures, and hands back to the
nominal controller as soon as the robot is back on the route.

Inputs are only what the robot has: localisation pose, IMU attitude and yaw rate, measured forward
speed, the 13x9 height grid, the LiDAR points (robot yaw frame), the route follower's output and the
route. The map enters through the manoeuvre list (``maneuvers.py``: where the stairs actor is
needed) and its surface (``map_check`` tells a new obstacle from mapped terrain, ``map_planner``
plans on it). Output: a body-velocity command and the SDK joint-owner request (``official`` =
walking actor, ``stairs_stable`` = stairs actor).

Nominal (the first version)::

    WALK      The route follower drives the walking actor, its forward command held at
              walk_floor or more (the walking actor stalls on any bump below ~0.5 m/s).
    TRACK     The follower refuses (BLOCKED / OFF_CORRIDOR for track_after s; at once when the
              segment forbids detours): pure pursuit on the route at walking speed, the lateral
              error strafed back. The follower's local grid reads rough ground as blocked; the
              route runs on ground the map has. Back to WALK once the follower drives again.
    APPROACH  approach m before a manoeuvre: pursuit at approach_v, strafing onto the line.
    ALIGN     At the manoeuvre start the walking actor turns in place to the route's heading at
              the first edge (align_tol, or align_timeout). New: it strafes against the lateral
              error while it turns -- the first version slid 1 m down WP15's side slope turning
              there, and fell on three seeds of four.
    CLIMB     The stairs actor. Heading: the route's own direction, corrected towards a carrot
              0.8 m ahead by at most climb_correction; slower near a waypoint, when faster than
              asked, and when the grid shows a drop more than drop_depth below the route ahead.
              Back to WALK past the manoeuvre -- new: not while still sliding faster than
              settle_v, since the SDK's hand-back holds the joints rigid for 0.25 s.
    DESCEND   A ledge to step off (nothing rises): the walking actor at walking speed. New:
              straight across at the aligned heading until the body is clear of the ledge (no
              turning on it) -- WP14's ledge then takes 3-4 s where the first version bounced at
              its lip; stuck there anyway for descend_stall s: back off and try again.
    RECOVER   The stairs actor ran past a waypoint it did not score: go back for it walking
              (once it has stopped sliding). New: the stairs actor first carries on to ground the
              walking actor takes (the map's slope/step check), and the way back is straight
              only if the map says it is walkable -- else a map path to it, else a map path back
              before the manoeuvre to climb it once more, else HOLD.

Recovery (on a measured deviation only)::

    DETOUR    A path on the map (``map_planner``) from where the robot is to the route ahead,
              followed by pure pursuit, when
                * something the map does not have stands in the route's corridor ahead
                  (``map_check``); what was seen is blocked in the plan and remembered,
                * the robot is off_route or more off the route, or the straight way back to
                  the carrot crosses a hazard of the map,
                * the route has not advanced for stall_time s -- after backing off first
                  (BACKUP, where the map is clear behind: a hip caught on a rock does not come
                  free by pushing on).
              Back to WALK on the route past the detour's goal.
    WAIT      No way round (a person in a narrow passage): stopped, re-planned every
              replan_every s, on as soon as the corridor clears; HOLD after wait_timeout.
    HOLD      Stopped for an operator, reason given.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum

import numpy as np

from s10_auto_nav.rl_nav.map_check import unexpected_ahead, unexplained

MOVING = {"RUNNING", "DETOUR", "ASTAR", "GATE"}


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class Polyline:
    """A detour path: arc length, projection and pure pursuit along it."""

    def __init__(self, xy):
        self.xy = np.asarray(xy, float)
        seg = np.hypot(np.diff(self.xy[:, 0]), np.diff(self.xy[:, 1]))
        self.s = np.r_[0.0, np.cumsum(seg)]
        self.length = float(self.s[-1])

    def project(self, p):
        """(arc length, distance) of the nearest point."""
        a, b = self.xy[:-1], self.xy[1:]
        ab = b - a
        t = np.clip(np.einsum("ij,ij->i", p - a, ab) / np.maximum((ab * ab).sum(1), 1e-12), 0, 1)
        foot = a + t[:, None] * ab
        dist = np.hypot(*(foot - p).T)
        k = int(np.argmin(dist))
        return float(self.s[k] + t[k] * (self.s[k + 1] - self.s[k])), float(dist[k])

    def point_at(self, s):
        s = min(max(s, 0.0), self.length)
        return np.array([np.interp(s, self.s, self.xy[:, 0]), np.interp(s, self.s, self.xy[:, 1])])

    def pursue(self, pose, v, wmax=0.6, lookahead=0.6):
        """The first version's pursuit law on this path: turn in place beyond 35 deg."""
        x, y, yaw = pose
        s, _ = self.project(np.array([x, y]))
        c = self.point_at(s + lookahead)
        err = wrap(math.atan2(c[1] - y, c[0] - x) - yaw)
        w = float(np.clip(1.5 * err, -wmax, wmax))
        if abs(err) > math.radians(35):
            return (0.0, 0.0, w)
        return (v * max(0.3, 1.0 - abs(err) / math.radians(60)), 0.0, w)


class Mode(Enum):
    WALK = "WALK"
    TRACK = "TRACK"
    APPROACH = "APPROACH"
    ALIGN = "ALIGN"
    CLIMB = "CLIMB"
    DESCEND = "DESCEND"
    RECOVER = "RECOVER"
    BACKUP = "BACKUP"
    DETOUR = "DETOUR"
    WAIT = "WAIT"
    HOLD = "HOLD"
    DONE = "DONE"


WALKING = (Mode.WALK, Mode.TRACK, Mode.APPROACH)


@dataclass
class RunnerParams:
    # --- the first version (route_follow_mujoco.ClimbRouter), unchanged -----------------------
    walk_v: float = 0.6
    walk_floor: float = 0.5
    climb_v: float = 0.3
    climb_turn_v: float = 0.2
    climb_w: float = 0.35
    climb_kp: float = 2.0
    climb_correction: float = math.radians(12.0)
    climb_carrot: float = 0.8
    climb_gate_slow: float = 1.2
    drop_v: float = 0.1
    drop_depth: float = 0.3
    drop_cells: int = 3
    approach: float = 1.2
    approach_v: float = 0.45
    align_tol: float = math.radians(6.0)
    align_yaw_rate: float = 0.15
    align_timeout: float = 10.0
    align_w: float = 0.5
    track_after: float = 3.0
    track_lookahead: float = 1.0
    lateral: float = 0.25
    walking_again: float = 1.5
    recover_margin: float = 0.3
    # --- new: ALIGN holds the line ----------------------------------------------------------------
    align_strafe: float = 0.2
    # --- new: recovery ----------------------------------------------------------------------------
    #: Further off the route than this: plan the way back on the map. Closer, the first version's
    #: pursuit brings it back, unless its straight line crosses a hazard of the map.
    off_route: float = 0.8
    back_on: float = 0.25
    obstacle_reach: float = 1.6
    obstacle_half_width: float = 0.45
    obstacle_memory: float = 30.0
    stall_time: float = 8.0
    stall_progress: float = 0.3
    detour_v: float = 0.45
    detour_goal: tuple = (1.0, 3.0)
    detour_stall: float = 6.0
    max_replans: int = 3
    replan_every: float = 1.0
    wait_timeout: float = 30.0
    zone_keepout: float = 0.3
    climb_stall: float = 25.0
    #: The SDK hands the stairs actor's joints back to the walking actor through SafeHold (legs
    #: held rigid, wheels damped, 0.25 s): never while the robot still slides faster than this
    #: (it did, 1.5 m/s down WP11's crest, and pitched over). The stairs actor keeps it, creeping,
    #: until it is slower, or settle_timeout has passed.
    settle_v: float = 0.4
    settle_timeout: float = 3.0
    #: Stepping off a ledge, the walking actor holds the heading it aligned to until its body is
    #: this far past the last edge (turning with the rear wheels on the edge catches a knee).
    ledge_clear: float = 0.6
    #: Stalled (driving, no progress: a hip or a knee caught on a rock): back off before planning
    #: again, where the map is clear behind (Nav2's back-up recovery). The walking actor reverses
    #: on flat ground at about this speed.
    backup_v: float = 0.2
    backup_time: float = 1.5
    #: A waypoint missed on the climb: back to it walking only on ground the map says the walking
    #: actor takes (the first version drove straight back, and slid down WP11's crest doing so
    #: with 0.2 m of localisation bias); else walk back before the manoeuvre and climb it once more,
    #: turning harder for that waypoint; else HOLD.
    #: Stepping off a ledge, no progress for descend_stall s (a knee caught on the lip): back off
    #: and try again; HOLD after descend_retries. (Held straight, WP14's ledge takes 3-4 s.)
    descend_stall: float = 6.0
    descend_retries: int = 3
    recover_v: float = 0.35
    recover_timeout: float = 60.0
    reclimb_correction: float = math.radians(20.0)

    @classmethod
    def from_profile(cls, profile) -> RunnerParams:
        """The stairs actor's forward speed is the profile's floor if it has one (0 = any)."""
        p = cls()
        if profile is not None and profile.walk.min_speed > 0:
            p.walk_floor = float(profile.walk.min_speed)
        return p


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
    follower: object
    s_gate: float
    owner: str | None = None
    points: np.ndarray | None = None


@dataclass
class NavOutput:
    command: tuple
    owner: str
    mode: Mode
    reason: str = ""
    info: dict = field(default_factory=dict)


class RouteRunner:
    def __init__(
        self, path, maneuvers, params: RunnerParams | None = None, surface=None, planner=None
    ):
        self.path = path
        self.zones = sorted(maneuvers, key=lambda m: m.s0)
        self.p = params or RunnerParams()
        self.surface = surface
        self.planner = planner
        self.k = 0
        self.mode, self.mode_t = Mode.WALK, 0.0
        self.log: list = []
        self.refuse_since = None
        self.recover_gate = None
        self.s_gate = math.inf
        self.d = 0.0
        self.best_s, self.best_s_t = -math.inf, 0.0
        self.detour = None  # dict(path, goal, why, replans, best, best_t)
        self.seen: list = []  # (t, xy array) of returns the map does not explain
        self.wait_why, self.wait_kind = "", "rejoin"
        self.next_plan_t = -math.inf
        self.climb_best = (-math.inf, 0.0)
        self.settle_since = None
        self.backup_until, self.backup_why, self.backup_kind = -math.inf, "", "rejoin"
        self.descend_best, self.descend_tries = (-math.inf, 0.0), 0
        self.descend_heading = 0.0
        self.recover = None  # dict(t0, how, line, best, best_t) while RECOVER runs
        self.reclimbed: set = set()  # gates a manoeuvre was climbed again for
        self.reclimb_gate = None

    # ------------------------------------------------------------------ helpers
    def _set(self, mode, t, s, why=""):
        self.log.append(
            {
                "t": round(t, 2),
                "s": round(float(s), 2),
                "from": self.mode.value,
                "to": mode.value,
                "zone": self.zones[self.k].id if self.k < len(self.zones) else None,
                "why": why,
            }
        )
        if mode in WALKING and self.mode not in WALKING:
            self.best_s_t = t
        self.mode, self.mode_t = mode, t

    def _zone(self):
        return self.zones[self.k] if self.k < len(self.zones) else None

    def _tangent(self, s):
        return float(np.atleast_1d(self.path.tangent_yaw_at(min(max(s, 0.0), self.path.length)))[0])

    def _pursue(self, s, inp, v, wmax, lookahead=0.8, lateral=0.0):
        """The first version's pursuit: never aim past the unscored waypoint; the walking actor
        strafes the lateral error back (``lateral``), which counters its downhill slide."""
        carrot = self.path.point_at(min(s + lookahead, self.path.length, self.s_gate))
        err = wrap(math.atan2(carrot[1] - inp.y, carrot[0] - inp.x) - inp.yaw)
        vy = float(np.clip(-1.0 * self.d, -lateral, lateral)) if lateral else 0.0
        w = float(np.clip(1.5 * err, -wmax, wmax))
        if abs(err) > math.radians(35):
            return (0.0, 0.0, w)
        return (v * max(0.3, 1.0 - abs(err) / math.radians(60)), vy, w)

    def _drop_ahead(self, inp, s):
        """The first version's check: grid cells 0.45-1.05 m ahead within +-0.3 m that are more
        than drop_depth below the ground under the robot and the route's own profile. Measured
        cells only: an unknown cell is a riser's shadow as often as a hole."""
        if inp.grid is None:
            return False
        g = np.asarray(inp.grid, float).reshape(13, 9)
        v = np.asarray(inp.valid, bool).reshape(13, 9)
        under = g[3:6, 2:7][v[3:6, 2:7]]
        ref = float(np.median(under)) if under.size else -0.42
        zs = self.path.points[:, 2]
        expected = float(np.interp(s + 0.75, self.path.s, zs) - np.interp(s, self.path.s, zs))
        low = (g[7:12, 2:7] < ref + min(expected, 0.0) - self.p.drop_depth) & v[7:12, 2:7]
        return int(low.sum()) >= self.p.drop_cells

    def _remember(self, inp):
        if self.surface is None or inp.points is None:
            return
        new = unexplained(inp.points, (inp.x, inp.y, inp.z, inp.yaw), self.surface, reach=3.0)
        if len(new):
            self.seen.append((inp.t, new))
        self.seen = [(t, xy) for t, xy in self.seen if inp.t - t <= self.p.obstacle_memory]

    def _seen_xy(self):
        return np.vstack([xy for _, xy in self.seen]) if self.seen else np.zeros((0, 2))

    def _obstacle(self, inp, s):
        """Distance to something the map does not have in the route's corridor ahead, or None."""
        if self.surface is None or inp.points is None:
            return None
        return unexpected_ahead(
            inp.points,
            (inp.x, inp.y, inp.z, inp.yaw),
            self.path,
            s,
            self.surface,
            reach=self.p.obstacle_reach,
            half_width=self.p.obstacle_half_width,
        )

    def _goal_range(self, s, kind):
        """Where a detour may rejoin: past what was seen for an obstacle, a little ahead for a way
        back. Never inside the next manoeuvre -- a climb starts from APPROACH, not from a detour
        path -- so an obstacle right before one leaves nothing to plan to (WAIT)."""
        if kind == "obstacle":
            lo, hi = self._past_obstacle(s)
        else:
            lo, hi = s + self.p.detour_goal[0], s + self.p.detour_goal[1]
        zone = self._zone()
        if zone is not None and hi > zone.s0 - self.p.zone_keepout:
            hi = zone.s0 - self.p.zone_keepout
            if kind != "obstacle":
                lo = min(lo, hi - 1.5)
        lo, hi = max(0.0, lo), min(self.path.length, hi)
        return (lo, hi) if hi - lo >= 0.5 else None

    def _plan(self, inp, s, why, kind="rejoin"):
        """Plan a detour; DETOUR on success, WAIT on failure."""
        if self.planner is None:
            self._set(Mode.HOLD, inp.t, s, f"{why}; no map to plan on")
            return
        goal = self._goal_range(s, kind)
        path = None
        if goal is not None:
            lo, hi = goal
            path = self.planner.plan(
                (inp.x, inp.y), self.path, goal, s_target=lo, extra_xy=self._seen_xy()
            )
        replans = self.detour["replans"] + 1 if self.detour is not None else 0
        self.next_plan_t = inp.t + self.p.replan_every
        if path is None:
            if self.mode != Mode.WAIT:
                self.wait_why, self.wait_kind = why, kind
                self._set(Mode.WAIT, inp.t, s, f"{why}: no way round on the map, waiting")
            return
        self.detour = {
            "path": path,
            "line": Polyline(path),
            "goal": (lo, hi),
            "why": why,
            "kind": kind,
            "replans": replans,
            "best": -math.inf,
            "best_t": inp.t,
            "t": inp.t,
        }
        self._set(Mode.DETOUR, inp.t, s, f"{why}: map path {len(path)} pts to s {lo:.1f}-{hi:.1f}")

    # ------------------------------------------------------------------ step
    def step(self, inp: NavInput) -> NavOutput:
        p, f = self.p, inp.follower
        s = f.s if math.isfinite(f.s) else self.path.project((inp.x, inp.y)).s
        self.d = f.d if math.isfinite(f.d) else 0.0
        self.s_gate = inp.s_gate
        moving = f.status in MOVING
        self._remember(inp)
        if f.finished:
            if self.mode != Mode.DONE:
                self._set(Mode.DONE, inp.t, s, "route finished")
            return NavOutput((0.0, 0.0, 0.0), "official", self.mode, "done")
        if self.mode == Mode.HOLD:
            owner = "stairs_stable" if inp.owner == "stairs_stable" else "official"
            return NavOutput((0.0, 0.0, 0.0), owner, self.mode, "hold")

        while (
            self.k < len(self.zones)
            and self.mode in (*WALKING, Mode.BACKUP, Mode.DETOUR, Mode.WAIT)
            and s > self.zones[self.k].s1
        ):
            self.k += 1  # passed without needing it (spawned inside, or detoured round it)
        zone = self._zone()

        # ---- recovery monitors: walking modes only, and only on a deviation ------------------
        if self.mode in WALKING:
            if s > self.best_s + p.stall_progress:
                self.best_s, self.best_s_t = s, inp.t
            obstacle = self._obstacle(inp, s)
            if obstacle is not None:
                self.detour = None
                self._plan(
                    inp, s, f"something the map does not have {obstacle:.1f} m ahead", "obstacle"
                )
            elif abs(self.d) > p.off_route:
                self.detour = None
                self._plan(inp, s, f"{abs(self.d):.2f} m off the route")
            elif self.mode != Mode.WALK and not self._way_back_clear(inp, s):
                self.detour = None
                self._plan(inp, s, "the way back to the route crosses a hazard of the map")
            elif inp.t - self.best_s_t > p.stall_time:
                self.detour = None
                self.best_s_t = inp.t
                self._backup(inp, s, f"no progress for {p.stall_time:.0f} s", "rejoin")
        if self.mode == Mode.BACKUP:
            if inp.t < self.backup_until:
                return NavOutput((-p.backup_v, 0.0, 0.0), "official", self.mode, "backing off")
            if self.backup_kind == "ledge":
                self._set(
                    Mode.DESCEND, inp.t, s, f"at the ledge again (try {self.descend_tries + 1})"
                )
                self.descend_best = (s, inp.t)
            else:
                self._plan(inp, s, self.backup_why, self.backup_kind)
        if self.mode == Mode.DETOUR:
            out = self._detour(inp, s)
            if out is not None:
                return out
        if self.mode == Mode.WAIT:
            return self._wait(inp, s)
        if self.mode == Mode.HOLD:  # a recovery that needed the map, without one
            return NavOutput((0.0, 0.0, 0.0), "official", self.mode, "hold")

        # ---- the first version -----------------------------------------------------------------
        if self.mode in (Mode.WALK, Mode.TRACK) and zone is not None and s >= zone.s0 - p.approach:
            self._set(Mode.APPROACH, inp.t, s, f"{zone.id} {', '.join(zone.kinds)}")
        refusing = f.status in ("OFF_CORRIDOR", "BLOCKED")
        # The follower says why it refuses: the segment forbids detours, or the robot is off its
        # corridor. Either way the route is the way on, at once (the first version waited 3 s
        # for the second).
        taught_only = f.status == "OFF_CORRIDOR" or (
            f.status == "BLOCKED" and f.reason.startswith("centerline_blocked_detour_forbidden")
        )
        if self.mode in (Mode.WALK, Mode.TRACK):
            self.refuse_since = (self.refuse_since or inp.t) if refusing else None
        if (
            self.mode == Mode.WALK
            and self.refuse_since is not None
            and (taught_only or inp.t - self.refuse_since > p.track_after)
        ):
            self._set(Mode.TRACK, inp.t, s, f"{f.status} {f.reason}")
        if self.mode == Mode.TRACK:
            if moving and inp.t - self.mode_t > p.walking_again:
                self._set(Mode.WALK, inp.t, s, "follower driving again")
            else:
                cmd = self._pursue(s, inp, p.walk_floor, 0.6, p.track_lookahead, p.lateral)
                return NavOutput(cmd, "official", self.mode, "route")
        if self.mode == Mode.APPROACH and s >= zone.s0:
            self._set(Mode.ALIGN, inp.t, s)
        if self.mode == Mode.ALIGN:
            out = self._align(inp, s, zone)
            if out is not None:
                return out
        if self.mode == Mode.DESCEND:
            if s >= zone.s1:
                self._set(Mode.WALK, inp.t, s, "past the ledge")
                self.k += 1
            else:
                if s > self.descend_best[0] + 0.15:
                    self.descend_best = (s, inp.t)
                elif inp.t - self.descend_best[1] > p.descend_stall:
                    if self.descend_tries >= p.descend_retries:
                        self._set(Mode.HOLD, inp.t, s, "could not step off the ledge")
                        return NavOutput((0.0, 0.0, 0.0), "official", self.mode, "hold")
                    self.descend_tries += 1
                    self._backup(inp, s, "stuck at the ledge", "ledge")
                    if self.mode == Mode.BACKUP:
                        return NavOutput(
                            (-p.backup_v, 0.0, 0.0), "official", self.mode, "backing off"
                        )
                    self._set(Mode.HOLD, inp.t, s, "stuck at the ledge, not clear behind")
                    return NavOutput((0.0, 0.0, 0.0), "official", self.mode, "hold")
                v = p.walk_floor
                if s < zone.s_last + p.ledge_clear:
                    # Straight across at the heading it aligned to, until the body is clear of the
                    # ledge: turning on the way (the route bends just past WP14's ledge) met the lip
                    # at an angle and bounced, and a turn with the rear on the edge caught a knee.
                    err = wrap(self.descend_heading - inp.yaw)
                    wz = float(np.clip(1.5 * err, -0.3, 0.3))
                    return NavOutput((v, 0.0, wz), "official", self.mode, "descend straight")
                return NavOutput(self._pursue(s, inp, v, 0.3), "official", self.mode, "descend")
        if self.mode == Mode.CLIMB and s > self.s_gate + p.recover_margin:
            self._set(Mode.RECOVER, inp.t, s, f"passed {f.target_id} unscored")
            self.recover_gate = self.s_gate
            self.recover = {
                "t0": inp.t,
                "how": None,
                "line": None,
                "best": -math.inf,
                "best_t": inp.t,
                "target": f.target_id,
            }
        if self.mode == Mode.RECOVER:
            out = self._recover(inp, s)
            if out is not None:
                return out
        if self.mode == Mode.CLIMB:
            if s >= zone.s1:
                if self._sliding(inp):
                    return NavOutput((p.drop_v, 0.0, 0.0), "stairs_stable", self.mode, "settling")
                self._set(Mode.WALK, inp.t, s, "past the manoeuvre")
                self.k += 1
                self.best_s, self.best_s_t = s, inp.t
            else:
                self.settle_since = None
                return self._climb(inp, s)
        if self.mode == Mode.APPROACH:
            cmd = self._pursue(s, inp, max(p.climb_v, p.approach_v), 0.6, lateral=p.lateral)
            return NavOutput(cmd, "official", self.mode, "approach")
        if not moving:
            return NavOutput((0.0, 0.0, 0.0), "official", self.mode, f"follower {f.status}")
        vx, vy, wz = f.command
        if vx > 0.25:
            vx = max(vx, p.walk_floor)
        return NavOutput((min(vx, p.walk_v), vy, wz), "official", self.mode, "follower")

    # ------------------------------------------------------------------ nominal pieces
    def _align(self, inp, s, zone):
        p = self.p
        a0 = max(s + 0.3, zone.s_first)
        ahead = np.linspace(a0, min(a0 + 0.8, self.path.length), 8)
        yaws = np.atleast_1d(self.path.tangent_yaw_at(ahead))
        tangent = math.atan2(float(np.mean(np.sin(yaws))), float(np.mean(np.cos(yaws))))
        err = wrap(tangent - inp.yaw)
        aligned = abs(err) <= p.align_tol and abs(inp.yaw_rate) < p.align_yaw_rate
        if aligned or inp.t - self.mode_t > p.align_timeout:
            why = f"heading error {math.degrees(err):.1f} deg, {self.d:+.2f} m off the route"
            if zone.policy == "walk_descend":
                self._set(Mode.DESCEND, inp.t, s, why)
                self.descend_best, self.descend_tries = (s, inp.t), 0
                self.descend_heading = tangent
            else:
                self._set(Mode.CLIMB, inp.t, s, why)
                self.climb_best = (s, inp.t)
            return None
        vy = float(np.clip(-1.0 * self.d, -p.align_strafe, p.align_strafe))
        return NavOutput(
            (0.0, vy, float(np.clip(1.5 * err, -p.align_w, p.align_w))),
            "official",
            self.mode,
            "align",
            {"err_deg": round(math.degrees(err), 1), "d": round(self.d, 3)},
        )

    def _climb(self, inp, s, cap_at_gate=True):
        """The first version's climb law. ``cap_at_gate``: never aim past the unscored waypoint
        (off only while RECOVER carries on to walkable ground past one)."""
        p = self.p
        if s > self.climb_best[0] + 0.2:
            self.climb_best = (s, inp.t)
        elif inp.t - self.climb_best[1] > p.climb_stall:
            self._set(Mode.HOLD, inp.t, s, f"no progress on the climb for {p.climb_stall:.0f} s")
            return NavOutput((0.0, 0.0, 0.0), "stairs_stable", self.mode, "hold")
        if self._obstacle(inp, s) is not None:
            # The stairs actor cannot go round anything: stop on the flight and wait.
            self.wait_why = "something the map does not have on the climb"
            self.wait_kind = "obstacle"
            self._set(Mode.WAIT, inp.t, s, self.wait_why)
            return NavOutput((0.0, 0.0, 0.0), "stairs_stable", self.mode, "wait")
        cap = self.s_gate if cap_at_gate else self.path.length
        carrot = self.path.point_at(min(s + p.climb_carrot, self.path.length, cap))
        tangent = self._tangent(s + 0.4)
        bearing = math.atan2(carrot[1] - inp.y, carrot[0] - inp.x)
        near_gate = 0.0 <= self.s_gate - s < p.climb_gate_slow
        corr = p.climb_correction
        if (
            near_gate
            and self.reclimb_gate is not None
            and abs(self.s_gate - self.reclimb_gate) < 1e-6
        ):
            corr = p.reclimb_correction  # the second go at a waypoint missed on the first
        aim = tangent + float(np.clip(wrap(bearing - tangent), -corr, corr))
        err = wrap(aim - inp.yaw)
        v = p.climb_turn_v if (abs(err) > math.radians(8) or near_gate) else p.climb_v
        v = max(0.1, min(v, v - max(0.0, inp.v_forward - v)))
        drop = self._drop_ahead(inp, s)
        if drop:
            v = p.drop_v
        return NavOutput(
            (v, 0.0, float(np.clip(p.climb_kp * err, -p.climb_w, p.climb_w))),
            "stairs_stable",
            self.mode,
            "drop ahead" if drop else "climb",
            {"aim_deg": round(math.degrees(aim), 1)},
        )

    def _sliding(self, inp):
        """Still too fast to hand the joints over (see settle_v); False once settle_timeout has
        passed since the first ask."""
        if abs(inp.v_forward) <= self.p.settle_v:
            self.settle_since = None
            return False
        self.settle_since = self.settle_since if self.settle_since is not None else inp.t
        return inp.t - self.settle_since < self.p.settle_timeout

    def _recover(self, inp, s):
        p, rc = self.p, self.recover
        zone = self._zone()
        stairs = inp.owner == "stairs_stable"
        if stairs and self._sliding(inp):
            return NavOutput((0.0, 0.0, 0.0), "stairs_stable", self.mode, "settling")
        if self.s_gate != self.recover_gate:
            self._set(Mode.WALK, inp.t, s, "waypoint scored")
            self.recover = None
            self.best_s, self.best_s_t = s, inp.t
            return None
        if (
            stairs
            and self.planner is not None
            and zone is not None
            and s < zone.s1
            and bool(self.planner.rough_at((inp.x, inp.y))[0])
        ):
            # Still on the climb (steps, a steep slope): the walking actor must not take over and
            # turn here. The stairs actor carries on along the route to walkable ground.
            return self._climb(inp, s, cap_at_gate=False)
        if inp.t - rc["t0"] > p.recover_timeout:
            self._set(Mode.HOLD, inp.t, s, f"could not get back to {rc['target']}")
            return NavOutput((0.0, 0.0, 0.0), "official", self.mode, "hold")
        gate = np.asarray(self.path.point_at(self.s_gate))[:2]
        if rc["how"] is None:
            rc["how"] = self._recover_plan(inp, gate, zone)
            if rc["how"] is None:
                self._set(
                    Mode.HOLD,
                    inp.t,
                    s,
                    f"missed {rc['target']} and no walking way back to it on the map",
                )
                return NavOutput((0.0, 0.0, 0.0), "official", self.mode, "hold")
        if rc["how"] == "straight":
            # The first version: turn to it, drive to it.
            err = wrap(math.atan2(gate[1] - inp.y, gate[0] - inp.x) - inp.yaw)
            w = float(np.clip(1.5 * err, -0.5, 0.5))
            if abs(err) > math.radians(25):
                return NavOutput((0.0, 0.0, w), "official", self.mode, "turn to the waypoint")
            dist = math.hypot(gate[0] - inp.x, gate[1] - inp.y)
            return NavOutput(
                (float(np.clip(dist, 0.15, 0.45)), 0.0, w),
                "official",
                self.mode,
                "back to the waypoint",
            )
        line = rc["line"]
        here, _ = line.project(np.array([inp.x, inp.y]))
        if rc["how"] == "reclimb" and line.length - here < 0.3:
            # Back before the manoeuvre: climb it again (APPROACH, ALIGN, CLIMB), turning harder
            # for the waypoint missed the first time.
            self.reclimbed.add(round(self.s_gate, 3))
            self.reclimb_gate = self.s_gate
            self.recover = None
            self._set(Mode.WALK, inp.t, s, f"back before {zone.id}: climbing it again")
            self.best_s, self.best_s_t = s, inp.t
            return None
        if here > rc["best"] + 0.2:
            rc["best"], rc["best_t"] = here, inp.t
        elif inp.t - rc["best_t"] > p.detour_stall:
            self._set(Mode.HOLD, inp.t, s, f"could not get back to {rc['target']}")
            return NavOutput((0.0, 0.0, 0.0), "official", self.mode, "hold")
        return NavOutput(
            line.pursue((inp.x, inp.y, inp.yaw), p.recover_v),
            "official",
            self.mode,
            f"{rc['how']} for {rc['target']}",
        )

    def _recover_plan(self, inp, gate, zone):
        """How to get back to a missed waypoint, decided once on walkable ground: straight (the
        way there is walkable on the map, or there is no map), a map path to it, or a map path
        back before the manoeuvre to climb it again (once per waypoint)."""
        rc = self.recover
        if self.planner is None or self._walkable_line((inp.x, inp.y), gate):
            return "straight"
        seen = self._seen_xy()
        path = self.planner.plan(
            (inp.x, inp.y),
            self.path,
            (self.s_gate - 0.1, self.s_gate + 0.1),
            s_target=self.s_gate,
            extra_xy=seen,
        )
        if path is not None:
            rc["line"] = Polyline(path)
            return "path"
        if (
            zone is not None
            and zone.s0 - 0.5 <= self.s_gate <= zone.s1
            and round(self.s_gate, 3) not in self.reclimbed
        ):
            path = self.planner.plan(
                (inp.x, inp.y),
                self.path,
                (zone.s0 - 1.5, zone.s0 - 0.3),
                s_target=zone.s0 - 0.3,
                extra_xy=seen,
            )
            if path is not None:
                rc["line"] = Polyline(path)
                return "reclimb"
        return None

    def _walkable_line(self, a, b, step=0.1):
        a, b = np.asarray(a, float)[:2], np.asarray(b, float)[:2]
        n = max(2, math.ceil(float(np.hypot(*(b - a))) / step) + 1)
        pts = a + np.linspace(0.0, 1.0, n)[:, None] * (b - a)
        return self.planner.line_clear(a, b) and not bool(np.any(self.planner.rough_at(pts)))

    # ------------------------------------------------------------------ recovery pieces
    def _past_obstacle(self, s):
        """Detour goal range: past everything seen in the corridor ahead."""
        seen = self._seen_xy()
        far = s + self.p.obstacle_reach
        if len(seen):
            s_pt, dist = self.path.project_many(seen, s, min(self.path.length, s + 6.0))
            near = dist <= self.p.obstacle_half_width + 0.3
            if near.any():
                far = float(s_pt[near].max())
        return far + 0.8, far + 3.0

    def _way_back_clear(self, inp, s):
        """TRACK and APPROACH drive straight at a carrot on the route: fine unless that line
        crosses a hazard of the map (off the route beside a drop, or cutting a corner)."""
        if self.planner is None or abs(self.d) < 0.15:
            return True
        carrot = self.path.point_at(min(s + self.p.track_lookahead, self.path.length, self.s_gate))
        return self.planner.line_clear((inp.x, inp.y), carrot)

    def _detour(self, inp, s):
        p, dt = self.p, self.detour
        path = dt["path"]
        lo = dt["goal"][0]
        if (abs(self.d) < p.back_on and s >= lo - 0.2) or (
            math.hypot(path[-1][0] - inp.x, path[-1][1] - inp.y) < 0.2
        ):
            self.detour = None
            self.best_s, self.best_s_t = s, inp.t
            self.refuse_since = None
            self._set(Mode.WALK, inp.t, s, "back on the route")
            return None
        # Something seen since the plan, on the path ahead: plan again round it too.
        if inp.t >= self.next_plan_t:
            fresh = [xy for t, xy in self.seen if t > dt["t"]]
            if fresh:
                new = np.vstack(fresh)
                here, _ = dt["line"].project(np.array([inp.x, inp.y]))
                ahead = np.array(
                    [dt["line"].point_at(u) for u in np.arange(here, dt["line"].length, 0.1)]
                    or [path[-1]]
                )
                gap = np.min(
                    np.hypot(
                        new[:, None, 0] - ahead[None, :, 0], new[:, None, 1] - ahead[None, :, 1]
                    ),
                    axis=1,
                )
                if np.any(gap < self.planner.half_width + self.planner.margin):
                    self._plan(inp, s, "something new on the detour", dt["kind"])
                    if self.mode != Mode.DETOUR:
                        return self._wait(inp, s)
                    dt = self.detour
                    path = dt["path"]
        progress, _ = dt["line"].project(np.array([inp.x, inp.y]))
        if progress > dt["best"] + 0.2:
            dt["best"], dt["best_t"] = progress, inp.t
        elif inp.t - dt["best_t"] > p.detour_stall:
            if dt["replans"] >= p.max_replans:
                self._set(Mode.HOLD, inp.t, s, f"detour made no progress ({dt['why']})")
                return NavOutput((0.0, 0.0, 0.0), "official", self.mode, "hold")
            self._backup(inp, s, "detour stalled", dt["kind"])
            if self.mode == Mode.BACKUP:
                return NavOutput((-p.backup_v, 0.0, 0.0), "official", self.mode, "backing off")
            if self.mode != Mode.DETOUR:
                return self._wait(inp, s)
            dt = self.detour
        cmd = dt["line"].pursue((inp.x, inp.y, inp.yaw), p.detour_v)
        return NavOutput(cmd, "official", self.mode, "detour", {"why": dt["why"]})

    def _backup(self, inp, s, why, kind):
        """Back off backup_time s, then plan (BACKUP); plan at once where the map is not clear
        behind."""
        c, sn = math.cos(inp.yaw), math.sin(inp.yaw)
        reach = self.p.backup_v * self.p.backup_time + 0.45  # plus the body's rear half
        behind = (inp.x - reach * c, inp.y - reach * sn)
        if self.planner is None or not self.planner.line_clear((inp.x, inp.y), behind):
            self._plan(inp, s, why, kind)
            return
        self.backup_until, self.backup_why, self.backup_kind = inp.t + self.p.backup_time, why, kind
        self._set(Mode.BACKUP, inp.t, s, f"{why}: backing off")

    def _wait(self, inp, s):
        p = self.p
        stairs = inp.owner == "stairs_stable"
        owner = "stairs_stable" if stairs else "official"
        if inp.t - self.mode_t > p.wait_timeout:
            self._set(Mode.HOLD, inp.t, s, f"{self.wait_why}; waited {p.wait_timeout:.0f} s")
            return NavOutput((0.0, 0.0, 0.0), owner, self.mode, "hold")
        if inp.t >= self.next_plan_t:
            self.next_plan_t = inp.t + p.replan_every
            if self.wait_kind == "obstacle" and self._obstacle(inp, s) is None:
                # It moved away: carry on as before.
                back = Mode.CLIMB if stairs and self._zone() is not None else Mode.WALK
                self._set(back, inp.t, s, "the way is clear again")
                if back == Mode.CLIMB:
                    self.climb_best = (s, inp.t)
                return NavOutput((0.0, 0.0, 0.0), owner, self.mode, "clear")
            if not stairs:
                self._plan(inp, s, self.wait_why, self.wait_kind)
                if self.mode == Mode.DETOUR:
                    return NavOutput((0.0, 0.0, 0.0), owner, self.mode, "detour planned")
        return NavOutput((0.0, 0.0, 0.0), owner, self.mode, "wait", {"why": self.wait_why})
