"""What each actor can do, as numbers the route preparation and the router read -- not as code.

The walking actor (official slot) and the stairs actor (stairs_stable slot) were measured in MuJoCo
on synthetic terrain: steps, slopes and side slopes, one at a time, at several speeds. When a
retrained policy changes what it can take, re-measure and update ``policy_profile.json`` next to
this file (or pass another file); route preparation (margins), manoeuvre annotation (where the
stairs actor is needed, what to warn about) and the router (speed floor, entry skew) all read it,
nothing else changes.

    walk.max_step_up     edges above this need the stairs actor
    walk.max_slope_deg   climbs steeper than this need it
    walk.max_cross_deg   side slopes steeper than this (held for 1 m) need it: the walking actor
    slides
    walk.min_speed       the walking actor stalls on small bumps below this; the router holds it
    above
    walk.lateral_error   route margin off the steps
    climb.max_step_up    manoeuvres with a taller edge are flagged
    climb.lateral_error  route margin on the steps (the stairs actor also drifts)
    climb.max_entry_skew_deg  how far off square the router lets it cross an edge
    climb.turns_on_stairs     False: turns inside a climb are flagged
"""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, field
from pathlib import Path


@dataclass
class ActorProfile:
    max_step_up: float  # m, a single riser it climbs reliably
    max_step_down: float  # m
    max_slope_deg: float  # climbing grade
    max_cross_deg: float  # side slope it holds a line on
    min_speed: float  # m/s below which it stalls on small edges (0 = none)
    max_entry_skew_deg: float  # crossing an edge this far off square is still fine
    lateral_error: float  # m, how far it wanders off a line it is steered along
    turns_on_stairs: bool  # can it pivot / turn meaningfully on a flight


@dataclass
class PolicyProfile:
    walk: ActorProfile = field(
        default_factory=lambda: ActorProfile(
            max_step_up=0.035,
            max_step_down=0.25,
            max_slope_deg=11.0,
            max_cross_deg=5.0,
            min_speed=0.5,
            max_entry_skew_deg=30.0,
            lateral_error=0.1,
            turns_on_stairs=False,
        )
    )
    climb: ActorProfile = field(
        default_factory=lambda: ActorProfile(
            max_step_up=0.18,
            max_step_down=0.25,
            max_slope_deg=20.0,
            max_cross_deg=10.0,
            min_speed=0.0,
            max_entry_skew_deg=25.0,
            lateral_error=0.2,
            turns_on_stairs=False,
        )
    )
    body_half_width: float = 0.3
    source: str = "MuJoCo synthetic tests 2026-09-19: J3100 walk, 1150 stairs"

    @property
    def max_step_up(self) -> float:
        return max(self.walk.max_step_up, self.climb.max_step_up)

    @property
    def max_step_down(self) -> float:
        return max(self.walk.max_step_down, self.climb.max_step_down)

    @property
    def max_slope(self) -> float:
        return math.radians(max(self.walk.max_slope_deg, self.climb.max_slope_deg))

    def save(self, path):
        with open(path, "w") as f:
            json.dump(asdict(self), f, indent=1)

    @classmethod
    def default(cls) -> PolicyProfile:
        """The packaged profile (policy_profile.json beside this module), else the defaults."""
        packaged = Path(__file__).with_name("policy_profile.json")
        return cls.load(packaged) if packaged.exists() else cls()

    @classmethod
    def load(cls, path) -> PolicyProfile:
        with open(path) as f:
            d = json.load(f)
        return cls(
            walk=ActorProfile(**d["walk"]),
            climb=ActorProfile(**d["climb"]),
            body_half_width=d.get("body_half_width", 0.3),
            source=d.get("source", ""),
        )
