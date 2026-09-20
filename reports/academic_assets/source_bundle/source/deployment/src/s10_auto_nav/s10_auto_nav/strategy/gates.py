"""Two small pieces that stop the router chattering, kept separate so they can be tested.

Every transition in ``router.py`` is expressed through one of these rather than as a bare
comparison, and that is a deliberate constraint. A bare ``if distance < 1.0`` at the edge of
its threshold flips state at the control rate -- 50 times a second -- and the observable
symptom is not a wrong decision but a robot that alternates between two behaviours until
something times out. That has already cost this project one run, in the stall watchdog
interrupting the step commit every 2.5 s.

The two mechanisms are different and both are needed:

``Hysteresis`` separates the threshold for entering from the threshold for leaving, so a
signal hovering on the boundary stays where it is. It fixes chatter caused by *noise near a
threshold*.

``Debounced`` requires a condition to hold for a dwell time before it is acted on. It fixes
chatter caused by a condition being *briefly true* -- a contact spike, one clean lidar frame
through a gap. Hysteresis alone cannot do this: a signal that crosses the enter threshold
decisively and then comes back is not noise, and hysteresis will happily follow it.
"""

from __future__ import annotations


class Hysteresis:
    """A threshold with different entry and exit points.

    ``rising=True`` means the condition becomes true when the value rises past ``enter`` and
    false only when it falls back below ``exit_``; ``rising=False`` is the mirror, which is
    the common case here because most router conditions are "close enough" or "small enough".
    """

    def __init__(self, enter: float, exit_: float, *, rising: bool, initial: bool = False):
        if rising and not exit_ < enter:
            raise ValueError("a rising hysteresis needs exit_ < enter, or it is a plain threshold")
        if not rising and not exit_ > enter:
            raise ValueError("a falling hysteresis needs exit_ > enter")
        self.enter = float(enter)
        self.exit_ = float(exit_)
        self.rising = bool(rising)
        self.state = bool(initial)

    def update(self, value: float) -> bool:
        if self.rising:
            self.state = value >= self.enter if not self.state else value >= self.exit_
        else:
            self.state = value <= self.enter if not self.state else value <= self.exit_
        return self.state

    def reset(self, state: bool = False) -> None:
        self.state = bool(state)


class Debounced:
    """A boolean that only takes a new value after holding it for a dwell time.

    Entry and exit dwells are separate because the two directions are rarely equally urgent.
    Deciding the robot is aligned well enough to start a climb should be slow and sure;
    deciding it has stopped being aligned should be quick. Passing one dwell sets both.
    """

    def __init__(
        self, dwell_true: float, dwell_false: float | None = None, *, initial: bool = False
    ):
        self.dwell_true = float(dwell_true)
        self.dwell_false = float(dwell_true if dwell_false is None else dwell_false)
        self.state = bool(initial)
        self.held = 0.0

    def update(self, raw: bool, dt: float) -> bool:
        if bool(raw) == self.state:
            self.held = 0.0
            return self.state
        self.held += dt
        if self.held >= (self.dwell_true if raw else self.dwell_false):
            self.state = bool(raw)
            self.held = 0.0
        return self.state

    @property
    def settling(self) -> bool:
        """True while a change is pending -- useful in logs to explain a non-transition."""
        return self.held > 0.0

    def reset(self, state: bool = False) -> None:
        self.state = bool(state)
        self.held = 0.0
