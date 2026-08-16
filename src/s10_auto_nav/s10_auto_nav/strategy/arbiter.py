"""Single-owner gate for the joint command topic.

The thing that must never happen is two controllers driving sixteen actuators at once: that
is not a handover, it is a fight at 50 Hz, and the result is a thrash rather than a stop. ROS
will not prevent it -- multiple publishers on one topic is a legal, silent merge -- so it is
prevented here.

**What this does and does not guarantee.** ``/JOINTS_CMD`` is written by the contest SDK from
inside ``rl_deploy``, as a DDS message type, and no external node can take it away or stop the
official policy writing to it. This arbiter gates the joint stream *this* project emits
(``/strategy/climb_joints``). Extending the guarantee to the actuators means putting the
override where ``/JOINTS_CMD`` is written, in ``integration/ros_cmd_interface.hpp``, and that
is not yet implemented. Claiming otherwise would be the more dangerous kind of wrong, because
the claim is about a safety property.

This is deliberately free of ROS so it can be tested without a node, a simulator or a running
daemon. The node passes a ``publish`` callable; everything else is bookkeeping and refusal.

The same reasoning applies one level up at ``/cmd_vel``, but there the router's
:class:`~s10_auto_nav.strategy.router.Source` already carries ownership on every tick, so no
second mechanism is needed.
"""

from __future__ import annotations

from collections.abc import Callable

import numpy as np

OFFICIAL = "official"
CLIMB = "climb"


class JointArbiter:
    """Forwards joint commands only from the current owner.

    ``forward`` refuses rather than queues when the caller is not the owner. Queuing would
    mean the refused command arrives later, out of order, which is worse than not arriving.
    """

    OFFICIAL = OFFICIAL
    CLIMB = CLIMB

    def __init__(self, publish: Callable[[np.ndarray], None] | None = None):
        self.owner = OFFICIAL
        self.publish = publish
        self.forwarded = 0
        self.refused = 0

    def grant(self, owner: str) -> None:
        if owner not in (OFFICIAL, CLIMB):
            raise ValueError(f"unknown joint command owner: {owner}")
        self.owner = owner

    def forward(self, owner: str, joints) -> bool:
        """Publish ``joints`` if ``owner`` holds the topic. Returns whether it did."""
        if owner != self.owner:
            self.refused += 1
            return False
        if joints is None:
            return False
        values = np.asarray(joints, float).reshape(-1)
        # A wrong-length or non-finite action reaching sixteen actuators is the one failure
        # here with no recovery, so it is checked even though a correct policy cannot produce
        # it. PolicyAction validates the same thing; this is the layer that has to be right
        # when something bypasses it.
        if values.size != 16 or not np.all(np.isfinite(values)):
            self.refused += 1
            return False
        self.forwarded += 1
        if self.publish is not None:
            self.publish(values)
        return True
