"""Single-owner gate for the joint command topic.

The thing that must never happen is two controllers driving sixteen actuators at once: that
is not a handover, it is a fight at 50 Hz, and the result is a thrash rather than a stop. ROS
will not prevent it -- multiple publishers on one topic is a legal, silent merge -- so it is
prevented here.

**What this does and does not guarantee.** ``/JOINTS_CMD`` is written by the contest SDK from
inside ``rl_deploy``, as a DDS message type, and no external node can take it away or stop the
official policy writing to it. This arbiter gates the joint stream *this* project emits
(``/strategy/climb_joints``), which is a real property but a weaker one than it sounds.

The guarantee about the actuators is enforced elsewhere, by
``integration/joint_command_owner.hpp``, which ``scripts/patch_upstream.py`` installs into the
SDK and wires into the one call in ``RLControlState`` that turns a policy action into a joint
command. That gate is what makes single ownership true; this class is what makes sure a
request the gate would refuse is never sent. Keeping both is not redundancy -- they answer to
different failures, and only one of them is a safety property.

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
GATE16 = "gate16"
GATE16_SHADOW = "gate16_shadow"
GATE16_CLIMB = "gate16_climb"
GATE16_CLIMB_FALLBACK = "gate16_climb_fallback"
STAIRS57 = "stairs57"
STOP = "stop"


class JointArbiter:
    """Forwards joint commands only from the current owner.

    ``forward`` refuses rather than queues when the caller is not the owner. Queuing would
    mean the refused command arrives later, out of order, which is worse than not arriving.
    """

    OFFICIAL = OFFICIAL
    CLIMB = CLIMB
    GATE16 = GATE16
    GATE16_SHADOW = GATE16_SHADOW
    GATE16_CLIMB = GATE16_CLIMB
    GATE16_CLIMB_FALLBACK = GATE16_CLIMB_FALLBACK
    STAIRS57 = STAIRS57
    STOP = STOP

    def __init__(self, publish: Callable[[np.ndarray], None] | None = None):
        self.owner = OFFICIAL
        self.publish = publish
        self.forwarded = 0
        self.refused = 0

    def grant(self, owner: str) -> None:
        if owner not in (
            OFFICIAL,
            CLIMB,
            GATE16,
            GATE16_SHADOW,
            GATE16_CLIMB,
            GATE16_CLIMB_FALLBACK,
            STAIRS57,
            STOP,
        ):
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
