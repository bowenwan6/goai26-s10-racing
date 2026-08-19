"""Checks on the parts of the segment runner that fail quietly.

A wrong joint permutation does not raise. It produces a robot that stands up, drives, and
falls over slightly sooner than it should, and the conclusion drawn from the run -- "this
segment needs a new policy" -- is then an artefact of the harness. So the two orderings and
the constants derived from them are checked against the literals in the SDK header the
deployed runner is built from, rather than against the training code that is supposed to
agree with it.
"""

from __future__ import annotations

import numpy as np
import pytest
from s10_climb.segment_runner import (
    DEFAULT_ROBOT,
    POLICY2ROBOT,
    ROBOT2POLICY,
    SCALE_ROBOT,
    _write_png,
)

from s10_rl.observation import POLICY_ORDER, ROBOT_ORDER

#: ``dof_default_eigen_robot`` from run_policy/s10_policy_runner.hpp, copied by hand. The
#: point of the check is that it is an independent transcription, so it must not be imported.
SDK_DEFAULT_ROBOT = [
    0.0,
    -0.3,
    0.6,
    0.0,
    0.0,
    -0.3,
    0.6,
    0.0,
    0.0,
    0.3,
    -0.6,
    0.0,
    0.0,
    0.3,
    -0.6,
    0.0,
]
#: ``action_scale_robot`` from the same header.
SDK_ACTION_SCALE = [0.125, 0.25, 0.25, 5.0] * 4


def test_the_permutations_are_inverses():
    """If they are not, an action lands on the wrong joint and the robot is subtly lame."""
    assert list(np.asarray(ROBOT2POLICY)[POLICY2ROBOT]) == list(range(16))
    assert list(np.asarray(POLICY2ROBOT)[ROBOT2POLICY]) == list(range(16))


def test_the_permutations_actually_reorder_the_names():
    for policy_slot, robot_slot in enumerate(ROBOT2POLICY):
        assert POLICY_ORDER[policy_slot] == ROBOT_ORDER[robot_slot]
    for robot_slot, policy_slot in enumerate(POLICY2ROBOT):
        assert ROBOT_ORDER[robot_slot] == POLICY_ORDER[policy_slot]


def test_the_default_pose_matches_the_sdk_header():
    """Derived by permuting the policy-order defaults; the header writes it out longhand."""
    assert np.allclose(DEFAULT_ROBOT, SDK_DEFAULT_ROBOT)


def test_the_action_scale_matches_the_sdk_header():
    assert np.allclose(SCALE_ROBOT, SDK_ACTION_SCALE)


def test_the_wheel_channels_are_the_ones_the_sdk_treats_as_velocity():
    """The SDK reads every fourth robot slot as a wheel. That has to be where wheels are."""
    for leg in range(4):
        assert ROBOT_ORDER[leg * 4 + 3].endswith("_wheel_joint")
        for part in range(3):
            assert not ROBOT_ORDER[leg * 4 + part].endswith("_wheel_joint")


def _straight_course(n: int, spacing: float = 5.0):
    from s10_auto_nav.waypoints import Course, Waypoint

    return Course([Waypoint(index=i, position=np.array([i * spacing, 0.0, 0.0])) for i in range(n)])


@pytest.mark.parametrize("n_waypoints", [2, 3])
def test_the_approach_taper_applies_at_every_gate_not_just_the_last(n_waypoints):
    """Pinned because a run was misdiagnosed on the assumption that it did not.

    `pure_pursuit` tapers forward speed to `max_forward * distance / lookahead` on the run
    in to a waypoint. It is tempting to read that as a finish-line brake, and therefore to
    read a segment runner's terminal stall as an artefact of cutting the course short. It is
    not: `Course.lookahead_point` deliberately never runs the carrot onto the next leg, so
    the carrot sits on the gate and the taper fires at *every* waypoint. Adding a waypoint
    past the segment end changes nothing, which is what the parametrisation asserts.

    The consequence is real and belongs to the race, not the harness: half a metre out the
    robot is asking for 0.7 * 0.5 / 1.4 = 0.25 m/s, and half a metre out is exactly where a
    robot cresting a riser still has its rear axle on the step.
    """
    from s10_auto_nav.pure_pursuit import PurePursuitController, PursuitGains

    gains = PursuitGains()
    course = _straight_course(n_waypoints)
    course.update(course.waypoints[0].xy)  # consume the spawn waypoint
    assert course.cursor == 1

    controller = PurePursuitController(gains)
    position = np.array([course.waypoints[1].position[0] - 0.5, 0.0])
    for _ in range(50):  # let the slew limiter settle
        target = course.lookahead_point(position, controller.lookahead_distance())
        command = controller.compute(position, 0.0, target, 0.02)

    assert command.forward == pytest.approx(gains.max_forward * 0.5 / gains.lookahead, rel=1e-2)


def test_the_taper_cannot_stall_the_robot_before_the_gate_is_consumed():
    """The floor on the taper: the slowest command the robot ever gets on a clean approach.

    `Course` consumes a gate at `score_radius`, so the command bottoms out at
    `max_forward * score_radius / lookahead` and then jumps back up. Worth a number rather
    than a shrug, because tightening the acceptance radius lowers this sandbox-only taper
    floor. The production follower separately enforces its configured capture behavior.
    """
    from s10_auto_nav.pure_pursuit import PursuitGains

    gains = PursuitGains()
    course = _straight_course(3)
    floor = gains.max_forward * course.score_radius / gains.lookahead
    assert floor == pytest.approx(0.09, abs=1e-3)


def test_the_png_writer_produces_a_file_a_decoder_accepts(tmp_path):
    """Written by hand because the image has no imageio, PIL, OpenCV or ffmpeg.

    Checked by decoding rather than by eye: a PNG with a bad CRC or a missing filter byte
    still looks like a file and still has a plausible size.
    """
    zlib = pytest.importorskip("zlib")
    rgb = np.arange(4 * 3 * 3, dtype=np.uint8).reshape(4, 3, 3)
    path = tmp_path / "frame.png"
    _write_png(path, rgb)

    blob = path.read_bytes()
    assert blob[:8] == b"\x89PNG\r\n\x1a\n"

    # Walk the chunks, verifying each CRC, and pull the pixels back out of IDAT.
    cursor, idat = 8, b""
    seen = []
    while cursor < len(blob):
        length = int.from_bytes(blob[cursor : cursor + 4], "big")
        tag = blob[cursor + 4 : cursor + 8]
        payload = blob[cursor + 8 : cursor + 8 + length]
        crc = int.from_bytes(blob[cursor + 8 + length : cursor + 12 + length], "big")
        assert crc == zlib.crc32(tag + payload), f"{tag!r} chunk is corrupt"
        seen.append(tag)
        if tag == b"IDAT":
            idat += payload
        cursor += 12 + length
    assert seen == [b"IHDR", b"IDAT", b"IEND"]

    raw = zlib.decompress(idat)
    stride = 3 * 3
    for row in range(4):
        assert raw[row * (stride + 1)] == 0, "filter byte must be 0 (None)"
        start = row * (stride + 1) + 1
        assert list(raw[start : start + stride]) == list(rgb[row].reshape(-1))
