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
    0.0, -0.3, 0.6, 0.0,
    0.0, -0.3, 0.6, 0.0,
    0.0, 0.3, -0.6, 0.0,
    0.0, 0.3, -0.6, 0.0,
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
