from __future__ import annotations

import math
import struct
from types import SimpleNamespace as Obj

import numpy as np
import pytest
from conftest import flat_points, snapshot

from real_transfer.geometry import (
    base_pose,
    decode_pointcloud2,
    height_grid,
    points_in_yaw_frame,
    pose_matrix,
    quaternion_matrix,
    rigid_transform,
)


def test_heightmap_matches_exact_upstream_layout_and_convention():
    from s10_perception.heightmap import HeightmapConfig, grid_points

    values, valid, counts = height_grid(flat_points())
    assert values.shape == valid.shape == (13, 9)
    assert valid.all() and np.all(counts == 3)
    np.testing.assert_allclose(values, -0.42)
    upstream_xy = grid_points(HeightmapConfig())
    np.testing.assert_allclose(np.array(flat_points())[1::3, :2], upstream_xy)


@pytest.mark.parametrize("angle", [0, math.pi / 2, -math.pi / 2, math.pi])
def test_yaw_rotation_does_not_rotate_body_aligned_grid(angle):
    t = pose_matrix([4, 5, 7], [0, 0, math.sin(angle / 2), math.cos(angle / 2)])
    points = np.array(flat_points())
    result = points_in_yaw_frame(points, np.eye(4), t)
    np.testing.assert_allclose(result, points, atol=1e-12)


@pytest.mark.parametrize("angle", [-0.2, 0.2])
def test_pitch_uses_full_rotation_not_yaw_only(angle):
    t = pose_matrix([0, 0, 1], [0, math.sin(angle / 2), 0, math.cos(angle / 2)])
    observed = np.array(flat_points()) @ t[:3, :3]
    result = points_in_yaw_frame(observed, np.eye(4), t)
    np.testing.assert_allclose(result, flat_points(), atol=1e-12)


def test_extrinsic_translation_is_not_ignored():
    extrinsic = np.eye(4)
    extrinsic[:3, 3] = [0.1, 0.2, 0.3]
    source = np.array(flat_points()) - extrinsic[:3, 3]
    np.testing.assert_allclose(points_in_yaw_frame(source, extrinsic, np.eye(4)), flat_points())


def test_odometry_origin_is_converted_to_base_before_route_comparison():
    raw = snapshot()["inputs"]["pose"]
    extrinsic = np.eye(4)
    extrinsic[:3, 3] = [0.2, -0.1, 0.05]
    result = base_pose(raw, extrinsic)
    np.testing.assert_allclose(result[:3, 3], [0.2, -0.1, 0.47])


@pytest.mark.parametrize("q", [[0, 0, 0, 0], [0, 0, 0, 2], [0, 0, 0, math.nan]])
def test_invalid_quaternion_is_not_silently_fixed(q):
    with pytest.raises(ValueError):
        quaternion_matrix(q)


@pytest.mark.parametrize(
    "transform", [None, np.zeros((4, 4)), np.diag([-1, 1, 1, 1]), np.diag([2, 1, 1, 1]), np.eye(3)]
)
def test_unknown_reflected_or_scaled_extrinsics_are_refused(transform):
    with pytest.raises(ValueError):
        rigid_transform(transform)


@pytest.mark.parametrize("z", [-2, -1, 1, 2])
def test_clipped_terrain_is_not_aliased_to_a_valid_minus_one(z):
    values, valid, _ = height_grid(flat_points(z))
    assert not valid.any() and np.all(values == -1)


def test_holes_and_mixed_storeys_are_invalid_not_filled():
    p = flat_points()
    p = p[3:]  # One unseen cell.
    p += [[-0.45, -0.6, 0.1]]  # A second layer in the next x row.
    _, valid, _ = height_grid(p)
    assert not valid[0, 0]
    assert not valid[1, 0]
    assert valid.sum() == 115


def test_noisy_points_preserve_mask_when_within_spread():
    p = np.array(flat_points())
    p[:, 2] += np.random.default_rng(42).uniform(-0.01, 0.01, len(p))
    heights, valid, _ = height_grid(p)
    assert valid.all()
    assert np.max(np.abs(heights + 0.42)) <= 0.01


def test_empty_or_nonfinite_cloud_never_becomes_flat_ground():
    for p in (np.empty((0, 3)), np.full((10, 3), np.nan)):
        _, mask, _ = height_grid(p)
        assert not mask.any()


@pytest.mark.parametrize("endian", ["<", ">"])
@pytest.mark.parametrize("bits", [32, 64])
def test_cloud_decoder_handles_organized_row_padding_and_float_width(endian, bits):
    width = bits // 8
    step, row = 3 * width + 4, 2 * (3 * width + 4) + 8
    buffer = bytearray(row * 2)
    expected = []
    for y in range(2):
        for x in range(2):
            xyz = (x + 1.0, y + 2.0, -0.42)
            expected.append(xyz)
            struct.pack_into(
                endian + ("fff" if bits == 32 else "ddd"), buffer, y * row + x * step, *xyz
            )
    msg = Obj(
        width=2,
        height=2,
        point_step=step,
        row_step=row,
        data=bytes(buffer),
        is_bigendian=endian == ">",
        fields=[
            Obj(name=n, offset=i * width, datatype=7 if bits == 32 else 8, count=1)
            for i, n in enumerate(("x", "y", "z"))
        ],
    )
    np.testing.assert_allclose(decode_pointcloud2(msg), expected, atol=1e-6)
    msg.data = msg.data[:-1]
    with pytest.raises(ValueError, match="truncated"):
        decode_pointcloud2(msg)
