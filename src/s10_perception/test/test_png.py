"""The PNG writer is hand-rolled, so it is checked against a decoder rather than by eye."""

from __future__ import annotations

import struct
import zlib

import numpy as np
import pytest

from s10_perception.png import write_png


def decode(path) -> np.ndarray:
    """Minimal reader for exactly the subset write_png emits: 8-bit RGB, filter 0."""
    blob = path.read_bytes()
    assert blob[:8] == b"\x89PNG\r\n\x1a\n"
    chunks: dict[bytes, bytes] = {}
    offset = 8
    while offset < len(blob):
        (length,) = struct.unpack(">I", blob[offset : offset + 4])
        tag = blob[offset + 4 : offset + 8]
        payload = blob[offset + 8 : offset + 8 + length]
        (crc,) = struct.unpack(">I", blob[offset + 8 + length : offset + 12 + length])
        assert crc == zlib.crc32(tag + payload), f"{tag!r} chunk is corrupt"
        chunks.setdefault(tag, b"")
        chunks[tag] += payload
        offset += 12 + length

    width, height, depth, colour = struct.unpack(">IIBB", chunks[b"IHDR"][:10])
    assert (depth, colour) == (8, 2)
    raw = zlib.decompress(chunks[b"IDAT"])
    stride = width * 3 + 1
    rows = [raw[y * stride + 1 : (y + 1) * stride] for y in range(height)]
    assert all(raw[y * stride] == 0 for y in range(height)), "expected filter 0 on every row"
    return np.frombuffer(b"".join(rows), dtype=np.uint8).reshape(height, width, 3)


def test_a_frame_survives_the_round_trip(tmp_path):
    rng = np.random.default_rng(0)
    frame = rng.integers(0, 256, (7, 11, 3), dtype=np.uint8)
    path = tmp_path / "frame.png"
    write_png(path, frame)
    assert decode(path) == pytest.approx(frame)


def test_the_shape_is_checked_rather_than_reinterpreted(tmp_path):
    with pytest.raises(ValueError, match="RGB"):
        write_png(tmp_path / "bad.png", np.zeros((4, 4), dtype=np.uint8))
    with pytest.raises(ValueError, match="RGB"):
        write_png(tmp_path / "bad.png", np.zeros((4, 4, 4), dtype=np.uint8))


def test_compression_level_is_validated(tmp_path):
    with pytest.raises(ValueError, match="compression"):
        write_png(
            tmp_path / "bad.png",
            np.zeros((1, 1, 3), dtype=np.uint8),
            compression=10,
        )
