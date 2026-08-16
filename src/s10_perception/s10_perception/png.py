"""Write a PNG with no image library.

The container has no imageio, no Pillow, no OpenCV and no ffmpeg. Adding one of them to the
production dependencies so that a debug run can save frames would be the wrong trade -- the
race needs none of it. PNG's stored format is simple enough to emit directly: a filter byte
in front of every row, zlib over the lot, three CRC-checked chunks.
"""

from __future__ import annotations

import struct
import zlib
from pathlib import Path

import numpy as np


def _chunk(tag: bytes, payload: bytes) -> bytes:
    body = tag + payload
    return struct.pack(">I", len(payload)) + body + struct.pack(">I", zlib.crc32(body))


def write_png(path: str | Path, rgb: np.ndarray) -> None:
    """Write an ``(h, w, 3)`` uint8 array to ``path``."""
    rgb = np.ascontiguousarray(rgb, dtype=np.uint8)
    if rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError(f"expected an (h, w, 3) RGB array, got {rgb.shape}")
    height, width, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(height))
    Path(path).write_bytes(
        b"\x89PNG\r\n\x1a\n"
        + _chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
        + _chunk(b"IDAT", zlib.compress(raw, 6))
        + _chunk(b"IEND", b"")
    )
