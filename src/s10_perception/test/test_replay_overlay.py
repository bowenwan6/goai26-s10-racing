from __future__ import annotations

import importlib.util
import shutil
import subprocess
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location(
    "render_replay_3d", ROOT / "scripts" / "render_replay_3d.py"
)
RENDER = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(RENDER)


class _Trace:
    def __init__(self):
        self.data = {
            "target_waypoint": np.array([0, 0, 1, 1, 2]),
            "active_policy": np.array(["official", "official", "WP16", "WP16", "official"]),
            "joint_owner": np.array(["official", "official", "gate16", "gate16", "official"]),
        }
        self.files = list(self.data)

    def __getitem__(self, key):
        return self.data[key]


def test_overlay_contains_elapsed_policy_owner_target_and_pass_events(tmp_path):
    output = tmp_path / "overlay_filters.txt"
    font = "/System/Library/Fonts/Supplemental/Arial.ttf"
    RENDER._write_overlay_filter(
        _Trace(), np.array([5.0, 5.1, 5.2, 5.3, 5.4]), output, font
    )
    text = output.read_text()
    assert "Elapsed" in text
    assert f"fontfile='{font}'" in text
    assert "Target WP00" in text and "Target WP02" in text
    assert "Policy WP16" in text and "Joint owner gate16" in text
    assert "WP00 PASSED" in text and "WP01 PASSED" in text
    assert "between(t\\,0.200\\,0.400)" in text
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is not None:
        subprocess.run(
            [
                ffmpeg,
                "-v",
                "error",
                "-f",
                "lavfi",
                "-i",
                "color=size=320x180:rate=10:duration=0.5",
                "-filter_script:v",
                str(output),
                "-f",
                "null",
                "-",
            ],
            check=True,
        )
