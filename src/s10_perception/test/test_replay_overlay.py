from __future__ import annotations

import importlib.util
import shutil
import subprocess
from pathlib import Path
from types import SimpleNamespace

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


def test_timing_manifest_uses_one_frame_for_unmeasured_final_sample(tmp_path):
    timing = np.array([5.0, 5.1, 5.2, 5.3, 5.4])
    args = SimpleNamespace(
        out=tmp_path,
        stride=1,
        timing="wall",
        overlay_font="/System/Library/Fonts/Supplemental/Arial.ttf",
    )

    RENDER._write_timing_files(args, _Trace(), timing, qpos_count=len(timing))

    manifest = (tmp_path / "frames.ffconcat").read_text().splitlines()
    assert manifest.count("file '00004.png'") == 2
    assert manifest[-2] == "duration 0.033333333"
    assert manifest[-1] == "file '00004.png'"
    assert (tmp_path / "overlay_filters.txt").is_file()


def test_simulation_timing_manifest_writes_overlay(tmp_path):
    timing = np.array([0.02, 0.12, 0.22, 0.32, 0.42])
    args = SimpleNamespace(
        out=tmp_path,
        stride=2,
        timing="simulation",
        overlay_font="/System/Library/Fonts/Supplemental/Arial.ttf",
    )

    RENDER._write_timing_files(args, _Trace(), timing, qpos_count=len(timing))

    overlay = (tmp_path / "overlay_filters.txt").read_text()
    assert "Elapsed" in overlay
    assert "WP00 PASSED" in overlay
