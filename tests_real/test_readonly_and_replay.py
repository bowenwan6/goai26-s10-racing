from __future__ import annotations

import ast
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import pytest
from conftest import snapshot

from real_transfer.vendor import parse_status

ROOT = Path(__file__).resolve().parents[1]


def test_no_motion_transport_or_launch_in_new_modules():
    forbidden = {
        "create_publisher",
        "create_client",
        "create_service",
        "sendto",
        "sendall",
        "Popen",
        "system",
        "exec",
        "eval",
    }
    for path in (ROOT / "real_transfer").glob("*.py"):
        tree = ast.parse(path.read_text())
        for node in ast.walk(tree):
            if isinstance(node, ast.Call):
                name = getattr(node.func, "attr", getattr(node.func, "id", ""))
                assert name not in forbidden, f"{path}: {name}"
                if name == "run":
                    assert path.name == "vendor.py"
                    argv = ast.literal_eval(node.args[0])
                    assert argv[:3] == ["systemctl", "show", "localization.service"]


def test_pending_profile_contains_no_fake_waypoints_or_identity_extrinsics():
    profile = json.loads((ROOT / "real_transfer/config.pending.json").read_text())
    assert profile["route"] == []
    assert not any(profile["verified"].values())
    assert profile["base_from_cloud"] is None
    assert profile["odom_child_from_base"] is None
    assert profile["topics"]["scan"] is None


@pytest.mark.parametrize("code,mode,expected", [(0, "全局", True), (3, "局部", False)])
def test_vendor_status_parser(code, mode, expected):
    stamp = time.mktime(time.strptime("2026-09-12 20:35:00", "%Y-%m-%d %H:%M:%S"))
    line = (
        f"[2026-09-12 20:35:00.100.000] [INFO ] [monitor] "
        f"上报状态={code}(状态), 可用=0, 运行状态={mode}"
    )
    status = parse_status(line, stamp + 0.2, stamp - 1, "session", "map-id", 10)
    assert (status["code"] == 0 and status["global"]) is expected
    assert status["stamp"] == pytest.approx(stamp + 0.1)
    with pytest.raises(ValueError, match="stale"):
        parse_status(line, stamp + 10, stamp - 1, "session", "map-id", 10)
    with pytest.raises(ValueError, match="predates"):
        parse_status(line, stamp + 0.2, stamp + 0.15, "session", "map-id", 10)


def test_replay_cli_roundtrip_and_exclusive_output(config, tmp_path):
    profile, source, target = [
        tmp_path / name for name in ("profile.json", "in.jsonl", "out.jsonl")
    ]
    profile.write_text(json.dumps(config))
    records = []
    for i in range(80):
        data = snapshot(i * 0.02)
        data["inputs"]["scan"]["ranges"] = [None] * 72
        data["inputs"]["scan"]["no_return"] = [True] * 72
        records.append(json.dumps(data, allow_nan=False))
    records += ["not json", json.dumps(data, allow_nan=False)]
    source.write_text("\n".join(records) + "\n")
    command = [
        sys.executable,
        "-m",
        "real_transfer.replay",
        "--config",
        str(profile),
        "--input",
        str(source),
        "--output",
        str(target),
    ]
    run = subprocess.run(command, cwd=ROOT, env=os.environ.copy(), capture_output=True, text=True)
    assert run.returncode == 0, run.stderr
    output = [json.loads(row) for row in target.read_text().splitlines()]
    assert len(output) == 82
    assert any(row["candidate_computed"] for row in output)
    assert all(row["transport_command"] == [0, 0, 0] for row in output)
    assert not output[-1]["candidate_computed"]
    previous = target.read_bytes()
    run = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
    assert run.returncode != 0
    assert target.read_bytes() == previous
