"""Reproducible offline regression runner, with explicit untested scopes."""

from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
import os
import platform
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EXCLUDED = {
    "src/s10_auto_nav/test/test_gate16_policy.py": "Gate16 ONNX/manifest bundle absent",
    "src/s10_auto_nav/test/test_stairs_stable_policy.py": "stairs ONNX/manifest bundle absent",
    "src/s10_auto_nav/test/test_runtime_contract.py": "model manifest and built SDK absent",
    "src/s10_perception/test/test_viewer_node.py": "upstream SDK model/module absent",
}
RECORDER_CASE = (
    "src/s10_auto_nav/test/test_segment_recorder.py::"
    "test_ordered_evidence_accepts_exactly_018_and_only_one_gate_per_tick"
)
UNAVAILABLE_CASES = {
    RECORDER_CASE: "Unguarded rclpy import: failed in run-01; ROS 2 unavailable, not a pass",
}


def counts(path):
    root = ET.parse(path).getroot()
    suites = root.findall("testsuite") if root.tag == "testsuites" else [root]
    return {
        name: sum(int(s.get(name, 0)) for s in suites)
        for name in ("tests", "failures", "errors", "skipped")
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=False)
    manifest = json.loads((ROOT / "real_transfer/source_snapshot.json").read_text())
    changed = [
        name
        for name, digest in manifest["materialized_files"].items()
        if not (ROOT / name).is_file()
        or hashlib.sha256((ROOT / name).read_bytes()).hexdigest() != digest
    ]
    summary = {
        "source_commit": manifest["source_commit"],
        "source_files_changed": changed,
        "platform": platform.platform(),
        "python": sys.version,
        "versions": {
            p: importlib.metadata.version(p)
            for p in ("numpy", "PyYAML", "pytest", "mujoco", "ruff")
        },
        "excluded_test_files": EXCLUDED,
        "unavailable_test_cases": UNAVAILABLE_CASES,
        "full_course_dynamics_run": False,
        "live_ros_run": False,
        "robot_contacted": False,
        "hardware_motion_interface_implemented": False,
        "transfer_sources": {
            str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
            for folder in ("real_transfer", "tests_real")
            for path in sorted((ROOT / folder).glob("*.py"))
        },
        "runs": {},
    }
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join(
        str(ROOT / p) for p in (".", "src/s10_auto_nav", "src/s10_perception")
    )
    jobs = {
        "baseline": [
            sys.executable,
            "-m",
            "pytest",
            "-q",
            "src",
            *("--ignore=" + p for p in EXCLUDED),
            *("--deselect=" + p for p in UNAVAILABLE_CASES),
        ],
        "transfer": [sys.executable, "-m", "pytest", "-q", "tests_real"],
        "lint": [
            sys.executable,
            "-m",
            "ruff",
            "check",
            "real_transfer",
            "tests_real",
            "scripts/real_transfer_snapshot.py",
            "scripts/real_transfer_verify.py",
        ],
    }
    failed = bool(changed)
    for name, command in jobs.items():
        if name != "lint":
            command += [f"--junitxml={out / (name + '.xml')}"]
        started = time.monotonic()
        with (out / f"{name}.log").open("w") as log:
            result = subprocess.run(
                command, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT, timeout=180
            )
        record = {
            "exit_code": result.returncode,
            "elapsed_s": round(time.monotonic() - started, 3),
            "command": command,
        }
        if name != "lint" and (out / f"{name}.xml").is_file():
            record.update(counts(out / f"{name}.xml"))
        summary["runs"][name] = record
        failed |= result.returncode != 0
        print(name, json.dumps({k: v for k, v in record.items() if k != "command"}), flush=True)
    summary["offline_checks_passed"] = not failed
    (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(f"Evidence: {out / 'summary.json'}")
    return int(failed)


if __name__ == "__main__":
    raise SystemExit(main())
