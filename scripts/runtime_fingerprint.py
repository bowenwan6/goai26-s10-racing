#!/usr/bin/env python3
"""Bind a built runtime to the repository files that define its control contract."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
FINGERPRINT_NAME = ".s10_runtime_fingerprint.json"
CONTRACT_FILES = (
    "integration/gate16_policy_runner.hpp",
    "integration/gate16_perception_buffer.hpp",
    "integration/gate16_skill_gate.hpp",
    "integration/gate16_policy_symmetry.hpp",
    "integration/joint_command_owner.hpp",
    "policy/gate16/climb_policy_manifest.json",
    "policy/gate16/front_tuck_command_profiles.json",
    "src/s10_auto_nav/s10_auto_nav/strategy/router.py",
    "src/s10_auto_nav/s10_auto_nav/follower_node.py",
    "src/s10_auto_nav/s10_auto_nav/local_planner.py",
    "src/s10_bringup/config/strategy_gate16.yaml",
    "src/s10_bringup/config/nav.yaml",
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def source_fingerprint(root: Path = ROOT) -> dict[str, str]:
    missing = [name for name in CONTRACT_FILES if not (root / name).is_file()]
    if missing:
        raise RuntimeError(f"runtime contract files missing: {', '.join(missing)}")
    return {name: _sha256(root / name) for name in CONTRACT_FILES}


def fingerprint_path(install_base: Path) -> Path:
    return install_base / FINGERPRINT_NAME


def write_fingerprint(install_base: Path, root: Path = ROOT) -> Path:
    binary = install_base / "s10_sdk_deploy/lib/s10_sdk_deploy/rl_deploy"
    if not binary.is_file():
        raise RuntimeError(f"built Gate16 executable missing: {binary}")
    payload = {
        "schema": 1,
        "sources": source_fingerprint(root),
        "rl_deploy_sha256": _sha256(binary),
    }
    output = fingerprint_path(install_base)
    output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    return output


def check_fingerprint(install_base: Path, root: Path = ROOT) -> None:
    path = fingerprint_path(install_base)
    if not path.is_file():
        raise RuntimeError(
            f"runtime fingerprint missing: {path}; rebuild this checkout before running"
        )
    recorded = json.loads(path.read_text())
    if recorded.get("schema") != 1:
        raise RuntimeError("unsupported runtime fingerprint schema")
    current = source_fingerprint(root)
    if recorded.get("sources") != current:
        changed = sorted(
            name
            for name in set(recorded.get("sources", {})) | set(current)
            if recorded.get("sources", {}).get(name) != current.get(name)
        )
        raise RuntimeError(
            "built runtime does not match this checkout: " + ", ".join(changed)
        )
    binary = install_base / "s10_sdk_deploy/lib/s10_sdk_deploy/rl_deploy"
    if not binary.is_file() or _sha256(binary) != recorded.get("rl_deploy_sha256"):
        raise RuntimeError("rl_deploy binary does not match the recorded build")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("write", "check"))
    parser.add_argument("--install-base", required=True, type=Path)
    args = parser.parse_args()
    try:
        if args.mode == "write":
            path = write_fingerprint(args.install_base)
            print(f"wrote runtime fingerprint: {path}")
        else:
            check_fingerprint(args.install_base)
            print("ok: build and source runtime contracts match")
    except (OSError, RuntimeError, ValueError, json.JSONDecodeError) as error:
        raise SystemExit(f"error: {error}") from error
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
