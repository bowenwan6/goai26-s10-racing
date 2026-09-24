#!/usr/bin/env python3
"""Bind a built runtime to the repository files that define its control contract."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]   # <repo>/robot/scripts/
FINGERPRINT_NAME = ".s10_runtime_fingerprint.json"
CONTRACT_FILES = (
    "robot/integration/gate16_policy_runner.hpp",
    "robot/integration/gate16_perception_buffer.hpp",
    "robot/integration/gate16_skill_gate.hpp",
    "robot/integration/gate16_policy_symmetry.hpp",
    "robot/integration/joint_command_owner.hpp",
    "models/deployed/gate16/climb_policy_manifest.json",
    "models/deployed/gate16/front_tuck_command_profiles.json",
    "src/s10_auto_nav/s10_auto_nav/strategy/router.py",
    "src/s10_auto_nav/s10_auto_nav/follower_node.py",
    "src/s10_auto_nav/s10_auto_nav/local_planner.py",
    "src/s10_bringup/config/strategy_gate16.yaml",
    "src/s10_bringup/config/nav.yaml",
)
HIM_CONTRACT_FILES = (
    "robot/integration/s10_policy_runner.hpp",
    "robot/integration/rl_control_state.hpp",
    "robot/integration/standup_state.hpp",
    "scripts/patch_him_upstream.py",
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def source_fingerprint(root: Path = ROOT, controller: str = "gate16") -> dict[str, str]:
    files = CONTRACT_FILES + (HIM_CONTRACT_FILES if controller == "him" else ())
    missing = [name for name in files if not (root / name).is_file()]
    if missing:
        raise RuntimeError(f"runtime contract files missing: {', '.join(missing)}")
    return {name: _sha256(root / name) for name in files}


def fingerprint_path(install_base: Path) -> Path:
    return install_base / FINGERPRINT_NAME


def write_fingerprint(install_base: Path, root: Path = ROOT) -> Path:
    binary = install_base / "s10_sdk_deploy/lib/s10_sdk_deploy/rl_deploy"
    if not binary.is_file():
        raise RuntimeError(f"built SDK executable missing: {binary}")
    sdk_controller = root / "upstream/goai_embodied_future_material/src/S10_sdk_deploy/state_machine/quadruped_wheel/rl_control_state.hpp"
    controller = "him" if sdk_controller.is_file() and "BlendHimHandover" in sdk_controller.read_text() else "gate16"
    payload = {
        "schema": 1,
        "controller": controller,
        "sources": source_fingerprint(root, controller),
        "rl_deploy_sha256": _sha256(binary),
    }
    output = fingerprint_path(install_base)
    output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    return output


def check_fingerprint(install_base: Path, root: Path = ROOT, controller: str = "gate16") -> None:
    path = fingerprint_path(install_base)
    if not path.is_file():
        raise RuntimeError(
            f"runtime fingerprint missing: {path}; rebuild this checkout before running"
        )
    recorded = json.loads(path.read_text())
    if recorded.get("schema") != 1:
        raise RuntimeError("unsupported runtime fingerprint schema")
    if recorded.get("controller", "gate16") != controller:
        raise RuntimeError(f"built SDK controller is not {controller}; select its patcher and rebuild")
    current = source_fingerprint(root, controller)
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
    parser.add_argument("--controller", choices=("gate16", "him"), default="gate16")
    args = parser.parse_args()
    try:
        if args.mode == "write":
            path = write_fingerprint(args.install_base)
            print(f"wrote runtime fingerprint: {path}")
        else:
            check_fingerprint(args.install_base, controller=args.controller)
            print("ok: build and source runtime contracts match")
    except (OSError, RuntimeError, ValueError, json.JSONDecodeError) as error:
        raise SystemExit(f"error: {error}") from error
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
