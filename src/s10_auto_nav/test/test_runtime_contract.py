from __future__ import annotations

import copy
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "robot" / "scripts"))

from patch_upstream import validate_gate16_manifest  # noqa: E402
from runtime_fingerprint import (  # noqa: E402
    CONTRACT_FILES,
    HIM_CONTRACT_FILES,
    check_fingerprint,
    write_fingerprint,
)


def test_him_build_cannot_run_through_the_default_gate16_launcher(tmp_path):
    for relative in CONTRACT_FILES + HIM_CONTRACT_FILES:
        path = tmp_path / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("source")
    sdk = tmp_path / "upstream/goai_embodied_future_material/src/S10_sdk_deploy/state_machine/quadruped_wheel/rl_control_state.hpp"
    sdk.parent.mkdir(parents=True)
    sdk.write_text("BlendHimHandover")
    install = tmp_path / "install"
    binary = install / "s10_sdk_deploy/lib/s10_sdk_deploy/rl_deploy"
    binary.parent.mkdir(parents=True)
    binary.write_bytes(b"HIM executable")
    write_fingerprint(install, tmp_path)
    with pytest.raises(RuntimeError, match="controller is not gate16"):
        check_fingerprint(install, tmp_path)
    check_fingerprint(install, tmp_path, controller="him")
    (tmp_path / HIM_CONTRACT_FILES[0]).write_text("changed HIM source")
    with pytest.raises(RuntimeError, match="does not match this checkout"):
        check_fingerprint(install, tmp_path, controller="him")


def test_gate16_deployment_contract_rejects_the_old_adaptive_manifest():
    manifest = json.loads((ROOT / "models/deployed/gate16/climb_policy_manifest.json").read_text())
    validate_gate16_manifest(manifest)

    adaptive = copy.deepcopy(manifest)
    adaptive["format"] = "s10-gated-residual-front-tuck-v4"
    adaptive["full_stack_runtime"].pop("fallback_max_forward_mps")
    adaptive["racing_integration"].pop("fallback_owner_request")
    with pytest.raises(ValueError, match="manifest format"):
        validate_gate16_manifest(adaptive)


def test_runtime_fingerprint_rejects_a_mixed_source_and_binary(tmp_path):
    source = tmp_path / "source"
    for relative in CONTRACT_FILES:
        path = source / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(f"contract for {relative}\n")
    install = tmp_path / "install"
    binary = install / "s10_sdk_deploy/lib/s10_sdk_deploy/rl_deploy"
    binary.parent.mkdir(parents=True)
    binary.write_bytes(b"compiled runtime")

    write_fingerprint(install, source)
    check_fingerprint(install, source)

    (source / CONTRACT_FILES[0]).write_text("stale source mounted over the build\n")
    with pytest.raises(RuntimeError, match="does not match this checkout"):
        check_fingerprint(install, source)

    (source / CONTRACT_FILES[0]).write_text(f"contract for {CONTRACT_FILES[0]}\n")
    binary.write_bytes(b"different compiled runtime")
    with pytest.raises(RuntimeError, match="binary does not match"):
        check_fingerprint(install, source)


@pytest.mark.parametrize("use_venv", [False, True])
def test_build_preserves_output_paths_with_both_colcon_launchers(tmp_path, use_venv):
    bash = shutil.which("bash")
    if bash is None:
        pytest.skip("bash is required")
    root = tmp_path / "workspace with spaces"
    (root / "robot" / "scripts").mkdir(parents=True)
    (root / "upstream/goai_embodied_future_material/src").mkdir(parents=True)
    shutil.copyfile(ROOT / "robot/scripts/build.sh", root / "robot/scripts/build.sh")
    setup = tmp_path / "ros-setup.bash"
    setup.write_text("")
    bindir = tmp_path / "bin"
    bindir.mkdir()
    launcher = root / ".venv/bin/python" if use_venv else bindir / "colcon"
    launcher.parent.mkdir(parents=True, exist_ok=True)
    launcher.write_text('#!/bin/sh\nprintf "%s\\n" "$@" > "$BUILD_ARGS"\n')
    launcher.chmod(0o755)
    python = bindir / "python3"
    python.write_text('#!/bin/sh\nprintf "%s\\n" "$@" > "$FINGERPRINT_ARGS"\n')
    python.chmod(0o755)
    args_file = tmp_path / "build-args"
    fingerprint_file = tmp_path / "fingerprint-args"
    env = {
        **os.environ,
        "PATH": f"{bindir}{os.pathsep}{os.environ['PATH']}",
        "ROS_DISTRO_SETUP": str(setup),
        "BUILD_PLATFORM": "arm64",
        "S10_BUILD_BASE": str(tmp_path / "build output"),
        "S10_INSTALL_BASE": str(tmp_path / "install output"),
        "S10_LOG_BASE": str(tmp_path / "log output"),
        "BUILD_ARGS": str(args_file),
        "FINGERPRINT_ARGS": str(fingerprint_file),
    }
    subprocess.run(
        [bash, str(root / "robot/scripts/build.sh"), "--packages-up-to", "s10_perception"],
        env=env,
        check=True,
        capture_output=True,
        text=True,
    )
    assert args_file.read_text().splitlines() == [
        *(["-m", "colcon"] if use_venv else []),
        "--log-base",
        env["S10_LOG_BASE"],
        "build",
        "--base-paths",
        "src",
        str(root / "upstream/goai_embodied_future_material/src"),
        "--build-base",
        env["S10_BUILD_BASE"],
        "--install-base",
        env["S10_INSTALL_BASE"],
        "--symlink-install",
        "--cmake-args",
        "-DBUILD_PLATFORM=arm",
        "--packages-up-to",
        "s10_perception",
    ]
    assert fingerprint_file.read_text().splitlines() == [
        str(root / "robot/scripts/runtime_fingerprint.py"),
        "write",
        "--install-base",
        env["S10_INSTALL_BASE"],
    ]
