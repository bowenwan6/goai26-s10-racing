from __future__ import annotations

import copy
import json
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "scripts"))

from patch_upstream import validate_gate16_manifest  # noqa: E402
from runtime_fingerprint import (  # noqa: E402
    CONTRACT_FILES,
    check_fingerprint,
    write_fingerprint,
)


def test_gate16_deployment_contract_rejects_the_old_adaptive_manifest():
    manifest = json.loads((ROOT / "policy/gate16/climb_policy_manifest.json").read_text())
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
