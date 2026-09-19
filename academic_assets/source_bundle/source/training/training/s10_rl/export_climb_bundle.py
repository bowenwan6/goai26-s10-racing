#!/usr/bin/env python3
"""Export the deployable S10 base actor plus gated climbing residual.

The residual checkpoint is not a standalone robot policy.  Deployment must keep the
174-D base actor and add the residual only while the stateful height-map gate is active.
This command emits both ONNX graphs and a manifest consumed by the patched SDK runner.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from dataclasses import asdict
from pathlib import Path

import numpy as np

from s10_rl.export_onnx import export, load_policy, verify
from s10_rl.observation import PERCEPTIVE
from s10_rl.skill_gate import SkillGateConfig


BASE_NAME = "policy.onnx"
RESIDUAL_NAME = "climb_residual.onnx"
MANIFEST_NAME = "climb_policy_manifest.json"
CORRECTION_SCALE = [1.0] * 12 + [6.0] * 4
ACTION_GUARD = [8.0] * 12 + [25.0] * 4


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _numeric_check(path: Path, policy, seed: int) -> float:
    try:
        import onnxruntime as ort
    except ImportError:
        raise SystemExit("numeric verification needs onnxruntime") from None
    import torch

    rng = np.random.default_rng(seed)
    observations = rng.normal(0.0, 0.25, (8, PERCEPTIVE.dim)).astype(np.float32)
    with torch.no_grad():
        expected = policy(torch.as_tensor(observations)).cpu().numpy()
    session = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
    actual = session.run(["actions"], {"obs": observations})[0]
    return float(np.max(np.abs(expected - actual)))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-checkpoint", type=Path, required=True)
    parser.add_argument("--residual-checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--opset", type=int, default=17)
    args = parser.parse_args()

    args.output.mkdir(parents=True, exist_ok=True)
    base_path = args.output / BASE_NAME
    residual_path = args.output / RESIDUAL_NAME
    base = load_policy(args.base_checkpoint)
    residual = load_policy(args.residual_checkpoint)
    export(base, PERCEPTIVE, base_path, args.opset)
    export(residual, PERCEPTIVE, residual_path, args.opset)
    verify(base_path, PERCEPTIVE)
    verify(residual_path, PERCEPTIVE)
    base_error = _numeric_check(base_path, base, 501)
    residual_error = _numeric_check(residual_path, residual, 502)
    if max(base_error, residual_error) > 1.0e-4:
        raise SystemExit(
            f"ONNX numeric mismatch: base={base_error:.3e}, residual={residual_error:.3e}"
        )

    manifest = {
        "format": "s10-gated-residual-v1",
        "observation_dim": PERCEPTIVE.dim,
        "action_dim": 16,
        "base_onnx": BASE_NAME,
        "residual_onnx": RESIDUAL_NAME,
        "base_sha256": _sha256(base_path),
        "residual_sha256": _sha256(residual_path),
        "base_checkpoint": str(args.base_checkpoint.resolve()),
        "residual_checkpoint": str(args.residual_checkpoint.resolve()),
        "correction_scale": CORRECTION_SCALE,
        "correction_limit": 4.0,
        "action_guard": ACTION_GUARD,
        "skill_gate": asdict(SkillGateConfig()),
        "numeric_max_abs_error": {
            "base": base_error,
            "residual": residual_error,
        },
    }
    (args.output / MANIFEST_NAME).write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps(manifest, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

