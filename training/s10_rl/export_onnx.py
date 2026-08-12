#!/usr/bin/env python3
"""Export a trained policy to the ONNX file the deployment SDK loads.

The C++ runner binds its input and output tensors by name, so an export with different
names loads and then fails at inference. It also hardcodes the observation width, so the
exported graph and ``observation_dim`` must agree. Both are checked here rather than
discovered on the robot.

Usage:
    python -m s10_rl.export_onnx checkpoint.pt -o policy.onnx
    python -m s10_rl.export_onnx checkpoint.pt --spec baseline --verify
"""

from __future__ import annotations

import argparse
from pathlib import Path

import torch

from s10_rl.observation import ACTION_DIM, BASELINE, PERCEPTIVE, ObservationSpec, cpp_constant

#: Must match `input_names_` and `output_names_` in s10_policy_runner.hpp.
INPUT_NAME = "obs"
OUTPUT_NAME = "actions"

SPECS: dict[str, ObservationSpec] = {"baseline": BASELINE, "perceptive": PERCEPTIVE}


def load_policy(checkpoint: Path) -> torch.nn.Module:
    """Load a checkpoint as an inference-ready module.

    Accepts either a scripted/pickled ``nn.Module`` or a dict checkpoint carrying one
    under a common key, which covers the layouts the usual RL frameworks emit.
    """
    obj = torch.load(checkpoint, map_location="cpu", weights_only=False)

    if isinstance(obj, dict):
        for key in ("policy", "model", "actor", "network"):
            if isinstance(obj.get(key), torch.nn.Module):
                obj = obj[key]
                break
        else:
            raise SystemExit(
                f"{checkpoint} is a dict with keys {sorted(obj)}; none hold an nn.Module. "
                "Export the module itself, or extend load_policy for your framework."
            )

    if not isinstance(obj, torch.nn.Module):
        raise SystemExit(f"{checkpoint} did not yield an nn.Module (got {type(obj).__name__})")

    return obj.eval()


def export(policy: torch.nn.Module, spec: ObservationSpec, output: Path, opset: int = 17) -> None:
    dummy = torch.zeros(1, spec.dim, dtype=torch.float32)

    with torch.no_grad():
        actions = policy(dummy)
    if actions.shape[-1] != ACTION_DIM:
        raise SystemExit(
            f"Policy emits {actions.shape[-1]} actions but the robot has {ACTION_DIM} joints"
        )

    output.parent.mkdir(parents=True, exist_ok=True)
    torch.onnx.export(
        policy,
        dummy,
        str(output),
        input_names=[INPUT_NAME],
        output_names=[OUTPUT_NAME],
        opset_version=opset,
        dynamic_axes={INPUT_NAME: {0: "batch"}, OUTPUT_NAME: {0: "batch"}},
    )


def verify(path: Path, spec: ObservationSpec) -> None:
    """Load the exported graph the same way the deployed runner does."""
    try:
        import onnxruntime as ort
    except ImportError:
        raise SystemExit("--verify needs onnxruntime: pip install onnxruntime") from None

    import numpy as np

    session = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])

    inputs = {i.name: i.shape for i in session.get_inputs()}
    outputs = [o.name for o in session.get_outputs()]
    if INPUT_NAME not in inputs:
        raise SystemExit(f"Input must be named '{INPUT_NAME}', found {sorted(inputs)}")
    if OUTPUT_NAME not in outputs:
        raise SystemExit(f"Output must be named '{OUTPUT_NAME}', found {outputs}")

    width = inputs[INPUT_NAME][-1]
    if isinstance(width, int) and width != spec.dim:
        raise SystemExit(f"Graph takes {width} inputs, spec '{spec.name}' declares {spec.dim}")

    result = session.run([OUTPUT_NAME], {INPUT_NAME: np.zeros((1, spec.dim), np.float32)})[0]
    if result.shape[-1] != ACTION_DIM:
        raise SystemExit(f"Graph emits {result.shape[-1]} actions, expected {ACTION_DIM}")

    print(f"Verified {path}: {spec.dim} -> {ACTION_DIM}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("checkpoint", type=Path)
    parser.add_argument("-o", "--output", type=Path, default=Path("policy.onnx"))
    parser.add_argument("--spec", choices=sorted(SPECS), default="perceptive")
    parser.add_argument("--opset", type=int, default=17)
    parser.add_argument("--verify", action="store_true", help="Re-load and test the export")
    args = parser.parse_args()

    spec = SPECS[args.spec]
    export(load_policy(args.checkpoint), spec, args.output, args.opset)
    print(f"Wrote {args.output} ({spec.name}, {spec.dim} -> {ACTION_DIM})")

    if args.verify:
        verify(args.output, spec)

    print()
    print("Ensure the deployment SDK agrees:")
    print(f"  {cpp_constant(spec)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
