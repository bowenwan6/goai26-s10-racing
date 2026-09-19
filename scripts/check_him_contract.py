#!/usr/bin/env python3
"""Build/run the actual C++ runner contract check under ROS 2 (no training, no robot).

source /opt/ros/jazzy/setup.bash
python scripts/check_him_contract.py --model /path/to/policy.onnx
Optional --pristine-upstream checks reconstruction from that checkout's Git HEAD.
"""
from __future__ import annotations

import argparse
import copy
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper

ROOT = Path(__file__).resolve().parents[1]


def run(*args):
    subprocess.run([str(a) for a in args], check=True, cwd=ROOT)


def make_models(dest, fixture=None):
    # Deliberately synthetic constant actions. These exercise the interface, not gait quality.
    cfg = dict(onnx_contract_version=1, policy_type="s10_him", robot="s10",
               interface_version=2, reset_history="zero", input_name="obs", input_shape=[1, 342],
               output_name="actions", output_shape=[1, 16], one_step_observation_dim=57,
               history_length=6, history_order="newest_first", policy_dt=.02, sim_dt=.0025,
               decimation=8, self_collisions=1, initial_position=[0, 0, .45],
               dof_names=[f"{leg}_{joint}_joint" for leg in ["fl", "fr", "hl", "hr"]
                          for joint in ["hipx", "hipy", "knee", "wheel"]],
               wheel_indices=[3, 7, 11, 15],
               default_dof_pos=[0, -.3, .6, 0] * 2 + [0, .3, -.6, 0] * 2,
               p_gains=[81, 81, 81, 0] * 4, d_gains=[2.1, 2.1, 2.1, .7] * 4,
               action_scale=[.125, .25, .25, 0] * 4, vel_scale=5,
               torque_limits=[50, 50, 50, 14] * 4, dof_vel_limits=[25.76, 25.76, 25.76, 65.5] * 4,
               commands_scale=[2, 2, .25], obs_scales=dict(ang_vel=.25, dof_pos=1, dof_vel=.05),
               clip_actions=2, clip_observations=100)

    def model(name, width=342, batch=1, output_batch=1, output_width=16,
              in_name="obs", out_name="actions", dtype=TensorProto.FLOAT, nan=False, sidecar=True):
        values = (np.arange(output_width, dtype=np.float32) - 8) * .75
        if nan:
            values[0] = np.nan
        values = np.tile(values, (output_batch if isinstance(output_batch, int) else 1, 1))
        tensor = numpy_helper.from_array(values.astype(np.float64 if dtype == TensorProto.DOUBLE else np.float32))
        graph = helper.make_graph(
            [helper.make_node("Constant", [], [out_name], value=tensor)], name,
            [helper.make_tensor_value_info(in_name, dtype, [batch, width])],
            [helper.make_tensor_value_info(out_name, dtype, [output_batch, output_width])])
        if isinstance(batch, str) and width in (57, 174):
            # Keep batch genuinely symbolic in the output; Constant would infer batch=1.
            graph = helper.make_graph([
                helper.make_node("MatMul", [in_name, "weights"], ["zero"]),
                helper.make_node("Add", ["zero", "bias"], [out_name])], name,
                list(graph.input), list(graph.output), initializer=[
                    numpy_helper.from_array(np.zeros((width, output_width), dtype=np.float32), "weights"),
                    numpy_helper.from_array(values.astype(np.float32), "bias")])
        graph_model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 17)])
        graph_model.ir_version = 8  # Compatible with the SDK's bundled ORT.
        onnx.save(graph_model, dest / f"{name}.onnx")
        if sidecar:
            (dest / f"{name}.json").write_text(json.dumps(cfg))

    model("him")
    model("legacy", width=57, batch="batch", output_batch="batch", sidecar=False)
    model("heightmap", width=174, batch="batch", output_batch="batch", sidecar=False)
    model("nan", nan=True)
    model("reject_name", in_name="observations")
    model("reject_output_name", out_name="action")
    model("reject_type", dtype=TensorProto.DOUBLE)
    model("reject_width", width=341)
    model("reject_batch", batch=2)
    model("reject_him_dynamic", batch="batch")
    model("reject_action_width", output_width=15)
    model("reject_output_batch", output_batch=2)
    model("reject_missing_json", sidecar=False)
    model("reject_legacy_batch", width=57, batch=2)
    # Matching flattened width does not make a rank-3 input valid.
    model("reject_rank")
    m = onnx.load(dest / "reject_rank.onnx")
    m.graph.input[0].type.tensor_type.shape.dim.add().dim_value = 1
    onnx.save(m, dest / "reject_rank.onnx")
    for key, value in [("history_order", "oldest_first"), ("dof_names", list(reversed(cfg["dof_names"]))),
                       ("wheel_indices", [12, 13, 14, 15]), ("policy_dt", .04),
                       ("p_gains", [80] * 16), ("clip_actions", -1),
                       ("action_scale", [1] * 15), ("commands_scale", [2, 2, "bad"])]:
        name = "reject_" + key
        model(name)
        bad = copy.deepcopy(cfg)
        bad[key] = value
        (dest / f"{name}.json").write_text(json.dumps(bad))
    if fixture:
        data = json.loads(fixture.read_text())
        for name, raw in [("reference_history", data["steps"][1]["inputs"]["last_action"]),
                          ("reference_actions", data["action"]["raw"])]:
            model(name)
            graph = onnx.load(dest / f"{name}.onnx")
            graph.graph.node[0].attribute[0].t.CopyFrom(
                numpy_helper.from_array(np.asarray([raw], dtype=np.float32)))
            onnx.save(graph, dest / f"{name}.onnx")
            (dest / f"{name}.json").write_text(json.dumps(cfg | data["metadata"]))
        (dest / "deployment_fixture.json").write_text(json.dumps(data))


def check_rebuild(pristine):
    import patch_him_upstream as patch
    with tempfile.TemporaryDirectory(prefix="him-rebuild-") as tmp:
        root = Path(tmp)
        for rel in {e.path for e in patch.EDITS} | {patch.SDK / "main.cpp"}:
            target = root / rel
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(subprocess.check_output(
                ["git", "-C", str(pristine), "show", f"HEAD:{rel.as_posix()}"]))
        for _, dest in patch.HEADERS:
            (root / patch.SDK / dest).parent.mkdir(parents=True, exist_ok=True)
        run("python3", ROOT / "scripts/patch_him_upstream.py", "--upstream", root)
        before = {p.relative_to(root): p.read_bytes() for p in root.rglob("*") if p.is_file()}
        run("python3", ROOT / "scripts/patch_him_upstream.py", "--upstream", root)
        after = {p.relative_to(root): p.read_bytes() for p in root.rglob("*") if p.is_file()}
        assert before == after, "second patch application changed files"
        run("python3", ROOT / "scripts/patch_him_upstream.py", "--upstream", root, "--check")
        for _, dest in patch.HEADERS:
            target = root / patch.SDK / dest
            target.write_text(target.read_text() + "// stale\n")
            assert subprocess.run(["python3", str(ROOT / "scripts/patch_him_upstream.py"),
                                   "--upstream", str(root), "--check"], capture_output=True).returncode == 1
            patch.install_headers(root)
        print("PASS: pristine Git HEAD reconstruction, idempotence, stale-header rejection")



def check_launcher():
    with tempfile.TemporaryDirectory(prefix="him-launch-") as tmp:
        root = Path(tmp)
        for directory in ("scripts", "install", "bin"):
            (root / directory).mkdir()
        shutil.copyfile(ROOT / "scripts/run_him.sh", root / "scripts/run_him.sh")
        (root / "scripts/patch_him_upstream.py").write_text("")
        (root / "scripts/runtime_fingerprint.py").write_text("")
        (root / "install/setup.bash").write_text("")
        (root / "policy.onnx").write_text("launcher fixture")
        ros = root / "bin/ros2"
        ros.write_text('#!/bin/sh\nprintf "%s|%s|%s|%s\\n" "$*" "$S10_SECOND_POLICY_PATH" "$S10_DOWN_POLICY_PATH" "$S10_SPEEDTURN_POLICY_PATH" >> "$LAUNCH_LOG"\n/bin/sleep .2\n')
        ros.chmod(0o755)
        delay = root / "bin/sleep"
        delay.write_text("#!/bin/sh\nexit 0\n")
        delay.chmod(0o755)
        env = dict(os.environ, PATH=str(root / "bin") + os.pathsep + os.environ["PATH"],
                   S10_POLICY_PATH=str(root / "policy.onnx"), LAUNCH_LOG=str(root / "launch.log"))
        for key in ("S10_SECOND_POLICY_PATH", "S10_DOWN_POLICY_PATH", "S10_SPEEDTURN_POLICY_PATH", "S10_INSTALL_BASE"):
            env.pop(key, None)
        subprocess.run(["bash", str(root / "scripts/run_him.sh")], env=env, check=True)
        log = (root / "launch.log").read_text()
        assert "run s10_sdk_deploy rl_deploy|||" in log
        assert "launch s10_bringup race.launch.py strategy_router:=false|||" in log
        env["S10_SECOND_POLICY_PATH"] = str(root / "missing.onnx")
        failed = subprocess.run(["bash", str(root / "scripts/run_him.sh")], env=env, capture_output=True)
        assert failed.returncode != 0 and b"policy not found" in failed.stderr
        print("PASS: HIM launcher defaults, Gate16 exclusion, missing-model rejection")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path)
    parser.add_argument("--pristine-upstream", type=Path)
    parser.add_argument("--fixture", type=Path, help="Training deploy/s10_mujoco.py reference fixture")
    args = parser.parse_args()
    if args.pristine_upstream:
        check_rebuild(args.pristine_upstream)
    check_launcher()
    output = ROOT / "results/him-contract"
    output.mkdir(parents=True, exist_ok=True)
    make_models(output, args.fixture)
    run("cmake", "-S", ROOT / "integration/test", "-B", output / "build")
    run("cmake", "--build", output / "build", "-j2")
    command = [output / "build/him_contract", output]
    if args.model:
        command.append(args.model.resolve())
    os.environ.setdefault("ROS_DOMAIN_ID", "83")
    os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")
    result = subprocess.run([str(x) for x in command], cwd=ROOT, text=True, capture_output=True)
    print(result.stdout)
    (output / "verification.log").write_text(result.stdout + result.stderr)
    result.check_returncode()


if __name__ == "__main__":
    main()
