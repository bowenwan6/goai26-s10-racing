#!/usr/bin/env python3
"""Wire the autonomy command and Gate 16 controllers into the contest SDK.

Both are things the SDK has no hook for and that cannot be done from outside the process.

The first is a command source. The SDK selects its velocity command from a fixed enum of
input devices, all of them an operator holding something; autonomy needs one more option, a
ROS topic. That is integration/ros_cmd_interface.hpp.

The second is ownership of the actuators. /JOINTS_CMD is created and written by DdsInterface
inside rl_deploy, so no external node can take it away, and ROS treats a second publisher on
that topic as a legal silent merge rather than as the collision it is. The gate therefore has
to live where the writing happens, and the edits below put it there: every joint command
RLControlState produces passes through integration/joint_command_owner.hpp first.

Applying the edits from a script keeps upstream a pristine checkout that can be re-cloned or
updated at any time, and keeps our actual contribution in two reviewable headers rather than
in a diff buried in a vendored tree.

Every edit is idempotent: running this twice is a no-op, and --check reports whether the
checkout is patched without modifying it.

Usage:
    scripts/patch_upstream.py            # apply
    scripts/patch_upstream.py --check    # report status, exit 1 if unpatched
    scripts/patch_upstream.py --revert   # undo via git checkout, then remove our headers
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]   # <repo>/robot/scripts/
DEFAULT_UPSTREAM = REPO_ROOT / "upstream/goai_embodied_future_material"

SDK = Path("src/S10_sdk_deploy")

GATE16_DEPLOYMENT_FORMAT = "s10-gated-residual-front-tuck-v1.5"
GATE16_RUNTIME_KEYS = (
    "residual_engage_edge_distance_m",
    "front_support_edge_distance_m",
    "front_support_confirm_policy_steps",
    "rear_push_hold_policy_steps",
    "fallback_max_forward_mps",
    "confidence_gated_fast_adapter",
)


def validate_gate16_manifest(manifest: dict) -> None:
    """Reject a policy bundle that cannot satisfy the deployed runner contract.

    The ONNX graphs are deliberately shared by adaptive-v3 and v1.5, so model hashes alone
    cannot detect the dangerous mixed-version case.  A newer runner paired with the older
    manifest silently defaulted to immediate residual engagement, no fallback velocity cap,
    and the fast adapter.  Treat the control metadata as part of the executable contract.
    """
    if manifest.get("format") != GATE16_DEPLOYMENT_FORMAT:
        raise ValueError(
            f"Gate16 manifest format {manifest.get('format')!r}; "
            f"expected {GATE16_DEPLOYMENT_FORMAT!r}"
        )
    if (manifest.get("observation_dim"), manifest.get("action_dim")) != (174, 16):
        raise ValueError("Gate16 manifest must declare the 174D->16D contract")

    profile = manifest.get("front_tuck_command_profile")
    if not isinstance(profile, dict) or profile.get("runner_applies_profile") is not True:
        raise ValueError("Gate16 v1.5 runner profile contract is missing or disabled")

    runtime = manifest.get("full_stack_runtime")
    if not isinstance(runtime, dict):
        raise ValueError("Gate16 full_stack_runtime is required")
    missing = [key for key in GATE16_RUNTIME_KEYS if key not in runtime]
    if missing:
        raise ValueError(f"Gate16 full_stack_runtime missing: {', '.join(missing)}")
    fast = runtime["confidence_gated_fast_adapter"]
    if not isinstance(fast, dict) or "enabled" not in fast:
        raise ValueError("Gate16 confidence-gated fast-adapter contract is incomplete")

    integration = manifest.get("racing_integration")
    if not isinstance(integration, dict):
        raise ValueError("Gate16 racing_integration is required")
    if integration.get("fallback_owner_request") != "gate16_climb_fallback":
        raise ValueError("Gate16 fallback owner request must be gate16_climb_fallback")
    if integration.get("competition_fast_adapter_enabled") is not False:
        raise ValueError("competition Gate16 manifest must default the fast adapter off")


def verify_gate16_assets() -> None:
    directory = REPO_ROOT / "models/deployed/gate16"
    manifest = json.loads((directory / "climb_policy_manifest.json").read_text())
    try:
        validate_gate16_manifest(manifest)
    except ValueError as error:
        raise SystemExit(f"Gate16 deployment contract mismatch: {error}") from error
    expected = {
        manifest["base_onnx"]: manifest["base_sha256"],
        manifest["residual_onnx"]: manifest["residual_sha256"],
    }
    for name, digest in expected.items():
        actual = hashlib.sha256((directory / name).read_bytes()).hexdigest()
        if actual != digest:
            raise SystemExit(f"Gate16 asset hash mismatch for {name}: {actual} != {digest}")
    profile = manifest.get("front_tuck_command_profile")
    if profile:
        name = profile["file"]
        if Path(name).name != name:
            raise SystemExit(f"Gate16 profile must be bundle-local: {name}")
        actual = hashlib.sha256((directory / name).read_bytes()).hexdigest()
        if actual != profile["sha256"]:
            raise SystemExit(
                f"Gate16 profile hash mismatch for {name}: {actual} != {profile['sha256']}"
            )


def verify_stairs_stable_assets() -> None:
    directory = REPO_ROOT / "models/deployed/stairs_stable"
    manifest = json.loads((directory / "policy_manifest.json").read_text())
    model = directory / manifest["onnx"]
    actual = hashlib.sha256(model.read_bytes()).hexdigest()
    if actual != manifest["sha256"]:
        raise SystemExit(
            f"stairs_stable asset hash mismatch for {model.name}: {actual} != {manifest['sha256']}"
        )
    if manifest["observation_dim"] != 57 or manifest["action_dim"] != 16:
        raise SystemExit("stairs_stable manifest must retain the official 57D->16D contract")


#: Repository-owned integration files and essential frozen policy assets. Binary files are
#: copied byte-for-byte and verified by the Gate16 runner before use.
FILES = {
    REPO_ROOT / "robot/integration/ros_cmd_interface.hpp": SDK
    / "interface/user_command/ros_cmd_interface.hpp",
    REPO_ROOT / "robot/integration/joint_command_owner.hpp": SDK
    / "interface/user_command/joint_command_owner.hpp",
    REPO_ROOT / "robot/integration/gate16_perception_buffer.hpp": SDK
    / "run_policy/gate16_perception_buffer.hpp",
    REPO_ROOT / "robot/integration/gate16_skill_gate.hpp": SDK / "run_policy/gate16_skill_gate.hpp",
    REPO_ROOT / "robot/integration/gate16_policy_symmetry.hpp": SDK
    / "run_policy/gate16_policy_symmetry.hpp",
    REPO_ROOT / "robot/integration/gate16_policy_runner.hpp": SDK / "run_policy/gate16_policy_runner.hpp",
    REPO_ROOT / "models/deployed/gate16/policy.onnx": SDK / "models/deployed/gate16/policy.onnx",
    REPO_ROOT / "models/deployed/gate16/climb_residual.onnx": SDK / "models/deployed/gate16/climb_residual.onnx",
    REPO_ROOT / "models/deployed/gate16/climb_policy_manifest.json": SDK
    / "models/deployed/gate16/climb_policy_manifest.json",
    REPO_ROOT / "models/deployed/gate16/front_tuck_command_profiles.json": SDK
    / "models/deployed/gate16/front_tuck_command_profiles.json",
    REPO_ROOT / "models/deployed/stairs_stable/policy.onnx": SDK / "models/deployed/stairs_stable/policy.onnx",
    REPO_ROOT / "models/deployed/stairs_stable/policy_manifest.json": SDK
    / "models/deployed/stairs_stable/policy_manifest.json",
}


@dataclass
class Edit:
    """A single anchored insertion or replacement."""

    path: Path
    anchor: str
    addition: str
    marker: str
    mode: str = "after"  # "after" | "replace"

    def apply(self, root: Path) -> bool:
        target = root / self.path
        text = target.read_text()
        if self.marker in text:
            return False
        if self.anchor not in text:
            raise SystemExit(
                f"Anchor not found in {self.path}:\n  {self.anchor!r}\n"
                "Upstream has changed; update scripts/patch_upstream.py."
            )
        if self.mode == "replace":
            text = text.replace(self.anchor, self.addition, 1)
        else:
            text = text.replace(self.anchor, self.anchor + self.addition, 1)
        target.write_text(text)
        return True

    def is_applied(self, root: Path) -> bool:
        return self.marker in (root / self.path).read_text()


EDITS = [
    # 1. Extend the input-device enum.
    Edit(
        path=SDK / "include/types/custom_types.h",
        anchor="        kGamepad,\n",
        addition="        kRosTopic,  // added by goai26-s10-racing\n",
        marker="kRosTopic",
    ),
    # 2. Instantiate the bridge when that mode is selected.
    Edit(
        path=SDK / "state_machine/quadruped_wheel/qw_state_machine.hpp",
        anchor='#include "keyboard_interface.hpp"\n',
        addition='#include "ros_cmd_interface.hpp"\n',
        marker="ros_cmd_interface.hpp",
    ),
    Edit(
        path=SDK / "state_machine/quadruped_wheel/qw_state_machine.hpp",
        anchor="""        }else if(remote_cmd_type_ == RemoteCommandType::kGamepad){
            auto gp_ptr = std::make_shared<GamepadInterface>(robot_name_);
            udp_server_ = std::make_shared<UdpServer>(gp_ptr.get());
            uc_ptr_ = gp_ptr;
        }""",
        addition="""else if(remote_cmd_type_ == RemoteCommandType::kRosTopic){
            uc_ptr_ = std::make_shared<RosCmdInterface>(robot_name_);
        }""",
        marker="RosCmdInterface",
    ),
    # 3. Default the deploy binary to the autonomous source.
    Edit(
        path=SDK / "main.cpp",
        anchor="RemoteCommandType::kKeyBoard",
        addition="RemoteCommandType::kRosTopic",
        marker="kRosTopic",
        mode="replace",
    ),
    # 4. The bridge uses geometry_msgs/Twist and std_msgs/UInt8; declare both.
    Edit(
        path=SDK / "CMakeLists.txt",
        anchor="find_package(drdds REQUIRED)\n",
        addition="find_package(geometry_msgs REQUIRED)\nfind_package(std_msgs REQUIRED)\n",
        marker="find_package(geometry_msgs",
    ),
    Edit(
        path=SDK / "CMakeLists.txt",
        anchor="""ament_target_dependencies(rl_deploy
  rclcpp
  drdds
""",
        addition="  geometry_msgs\n  std_msgs\n",
        marker="  geometry_msgs\n",
    ),
    Edit(
        path=SDK / "package.xml",
        anchor="<depend>drdds</depend>",
        addition="\n  <depend>geometry_msgs</depend>\n  <depend>std_msgs</depend>",
        marker="<depend>geometry_msgs</depend>",
    ),
    # 5. Route the one call that produces a joint command through the ownership gate.
    #
    # This is the edit that makes single ownership of /JOINTS_CMD a fact rather than a
    # convention. The topic is created and written by DdsInterface inside this process; an
    # external node cannot take it away, and two publishers on one topic is a silent merge
    # rather than an error. RLControlState::PolicyRunner is the only place in the running
    # stack that turns a policy action into a joint command, so gating it gates everything.
    #
    # The reset is not incidental. The policy's observation contains its own previous
    # action; after a climb policy has been driving, the last action it produced describes a
    # robot that no longer exists, and feeding it back is how a handover becomes a lurch.
    Edit(
        path=SDK / "state_machine/quadruped_wheel/rl_control_state.hpp",
        anchor='#include "basic_function.hpp"\n',
        addition='#include "joint_command_owner.hpp"  // added by goai26-s10-racing\n',
        marker="joint_command_owner.hpp",
    ),
    Edit(
        path=SDK / "state_machine/quadruped_wheel/rl_control_state.hpp",
        anchor="""                    MatXf res = ra.ConvertToMat();

                    ri_ptr_->SetJointCommand(res);""",
        addition="""                    MatXf res = ra.ConvertToMat();

                    // added by goai26-s10-racing: one owner of the actuators, always
                    bool reset_official = false;
                    MatXf gated = s10::JointCommandOwner::Instance().Arbitrate(
                            res, rbs_[getrbsReadIndex()].joint_pos, &reset_official);
                    if (reset_official) policy_ptr_->OnEnter();
                    ri_ptr_->SetJointCommand(gated);""",
        marker="owner.Arbitrate",
        mode="replace",
    ),
    # 6. Add the stable 174D Gate16 runner beside, never in place of, the official 57D actor.
    Edit(
        path=SDK / "state_machine/quadruped_wheel/rl_control_state.hpp",
        anchor='#include "joint_command_owner.hpp"  // added by goai26-s10-racing\n',
        addition='#include "gate16_policy_runner.hpp"  // added by goai26-s10-racing\n',
        marker="gate16_policy_runner.hpp",
    ),
    Edit(
        path=SDK / "state_machine/quadruped_wheel/rl_control_state.hpp",
        anchor="        std::shared_ptr<S10PolicyRunner> s10_policy_;\n",
        addition=(
            "        std::shared_ptr<Gate16PolicyRunner> gate16_policy_;\n"
            "        bool gate16_running_ = false;\n"
        ),
        marker="gate16_running_",
    ),
    Edit(
        path=SDK / "state_machine/quadruped_wheel/rl_control_state.hpp",
        anchor="        bool gate16_running_ = false;\n",
        addition=(
            "        std::shared_ptr<S10PolicyRunner> stairs_stable_policy_;\n"
            "        bool stairs_stable_running_ = false;\n"
            "        int stairs_stable_blend_step_ = 0;\n"
            "        static constexpr int kStairsStableBlendSteps = 20;  // 0.4 s at 50 Hz\n"
        ),
        marker="stairs_stable_running_",
    ),
    Edit(
        path=SDK / "state_machine/quadruped_wheel/rl_control_state.hpp",
        anchor=(
            "                s10_policy_ = std::make_shared<S10PolicyRunner>"
            '("s10_policy", model_path.string());\n'
        ),
        addition="""                auto gate16_path = fs::canonical(
                    base / ".." / ".." / "policy" / "gate16" / "policy.onnx");
                gate16_policy_ = std::make_shared<Gate16PolicyRunner>(
                    "gate16_stable", gate16_path.string());
""",
        marker="gate16_stable",
    ),
    Edit(
        path=SDK / "state_machine/quadruped_wheel/rl_control_state.hpp",
        anchor=(
            "                gate16_policy_ = std::make_shared<Gate16PolicyRunner>(\n"
            '                    "gate16_stable", gate16_path.string());\n'
        ),
        addition="""                auto stairs_stable_path = fs::canonical(
                    base / ".." / ".." / "policy" / "stairs_stable" / "policy.onnx");
                stairs_stable_policy_ = std::make_shared<S10PolicyRunner>(
                    "stairs_stable", stairs_stable_path.string());
""",
        marker='"stairs_stable", stairs_stable_path.string()',
    ),
    Edit(
        path=SDK / "state_machine/quadruped_wheel/rl_control_state.hpp",
        anchor="""                    MatXf res = ra.ConvertToMat();

                    // added by goai26-s10-racing: one owner of the actuators, always
                    bool reset_official = false;
                    MatXf gated = s10::JointCommandOwner::Instance().Arbitrate(
                            res, rbs_[getrbsReadIndex()].joint_pos, &reset_official);
                    if (reset_official) policy_ptr_->OnEnter();
                    ri_ptr_->SetJointCommand(gated);""",
        addition="""                    MatXf res = ra.ConvertToMat();

                    // Shadow-evaluate Gate16 through APPROACH/ALIGN to preserve the action
                    // history expected by the reference full-stack runner. The official
                    // matrix remains the sole actuator command until the armed handoff.
                    auto& owner = s10::JointCommandOwner::Instance();
                    MatXf gate16_command;
                    const MatXf* gate16_ptr = nullptr;
                    if (owner.owner() == s10::JointOwner::kGate16 ||
                        owner.gate16_armed() || owner.gate16_shadow()) {
                        if (!gate16_running_) {
                            gate16_policy_->OnEnter();
                            gate16_running_ = true;
                        }
                        gate16_policy_->SetActuatorOwnership(
                            owner.owner() == s10::JointOwner::kGate16);
                        gate16_policy_->SetForceFallback(owner.gate16_fallback());
                        gate16_policy_->SetClimbArmed(owner.gate16_armed());
                        gate16_command = gate16_policy_->getRobotAction(
                            rbs_[getrbsReadIndex()], *(uc_ptr_->GetUserCommand())).ConvertToMat();
                        gate16_ptr = &gate16_command;
                    } else {
                        gate16_running_ = false;
                    }

                    bool reset_official = false;
                    MatXf gated = owner.Arbitrate(
                            res, gate16_ptr, rbs_[getrbsReadIndex()].joint_pos,
                            &reset_official);
                    if (reset_official) policy_ptr_->OnEnter();
                    ri_ptr_->SetJointCommand(gated);""",
        marker="gate16_ptr",
        mode="replace",
    ),
    Edit(
        path=SDK / "state_machine/quadruped_wheel/rl_control_state.hpp",
        anchor="""                    } else {
                        gate16_running_ = false;
                    }

                    bool reset_official = false;
""",
        addition="""                    } else {
                        gate16_running_ = false;
                    }

                    MatXf stairs_stable_command;
                    const MatXf* stairs_stable_ptr = nullptr;
                    if (owner.owner() == s10::JointOwner::kStairsStable ||
                        owner.requested_owner() == s10::JointOwner::kStairsStable) {
                        if (!stairs_stable_running_) {
                            // Both policies use the official 57D/16D S10 contract. Seed the
                            // stair actor with the action currently driving the robot, then
                            // retain the actual blended action as its recurrent history.
                            stairs_stable_policy_->OnEnter(s10_policy_->GetLastAction());
                            stairs_stable_running_ = true;
                            stairs_stable_blend_step_ = 0;
                        }
                        const float blend_phase = std::min(
                            1.0f, static_cast<float>(stairs_stable_blend_step_) /
                                      static_cast<float>(kStairsStableBlendSteps));
                        const float blend_alpha =
                            blend_phase * blend_phase * (3.0f - 2.0f * blend_phase);
                        UserCommand stairs_stable_user_command = *(uc_ptr_->GetUserCommand());
                        stairs_stable_command = stairs_stable_policy_->getRobotActionBlended(
                            rbs_[getrbsReadIndex()], stairs_stable_user_command,
                            &s10_policy_->GetLastAction(), blend_alpha).ConvertToMat();
                        if (stairs_stable_blend_step_ < kStairsStableBlendSteps) {
                            ++stairs_stable_blend_step_;
                        }
                        stairs_stable_ptr = &stairs_stable_command;
                    } else {
                        stairs_stable_running_ = false;
                        stairs_stable_blend_step_ = 0;
                    }

                    bool reset_official = false;
""",
        marker="stairs_stable_ptr",
        mode="replace",
    ),
    Edit(
        path=SDK / "state_machine/quadruped_wheel/rl_control_state.hpp",
        anchor="""                    MatXf gated = owner.Arbitrate(
                            res, gate16_ptr, rbs_[getrbsReadIndex()].joint_pos,
                            &reset_official);""",
        addition="""                    MatXf gated = owner.Arbitrate(
                            res, gate16_ptr, stairs_stable_ptr,
                            rbs_[getrbsReadIndex()].joint_pos, &reset_official);""",
        marker="gate16_ptr, stairs_stable_ptr",
        mode="replace",
    ),
    Edit(
        path=SDK / "state_machine/quadruped_wheel/rl_control_state.hpp",
        anchor="                        gate16_policy_->SetClimbArmed(owner.gate16_armed());\n",
        addition="""                        gate16_policy_->SetActuatorOwnership(
                            owner.owner() == s10::JointOwner::kGate16);
                        gate16_policy_->SetClimbArmed(owner.gate16_armed());
""",
        marker="SetActuatorOwnership",
        mode="replace",
    ),
    # Upgrade already-patched workspaces so the router's attempt-level fallback choice is
    # not reclassified by the runner's independent height-map confidence check.
    Edit(
        path=SDK / "state_machine/quadruped_wheel/rl_control_state.hpp",
        anchor=(
            "                        gate16_policy_->SetActuatorOwnership(\n"
            "                            owner.owner() == s10::JointOwner::kGate16);\n"
            "                        gate16_policy_->SetClimbArmed(owner.gate16_armed());\n"
        ),
        addition=(
            "                        gate16_policy_->SetActuatorOwnership(\n"
            "                            owner.owner() == s10::JointOwner::kGate16);\n"
            "                        gate16_policy_->SetForceFallback(\n"
            "                            owner.gate16_fallback());\n"
            "                        gate16_policy_->SetClimbArmed(owner.gate16_armed());\n"
        ),
        marker="owner.gate16_fallback()",
        mode="replace",
    ),
    # The stair actor shares the official 57D observation/action contract. Preserve the
    # command actually sent during a smooth policy transition as its last_action history;
    # resetting that slice to zero at a moving handoff is outside the training contract.
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor='#include "policy_runner_base.hpp"\n',
        addition="#include <algorithm>\n",
        marker="#include <algorithm>",
    ),
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor="""    void OnEnter() {
        run_cnt_ = 0;
        cmd_vel_input_.setZero();
        last_action_eigen.setZero(action_dim);
        tmp_action_eigen.setZero(action_dim);
        motor_p_eigen.setZero(12);
        motor_v_eigen.setZero(motor_num);
    }
""",
        addition="""    void OnEnter() {
        OnEnter(VecXf());
    }

    void OnEnter(const VecXf& action_history_seed) {
        run_cnt_ = 0;
        cmd_vel_input_.setZero();
        if (action_history_seed.size() == action_dim && action_history_seed.allFinite()) {
            last_action_eigen = action_history_seed;
        } else {
            last_action_eigen.setZero(action_dim);
        }
        tmp_action_eigen.setZero(action_dim);
        motor_p_eigen.setZero(12);
        motor_v_eigen.setZero(motor_num);
    }

    const VecXf& GetLastAction() const {
        return last_action_eigen;
    }
""",
        marker="action_history_seed",
        mode="replace",
    ),
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor="""    RobotAction getRobotAction(const RobotBasicState &ro, const UserCommand &uc) {

        Vec3f base_omgea = ro.base_omega * omega_scale_;
""",
        addition="""    RobotAction getRobotAction(const RobotBasicState &ro, const UserCommand &uc) {
        return getRobotActionBlended(ro, uc, nullptr, 1.0f);
    }

    RobotAction getRobotActionBlended(const RobotBasicState &ro, const UserCommand &uc,
                                      const VecXf* blend_from, float blend_alpha) {

        Vec3f base_omgea = ro.base_omega * omega_scale_;
""",
        marker="getRobotActionBlended",
        mode="replace",
    ),
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor="""        current_action_eigen = Onnx_infer(current_observation_);
        last_action_eigen = current_action_eigen;
""",
        addition="""        current_action_eigen = Onnx_infer(current_observation_);
        if (blend_from != nullptr && blend_from->size() == action_dim && blend_from->allFinite()) {
            const float alpha = std::isfinite(blend_alpha)
                                    ? std::clamp(blend_alpha, 0.0f, 1.0f)
                                    : 1.0f;
            current_action_eigen =
                (1.0f - alpha) * (*blend_from) + alpha * current_action_eigen;
        }
        last_action_eigen = current_action_eigen;
""",
        marker="std::clamp(blend_alpha",
        mode="replace",
    ),
    # Upgrade workspaces patched by the earlier owner-only runner block. The main edit is
    # intentionally idempotent via ``gate16_ptr`` and therefore cannot rewrite itself.
    Edit(
        path=SDK / "state_machine/quadruped_wheel/rl_control_state.hpp",
        anchor="if (owner.owner() == s10::JointOwner::kGate16) {",
        addition=(
            "if (owner.owner() == s10::JointOwner::kGate16 ||\n"
            "                        owner.gate16_armed() || owner.gate16_shadow()) {"
        ),
        marker="owner.gate16_shadow()",
        mode="replace",
    ),
    Edit(
        path=SDK / "state_machine/quadruped_wheel/rl_control_state.hpp",
        anchor="""            run_policy_thread_ = std::thread(std::bind(&RLControlState::PolicyRunner, this));
            policy_ptr_->OnEnter();""",
        addition="""            run_policy_thread_ = std::thread(std::bind(&RLControlState::PolicyRunner, this));
            s10::JointCommandOwner::Instance().Start();  // added by goai26-s10-racing
            policy_ptr_->OnEnter();""",
        marker="JointCommandOwner::Instance().Start()",
        mode="replace",
    ),
    Edit(
        path=SDK / "state_machine/quadruped_wheel/rl_control_state.hpp",
        anchor="""        virtual void OnExit() {
            start_flag_ = false;""",
        addition="""        virtual void OnExit() {
            s10::JointCommandOwner::Instance().Stop();  // added by goai26-s10-racing
            start_flag_ = false;""",
        marker="JointCommandOwner::Instance().Stop()",
        mode="replace",
    ),
]


def install_files(root: Path) -> bool:
    changed = False
    for source, destination in FILES.items():
        target = root / destination
        content = source.read_bytes()
        if target.is_file() and target.read_bytes() == content:
            continue
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(content)
        changed = True
    return changed


def resolve_upstream(path: Path) -> Path:
    if not (path / SDK / "main.cpp").is_file():
        raise SystemExit(
            f"Not a contest SDK checkout: {path}\nRun scripts/setup_upstream.sh first."
        )
    return path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--upstream", type=Path, default=DEFAULT_UPSTREAM)
    parser.add_argument("--check", action="store_true", help="Report status without writing")
    parser.add_argument("--revert", action="store_true", help="Restore the pristine checkout")
    args = parser.parse_args()

    root = resolve_upstream(args.upstream.resolve())
    verify_gate16_assets()
    verify_stairs_stable_assets()

    if args.revert:
        subprocess.run(
            ["git", "-c", f"safe.directory={root}", "-C", str(root), "checkout", "--", "."],
            check=True,
        )
        for destination in FILES.values():
            (root / destination).unlink(missing_ok=True)
        print(f"Reverted {root} to a pristine checkout")
        return 0

    if args.check:
        missing = [d for d in FILES.values() if not (root / d).is_file()]
        outdated = [
            destination
            for source, destination in FILES.items()
            if (root / destination).is_file()
            and (root / destination).read_bytes() != source.read_bytes()
        ]
        pending = [e.path for e in EDITS if not e.is_applied(root)]
        if not missing and not outdated and not pending:
            print(f"{root} is patched")
            return 0
        for destination in missing:
            print(f"Missing: {destination}", file=sys.stderr)
        for destination in outdated:
            print(f"Outdated: {destination}", file=sys.stderr)
        for path in dict.fromkeys(pending):
            print(f"Unpatched: {path}", file=sys.stderr)
        return 1

    changed = install_files(root)
    for edit in EDITS:
        changed |= edit.apply(root)

    print(f"{root} patched" if changed else f"{root} already patched")
    print("Rebuild with: scripts/build.sh")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
