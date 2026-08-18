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

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_UPSTREAM = REPO_ROOT / "upstream/goai_embodied_future_material"

SDK = Path("src/S10_sdk_deploy")


def verify_gate16_assets() -> None:
    directory = REPO_ROOT / "policy/gate16"
    manifest = json.loads((directory / "climb_policy_manifest.json").read_text())
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

#: Repository-owned integration files and essential frozen policy assets. Binary files are
#: copied byte-for-byte and verified by the Gate16 runner before use.
FILES = {
    REPO_ROOT / "integration/ros_cmd_interface.hpp": SDK
    / "interface/user_command/ros_cmd_interface.hpp",
    REPO_ROOT / "integration/joint_command_owner.hpp": SDK
    / "interface/user_command/joint_command_owner.hpp",
    REPO_ROOT / "integration/gate16_perception_buffer.hpp": SDK
    / "run_policy/gate16_perception_buffer.hpp",
    REPO_ROOT / "integration/gate16_skill_gate.hpp": SDK
    / "run_policy/gate16_skill_gate.hpp",
    REPO_ROOT / "integration/gate16_policy_symmetry.hpp": SDK
    / "run_policy/gate16_policy_symmetry.hpp",
    REPO_ROOT / "integration/gate16_policy_runner.hpp": SDK
    / "run_policy/gate16_policy_runner.hpp",
    REPO_ROOT / "policy/gate16/policy.onnx": SDK / "policy/gate16/policy.onnx",
    REPO_ROOT / "policy/gate16/climb_residual.onnx": SDK
    / "policy/gate16/climb_residual.onnx",
    REPO_ROOT / "policy/gate16/climb_policy_manifest.json": SDK
    / "policy/gate16/climb_policy_manifest.json",
    REPO_ROOT / "policy/gate16/front_tuck_command_profiles.json": SDK
    / "policy/gate16/front_tuck_command_profiles.json",
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
        anchor=(
            "                s10_policy_ = std::make_shared<S10PolicyRunner>"
            "(\"s10_policy\", model_path.string());\n"
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
        anchor="""                    MatXf res = ra.ConvertToMat();

                    // added by goai26-s10-racing: one owner of the actuators, always
                    bool reset_official = false;
                    MatXf gated = s10::JointCommandOwner::Instance().Arbitrate(
                            res, rbs_[getrbsReadIndex()].joint_pos, &reset_official);
                    if (reset_official) policy_ptr_->OnEnter();
                    ri_ptr_->SetJointCommand(gated);""",
        addition="""                    MatXf res = ra.ConvertToMat();

                    // The stable Gate16 actor is evaluated only after the owner gate has
                    // completed its safe hold. Its history therefore starts at the actual
                    // control handoff, not when the router first asks for it.
                    auto& owner = s10::JointCommandOwner::Instance();
                    MatXf gate16_command;
                    const MatXf* gate16_ptr = nullptr;
                    if (owner.owner() == s10::JointOwner::kGate16) {
                        if (!gate16_running_) {
                            gate16_policy_->OnEnter();
                            gate16_running_ = true;
                        }
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

    if args.revert:
        subprocess.run(["git", "-C", str(root), "checkout", "--", "."], check=True)
        for destination in FILES.values():
            (root / destination).unlink(missing_ok=True)
        print(f"Reverted {root} to a pristine checkout")
        return 0

    if args.check:
        missing = [d for d in FILES.values() if not (root / d).is_file()]
        pending = [e.path for e in EDITS if not e.is_applied(root)]
        if not missing and not pending:
            print(f"{root} is patched")
            return 0
        for destination in missing:
            print(f"Missing: {destination}", file=sys.stderr)
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
