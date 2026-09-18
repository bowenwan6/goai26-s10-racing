#!/usr/bin/env python3
"""Wire our ROS command bridge into the contest SDK.

The SDK selects its velocity-command source from a fixed enum of input devices. Autonomy
needs one more option -- a ROS topic -- which is a handful of lines spread across four
upstream files. Applying them from a script keeps upstream a pristine checkout that can be
re-cloned or updated at any time, and keeps our actual contribution in one reviewable
header (integration/ros_cmd_interface.hpp).

Every edit is idempotent: running this twice is a no-op, and --check reports whether the
checkout is patched without modifying it.

Usage:
    scripts/patch_upstream.py            # apply
    scripts/patch_upstream.py --check    # report status, exit 1 if unpatched
    scripts/patch_upstream.py --revert   # undo via git checkout, then remove our header
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
BRIDGE_HEADER = REPO_ROOT / "integration/ros_cmd_interface.hpp"
OBSTACLE_HEADER = REPO_ROOT / "integration/obstacle_state.hpp"
DEFAULT_UPSTREAM = REPO_ROOT / "upstream/goai_embodied_future_material"

SDK = Path("src/S10_sdk_deploy")


@dataclass
class Edit:
    """A single anchored insertion or replacement."""

    path: Path
    anchor: str
    addition: str
    marker: str
    mode: str = "after"  # "after" | "replace"
    superseded_marker: str | None = None

    def apply(self, root: Path) -> bool:
        target = root / self.path
        text = target.read_text()
        if self.marker in text or (self.superseded_marker and self.superseded_marker in text):
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
        text = (root / self.path).read_text()
        return self.marker in text or bool(
            self.superseded_marker and self.superseded_marker in text
        )


EDITS = [
    # 0. Add the direct-control obstacle state.
    Edit(
        path=SDK / "include/types/custom_types.h",
        anchor="        RLControlMode   = 6,\n",
        addition="        ObstacleMode    = 7,  // added by goai26-s10-racing\n",
        marker="ObstacleMode",
    ),
    Edit(
        path=SDK / "include/types/custom_types.h",
        anchor="        kRLControl    = 6,\n",
        addition="        kObstacle     = 7,  // added by goai26-s10-racing\n",
        marker="kObstacle",
    ),
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
        anchor='#include "quadruped_wheel/rl_control_state.hpp"\n',
        addition='#include "quadruped_wheel/obstacle_state.hpp"\n',
        marker="obstacle_state.hpp",
    ),
    Edit(
        path=SDK / "state_machine/quadruped_wheel/qw_state_machine.hpp",
        anchor="    std::shared_ptr<StateBase> rl_controller_;\n",
        addition="    std::shared_ptr<StateBase> obstacle_controller_;\n",
        marker="obstacle_controller_",
    ),
    Edit(
        path=SDK / "state_machine/quadruped_wheel/qw_state_machine.hpp",
        anchor=(
            '        rl_controller_ = std::make_shared<RLControlState>'
            '(robot_name_, "rl_control", data_ptr);\n'
        ),
        addition=(
            '        obstacle_controller_ = std::make_shared<ObstacleState>'
            '(robot_name_, "obstacle", data_ptr);\n'
        ),
        marker="std::make_shared<ObstacleState>",
    ),
    Edit(
        path=SDK / "state_machine/quadruped_wheel/qw_state_machine.hpp",
        anchor=(
            "            case StateName::kRLControl:{\n"
            "                return rl_controller_;\n"
            "            }\n"
        ),
        addition=(
            "            case StateName::kObstacle:{\n"
            "                return obstacle_controller_;\n"
            "            }\n"
        ),
        marker="case StateName::kObstacle",
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
    # 3. Default to autonomy, while allowing the runner to select the SDK keyboard.
    Edit(
        path=SDK / "main.cpp",
        anchor='#include "quadruped_wheel/qw_state_machine.hpp"\n',
        addition="#include <cstdlib>\n",
        marker="#include <cstdlib>",
    ),
    Edit(
        path=SDK / "main.cpp",
        anchor=(
            "    //KeyBoard control\n"
            "    std::shared_ptr<StateMachineBase> fsm = "
            "std::make_shared<qw::QwStateMachine>(RobotName::S10, "
            "RemoteCommandType::kKeyBoard);"
        ),
        addition=(
            "    const auto command_source =\n"
            '        std::getenv("S10_MANUAL") ? RemoteCommandType::kKeyBoard\n'
            "                                  : RemoteCommandType::kRosTopic;\n"
            "    std::shared_ptr<StateMachineBase> fsm =\n"
            "        std::make_shared<qw::QwStateMachine>(RobotName::S10, command_source);"
        ),
        marker="S10_MANUAL",
        mode="replace",
    ),
    # 4. The RL controller owns a worker thread that must be joined on shutdown.
    Edit(
        path=SDK / "state_machine/quadruped_wheel/qw_state_machine.hpp",
        anchor="    void Stop(){\n        sc_ptr_->Stop();",
        addition=(
            "    void Stop(){\n"
            "        // Join controller-owned threads before members are destroyed.\n"
            "        current_controller_->OnExit();\n"
            "        sc_ptr_->Stop();"
        ),
        marker="Join controller-owned threads",
        mode="replace",
    ),
    # 5. Let the state machine, not unsynchronised feedback, gate keyboard velocity.
    Edit(
        path=SDK / "interface/user_command/keyboard_interface.hpp",
        anchor=(
            "            if (msfb_->GetCurrentState() == RobotMotionState::RLControlMode) {\n"
            "                compute_velocity_from_held_keys(fwd, side, yaw);\n"
            "            }"
        ),
        addition=(
            "            // State controllers ignore velocity until RL is active.\n"
            "            compute_velocity_from_held_keys(fwd, side, yaw);"
        ),
        marker="State controllers ignore velocity",
        mode="replace",
    ),
    Edit(
        path=SDK / "interface/user_command/keyboard_interface.hpp",
        anchor="else if (k == 'c' && msfb_->GetCurrentState() == RobotMotionState::StandingUp) {",
        addition="else if (k == 'c') {",
        marker="else if (k == 'c') {",
        mode="replace",
    ),
    Edit(
        path=SDK / "interface/user_command/keyboard_interface.hpp",
        anchor='            std::cout << "[MODE] RL Control\\n";\n',
        addition='            std::cout << "[MODE] RL Control queued\\n";\n',
        marker="RL Control queued",
        mode="replace",
    ),
    Edit(
        path=SDK / "interface/user_command/keyboard_interface.hpp",
        anchor=(
            "        else if (k == 'c') {\n"
            "            usr_cmd_->target_mode = uint8_t(RobotMotionState::RLControlMode);\n"
            "            std::cout << \"[MODE] RL Control queued\\n\";\n"
            "        }\n"
        ),
        addition=(
            "        else if (k == 'v' && msfb_->GetCurrentState() == RobotMotionState::RLControlMode) {\n"
            "            setenv(\"S10_OBSTACLE_MANUAL_PHASE\", \"0\", 1);\n"
            "            usr_cmd_->target_mode = uint8_t(RobotMotionState::ObstacleMode);\n"
            "            std::cout << \"[OBSTACLE] V: rear wheels approach face\\n\";\n"
            "        }\n"
            "        else if (k == 'b' && msfb_->GetCurrentState() == RobotMotionState::ObstacleMode) {\n"
            "            setenv(\"S10_OBSTACLE_MANUAL_PHASE\", \"1\", 1);\n"
            "            std::cout << \"[OBSTACLE] B: brace and lift body\\n\";\n"
            "        }\n"
            "        else if (k == 'n' && msfb_->GetCurrentState() == RobotMotionState::ObstacleMode) {\n"
            "            const char* value = std::getenv(\"S10_OBSTACLE_MANUAL_PHASE\");\n"
            "            const int phase = std::clamp(value ? std::atoi(value) + 1 : 2, 2, 5);\n"
            "            const std::string text = std::to_string(phase);\n"
            "            setenv(\"S10_OBSTACLE_MANUAL_PHASE\", text.c_str(), 1);\n"
            "            static const char* actions[] = {\"lift left rear\", \"land left rear\",\n"
            "                                            \"lift right rear\", \"land right rear / recover\"};\n"
            "            std::cout << \"[OBSTACLE] N: \" << actions[phase - 2] << \"\\n\";\n"
            "        }\n"
        ),
        marker="rear wheels approach face",
    ),
    Edit(
        path=SDK / "interface/user_command/keyboard_interface.hpp",
        anchor=(
            "        else if (k == 'b' && msfb_->GetCurrentState() == RobotMotionState::ObstacleMode) {\n"
            "            setenv(\"S10_OBSTACLE_MANUAL_PHASE\", \"1\", 1);\n"
            "            std::cout << \"[OBSTACLE] B: rear wheels climb face\\n\";\n"
            "        }\n"
            "        else if (k == 'n' && msfb_->GetCurrentState() == RobotMotionState::ObstacleMode) {\n"
            "            setenv(\"S10_OBSTACLE_MANUAL_PHASE\", \"2\", 1);\n"
            "            std::cout << \"[OBSTACLE] N: recover to RL\\n\";\n"
            "        }\n"
        ),
        addition=(
            "        else if (k == 'b' && msfb_->GetCurrentState() == RobotMotionState::ObstacleMode) {\n"
            "            setenv(\"S10_OBSTACLE_MANUAL_PHASE\", \"1\", 1);\n"
            "            std::cout << \"[OBSTACLE] B: brace and lift body\\n\";\n"
            "        }\n"
            "        else if (k == 'n' && msfb_->GetCurrentState() == RobotMotionState::ObstacleMode) {\n"
            "            const char* value = std::getenv(\"S10_OBSTACLE_MANUAL_PHASE\");\n"
            "            const int phase = std::clamp(value ? std::atoi(value) + 1 : 2, 2, 5);\n"
            "            const std::string text = std::to_string(phase);\n"
            "            setenv(\"S10_OBSTACLE_MANUAL_PHASE\", text.c_str(), 1);\n"
            "            static const char* actions[] = {\"lift left rear\", \"land left rear\",\n"
            "                                            \"lift right rear\", \"land right rear / recover\"};\n"
            "            std::cout << \"[OBSTACLE] N: \" << actions[phase - 2] << \"\\n\";\n"
            "        }\n"
        ),
        marker="brace and lift body",
        mode="replace",
    ),
    Edit(
        path=SDK / "interface/user_command/keyboard_interface.hpp",
        anchor="if (k == 'r' || k == 'z' || k == 'c' || k == 'x')",
        addition="if (k == 'r' || k == 'z' || k == 'c' || k == 'x' || k == 'v' || k == 'b' || k == 'n')",
        marker="|| k == 'v' || k == 'b'",
        mode="replace",
        superseded_marker="void HandleKey",
    ),
    Edit(
        path=SDK / "interface/user_command/keyboard_interface.hpp",
        anchor="    void process_mode_command(char k)\n    {",
        addition=(
            "    float ReadObstacleParameter(const char* name, float fallback) {\n"
            "        const char* text = std::getenv(name);\n"
            "        if (!text) return fallback;\n"
            "        char* end = nullptr;\n"
            "        const float value = std::strtof(text, &end);\n"
            "        return end != text && std::isfinite(value) ? value : fallback;\n"
            "    }\n\n"
            "    void AdjustObstacleParameter(char key) {\n"
            "        static const char* names[] = {\n"
            "            \"S10_OBSTACLE_PHASE_TIME\", \"S10_OBSTACLE_TUCK_HIP\",\n"
            "            \"S10_OBSTACLE_TUCK_KNEE\", \"S10_OBSTACLE_WHEEL_SPEED\",\n"
            "            \"S10_OBSTACLE_FRONT_PRESS\"};\n"
            "        static const float defaults[] = {2.5f, 2.35f, 2.35f, 2.0f, 0.15f};\n"
            "        static const float steps[] = {0.05f, 0.05f, 0.1f, 1.0f, 0.05f};\n"
            "        static const float lows[] = {0.1f, 0.2f, 0.2f, 1.0f, 0.05f};\n"
            "        static const float highs[] = {3.0f, 2.4f, 2.7f, 30.0f, 0.8f};\n"
            "        const bool press = key == '[' || key == ']';\n"
            "        const int index = press ? 4 : (key - '1') / 2;\n"
            "        float value = ReadObstacleParameter(names[index], defaults[index]);\n"
            "        value += (press ? (key == '[' ? -1.0f : 1.0f)\n"
            "                        : ((key - '0') % 2 ? -1.0f : 1.0f)) * steps[index];\n"
            "        ClipNumber(value, lows[index], highs[index]);\n"
            "        const std::string text = std::to_string(value);\n"
            "        setenv(names[index], text.c_str(), 1);\n"
            "        std::cout << \"[TUNE] \" << names[index] << \"=\" << value << \"\\n\";\n"
            "    }\n\n"
            "    void process_mode_command(char k)\n"
            "    {\n"
            "        if ((k >= '1' && k <= '8') || k == '[' || k == ']') {\n"
            "            AdjustObstacleParameter(k);\n"
            "            return;\n"
            "        }"
        ),
        marker="[TUNE]",
        mode="replace",
    ),
    Edit(
        path=SDK / "interface/user_command/keyboard_interface.hpp",
        anchor='#include "custom_types.h"\n',
        addition='#include "rclcpp/rclcpp.hpp"\n#include "std_msgs/msg/empty.hpp"\n',
        marker="std_msgs/msg/empty.hpp",
    ),
    Edit(
        path=SDK / "interface/user_command/keyboard_interface.hpp",
        anchor="    mutable std::mutex keys_mutex_;\n",
        addition=(
            "    rclcpp::Node::SharedPtr reset_node_;\n"
            "    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr reset_pub_;\n"
            "    int reset_recovery_stage_ = 0;\n"
        ),
        marker="reset_pub_",
    ),
    Edit(
        path=SDK / "interface/user_command/keyboard_interface.hpp",
        anchor="    void process_mode_command(char k)\n    {\n",
        addition=(
            "        if (k == '0') {\n"
            "            usr_cmd_->target_mode = uint8_t(RobotMotionState::JointDamping);\n"
            "            reset_recovery_stage_ = 1;\n"
            "            reset_pub_->publish(std_msgs::msg::Empty());\n"
            "            std::cout << \"[RESET] Automatic recovery started\\n\";\n"
            "            return;\n"
            "        }\n"
        ),
        marker="[RESET] Automatic recovery started",
    ),
    Edit(
        path=SDK / "interface/user_command/keyboard_interface.hpp",
        anchor=(
            "        else if (k == 'z' && (msfb_->GetCurrentState() == "
            "RobotMotionState::WaitingForStand\n"
            "            || msfb_->GetCurrentState() == RobotMotionState::LieDown)) {"
        ),
        addition=(
            "        // Queue stand-up while reset damping completes.\n"
            "        else if (k == 'z' && (msfb_->GetCurrentState() == "
            "RobotMotionState::WaitingForStand\n"
            "            || msfb_->GetCurrentState() == RobotMotionState::LieDown\n"
            "            || msfb_->GetCurrentState() == RobotMotionState::JointDamping)) {"
        ),
        marker="Queue stand-up while reset damping completes",
        mode="replace",
    ),
    Edit(
        path=SDK / "interface/user_command/keyboard_interface.hpp",
        anchor="            // Remove keys that haven't been seen recently (released)\n",
        addition=(
            "            const auto state = msfb_->GetCurrentState();\n"
            "            if (reset_recovery_stage_ != 0 && usr_cmd_->safe_control_mode != 0) {\n"
            "                std::cout << \"[RESET] Clearing latched safety mode \"\n"
            "                          << int(usr_cmd_->safe_control_mode) << \"\\n\";\n"
            "                usr_cmd_->safe_control_mode = 0;\n"
            "            }\n"
            "            if (reset_recovery_stage_ == 1\n"
            "                && (state == RobotMotionState::JointDamping\n"
            "                    || state == RobotMotionState::WaitingForStand) {\n"
            "                    usr_cmd_->target_mode = uint8_t(RobotMotionState::StandingUp);\n"
            "                    reset_recovery_stage_ = 2;\n"
            "                    std::cout << \"[RESET] Stand-up queued\\n\";\n"
            "            } else if (reset_recovery_stage_ == 2\n"
            "                && state == RobotMotionState::StandingUp) {\n"
            "                usr_cmd_->target_mode = uint8_t(RobotMotionState::RLControlMode);\n"
            "                reset_recovery_stage_ = 0;\n"
            "                std::cout << \"[RESET] RL control queued\\n\";\n"
            "            }\n\n"
        ),
        marker="[RESET] Clearing latched safety mode",
    ),
    Edit(
        path=SDK / "interface/user_command/keyboard_interface.hpp",
        anchor=(
            "if (k == 'r' || k == 'z' || k == 'c' || k == 'x' || k == 'v' "
            "|| k == 'b' || k == 'n')"
        ),
        addition=(
            "if (k == 'r' || k == 'z' || k == 'c' || k == 'x' || k == 'v' "
            "|| k == 'b' || k == 'n' || (k >= '1' && k <= '8') || k == '[' || k == ']')"
        ),
        marker="|| (k >= '1'",
        mode="replace",
    ),
    Edit(
        path=SDK / "interface/user_command/keyboard_interface.hpp",
        anchor=(
            "if (k == 'r' || k == 'z' || k == 'c' || k == 'x' || k == 'v' "
            "|| k == 'b' || k == 'n' || (k >= '1' && k <= '8') || k == '[' || k == ']')"
        ),
        addition=(
            "if (k == '0' || k == 'r' || k == 'z' || k == 'c' || k == 'x' "
            "|| k == 'v' || k == 'b' || k == 'n' || (k >= '1' && k <= '8') || k == '[' || k == ']')"
        ),
        marker="if (k == '0' || k == 'r'",
        mode="replace",
    ),
    Edit(
        path=SDK / "interface/user_command/keyboard_interface.hpp",
        anchor='                  << "  Rotation:  Q (CCW)  E (CW)\\n"\n',
        addition='                  << "  Obstacle:  V (approach)  B (lift)  N x4 (transfer)\\n"\n',
        marker="V (approach)",
    ),
    Edit(
        path=SDK / "interface/user_command/keyboard_interface.hpp",
        anchor='                  << "  Obstacle:  V (approach)  B (climb)  N (recover)\\n"\n',
        addition='                  << "  Obstacle:  V (approach)  B (lift)  N x4 (transfer)\\n"\n',
        marker="N x4 (transfer)",
        mode="replace",
        superseded_marker="H (low/high speed toggle)",
    ),
    Edit(
        path=SDK / "interface/user_command/keyboard_interface.hpp",
        anchor='                  << "  Obstacle:  V (approach)  B (lift)  N x4 (transfer)\\n"\n',
        addition=(
            '                  << "  Tune:      1/2 time  3/4 hip  5/6 knee  7/8 wheel  [/] press\\n"\n'
        ),
        marker="1/2 time",
    ),
    Edit(
        path=SDK / "interface/user_command/keyboard_interface.hpp",
        anchor=(
            '                  << "  Tune:      1/2 time  3/4 hip  5/6 knee  7/8 wheel\\n"\n'
        ),
        addition='                  << "  Reset:     0 (full automatic recovery)\\n"\n',
        marker="full automatic recovery",
    ),
    Edit(
        path=SDK / "interface/user_command/keyboard_interface.hpp",
        anchor='            setenv("S10_OBSTACLE_MANUAL_PHASE", "0", 1);\n',
        addition='            setenv("S10_OBSTACLE_FRONT_EXTEND", "0", 1);\n',
        marker='S10_OBSTACLE_FRONT_EXTEND", "0"',
    ),
    Edit(
        path=SDK / "interface/user_command/keyboard_interface.hpp",
        anchor=(
            "        else if (k == 'b' && msfb_->GetCurrentState() == RobotMotionState::ObstacleMode) {\n"
            "            setenv(\"S10_OBSTACLE_MANUAL_PHASE\", \"1\", 1);\n"
            "            std::cout << \"[OBSTACLE] B: brace and lift body\\n\";\n"
            "        }\n"
        ),
        addition=(
            "        else if (k == 'b' && msfb_->GetCurrentState() == RobotMotionState::ObstacleMode) {\n"
            "            setenv(\"S10_OBSTACLE_MANUAL_PHASE\", \"1\", 1);\n"
            "            std::cout << \"[OBSTACLE] B: brace and lift body\\n\";\n"
            "        }\n"
            "        else if (k == 'm' && msfb_->GetCurrentState() == RobotMotionState::ObstacleMode) {\n"
            "            const char* value = std::getenv(\"S10_OBSTACLE_FRONT_EXTEND\");\n"
            "            const bool extend = !(value && std::atoi(value) != 0);\n"
            "            setenv(\"S10_OBSTACLE_FRONT_EXTEND\", extend ? \"1\" : \"0\", 1);\n"
            "            std::cout << \"[OBSTACLE] M: front knees \"\n"
            "                      << (extend ? \"extend\" : \"fold\") << \"\\n\";\n"
            "        }\n"
        ),
        marker="[OBSTACLE] M: front knees",
        mode="replace",
    ),
    Edit(
        path=SDK / "interface/user_command/keyboard_interface.hpp",
        anchor=(
            "if (k == '0' || k == 'r' || k == 'z' || k == 'c' || k == 'x' "
            "|| k == 'v' || k == 'b' || k == 'n' || (k >= '1' && k <= '8') || k == '[' || k == ']')"
        ),
        addition=(
            "if (k == '0' || k == 'r' || k == 'z' || k == 'c' || k == 'x' "
            "|| k == 'v' || k == 'b' || k == 'm' || k == 'n' || (k >= '1' && k <= '8') || k == '[' || k == ']')"
        ),
        marker="|| k == 'm' || k == 'n'",
        mode="replace",
        superseded_marker="void HandleKey",
    ),
    Edit(
        path=SDK / "interface/user_command/keyboard_interface.hpp",
        anchor='                  << "  Obstacle:  V (approach)  B (lift)  N x4 (transfer)\\n"\n',
        addition='                  << "  Obstacle:  V (approach)  B (lift)  M (front knees)  N x4 (transfer)\\n"\n',
        marker="M (front knees)",
        mode="replace",
        superseded_marker="H (low/high speed toggle)",
    ),
    Edit(
        path=SDK / "interface/user_command/keyboard_interface.hpp",
        anchor="        std::memset(usr_cmd_, 0, sizeof(UserCommand));\n",
        addition=(
            '        reset_node_ = std::make_shared<rclcpp::Node>("s10_keyboard_reset");\n'
            '        reset_pub_ = reset_node_->create_publisher<std_msgs::msg::Empty>'
            '("/sim/reset", 1);\n'
        ),
        marker="s10_keyboard_reset",
    ),
    Edit(
        path=SDK / "state_machine/quadruped_wheel/rl_control_state.hpp",
        anchor="        float policy_cost_time_ = 1;\n",
        addition=(
            "        float obstacle_pitch_ = -0.58f;\n"
            "        float obstacle_front_speed_ = 20.0f;\n"
            "        float obstacle_front_torque_ = 4.0f;\n"
        ),
        marker="obstacle_front_torque_",
    ),
    Edit(
        path=SDK / "state_machine/quadruped_wheel/rl_control_state.hpp",
        anchor=(
            "            if (uc_ptr_->GetUserCommand()->target_mode == "
            "uint8_t(RobotMotionState::LieDown))\n"
            "                return StateName::kLieDown;\n"
        ),
        addition=(
            "            if (uc_ptr_->GetUserCommand()->target_mode == "
            "uint8_t(RobotMotionState::ObstacleMode))\n"
            "                return StateName::kObstacle;\n"
        ),
        marker="return StateName::kObstacle",
    ),
    # 6. Add a two-stage flat-ground high-speed override to RL control.
    Edit(
        path=SDK / "interface/user_command/keyboard_interface.hpp",
        anchor=(
            "        else if (k == 'c') {\n"
            "            usr_cmd_->target_mode = uint8_t(RobotMotionState::RLControlMode);\n"
            "            std::cout << \"[MODE] RL Control queued\\n\";\n"
            "        }\n"
        ),
        addition=(
            "        else if (k == 'c') {\n"
            "            setenv(\"S10_HIGH_SPEED\", \"0\", 1);\n"
            "            usr_cmd_->target_mode = uint8_t(RobotMotionState::RLControlMode);\n"
            "            std::cout << \"[MODE] RL Control queued\\n\";\n"
            "        }\n"
        ),
        marker='setenv("S10_HIGH_SPEED", "0"',
        mode="replace",
    ),
    Edit(
        path=SDK / "interface/user_command/keyboard_interface.hpp",
        anchor=(
            "        else if (k == 'm' && msfb_->GetCurrentState() == RobotMotionState::ObstacleMode) {\n"
            "            const char* value = std::getenv(\"S10_OBSTACLE_FRONT_EXTEND\");\n"
            "            const bool extend = !(value && std::atoi(value) != 0);\n"
            "            setenv(\"S10_OBSTACLE_FRONT_EXTEND\", extend ? \"1\" : \"0\", 1);\n"
            "            std::cout << \"[OBSTACLE] M: front knees \"\n"
            "                      << (extend ? \"extend\" : \"fold\") << \"\\n\";\n"
            "        }\n"
        ),
        addition=(
            "        else if (k == 'h' && msfb_->GetCurrentState() == RobotMotionState::RLControlMode) {\n"
            "            const char* value = std::getenv(\"S10_HIGH_SPEED\");\n"
            "            const bool enabled = !(value && std::atoi(value) != 0);\n"
            "            setenv(\"S10_HIGH_SPEED\", enabled ? \"1\" : \"0\", 1);\n"
            "            std::cout << \"[HIGH SPEED] H: \"\n"
            "                      << (enabled ? \"ON (crouch then accelerate)\" : \"OFF\") << \"\\n\";\n"
            "        }\n"
        ),
        marker="[HIGH SPEED] H:",
    ),
    Edit(
        path=SDK / "interface/user_command/keyboard_interface.hpp",
        anchor=(
            "if (k == '0' || k == 'r' || k == 'z' || k == 'c' || k == 'x' "
            "|| k == 'v' || k == 'b' || k == 'm' || k == 'n' || (k >= '1' && k <= '8') || k == '[' || k == ']')"
        ),
        addition=(
            "if (k == '0' || k == 'r' || k == 'z' || k == 'c' || k == 'x' "
            "|| k == 'v' || k == 'm' || k == 'h' || (k >= '1' && k <= '8') || k == '[' || k == ']')"
        ),
        marker="|| k == 'm' || k == 'h' ||",
        mode="replace",
        superseded_marker="void HandleKey",
    ),
    Edit(
        path=SDK / "interface/user_command/keyboard_interface.hpp",
        anchor='                  << "  Obstacle:  V (approach)  B (lift)  M (front knees)  N x4 (transfer)\\n"\n',
        addition=(
            '                  << "  Obstacle:  V (approach)  M (extend/pull)\\n"\n'
            '                  << "  Flat:      H (low/high speed toggle)\\n"\n'
        ),
        marker="H (low/high speed toggle)",
        mode="replace",
    ),
    Edit(
        path=SDK / "state_machine/quadruped_wheel/rl_control_state.hpp",
        anchor="        float obstacle_front_torque_ = 4.0f;\n",
        addition=(
            "        static float EnvFloat(const char* name, float fallback) {\n"
            "            const char* value = std::getenv(name);\n"
            "            if (!value) return fallback;\n"
            "            char* end = nullptr;\n"
            "            const float parsed = std::strtof(value, &end);\n"
            "            return end != value && std::isfinite(parsed) && parsed > 0.0f ? parsed : fallback;\n"
            "        }\n\n"
            "        static float Blend(float value) {\n"
            "            value = LimitNumber(value, 0.0f, 1.0f);\n"
            "            return value * value * (3.0f - 2.0f * value);\n"
            "        }\n\n"
            "        static bool HighSpeedRequested() {\n"
            "            const char* value = std::getenv(\"S10_HIGH_SPEED\");\n"
            "            return value && std::atoi(value) != 0;\n"
            "        }\n\n"
            "        float high_speed_phase_ = 0.0f;\n"
            "        float high_speed_hip_ = EnvFloat(\"S10_HIGH_SPEED_HIP\", 0.75f);\n"
            "        float high_speed_knee_ = EnvFloat(\"S10_HIGH_SPEED_KNEE\", 1.50f);\n"
            "        float high_speed_wheel_ = EnvFloat(\"S10_HIGH_SPEED_WHEEL\", 20.0f);\n"
            "        float high_speed_wheel_kd_ = EnvFloat(\"S10_HIGH_SPEED_WHEEL_KD\", 2.0f);\n"
            "        float high_speed_ramp_ = EnvFloat(\"S10_HIGH_SPEED_RAMP\", 1.0f);\n"
        ),
        marker="high_speed_phase_",
    ),
    Edit(
        path=SDK / "state_machine/quadruped_wheel/rl_control_state.hpp",
        anchor="        void PolicyRunner() {\n",
        addition=(
            "        void ApplyHighSpeed(MatXf& command) {\n"
            "            const float step = 0.005f * policy_ptr_->decimation_ / high_speed_ramp_;\n"
            "            high_speed_phase_ = LimitNumber(\n"
            "                high_speed_phase_ + (HighSpeedRequested() ? step : -step), 0.0f, 2.0f);\n"
            "            const float crouch = Blend(high_speed_phase_);\n"
            "            const float drive = Blend(high_speed_phase_ - 1.0f);\n"
            "            for (int leg = 0; leg < 4; ++leg) {\n"
            "                const int joint = leg * 4;\n"
            "                const float sign = leg < 2 ? -1.0f : 1.0f;\n"
            "                command(joint, 1) *= 1.0f - crouch;\n"
            "                command(joint + 1, 1) += crouch * (sign * high_speed_hip_ - command(joint + 1, 1));\n"
            "                command(joint + 2, 1) += crouch * (-sign * high_speed_knee_ - command(joint + 2, 1));\n"
            "                if (command(joint + 3, 2) < high_speed_wheel_kd_)\n"
            "                    command(joint + 3, 2) += crouch * (high_speed_wheel_kd_ - command(joint + 3, 2));\n"
            "                command(joint + 3, 3) += drive * (-high_speed_wheel_ - command(joint + 3, 3));\n"
            "            }\n"
            "        }\n\n"
            "        void PolicyRunner() {\n"
        ),
        marker="void ApplyHighSpeed",
        mode="replace",
    ),
    Edit(
        path=SDK / "state_machine/quadruped_wheel/rl_control_state.hpp",
        anchor="                    MatXf res = ra.ConvertToMat();\n",
        addition="                    ApplyHighSpeed(res);\n",
        marker="ApplyHighSpeed(res)",
    ),
    Edit(
        path=SDK / "state_machine/quadruped_wheel/rl_control_state.hpp",
        anchor="            state_run_cnt_ = -1;\n            start_flag_ = true;\n",
        addition="            high_speed_phase_ = 0.0f;\n",
        marker="start_flag_ = true;\n            high_speed_phase_ = 0.0f;",
    ),
    # 7. Accept keyboard events forwarded by the focused native MuJoCo window.
    Edit(
        path=SDK / "interface/user_command/keyboard_interface.hpp",
        anchor='#include "std_msgs/msg/empty.hpp"\n',
        addition='#include "std_msgs/msg/u_int8.hpp"\n',
        marker="std_msgs/msg/u_int8.hpp",
    ),
    Edit(
        path=SDK / "interface/user_command/keyboard_interface.hpp",
        anchor="    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr reset_pub_;\n",
        addition="    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr key_sub_;\n",
        marker="key_sub_",
    ),
    Edit(
        path=SDK / "interface/user_command/keyboard_interface.hpp",
        anchor="    void keyboard_loop()\n",
        addition=(
            "    void HandleKey(char raw_key, double now) {\n"
            "        const char k = std::tolower(static_cast<unsigned char>(raw_key));\n"
            "        if (k == '0' || k == 'r' || k == 'z' || k == 'c' || k == 'x'\n"
            "            || k == 'v' || k == 'm' || k == 'h'\n"
            "            || (k >= '1' && k <= '8') || k == '[' || k == ']') {\n"
            "            process_mode_command(k);\n"
            "            return;\n"
            "        }\n"
            "        if (velocity_keys_.count(k)) {\n"
            "            std::lock_guard<std::mutex> lock(keys_mutex_);\n"
            "            held_keys_.insert(k);\n"
            "            last_seen_time_[k] = now;\n"
            "        }\n"
            "    }\n\n"
            "    void keyboard_loop()\n"
        ),
        marker="void HandleKey",
        mode="replace",
    ),
    Edit(
        path=SDK / "interface/user_command/keyboard_interface.hpp",
        anchor=(
            "                char k = std::tolower(static_cast<unsigned char>(ch));\n\n"
            "                // Handle mode commands\n"
            "                if (k == '0' || k == 'r' || k == 'z' || k == 'c' || k == 'x' || k == 'v' || k == 'm' || k == 'h' || (k >= '1' && k <= '8') || k == '[' || k == ']') {\n"
            "                    process_mode_command(k);\n"
            "                    continue;\n"
            "                }\n\n"
            "                // Track velocity keys\n"
            "                if (velocity_keys_.count(k)) {\n"
            "                    std::lock_guard<std::mutex> lock(keys_mutex_);\n"
            "                    held_keys_.insert(k);\n"
            "                    last_seen_time_[k] = now;\n"
            "                }\n"
        ),
        addition="                HandleKey(ch, now);\n",
        marker="HandleKey(ch, now);",
        mode="replace",
    ),
    Edit(
        path=SDK / "interface/user_command/keyboard_interface.hpp",
        anchor="        while (running_) {\n",
        addition="            rclcpp::spin_some(reset_node_);\n",
        marker="rclcpp::spin_some(reset_node_)",
    ),
    Edit(
        path=SDK / "interface/user_command/keyboard_interface.hpp",
        anchor=(
            '        reset_pub_ = reset_node_->create_publisher<std_msgs::msg::Empty>'
            '("/sim/reset", 1);\n'
        ),
        addition=(
            "        key_sub_ = reset_node_->create_subscription<std_msgs::msg::UInt8>(\n"
            "            \"/keyboard/key\", 10,\n"
            "            [this](const std_msgs::msg::UInt8::SharedPtr msg) {\n"
            "                HandleKey(static_cast<char>(msg->data), GetCurrentTimeStamp());\n"
            "            });\n"
        ),
        marker='"/keyboard/key"',
    ),
    # 8. Sync the WSLg viewer near 60 Hz instead of every tenth 1 kHz physics step.
    Edit(
        path=SDK / "interface/robot/simulation/mujoco_simulation_ros2.py",
        anchor="RENDER_INTERVAL = 10",
        addition='RENDER_INTERVAL = max(1, int(os.environ.get("S10_RENDER_INTERVAL", "17")))',
        marker="S10_RENDER_INTERVAL",
        mode="replace",
    ),
    # 8. Show the scorer's 0.2 m acceptance radius at every waypoint.
    Edit(
        path=SDK / "S10_description/s10_mjcf/mjcf/track_overlay.xml",
        anchor="    </body>\n  </worldbody>",
        addition=(
            '      <geom name="track_radius_000" type="cylinder" pos="0.0000 -1.7250 0.0080" size="0.2000 0.0040" rgba="1.00 0.60 0.05 0.38" contype="0" conaffinity="0" group="2"/>\n'
            '      <geom name="track_radius_001" type="cylinder" pos="-0.7125 11.6550 0.4830" size="0.2000 0.0040" rgba="1.00 0.60 0.05 0.38" contype="0" conaffinity="0" group="2"/>\n'
            '      <geom name="track_radius_002" type="cylinder" pos="-8.7300 11.8425 0.4830" size="0.2000 0.0040" rgba="1.00 0.60 0.05 0.38" contype="0" conaffinity="0" group="2"/>\n'
            '      <geom name="track_radius_003" type="cylinder" pos="-10.5375 16.3275 0.4830" size="0.2000 0.0040" rgba="1.00 0.60 0.05 0.38" contype="0" conaffinity="0" group="2"/>\n'
            '      <geom name="track_radius_004" type="cylinder" pos="-15.0225 17.3700 0.4830" size="0.2000 0.0040" rgba="1.00 0.60 0.05 0.38" contype="0" conaffinity="0" group="2"/>\n'
            '      <geom name="track_radius_005" type="cylinder" pos="-15.6000 23.2800 0.4830" size="0.2000 0.0040" rgba="1.00 0.60 0.05 0.38" contype="0" conaffinity="0" group="2"/>\n'
            '      <geom name="track_radius_006" type="cylinder" pos="-15.1200 31.8600 0.6080" size="0.2000 0.0040" rgba="1.00 0.60 0.05 0.38" contype="0" conaffinity="0" group="2"/>\n'
            '      <geom name="track_radius_007" type="cylinder" pos="-14.9325 41.2050 1.1730" size="0.2000 0.0040" rgba="1.00 0.60 0.05 0.38" contype="0" conaffinity="0" group="2"/>\n'
            '      <geom name="track_radius_008" type="cylinder" pos="-20.4600 43.1100 1.1730" size="0.2000 0.0040" rgba="1.00 0.60 0.05 0.38" contype="0" conaffinity="0" group="2"/>\n'
            '      <geom name="track_radius_009" type="cylinder" pos="-20.6550 47.7825 1.1730" size="0.2000 0.0040" rgba="1.00 0.60 0.05 0.38" contype="0" conaffinity="0" group="2"/>\n'
            '      <geom name="track_radius_010" type="cylinder" pos="-4.0500 47.9700 1.1730" size="0.2000 0.0040" rgba="1.00 0.60 0.05 0.38" contype="0" conaffinity="0" group="2"/>\n'
            '      <geom name="track_radius_011" type="cylinder" pos="-4.2450 42.5400 1.1730" size="0.2000 0.0040" rgba="1.00 0.60 0.05 0.38" contype="0" conaffinity="0" group="2"/>\n'
            '      <geom name="track_radius_012" type="cylinder" pos="-13.5000 41.5500 1.1730" size="0.2000 0.0040" rgba="1.00 0.60 0.05 0.38" contype="0" conaffinity="0" group="2"/>\n'
            '      <geom name="track_radius_013" type="cylinder" pos="-12.9225 34.6275 0.4830" size="0.2000 0.0040" rgba="1.00 0.60 0.05 0.38" contype="0" conaffinity="0" group="2"/>\n'
            '      <geom name="track_radius_014" type="cylinder" pos="2.7225 34.5300 0.4830" size="0.2000 0.0040" rgba="1.00 0.60 0.05 0.38" contype="0" conaffinity="0" group="2"/>\n'
            '      <geom name="track_radius_015" type="cylinder" pos="11.1225 33.0075 0.1080" size="0.2000 0.0040" rgba="1.00 0.60 0.05 0.38" contype="0" conaffinity="0" group="2"/>\n'
            '      <geom name="track_radius_016" type="cylinder" pos="16.2750 31.2900 0.4830" size="0.2000 0.0040" rgba="1.00 0.60 0.05 0.38" contype="0" conaffinity="0" group="2"/>\n'
            '      <geom name="track_radius_017" type="cylinder" pos="17.6100 29.1900 0.4830" size="0.2000 0.0040" rgba="1.00 0.60 0.05 0.38" contype="0" conaffinity="0" group="2"/>\n'
            '      <geom name="track_radius_018" type="cylinder" pos="25.7175 29.9550 2.2080" size="0.2000 0.0040" rgba="1.00 0.60 0.05 0.38" contype="0" conaffinity="0" group="2"/>\n'
            '      <geom name="track_radius_019" type="cylinder" pos="34.5975 29.9550 2.3680" size="0.2000 0.0040" rgba="1.00 0.60 0.05 0.38" contype="0" conaffinity="0" group="2"/>\n'
            '      <geom name="track_radius_020" type="cylinder" pos="33.9225 24.4275 2.3680" size="0.2000 0.0040" rgba="1.00 0.60 0.05 0.38" contype="0" conaffinity="0" group="2"/>\n'
            '      <geom name="track_radius_021" type="cylinder" pos="17.8950 24.6150 0.4830" size="0.2000 0.0040" rgba="1.00 0.60 0.05 0.38" contype="0" conaffinity="0" group="2"/>\n'
            '      <geom name="track_radius_022" type="cylinder" pos="19.0425 16.8000 0.4830" size="0.2000 0.0040" rgba="1.00 0.60 0.05 0.38" contype="0" conaffinity="0" group="2"/>\n'
            '      <geom name="track_radius_023" type="cylinder" pos="26.4825 17.1825 1.6780" size="0.2000 0.0040" rgba="1.00 0.60 0.05 0.38" contype="0" conaffinity="0" group="2"/>\n'
            '      <geom name="track_radius_024" type="cylinder" pos="31.6350 15.4650 1.6780" size="0.2000 0.0040" rgba="1.00 0.60 0.05 0.38" contype="0" conaffinity="0" group="2"/>\n'
            '      <geom name="track_radius_025" type="cylinder" pos="33.1650 15.1800 1.6780" size="0.2000 0.0040" rgba="1.00 0.60 0.05 0.38" contype="0" conaffinity="0" group="2"/>\n'
            '      <geom name="track_radius_026" type="cylinder" pos="33.6000 21.3750 2.7080" size="0.2000 0.0040" rgba="1.00 0.60 0.05 0.38" contype="0" conaffinity="0" group="2"/>\n'
            '      <geom name="track_radius_027" type="cylinder" pos="34.8825 21.3750 2.7080" size="0.2000 0.0040" rgba="1.00 0.60 0.05 0.38" contype="0" conaffinity="0" group="2"/>\n'
            '      <geom name="track_radius_028" type="cylinder" pos="35.0700 15.6600 3.7580" size="0.2000 0.0040" rgba="1.00 0.60 0.05 0.38" contype="0" conaffinity="0" group="2"/>\n'
            '      <geom name="track_radius_029" type="cylinder" pos="30.9350 14.7150 3.7580" size="0.2000 0.0040" rgba="1.00 0.60 0.05 0.38" contype="0" conaffinity="0" group="2"/>\n'
            '      <geom name="track_radius_030" type="cylinder" pos="29.5350 16.3275 3.7580" size="0.2000 0.0040" rgba="1.00 0.60 0.05 0.38" contype="0" conaffinity="0" group="2"/>\n'
            '      <geom name="track_radius_031" type="cylinder" pos="29.9175 20.6175 3.7580" size="0.2000 0.0040" rgba="1.00 0.60 0.05 0.38" contype="0" conaffinity="0" group="2"/>\n'
            '      <geom name="track_radius_032" type="cylinder" pos="32.9250 18.4500 3.7580" size="0.2000 0.0040" rgba="1.00 0.60 0.05 0.38" contype="0" conaffinity="0" group="2"/>\n'
            "    </body>\n  </worldbody>"
        ),
        marker="track_radius_032",
        mode="replace",
    ),
    # 9. Select an exported checkpoint without overwriting the pristine SDK policy.
    Edit(
        path=SDK / "state_machine/quadruped_wheel/rl_control_state.hpp",
        anchor='#include "basic_function.hpp"\n',
        addition="#include <cstdlib>\n",
        marker="#include <cstdlib>",
    ),
    Edit(
        path=SDK / "state_machine/quadruped_wheel/rl_control_state.hpp",
        anchor=(
            "                fs::path base = fs::path(__FILE__).parent_path();\n"
            "                auto model_path = fs::canonical("
            "base / \"..\" / \"..\" / \"policy\" / \"policy.onnx\");"
        ),
        addition=(
            "                fs::path base = fs::path(__FILE__).parent_path();\n"
            "                const char* override_path = std::getenv(\"S10_POLICY_PATH\");\n"
            "                auto model_path = fs::canonical(\n"
            "                    override_path ? fs::path(override_path)\n"
            "                                  : base / \"..\" / \"..\" / \"policy\" / \"policy.onnx\");"
        ),
        marker="S10_POLICY_PATH",
        mode="replace",
    ),
    # 8. Feed the perceptive policy the simulator's 13x9 height map.
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor='#include "policy_runner_base.hpp"\n',
        addition=(
            "#include <atomic>\n"
            "#include <memory>\n"
            "#include <thread>\n"
            "#include <rclcpp/rclcpp.hpp>\n"
            "#include <std_msgs/msg/float32_multi_array.hpp>\n"
        ),
        marker="float32_multi_array.hpp",
    ),
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor="    const int observation_dim = 57;",
        addition=(
            "    static constexpr int kProprioceptionDim = 57;\n"
            "    static constexpr int kHeightmapDim = 117;\n"
            "    int observation_dim = kProprioceptionDim;"
        ),
        marker="kHeightmapDim",
        mode="replace",
    ),
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor=(
            "    const std::array<int64_t, 2> input_observationShape = "
            "{1, observation_dim};"
        ),
        addition=(
            "    std::array<int64_t, 2> input_observationShape = "
            "{1, kProprioceptionDim};"
        ),
        marker="{1, kProprioceptionDim}",
        mode="replace",
    ),
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor="    bool is_fallen = true;\n",
        addition=(
            "\n"
            "    std::array<std::atomic<float>, kHeightmapDim> heightmap_{};\n"
            "    rclcpp::Node::SharedPtr heightmap_node_;\n"
            "    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr heightmap_sub_;\n"
            "    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> heightmap_executor_;\n"
            "    std::thread heightmap_thread_;\n"
        ),
        marker="heightmap_node_",
    ),
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor="public:\n",
        addition=(
            "    void StartHeightmapSubscriber() {\n"
            "        for (auto& value : heightmap_) value.store(0.0f);\n"
            "        heightmap_node_ = std::make_shared<rclcpp::Node>(\"s10_policy_heightmap\");\n"
            "        heightmap_sub_ = heightmap_node_->create_subscription<\n"
            "            std_msgs::msg::Float32MultiArray>(\n"
            "            \"/perception/heightmap\", 10,\n"
            "            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {\n"
            "                if (msg->data.size() != kHeightmapDim) return;\n"
            "                for (int i = 0; i < kHeightmapDim; ++i)\n"
            "                    heightmap_[i].store(msg->data[i], std::memory_order_relaxed);\n"
            "            });\n"
            "        heightmap_executor_ =\n"
            "            std::make_shared<rclcpp::executors::SingleThreadedExecutor>();\n"
            "        heightmap_executor_->add_node(heightmap_node_);\n"
            "        heightmap_thread_ = std::thread([this]() { heightmap_executor_->spin(); });\n"
            "    }\n\n"
            "    void StopHeightmapSubscriber() {\n"
            "        if (heightmap_executor_) heightmap_executor_->cancel();\n"
            "        if (heightmap_thread_.joinable()) heightmap_thread_.join();\n"
            "    }\n\n"
            "public:\n"
        ),
        marker="StartHeightmapSubscriber",
        mode="replace",
    ),
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor="        session_ = Ort::Session(env_, policy_path_.c_str(), session_options_);\n",
        addition=(
            "        const auto input_shape = session_.GetInputTypeInfo(0)\n"
            "            .GetTensorTypeAndShapeInfo().GetShape();\n"
            "        if (input_shape.size() != 2\n"
            "            || (input_shape[1] != kProprioceptionDim\n"
            "                && input_shape[1] != kProprioceptionDim + kHeightmapDim))\n"
            "            throw std::runtime_error(\"Unsupported ONNX observation width\");\n"
            "        observation_dim = static_cast<int>(input_shape[1]);\n"
            "        input_observationShape[1] = observation_dim;\n"
        ),
        marker="Unsupported ONNX observation width",
        superseded_marker="Preloaded stairs model",
    ),
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor=(
            "        memory_info = Ort::MemoryInfo::CreateCpu("
            "OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);\n"
        ),
        addition=(
            "        if (observation_dim == kProprioceptionDim + kHeightmapDim)\n"
            "            StartHeightmapSubscriber();\n"
        ),
        marker="if (observation_dim == kProprioceptionDim + kHeightmapDim)",
    ),
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor="    ~S10PolicyRunner() override = default;",
        addition="    ~S10PolicyRunner() override { StopHeightmapSubscriber(); }",
        marker="StopHeightmapSubscriber(); }",
        mode="replace",
    ),
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor=(
            "        current_observation_<<base_omgea, \n"
            "                              projected_gravity, \n"
            "                              command, \n"
            "                              joint_pos_rl, \n"
            "                              joint_vel_rl, \n"
            "                              last_action_eigen;"
        ),
        addition=(
            "        current_observation_.head(kProprioceptionDim) << base_omgea,\n"
            "                              projected_gravity,\n"
            "                              command,\n"
            "                              joint_pos_rl,\n"
            "                              joint_vel_rl,\n"
            "                              last_action_eigen;\n"
            "        if (observation_dim == kProprioceptionDim + kHeightmapDim)\n"
            "            for (int i = 0; i < kHeightmapDim; ++i)\n"
            "                current_observation_(kProprioceptionDim + i) =\n"
            "                    heightmap_[i].load(std::memory_order_relaxed);"
        ),
        marker="current_observation_.head(kProprioceptionDim)",
        mode="replace",
    ),
    # 9. The bridge and perceptive runner use ROS message packages.
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
    # 10. Preload a second policy and switch inference sessions at runtime.
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor="#include <utility>\n",
        addition="#include <cstdlib>\n",
        marker="#include <cstdlib>",
    ),
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor="    const std::string policy_path_;\n",
        addition="    const std::string secondary_policy_path_;\n",
        marker="secondary_policy_path_",
    ),
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor="    Ort::Session session_{nullptr};\n",
        addition=(
            "    Ort::Session secondary_session_{nullptr};\n"
            "    bool has_secondary_policy_ = false;\n"
            "    bool using_secondary_policy_ = false;\n"
        ),
        marker="secondary_session_",
    ),
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor="    void StartHeightmapSubscriber() {\n",
        addition=(
            "    int ValidateSession(Ort::Session& session, const char* label) {\n"
            "        if (session.GetInputCount() != 1 || session.GetOutputCount() != 1)\n"
            "            throw std::runtime_error(std::string(label) +\n"
            "                                     \" ONNX must have one input and one output\");\n"
            "        const auto input_shape = session.GetInputTypeInfo(0)\n"
            "            .GetTensorTypeAndShapeInfo().GetShape();\n"
            "        if (input_shape.size() != 2\n"
            "            || (input_shape[1] != kProprioceptionDim\n"
            "                && input_shape[1] != kProprioceptionDim + kHeightmapDim))\n"
            "            throw std::runtime_error(std::string(label) +\n"
            "                                     \" ONNX observation width is unsupported\");\n"
            "        const auto output_shape = session.GetOutputTypeInfo(0)\n"
            "            .GetTensorTypeAndShapeInfo().GetShape();\n"
            "        if (output_shape.size() != 2 || output_shape[1] != action_dim)\n"
            "            throw std::runtime_error(std::string(label) +\n"
            "                                     \" ONNX action width is unsupported\");\n"
            "        return static_cast<int>(input_shape[1]);\n"
            "    }\n\n"
            "    static bool SecondaryPolicyRequested() {\n"
            "        const char* value = std::getenv(\"S10_USE_SECOND_POLICY\");\n"
            "        return value && std::atoi(value) != 0;\n"
            "    }\n\n"
            "    void UpdatePolicySelection() {\n"
            "        const bool use_secondary =\n"
            "            has_secondary_policy_ && SecondaryPolicyRequested();\n"
            "        if (use_secondary == using_secondary_policy_) return;\n"
            "        using_secondary_policy_ = use_secondary;\n"
            "        last_action_eigen.setZero(action_dim);\n"
            "        current_action_eigen.setZero(action_dim);\n"
            "        tmp_action_eigen.setZero(action_dim);\n"
            "        std::cout << \"[POLICY] Active model: \"\n"
            "                  << (use_secondary ? \"stairs\" : \"default\") << \"\\n\";\n"
            "    }\n\n"
            "    void StartHeightmapSubscriber() {\n"
        ),
        marker="SecondaryPolicyRequested",
        superseded_marker="RequestedPolicySlot",
        mode="replace",
    ),
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor=(
            "    S10PolicyRunner(const std::string &policy_name, const std::string &policy_path) :\n"
            "            PolicyRunnerBase(policy_name), policy_path_(policy_path),env_(ORT_LOGGING_LEVEL_WARNING, \"S10PolicyRunner\"),\n"
        ),
        addition=(
            "    S10PolicyRunner(const std::string &policy_name, const std::string &policy_path,\n"
            "                    const std::string &secondary_policy_path = \"\") :\n"
            "            PolicyRunnerBase(policy_name), policy_path_(policy_path),\n"
            "            secondary_policy_path_(secondary_policy_path),\n"
            "            env_(ORT_LOGGING_LEVEL_WARNING, \"S10PolicyRunner\"),\n"
        ),
        marker="const std::string &secondary_policy_path",
        mode="replace",
    ),
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor=(
            "        const auto input_shape = session_.GetInputTypeInfo(0)\n"
            "            .GetTensorTypeAndShapeInfo().GetShape();\n"
            "        if (input_shape.size() != 2\n"
            "            || (input_shape[1] != kProprioceptionDim\n"
            "                && input_shape[1] != kProprioceptionDim + kHeightmapDim))\n"
            "            throw std::runtime_error(\"Unsupported ONNX observation width\");\n"
            "        observation_dim = static_cast<int>(input_shape[1]);\n"
            "        input_observationShape[1] = observation_dim;\n"
        ),
        addition=(
            "        observation_dim = ValidateSession(session_, \"Primary\");\n"
            "        input_observationShape[1] = observation_dim;\n"
            "        if (!secondary_policy_path_.empty()) {\n"
            "            if (access(secondary_policy_path_.c_str(), F_OK) != 0)\n"
            "                throw std::runtime_error(\"Secondary model file missing: \" +\n"
            "                                         secondary_policy_path_);\n"
            "            secondary_session_ = Ort::Session(\n"
            "                env_, secondary_policy_path_.c_str(), session_options_);\n"
            "            const int secondary_observation_dim =\n"
            "                ValidateSession(secondary_session_, \"Secondary\");\n"
            "            if (secondary_observation_dim != observation_dim)\n"
            "                throw std::runtime_error(\n"
            "                    \"Primary and secondary ONNX observation widths differ\");\n"
            "            has_secondary_policy_ = true;\n"
            "            std::cout << \"[POLICY] Preloaded stairs model: \"\n"
            "                      << secondary_policy_path_ << \"\\n\";\n"
            "        }\n"
        ),
        marker="Preloaded stairs model",
        mode="replace",
    ),
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor="        auto outputs = session_.Run(\n",
        addition=(
            "        auto& active_session =\n"
            "            using_secondary_policy_ ? secondary_session_ : session_;\n"
            "        auto outputs = active_session.Run(\n"
        ),
        marker="active_session.Run",
        mode="replace",
    ),
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor=(
            "    RobotAction getRobotAction(const RobotBasicState &ro, const UserCommand &uc) {\n\n"
        ),
        addition=(
            "    RobotAction getRobotAction(const RobotBasicState &ro, const UserCommand &uc) {\n\n"
            "        UpdatePolicySelection();\n"
        ),
        marker="UpdatePolicySelection();",
        mode="replace",
    ),
    Edit(
        path=SDK / "state_machine/quadruped_wheel/rl_control_state.hpp",
        anchor=(
            "                const char* override_path = std::getenv(\"S10_POLICY_PATH\");\n"
            "                auto model_path = fs::canonical(\n"
            "                    override_path ? fs::path(override_path)\n"
            "                                  : base / \"..\" / \"..\" / \"policy\" / \"policy.onnx\");\n"
            "                s10_policy_ = std::make_shared<S10PolicyRunner>(\"s10_policy\", model_path.string());\n"
        ),
        addition=(
            "                const char* override_path = std::getenv(\"S10_POLICY_PATH\");\n"
            "                auto model_path = fs::canonical(\n"
            "                    override_path ? fs::path(override_path)\n"
            "                                  : base / \"..\" / \"..\" / \"policy\" / \"policy.onnx\");\n"
            "                const char* secondary_path = std::getenv(\"S10_SECOND_POLICY_PATH\");\n"
            "                const auto secondary_model_path = secondary_path && *secondary_path\n"
            "                    ? fs::canonical(fs::path(secondary_path)).string()\n"
            "                    : std::string();\n"
            "                s10_policy_ = std::make_shared<S10PolicyRunner>(\n"
            "                    \"s10_policy\", model_path.string(), secondary_model_path);\n"
        ),
        marker="S10_SECOND_POLICY_PATH",
        mode="replace",
    ),
    Edit(
        path=SDK / "interface/user_command/keyboard_interface.hpp",
        anchor=(
            "        else if (k == 'h' && msfb_->GetCurrentState() == RobotMotionState::RLControlMode) {\n"
            "            const char* value = std::getenv(\"S10_HIGH_SPEED\");\n"
            "            const bool enabled = !(value && std::atoi(value) != 0);\n"
            "            setenv(\"S10_HIGH_SPEED\", enabled ? \"1\" : \"0\", 1);\n"
            "            std::cout << \"[HIGH SPEED] H: \"\n"
            "                      << (enabled ? \"ON (crouch then accelerate)\" : \"OFF\") << \"\\n\";\n"
            "        }\n"
        ),
        addition=(
            "        else if (k == 'p') {\n"
            "            const char* path = std::getenv(\"S10_SECOND_POLICY_PATH\");\n"
            "            if (!path || !*path) {\n"
            "                std::cout << \"[POLICY] P: stairs model unavailable\\n\";\n"
            "            } else {\n"
            "                const char* value = std::getenv(\"S10_USE_SECOND_POLICY\");\n"
            "                const bool enabled = !(value && std::atoi(value) != 0);\n"
            "                setenv(\"S10_USE_SECOND_POLICY\", enabled ? \"1\" : \"0\", 1);\n"
            "                if (enabled) setenv(\"S10_HIGH_SPEED\", \"0\", 1);\n"
            "                std::cout << \"[POLICY] P: \"\n"
            "                          << (enabled ? \"stairs\" : \"default\") << \"\\n\";\n"
            "            }\n"
            "        }\n"
        ),
        marker="[POLICY] P:",
        superseded_marker="TogglePolicy(1, 'P'",
    ),
    Edit(
        path=SDK / "interface/user_command/keyboard_interface.hpp",
        anchor="            || k == 'v' || k == 'm' || k == 'h'\n",
        addition="            || k == 'v' || k == 'm' || k == 'h' || k == 'p'\n",
        marker="|| k == 'p'",
        mode="replace",
    ),
    Edit(
        path=SDK / "interface/user_command/keyboard_interface.hpp",
        anchor='                  << "  Flat:      H (low/high speed toggle)\\n"\n',
        addition='                  << "  Policy:    P (default/stairs toggle)\\n"\n',
        marker="default/stairs toggle",
        superseded_marker="K (speed-turn)",
    ),
    # 11. Add a third preloaded slot for the stairs-down policy.
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor="    const std::string secondary_policy_path_;\n",
        addition="    const std::string down_policy_path_;\n",
        marker="down_policy_path_",
    ),
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor="    Ort::Session secondary_session_{nullptr};\n",
        addition="    Ort::Session down_session_{nullptr};\n",
        marker="down_session_",
    ),
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor=(
            "    bool has_secondary_policy_ = false;\n"
            "    bool using_secondary_policy_ = false;\n"
        ),
        addition=(
            "    bool has_secondary_policy_ = false;\n"
            "    bool has_down_policy_ = false;\n"
            "    int active_policy_ = 0;\n"
        ),
        marker="active_policy_",
        mode="replace",
    ),
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor=(
            "    static bool SecondaryPolicyRequested() {\n"
            "        const char* value = std::getenv(\"S10_USE_SECOND_POLICY\");\n"
            "        return value && std::atoi(value) != 0;\n"
            "    }\n\n"
            "    void UpdatePolicySelection() {\n"
            "        const bool use_secondary =\n"
            "            has_secondary_policy_ && SecondaryPolicyRequested();\n"
            "        if (use_secondary == using_secondary_policy_) return;\n"
            "        using_secondary_policy_ = use_secondary;\n"
            "        last_action_eigen.setZero(action_dim);\n"
            "        current_action_eigen.setZero(action_dim);\n"
            "        tmp_action_eigen.setZero(action_dim);\n"
            "        std::cout << \"[POLICY] Active model: \"\n"
            "                  << (use_secondary ? \"stairs\" : \"default\") << \"\\n\";\n"
            "    }\n"
        ),
        addition=(
            "    static int RequestedPolicySlot() {\n"
            "        const char* value = std::getenv(\"S10_POLICY_SLOT\");\n"
            "        return value ? std::atoi(value) : 0;\n"
            "    }\n\n"
            "    void UpdatePolicySelection() {\n"
            "        const int requested = RequestedPolicySlot();\n"
            "        const int selected =\n"
            "            requested == 1 && has_secondary_policy_ ? 1 :\n"
            "            requested == 2 && has_down_policy_ ? 2 : 0;\n"
            "        if (selected == active_policy_) return;\n"
            "        active_policy_ = selected;\n"
            "        last_action_eigen.setZero(action_dim);\n"
            "        current_action_eigen.setZero(action_dim);\n"
            "        tmp_action_eigen.setZero(action_dim);\n"
            "        const char* names[] = {\"default\", \"stairs-up\", \"stairs-down\"};\n"
            "        std::cout << \"[POLICY] Active model: \"\n"
            "                  << names[active_policy_] << \"\\n\";\n"
            "    }\n"
        ),
        marker="RequestedPolicySlot",
        mode="replace",
    ),
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor=(
            "    S10PolicyRunner(const std::string &policy_name, const std::string &policy_path,\n"
            "                    const std::string &secondary_policy_path = \"\") :\n"
            "            PolicyRunnerBase(policy_name), policy_path_(policy_path),\n"
            "            secondary_policy_path_(secondary_policy_path),\n"
        ),
        addition=(
            "    S10PolicyRunner(const std::string &policy_name, const std::string &policy_path,\n"
            "                    const std::string &secondary_policy_path = \"\",\n"
            "                    const std::string &down_policy_path = \"\") :\n"
            "            PolicyRunnerBase(policy_name), policy_path_(policy_path),\n"
            "            secondary_policy_path_(secondary_policy_path),\n"
            "            down_policy_path_(down_policy_path),\n"
        ),
        marker="const std::string &down_policy_path",
        mode="replace",
    ),
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor=(
            "            std::cout << \"[POLICY] Preloaded stairs model: \"\n"
            "                      << secondary_policy_path_ << \"\\n\";\n"
            "        }\n"
        ),
        addition=(
            "            std::cout << \"[POLICY] Preloaded stairs model: \"\n"
            "                      << secondary_policy_path_ << \"\\n\";\n"
            "        }\n"
            "        if (!down_policy_path_.empty()) {\n"
            "            if (access(down_policy_path_.c_str(), F_OK) != 0)\n"
            "                throw std::runtime_error(\"Down model file missing: \" +\n"
            "                                         down_policy_path_);\n"
            "            down_session_ = Ort::Session(\n"
            "                env_, down_policy_path_.c_str(), session_options_);\n"
            "            const int down_observation_dim =\n"
            "                ValidateSession(down_session_, \"Down\");\n"
            "            if (down_observation_dim != observation_dim)\n"
            "                throw std::runtime_error(\n"
            "                    \"Primary and down ONNX observation widths differ\");\n"
            "            has_down_policy_ = true;\n"
            "            std::cout << \"[POLICY] Preloaded stairs-down model: \"\n"
            "                      << down_policy_path_ << \"\\n\";\n"
            "        }\n"
        ),
        marker="Preloaded stairs-down model",
        mode="replace",
    ),
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor=(
            "        auto& active_session =\n"
            "            using_secondary_policy_ ? secondary_session_ : session_;\n"
        ),
        addition=(
            "        auto& active_session = active_policy_ == 2 ? down_session_\n"
            "            : active_policy_ == 1 ? secondary_session_ : session_;\n"
        ),
        marker="active_policy_ == 2 ? down_session_",
        mode="replace",
    ),
    Edit(
        path=SDK / "state_machine/quadruped_wheel/rl_control_state.hpp",
        anchor=(
            "                const char* secondary_path = std::getenv(\"S10_SECOND_POLICY_PATH\");\n"
            "                const auto secondary_model_path = secondary_path && *secondary_path\n"
            "                    ? fs::canonical(fs::path(secondary_path)).string()\n"
            "                    : std::string();\n"
            "                s10_policy_ = std::make_shared<S10PolicyRunner>(\n"
            "                    \"s10_policy\", model_path.string(), secondary_model_path);\n"
        ),
        addition=(
            "                const char* secondary_path = std::getenv(\"S10_SECOND_POLICY_PATH\");\n"
            "                const auto secondary_model_path = secondary_path && *secondary_path\n"
            "                    ? fs::canonical(fs::path(secondary_path)).string()\n"
            "                    : std::string();\n"
            "                const char* down_path = std::getenv(\"S10_DOWN_POLICY_PATH\");\n"
            "                const auto down_model_path = down_path && *down_path\n"
            "                    ? fs::canonical(fs::path(down_path)).string()\n"
            "                    : std::string();\n"
            "                s10_policy_ = std::make_shared<S10PolicyRunner>(\n"
            "                    \"s10_policy\", model_path.string(),\n"
            "                    secondary_model_path, down_model_path);\n"
        ),
        marker="S10_DOWN_POLICY_PATH",
        mode="replace",
    ),
    Edit(
        path=SDK / "interface/user_command/keyboard_interface.hpp",
        anchor="    void process_mode_command(char k)\n",
        addition=(
            "    void TogglePolicy(int slot, char key, const char* label, const char* path_env) {\n"
            "        const char* path = std::getenv(path_env);\n"
            "        if (!path || !*path) {\n"
            "            std::cout << \"[POLICY] \" << key << \": \"\n"
            "                      << label << \" model unavailable\\n\";\n"
            "            return;\n"
            "        }\n"
            "        const char* value = std::getenv(\"S10_POLICY_SLOT\");\n"
            "        const int selected = value && std::atoi(value) == slot ? 0 : slot;\n"
            "        const std::string text = std::to_string(selected);\n"
            "        setenv(\"S10_POLICY_SLOT\", text.c_str(), 1);\n"
            "        if (selected != 0) setenv(\"S10_HIGH_SPEED\", \"0\", 1);\n"
            "        std::cout << \"[POLICY] \" << key << \": \"\n"
            "                  << (selected == 0 ? \"default\" : label) << \"\\n\";\n"
            "    }\n\n"
            "    void process_mode_command(char k)\n"
        ),
        marker="void TogglePolicy",
        mode="replace",
    ),
    Edit(
        path=SDK / "interface/user_command/keyboard_interface.hpp",
        anchor=(
            "        else if (k == 'p') {\n"
            "            const char* path = std::getenv(\"S10_SECOND_POLICY_PATH\");\n"
            "            if (!path || !*path) {\n"
            "                std::cout << \"[POLICY] P: stairs model unavailable\\n\";\n"
            "            } else {\n"
            "                const char* value = std::getenv(\"S10_USE_SECOND_POLICY\");\n"
            "                const bool enabled = !(value && std::atoi(value) != 0);\n"
            "                setenv(\"S10_USE_SECOND_POLICY\", enabled ? \"1\" : \"0\", 1);\n"
            "                if (enabled) setenv(\"S10_HIGH_SPEED\", \"0\", 1);\n"
            "                std::cout << \"[POLICY] P: \"\n"
            "                          << (enabled ? \"stairs\" : \"default\") << \"\\n\";\n"
            "            }\n"
            "        }\n"
        ),
        addition=(
            "        else if (k == 'p') {\n"
            "            TogglePolicy(1, 'P', \"stairs-up\", \"S10_SECOND_POLICY_PATH\");\n"
            "        }\n"
            "        else if (k == 'l') {\n"
            "            TogglePolicy(2, 'L', \"stairs-down\", \"S10_DOWN_POLICY_PATH\");\n"
            "        }\n"
        ),
        marker="TogglePolicy(2, 'L'",
        mode="replace",
    ),
    Edit(
        path=SDK / "interface/user_command/keyboard_interface.hpp",
        anchor="            || k == 'v' || k == 'm' || k == 'h' || k == 'p'\n",
        addition="            || k == 'v' || k == 'm' || k == 'h' || k == 'p' || k == 'l'\n",
        marker="|| k == 'l'",
        mode="replace",
    ),
    Edit(
        path=SDK / "interface/user_command/keyboard_interface.hpp",
        anchor='                  << "  Policy:    P (default/stairs toggle)\\n"\n',
        addition='                  << "  Policy:    P (default/stairs toggle)  L (default/down toggle)\\n"\n',
        marker="default/down toggle",
        superseded_marker="K (speed-turn)",
        mode="replace",
    ),
    # 12. Add a fourth preloaded slot for the speed-turn policy.
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor="    const std::string down_policy_path_;\n",
        addition="    const std::string speedturn_policy_path_;\n",
        marker="speedturn_policy_path_",
    ),
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor="    Ort::Session down_session_{nullptr};\n",
        addition="    Ort::Session speedturn_session_{nullptr};\n",
        marker="speedturn_session_",
    ),
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor="    bool has_down_policy_ = false;\n",
        addition="    bool has_speedturn_policy_ = false;\n",
        marker="has_speedturn_policy_",
    ),
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor=(
            "    void UpdatePolicySelection() {\n"
            "        const int requested = RequestedPolicySlot();\n"
            "        const int selected =\n"
            "            requested == 1 && has_secondary_policy_ ? 1 :\n"
            "            requested == 2 && has_down_policy_ ? 2 : 0;\n"
            "        if (selected == active_policy_) return;\n"
            "        active_policy_ = selected;\n"
            "        last_action_eigen.setZero(action_dim);\n"
            "        current_action_eigen.setZero(action_dim);\n"
            "        tmp_action_eigen.setZero(action_dim);\n"
            "        const char* names[] = {\"default\", \"stairs-up\", \"stairs-down\"};\n"
            "        std::cout << \"[POLICY] Active model: \"\n"
            "                  << names[active_policy_] << \"\\n\";\n"
            "    }\n"
        ),
        addition=(
            "    void UpdatePolicySelection() {\n"
            "        const int requested = RequestedPolicySlot();\n"
            "        const int selected =\n"
            "            requested == 1 && has_secondary_policy_ ? 1 :\n"
            "            requested == 2 && has_down_policy_ ? 2 :\n"
            "            requested == 3 && has_speedturn_policy_ ? 3 : 0;\n"
            "        if (selected == active_policy_) return;\n"
            "        active_policy_ = selected;\n"
            "        last_action_eigen.setZero(action_dim);\n"
            "        current_action_eigen.setZero(action_dim);\n"
            "        tmp_action_eigen.setZero(action_dim);\n"
            "        const char* names[] = {\"default\", \"stairs-up\", \"stairs-down\", \"speed-turn\"};\n"
            "        std::cout << \"[POLICY] Active model: \"\n"
            "                  << names[active_policy_] << \"\\n\";\n"
            "    }\n"
        ),
        marker="requested == 3 && has_speedturn_policy_",
        mode="replace",
    ),
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor=(
            "    S10PolicyRunner(const std::string &policy_name, const std::string &policy_path,\n"
            "                    const std::string &secondary_policy_path = \"\",\n"
            "                    const std::string &down_policy_path = \"\") :\n"
            "            PolicyRunnerBase(policy_name), policy_path_(policy_path),\n"
            "            secondary_policy_path_(secondary_policy_path),\n"
            "            down_policy_path_(down_policy_path),\n"
        ),
        addition=(
            "    S10PolicyRunner(const std::string &policy_name, const std::string &policy_path,\n"
            "                    const std::string &secondary_policy_path = \"\",\n"
            "                    const std::string &down_policy_path = \"\",\n"
            "                    const std::string &speedturn_policy_path = \"\") :\n"
            "            PolicyRunnerBase(policy_name), policy_path_(policy_path),\n"
            "            secondary_policy_path_(secondary_policy_path),\n"
            "            down_policy_path_(down_policy_path),\n"
            "            speedturn_policy_path_(speedturn_policy_path),\n"
        ),
        marker="const std::string &speedturn_policy_path",
        mode="replace",
    ),
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor=(
            "            std::cout << \"[POLICY] Preloaded stairs-down model: \"\n"
            "                      << down_policy_path_ << \"\\n\";\n"
            "        }\n"
        ),
        addition=(
            "            std::cout << \"[POLICY] Preloaded stairs-down model: \"\n"
            "                      << down_policy_path_ << \"\\n\";\n"
            "        }\n"
            "        if (!speedturn_policy_path_.empty()) {\n"
            "            if (access(speedturn_policy_path_.c_str(), F_OK) != 0)\n"
            "                throw std::runtime_error(\"Speed-turn model file missing: \" +\n"
            "                                         speedturn_policy_path_);\n"
            "            speedturn_session_ = Ort::Session(\n"
            "                env_, speedturn_policy_path_.c_str(), session_options_);\n"
            "            const int speedturn_observation_dim =\n"
            "                ValidateSession(speedturn_session_, \"Speed-turn\");\n"
            "            if (speedturn_observation_dim != observation_dim)\n"
            "                throw std::runtime_error(\n"
            "                    \"Primary and speed-turn ONNX observation widths differ\");\n"
            "            has_speedturn_policy_ = true;\n"
            "            std::cout << \"[POLICY] Preloaded speed-turn model: \"\n"
            "                      << speedturn_policy_path_ << \"\\n\";\n"
            "        }\n"
        ),
        marker="Preloaded speed-turn model",
        mode="replace",
    ),
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor=(
            "        auto& active_session = active_policy_ == 2 ? down_session_\n"
            "            : active_policy_ == 1 ? secondary_session_ : session_;\n"
        ),
        addition=(
            "        auto& active_session = active_policy_ == 3 ? speedturn_session_\n"
            "            : active_policy_ == 2 ? down_session_\n"
            "            : active_policy_ == 1 ? secondary_session_ : session_;\n"
        ),
        marker="active_policy_ == 3 ? speedturn_session_",
        mode="replace",
    ),
    Edit(
        path=SDK / "state_machine/quadruped_wheel/rl_control_state.hpp",
        anchor=(
            "                const char* down_path = std::getenv(\"S10_DOWN_POLICY_PATH\");\n"
            "                const auto down_model_path = down_path && *down_path\n"
            "                    ? fs::canonical(fs::path(down_path)).string()\n"
            "                    : std::string();\n"
            "                s10_policy_ = std::make_shared<S10PolicyRunner>(\n"
            "                    \"s10_policy\", model_path.string(),\n"
            "                    secondary_model_path, down_model_path);\n"
        ),
        addition=(
            "                const char* down_path = std::getenv(\"S10_DOWN_POLICY_PATH\");\n"
            "                const auto down_model_path = down_path && *down_path\n"
            "                    ? fs::canonical(fs::path(down_path)).string()\n"
            "                    : std::string();\n"
            "                const char* speedturn_path = std::getenv(\"S10_SPEEDTURN_POLICY_PATH\");\n"
            "                const auto speedturn_model_path = speedturn_path && *speedturn_path\n"
            "                    ? fs::canonical(fs::path(speedturn_path)).string()\n"
            "                    : std::string();\n"
            "                s10_policy_ = std::make_shared<S10PolicyRunner>(\n"
            "                    \"s10_policy\", model_path.string(),\n"
            "                    secondary_model_path, down_model_path, speedturn_model_path);\n"
        ),
        marker="S10_SPEEDTURN_POLICY_PATH",
        mode="replace",
    ),
    Edit(
        path=SDK / "interface/user_command/keyboard_interface.hpp",
        anchor=(
            "        else if (k == 'p') {\n"
            "            TogglePolicy(1, 'P', \"stairs-up\", \"S10_SECOND_POLICY_PATH\");\n"
            "        }\n"
            "        else if (k == 'l') {\n"
            "            TogglePolicy(2, 'L', \"stairs-down\", \"S10_DOWN_POLICY_PATH\");\n"
            "        }\n"
        ),
        addition=(
            "        else if (k == 'p') {\n"
            "            TogglePolicy(1, 'P', \"stairs-up\", \"S10_SECOND_POLICY_PATH\");\n"
            "        }\n"
            "        else if (k == 'l') {\n"
            "            TogglePolicy(2, 'L', \"stairs-down\", \"S10_DOWN_POLICY_PATH\");\n"
            "        }\n"
            "        else if (k == 'k') {\n"
            "            TogglePolicy(3, 'K', \"speed-turn\", \"S10_SPEEDTURN_POLICY_PATH\");\n"
            "        }\n"
        ),
        marker="TogglePolicy(3, 'K'",
        mode="replace",
    ),
    Edit(
        path=SDK / "interface/user_command/keyboard_interface.hpp",
        anchor="            || k == 'v' || k == 'm' || k == 'h' || k == 'p' || k == 'l'\n",
        addition="            || k == 'v' || k == 'm' || k == 'h' || k == 'p' || k == 'l' || k == 'k'\n",
        marker="|| k == 'k'",
        mode="replace",
    ),
    Edit(
        path=SDK / "interface/user_command/keyboard_interface.hpp",
        anchor='                  << "  Policy:    P (default/stairs toggle)  L (default/down toggle)\\n"\n',
        addition='                  << "  Policy:    P (stairs-up)  L (stairs-down)  K (speed-turn)\\n"\n',
        marker="K (speed-turn)",
        mode="replace",
    ),
    # 16. Optional one-shot policy trace for cross-runtime interface verification.
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor="#include <cstdlib>\n",
        addition="#include <fstream>\n#include <iomanip>\n",
        marker="#include <fstream>",
    ),
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor="    std::thread heightmap_thread_;\n",
        addition=(
            "    std::ofstream trace_file_;\n"
            "    int trace_steps_remaining_ = 0;\n"
        ),
        marker="trace_steps_remaining_",
    ),
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor=(
            "    void StopHeightmapSubscriber() {\n"
            "        if (heightmap_executor_) heightmap_executor_->cancel();\n"
            "        if (heightmap_thread_.joinable()) heightmap_thread_.join();\n"
            "    }\n"
        ),
        addition=(
            "\n    void WriteTrace(const RobotBasicState& state, const Vec3f& command,\n"
            "                    const VecXf& raw_action, const VecXf& decoded_command) {\n"
            "        if (!trace_file_.is_open() || trace_steps_remaining_ <= 0) return;\n"
            "        auto write_vector = [this](const auto& values) {\n"
            "            trace_file_ << \"[\";\n"
            "            for (int i = 0; i < values.size(); ++i) {\n"
            "                if (i) trace_file_ << \",\";\n"
            "                trace_file_ << values(i);\n"
            "            }\n"
            "            trace_file_ << \"]\";\n"
            "        };\n"
            "        trace_file_ << std::setprecision(9) << \"{\\\"base_omega\\\":\";\n"
            "        write_vector(state.base_omega);\n"
            "        trace_file_ << \",\\\"joint_pos_robot\\\":\";\n"
            "        write_vector(state.joint_pos);\n"
            "        trace_file_ << \",\\\"joint_vel_robot\\\":\";\n"
            "        write_vector(state.joint_vel);\n"
            "        trace_file_ << \",\\\"command\\\":\";\n"
            "        write_vector(command);\n"
            "        trace_file_ << \",\\\"observation\\\":\";\n"
            "        write_vector(current_observation_);\n"
            "        trace_file_ << \",\\\"raw_action_policy\\\":\";\n"
            "        write_vector(raw_action);\n"
            "        trace_file_ << \",\\\"decoded_command_robot\\\":\";\n"
            "        write_vector(decoded_command);\n"
            "        trace_file_ << \"}\\n\";\n"
            "        trace_file_.flush();\n"
            "        --trace_steps_remaining_;\n"
            "    }\n"
        ),
        marker="void WriteTrace(",
    ),
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor=(
            "        memory_info = Ort::MemoryInfo::CreateCpu("
            "OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);\n"
        ),
        addition=(
            "        const char* trace_path = std::getenv(\"S10_POLICY_TRACE_PATH\");\n"
            "        if (trace_path && *trace_path) {\n"
            "            trace_file_.open(trace_path, std::ios::out | std::ios::trunc);\n"
            "            const char* trace_steps = std::getenv(\"S10_POLICY_TRACE_STEPS\");\n"
            "            trace_steps_remaining_ = trace_steps ? std::atoi(trace_steps) : 1;\n"
            "            if (trace_steps_remaining_ < 1) trace_steps_remaining_ = 1;\n"
            "        }\n"
        ),
        marker="S10_POLICY_TRACE_PATH",
    ),
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor="        tmp_action_eigen += dof_default_eigen_robot;\n",
        addition="        WriteTrace(ro, command, current_action_eigen, tmp_action_eigen);\n",
        marker="WriteTrace(ro, command",
    ),
    # 17. Phase policies append sin/cos to the unchanged 57D prefix. Slots may mix widths.
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor="    int observation_dim = kProprioceptionDim;\n",
        addition="""    static constexpr int kPhaseDim = 2;
    std::array<int, 4> policy_observation_dims_{};
    std::array<double, 4> policy_phase_cycles_{};
    unsigned long long phase_step_ = 0;
""",
        marker="policy_observation_dims_",
    ),
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor="            || (input_shape[1] != kProprioceptionDim\n",
        addition="                && input_shape[1] != kProprioceptionDim + kPhaseDim\n",
        marker="input_shape[1] != kProprioceptionDim + kPhaseDim",
    ),
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor="    static int RequestedPolicySlot() {\n",
        addition="""    static double PhaseCycle(Ort::Session& session, int width) {
        if (width != kProprioceptionDim + kPhaseDim) return 0.6;
        Ort::AllocatorWithDefaultOptions allocator;
        auto value = session.GetModelMetadata().LookupCustomMetadataMapAllocated(
            "phase_cycle_time", allocator);
        if (!value) return 0.6;  // Current S10 phase contract when metadata is absent.
        char* end = nullptr;
        const double cycle = std::strtod(value.get(), &end);
        if (end == value.get() || *end || !std::isfinite(cycle) || cycle <= 0.0)
            throw std::runtime_error("Invalid ONNX phase_cycle_time");
        return cycle;
    }

    static int RequestedPolicySlot() {
""",
        marker="static double PhaseCycle",
        mode="replace",
    ),
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor="        active_policy_ = selected;\n",
        addition="""        observation_dim = policy_observation_dims_[selected];
        input_observationShape[1] = observation_dim;
        current_observation_.setZero(observation_dim);
        phase_step_ = 0;
""",
        marker="observation_dim = policy_observation_dims_[selected]",
    ),
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor='        observation_dim = ValidateSession(session_, "Primary");\n',
        addition="""        policy_observation_dims_[0] = observation_dim;
        policy_phase_cycles_[0] = PhaseCycle(session_, observation_dim);
""",
        marker="policy_observation_dims_[0] =",
    ),
    *[
        Edit(
            path=SDK / "run_policy/s10_policy_runner.hpp",
            anchor=(f"            if ({name}_observation_dim != observation_dim)\n"
                    "                throw std::runtime_error(\n"
                    f'                    "Primary and {label} ONNX observation widths differ");\n'),
            addition=(f"            policy_observation_dims_[{slot}] = {name}_observation_dim;\n"
                      f"            policy_phase_cycles_[{slot}] = PhaseCycle({name}_session_, {name}_observation_dim);\n"),
            marker=f"policy_observation_dims_[{slot}] =",
            mode="replace",
        )
        for slot, name, label in ((1, "secondary", "secondary"), (2, "down", "down"),
                                  (3, "speedturn", "speed-turn"))
    ],
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor="""        if (observation_dim == kProprioceptionDim + kHeightmapDim)
            StartHeightmapSubscriber();
""",
        addition="""        for (int width : policy_observation_dims_)
            if (width == kProprioceptionDim + kHeightmapDim) {
                StartHeightmapSubscriber();
                break;
            }
""",
        marker="for (int width : policy_observation_dims_)",
        mode="replace",
    ),
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor="        run_cnt_ = 0;\n",
        addition="        phase_step_ = 0;\n",
        marker="run_cnt_ = 0;\n        phase_step_ = 0;",
    ),
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor="        current_action_eigen = Onnx_infer(current_observation_);\n",
        addition="""        if (observation_dim == kProprioceptionDim + kPhaseDim) {
            const double angle = 2.0 * PI * std::fmod(
                phase_step_ * double(agent_timestep) / policy_phase_cycles_[active_policy_], 1.0);
            current_observation_(57) = std::sin(angle);
            current_observation_(58) = std::cos(angle);
        }
        current_action_eigen = Onnx_infer(current_observation_);
""",
        marker="phase_step_ * double(agent_timestep)",
        mode="replace",
    ),
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor="        ++run_cnt_;\n",
        addition="        ++phase_step_;\n",
        marker="++phase_step_;",
    ),
    Edit(
        path=SDK / "run_policy/s10_policy_runner.hpp",
        anchor="        write_vector(command);\n",
        addition='        trace_file_ << ",\\\"policy_slot\\\":" << active_policy_\n'
                 '                    << ",\\\"phase_step\\\":" << phase_step_;\n',
        marker='<< phase_step_;',
    ),
]


def install_headers(root: Path) -> bool:
    changed = False
    for source, relative_destination in (
        (BRIDGE_HEADER, "interface/user_command/ros_cmd_interface.hpp"),
        (OBSTACLE_HEADER, "state_machine/quadruped_wheel/obstacle_state.hpp"),
    ):
        destination = root / SDK / relative_destination
        content = source.read_text()
        if not destination.is_file() or destination.read_text() != content:
            destination.write_text(content)
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

    if args.revert:
        subprocess.run(["git", "-C", str(root), "checkout", "--", "."], check=True)
        for header in (
            root / SDK / "interface/user_command/ros_cmd_interface.hpp",
            root / SDK / "state_machine/quadruped_wheel/obstacle_state.hpp",
        ):
            header.unlink(missing_ok=True)
        print(f"Reverted {root} to a pristine checkout")
        return 0

    if args.check:
        headers = (
            root / SDK / "interface/user_command/ros_cmd_interface.hpp",
            root / SDK / "state_machine/quadruped_wheel/obstacle_state.hpp",
        )
        header_ok = all(header.is_file() for header in headers)
        pending = [e.path for e in EDITS if not e.is_applied(root)]
        if header_ok and not pending:
            print(f"{root} is patched")
            return 0
        if not header_ok:
            print("Missing: interface/user_command/ros_cmd_interface.hpp", file=sys.stderr)
        for path in dict.fromkeys(pending):
            print(f"Unpatched: {path}", file=sys.stderr)
        return 1

    changed = install_headers(root)
    for edit in EDITS:
        changed |= edit.apply(root)

    print(f"{root} patched" if changed else f"{root} already patched")
    print("Rebuild with: scripts/build.sh")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
