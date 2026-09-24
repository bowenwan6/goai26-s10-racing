/**
 * @file ros_cmd_interface.hpp
 * @brief Feeds the locomotion policy from ROS topics instead of a keyboard or gamepad.
 *
 * The contest SDK drives the policy's velocity command through UserCommandInterface, with
 * implementations for keyboard and gamepad input. Autonomy needs a third: one that takes
 * the command from /cmd_vel so the waypoint follower can steer the robot.
 *
 * It also automates the startup sequence. From the keyboard an operator presses keys to
 * walk the state machine from idle through stand-up into RL control; an autonomous run has
 * no operator, so the bridge advances those states itself by watching the motion-state
 * feedback and only begins forwarding velocity once the policy is actually in control.
 *
 * This file is installed into the contest SDK by scripts/patch_upstream.py. It lives here,
 * outside upstream/, so our contribution stays a single reviewable header rather than a
 * diff buried in a vendored tree.
 *
 * Topics
 *   subscribe  /cmd_vel     geometry_msgs/Twist   linear.x, linear.y, angular.z
 *   subscribe  /robot_mode  std_msgs/UInt8        manual RobotMotionState override
 *   publish    /robot_state std_msgs/UInt8        current state, for observability
 */

#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "custom_types.h"
#include "user_command_interface.h"

using namespace interface;
using namespace types;

class RosCmdInterface : public UserCommandInterface {
 public:
  /**
   * @param robot_name         robot identity forwarded to the base interface
   * @param command_timeout_s  velocity is zeroed if no command arrives within this window
   * @param autostart          drive the state machine into RL control without an operator
   */
  explicit RosCmdInterface(RobotName robot_name, double command_timeout_s = 0.5,
                           bool autostart = true)
      : UserCommandInterface(robot_name),
        command_timeout_s_(command_timeout_s),
        autostart_(autostart) {
    node_ = std::make_shared<rclcpp::Node>("s10_cmd_bridge");

    cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) { OnCommand(*msg); });

    mode_sub_ = node_->create_subscription<std_msgs::msg::UInt8>(
        "/robot_mode", 10, [this](const std_msgs::msg::UInt8::SharedPtr msg) {
          requested_mode_ = msg->data;
          manual_override_ = true;
        });

    state_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/robot_state", 10);

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);

    usr_cmd_->safe_control_mode = 0;
    requested_mode_ = static_cast<uint8_t>(RobotMotionState::WaitingForStand);
    start_time_ = Clock::now();
  }

  ~RosCmdInterface() override { Stop(); }

  void Start() override {
    if (running_.exchange(true)) return;
    spin_thread_ = std::thread([this]() { executor_->spin(); });
  }

  void Stop() override {
    if (!running_.exchange(false)) return;
    executor_->cancel();
    if (spin_thread_.joinable()) spin_thread_.join();
  }

  UserCommand* GetUserCommand() override {
    const uint8_t current_state = msfb_ ? msfb_->GetCurrentState() : 0;
    state_pub_->publish(MakeUInt8(current_state));

    if (autostart_ && !manual_override_) AdvanceStartup(current_state);
    usr_cmd_->target_mode = requested_mode_;

    // Hold still unless the policy is actually driving. Forwarding velocity during
    // stand-up would fight the stand-up controller.
    const bool policy_active = current_state == static_cast<uint8_t>(StateName::kRLControl);
    const double age =
        std::chrono::duration<double>(Clock::now() - last_command_time_).count();

    if (!policy_active || age > command_timeout_s_) {
      usr_cmd_->forward_vel_scale = 0.0f;
      usr_cmd_->side_vel_scale = 0.0f;
      usr_cmd_->turnning_vel_scale = 0.0f;
    } else {
      usr_cmd_->forward_vel_scale = forward_.load();
      usr_cmd_->side_vel_scale = lateral_.load();
      usr_cmd_->turnning_vel_scale = yaw_rate_.load();
    }

    return usr_cmd_;
  }

 private:
  using Clock = std::chrono::steady_clock;

  //: Settling time before requesting stand-up, so the simulator and DDS are both up.
  static constexpr double kStartupDelayS = 1.0;
  //: Settling time after standing before handing over to the policy.
  static constexpr double kStandSettleS = 1.5;

  void OnCommand(const geometry_msgs::msg::Twist& msg) {
    forward_.store(static_cast<float>(msg.linear.x));
    lateral_.store(static_cast<float>(msg.linear.y));
    yaw_rate_.store(static_cast<float>(msg.angular.z));
    usr_cmd_->time_stamp =
        std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
    last_command_time_ = Clock::now();
  }

  /** Walk idle -> stand-up -> RL control, waiting for each state to be reached. */
  void AdvanceStartup(uint8_t current_state) {
    const double elapsed = std::chrono::duration<double>(Clock::now() - start_time_).count();

    if (current_state == static_cast<uint8_t>(StateName::kRLControl)) {
      requested_mode_ = static_cast<uint8_t>(RobotMotionState::RLControlMode);
      return;
    }

    if (current_state == static_cast<uint8_t>(StateName::kStandUp)) {
      if (stand_reached_time_ == Clock::time_point{}) stand_reached_time_ = Clock::now();
      const double standing =
          std::chrono::duration<double>(Clock::now() - stand_reached_time_).count();
      if (standing >= kStandSettleS) {
        requested_mode_ = static_cast<uint8_t>(RobotMotionState::RLControlMode);
      }
      return;
    }

    if (elapsed >= kStartupDelayS) {
      requested_mode_ = static_cast<uint8_t>(RobotMotionState::StandingUp);
    }
  }

  static std_msgs::msg::UInt8 MakeUInt8(uint8_t value) {
    std_msgs::msg::UInt8 msg;
    msg.data = value;
    return msg;
  }

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr mode_sub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_pub_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;

  std::thread spin_thread_;
  std::atomic<bool> running_{false};

  std::atomic<float> forward_{0.0f};
  std::atomic<float> lateral_{0.0f};
  std::atomic<float> yaw_rate_{0.0f};

  double command_timeout_s_;
  bool autostart_;
  std::atomic<bool> manual_override_{false};
  std::atomic<uint8_t> requested_mode_{0};

  Clock::time_point start_time_{};
  Clock::time_point stand_reached_time_{};
  Clock::time_point last_command_time_{};
};
