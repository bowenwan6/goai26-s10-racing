/**
 * @file s10_policy_runner.hpp
 * @brief Official S10 actor plus an optional, height-map-gated climbing residual.
 *
 * ``policy.onnx`` is always the 57-D or 174-D base actor.  If a sibling file named
 * ``climb_residual.onnx`` exists, both graphs must be 174-D -> 16-D and the residual is
 * added only while HeightmapSkillGate is active.  This preserves normal locomotion and
 * matches the residual composition used during MuJoCo training.
 */

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <onnxruntime_c_api.h>
#include <onnxruntime_cxx_api.h>
#include <unistd.h>

#include "policy_runner_base.hpp"
#include "gate16_perception_buffer.hpp"
#include "gate16_skill_gate.hpp"

class Gate16PolicyRunner : public PolicyRunnerBase {
 private:
  VecXf kp_, kd_;
  VecXf dof_default_eigen_policy, dof_default_eigen_robot;
  Vec3f gravity_direction = Vec3f(0.0, 0.0, -1.0);
  VecXf dof_pos_default_;
  timespec system_time;

  static constexpr int motor_num = 16;
  static constexpr int action_dim = 16;
  int observation_dim = 57;

  VecXf joint_pos_rl = VecXf(action_dim);
  VecXf joint_vel_rl = VecXf(action_dim);

  const std::string policy_path_;
  float omega_scale_ = 0.25f;
  float dof_vel_scale_ = 0.05f;
  VecXf motor_p_eigen, motor_v_eigen, current_action_eigen, last_action_eigen,
      current_observation_, tmp_action_eigen;

  RobotAction robot_action;
  std::vector<std::string> robot_order = {
      "fl_hipx_joint", "fl_hipy_joint", "fl_knee_joint", "fl_wheel_joint",
      "fr_hipx_joint", "fr_hipy_joint", "fr_knee_joint", "fr_wheel_joint",
      "hl_hipx_joint", "hl_hipy_joint", "hl_knee_joint", "hl_wheel_joint",
      "hr_hipx_joint", "hr_hipy_joint", "hr_knee_joint", "hr_wheel_joint"};
  std::vector<std::string> policy_order = {
      "fl_hipx_joint",  "fl_hipy_joint",  "fl_knee_joint",
      "fr_hipx_joint",  "fr_hipy_joint",  "fr_knee_joint",
      "hl_hipx_joint",  "hl_hipy_joint",  "hl_knee_joint",
      "hr_hipx_joint",  "hr_hipy_joint",  "hr_knee_joint",
      "fl_wheel_joint", "fr_wheel_joint", "hl_wheel_joint", "hr_wheel_joint"};
  std::vector<float> action_scale_robot = {0.125f, 0.25f, 0.25f, 5.0f,
                                            0.125f, 0.25f, 0.25f, 5.0f,
                                            0.125f, 0.25f, 0.25f, 5.0f,
                                            0.125f, 0.25f, 0.25f, 5.0f};

  const std::array<float, action_dim> correction_scale_ = {
      1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
      1.0f, 1.0f, 1.0f, 1.0f, 6.0f, 6.0f, 6.0f, 6.0f};
  const std::array<float, action_dim> action_guard_ = {
      8.0f, 8.0f, 8.0f, 8.0f, 8.0f, 8.0f, 8.0f, 8.0f,
      8.0f, 8.0f, 8.0f, 8.0f, 25.0f, 25.0f, 25.0f, 25.0f};

  Ort::SessionOptions session_options_;
  Ort::Session base_session_{nullptr};
  Ort::Session residual_session_{nullptr};
  bool residual_available_ = false;
  Ort::Env env_;
  Ort::MemoryInfo memory_info{nullptr};
  std::array<int64_t, 2> input_shape_ = {1, 57};
  std::vector<int> robot2policy_idx, policy2robot_idx;
  const char* input_names_[1] = {"obs"};
  const char* output_names_[1] = {"actions"};
  s10_policy::HeightmapSkillGate skill_gate_;
  bool previous_gate_state_ = false;

  static int ValidateSession(Ort::Session& session, const std::string& label) {
    const auto input_shape =
        session.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    const auto output_shape =
        session.GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    if (input_shape.size() != 2 ||
        (input_shape.back() != 57 && input_shape.back() != 174)) {
      throw std::runtime_error(label + " input must be [batch,57] or [batch,174]");
    }
    if (output_shape.size() != 2 || output_shape.back() != action_dim) {
      throw std::runtime_error(label + " output must be [batch,16]");
    }
    return static_cast<int>(input_shape.back());
  }

  VecXf Infer(Ort::Session& session, const VecXf& observation) {
    Ort::Value input = Ort::Value::CreateTensor<float>(
        memory_info, const_cast<float*>(observation.data()), observation.size(),
        input_shape_.data(), input_shape_.size());
    auto outputs = session.Run(Ort::RunOptions{nullptr}, input_names_, &input, 1,
                               output_names_, 1);
    const float* action_data = outputs[0].GetTensorData<float>();
    return VecXf(Eigen::Map<const Eigen::VectorXf>(action_data, action_dim));
  }

 public:
  Gate16PolicyRunner(const std::string& policy_name, const std::string& policy_path)
      : PolicyRunnerBase(policy_name),
        policy_path_(policy_path),
        env_(ORT_LOGGING_LEVEL_WARNING, "S10PolicyRunner"),
        session_options_{},
        memory_info(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator,
                                                OrtMemTypeDefault)) {
    dof_default_eigen_policy.setZero(action_dim);
    dof_default_eigen_robot.setZero(action_dim);
    dof_default_eigen_policy << 0.0, -0.3, 0.6, 0.0, -0.3, 0.6, 0.0, 0.3,
        -0.6, 0.0, 0.3, -0.6, 0.0, 0.0, 0.0, 0.0;
    dof_default_eigen_robot << 0.0, -0.3, 0.6, 0.0, 0.0, -0.3, 0.6, 0.0,
        0.0, 0.3, -0.6, 0.0, 0.0, 0.3, -0.6, 0.0;
    SetDecimation(4);
    session_options_.SetIntraOpNumThreads(4);
    session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

    if (access(policy_path_.c_str(), F_OK) != 0) {
      throw std::runtime_error("S10 base model missing: " + policy_path_);
    }
    base_session_ = Ort::Session(env_, policy_path_.c_str(), session_options_);
    observation_dim = ValidateSession(base_session_, "S10 base policy");
    input_shape_[1] = observation_dim;

    const auto residual_path =
        std::filesystem::path(policy_path_).parent_path() / "climb_residual.onnx";
    if (std::filesystem::exists(residual_path)) {
      residual_session_ =
          Ort::Session(env_, residual_path.string().c_str(), session_options_);
      const int residual_width = ValidateSession(residual_session_, "S10 climb residual");
      if (observation_dim != 174 || residual_width != 174) {
        throw std::runtime_error(
            "gated climb deployment requires both ONNX models to use 174 observations");
      }
      residual_available_ = true;
      std::cout << "S10 climb residual enabled: " << residual_path << std::endl;
    } else {
      std::cout << "S10 base policy only; no climb_residual.onnx" << std::endl;
    }
    std::cout << "S10 policy observation width: " << observation_dim << std::endl;

    kp_ = Vec4f(80, 80, 80, 0).replicate(4, 1);
    kd_ = Vec4f(2, 2, 2, 0.6).replicate(4, 1);
    robot2policy_idx = generate_permutation(robot_order, policy_order);
    policy2robot_idx = generate_permutation(policy_order, robot_order);
    robot_action.kp = kp_;
    robot_action.kd = kd_;
    robot_action.tau_ff = VecXf::Zero(motor_num);
    robot_action.goal_joint_pos = VecXf::Zero(motor_num);
    robot_action.goal_joint_vel = VecXf::Zero(motor_num);
    current_observation_.setZero(observation_dim);
    last_action_eigen.setZero(action_dim);
    tmp_action_eigen.setZero(action_dim);
    current_action_eigen.setZero(action_dim);
  }

  ~Gate16PolicyRunner() override = default;

  std::vector<int> generate_permutation(const std::vector<std::string>& from,
                                        const std::vector<std::string>& to,
                                        int default_index = 0) {
    std::unordered_map<std::string, int> index;
    for (int i = 0; i < static_cast<int>(from.size()); ++i) index[from[i]] = i;
    std::vector<int> result;
    for (const auto& name : to) {
      const auto found = index.find(name);
      result.push_back(found == index.end() ? default_index : found->second);
    }
    return result;
  }

  void DisplayPolicyInfo() override {}

  void OnEnter() override {
    run_cnt_ = 0;
    cmd_vel_input_.setZero();
    last_action_eigen.setZero(action_dim);
    tmp_action_eigen.setZero(action_dim);
    motor_p_eigen.setZero(12);
    motor_v_eigen.setZero(motor_num);
    skill_gate_.Reset();
    previous_gate_state_ = false;
  }

  RobotAction getRobotAction(const RobotBasicState& ro, const UserCommand& uc) override {
    const Vec3f base_omega = ro.base_omega * omega_scale_;
    const Vec3f projected_gravity = ro.base_rot_mat.inverse() * gravity_direction;
    const Vec3f command(uc.forward_vel_scale, uc.side_vel_scale,
                        uc.turnning_vel_scale);
    for (int i = 0; i < action_dim; ++i) {
      joint_pos_rl(i) = ro.joint_pos(robot2policy_idx[i]);
      joint_vel_rl(i) = ro.joint_vel(robot2policy_idx[i]) * dof_vel_scale_;
    }
    joint_pos_rl.segment(12, 4).setZero();
    joint_pos_rl -= dof_default_eigen_policy;
    current_observation_.head(57) << base_omega, projected_gravity, command,
        joint_pos_rl, joint_vel_rl, last_action_eigen;

    s10_perception::HeightSnapshot height;
    if (observation_dim == 174) {
      height = s10_perception::SharedHeightmap().Read();
      current_observation_.segment(57, s10_perception::kHeightCells) =
          Eigen::Map<const Eigen::VectorXf>(height.values.data(), height.values.size());
    }

    current_action_eigen = Infer(base_session_, current_observation_);
    if (residual_available_) {
      const auto gate = skill_gate_.Update(height);
      if (gate.active != previous_gate_state_) {
        std::cout << "S10 climb gate " << (gate.active ? "entered" : "exited")
                  << " max_up_step=" << gate.max_up_step << std::endl;
        previous_gate_state_ = gate.active;
      }
      if (gate.active) {
        const VecXf residual = Infer(residual_session_, current_observation_);
        for (int i = 0; i < action_dim; ++i) {
          const float bounded_residual = std::clamp(residual(i), -4.0f, 4.0f);
          current_action_eigen(i) += bounded_residual * correction_scale_[i];
        }
      }
    }
    for (int i = 0; i < action_dim; ++i) {
      current_action_eigen(i) = std::clamp(
          current_action_eigen(i), -action_guard_[i], action_guard_[i]);
    }
    last_action_eigen = current_action_eigen;

    for (int i = 0; i < action_dim; ++i) {
      tmp_action_eigen(i) =
          current_action_eigen(policy2robot_idx[i]) * action_scale_robot[i];
    }
    tmp_action_eigen += dof_default_eigen_robot;
    for (int leg = 0; leg < 4; ++leg) {
      robot_action.goal_joint_pos.segment(leg * 4, 3) =
          tmp_action_eigen.segment(leg * 4, 3);
      robot_action.goal_joint_vel(leg * 4 + 3) = tmp_action_eigen(leg * 4 + 3);
    }
    ++run_cnt_;
    return robot_action;
  }

  void setDefaultJointPos(const VecXf& pos) {
    dof_pos_default_.setZero(motor_num);
    for (int i = 0; i < motor_num; ++i) dof_pos_default_(i) = pos(i);
  }

  double getCurrentTime() {
    clock_gettime(1, &system_time);
    return system_time.tv_sec + system_time.tv_nsec / 1e9;
  }
};
