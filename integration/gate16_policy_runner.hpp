/**
 * @file gate16_policy_runner.hpp
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
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <onnxruntime_c_api.h>
#include <onnxruntime_cxx_api.h>
#include <unistd.h>

#include "policy_runner_base.hpp"
#include "json.hpp"
#include "gate16_perception_buffer.hpp"
#include "gate16_policy_symmetry.hpp"
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
  float mirror_positive_yaw_min_deg_ = 20.0f;
  std::vector<std::pair<float, float>> mirror_yaw_bands_deg_;
  float climb_entry_heading_error_deg_ = 0.0f;
  bool mirror_policy_frame_ = false;
  bool actuator_owned_ = false;
  bool seed_action_history_on_next_tick_ = false;

  struct FrontTuckProfile {
    std::string name;
    float entry_speed_mps = 0.0f;
    float entry_yaw_deg = 0.0f;
    float speed_tolerance_mps = 0.0f;
    float yaw_tolerance_deg = 0.0f;
    float settle_forward_mps = 0.0f;
    float push_forward_mps = 0.0f;
    float front_hipy_delta = 0.0f;
    float front_knee_delta = 0.0f;
  };
  std::vector<FrontTuckProfile> command_profiles_;
  int active_profile_index_ = -1;
  int front_support_frames_ = 0;
  int settle_frames_ = 0;
  bool front_supported_ = false;
  bool climb_armed_ = false;

  static constexpr float kRadiansToDegrees = 57.29577951308232f;
  static constexpr float kFrontSupportEdgeX = 0.40f;
  static constexpr int kFrontSupportConfirmFrames = 2;
  static constexpr int kSettleFrames = 30;

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

  VecXf InferInPolicyFrame(Ort::Session& session,
                           const VecXf& physical_observation) {
    if (!mirror_policy_frame_) return Infer(session, physical_observation);
    const VecXf mirrored_observation =
        s10_policy::MirrorPolicyObservation(physical_observation);
    return s10_policy::MirrorPolicyAction(Infer(session, mirrored_observation));
  }

  bool ShouldMirrorPolicyFrame(float heading_error_deg) const {
    if (!mirror_yaw_bands_deg_.empty()) {
      return s10_policy::ShouldMirrorYawBands(heading_error_deg,
                                              mirror_yaw_bands_deg_);
    }
    return s10_policy::ShouldMirrorPositiveYaw(
        heading_error_deg, mirror_positive_yaw_min_deg_);
  }

  void UpdatePolicyFrameBeforeArm(float base_yaw_rad) {
    if (climb_armed_) return;
    const float heading_error_deg = base_yaw_rad * kRadiansToDegrees;
    const bool mirrored = ShouldMirrorPolicyFrame(heading_error_deg);
    if (mirrored != mirror_policy_frame_) {
      std::cout << "S10 adaptive-v3 pre-arm policy frame: "
                << (mirrored ? "mirrored" : "native")
                << " yaw=" << heading_error_deg << std::endl;
    }
    climb_entry_heading_error_deg_ = heading_error_deg;
    mirror_policy_frame_ = mirrored;
  }

  void SeedActionHistoryFromMeasuredState() {
    // The first twelve actions are joint-position offsets. The last four are wheel
    // velocity targets. Invert the exact decoder below so observation[41:57] describes
    // the physical state inherited from the official actor on the ownership edge.
    for (int policy_index = 0; policy_index < 12; ++policy_index) {
      const int robot_index = policy2robot_idx[policy_index];
      last_action_eigen(policy_index) =
          joint_pos_rl(policy_index) / action_scale_robot[robot_index];
    }
    for (int policy_index = 12; policy_index < action_dim; ++policy_index) {
      const int robot_index = policy2robot_idx[policy_index];
      last_action_eigen(policy_index) =
          joint_vel_rl(policy_index) /
          (dof_vel_scale_ * action_scale_robot[robot_index]);
    }
    for (int i = 0; i < action_dim; ++i) {
      last_action_eigen(i) = std::clamp(
          last_action_eigen(i), -action_guard_[i], action_guard_[i]);
    }
    seed_action_history_on_next_tick_ = false;
    std::cout << "S10 Gate16 action history seeded from measured state on actuator takeover"
              << std::endl;
  }

  void LoadDeploymentConfig(const std::filesystem::path& policy_directory) {
    const auto manifest_path = policy_directory / "climb_policy_manifest.json";
    if (!std::filesystem::exists(manifest_path)) return;

    try {
      std::ifstream manifest_stream(manifest_path);
      nlohmann::json manifest;
      manifest_stream >> manifest;

      if (manifest.contains("policy_symmetry")) {
        const auto& symmetry = manifest.at("policy_symmetry");
        std::vector<std::pair<float, float>> bands;
        for (const auto& band : symmetry.value("yaw_bands_deg", nlohmann::json::array())) {
          if (!band.is_array() || band.size() != 2) {
            throw std::runtime_error("yaw mirror band must contain two values");
          }
          bands.emplace_back(band.at(0).get<float>(), band.at(1).get<float>());
        }
        if (!bands.empty()) {
          s10_policy::ShouldMirrorYawBands(0.0f, bands);  // validates the bands
          mirror_yaw_bands_deg_ = std::move(bands);
        }
      }

      if (!manifest.contains("front_tuck_command_profile")) return;
      const auto& profile_ref = manifest.at("front_tuck_command_profile");
      if (!profile_ref.value("runner_applies_profile", false)) {
        std::cout << "S10 adaptive-v3 command profile inactive: no verified "
                     "front-wheel support-height input"
                  << std::endl;
        return;
      }
      const auto profile_name = profile_ref.at("file").get<std::string>();
      if (std::filesystem::path(profile_name).filename() != profile_name) {
        throw std::runtime_error("front-tuck profile must be bundle-local");
      }
      std::ifstream profile_stream(policy_directory / profile_name);
      nlohmann::json profile_root;
      profile_stream >> profile_root;
      const float default_speed_tolerance =
          profile_root.value("speed_tolerance_mps", 0.0f);
      const float default_yaw_tolerance =
          profile_root.value("yaw_tolerance_deg", 0.0f);
      for (const auto& item : profile_root.at("profiles")) {
        FrontTuckProfile profile;
        profile.name = item.at("name").get<std::string>();
        profile.entry_speed_mps = item.at("entry_speed_mps").get<float>();
        profile.entry_yaw_deg = item.at("entry_yaw_deg").get<float>();
        profile.speed_tolerance_mps =
            item.value("speed_tolerance_mps", default_speed_tolerance);
        profile.yaw_tolerance_deg =
            item.value("yaw_tolerance_deg", default_yaw_tolerance);
        profile.settle_forward_mps =
            item.at("settle_command_forward_mps").get<float>();
        profile.push_forward_mps =
            item.at("push_command_forward_mps").get<float>();
        profile.front_hipy_delta =
            item.value("precontact_front_hipy_action_delta", 0.0f);
        profile.front_knee_delta =
            item.value("precontact_front_knee_action_delta", 0.0f);
        if (profile.speed_tolerance_mps <= 0.0f ||
            profile.yaw_tolerance_deg <= 0.0f) {
          throw std::runtime_error("front-tuck tolerances must be positive");
        }
        command_profiles_.push_back(std::move(profile));
      }
      std::cout << "S10 adaptive-v3 climb config loaded: "
                << command_profiles_.size() << " command profiles, "
                << mirror_yaw_bands_deg_.size() << " mirror bands" << std::endl;
    } catch (const std::exception& error) {
      throw std::runtime_error("invalid S10 climb deployment config: " +
                               std::string(error.what()));
    }
  }

  int SelectCommandProfile(float speed_mps, float yaw_deg) const {
    int selected = -1;
    float best_score = std::numeric_limits<float>::infinity();
    for (int index = 0; index < static_cast<int>(command_profiles_.size()); ++index) {
      const auto& profile = command_profiles_[index];
      const float speed_error = std::abs(speed_mps - profile.entry_speed_mps);
      const float yaw_error = std::abs(yaw_deg - profile.entry_yaw_deg);
      if (speed_error > profile.speed_tolerance_mps ||
          yaw_error > profile.yaw_tolerance_deg) {
        continue;
      }
      const float score = speed_error / profile.speed_tolerance_mps +
                          yaw_error / profile.yaw_tolerance_deg;
      if (score < best_score) {
        best_score = score;
        selected = index;
      }
    }
    return selected;
  }

  void BeginClimb(const s10_policy::SkillGateReport& gate,
                  const UserCommand& command, float base_yaw_rad) {
    // This isolated runner is deployed only for the measured Gate16 lip, whose normal is
    // world +X. Use the calibrated base yaw that the router actually gated instead of a
    // five-column, 0.15 m height-map line fit. The failed full run was physically at -4 deg
    // but the fit reported -11.3 deg, selecting the wrong profile and mirror frame.
    const float heightmap_heading_deg =
        -gate.edge_heading_rad * kRadiansToDegrees;
    climb_entry_heading_error_deg_ = base_yaw_rad * kRadiansToDegrees;
    mirror_policy_frame_ = ShouldMirrorPolicyFrame(climb_entry_heading_error_deg_);
    float entry_speed_mps = std::abs(command.forward_vel_scale);
    // The follower may still have one pre-gate cruise command in flight. Clamp that
    // transient to the 0.25 m/s handoff used by the deployed climb controller.
    if (entry_speed_mps > 0.30f) entry_speed_mps = 0.25f;
    active_profile_index_ = SelectCommandProfile(
        entry_speed_mps, climb_entry_heading_error_deg_);
    front_support_frames_ = 0;
    settle_frames_ = 0;
    front_supported_ = false;
    std::cout << "S10 climb entry speed=" << entry_speed_mps
              << " yaw=" << climb_entry_heading_error_deg_
              << " heightmap_yaw=" << heightmap_heading_deg
              << " valid=" << (gate.edge_heading_valid ? "yes" : "no")
              << " samples=" << gate.edge_heading_samples
              << " peak_step=" << gate.edge_heading_peak_step
              << " policy_frame=" << (mirror_policy_frame_ ? "mirrored" : "native")
              << " profile="
              << (active_profile_index_ >= 0
                      ? command_profiles_[active_profile_index_].name
                      : std::string("frozen-default"))
              << std::endl;
  }

  void UpdateFrontSupport(const s10_policy::SkillGateReport& gate) {
    if (front_supported_) {
      ++settle_frames_;
      return;
    }
    const bool both_at_edge = gate.both_side_edges_visible &&
                              gate.left_edge_distance <= kFrontSupportEdgeX &&
                              gate.right_edge_distance <= kFrontSupportEdgeX;
    front_support_frames_ = both_at_edge ? front_support_frames_ + 1 : 0;
    if (front_support_frames_ >= kFrontSupportConfirmFrames) {
      front_supported_ = true;
      settle_frames_ = 0;
      std::cout << "S10 adaptive-v3 both-front support: edge=("
                << gate.left_edge_distance << "," << gate.right_edge_distance
                << ")" << std::endl;
    }
  }

  void EndClimb() {
    active_profile_index_ = -1;
    front_support_frames_ = 0;
    settle_frames_ = 0;
    front_supported_ = false;
    climb_entry_heading_error_deg_ = 0.0f;
    mirror_policy_frame_ = false;
  }

  Vec3f ApplyCommandProfile(const Vec3f& original,
                            bool climb_active) const {
    if (!climb_active || !front_supported_ || active_profile_index_ < 0) {
      return original;
    }
    Vec3f adjusted = original;
    const auto& profile = command_profiles_[active_profile_index_];
    adjusted(0) = settle_frames_ < kSettleFrames
                      ? profile.settle_forward_mps
                      : profile.push_forward_mps;
    return adjusted;
  }

  void ApplyPrecontactFrontTuck(VecXf& action, bool climb_active,
                                const s10_policy::SkillGateReport& gate) const {
    if (!climb_active || front_supported_ || active_profile_index_ < 0) {
      return;
    }
    const auto& profile = command_profiles_[active_profile_index_];
    const bool front_height_ready = gate.both_side_edges_visible &&
                                    gate.left_edge_distance <= kFrontSupportEdgeX + 0.15f &&
                                    gate.right_edge_distance <= kFrontSupportEdgeX + 0.15f;
    if (!front_height_ready) return;
    action(1) += profile.front_hipy_delta;
    action(2) += profile.front_knee_delta;
    action(4) += profile.front_hipy_delta;
    action(5) += profile.front_knee_delta;
  }

 public:
  Gate16PolicyRunner(const std::string& policy_name, const std::string& policy_path)
      : PolicyRunnerBase(policy_name),
        policy_path_(policy_path),
        env_(ORT_LOGGING_LEVEL_WARNING, "Gate16PolicyRunner"),
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
    LoadDeploymentConfig(std::filesystem::path(policy_path_).parent_path());
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
    climb_armed_ = false;
    actuator_owned_ = false;
    seed_action_history_on_next_tick_ = false;
    EndClimb();
  }

  /** Arm residual inference only after the router proves the moving-entry contract.

      The 174D base actor is deliberately warm during APPROACH/ALIGN. Resetting the skill
      gate on the arm edge prevents terrain observed during staging from carrying a stale
      front-tuck event into the climb. */
  void SetClimbArmed(bool armed) {
    if (armed == climb_armed_) return;
    const float staged_heading_error_deg = climb_entry_heading_error_deg_;
    const bool staged_mirror_policy_frame = mirror_policy_frame_;
    climb_armed_ = armed;
    skill_gate_.Reset();
    previous_gate_state_ = false;
    EndClimb();
    if (armed) {
      climb_entry_heading_error_deg_ = staged_heading_error_deg;
      mirror_policy_frame_ = staged_mirror_policy_frame;
      // Ownership changes on this arm edge. SetActuatorOwnership schedules replacement of
      // hypothetical shadow history from measured state before this tick is inferred.
    }
    std::cout << "S10 climb residual " << (armed ? "armed" : "disarmed")
              << " policy_frame="
              << (mirror_policy_frame_ ? "mirrored" : "native")
              << " by router" << std::endl;
  }

  void SetActuatorOwnership(bool owned) {
    if (owned && !actuator_owned_) {
      // Shadow actions were never applied. Replace their history on the first owned tick
      // with the inverse-decoded measured state before base+residual inference.
      last_action_eigen.setZero(action_dim);
      current_action_eigen.setZero(action_dim);
      tmp_action_eigen.setZero(action_dim);
      seed_action_history_on_next_tick_ = true;
    }
    actuator_owned_ = owned;
  }

  RobotAction getRobotAction(const RobotBasicState& ro, const UserCommand& uc) override {
    UpdatePolicyFrameBeforeArm(ro.base_rpy(2));
    const Vec3f base_omega = ro.base_omega * omega_scale_;
    const Vec3f projected_gravity = ro.base_rot_mat.inverse() * gravity_direction;
    const Vec3f original_command(uc.forward_vel_scale, uc.side_vel_scale,
                                 uc.turnning_vel_scale);
    for (int i = 0; i < action_dim; ++i) {
      joint_pos_rl(i) = ro.joint_pos(robot2policy_idx[i]);
      joint_vel_rl(i) = ro.joint_vel(robot2policy_idx[i]) * dof_vel_scale_;
    }
    joint_pos_rl.segment(12, 4).setZero();
    joint_pos_rl -= dof_default_eigen_policy;
    if (seed_action_history_on_next_tick_) {
      SeedActionHistoryFromMeasuredState();
    }
    current_observation_.head(57) << base_omega, projected_gravity,
        original_command,
        joint_pos_rl, joint_vel_rl, last_action_eigen;

    s10_perception::HeightSnapshot height;
    s10_policy::SkillGateReport gate;
    if (observation_dim == 174) {
      height = s10_perception::SharedHeightmap().Read();
      if (residual_available_ && climb_armed_) {
        gate = skill_gate_.Update(height);
        if (gate.active && !previous_gate_state_) {
          BeginClimb(gate, uc, ro.base_rpy(2));
        }
        if (gate.active) {
          UpdateFrontSupport(gate);
        }
        if (!gate.active && previous_gate_state_) EndClimb();
        if (gate.active != previous_gate_state_) {
          std::cout << "S10 climb gate " << (gate.active ? "entered" : "exited")
                    << " max_up_step=" << gate.max_up_step << std::endl;
          previous_gate_state_ = gate.active;
        }
      }
      current_observation_.segment(57, s10_perception::kHeightCells) =
          Eigen::Map<const Eigen::VectorXf>(height.values.data(), height.values.size());
    }

    const Vec3f command = ApplyCommandProfile(original_command, gate.active);
    current_observation_.segment<3>(6) = command;

    current_action_eigen = InferInPolicyFrame(base_session_, current_observation_);
    if (residual_available_) {
      if (gate.active) {
        const VecXf residual =
            InferInPolicyFrame(residual_session_, current_observation_);
        for (int i = 0; i < action_dim; ++i) {
          const float bounded_residual = std::clamp(residual(i), -4.0f, 4.0f);
          current_action_eigen(i) += bounded_residual * correction_scale_[i];
        }
      }
    }
    ApplyPrecontactFrontTuck(current_action_eigen, gate.active, gate);
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
