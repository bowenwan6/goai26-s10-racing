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
  int left_front_support_frames_ = 0;
  int right_front_support_frames_ = 0;
  int settle_frames_ = 0;
  bool left_front_supported_ = false;
  bool right_front_supported_ = false;
  bool front_supported_ = false;
  bool climb_armed_ = false;
  float supported_front_hipy_delta_ = 0.0f;
  float supported_front_knee_delta_ = 0.0f;
  float supported_forward_floor_mps_ = 0.0f;
  float supported_policy_frame_yaw_bias_rps_ = 0.0f;
  float residual_engage_edge_distance_m_ =
      std::numeric_limits<float>::infinity();
  bool residual_engaged_ = false;
  s10_policy::FastAdapterEnvelope fast_adapter_envelope_;
  bool fast_adapter_active_ = true;
  bool fallback_forced_by_router_ = false;
  bool fast_adapter_allow_mirroring_ = true;
  float front_support_edge_x_ = 0.40f;
  int front_support_confirm_frames_ = 2;
  int rear_push_hold_frames_ = 180;
  bool rear_push_hold_on_each_side_ = true;
  float fallback_forward_cap_mps_ = 0.0f;

  static constexpr float kRadiansToDegrees = 57.29577951308232f;
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

      // The adaptive-v3 and v1.5 bundles intentionally share the same ONNX hashes.  The
      // manifest is therefore part of the executable policy contract, not optional tuning.
      // Missing v1.5 fields used to fall through to unsafe defaults: residual-on-immediately,
      // no fallback velocity cap and fast-adapter enabled.  Refuse that mixed deployment
      // before creating a command instead of silently changing the controller semantics.
      if (manifest.value("format", std::string()) !=
              "s10-gated-residual-front-tuck-v1.5" ||
          manifest.value("observation_dim", 0) != 174 ||
          manifest.value("action_dim", 0) != 16) {
        throw std::runtime_error("Gate16 runner requires the v1.5 174D->16D manifest");
      }
      const auto& required_runtime = manifest.at("full_stack_runtime");
      for (const char* key : {"residual_engage_edge_distance_m",
                              "front_support_edge_distance_m",
                              "front_support_confirm_policy_steps",
                              "rear_push_hold_policy_steps",
                              "fallback_max_forward_mps",
                              "confidence_gated_fast_adapter"}) {
        if (!required_runtime.contains(key)) {
          throw std::runtime_error(std::string("Gate16 full_stack_runtime missing ") + key);
        }
      }
      const auto& required_integration = manifest.at("racing_integration");
      if (required_integration.value("fallback_owner_request", std::string()) !=
              "gate16_climb_fallback" ||
          required_integration.value("competition_fast_adapter_enabled", true)) {
        throw std::runtime_error("Gate16 manifest does not fail closed to stable fallback");
      }

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

      if (!manifest.contains("front_tuck_command_profile")) {
        throw std::runtime_error("Gate16 v1.5 command profile reference is missing");
      }
      const auto& profile_ref = manifest.at("front_tuck_command_profile");
      if (!profile_ref.value("runner_applies_profile", false)) {
        throw std::runtime_error("Gate16 v1.5 runner profile is disabled");
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
      if (manifest.contains("full_stack_runtime")) {
        const auto& runtime = manifest.at("full_stack_runtime");
        const auto deltas = runtime.value(
            "supported_front_tuck_action_delta", nlohmann::json::array());
        if (deltas.is_array() && deltas.size() == 2) {
          supported_front_hipy_delta_ = deltas.at(0).get<float>();
          supported_front_knee_delta_ = deltas.at(1).get<float>();
        }
        supported_forward_floor_mps_ = runtime.value(
            "aggressive_supported_forward_floor_mps", 0.0f);
        supported_policy_frame_yaw_bias_rps_ = runtime.value(
            "supported_policy_frame_yaw_bias_rps", 0.0f);
        residual_engage_edge_distance_m_ =
            runtime.at("residual_engage_edge_distance_m").get<float>();
        front_support_edge_x_ =
            runtime.at("front_support_edge_distance_m").get<float>();
        front_support_confirm_frames_ =
            runtime.at("front_support_confirm_policy_steps").get<int>();
        rear_push_hold_frames_ =
            runtime.at("rear_push_hold_policy_steps").get<int>();
        rear_push_hold_on_each_side_ =
            runtime.value("rear_push_hold_on_each_side", true);
        fallback_forward_cap_mps_ =
            runtime.at("fallback_max_forward_mps").get<float>();
        {
          const auto& fast = runtime.at("confidence_gated_fast_adapter");
          fast_adapter_envelope_.confidence_gate_enabled =
              fast.value("enabled", false);
          fast_adapter_envelope_.min_edge_heading_samples =
              fast.value("min_edge_heading_samples", 0);
          fast_adapter_envelope_.min_edge_heading_peak_step =
              fast.value("min_edge_heading_peak_step_m", 0.0f);
          fast_adapter_envelope_.max_abs_entry_yaw_deg =
              fast.value("max_abs_entry_yaw_deg", 90.0f);
          fast_adapter_envelope_.min_entry_speed_mps =
              fast.value("min_entry_speed_mps", 0.0f);
          fast_adapter_envelope_.max_entry_speed_mps = fast.value(
              "max_entry_speed_mps", std::numeric_limits<float>::infinity());
          fast_adapter_envelope_.max_side_edge_skew_m = fast.value(
              "max_side_edge_skew_m", std::numeric_limits<float>::infinity());
          fast_adapter_allow_mirroring_ =
              fast.value("allow_policy_mirroring", true);
        }
        if (residual_engage_edge_distance_m_ <= 0.0f ||
            front_support_edge_x_ <= 0.0f ||
            front_support_confirm_frames_ < 1 || rear_push_hold_frames_ < 0 ||
            fallback_forward_cap_mps_ < 0.0f ||
            fast_adapter_envelope_.min_edge_heading_samples < 0 ||
            fast_adapter_envelope_.min_edge_heading_peak_step < 0.0f ||
            fast_adapter_envelope_.max_abs_entry_yaw_deg < 0.0f ||
            fast_adapter_envelope_.min_entry_speed_mps < 0.0f ||
            fast_adapter_envelope_.max_entry_speed_mps <
                fast_adapter_envelope_.min_entry_speed_mps ||
            fast_adapter_envelope_.max_side_edge_skew_m < 0.0f) {
          throw std::runtime_error("invalid confidence-gated fast-adapter limits");
        }
      }
      std::cout << "S10 v1.5 confidence-fallback climb config loaded: "
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
      if (speed_error > std::max(profile.speed_tolerance_mps, 0.061f) ||
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
    const float heightmap_heading_deg =
        -gate.edge_heading_rad * kRadiansToDegrees;
    const float base_heading_deg = base_yaw_rad * kRadiansToDegrees;
    float entry_speed_mps = std::abs(command.forward_vel_scale);
    if (entry_speed_mps > 0.30f) entry_speed_mps = 0.25f;

    // v1.5 uses the height-map fit only as a confidence signal.  The policy transform
    // itself keeps the calibrated base yaw that the router gated; this preserves the
    // existing fix for a noisy five-column edge fit while still failing safely to the
    // native release behavior when the perception evidence is weak.
    fast_adapter_active_ = s10_policy::ShouldUseFastAdapter(
        gate, heightmap_heading_deg, entry_speed_mps, fast_adapter_envelope_,
        fallback_forced_by_router_);
    climb_entry_heading_error_deg_ = base_heading_deg;
    mirror_policy_frame_ =
        fast_adapter_active_ && fast_adapter_allow_mirroring_ &&
        ShouldMirrorPolicyFrame(climb_entry_heading_error_deg_);
    active_profile_index_ =
        fast_adapter_active_
            ? SelectCommandProfile(entry_speed_mps,
                                   climb_entry_heading_error_deg_)
            : -1;
    left_front_support_frames_ = 0;
    right_front_support_frames_ = 0;
    settle_frames_ = 0;
    left_front_supported_ = false;
    right_front_supported_ = false;
    front_supported_ = false;
    residual_engaged_ = !fast_adapter_active_;
    std::cout << "S10 climb entry speed=" << entry_speed_mps
              << " base_yaw=" << climb_entry_heading_error_deg_
              << " heightmap_yaw=" << heightmap_heading_deg
              << " valid=" << (gate.edge_heading_valid ? "yes" : "no")
              << " samples=" << gate.edge_heading_samples
              << " peak_step=" << gate.edge_heading_peak_step
              << " policy_frame=" << (mirror_policy_frame_ ? "mirrored" : "native")
              << " fast_adapter="
              << (fast_adapter_active_ ? "yes" : "fallback")
              << " forced_by_router="
              << (fallback_forced_by_router_ ? "yes" : "no")
              << " profile="
              << (active_profile_index_ >= 0
                      ? command_profiles_[active_profile_index_].name
                      : std::string("frozen-default"))
              << std::endl;
  }

  void UpdateResidualEngagement(const s10_policy::SkillGateReport& gate) {
    if (residual_engaged_) return;
    const bool delay_until_edge =
        std::isfinite(residual_engage_edge_distance_m_);
    const bool edge_is_close =
        std::min(gate.left_edge_distance, gate.right_edge_distance) <=
        residual_engage_edge_distance_m_;
    if (!delay_until_edge || edge_is_close) {
      residual_engaged_ = true;
      std::cout << "S10 climb residual engaged: edge=("
                << gate.left_edge_distance << "," << gate.right_edge_distance
                << ") threshold=" << residual_engage_edge_distance_m_
                << std::endl;
    }
  }

  void UpdateFrontSupport(const s10_policy::SkillGateReport& gate) {
    const bool was_both_supported = front_supported_;
    const auto update_side = [this](float edge_distance, int& support_frames,
                                    bool& supported, const char* side) {
      if (supported) return;
      const bool side_at_edge =
          std::isfinite(edge_distance) && edge_distance <= front_support_edge_x_;
      support_frames = side_at_edge ? support_frames + 1 : 0;
      if (support_frames >= front_support_confirm_frames_) {
        supported = true;
        if (rear_push_hold_on_each_side_) {
          skill_gate_.HoldActiveForAdditionalFrames(rear_push_hold_frames_);
        }
        std::cout << "S10 " << side
                  << " front support event: immediate tuck edge="
                  << edge_distance << std::endl;
      }
    };
    update_side(gate.left_edge_distance, left_front_support_frames_,
                left_front_supported_, "left");
    update_side(gate.right_edge_distance, right_front_support_frames_,
                right_front_supported_, "right");
    front_supported_ = left_front_supported_ && right_front_supported_;
    if (front_supported_ && was_both_supported) {
      ++settle_frames_;
      return;
    }
    if (front_supported_) {
      settle_frames_ = 0;
      skill_gate_.HoldActiveForAdditionalFrames(rear_push_hold_frames_);
      std::cout << "S10 both-front support event: force line established"
                << std::endl;
    }
  }

  void EndClimb() {
    active_profile_index_ = -1;
    left_front_support_frames_ = 0;
    right_front_support_frames_ = 0;
    settle_frames_ = 0;
    left_front_supported_ = false;
    right_front_supported_ = false;
    front_supported_ = false;
    residual_engaged_ = false;
    fast_adapter_active_ = true;
    climb_entry_heading_error_deg_ = 0.0f;
    mirror_policy_frame_ = false;
  }

  Vec3f ApplyCommandProfile(const Vec3f& original,
                            bool climb_active) const {
    if (!climb_active) return original;
    if (!fast_adapter_active_) {
      Vec3f fallback = original;
      if (fallback_forward_cap_mps_ > 0.0f) {
        fallback(0) = std::clamp(original(0), -fallback_forward_cap_mps_,
                                 fallback_forward_cap_mps_);
      }
      return fallback;
    }
    if (!front_supported_) return original;
    Vec3f adjusted = original;
    float profiled_forward = original(0);
    if (active_profile_index_ >= 0) {
      const auto& profile = command_profiles_[active_profile_index_];
      profiled_forward = settle_frames_ < kSettleFrames
                             ? profile.settle_forward_mps
                             : profile.push_forward_mps;
    }
    adjusted(0) = std::max(profiled_forward, supported_forward_floor_mps_);
    adjusted(1) = 0.0f;
    adjusted(2) = settle_frames_ < kSettleFrames
                      ? (mirror_policy_frame_
                             ? -supported_policy_frame_yaw_bias_rps_
                             : supported_policy_frame_yaw_bias_rps_)
                      : 0.0f;
    return adjusted;
  }

  void ApplySupportedFrontTuck(VecXf& action) const {
    if ((!left_front_supported_ && !right_front_supported_) ||
        (front_supported_ && settle_frames_ >= kSettleFrames)) {
      return;
    }
    float hipy_delta = supported_front_hipy_delta_;
    float knee_delta = supported_front_knee_delta_;
    if (active_profile_index_ >= 0) {
      const auto& profile = command_profiles_[active_profile_index_];
      if (profile.front_hipy_delta != 0.0f) hipy_delta = profile.front_hipy_delta;
      if (profile.front_knee_delta != 0.0f) knee_delta = profile.front_knee_delta;
    }
    if (left_front_supported_) {
      action(1) += hipy_delta;
      action(2) += knee_delta;
    }
    if (right_front_supported_) {
      action(4) += hipy_delta;
      action(5) += knee_delta;
    }
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

  /** Bind the router's attempt-level entry contract to the low-level adapter choice.

      Height-map confidence may further reject the fast adapter, but it may never upgrade a
      router-selected stable fallback back to the fast path. */
  void SetForceFallback(bool forced) {
    if (forced == fallback_forced_by_router_) return;
    fallback_forced_by_router_ = forced;
    std::cout << "S10 climb fallback " << (forced ? "forced" : "released")
              << " by router" << std::endl;
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
          UpdateResidualEngagement(gate);
          if (fast_adapter_active_) UpdateFrontSupport(gate);
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
      if (gate.active && residual_engaged_) {
        const VecXf residual =
            InferInPolicyFrame(residual_session_, current_observation_);
        for (int i = 0; i < action_dim; ++i) {
          const float bounded_residual = std::clamp(residual(i), -4.0f, 4.0f);
          current_action_eigen(i) += bounded_residual * correction_scale_[i];
        }
      }
    }
    ApplySupportedFrontTuck(current_action_eigen);
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
