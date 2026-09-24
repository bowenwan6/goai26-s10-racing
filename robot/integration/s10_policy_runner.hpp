/** S10 legacy (57/174) and HIM (342) ONNX runner.
 * Installed into the actual SDK by scripts/patch_upstream.py.
 * See docs/S10_HIM_ONNX_CONTRACT_V1.md; motor calibration stays in S10Interface.
 */
#pragma once

#include "policy_runner_base.hpp"
#include "json.hpp"
#include <array>
#include <atomic>
#include <cstdlib>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

class S10PolicyRunner : public PolicyRunnerBase {
    static constexpr int kFrame = 57, kHistory = 6, kHeightmap = 117;
    // Legacy policy order: twelve leg joints, then four wheels.
    static constexpr std::array<int, 16> kPolicyToRobot =
        {0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14, 3, 7, 11, 15};
    struct Model {
        Ort::Session session{nullptr};
        int width = kFrame;
        VecXf defaults = (VecXf(16) << 0, -.3f, .6f, 0, 0, -.3f, .6f, 0,
                                      0, .3f, -.6f, 0, 0, .3f, -.6f, 0).finished();
        VecXf kp = Vec4f(80, 80, 80, 0).replicate(4, 1);
        VecXf kd = Vec4f(2, 2, 2, .6f).replicate(4, 1);
        VecXf scale = Vec4f(.125f, .25f, .25f, 0).replicate(4, 1);
        VecXf torque_limits, dof_vel_limits;
        Vec3f commands_scale = Vec3f::Ones();
        float ang_vel = .25f, dof_pos = 1, dof_vel = .05f, vel_scale = 5;
        float clip_actions = 100, clip_observations = 100;
        bool him() const { return width == kFrame * kHistory; }
    };

    // Environment must outlive every session.
    Ort::Env env_{ORT_LOGGING_LEVEL_WARNING, "S10PolicyRunner"};
    Ort::SessionOptions options_;
    std::array<Model, 4> models_;
    int active_policy_ = 0;
    Ort::MemoryInfo memory_ = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    std::array<int64_t, 2> input_shape_{1, kFrame};
    const char* input_names_[1] = {"obs"};
    const char* output_names_[1] = {"actions"};
    VecXf observation_, last_action_ = VecXf::Zero(16);
    std::atomic<bool> reset_pending_{false};
    std::array<std::atomic<float>, kHeightmap> heightmap_{};
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr heightmap_sub_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::thread event_thread_;
    std::ofstream trace_file_;
    int trace_steps_remaining_ = 0;

    static void Require(bool ok, const std::string& message) {
        if (!ok) throw std::runtime_error("S10 policy: " + message);
    }
    static float Number(const nlohmann::json& j, const char* key) {
        const auto& value = j.at(key);
        Require(value.is_number(), std::string(key) + " must be numeric");
        const float result = value.get<float>();
        Require(std::isfinite(result), std::string(key) + " must be finite");
        return result;
    }
    static VecXf Vector(const nlohmann::json& j, const char* key, int size) {
        const auto& values = j.at(key);
        Require(values.is_array() && values.size() == size,
                std::string(key) + " has incorrect length");
        VecXf result(size);
        for (int i = 0; i < size; ++i) {
            Require(values[i].is_number(), std::string(key) + " must be numeric");
            result(i) = values[i].get<float>();
        }
        Require(result.allFinite(), std::string(key) + " must be finite");
        return result;
    }
    static void LoadHimConfig(Model& model, std::filesystem::path path) {
        path.replace_extension(".json");
        std::ifstream file(path);
        Require(file.is_open(), "missing HIM sidecar: " + path.string());
        const auto j = nlohmann::json::parse(file);
        const nlohmann::json contract = {
            {"onnx_contract_version", 1}, {"policy_type", "s10_him"},
            {"robot", "s10"}, {"interface_version", 2}, {"reset_history", "zero"},
            {"input_name", "obs"}, {"input_shape", {1, 342}},
            {"output_name", "actions"}, {"output_shape", {1, 16}},
            {"one_step_observation_dim", 57}, {"history_length", 6},
            {"history_order", "newest_first"}, {"wheel_indices", {3, 7, 11, 15}},
            {"dof_names", {"fl_hipx_joint", "fl_hipy_joint", "fl_knee_joint", "fl_wheel_joint",
                           "fr_hipx_joint", "fr_hipy_joint", "fr_knee_joint", "fr_wheel_joint",
                           "hl_hipx_joint", "hl_hipy_joint", "hl_knee_joint", "hl_wheel_joint",
                           "hr_hipx_joint", "hr_hipy_joint", "hr_knee_joint", "hr_wheel_joint"}}
        };
        for (auto it = contract.begin(); it != contract.end(); ++it)
            Require(j.at(it.key()) == it.value(), "unsupported sidecar " + it.key());
        const float dt = Number(j, "policy_dt"), sim_dt = Number(j, "sim_dt");
        Require(j.at("decimation").is_number_integer() && j.at("decimation").get<int>() > 0,
                "decimation must be a positive integer");
        Require(sim_dt > 0 && std::abs(dt - .02f) < 1e-6f &&
                std::abs(dt - sim_dt * j.at("decimation").get<int>()) < 1e-6f,
                "policy_dt must match sim_dt * decimation and the SDK's 20 ms policy step");
        Require(j.at("self_collisions") == 0 || j.at("self_collisions") == 1,
                "invalid self_collisions");
        Vector(j, "initial_position", 3);
        model.defaults = Vector(j, "default_dof_pos", 16);
        model.kp = Vector(j, "p_gains", 16);
        model.kd = Vector(j, "d_gains", 16);
        model.scale = Vector(j, "action_scale", 16);
        model.torque_limits = Vector(j, "torque_limits", 16);
        model.dof_vel_limits = Vector(j, "dof_vel_limits", 16);
        model.commands_scale = Vector(j, "commands_scale", 3);
        model.ang_vel = Number(j.at("obs_scales"), "ang_vel");
        model.dof_pos = Number(j.at("obs_scales"), "dof_pos");
        model.dof_vel = Number(j.at("obs_scales"), "dof_vel");
        model.vel_scale = Number(j, "vel_scale");
        model.clip_actions = Number(j, "clip_actions");
        model.clip_observations = Number(j, "clip_observations");
        Require((model.kp.array() >= 0).all() && (model.kd.array() >= 0).all() &&
                (model.scale.array() >= 0).all() && (model.torque_limits.array() > 0).all() &&
                (model.dof_vel_limits.array() > 0).all() && (model.commands_scale.array() > 0).all() &&
                model.ang_vel > 0 && model.dof_pos > 0 && model.dof_vel > 0 &&
                model.vel_scale > 0 && model.clip_actions > 0 && model.clip_observations > 0,
                "invalid HIM scales, gains or limits");
        for (int i = 3; i < 16; i += 4)
            Require(model.kp(i) == 0 && model.scale(i) == 0 && model.defaults(i) == 0,
                    "HIM wheels require zero Kp, position scale and default position");
    }
    void LoadModel(Model& model, const std::string& path) {
        model.session = Ort::Session(env_, path.c_str(), options_);
        auto& session = model.session;
        Require(session.GetInputCount() == 1 && session.GetOutputCount() == 1,
                path + ": expected one input and one output");
        Ort::AllocatorWithDefaultOptions allocator;
        const auto input_name = session.GetInputNameAllocated(0, allocator);
        const auto output_name = session.GetOutputNameAllocated(0, allocator);
        Require(std::string(input_name.get()) == "obs" && std::string(output_name.get()) == "actions",
                path + ": expected tensor names obs / actions");
        // Keep owning type infos alive while using their tensor views.
        const auto input_type = session.GetInputTypeInfo(0);
        const auto output_type = session.GetOutputTypeInfo(0);
        Require(input_type.GetONNXType() == ONNX_TYPE_TENSOR &&
                output_type.GetONNXType() == ONNX_TYPE_TENSOR, path + ": expected tensors");
        const auto input = input_type.GetTensorTypeAndShapeInfo();
        const auto output = output_type.GetTensorTypeAndShapeInfo();
        Require(input.GetElementType() == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT &&
                output.GetElementType() == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT,
                path + ": expected float32 tensors");
        const auto in = input.GetShape(), out = output.GetShape();
        Require(in.size() == 2 && out.size() == 2 && out[1] == 16 &&
                (in[1] == 57 || in[1] == 174 || in[1] == 342), path + ": unsupported shape");
        model.width = static_cast<int>(in[1]);
        Require(model.him() ? in[0] == 1 && out[0] == 1
                            : (in[0] == 1 || in[0] == -1) && (out[0] == 1 || out[0] == -1),
                path + ": unsupported batch (HIM requires fixed batch 1)");
        if (model.him()) LoadHimConfig(model, path);
    }
    void ClearHistory() {
        observation_.setZero(models_[active_policy_].width);
        last_action_.setZero();
    }
    void UpdatePolicySelection() {
        const char* value = std::getenv("S10_POLICY_SLOT");
        int selected = value ? std::atoi(value) : 0;
        if (selected < 0 || selected >= 4 || !models_[selected].session) selected = 0;
        if (selected == active_policy_) return;
        active_policy_ = selected;
        input_shape_[1] = models_[selected].width;
        ClearHistory();
        std::cout << "[POLICY] Active slot: " << selected << " width=" << input_shape_[1] << "\n";
    }
    void StartEvents() {
        node_ = std::make_shared<rclcpp::Node>("s10_policy_events");
        reset_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
            "/sim/reset_done", 10, [this](std_msgs::msg::Empty::SharedPtr) { RequestReset(); });
        for (auto& value : heightmap_) value.store(0);
        for (const auto& model : models_) {
            if (!model.session || model.width != kFrame + kHeightmap) continue;
            heightmap_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
                "/perception/heightmap", 10,
                [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                    if (msg->data.size() != kHeightmap) return;
                    for (int i = 0; i < kHeightmap; ++i)
                        heightmap_[i].store(msg->data[i]);
                });
            break;
        }
        executor_.add_node(node_);
        event_thread_ = std::thread([this] { executor_.spin(); });
    }
    void WriteTrace(const RobotBasicState& state, const Vec3f& command,
                    const VecXf& raw, const VecXf& decoded) {
        if (!trace_file_.is_open() || trace_steps_remaining_ <= 0) return;
        auto vector = [this](const auto& v) {
            trace_file_ << "[";
            for (int i = 0; i < v.size(); ++i) {
                if (i) trace_file_ << ",";
                trace_file_ << v(i);
            }
            trace_file_ << "]";
        };
        trace_file_ << std::setprecision(9) << "{\"slot\":" << active_policy_
                    << ",\"monotonic_s\":" << std::setprecision(17)
                    << std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count()
                    << ",\"base_omega\":";
        vector(state.base_omega);
        trace_file_ << ",\"joint_pos_robot\":"; vector(state.joint_pos);
        trace_file_ << ",\"joint_vel_robot\":"; vector(state.joint_vel);
        trace_file_ << ",\"command\":"; vector(command);
        trace_file_ << ",\"observation\":"; vector(observation_);
        trace_file_ << ",\"raw_action_policy\":"; vector(raw);
        trace_file_ << ",\"clipped_action_policy\":"; vector(last_action_);
        trace_file_ << ",\"decoded_command_robot\":"; vector(decoded);
        trace_file_ << "}\n";
        trace_file_.flush();
        --trace_steps_remaining_;
    }

public:
    S10PolicyRunner(const std::string& name, const std::string& primary,
                    const std::string& secondary = "", const std::string& down = "",
                    const std::string& speedturn = "") : PolicyRunnerBase(name) {
        SetDecimation(4); // SDK state loop = 5 ms. Training's 2.5 ms * 8 is also 20 ms.
        options_.SetIntraOpNumThreads(4);
        options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
        const std::array<std::string, 4> paths = {primary, secondary, down, speedturn};
        Require(!primary.empty(), "primary model path is empty");
        for (int i = 0; i < 4; ++i) {
            if (paths[i].empty()) continue;
            LoadModel(models_[i], paths[i]);
            std::cout << "[POLICY] Loaded slot " << i << " width=" << models_[i].width
                      << " " << paths[i] << "\n";
        }
        input_shape_[1] = models_[0].width;
        OnEnter();
        const char* trace_path = std::getenv("S10_POLICY_TRACE_PATH");
        if (trace_path && *trace_path) {
            trace_file_.open(trace_path);
            Require(trace_file_.is_open(), "cannot open trace file");
            const char* steps = std::getenv("S10_POLICY_TRACE_STEPS");
            trace_steps_remaining_ = std::max(1, steps ? std::atoi(steps) : 1);
        }
        StartEvents();
    }
    ~S10PolicyRunner() override {
        executor_.cancel();
        if (event_thread_.joinable()) event_thread_.join();
    }
    void OnEnter() override {
        run_cnt_ = 0;
        cmd_vel_input_.setZero();
        ClearHistory();
    }
    // Callbacks only set a flag. The policy worker owns every mutable inference buffer.
    void RequestReset() { reset_pending_.store(true); }
    bool IsHim() const { return models_[active_policy_].him(); }
    const VecXf& DefaultJointPositions() const { return models_[active_policy_].defaults; }
    const VecXf& Observation() const { return observation_; }

    RobotAction getRobotAction(const RobotBasicState& state, const UserCommand& uc) override {
        UpdatePolicySelection();
        if (reset_pending_.exchange(false)) {
            ClearHistory();
            std::cout << "[POLICY] Reset history and previous action\n";
        }
        const auto& m = models_[active_policy_];
        const Vec3f command(uc.forward_vel_scale, uc.side_vel_scale, uc.turnning_vel_scale);
        VecXf frame(kFrame);
        frame.head(3) = state.base_omega * m.ang_vel;
        frame.segment(3, 3) = state.base_rot_mat.inverse() * Vec3f(0, 0, -1);
        frame.segment(6, 3) = command.cwiseProduct(m.commands_scale);
        for (int i = 0; i < 16; ++i) {
            const int joint = m.him() ? i : kPolicyToRobot[i];
            frame(9 + i) = joint % 4 == 3 ? 0 : (state.joint_pos(joint) - m.defaults(joint)) * m.dof_pos;
            frame(25 + i) = state.joint_vel(joint) * m.dof_vel;
        }
        frame.tail(16) = last_action_;
        Require(frame.allFinite(), "non-finite observation");
        if (m.him()) {
            frame = frame.cwiseMax(-m.clip_observations).cwiseMin(m.clip_observations);
            observation_.tail(kFrame * (kHistory - 1)) =
                observation_.head(kFrame * (kHistory - 1)).eval();
        }
        observation_.head(kFrame) = frame;
        if (m.width == kFrame + kHeightmap)
            for (int i = 0; i < kHeightmap; ++i) observation_(kFrame + i) = heightmap_[i].load();
        Require(observation_.allFinite(), "non-finite history / heightmap");
        auto input = Ort::Value::CreateTensor<float>(memory_, observation_.data(),
            observation_.size(), input_shape_.data(), input_shape_.size());
        auto outputs = models_[active_policy_].session.Run(Ort::RunOptions{nullptr},
            input_names_, &input, 1, output_names_, 1);
        Require(outputs[0].GetTensorTypeAndShapeInfo().GetShape() == std::vector<int64_t>({1, 16}),
                "runtime output shape must be [1,16]");
        const VecXf raw = Eigen::Map<const VecXf>(outputs[0].GetTensorData<float>(), 16);
        Require(raw.allFinite(), "non-finite ONNX action");
        last_action_ = m.him() ? raw.cwiseMax(-m.clip_actions).cwiseMin(m.clip_actions).eval() : raw;
        RobotAction action;
        action.kp = m.kp;
        action.kd = m.kd;
        action.tau_ff = VecXf::Zero(16);
        action.goal_joint_pos = m.defaults;
        action.goal_joint_vel = VecXf::Zero(16);
        for (int i = 0; i < 16; ++i) {
            const int joint = m.him() ? i : kPolicyToRobot[i];
            if (joint % 4 == 3) action.goal_joint_vel(joint) = last_action_(i) * m.vel_scale;
            else action.goal_joint_pos(joint) += last_action_(i) * m.scale(joint);
        }
        Require(action.goal_joint_pos.allFinite() && action.goal_joint_vel.allFinite(),
                "non-finite decoded action");
        VecXf decoded = action.goal_joint_pos;
        for (int i = 3; i < 16; i += 4) decoded(i) = action.goal_joint_vel(i);
        WriteTrace(state, command, raw, decoded);
        ++run_cnt_;
        return action;
    }
};
