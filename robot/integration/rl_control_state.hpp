/**
 * @file rl_control_state.hpp
 * @brief rl policy runnning state for quadruped-wheel robot
 * @author DeepRobotics
 * @version 1.0
 * @date 2025-11-07
 *
 * @copyright Copyright (c) 2025  DeepRobotics
 *
 */
#pragma once
#include "state_base.h"
#include "policy_runner_base.hpp"
#include "s10_policy_runner.hpp"
#include "robot_interface.h"
#include "user_command_interface.h"
#include "json.hpp"
#include "basic_function.hpp"
#include <cstdlib>
#include <mutex>

namespace qw {
    class RLControlState : public StateBase {
    private:
        RobotBasicState rbs_[2];
        std::mutex observation_mutex_;
        std::atomic<int> rbs_write_index_{0};
        int getrbsReadIndex() const { return 1 - rbs_write_index_.load(std::memory_order_acquire); }

        std::atomic<int> state_run_cnt_{-1};

        std::shared_ptr<PolicyRunnerBase> policy_ptr_;
        std::shared_ptr<S10PolicyRunner> s10_policy_;

        std::thread run_policy_thread_;
        std::atomic<bool> start_flag_{false};
        std::atomic<bool> policy_failed_{false};
        VecXf entry_joint_pos_, previous_target_;
        int handover_step_ = 0;

        float policy_cost_time_ = 1;
        Eigen::MatrixXf acc_rot = Eigen::MatrixXf::Zero(20, 3);
        int acc_rot_count = 0;

        void UpdateRobotObservation() {
            std::lock_guard<std::mutex> lock(observation_mutex_);
            int write_idx = rbs_write_index_.load(std::memory_order_relaxed);
            RobotBasicState& buffer = rbs_[write_idx];

            buffer.base_rpy = ri_ptr_->GetImuRpy();
            buffer.base_rot_mat = RpyToRm(buffer.base_rpy);
            buffer.base_omega = ri_ptr_->GetImuOmega();
            buffer.base_acc = ri_ptr_->GetImuAcc();
            buffer.joint_pos = ri_ptr_->GetJointPosition();
            buffer.joint_vel = ri_ptr_->GetJointVelocity();
            buffer.joint_tau = ri_ptr_->GetJointTorque();

            // 储存
            buffer.flt_base_acc_mat.row(acc_rot_count) = buffer.base_acc.transpose();
            acc_rot_count += 1;
            acc_rot_count = acc_rot_count % 20;

            rbs_write_index_.store(1 - write_idx,  std::memory_order_release);
        }

        void PolicyRunner() {
            int run_cnt_record = -1;
            while (start_flag_) {
                const int step = state_run_cnt_.load();
                if (step >= 0 && step % policy_ptr_->decimation_ == 0 && step != run_cnt_record) {
                    timespec start_timestamp, end_timestamp;
                    clock_gettime(CLOCK_MONOTONIC, &start_timestamp);
                    RobotBasicState state;
                    {
                        std::lock_guard<std::mutex> lock(observation_mutex_);
                        state = rbs_[getrbsReadIndex()];
                    }
                    try {
                    auto ra = policy_ptr_->getRobotAction(state, *(uc_ptr_->GetUserCommand()));

                    MatXf res = ra.ConvertToMat();
                    if (s10_policy_->IsHim()) {
                        res = BlendHimHandover(res, entry_joint_pos_, handover_step_ * .02f);
                        GuardHimCommand(res, state.joint_pos, state.joint_vel, previous_target_);
                        previous_target_ = res.col(1);
                        ++handover_step_;
                    }

                    ri_ptr_->SetJointCommand(res);
                    } catch (const std::exception& error) {
                        std::cerr << "[POLICY] " << error.what() << "; entering damping\n";
                        MatXf damping = MatXf::Zero(16, 5);
                        damping.col(2).setConstant(2.0f);
                        ri_ptr_->SetJointCommand(damping);
                        policy_failed_ = true;
                        break;
                    }
                    run_cnt_record = step;
                    clock_gettime(CLOCK_MONOTONIC, &end_timestamp);
                    policy_cost_time_ = (end_timestamp.tv_sec - start_timestamp.tv_sec) * 1e3
                                        + (end_timestamp.tv_nsec - start_timestamp.tv_nsec) / 1e6;

                }
                std::this_thread::sleep_for(std::chrono::microseconds(100));
            }
        }

    public:
        static MatXf BlendHimHandover(MatXf command, const VecXf& entry, float seconds) {
            // Entry ramp is outside the network; raw-action history stays unchanged.
            float alpha = std::clamp(seconds / .5f, 0.0f, 1.0f);
            alpha = alpha * alpha * (3.0f - 2.0f * alpha);
            for (int i = 0; i < 16; ++i) {
                if (i % 4 == 3) command(i, 3) *= alpha;
                else command(i, 1) = entry(i) + alpha * (command(i, 1) - entry(i));
            }
            return command;
        }

        static void GuardHimCommand(const MatXf& command, const VecXf& q,
                                    const VecXf& dq, const VecXf& previous) {
            if (!command.allFinite() || !q.allFinite() || !dq.allFinite())
                throw std::runtime_error("non-finite HIM motor command/state");
            for (int i = 0; i < 16; ++i) {
                const bool wheel = i % 4 == 3;
                const float torque = command(i, 0) * (command(i, 1) - q(i))
                    + command(i, 2) * (command(i, 3) - dq(i)) + command(i, 4);
                // Same diagnostic ceilings as the bounded hardware supervisor.
                if (std::abs(torque) > (wheel ? 12.0f : 45.0f))
                    throw std::runtime_error("HIM predicted torque exceeds test bound at joint " +
                        std::to_string(i) + ": " + std::to_string(torque));
                if (!wheel && std::abs(command(i, 1) - previous(i)) > .1f)
                    throw std::runtime_error("HIM target jumps over 0.1 rad at joint " + std::to_string(i));
            }
        }

        RLControlState(const RobotName &robot_name, const std::string &state_name,
                       std::shared_ptr<ControllerData> data_ptr) : StateBase(robot_name, state_name, data_ptr) {
            if (robot_name_ == RobotName::S10) {
                namespace fs = std::filesystem;
                fs::path base = fs::path(__FILE__).parent_path();
                const char* override_path = std::getenv("S10_POLICY_PATH");
                auto model_path = fs::canonical(
                    override_path ? fs::path(override_path)
                                  : base / ".." / ".." / "policy" / "policy.onnx");
                const char* secondary_path = std::getenv("S10_SECOND_POLICY_PATH");
                const auto secondary_model_path = secondary_path && *secondary_path
                    ? fs::canonical(fs::path(secondary_path)).string()
                    : std::string();
                const char* down_path = std::getenv("S10_DOWN_POLICY_PATH");
                const auto down_model_path = down_path && *down_path
                    ? fs::canonical(fs::path(down_path)).string()
                    : std::string();
                const char* speedturn_path = std::getenv("S10_SPEEDTURN_POLICY_PATH");
                const auto speedturn_model_path = speedturn_path && *speedturn_path
                    ? fs::canonical(fs::path(speedturn_path)).string()
                    : std::string();
                s10_policy_ = std::make_shared<S10PolicyRunner>(
                    "s10_policy", model_path.string(),
                    secondary_model_path, down_model_path, speedturn_model_path);
            }

            policy_ptr_ = s10_policy_;
            if (s10_policy_ && s10_policy_->IsHim()) {
                cp_ptr_->policy_stand_pose_ = s10_policy_->DefaultJointPositions();
                for (int i = 0; i < 16; ++i) {
                    if (i % 4 == 3) continue;
                    const float q = cp_ptr_->policy_stand_pose_(i);
                    if (q < cp_ptr_->fl_joint_lower_(i % 4) || q > cp_ptr_->fl_joint_upper_(i % 4))
                        throw std::runtime_error("HIM stand pose exceeds SDK joint limits");
                }
            }
            if (!policy_ptr_) {
                std::cerr << "error policy" << std::endl;
                exit(0);
            }
            policy_ptr_->DisplayPolicyInfo();
        }

        ~RLControlState() {}

        virtual void OnEnter() {
            state_run_cnt_ = -1;
            policy_failed_ = false;
            policy_ptr_->OnEnter();
            UpdateRobotObservation();
            entry_joint_pos_ = rbs_[getrbsReadIndex()].joint_pos;
            previous_target_ = entry_joint_pos_;
            handover_step_ = 0;
            start_flag_ = true;
            run_policy_thread_ = std::thread(std::bind(&RLControlState::PolicyRunner, this));
            StateBase::msfb_.UpdateCurrentState(RobotMotionState::RLControlMode);
        };

        virtual void OnExit() {
            start_flag_ = false;
            if (run_policy_thread_.joinable()) run_policy_thread_.join();
            state_run_cnt_ = -1;
        }

        virtual void Run() {
            UpdateRobotObservation();
            state_run_cnt_++;
        }

        virtual bool LoseControlJudge() {
            if (policy_failed_) return true;
            if (uc_ptr_->GetUserCommand()->target_mode == uint8_t(RobotMotionState::JointDamping)) return true;
            return PostureUnsafeCheck();
        }

        bool PostureUnsafeCheck() {
            // Vec3f rpy = ri_ptr_->GetImuRpy();
            // if(rpy(0) > 30./180*M_PI || rpy(1) > 45./180*M_PI){
            //     std::cout << "posture value: " << 180./M_PI*rpy.transpose() << std::endl;
            //     return true;
            // }
            return false;
        }

        virtual StateName GetNextStateName() {
            if (uc_ptr_->GetUserCommand()->safe_control_mode != 0)
                return StateName::kJointDamping;
            if (uc_ptr_->GetUserCommand()->target_mode == uint8_t(RobotMotionState::LieDown))
                return StateName::kLieDown;

            return StateName::kRLControl;
        }
    };
};
