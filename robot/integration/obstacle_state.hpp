/**
 * @file obstacle_state.hpp
 * @brief IMU-limited high-obstacle manoeuvre with direct joint commands.
 */
#pragma once

#include "state_base.h"

#include <cstdlib>

namespace qw {
class ObstacleState : public StateBase {
private:
    static constexpr float kDefaultFrontFoldHip = 1.328f;
    static constexpr float kDefaultFrontFoldKnee = 2.657f;
    static constexpr float kDefaultLiftHip = 0.10f;
    static constexpr float kDefaultLiftKnee = 0.15f;
    static_assert(kDefaultFrontFoldKnee > 2.0f * kDefaultFrontFoldHip - 0.01f
                      && kDefaultFrontFoldKnee < 2.0f * kDefaultFrontFoldHip + 0.01f,
                  "front fold pose must keep the wheel below the hip");

    VecXf start_pos_;
    double last_time_ = 0.0;
    float motion_time_ = 0.0f;
    float front_extend_progress_ = 0.0f;
    int phase_ = 0;
    bool complete_ = false;
    float overload_time_ = 0.0f;

    float phase_time_ = EnvFloat("S10_OBSTACLE_PHASE_TIME", 2.5f);
    float tuck_hip_ = EnvFloat("S10_OBSTACLE_TUCK_HIP", 2.35f);
    float tuck_knee_ = EnvFloat("S10_OBSTACLE_TUCK_KNEE", 2.35f);
    float lift_hip_ = EnvFloat("S10_OBSTACLE_LIFT_HIP", kDefaultLiftHip);
    float lift_knee_ = EnvFloat("S10_OBSTACLE_LIFT_KNEE", kDefaultLiftKnee);
    float front_fold_hip_ = EnvFloat("S10_OBSTACLE_FRONT_FOLD_HIP", kDefaultFrontFoldHip);
    float front_fold_knee_ = EnvFloat("S10_OBSTACLE_FRONT_FOLD_KNEE", kDefaultFrontFoldKnee);
    float front_extend_knee_ = EnvFloat("S10_OBSTACLE_FRONT_EXTEND_KNEE", 1.90f);
    float front_extend_time_ = EnvFloat("S10_OBSTACLE_FRONT_EXTEND_TIME", 1.50f);
    float m_rear_knee_ = EnvFloat("S10_OBSTACLE_M_REAR_KNEE", 0.35f);
    float m_wheel_speed_ = EnvFloat("S10_OBSTACLE_M_WHEEL_SPEED", 8.0f);
    float wheel_speed_ = EnvFloat("S10_OBSTACLE_WHEEL_SPEED", 4.0f);
    float wheel_kd_ = EnvFloat("S10_OBSTACLE_WHEEL_KD", 2.0f);
    float rear_kp_ = EnvFloat("S10_OBSTACLE_REAR_KP", 50.0f);
    float front_press_ = EnvFloat("S10_OBSTACLE_FRONT_PRESS", 0.15f);
    float roll_limit_ = EnvFloat("S10_OBSTACLE_ROLL_LIMIT", 0.75f);
    float pitch_limit_ = EnvFloat("S10_OBSTACLE_PITCH_LIMIT", 1.75f);

    static float EnvFloat(const char* name, float fallback) {
        const char* value = std::getenv(name);
        if (!value) return fallback;
        char* end = nullptr;
        const float parsed = std::strtof(value, &end);
        return end != value && std::isfinite(parsed) && parsed > 0.0f ? parsed : fallback;
    }

    static float Blend(float value) {
        value = LimitNumber(value, 0.0f, 1.0f);
        return value * value * (3.0f - 2.0f * value);
    }

    static int RequestedPhase() {
        const char* value = std::getenv("S10_OBSTACLE_MANUAL_PHASE");
        return value ? std::clamp(std::atoi(value), 0, 5) : 0;
    }

    static bool FrontExtendRequested() {
        const char* value = std::getenv("S10_OBSTACLE_FRONT_EXTEND");
        return value && std::atoi(value) != 0;
    }

    void SetLegTarget(MatXf& command, int leg, float hip, float knee, float blend) {
        const int joint = leg * 4;
        command(joint + 1, 1) = start_pos_(joint + 1) + blend * (hip - start_pos_(joint + 1));
        command(joint + 2, 1) = start_pos_(joint + 2) + blend * (knee - start_pos_(joint + 2));
    }

    static void SetLegTargetBetween(MatXf& command, int leg, float from_hip,
                                    float from_knee, float to_hip, float to_knee,
                                    float blend) {
        const int joint = leg * 4;
        command(joint + 1, 1) = from_hip + blend * (to_hip - from_hip);
        command(joint + 2, 1) = from_knee + blend * (to_knee - from_knee);
    }

    void SetFrontPress(MatXf& command, int leg, float blend) {
        SetLegTargetBetween(command, leg, -front_fold_hip_, front_fold_knee_,
                            -front_fold_hip_ - front_press_, front_fold_knee_, blend);
    }

    void SetForwardWheels(MatXf& command, bool front, float speed) {
        const int left = front ? 3 : 11;
        const int right = front ? 7 : 15;
        command(left, 2) = wheel_kd_;
        command(right, 2) = wheel_kd_;
        command(left, 3) = -speed;
        command(right, 3) = -speed;
    }

public:
    ObstacleState(const RobotName& robot_name, const std::string& state_name,
                  std::shared_ptr<ControllerData> data_ptr)
        : StateBase(robot_name, state_name, data_ptr) {}

    void OnEnter() override {
        phase_time_ = EnvFloat("S10_OBSTACLE_PHASE_TIME", 2.5f);
        tuck_hip_ = EnvFloat("S10_OBSTACLE_TUCK_HIP", 2.35f);
        tuck_knee_ = EnvFloat("S10_OBSTACLE_TUCK_KNEE", 2.35f);
        lift_hip_ = EnvFloat("S10_OBSTACLE_LIFT_HIP", kDefaultLiftHip);
        lift_knee_ = EnvFloat("S10_OBSTACLE_LIFT_KNEE", kDefaultLiftKnee);
        front_fold_hip_ = EnvFloat("S10_OBSTACLE_FRONT_FOLD_HIP", kDefaultFrontFoldHip);
        front_fold_knee_ = EnvFloat("S10_OBSTACLE_FRONT_FOLD_KNEE", kDefaultFrontFoldKnee);
        front_extend_knee_ = EnvFloat("S10_OBSTACLE_FRONT_EXTEND_KNEE", 1.90f);
        front_extend_time_ = EnvFloat("S10_OBSTACLE_FRONT_EXTEND_TIME", 1.50f);
        m_rear_knee_ = EnvFloat("S10_OBSTACLE_M_REAR_KNEE", 0.35f);
        m_wheel_speed_ = EnvFloat("S10_OBSTACLE_M_WHEEL_SPEED", 8.0f);
        wheel_speed_ = EnvFloat("S10_OBSTACLE_WHEEL_SPEED", 4.0f);
        wheel_kd_ = EnvFloat("S10_OBSTACLE_WHEEL_KD", 2.0f);
        rear_kp_ = EnvFloat("S10_OBSTACLE_REAR_KP", 50.0f);
        front_press_ = EnvFloat("S10_OBSTACLE_FRONT_PRESS", 0.15f);
        roll_limit_ = EnvFloat("S10_OBSTACLE_ROLL_LIMIT", 0.75f);
        pitch_limit_ = EnvFloat("S10_OBSTACLE_PITCH_LIMIT", 1.75f);
        start_pos_ = ri_ptr_->GetJointPosition();
        last_time_ = ri_ptr_->GetInterfaceTimeStamp();
        motion_time_ = 0.0f;
        front_extend_progress_ = 0.0f;
        phase_ = 0;
        complete_ = false;
        overload_time_ = 0.0f;
        msfb_.UpdateCurrentState(RobotMotionState::ObstacleMode);
        std::cout << "[OBSTACLE] phase=" << phase_time_ << "s hip=" << tuck_hip_
                  << " knee=" << tuck_knee_ << " wheel=" << wheel_speed_
                  << "rad/s wheel_kd=" << wheel_kd_
                  << " rear_kp=" << rear_kp_ << " lift=(" << lift_hip_
                  << "," << -lift_knee_ << ") front_fold=(" << -front_fold_hip_
                  << "," << front_fold_knee_ << ") front_extend_knee="
                  << front_extend_knee_ << " m_rear_knee=" << m_rear_knee_
                  << " m_wheel=" << m_wheel_speed_
                  << " front_press=" << front_press_ << "rad\n";
    }

    void OnExit() override {}

    void Run() override {
        const double now = ri_ptr_->GetInterfaceTimeStamp();
        const Vec3f rpy = ri_ptr_->GetImuRpy();
        const Vec3f omega = ri_ptr_->GetImuOmega();
        const float dt = LimitNumber(static_cast<float>(now - last_time_), 0.0f, 0.01f);
        last_time_ = now;
        const VecXf torque = ri_ptr_->GetJointTorque();
        bool overloaded = false;
        for (int joint = 0; joint < std::min<int>(16, torque.size()); ++joint) {
            const float limit = joint % 4 == 3 ? 13.5f : 49.0f;
            overloaded |= std::abs(torque(joint)) > limit;
        }
        overload_time_ = overloaded ? overload_time_ + dt : 0.0f;
        if (std::abs(rpy(0)) > roll_limit_ || rpy(1) < -pitch_limit_
            || rpy(1) > 0.6f || overload_time_ > 0.4f) {
            std::cout << "[OBSTACLE] safety limit rpy=" << rpy.transpose()
                      << " overload=" << overload_time_ << "s, entering damping\n";
            complete_ = true;
            uc_ptr_->GetUserCommand()->target_mode = uint8_t(RobotMotionState::JointDamping);
            return;
        }

        const int requested_phase = RequestedPhase();
        if (requested_phase != phase_) {
            phase_ = requested_phase;
            motion_time_ = 0.0f;
            std::cout << "[OBSTACLE] manual phase " << phase_ << "\n";
        }

        motion_time_ += dt * (std::abs(omega(1)) > 1.0f ? 0.2f : 1.0f);
        front_extend_progress_ = LimitNumber(
            front_extend_progress_
                + (FrontExtendRequested() ? 1.0f : -1.0f) * dt / front_extend_time_,
            0.0f, 1.0f);
        const float front_extend = Blend(front_extend_progress_);
        const float blend = Blend(motion_time_ / phase_time_);

        MatXf command = MatXf::Zero(16, 5);
        command.col(1) = start_pos_;
        for (int joint = 0; joint < 16; ++joint) {
            if (joint % 4 != 3) {
                command(joint, 0) = 80.0f;
                command(joint, 2) = 2.0f;
            } else {
                command(joint, 2) = 0.6f;
            }
        }
        command(9, 0) = command(10, 0) = rear_kp_;
        command(13, 0) = command(14, 0) = rear_kp_;
        const float prepare = Blend(2.0f * motion_time_ / phase_time_);
        const float lateral_blend = phase_ == 0 ? prepare : 1.0f;
        for (int leg = 0; leg < 4; ++leg) {
            const int abduction = leg * 4;
            command(abduction, 1) = start_pos_(abduction) * (1.0f - lateral_blend);
        }

        if (phase_ == 0) {
            // First establish a tall, stable support pose; only then roll forward.
            const float advance = Blend(2.0f * motion_time_ / phase_time_ - 1.0f);
            SetLegTarget(command, 0, -front_fold_hip_, front_fold_knee_, prepare);
            SetLegTarget(command, 1, -front_fold_hip_, front_fold_knee_, prepare);
            SetLegTarget(command, 2, lift_hip_, -lift_knee_, prepare);
            SetLegTarget(command, 3, lift_hip_, -lift_knee_, prepare);
            SetForwardWheels(command, true, wheel_speed_ * advance);
            SetForwardWheels(command, false, wheel_speed_ * advance);
        } else if (phase_ == 1) {
            // Brace the wheels and pull with the front legs; V already extended the rear.
            SetFrontPress(command, 0, blend);
            SetFrontPress(command, 1, blend);
            SetLegTargetBetween(command, 2, lift_hip_, -lift_knee_,
                                lift_hip_, -lift_knee_, blend);
            SetLegTargetBetween(command, 3, lift_hip_, -lift_knee_,
                                lift_hip_, -lift_knee_, blend);
            SetForwardWheels(command, true, wheel_speed_);
            SetForwardWheels(command, false, wheel_speed_ * (1.0f - blend));
        } else {
            // N advances these one-leg-at-a-time transfer phases; each ends at rest.
            const float crawl = 0.25f * wheel_speed_ * 4.0f * blend * (1.0f - blend);
            if (phase_ == 5) {
                SetLegTargetBetween(command, 0, -front_fold_hip_ - front_press_,
                                    front_fold_knee_, start_pos_(1), start_pos_(2), blend);
                SetLegTargetBetween(command, 1, -front_fold_hip_ - front_press_,
                                    front_fold_knee_, start_pos_(5), start_pos_(6), blend);
            } else {
                SetFrontPress(command, 0, 1.0f);
                SetFrontPress(command, 1, 1.0f);
            }

            if (phase_ == 2) {
                SetLegTargetBetween(command, 2, lift_hip_, -lift_knee_,
                                    tuck_hip_, -tuck_knee_, blend);
                SetLegTargetBetween(command, 3, lift_hip_, -lift_knee_,
                                    lift_hip_, -lift_knee_, blend);
            } else if (phase_ == 3) {
                SetLegTargetBetween(command, 2, tuck_hip_, -tuck_knee_,
                                    start_pos_(9), start_pos_(10), blend);
                SetLegTargetBetween(command, 3, lift_hip_, -lift_knee_,
                                    lift_hip_, -lift_knee_, blend);
            } else if (phase_ == 4) {
                SetLegTargetBetween(command, 3, lift_hip_, -lift_knee_,
                                    tuck_hip_, -tuck_knee_, blend);
            } else {
                SetLegTargetBetween(command, 3, tuck_hip_, -tuck_knee_,
                                    start_pos_(13), start_pos_(14), blend);
            }
            SetForwardWheels(command, true, crawl);
            SetForwardWheels(command, false, crawl);
        }

        if (phase_ < 5 && front_extend > 0.0f) {
            command(2, 1) += front_extend * (front_extend_knee_ - command(2, 1));
            command(6, 1) += front_extend * (front_extend_knee_ - command(6, 1));
            command(10, 1) += front_extend * (m_rear_knee_ - command(10, 1));
            command(14, 1) += front_extend * (m_rear_knee_ - command(14, 1));
            const float speed = wheel_speed_ + front_extend * (m_wheel_speed_ - wheel_speed_);
            SetForwardWheels(command, true, speed);
            SetForwardWheels(command, false, speed);
        }

        ri_ptr_->SetJointCommand(command);
        if (phase_ == 5 && motion_time_ >= phase_time_) {
            complete_ = true;
            uc_ptr_->GetUserCommand()->target_mode = uint8_t(RobotMotionState::RLControlMode);
        }
    }

    bool LoseControlJudge() override {
        return uc_ptr_->GetUserCommand()->target_mode == uint8_t(RobotMotionState::JointDamping)
            || uc_ptr_->GetUserCommand()->safe_control_mode != 0;
    }

    StateName GetNextStateName() override {
        if (uc_ptr_->GetUserCommand()->target_mode == uint8_t(RobotMotionState::JointDamping)) {
            return StateName::kJointDamping;
        }
        if (uc_ptr_->GetUserCommand()->target_mode == uint8_t(RobotMotionState::RLControlMode)) {
            return StateName::kRLControl;
        }
        return complete_ ? StateName::kRLControl : StateName::kObstacle;
    }
};
}  // namespace qw
