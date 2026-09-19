// No hardware interface or joint publisher: exercise the actual planner/guard with memory-only IO.
#include "standup_state.hpp"
#include "rl_control_state.hpp"
#include <cassert>

MotionStateFeedback StateBase::msfb_;
struct MemoryRobot : interface::RobotInterface {
    double now = 1800000000.0;
    VecXf q = Vec4f(0, -1.2f, 2.4f, 0).replicate(4, 1), dq = VecXf::Zero(16);
    MemoryRobot() : RobotInterface("memory_only", 16) {}
    void Start() override {}
    void Stop() override {}
    double GetInterfaceTimeStamp() override { return now; }
    VecXf GetJointPosition() override { return q; }
    VecXf GetJointVelocity() override { return dq; }
    VecXf GetJointTorque() override { return VecXf::Zero(16); }
    Vec3f GetImuRpy() override { return Vec3f::Zero(); }
    Vec3f GetImuAcc() override { return Vec3f(0, 0, 9.81f); }
    Vec3f GetImuOmega() override { return Vec3f::Zero(); }
    VecXf GetContactForce() override { return VecXf::Zero(4); }
    void SetJointCommand(Eigen::Matrix<float, Eigen::Dynamic, 5> x) override { joint_cmd_ = x; }
};
struct MemoryInput : interface::UserCommandInterface {
    MemoryInput() : UserCommandInterface(RobotName::S10) {}
    void Start() override {}
    void Stop() override {}
    UserCommand* GetUserCommand() override { return usr_cmd_; }
};

int main(int argc, char** argv) {
    assert(argc >= 2);
    setenv("S10_POLICY_PATH", argv[1], 1);
    unsetenv("S10_POLICY_SLOT");
    rclcpp::init(argc, argv);
    {
        auto data = std::make_shared<ControllerData>();
        auto robot = std::make_shared<MemoryRobot>();
        auto input = std::make_shared<MemoryInput>();
        data->ri_ptr = robot; data->uc_ptr = input;
        data->cp_ptr = std::make_shared<ControlParameters>(RobotName::S10);
        qw::StandUpState stand(RobotName::S10, "stand", data);
        qw::RLControlState rl(RobotName::S10, "rl", data);
        const VecXf expected = data->cp_ptr->policy_stand_pose_;
        assert(expected.size() == 16);
        robot->q.tail(8) *= -1;
        stand.OnEnter();
        VecXf previous = robot->q;
        for (int tick = 0; tick <= 900; ++tick) {
            robot->now = 1800000000.0 + .005 * tick;
            stand.Run();
            const MatXf x = robot->joint_cmd_;
            assert(x.allFinite());
            for (int j = 0; j < 16; ++j)
                if (j % 4 != 3) assert(std::abs(x(j, 1)-previous(j)) < .02f);
            previous = x.col(1); robot->q = previous; robot->dq = x.col(3);
        }
        assert(robot->q.isApprox(expected, 1e-5f));
        assert(robot->dq.isZero(1e-5f));

        // Regression for the observed 1.407 -> 0.707 rad knee step, before any publish.
        VecXf q = expected, dq = VecXf::Zero(16);
        q(6) = 1.4074825f;
        MatXf target = MatXf::Zero(16, 5);
        target.col(0) = Vec4f(80,80,80,0).replicate(4,1);
        target.col(2) = Vec4f(2,2,2,.6f).replicate(4,1);
        target.col(1) = expected; target(6,1) = .7070821f;
        target(3,3) = 4;
        bool rejected = false;
        try { qw::RLControlState::GuardHimCommand(target, q, dq, q); }
        catch (const std::runtime_error&) { rejected = true; }
        assert(rejected);
        const VecXf entry = q;
        previous = q;
        for (int tick = 0; tick <= 25; ++tick) {
            auto applied = qw::RLControlState::BlendHimHandover(target, entry, tick*.02f);
            if (!tick) { assert(applied.col(1).isApprox(entry)); assert(applied(3,3) == 0); }
            qw::RLControlState::GuardHimCommand(applied, q, dq, previous);
            previous = applied.col(1); q = previous;
            if (tick == 25) assert(applied.isApprox(target));
        }
        auto jump = target; jump(0,1) += .11f;
        rejected = false;
        try { qw::RLControlState::GuardHimCommand(jump, q, dq, previous); }
        catch (const std::runtime_error&) { rejected = true; }
        assert(rejected);
        std::cout << "HIM_STAND_HANDOVER_CHECK_OK\n";
    }
    rclcpp::shutdown();
}
