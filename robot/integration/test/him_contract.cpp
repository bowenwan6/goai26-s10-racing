#include "s10_policy_runner.hpp"
#include <cassert>

void near(float a, float b) { assert(std::abs(a - b) < 2e-5f); }

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    const std::filesystem::path dir(argv[1]);
    unsetenv("S10_POLICY_SLOT");
    for (const auto& entry : std::filesystem::directory_iterator(dir)) {
        if (entry.path().filename().string().rfind("reject_", 0) != 0 || entry.path().extension() != ".onnx") continue;
        bool rejected = false;
        try { S10PolicyRunner invalid("invalid", entry.path().string()); }
        catch (const std::exception& e) { rejected = true; std::cout << "Expected rejection: " << e.what() << "\n"; }
        assert(rejected);
    }
    {
        S10PolicyRunner runner("test", (dir / "him.onnx").string(),
            (dir / "legacy.onnx").string(), (dir / "heightmap.onnx").string(),
            (dir / "nan.onnx").string());
        RobotBasicState state;
        UserCommand command{};
        state.base_omega << 1, -2, 3;
        command.forward_vel_scale = .3f;
        command.side_vel_scale = -.4f;
        command.turnning_vel_scale = .5f;
        for (int j = 0; j < 16; ++j) {
            state.joint_pos(j) = .11f * j;
            state.joint_vel(j) = -.4f * j;
        }
        std::array<VecXf, 6> frames;
        for (int t = 0; t < 6; ++t) {
            command.forward_vel_scale = .3f + t * .1f;
            const auto action = runner.getRobotAction(state, command);
            const auto& obs = runner.Observation();
            assert(obs.size() == 342);
            near(obs(0), .25f); near(obs(1), -.5f); near(obs(2), .75f);
            near(obs(5), -1); near(obs(6), command.forward_vel_scale * 2);
            near(obs(7), -.8f); near(obs(8), .125f);
            for (int j = 0; j < 16; ++j) {
                const float def = j % 4 == 1 ? (j < 8 ? -.3f : .3f)
                    : j % 4 == 2 ? (j < 8 ? .6f : -.6f) : 0;
                const float clipped = std::clamp((j - 8) * .75f, -2.0f, 2.0f);
                near(obs(9 + j), j % 4 == 3 ? 0 : state.joint_pos(j) - def);
                near(obs(25 + j), state.joint_vel(j) * .05f);
                near(obs(41 + j), t == 0 ? 0 : clipped);
                near(action.kp(j), j % 4 == 3 ? 0 : 81);
                near(action.kd(j), j % 4 == 3 ? .7f : 2.1f);
                near(action.goal_joint_pos(j), j % 4 == 3 ? 0 : def + clipped * (j % 4 == 0 ? .125f : .25f));
                near(action.goal_joint_vel(j), j % 4 == 3 ? clipped * 5 : 0);
            }
            frames[t] = obs.head(57);
            for (int history = 0; history < 6; ++history) {
                const VecXf expected = history <= t ? frames[t-history] : VecXf::Zero(57);
                assert(obs.segment(history * 57, 57).isApprox(expected, 1e-6f));
            }
        }
        runner.RequestReset();
        runner.getRobotAction(state, command);
        assert(runner.Observation().tail(285).isZero());
        assert(runner.Observation().segment(41,16).isZero());
        runner.OnEnter();
        runner.getRobotAction(state, command);
        assert(runner.Observation().tail(285).isZero());
        setenv("S10_POLICY_SLOT", "1", 1);
        const auto legacy_action = runner.getRobotAction(state, command);
        assert(!runner.IsHim() && runner.Observation().size() == 57);
        near(runner.Observation()(6), command.forward_vel_scale);
        near(runner.Observation()(9 + 3), state.joint_pos(4));
        near(runner.Observation()(25 + 3), state.joint_vel(4) * .05f);
        assert(runner.Observation().tail(16).isZero());
        near(legacy_action.goal_joint_vel(3), (12 - 8) * .75f * 5);
        near(legacy_action.kp(0), 80);
        setenv("S10_POLICY_SLOT", "0", 1);
        runner.getRobotAction(state, command);
        assert(runner.IsHim() && runner.Observation().size() == 342);
        assert(runner.Observation().tail(285).isZero());
        assert(runner.Observation().segment(41,16).isZero());

        // A preloaded 174-dimensional secondary must subscribe even with a HIM primary.
        auto events = std::make_shared<rclcpp::Node>("him_contract_events");
        auto map_pub = events->create_publisher<std_msgs::msg::Float32MultiArray>("/perception/heightmap", 10);
        auto reset_pub = events->create_publisher<std_msgs::msg::Empty>("/sim/reset_done", 10);
        for (int i = 0; i < 200 && (!map_pub->get_subscription_count() || !reset_pub->get_subscription_count()); ++i)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        assert(map_pub->get_subscription_count() && reset_pub->get_subscription_count());
        std_msgs::msg::Float32MultiArray map;
        map.data.assign(117, .42f);
        map_pub->publish(map);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        setenv("S10_POLICY_SLOT", "2", 1);
        runner.getRobotAction(state, command);
        assert(runner.Observation().size() == 174);
        near(runner.Observation()(173), .42f);
        setenv("S10_POLICY_SLOT", "0", 1);
        runner.getRobotAction(state, command);
        runner.getRobotAction(state, command);
        reset_pub->publish(std_msgs::msg::Empty());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        runner.getRobotAction(state, command);
        assert(runner.Observation().tail(285).isZero());
        assert(runner.Observation().segment(41,16).isZero());
        command.forward_vel_scale = 1000;
        runner.getRobotAction(state, command);
        near(runner.Observation()(6), 100);
        setenv("S10_POLICY_SLOT", "3", 1);
        bool rejected = false;
        try { runner.getRobotAction(state, command); } catch (const std::exception&) { rejected = true; }
        assert(rejected);
    }
    if (std::filesystem::is_regular_file(dir / "deployment_fixture.json")) {
        std::ifstream file(dir / "deployment_fixture.json");
        const auto fixture = nlohmann::json::parse(file);
        auto vec = [](const nlohmann::json& values) -> VecXf {
            const auto data = values.get<std::vector<float>>();
            return Eigen::Map<const VecXf>(data.data(), data.size());
        };
        unsetenv("S10_POLICY_SLOT");
        S10PolicyRunner reference("reference", (dir / "reference_history.onnx").string());
        RobotBasicState state;
        UserCommand command{};
        float max_error = 0;
        for (const auto& step : fixture.at("steps")) {
            const auto& in = step.at("inputs");
            state.base_omega = vec(in.at("omega"));
            const Vec3f gravity = vec(in.at("gravity"));
            state.base_rot_mat = Eigen::Quaternionf::FromTwoVectors(gravity, Vec3f(0,0,-1)).toRotationMatrix();
            state.joint_pos = vec(in.at("q"));
            state.joint_vel = vec(in.at("dq"));
            command.forward_vel_scale = in.at("command")[0];
            command.side_vel_scale = in.at("command")[1];
            command.turnning_vel_scale = in.at("command")[2];
            reference.getRobotAction(state, command);
            const float error = (reference.Observation() - vec(step.at("expected_history"))).cwiseAbs().maxCoeff();
            max_error = std::max(max_error, error);
            assert(error < 2e-5f);
        }
        S10PolicyRunner actions("reference_actions", (dir / "reference_actions.onnx").string());
        auto decoded = actions.getRobotAction(state, command);
        assert((decoded.goal_joint_pos - vec(fixture.at("action").at("q_target"))).cwiseAbs().maxCoeff() < 2e-5f);
        assert((decoded.goal_joint_vel - vec(fixture.at("action").at("dq_target"))).cwiseAbs().maxCoeff() < 2e-5f);
        actions.getRobotAction(state, command);
        assert((actions.Observation().segment(41,16) - vec(fixture.at("action").at("clipped"))).cwiseAbs().maxCoeff() < 2e-5f);
        std::cout << "PASS: training reference seven-step history and targets; max_abs=" << max_error << "\n";
    }
    if (argc > 2) {
        unsetenv("S10_POLICY_SLOT");
        S10PolicyRunner real("real_export", argv[2]);
        RobotBasicState state;
        UserCommand command{};
        for (int step = 0; step < 6; ++step) {
            const auto action = real.getRobotAction(state, command);
            assert(action.goal_joint_pos.allFinite() && action.goal_joint_vel.allFinite());
        }
    }
    rclcpp::shutdown();
    std::cout << "PASS: rejection, HIM ordering/scales/clips, six frames, entry/reset, 57 <-> 342, 174 subscription, finite output\n";
}
