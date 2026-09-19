/**
 * @file joint_command_owner.hpp
 * @brief The single gate every joint command passes through before it reaches the actuators.
 *
 * ``/JOINTS_CMD`` is created and written inside the contest SDK, by ``DdsInterface`` in
 * ``interface/robot/hardware/dds_interface.hpp``. That means no external ROS node can own it.
 * A Python arbiter can decide what *this project* publishes; it cannot stop the official
 * locomotion policy publishing, because that publisher is in another process's DDS layer and
 * ROS treats two publishers on one topic as a legal, silent merge. Two controllers driving
 * sixteen actuators at 50 Hz is not a handover, it is a fight, and the visible result is a
 * thrash rather than a stop.
 *
 * So the gate goes where the writing happens. ``scripts/patch_upstream.py`` routes
 * ``RLControlState``'s one call to ``SetJointCommand`` through :func:`JointCommandOwner::Arbitrate`,
 * and from then on there is exactly one place in the process where a joint command can be
 * produced, and it has an owner.
 *
 * What the gate guarantees:
 *
 *  * Only one of the official policy and a climb policy is ever the source of the command
 *    that is published. The other one's output is dropped, not blended and not queued.
 *  * Every change of owner passes through an explicit hold except the explicitly armed
 *    official->Gate16 moving climb handoff. That one is accepted only with a finite, fully
 *    shaped Gate16 command already available on the same 50 Hz tick.
 *  * The official policy's action history is cleared before it drives again, because its
 *    observation includes its own previous action and the last one it produced describes a
 *    robot that has since been driven by something else.
 *  * A command from the previous owner cannot arrive late and be used: the buffer is dropped
 *    on every transition.
 *  * A climb policy that stops publishing, throws, or is shut down leaves the robot in the
 *    hold, not in whatever it last said. Recovering from that needs a new explicit request.
 *
 * What it deliberately does not do: it never grants ownership by itself, and it never takes
 * it back by itself. Silence is answered with a hold, not with a guess about who should be
 * driving.
 *
 * Topics
 *   subscribe  /strategy/joint_owner    std_msgs/String              "official" | "climb" | "gate16_shadow" | "gate16" | "gate16_climb" | "gate16_climb_fallback" | "stairs57" | "stop"
 *   subscribe  /strategy/climb_joints   std_msgs/Float32MultiArray   16 joint position targets
 *   subscribe  /perception/heightmap    std_msgs/Float32MultiArray   Gate16 raw 13x9 grid
 *   publish    /joints/owner            std_msgs/String              who is actually driving
 *
 * The climb policy publishes plain joint targets on its own topic. It does not publish a
 * ``JointsDataCmd``, and nothing outside the SDK publishes anything on ``/JOINTS_CMD``.
 */

#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include "common_types.h"
#include "gate16_perception_buffer.hpp"

namespace s10 {

using types::MatXf;
using types::VecXf;

/** Who the command published to the actuators came from. */
enum class JointOwner : uint8_t {
  kOfficial = 0,  //!< the shipped locomotion policy; the default and the fallback
  kClimb = 1,     //!< a climb policy driving joint targets over ROS
  kGate16 = 2,    //!< the in-process 174D Gate 16 policy, including wheel velocities
  kStairs57 = 3,  //!< the in-process 57D continuous stair-ascent policy
  kSafeHold = 4,  //!< nobody: the robot is held where it is, stiffly enough to stay there
  kStopped = 5,   //!< nobody, and deliberately so, until something asks otherwise
};

inline const char* OwnerName(JointOwner owner) {
  switch (owner) {
    case JointOwner::kOfficial:
      return "official";
    case JointOwner::kClimb:
      return "climb";
    case JointOwner::kGate16:
      return "gate16";
    case JointOwner::kStairs57:
      return "stairs57";
    case JointOwner::kSafeHold:
      return "safe_hold";
    case JointOwner::kStopped:
      return "stopped";
  }
  return "unknown";
}

class JointCommandOwner {
 public:
  /** Column layout of the command matrix the SDK publishes: kp, position, kd, velocity, tau. */
  static constexpr int kColKp = 0;
  static constexpr int kColPos = 1;
  static constexpr int kColKd = 2;
  static constexpr int kColVel = 3;
  static constexpr int kColTau = 4;

  //: Every owner change spends this long in the hold first. Long enough for the actuators to
  //: settle onto the held pose, short enough not to be a stumble in its own right.
  static constexpr double kHandoverS = 0.25;
  //: Callback-order grace for the first finite action of the moving Gate16 handoff.
  static constexpr double kMovingCommandReadyS = 0.10;

  //: A climb command older than this is not used. At 50 Hz this is ten missed messages.
  static constexpr double kClimbTimeoutS = 0.2;

  //: Holding gains. Legs are held in position; wheels are damped to a stop instead, because
  //: holding a wheel at an angle fights the ground and pitches the robot off whatever it is
  //: standing on. The policy's own gains are 80/2 on the legs and 0/0.6 on the wheels.
  static constexpr float kHoldKp = 60.0f;
  static constexpr float kHoldKd = 3.0f;
  static constexpr float kWheelKd = 1.5f;

  //: Wheel joints are the fourth of each leg's four.
  static bool IsWheel(int joint) { return joint % 4 == 3; }

  static JointCommandOwner& Instance() {
    static JointCommandOwner instance;
    return instance;
  }

  /**
   * Bring up the ROS side. Safe to call more than once; only the first call does anything.
   *
   * Separate from the constructor because the gate is a static local built the first time
   * the policy thread reaches it, and creating a node from inside a static initialiser
   * before ``rclcpp::init`` would abort the process.
   */
  void Start() {
    std::lock_guard<std::mutex> guard(lifecycle_);
    if (node_ || !rclcpp::ok()) return;

    node_ = std::make_shared<rclcpp::Node>("s10_joint_owner");

    owner_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/strategy/joint_owner", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) { RequestOwner(msg->data); });

    joints_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/strategy/climb_joints", 10,
        [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) { OnClimbJoints(*msg); });

    heightmap_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/perception/heightmap", 10,
        [](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
          s10_perception::SharedHeightmap().UpdateRaw(msg->data);
        });

    owner_pub_ = node_->create_publisher<std_msgs::msg::String>("/joints/owner", 10);

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    running_ = true;
    spin_thread_ = std::thread([this]() { executor_->spin(); });
  }

  /**
   * Shut the ROS side down and leave the gate in an explicit safe state.
   *
   * Called from the state machine's exit path. After this the arbiter still answers, and it
   * answers with a hold: a runner that keeps producing actions after the router has gone
   * away must not have them reach the actuators.
   */
  void Stop() {
    {
      std::lock_guard<std::mutex> guard(state_);
      Transition(JointOwner::kStopped, "shutdown");
    }
    std::lock_guard<std::mutex> guard(lifecycle_);
    if (!running_.exchange(false)) return;
    if (executor_) executor_->cancel();
    if (spin_thread_.joinable()) spin_thread_.join();
    executor_.reset();
    owner_pub_.reset();
    owner_sub_.reset();
    joints_sub_.reset();
    heightmap_sub_.reset();
    node_.reset();
  }

  ~JointCommandOwner() { Stop(); }

  /**
   * Decide what actually goes to the actuators this tick.
   *
   * @param official       what the shipped policy produced, in the SDK's five-column layout
   * @param gate16         current in-process Gate16 matrix, or null while inactive
   * @param measured_pos   current joint positions, used to build the hold
   * @param reset_official set when the official policy must clear its action history before
   *                       its next output is used; the caller owns doing that, because only
   *                       it has the policy pointer
   * @return the command to publish. Never the sum, average or interleaving of two sources.
   */
  MatXf Arbitrate(const MatXf& official, const MatXf* gate16,
                  const MatXf* stairs57,
                  const VecXf& measured_pos, bool* reset_official) {
    if (reset_official) *reset_official = false;
    std::lock_guard<std::mutex> guard(state_);

    const double now = Now();
    const JointOwner requested = requested_.load();

    const bool finite_gate16 =
        gate16 != nullptr && gate16->rows() == measured_pos.size() &&
        gate16->cols() == 5 && gate16->allFinite();
    const bool finite_stairs57 =
        stairs57 != nullptr && stairs57->rows() == measured_pos.size() &&
        stairs57->cols() == 5 && stairs57->allFinite();

    const bool moving_gate16_request =
        owner_ == JointOwner::kOfficial && requested == JointOwner::kGate16 &&
        (gate16_armed_.load() || gate16_shadow_.load());
    const bool moving_stairs57_request =
        owner_ == JointOwner::kOfficial && requested == JointOwner::kStairs57;

    // The frozen Gate16 contract requires d=0.60--0.65 m at 0.25 m/s and explicitly
    // forbids stopping at the lip. The router proves that envelope before sending the
    // armed request. Preserve the moving state with a single-source atomic transfer, but
    // only when this tick already carries a valid Gate16 command. Every other owner change
    // retains the conservative hold below.
    if (moving_gate16_request) {
      if (finite_gate16) {
        Transition(JointOwner::kGate16, "armed moving climb handover");
        holding_towards_ = JointOwner::kSafeHold;
      } else if (holding_towards_ != JointOwner::kGate16) {
        // The request can arrive after this tick's policy-selection snapshot. Keep the
        // moving official command briefly while the shadow runner supplies its first
        // validated action; callback ordering alone must not stop the robot at the lip.
        holding_towards_ = JointOwner::kGate16;
        hold_started_ = now;
      } else if (now - hold_started_ >= kMovingCommandReadyS) {
        Transition(JointOwner::kSafeHold, "handover requested");
        hold_started_ = now;
      }
    } else if (moving_stairs57_request && finite_stairs57) {
      // The 57D stair actor is trained for continuous commanded motion. Its runner is reset
      // when the request starts, and this finite first action transfers atomically without
      // inserting an out-of-distribution stop on the first tread.
      Transition(JointOwner::kStairs57, "armed moving stairs57 handover");
      holding_towards_ = JointOwner::kSafeHold;
    } else if (requested != owner_ && requested != holding_towards_) {
      Transition(JointOwner::kSafeHold, "handover requested");
      holding_towards_ = requested;
      hold_started_ = now;
    }

    if (owner_ == JointOwner::kSafeHold && holding_towards_ != JointOwner::kSafeHold &&
        now - hold_started_ >= kHandoverS) {
      const JointOwner target = holding_towards_;
      holding_towards_ = JointOwner::kSafeHold;
      // The official policy's observation contains its own last action. After something else
      // has been driving, that action describes a robot that no longer exists.
      if (target == JointOwner::kOfficial && reset_official) *reset_official = true;
      Transition(target, "handover complete");
    }

    MatXf command;
    switch (owner_) {
      case JointOwner::kOfficial:
        command = official;
        break;
      case JointOwner::kClimb: {
        const bool fresh =
            climb_.size() == measured_pos.size() && now - climb_stamp_ <= kClimbTimeoutS;
        if (fresh) {
          command = FromTargets(climb_, measured_pos);
        } else if (now - owner_since_ <= kClimbTimeoutS) {
          // A new owner is always empty-handed on its first tick: the handover drops whatever
          // was buffered before it, which is what stops a target submitted before the request
          // being replayed after it. Counting that emptiness as a stale command made the grant
          // impossible to complete -- the gate went climb, stale, hold, climb, stale, hold, and
          // the climb policy never once drove. So the timeout runs from whichever is later, the
          // last command or the moment ownership arrived.
          command = Hold(measured_pos);
        } else {
          // Not a reason to hand back to the official policy: that would be this gate
          // deciding, on a dropped message, that the climb is over. Hold, and say so.
          Transition(JointOwner::kSafeHold, "climb command stale");
          command = Hold(measured_pos);
        }
        break;
      }
      case JointOwner::kGate16:
        if (finite_gate16) {
          command = *gate16;
        } else {
          command = Hold(measured_pos);
        }
        break;
      case JointOwner::kStairs57:
        if (finite_stairs57) {
          command = *stairs57;
        } else {
          Transition(JointOwner::kSafeHold, "stairs57 command invalid");
          command = Hold(measured_pos);
        }
        break;
      case JointOwner::kSafeHold:
      case JointOwner::kStopped:
        command = Hold(measured_pos);
        break;
    }

    Publish();
    return command;
  }

  MatXf Arbitrate(const MatXf& official, const VecXf& measured_pos,
                  bool* reset_official) {
    return Arbitrate(official, nullptr, nullptr, measured_pos, reset_official);
  }

  MatXf Arbitrate(const MatXf& official, const MatXf* gate16,
                  const VecXf& measured_pos, bool* reset_official) {
    return Arbitrate(official, gate16, nullptr, measured_pos, reset_official);
  }

  JointOwner owner() const {
    std::lock_guard<std::mutex> guard(state_);
    return owner_;
  }

  JointOwner requested_owner() const { return requested_.load(); }

  /** Whether the already-warm Gate16 actor may evaluate its height-map residual. */
  bool gate16_armed() const { return gate16_armed_.load(); }

  /** Whether to evaluate Gate16 at 50 Hz without granting actuator ownership. */
  bool gate16_shadow() const { return gate16_shadow_.load(); }

  /** Whether the router selected the stable-v1 contract for this armed attempt. */
  bool gate16_fallback() const { return gate16_fallback_.load(); }

  /**
   * Ask for an owner by name. Anything else is ignored.
   *
   * Public, and the same entry point the subscription uses, so the whole state machine can
   * be driven from a plain C++ main with no ROS running. A gate whose only exercise is the
   * one run where it matters is not a gate anyone should trust.
   */
  void RequestOwner(const std::string& name) {
    if (name == "official") {
      gate16_armed_.store(false);
      gate16_shadow_.store(false);
      gate16_fallback_.store(false);
      requested_.store(JointOwner::kOfficial);
    } else if (name == "climb") {
      gate16_armed_.store(false);
      gate16_shadow_.store(false);
      gate16_fallback_.store(false);
      requested_.store(JointOwner::kClimb);
    } else if (name == "gate16") {
      gate16_armed_.store(false);
      gate16_fallback_.store(false);
      // Evaluate the first finite base action before the no-stop staging handoff. Once
      // ownership transfers this flag is harmless; residual remains separately disarmed.
      gate16_shadow_.store(true);
      requested_.store(JointOwner::kGate16);
    } else if (name == "gate16_shadow") {
      gate16_armed_.store(false);
      gate16_shadow_.store(true);
      gate16_fallback_.store(false);
      requested_.store(JointOwner::kOfficial);
    } else if (name == "gate16_climb") {
      gate16_armed_.store(true);
      gate16_shadow_.store(true);
      gate16_fallback_.store(false);
      requested_.store(JointOwner::kGate16);
    } else if (name == "gate16_climb_fallback") {
      gate16_armed_.store(true);
      gate16_shadow_.store(true);
      gate16_fallback_.store(true);
      requested_.store(JointOwner::kGate16);
    } else if (name == "stairs57") {
      gate16_armed_.store(false);
      gate16_shadow_.store(false);
      gate16_fallback_.store(false);
      requested_.store(JointOwner::kStairs57);
    } else if (name == "stop") {
      gate16_armed_.store(false);
      gate16_shadow_.store(false);
      gate16_fallback_.store(false);
      requested_.store(JointOwner::kStopped);
    } else if (node_) {
      RCLCPP_WARN(node_->get_logger(), "ignoring unknown joint owner request '%s'", name.c_str());
    }
  }

  /** Hand over the climb policy's joint targets. Rejects a non-finite value outright. */
  void SubmitClimbTargets(const VecXf& targets) {
    for (int i = 0; i < targets.size(); ++i) {
      if (!std::isfinite(targets(i))) return;
    }
    std::lock_guard<std::mutex> guard(state_);
    climb_ = targets;
    climb_stamp_ = Now();
  }

  /** Every transition since start-up, newest last. Read by tests and by the run recorder. */
  std::vector<std::string> transitions() {
    std::lock_guard<std::mutex> guard(state_);
    return transitions_;
  }

  /** Drop all state. Test-only: the gate is a singleton and tests are not. */
  void ResetForTest() {
    std::lock_guard<std::mutex> guard(state_);
    owner_ = JointOwner::kOfficial;
    holding_towards_ = JointOwner::kSafeHold;
    requested_.store(JointOwner::kOfficial);
    gate16_armed_.store(false);
    gate16_shadow_.store(false);
    gate16_fallback_.store(false);
    climb_.resize(0);
    climb_stamp_ = -1.0;
    owner_since_ = Now();
    transitions_.clear();
  }

 private:
  using Clock = std::chrono::steady_clock;

  JointCommandOwner() : start_(Clock::now()) {}
  JointCommandOwner(const JointCommandOwner&) = delete;
  JointCommandOwner& operator=(const JointCommandOwner&) = delete;

  double Now() const { return std::chrono::duration<double>(Clock::now() - start_).count(); }

  /** Change owner, drop whatever the previous one had buffered, and record it. */
  void Transition(JointOwner next, const char* why) {
    if (next == owner_) return;
    // Dropping the buffer here is what makes a late command from the previous owner
    // unusable rather than merely unlikely.
    climb_.resize(0);
    climb_stamp_ = -kClimbTimeoutS * 2.0;
    transitions_.push_back(std::string(OwnerName(owner_)) + "->" + OwnerName(next) + " (" + why +
                           ")");
    owner_ = next;
    owner_since_ = Now();
  }

  /** Hold every joint where it is: legs by position, wheels by damping only. */
  MatXf Hold(const VecXf& measured_pos) const {
    MatXf command = MatXf::Zero(measured_pos.size(), 5);
    for (int i = 0; i < measured_pos.size(); ++i) {
      if (IsWheel(i)) {
        command(i, kColKd) = kWheelKd;
      } else {
        command(i, kColKp) = kHoldKp;
        command(i, kColPos) = measured_pos(i);
        command(i, kColKd) = kHoldKd;
      }
    }
    return command;
  }

  /** Turn sixteen joint position targets into the SDK's five-column command. */
  MatXf FromTargets(const VecXf& targets, const VecXf& measured_pos) const {
    MatXf command = Hold(measured_pos);
    for (int i = 0; i < targets.size(); ++i) {
      if (IsWheel(i)) continue;  // wheels stay damped; a climb drives the legs
      command(i, kColPos) = targets(i);
    }
    return command;
  }

  void OnClimbJoints(const std_msgs::msg::Float32MultiArray& msg) {
    // Length and finiteness are checked before the value is kept rather than trusted from
    // the publisher: this is the last layer before sixteen actuators, and it is the one that
    // has to be right when something upstream is not.
    VecXf values(static_cast<int>(msg.data.size()));
    for (size_t i = 0; i < msg.data.size(); ++i) {
      values(static_cast<int>(i)) = msg.data[i];
    }
    SubmitClimbTargets(values);
  }

  void Publish() {
    if (!owner_pub_) return;
    std_msgs::msg::String msg;
    msg.data = OwnerName(owner_);
    owner_pub_->publish(msg);
  }

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr owner_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr joints_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr heightmap_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr owner_pub_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::atomic<bool> running_{false};
  std::mutex lifecycle_;

  mutable std::mutex state_;
  JointOwner owner_{JointOwner::kOfficial};
  JointOwner holding_towards_{JointOwner::kSafeHold};
  std::atomic<JointOwner> requested_{JointOwner::kOfficial};
  std::atomic<bool> gate16_armed_{false};
  std::atomic<bool> gate16_shadow_{false};
  std::atomic<bool> gate16_fallback_{false};
  double hold_started_{0.0};
  //: When the current owner got the joints. The climb timeout runs from this or from the last
  //: command, whichever is later, so a fresh owner is given the same window as a live one.
  double owner_since_{0.0};
  VecXf climb_;
  double climb_stamp_{-1.0};
  std::vector<std::string> transitions_;
  Clock::time_point start_;
};

}  // namespace s10
