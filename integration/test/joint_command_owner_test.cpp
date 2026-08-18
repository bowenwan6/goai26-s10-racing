/**
 * @file joint_command_owner_test.cpp
 * @brief Drives the /JOINTS_CMD ownership gate through every transition it has.
 *
 * A plain executable rather than a test framework, because the repository has no C++ test
 * infrastructure and adding one for a single header would be the larger change. It is
 * compiled and run by src/s10_auto_nav/test/test_joint_owner.py, which is where the result
 * appears; each check prints a line and a failure sets the exit status.
 *
 * Nothing here starts ROS. The gate's ROS side is three subscriptions and a publisher that
 * call the same public methods used below, so the state machine -- which is the part with a
 * safety property attached to it -- is exercised in full without a daemon.
 */

#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <thread>

#include "joint_command_owner.hpp"

namespace {

int failures = 0;

void Check(bool condition, const std::string& what) {
  std::printf("%s %s\n", condition ? "ok  " : "FAIL", what.c_str());
  if (!condition) ++failures;
}

/** The command layout the SDK publishes: sixteen rows of kp, position, kd, velocity, tau. */
types::MatXf OfficialAction(float position) {
  types::MatXf action = types::MatXf::Zero(16, 5);
  for (int i = 0; i < 16; ++i) {
    action(i, s10::JointCommandOwner::kColKp) = 80.0f;
    action(i, s10::JointCommandOwner::kColPos) = position;
    action(i, s10::JointCommandOwner::kColKd) = 2.0f;
  }
  return action;
}

types::VecXf Measured(float position) { return types::VecXf::Constant(16, position); }

/** Tick the gate until it settles, the way the 50 Hz policy thread does. */
types::MatXf Settle(s10::JointCommandOwner& gate, double seconds, float official_pos,
                    float measured_pos, bool* saw_reset = nullptr) {
  types::MatXf out;
  const int ticks = static_cast<int>(seconds / 0.02) + 1;
  for (int i = 0; i < ticks; ++i) {
    bool reset = false;
    out = gate.Arbitrate(OfficialAction(official_pos), Measured(measured_pos), &reset);
    if (reset && saw_reset) *saw_reset = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return out;
}

}  // namespace

int main() {
  auto& gate = s10::JointCommandOwner::Instance();

  // ------------------------------------------------ the default is the shipped policy
  gate.ResetForTest();
  bool reset = false;
  types::MatXf out = gate.Arbitrate(OfficialAction(0.5f), Measured(0.1f), &reset);
  Check(gate.owner() == s10::JointOwner::kOfficial, "the shipped policy owns the joints at start");
  Check(out(0, s10::JointCommandOwner::kColPos) == 0.5f, "and its command passes through intact");
  Check(!reset, "with no spurious history reset on the first tick");

  // ------------------------------------------------ a handover is never a step change
  gate.ResetForTest();
  gate.RequestOwner("climb");
  gate.SubmitClimbTargets(types::VecXf::Constant(16, 1.25f));
  out = gate.Arbitrate(OfficialAction(0.5f), Measured(0.1f), nullptr);
  Check(gate.owner() == s10::JointOwner::kSafeHold, "asking for the climb policy holds first");
  Check(out(0, s10::JointCommandOwner::kColPos) == 0.1f,
        "and the hold is at the measured position, not either policy's target");
  Check(out(0, s10::JointCommandOwner::kColPos) != 0.5f,
        "so the shipped policy is already off the actuators");

  // ------------------------------------------------ and it completes
  Settle(gate, s10::JointCommandOwner::kHandoverS + 0.05, 0.5f, 0.1f);
  gate.SubmitClimbTargets(types::VecXf::Constant(16, 1.25f));
  out = gate.Arbitrate(OfficialAction(0.5f), Measured(0.1f), nullptr);
  Check(gate.owner() == s10::JointOwner::kClimb, "the climb policy takes over after the hold");
  Check(out(0, s10::JointCommandOwner::kColPos) == 1.25f, "and its targets are what goes out");
  Check(out(3, s10::JointCommandOwner::kColKp) == 0.0f,
        "wheels are damped rather than held at an angle");
  Check(out(3, s10::JointCommandOwner::kColKd) > 0.0f, "with real damping on them");

  // ------------------------------------------------ only one source, ever
  Check(out(0, s10::JointCommandOwner::kColPos) != 0.5f,
        "the shipped policy contributes nothing while the climb policy drives");

  // ------------------------------------------------ Gate16 preserves the full mixed command
  gate.ResetForTest();
  gate.RequestOwner("gate16");
  Settle(gate, s10::JointCommandOwner::kHandoverS + 0.05, 0.5f, 0.1f);
  types::MatXf gate16 = types::MatXf::Zero(16, 5);
  gate16(0, s10::JointCommandOwner::kColKp) = 80.0f;
  gate16(0, s10::JointCommandOwner::kColPos) = -0.25f;
  gate16(3, s10::JointCommandOwner::kColKd) = 0.6f;
  gate16(3, s10::JointCommandOwner::kColVel) = 12.5f;
  out = gate.Arbitrate(OfficialAction(0.5f), &gate16, Measured(0.1f), nullptr);
  Check(gate.owner() == s10::JointOwner::kGate16, "the in-process Gate16 actor owns after hold");
  Check(out(0, s10::JointCommandOwner::kColPos) == -0.25f,
        "Gate16 leg position passes through unchanged");
  Check(out(3, s10::JointCommandOwner::kColVel) == 12.5f,
        "Gate16 wheel velocity passes through instead of being discarded");
  Check(out(0, s10::JointCommandOwner::kColPos) != 0.5f,
        "the official matrix is not blended into Gate16");
  Check(!gate.gate16_armed(), "Gate16 residual stays disarmed during base prewarm");
  gate.RequestOwner("gate16_climb");
  out = gate.Arbitrate(OfficialAction(0.5f), &gate16, Measured(0.1f), nullptr);
  Check(gate.owner() == s10::JointOwner::kGate16,
        "arming residual does not cause another actuator handover");
  Check(gate.gate16_armed(), "the explicit climb request arms residual");
  Check(out(0, s10::JointCommandOwner::kColPos) == -0.25f,
        "the warm Gate16 command remains continuous across the arm edge");
  gate.RequestOwner("official");
  Check(!gate.gate16_armed(), "requesting the follower disarms residual immediately");

  // ------------------------------------------------ the armed Gate16 handoff keeps rolling
  gate.ResetForTest();
  gate.RequestOwner("gate16_shadow");
  out = gate.Arbitrate(OfficialAction(0.5f), nullptr, Measured(0.1f), nullptr);
  Check(gate.owner() == s10::JointOwner::kOfficial && gate.gate16_shadow(),
        "shadow warmup evaluates Gate16 without taking the actuators");
  Check(out(0, s10::JointCommandOwner::kColPos) == 0.5f,
        "the official command remains intact during shadow warmup");
  gate.RequestOwner("gate16_climb");
  out = gate.Arbitrate(OfficialAction(0.5f), nullptr, Measured(0.1f), nullptr);
  Check(gate.owner() == s10::JointOwner::kOfficial,
        "callback ordering waits briefly for a finite Gate16 command without stopping");
  Check(out(0, s10::JointCommandOwner::kColPos) == 0.5f,
        "the moving official command continues while that first action is pending");
  out = gate.Arbitrate(OfficialAction(0.5f), &gate16, Measured(0.1f), nullptr);
  Check(gate.owner() == s10::JointOwner::kGate16,
        "an armed Gate16 request with a finite command transfers in one tick");
  Check(out(3, s10::JointCommandOwner::kColVel) == 12.5f,
        "the moving handoff does not insert a wheel-stopping hold");
  const auto moving_transitions = gate.transitions();
  Check(moving_transitions.size() == 1 &&
            moving_transitions.front().find("armed moving climb handover") != std::string::npos,
        "the exceptional transfer is explicit in the transition log");

  // ------------------------------------------------ a silent climb policy is a hold
  gate.ResetForTest();
  gate.RequestOwner("climb");
  gate.SubmitClimbTargets(types::VecXf::Constant(16, 1.25f));
  Settle(gate, s10::JointCommandOwner::kHandoverS + 0.05, 0.5f, 0.1f);
  gate.SubmitClimbTargets(types::VecXf::Constant(16, 1.25f));
  std::this_thread::sleep_for(
      std::chrono::milliseconds(static_cast<int>(s10::JointCommandOwner::kClimbTimeoutS * 1000) + 60));
  out = gate.Arbitrate(OfficialAction(0.5f), Measured(0.1f), nullptr);
  Check(gate.owner() == s10::JointOwner::kSafeHold, "a stale climb command falls back to the hold");
  Check(out(0, s10::JointCommandOwner::kColPos) == 0.1f, "holding where the robot is");
  Check(out(0, s10::JointCommandOwner::kColPos) != 0.5f,
        "and not silently handing back to the shipped policy");

  // ------------------------------------------------ handing back clears the action history
  gate.ResetForTest();
  gate.RequestOwner("climb");
  gate.SubmitClimbTargets(types::VecXf::Constant(16, 1.25f));
  Settle(gate, s10::JointCommandOwner::kHandoverS + 0.05, 0.5f, 0.1f);
  gate.RequestOwner("official");
  bool saw_reset = false;
  Settle(gate, s10::JointCommandOwner::kHandoverS + 0.1, 0.5f, 0.1f, &saw_reset);
  Check(gate.owner() == s10::JointOwner::kOfficial, "the shipped policy gets the joints back");
  Check(saw_reset, "and is told to clear the action history it built before the climb");

  // ------------------------------------------------ a late command from the old owner
  gate.ResetForTest();
  gate.SubmitClimbTargets(types::VecXf::Constant(16, 9.9f));
  gate.RequestOwner("climb");
  Settle(gate, s10::JointCommandOwner::kHandoverS + 0.05, 0.5f, 0.1f);
  out = gate.Arbitrate(OfficialAction(0.5f), Measured(0.1f), nullptr);
  Check(out(0, s10::JointCommandOwner::kColPos) != 9.9f,
        "a target submitted before the handover cannot be replayed after it");

  // ------------------------------------------------ shutdown is an explicit safe state
  gate.ResetForTest();
  gate.RequestOwner("stop");
  Settle(gate, s10::JointCommandOwner::kHandoverS + 0.05, 0.5f, 0.1f);
  out = gate.Arbitrate(OfficialAction(0.5f), Measured(0.1f), nullptr);
  Check(gate.owner() == s10::JointOwner::kStopped, "stop is an owner in its own right");
  Check(out(0, s10::JointCommandOwner::kColPos) == 0.1f,
        "and nothing reaches the actuators but the hold");

  // ------------------------------------------------ rubbish is refused, not forwarded
  //
  // Checked against a climb that is already driving, so that "the gate held" is attributable
  // to the NaN and not to a handover that had not finished yet.
  gate.ResetForTest();
  gate.RequestOwner("climb");
  Settle(gate, s10::JointCommandOwner::kHandoverS + 0.05, 0.5f, 0.1f);
  gate.SubmitClimbTargets(types::VecXf::Constant(16, 0.4f));
  out = gate.Arbitrate(OfficialAction(0.5f), Measured(0.1f), nullptr);
  Check(gate.owner() == s10::JointOwner::kClimb && out(0, s10::JointCommandOwner::kColPos) == 0.4f,
        "a valid target drives the legs");
  types::VecXf broken = types::VecXf::Constant(16, 0.4f);
  broken(7) = std::nan("");
  gate.SubmitClimbTargets(broken);
  std::this_thread::sleep_for(
      std::chrono::milliseconds(static_cast<int>(s10::JointCommandOwner::kClimbTimeoutS * 1000) + 60));
  out = gate.Arbitrate(OfficialAction(0.5f), Measured(0.1f), nullptr);
  Check(gate.owner() == s10::JointOwner::kSafeHold,
        "a non-finite target is dropped rather than kept, so the gate times out into the hold");
  Check(std::isfinite(out(7, s10::JointCommandOwner::kColPos)),
        "and no NaN reaches the actuators");

  // ------------------------------------------------ an unknown name changes nothing
  gate.ResetForTest();
  gate.RequestOwner("whatever");
  Settle(gate, 0.1, 0.5f, 0.1f);
  Check(gate.owner() == s10::JointOwner::kOfficial, "an unknown owner request is ignored");

  std::printf("%s\n", failures ? "FAILURES" : "all checks passed");
  return failures ? 1 : 0;
}
