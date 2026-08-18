/**
 * @file s10_skill_gate.hpp
 * @brief Stateful height-map gate for the S10 climbing residual.
 *
 * Keep these constants synchronized with training/s10_rl/skill_gate.py.  The policy
 * convention is base_z - terrain_z - 0.5, so an upward edge is near_value-far_value.
 */

#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <utility>

#include "gate16_perception_buffer.hpp"

namespace s10_policy {

struct SkillGateReport {
  bool active = false;
  float max_up_step = 0.0f;
  float retained_up_step = 0.0f;
  int active_frames = 0;
  int valid_edges = 0;
};

class HeightmapSkillGate {
 public:
  static constexpr int kRows = 13;
  static constexpr int kColumns = 9;
  static constexpr float kEnterStepHeight = 0.04f;
  static constexpr float kExitStepHeight = 0.02f;
  static constexpr int kEnterFrames = 2;
  static constexpr int kExitFrames = 15;
  static constexpr int kMinActiveFrames = 100;
  static constexpr int kMaxActiveFrames = 600;
  static constexpr int kFirstBoundaryRow = 4;
  static constexpr int kLastBoundaryRow = 8;
  static constexpr int kFirstRetentionRow = 0;
  static constexpr int kLastRetentionRow = 8;
  static constexpr int kFirstColumn = 2;
  static constexpr int kLastColumn = 6;

  void Reset() {
    active_ = false;
    enter_count_ = 0;
    exit_count_ = 0;
    active_frames_ = 0;
  }

  SkillGateReport Update(const s10_perception::HeightSnapshot& snapshot) {
    const auto forward = Measure(snapshot, kFirstBoundaryRow, kLastBoundaryRow);
    const auto retained = Measure(snapshot, kFirstRetentionRow, kLastRetentionRow);
    const float maximum = forward.first;
    const int valid_edges = forward.second;

    if (!active_) {
      enter_count_ = valid_edges > 0 && maximum >= kEnterStepHeight
                         ? enter_count_ + 1
                         : 0;
      if (enter_count_ >= kEnterFrames) {
        active_ = true;
        active_frames_ = 0;
        exit_count_ = 0;
      }
    } else {
      ++active_frames_;
      if (active_frames_ >= kMinActiveFrames) {
        // A stale map must not terminate a climb while the rear wheels are still
        // below the platform.  The hard timeout remains as the final fail-safe.
        exit_count_ = snapshot.valid && retained.second > 0 &&
                              retained.first <= kExitStepHeight
                          ? exit_count_ + 1
                          : 0;
      }
      if (exit_count_ >= kExitFrames || active_frames_ >= kMaxActiveFrames) {
        Reset();
      }
    }
    return {active_, maximum, retained.first, active_frames_, valid_edges};
  }

 private:
  static constexpr std::size_t Index(int row, int column) {
    return static_cast<std::size_t>(row * kColumns + column);
  }

  static std::pair<float, int> Measure(
      const s10_perception::HeightSnapshot& snapshot, int first_row,
      int last_row) {
    float maximum = 0.0f;
    int valid_edges = 0;
    if (!snapshot.valid) return {maximum, valid_edges};
    for (int row = first_row; row <= last_row; ++row) {
      for (int column = kFirstColumn; column <= kLastColumn; ++column) {
        const float near = snapshot.values[Index(row, column)];
        const float far = snapshot.values[Index(row + 1, column)];
        if (near <= s10_perception::kVoidSentinel ||
            far <= s10_perception::kVoidSentinel) {
          continue;
        }
        ++valid_edges;
        maximum = std::max(maximum, near - far);
      }
    }
    return {maximum, valid_edges};
  }

  bool active_ = false;
  int enter_count_ = 0;
  int exit_count_ = 0;
  int active_frames_ = 0;
};

}  // namespace s10_policy
