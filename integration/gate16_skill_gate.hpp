/**
 * @file gate16_skill_gate.hpp
 * @brief Stateful height-map gate for the S10 climbing residual.
 *
 * Keep these constants synchronized with training/s10_rl/skill_gate.py.  The policy
 * convention is base_z - terrain_z - 0.5, so an upward edge is near_value-far_value.
 */

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>

#include "gate16_perception_buffer.hpp"

namespace s10_policy {

struct SkillGateReport {
  bool active = false;
  float max_up_step = 0.0f;
  float retained_up_step = 0.0f;
  int active_frames = 0;
  int valid_edges = 0;
  float edge_heading_rad = 0.0f;
  bool edge_heading_valid = false;
  int edge_heading_samples = 0;
  float edge_heading_peak_step = 0.0f;
  float left_edge_distance = std::numeric_limits<float>::infinity();
  float right_edge_distance = std::numeric_limits<float>::infinity();
  bool both_side_edges_visible = false;
};

struct FastAdapterEnvelope {
  bool confidence_gate_enabled = false;
  int min_edge_heading_samples = 0;
  float min_edge_heading_peak_step = 0.0f;
  float max_abs_entry_yaw_deg = 90.0f;
  float min_entry_speed_mps = 0.0f;
  float max_entry_speed_mps = std::numeric_limits<float>::infinity();
  float max_side_edge_skew_m = std::numeric_limits<float>::infinity();
};

inline bool EntrySupportsFastAdapter(const SkillGateReport& report,
                                     float entry_yaw_deg,
                                     float entry_speed_mps,
                                     const FastAdapterEnvelope& envelope) {
  if (!envelope.confidence_gate_enabled) return true;
  if (!report.edge_heading_valid || !report.both_side_edges_visible ||
      report.edge_heading_samples < envelope.min_edge_heading_samples ||
      report.edge_heading_peak_step < envelope.min_edge_heading_peak_step ||
      !std::isfinite(report.left_edge_distance) ||
      !std::isfinite(report.right_edge_distance) ||
      !std::isfinite(entry_yaw_deg) || !std::isfinite(entry_speed_mps)) {
    return false;
  }
  return std::abs(entry_yaw_deg) <= envelope.max_abs_entry_yaw_deg &&
         entry_speed_mps >= envelope.min_entry_speed_mps &&
         entry_speed_mps <= envelope.max_entry_speed_mps &&
         std::abs(report.left_edge_distance - report.right_edge_distance) <=
             envelope.max_side_edge_skew_m;
}

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
  static constexpr int kReentryBlockFrames = 250;
  static constexpr float kDistinctForwardEdgeX = 0.55f;
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
    hold_until_active_frame_ = 0;
    entry_heading_sum_ = 0.0f;
    entry_heading_count_ = 0;
    entry_heading_samples_ = 0;
    entry_heading_peak_step_ = 0.0f;
    frozen_entry_heading_ = 0.0f;
    frozen_entry_heading_valid_ = false;
    frozen_entry_heading_samples_ = 0;
    frozen_entry_heading_peak_step_ = 0.0f;
    reentry_block_frames_ = 0;
  }

  void HoldActiveForAdditionalFrames(int frames) {
    if (!active_ || frames <= 0) return;
    hold_until_active_frame_ =
        std::max(hold_until_active_frame_, active_frames_ + frames);
    exit_count_ = 0;
  }

  SkillGateReport Update(const s10_perception::HeightSnapshot& snapshot) {
    const auto forward = Measure(snapshot, kFirstBoundaryRow, kLastBoundaryRow);
    const auto retained = Measure(snapshot, kFirstRetentionRow, kLastRetentionRow);
    const auto geometry = MeasureEdgeGeometry(snapshot);
    const float maximum = forward.first;
    const int valid_edges = forward.second;

    if (!active_) {
      bool reentry_blocked = reentry_block_frames_ > 0;
      if (reentry_blocked) {
        --reentry_block_frames_;
        const bool distinct_forward_edge =
            geometry.both_sides_visible &&
            std::min(geometry.left_distance, geometry.right_distance) >=
                kDistinctForwardEdgeX;
        if (distinct_forward_edge) {
          reentry_block_frames_ = 0;
          reentry_blocked = false;
        }
      }
      const bool entering = !reentry_blocked && valid_edges > 0 &&
                            maximum >= kEnterStepHeight;
      if (!entering) {
        enter_count_ = 0;
        entry_heading_sum_ = 0.0f;
        entry_heading_count_ = 0;
        entry_heading_samples_ = 0;
        entry_heading_peak_step_ = 0.0f;
      } else {
        ++enter_count_;
        entry_heading_samples_ += geometry.heading_samples;
        entry_heading_peak_step_ =
            std::max(entry_heading_peak_step_, geometry.heading_peak_step);
        if (geometry.heading_valid) {
          entry_heading_sum_ += geometry.heading_rad;
          ++entry_heading_count_;
        }
      }
      if (enter_count_ >= kEnterFrames) {
        active_ = true;
        active_frames_ = 0;
        exit_count_ = 0;
        frozen_entry_heading_valid_ = entry_heading_count_ > 0;
        frozen_entry_heading_samples_ = entry_heading_samples_;
        frozen_entry_heading_peak_step_ = entry_heading_peak_step_;
        if (frozen_entry_heading_valid_) {
          frozen_entry_heading_ =
              entry_heading_sum_ / static_cast<float>(entry_heading_count_);
        }
      }
    } else {
      ++active_frames_;
      if (active_frames_ >= kMinActiveFrames &&
          active_frames_ >= hold_until_active_frame_) {
        // A stale map must not terminate a climb while the rear wheels are still
        // below the platform.  The hard timeout remains as the final fail-safe.
        exit_count_ = snapshot.valid && retained.second > 0 &&
                              retained.first <= kExitStepHeight
                          ? exit_count_ + 1
                          : 0;
      }
      if (exit_count_ >= kExitFrames || active_frames_ >= kMaxActiveFrames) {
        Reset();
        reentry_block_frames_ = kReentryBlockFrames;
      }
    }
    const float report_heading = active_ && frozen_entry_heading_valid_
                                     ? frozen_entry_heading_
                                     : geometry.heading_rad;
    const bool report_heading_valid = active_
                                          ? frozen_entry_heading_valid_
                                          : geometry.heading_valid;
    const int report_heading_samples = active_
                                           ? frozen_entry_heading_samples_
                                           : geometry.heading_samples;
    const float report_heading_peak_step = active_
                                               ? frozen_entry_heading_peak_step_
                                               : geometry.heading_peak_step;
    return {active_,
            maximum,
            retained.first,
            active_frames_,
            valid_edges,
            report_heading,
            report_heading_valid,
            report_heading_samples,
            report_heading_peak_step,
            geometry.left_distance,
            geometry.right_distance,
            geometry.both_sides_visible};
  }

 private:
  struct EdgeGeometry {
    float heading_rad = 0.0f;
    bool heading_valid = false;
    int heading_samples = 0;
    float heading_peak_step = 0.0f;
    float left_distance = std::numeric_limits<float>::infinity();
    float right_distance = std::numeric_limits<float>::infinity();
    bool both_sides_visible = false;
  };

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

  static EdgeGeometry MeasureEdgeGeometry(
      const s10_perception::HeightSnapshot& snapshot) {
    EdgeGeometry result;
    if (!snapshot.valid) return result;

    constexpr float kXMin = -0.6f;
    constexpr float kYMin = -0.6f;
    constexpr float kGridStep = 0.15f;
    // Fit heading from the full forward half-map, matching TerrainAdvisor. The
    // previous retention scan also included edges behind the robot, while the
    // narrower activation band sometimes exposed fewer than three columns.
    constexpr int kFirstHeadingColumn = kFirstColumn;
    constexpr int kLastHeadingColumn = kLastColumn;
    std::array<float, kColumns> heading_edge_x{};
    std::array<bool, kColumns> heading_visible{};
    float mean_x = 0.0f;
    float mean_y = 0.0f;
    int samples = 0;

    float heading_peak_step = 0.0f;
    for (int row = kFirstBoundaryRow; row < kRows - 1; ++row) {
      for (int column = kFirstHeadingColumn;
           column <= kLastHeadingColumn; ++column) {
        const float near = snapshot.values[Index(row, column)];
        const float far = snapshot.values[Index(row + 1, column)];
        if (near <= s10_perception::kVoidSentinel ||
            far <= s10_perception::kVoidSentinel) {
          continue;
        }
        heading_peak_step = std::max(heading_peak_step, near - far);
      }
    }
    result.heading_peak_step = heading_peak_step;
    const float heading_threshold =
        std::max(kEnterStepHeight, 0.5f * heading_peak_step);

    for (int column = kFirstHeadingColumn;
         column <= kLastHeadingColumn; ++column) {
      float best_delta = heading_threshold;
      int best_row = -1;
      for (int row = kFirstBoundaryRow; row < kRows - 1; ++row) {
        const float near = snapshot.values[Index(row, column)];
        const float far = snapshot.values[Index(row + 1, column)];
        if (near <= s10_perception::kVoidSentinel ||
            far <= s10_perception::kVoidSentinel) {
          continue;
        }
        const float delta = near - far;
        if (delta > best_delta) {
          best_delta = delta;
          best_row = row;
        }
      }
      if (best_row < 0) continue;
      heading_visible[column] = true;
      heading_edge_x[column] = kXMin + kGridStep * (best_row + 0.5f);
      const float y = kYMin + kGridStep * column;
      mean_x += heading_edge_x[column];
      mean_y += y;
      ++samples;
    }

    if (samples >= 3) {
      mean_x /= samples;
      mean_y /= samples;
      float covariance = 0.0f;
      float variance_y = 0.0f;
      for (int column = kFirstHeadingColumn;
           column <= kLastHeadingColumn; ++column) {
        if (!heading_visible[column]) continue;
        const float y = kYMin + kGridStep * column;
        covariance +=
            (y - mean_y) * (heading_edge_x[column] - mean_x);
        variance_y += (y - mean_y) * (y - mean_y);
      }
      if (variance_y > 1.0e-6f) {
        result.heading_rad = std::clamp(-std::atan(covariance / variance_y),
                                        -0.6f, 0.6f);
        result.heading_valid = true;
      }
    }
    result.heading_samples = samples;

    float left_sum = 0.0f;
    float right_sum = 0.0f;
    int left_count = 0;
    int right_count = 0;
    for (int column = kFirstColumn; column <= kLastColumn; ++column) {
      float best_delta = kEnterStepHeight;
      int best_row = -1;
      for (int row = kFirstRetentionRow; row < kRows - 1; ++row) {
        const float near = snapshot.values[Index(row, column)];
        const float far = snapshot.values[Index(row + 1, column)];
        if (near <= s10_perception::kVoidSentinel ||
            far <= s10_perception::kVoidSentinel) {
          continue;
        }
        const float delta = near - far;
        if (delta > best_delta) {
          best_delta = delta;
          best_row = row;
        }
      }
      if (best_row < 0) continue;
      const float support_edge_x =
          kXMin + kGridStep * (best_row + 0.5f);
      if (column >= 5) {
        left_sum += support_edge_x;
        ++left_count;
      } else if (column <= 3) {
        right_sum += support_edge_x;
        ++right_count;
      }
    }
    result.both_sides_visible = left_count > 0 && right_count > 0;
    if (left_count > 0) result.left_distance = left_sum / left_count;
    if (right_count > 0) result.right_distance = right_sum / right_count;
    return result;
  }

  bool active_ = false;
  int enter_count_ = 0;
  int exit_count_ = 0;
  int active_frames_ = 0;
  int hold_until_active_frame_ = 0;
  float entry_heading_sum_ = 0.0f;
  int entry_heading_count_ = 0;
  int entry_heading_samples_ = 0;
  float entry_heading_peak_step_ = 0.0f;
  float frozen_entry_heading_ = 0.0f;
  bool frozen_entry_heading_valid_ = false;
  int frozen_entry_heading_samples_ = 0;
  float frozen_entry_heading_peak_step_ = 0.0f;
  int reentry_block_frames_ = 0;
};

}  // namespace s10_policy
