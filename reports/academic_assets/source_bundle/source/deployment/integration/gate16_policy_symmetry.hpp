/**
 * @file gate16_policy_symmetry.hpp
 * @brief Left-right transforms for the 57/174-D S10 observation and 16-D action.
 */

#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

namespace s10_policy {

inline constexpr std::array<int, 16> kActionMirrorIndices = {
    3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8, 13, 12, 15, 14};
inline constexpr std::array<float, 16> kActionMirrorSigns = {
    -1.0f, 1.0f, 1.0f, -1.0f, 1.0f, 1.0f, -1.0f, 1.0f,
    1.0f,  -1.0f, 1.0f, 1.0f,  1.0f, 1.0f,  1.0f, 1.0f};

template <typename Vector>
Vector MirrorPolicyAction(const Vector& action) {
  if (action.size() != 16) {
    throw std::invalid_argument("S10 policy action mirror expects 16 values");
  }
  Vector result = action;
  for (std::size_t i = 0; i < kActionMirrorIndices.size(); ++i) {
    result[i] = action[kActionMirrorIndices[i]] * kActionMirrorSigns[i];
  }
  return result;
}

template <typename Vector>
Vector MirrorPolicyObservation(const Vector& observation) {
  if (observation.size() != 57 && observation.size() != 174) {
    throw std::invalid_argument(
        "S10 policy observation mirror expects 57 or 174 values");
  }
  Vector result = observation;

  // Angular velocity is axial; gravity and navigation commands are polar.
  for (int index : {0, 2, 4, 7, 8}) result[index] = -observation[index];
  for (int start : {9, 25, 41}) {
    for (std::size_t i = 0; i < kActionMirrorIndices.size(); ++i) {
      result[start + i] = observation[start + kActionMirrorIndices[i]] *
                          kActionMirrorSigns[i];
    }
  }
  if (observation.size() == 174) {
    constexpr int kHeightStart = 57;
    constexpr int kRows = 13;
    constexpr int kColumns = 9;
    for (int row = 0; row < kRows; ++row) {
      for (int column = 0; column < kColumns; ++column) {
        result[kHeightStart + row * kColumns + column] =
            observation[kHeightStart + row * kColumns + (kColumns - 1 - column)];
      }
    }
  }
  return result;
}

inline bool ShouldMirrorPositiveYaw(float entry_heading_error_deg,
                                    float threshold_deg) {
  if (!(threshold_deg > 0.0f && threshold_deg <= 90.0f)) {
    throw std::invalid_argument(
        "positive-yaw mirror threshold must be in (0, 90] degrees");
  }
  return entry_heading_error_deg >= threshold_deg;
}

inline bool ShouldMirrorYawBands(
    float entry_heading_error_deg,
    const std::vector<std::pair<float, float>>& bands_deg) {
  if (!std::isfinite(entry_heading_error_deg)) {
    throw std::invalid_argument("entry heading error must be finite");
  }
  float previous_high = -91.0f;
  for (const auto& band : bands_deg) {
    if (!std::isfinite(band.first) || !std::isfinite(band.second) ||
        band.first < -90.0f || band.second > 90.0f ||
        band.first > band.second || band.first <= previous_high) {
      throw std::invalid_argument(
          "yaw mirror bands must be ordered, disjoint, and within [-90, 90]");
    }
    previous_high = band.second;
    if (entry_heading_error_deg >= band.first &&
        entry_heading_error_deg <= band.second) {
      return true;
    }
  }
  return false;
}

}  // namespace s10_policy

