/**
 * @file s10_perception_buffer.hpp
 * @brief Thread-safe bridge from ROS height maps to the ONNX policy runner.
 */

#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <mutex>
#include <vector>

namespace s10_perception {

constexpr std::size_t kHeightCells = 13 * 9;
constexpr float kIsaacHeightOffset = 0.5f;
constexpr float kVoidSentinel = -1.0f;

struct HeightSnapshot {
  std::array<float, kHeightCells> values{};
  bool valid = false;
};

class HeightmapBuffer {
 public:
  bool UpdateRaw(const std::vector<float>& raw) {
    if (raw.size() != kHeightCells) return false;

    std::lock_guard<std::mutex> lock(mutex_);
    for (std::size_t i = 0; i < kHeightCells; ++i) {
      // MuJoCo publishes terrain_z - base_z. Isaac Lab trains on
      // base_z - terrain_z - 0.5. Preserve the MuJoCo no-hit sentinel so a
      // void matches Isaac Lab's clipped negative infinity.
      values_[i] = raw[i] <= kVoidSentinel
                       ? kVoidSentinel
                       : std::clamp(-raw[i] - kIsaacHeightOffset, -1.0f, 1.0f);
    }
    updated_ = Clock::now();
    valid_ = true;
    return true;
  }

  HeightSnapshot Read(std::chrono::milliseconds max_age = std::chrono::milliseconds(200)) {
    std::lock_guard<std::mutex> lock(mutex_);
    HeightSnapshot result;
    result.values = values_;
    result.valid = valid_ && Clock::now() - updated_ <= max_age;
    if (!result.valid) result.values.fill(0.0f);
    return result;
  }

 private:
  using Clock = std::chrono::steady_clock;

  std::mutex mutex_;
  std::array<float, kHeightCells> values_{};
  Clock::time_point updated_{};
  bool valid_ = false;
};

inline HeightmapBuffer& SharedHeightmap() {
  static HeightmapBuffer buffer;
  return buffer;
}

}  // namespace s10_perception
