"""Compile/run the dependency-free Gate16 v4 geometry and symmetry contracts."""

from __future__ import annotations

import shutil
import subprocess
import tempfile
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]
INTEGRATION = ROOT / "integration"


def _compile_and_run(source: str) -> None:
    compiler = shutil.which("g++") or shutil.which("clang++") or shutil.which("c++")
    if compiler is None:
        pytest.skip("no C++ compiler")
    with tempfile.TemporaryDirectory() as tmp:
        cpp = Path(tmp) / "smoke.cpp"
        binary = Path(tmp) / "smoke"
        cpp.write_text(source)
        subprocess.run(
            [compiler, "-std=c++17", "-Wall", "-Wextra", "-Werror", "-I", INTEGRATION, cpp, "-o", binary],
            check=True,
        )
        subprocess.run([binary], check=True)


def test_v4_policy_frame_mirror_is_an_involution_and_uses_expected_bands():
    _compile_and_run(
        r'''
#include <array>
#include <cassert>
#include <utility>
#include <vector>
#include "gate16_policy_symmetry.hpp"
int main() {
  std::array<float, 174> observation{};
  for (std::size_t i = 0; i < observation.size(); ++i) observation[i] = i;
  const auto mirrored = s10_policy::MirrorPolicyObservation(observation);
  assert(s10_policy::MirrorPolicyObservation(mirrored) == observation);
  assert(mirrored[9] == -observation[12]);
  assert(mirrored[57] == observation[65]);
  std::array<float, 16> action{};
  for (std::size_t i = 0; i < action.size(); ++i) action[i] = i;
  assert(s10_policy::MirrorPolicyAction(s10_policy::MirrorPolicyAction(action)) == action);
  const std::vector<std::pair<float, float>> bands = {
      {-27.5f, -22.5f}, {-7.5f, 7.5f}, {17.5f, 90.0f}};
  assert(s10_policy::ShouldMirrorYawBands(-25.0f, bands));
  assert(s10_policy::ShouldMirrorYawBands(5.0f, bands));
  assert(!s10_policy::ShouldMirrorYawBands(-20.0f, bands));
}
'''
    )


def test_v4_skill_gate_measures_edge_and_enters_after_two_frames():
    _compile_and_run(
        r'''
#include <cassert>
#include "gate16_skill_gate.hpp"
int main() {
  s10_policy::HeightmapSkillGate gate;
  s10_perception::HeightSnapshot map;
  map.valid = true;
  map.values.fill(0.0f);
  for (int row = 8; row < 13; ++row) {
    for (int column = 0; column < 9; ++column) {
      map.values[static_cast<std::size_t>(row * 9 + column)] = -0.377f;
    }
  }
  assert(!gate.Update(map).active);
  const auto entered = gate.Update(map);
  assert(entered.active);
  assert(entered.max_up_step > 0.37f);
  assert(entered.both_side_edges_visible);
}
'''
    )
