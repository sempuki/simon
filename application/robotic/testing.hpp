// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <map>
#include <string>
#include <string_view>
#include <vector>

#include "application/robotic/simulation.hpp"
#include "application/testing.hpp"
#include "base/testing.hpp"
#include "framework/vocabulary.hpp"

// Shared by the robotic tests: where the models and MuJoCo's tables lie,
// MuJoCo's runs read from them, and simon stepped against them.
namespace simon::robotic::testing {

using simon::testing::load_table;
using simon::testing::parse_numbers;

inline constexpr std::string_view MODELS = "application/robotic/models/";
inline constexpr std::string_view REFERENCE = "application/robotic/reference/";
inline constexpr std::string_view HUMANOID = "3rd_party/mujoco/humanoid.xml";
inline constexpr std::string_view MENAGERIE = "3rd_party/menagerie/";

// A case MuJoCo ran: where it started, its positions and controls, if the
// table gives them, and its positions and velocities after each step, step
// 0 the start.
struct Run final {
  std::vector<double> start;
  std::vector<double> control;
  std::vector<std::vector<double>> qpos;
  std::vector<std::vector<double>> qvel;
};

// The cases of MuJoCo's table `name`, by name: a line of each case's
// positions and velocities after each step, and a line with the step
// "start" for its starting positions and controls.
inline auto load_runs(std::string_view name)
    -> std::map<std::string, Run, std::less<>> {
  std::map<std::string, Run, std::less<>> runs;
  for (std::vector<std::string>& cells :
       load_table(std::string{REFERENCE} + std::string{name}).lines) {
    REQUIRE(cells.size() >= 4);
    Run& run = runs[cells[0]];
    if (cells[1] == "start") {
      run.start = parse_numbers(cells[2]);
      run.control = parse_numbers(cells[3]);
    } else {
      run.qpos.push_back(parse_numbers(cells[2]));
      run.qvel.push_back(parse_numbers(cells[3]));
    }
  }
  return runs;
}

// Steps `simulation` one model step, the `k`th from the start.
inline auto step_once(InOut<Simulation> simulation, std::size_t k) -> void {
  auto dt = std::chrono::nanoseconds{
      std::llround(simulation->mechanics().model().physics.timestep * 1e9)};
  REQUIRE(simulation->step(
      framework::Step{.time = framework::TimePoint{} + k * dt, .dt = dt}));
}

// Steps `simulation` for `seconds` of model steps from the start.
inline auto step_for(InOut<Simulation> simulation, double seconds) -> void {
  double h = simulation->mechanics().model().physics.timestep;
  auto steps = static_cast<std::size_t>(std::llround(seconds / h));
  for (std::size_t k = 0; k < steps; ++k) {
    step_once(simulation, k);
  }
}

// The largest differences in position and velocity.
struct Apart final {
  double position = 0.0;
  double velocity = 0.0;
};

// Steps `simulation` through the first `steps` steps of `run`, every step
// unless given, comparing its positions and velocities with MuJoCo's.
inline auto compare_run(
    InOut<Simulation> simulation, const Run& run,
    std::size_t steps = std::numeric_limits<std::size_t>::max()) -> Apart {
  Apart apart;
  std::size_t last = std::min(steps, run.qpos.size() - 1);
  for (std::size_t k = 0; k <= last; ++k) {
    if (k > 0) {
      step_once(simulation, k - 1);
    }
    std::vector<double> qpos = simulation->read_qpos();
    std::vector<double> qvel = simulation->read_qvel();
    REQUIRE(qpos.size() == run.qpos[k].size());
    REQUIRE(qvel.size() == run.qvel[k].size());
    for (std::size_t i = 0; i < qpos.size(); ++i) {
      apart.position =
          std::max(apart.position, std::abs(qpos[i] - run.qpos[k][i]));
    }
    for (std::size_t i = 0; i < qvel.size(); ++i) {
      apart.velocity =
          std::max(apart.velocity, std::abs(qvel[i] - run.qvel[k][i]));
    }
  }
  return apart;
}

}  // namespace simon::robotic::testing
