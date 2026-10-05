// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "application/robotic/simulation.hpp"
#include "base/testing.hpp"

// Actuators and feedback by simon and by MuJoCo, against the tables
// reference/mujoco_control.py recorded: an arm under position, velocity and
// general actuators, and a cart-pole balanced by MuJoCo's own linear
// quadratic regulator.
namespace simon::robotic {

namespace {

constexpr std::string_view MODELS = "application/robotic/models/";

auto parse_numbers(const std::string& text) -> std::vector<double> {
  std::vector<double> values;
  std::stringstream stream{text};
  double value = 0.0;
  while (stream >> value) {
    values.push_back(value);
  }
  return values;
}

// A table's rows, split at commas.
auto load_rows(std::string_view name) -> std::vector<std::vector<std::string>> {
  std::ifstream file{"application/robotic/reference/" + std::string{name}};
  REQUIRE(file);
  std::vector<std::vector<std::string>> rows;
  std::string line;
  std::getline(file, line);
  while (std::getline(file, line)) {
    std::vector<std::string> cells;
    std::stringstream stream{line};
    std::string cell;
    while (std::getline(stream, cell, ',')) {
      cells.push_back(cell);
    }
    rows.push_back(std::move(cells));
  }
  return rows;
}

// The largest differences from MuJoCo over every step of `case_name`.
auto compare(InOut<Simulation> simulation, std::string_view case_name)
    -> std::pair<double, double> {
  std::vector<std::vector<double>> qpos;
  std::vector<std::vector<double>> qvel;
  for (const auto& row : load_rows("mujoco_control.csv")) {
    if (row[0] == case_name) {
      qpos.push_back(parse_numbers(row[2]));
      qvel.push_back(parse_numbers(row[3]));
    }
  }
  auto dt = std::chrono::nanoseconds{
      std::llround(simulation->mechanics().model().physics.timestep * 1e9)};
  double position = 0.0;
  double velocity = 0.0;
  for (std::size_t k = 0; k < qpos.size(); ++k) {
    if (k > 0) {
      REQUIRE(simulation->step(framework::Step{
          .time = framework::TimePoint{} + (k - 1) * dt, .dt = dt}));
    }
    std::vector<double> q = simulation->read_qpos();
    std::vector<double> v = simulation->read_qvel();
    for (std::size_t i = 0; i < q.size(); ++i) {
      position = std::max(position, std::abs(q[i] - qpos[k][i]));
    }
    for (std::size_t i = 0; i < v.size(); ++i) {
      velocity = std::max(velocity, std::abs(v[i] - qvel[k][i]));
    }
  }
  return {position, velocity};
}

}  // namespace

TEST_CASE("ControlAgainstMuJoCo") {
  SECTION("ShouldServoAsMuJoCoDoesGivenEveryActuator") {
    // Position servos, one geared, a velocity servo with damping taken
    // implicitly, and a general actuator against its force range, held at
    // their controls for 3 s: every step equal to MuJoCo's.
    Simulation simulation{Scenario{.model = std::string{MODELS} + "arm.xml",
                                   .qpos = {{0, 0.3}, {1, -0.5}},
                                   .control = {0.8, -1.2, 2.0, 0.5}}};
    REQUIRE(simulation.configure());
    auto [position, velocity] = compare(InOut(simulation), "arm");
    CAPTURE(position, velocity);
    CHECK(position < 1e-15);
    CHECK(velocity < 1e-15);
  }

  SECTION("ShouldBalanceAsMuJoCoDoesGivenFeedback") {
    // The cart-pole from 0.2 rad, its motor driven by MuJoCo's linear
    // quadratic regulator each step, for 15 s: every step equal to
    // MuJoCo's, and the pole upright within a microradian at the end.
    std::vector<std::string> law = load_rows("mujoco_feedback.csv").at(0);
    Simulation simulation{
        Scenario{.model = std::string{MODELS} + "cartpole.xml",
                 .qpos = {{1, 0.2}},
                 .feedback = {.gains = parse_numbers(law[1]),
                              .reference = parse_numbers(law[2]),
                              .offset = parse_numbers(law[3])}}};
    REQUIRE(simulation.configure());
    auto [position, velocity] = compare(InOut(simulation), "balance");
    CAPTURE(position, velocity);
    CHECK(position < 1e-15);
    CHECK(velocity < 1e-15);
    CHECK(std::abs(simulation.read_qpos()[1]) < 1e-6);
  }
}

}  // namespace simon::robotic
