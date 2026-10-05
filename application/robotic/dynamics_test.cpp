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

// robotic's test models stepped without constraints by simon and by MuJoCo,
// every position and velocity at every step, against the table
// reference/mujoco_dynamics.py recorded.
namespace simon::robotic {

namespace {

using namespace std::chrono_literals;

constexpr std::string_view MODELS = "application/robotic/models/";

// Each case's positions and velocities after each step, step 0 the start.
struct Run final {
  std::vector<std::vector<double>> qpos;
  std::vector<std::vector<double>> qvel;
};

auto parse_numbers(const std::string& text) -> std::vector<double> {
  std::vector<double> values;
  std::stringstream stream{text};
  double value = 0.0;
  while (stream >> value) {
    values.push_back(value);
  }
  return values;
}

auto load_runs() -> std::map<std::string, Run> {
  std::ifstream file{"application/robotic/reference/mujoco_dynamics.csv"};
  REQUIRE(file);
  std::map<std::string, Run> runs;
  std::string line;
  std::getline(file, line);
  while (std::getline(file, line)) {
    std::vector<std::string> cells;
    std::stringstream stream{line};
    std::string cell;
    while (std::getline(stream, cell, ',')) {
      cells.push_back(cell);
    }
    REQUIRE(cells.size() == 4);
    Run& run = runs[cells[0]];
    run.qpos.push_back(parse_numbers(cells[2]));
    run.qvel.push_back(parse_numbers(cells[3]));
  }
  return runs;
}

struct Case final {
  std::string name;
  std::string file;
  Scenario scenario;
  double position = 0.0;  // Bounds on the largest difference.
  double velocity = 0.0;
};

}  // namespace

TEST_CASE("DynamicsAgainstMuJoCo") {
  SECTION("ShouldStepAsMuJoCoDoesGivenNoConstraints") {
    // Every step equal to MuJoCo's but where a ball or free joint's
    // quaternion is in play, there within 2e-14: the double pendulum, chaotic,
    // equal through all 3,000 steps.
    std::map<std::string, Run> runs = load_runs();
    // clang-format off
    std::vector<Case> cases{
        {"pendulum", "pendulum.xml", {.qpos = {{0, 0.7}}}, 1e-15, 1e-15},
        {"double pendulum", "double_pendulum.xml",
         {.qpos = {{0, 1.2}, {1, -0.4}}}, 1e-15, 1e-15},
        {"free body", "free_body.xml",
         {.qvel = {{0, 0.3}, {1, -0.1}, {2, 0.2}, {3, 4.0}, {4, 0.5}, {5, 1.5}}},
         1e-14, 1e-14},
        {"features", "features.xml",
         {.qpos = {{0, 0.1}, {1, 0.9}, {2, 0.2}, {3, -0.3}, {4, 0.25}, {5, 0.3},
                   {6, 0.5}},
          .qvel = {{2, 1.0}}},
         1e-14, 2e-14},
        {"cartpole", "cartpole.xml", {.qpos = {{1, 0.05}}, .control = {0.3}},
         1e-15, 1e-15},
        {"boxes", "boxes.xml", {.qvel = {{7, 0.5}, {23, -1.0}}}, 1e-15, 1e-15},
    };
    // clang-format on
    for (Case& c : cases) {
      CAPTURE(c.name);
      const Run& theirs = runs.at(c.name);
      c.scenario.model = std::string{MODELS} + c.file;
      c.scenario.constrained = false;
      Simulation simulation{c.scenario};
      auto configured = simulation.configure();
      if (!configured) {
        FAIL(configured.error().message());
      }
      auto dt = std::chrono::nanoseconds{
          std::llround(simulation.mechanics().model().physics.timestep * 1e9)};
      double position = 0.0;
      double velocity = 0.0;
      for (std::size_t k = 0; k < theirs.qpos.size(); ++k) {
        if (k > 0) {
          REQUIRE(simulation.step(framework::Step{
              .time = framework::TimePoint{} + (k - 1) * dt, .dt = dt}));
        }
        std::vector<double> qpos = simulation.read_qpos();
        std::vector<double> qvel = simulation.read_qvel();
        REQUIRE(qpos.size() == theirs.qpos[k].size());
        REQUIRE(qvel.size() == theirs.qvel[k].size());
        for (std::size_t i = 0; i < qpos.size(); ++i) {
          position = std::max(position, std::abs(qpos[i] - theirs.qpos[k][i]));
        }
        for (std::size_t i = 0; i < qvel.size(); ++i) {
          velocity = std::max(velocity, std::abs(qvel[i] - theirs.qvel[k][i]));
        }
      }
      CAPTURE(theirs.qpos.size(), position, velocity);
      CHECK(position < c.position);
      CHECK(velocity < c.velocity);
    }
  }
}

}  // namespace simon::robotic
