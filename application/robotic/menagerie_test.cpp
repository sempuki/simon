// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "application/robotic/simulation.hpp"
#include "base/testing.hpp"

// Four robots of MuJoCo Menagerie by simon and by MuJoCo, against the table
// reference/mujoco_menagerie.py recorded: each from its home keyframe, its
// actuators held at the keyframe's controls.
namespace simon::robotic {

namespace {

auto parse_numbers(const std::string& text) -> std::vector<double> {
  std::vector<double> values;
  std::stringstream stream{text};
  double value = 0.0;
  while (stream >> value) {
    values.push_back(value);
  }
  return values;
}

struct Run final {
  std::vector<double> start;
  std::vector<double> control;
  std::vector<std::vector<double>> qpos;
  std::vector<std::vector<double>> qvel;
};

auto load_runs() -> std::map<std::string, Run> {
  std::ifstream file{"application/robotic/reference/mujoco_menagerie.csv"};
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
    cells.resize(4);
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

struct Case final {
  std::string robot;
  double position = 0.0;  // Bounds on the largest difference.
  double velocity = 0.0;
  std::size_t steps = 1000;
};

}  // namespace

TEST_CASE("MenagerieAgainstMuJoCo") {
  SECTION("ShouldStepAsMuJoCoDoesGivenMenageriesRobots") {
    // Go1 standing on the elliptic cone, its hips cylinders; H1 falling, a
    // tree of 20 bodies; UR5e by implicitfast; ANYmal C standing: every
    // step within 1e-11 of MuJoCo's.
    std::map<std::string, Run> runs = load_runs();
    std::vector<Case> cases{
        {"unitree_go1", 1e-13, 1e-11},
        {"unitree_h1", 1e-13, 1e-11},
        {"universal_robots_ur5e", 1e-14, 1e-14},
        {"anybotics_anymal_c", 1e-13, 1e-11},
    };
    for (const Case& c : cases) {
      CAPTURE(c.robot);
      const Run& run = runs.at(c.robot);
      Scenario scenario{
          .model = "3rd_party/menagerie/" + c.robot + "/scene.xml",
          .control = run.control};
      for (std::uint32_t q = 0; q < run.start.size(); ++q) {
        scenario.qpos.emplace_back(q, run.start[q]);
      }
      Simulation simulation{scenario};
      auto configured = simulation.configure();
      if (!configured) {
        FAIL(configured.error().message());
      }
      auto dt = std::chrono::nanoseconds{
          std::llround(simulation.mechanics().model().physics.timestep * 1e9)};
      double position = 0.0;
      double velocity = 0.0;
      std::size_t steps = std::min(c.steps, run.qpos.size() - 1);
      for (std::size_t k = 0; k <= steps; ++k) {
        if (k > 0) {
          REQUIRE(simulation.step(framework::Step{
              .time = framework::TimePoint{} + (k - 1) * dt, .dt = dt}));
        }
        std::vector<double> q = simulation.read_qpos();
        std::vector<double> v = simulation.read_qvel();
        for (std::size_t i = 0; i < q.size(); ++i) {
          position = std::max(position, std::abs(q[i] - run.qpos[k][i]));
        }
        for (std::size_t i = 0; i < v.size(); ++i) {
          velocity = std::max(velocity, std::abs(v[i] - run.qvel[k][i]));
        }
      }
      CAPTURE(position, velocity);
      CHECK(position < c.position);
      CHECK(velocity < c.velocity);
    }
  }
}

}  // namespace simon::robotic
