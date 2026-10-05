// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <string>
#include <vector>

#include "application/robotic/simulation.hpp"
#include "application/robotic/testing.hpp"
#include "base/testing.hpp"

// robotic's test models stepped without constraints by simon and by MuJoCo,
// every position and velocity at every step, against the table
// reference/mujoco_dynamics.py recorded.
namespace simon::robotic {

namespace {

using namespace std::chrono_literals;
using namespace testing;

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
    std::map<std::string, Run, std::less<>> runs =
        load_runs("mujoco_dynamics.csv");
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
      auto [position, velocity] = compare_run(InOut(simulation), theirs);
      CAPTURE(theirs.qpos.size(), position, velocity);
      CHECK(position < c.position);
      CHECK(velocity < c.velocity);
    }
  }
}

}  // namespace simon::robotic
