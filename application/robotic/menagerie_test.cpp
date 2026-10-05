// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "application/robotic/simulation.hpp"
#include "application/robotic/testing.hpp"
#include "base/testing.hpp"

// Four robots of MuJoCo Menagerie by simon and by MuJoCo, against the table
// reference/mujoco_menagerie.py recorded: each from its home keyframe, its
// actuators held at the keyframe's controls.
namespace simon::robotic {

namespace {

using namespace testing;

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
    std::map<std::string, Run, std::less<>> runs =
        load_runs("mujoco_menagerie.csv");
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
          .model = std::string{MENAGERIE} + c.robot + "/scene.xml",
          .control = run.control};
      for (std::uint32_t q = 0; q < run.start.size(); ++q) {
        scenario.qpos.emplace_back(q, run.start[q]);
      }
      Simulation simulation{scenario};
      auto configured = simulation.configure();
      if (!configured) {
        FAIL(configured.error().message());
      }
      auto [position, velocity] = compare_run(InOut(simulation), run, c.steps);
      CAPTURE(position, velocity);
      CHECK(position < c.position);
      CHECK(velocity < c.velocity);
    }
  }
}

}  // namespace simon::robotic
