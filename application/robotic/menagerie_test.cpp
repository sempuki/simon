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

}  // namespace

TEST_CASE("MenagerieAgainstMuJoCo") {
  SECTION("ShouldStepAsMuJoCoDoesGivenMenageriesRobots") {
    // Go1 standing on the elliptic cone, its hips cylinders; H1 falling, a
    // tree of 20 bodies; UR5e by implicitfast; ANYmal C standing: each step
    // from MuJoCo's states, and every step of each run, as MuJoCo's to
    // rounding.
    std::map<std::string, Run, std::less<>> runs =
        load_runs("mujoco_menagerie.csv");
    auto local = load_local_steps();
    auto spreads = load_spreads();
    auto solved = load_solved_runs();
    for (const std::string& robot :
         {"unitree_go1", "unitree_h1", "universal_robots_ur5e",
          "anybotics_anymal_c"}) {
      CAPTURE(robot);
      const Run& run = runs.at(robot);
      Scenario scenario{.model = std::string{MENAGERIE} + robot + "/scene.xml",
                        .control = run.control};
      for (std::uint32_t q = 0; q < run.start.size(); ++q) {
        scenario.qpos.emplace_back(q, run.start[q]);
      }
      check_local_steps(scenario, local.at(robot));
      Simulation simulation{scenario};
      auto configured = simulation.configure();
      if (!configured) {
        FAIL(configured.error().message());
      }
      check_run(record_run(InOut(simulation), run.qpos.size() - 1), run,
                spreads.at(robot), &solved.at(robot));
    }
  }
}

}  // namespace simon::robotic
