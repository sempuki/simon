// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <cmath>
#include <string>
#include <vector>

#include "application/robotic/simulation.hpp"
#include "application/robotic/testing.hpp"
#include "base/testing.hpp"

// Actuators and feedback by simon and by MuJoCo, against the tables
// reference/mujoco_control.py and reference/mujoco_checks.py recorded: an
// arm under position, velocity and general actuators, and a cart-pole
// balanced by MuJoCo's own linear quadratic regulator.
namespace simon::robotic {

namespace {

using namespace testing;

auto make_arm() -> Scenario {
  return Scenario{.model = std::string{MODELS} + "arm.xml",
                  .qpos = {{0, 0.3}, {1, -0.5}},
                  .control = {0.8, -1.2, 2.0, 0.5},
                  .constrained = false};
}

auto make_balance() -> Scenario {
  std::vector<std::string> law =
      load_table(std::string{REFERENCE} + "mujoco_feedback.csv").lines.at(0);
  return Scenario{.model = std::string{MODELS} + "cartpole.xml",
                  .qpos = {{1, 0.2}},
                  .feedback = {.gains = parse_numbers(law[1]),
                               .reference = parse_numbers(law[2]),
                               .offset = parse_numbers(law[3])},
                  .constrained = false};
}

}  // namespace

TEST_CASE("ControlAgainstMuJoCo") {
  auto local = load_local_steps();
  auto spreads = load_spreads();
  auto runs = load_runs("mujoco_control.csv");

  SECTION("ShouldServoAsMuJoCoDoesGivenEveryActuator") {
    // Position servos, one geared, a velocity servo with damping taken
    // implicitly, and a general actuator against its force range, held at
    // their controls for 3 s: each step from MuJoCo's states, and the whole
    // run, as MuJoCo's to rounding.
    check_local_steps(make_arm(), local.at("arm"));
    Simulation simulation{make_arm()};
    REQUIRE(simulation.configure());
    const Run& theirs = runs.at("arm");
    check_run(record_run(InOut(simulation), theirs.qpos.size() - 1), theirs,
              spreads.at("arm"));
  }

  SECTION("ShouldBalanceAsMuJoCoDoesGivenFeedback") {
    // The cart-pole from 0.2 rad, its motor driven by MuJoCo's linear
    // quadratic regulator each step, for 15 s: as MuJoCo's to rounding, and
    // the pole upright within a microradian at the end.
    check_local_steps(make_balance(), local.at("balance"));
    Simulation simulation{make_balance()};
    REQUIRE(simulation.configure());
    const Run& theirs = runs.at("balance");
    check_run(record_run(InOut(simulation), theirs.qpos.size() - 1), theirs,
              spreads.at("balance"));
    CHECK(std::abs(simulation.read_qpos()[1]) < 1e-6);
  }
}

}  // namespace simon::robotic
