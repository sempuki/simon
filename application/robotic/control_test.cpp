// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "application/robotic/simulation.hpp"
#include "application/robotic/testing.hpp"
#include "base/testing.hpp"

// Actuators and feedback by simon and by MuJoCo, against the tables
// reference/mujoco_control.py recorded: an arm under position, velocity and
// general actuators, and a cart-pole balanced by MuJoCo's own linear
// quadratic regulator.
namespace simon::robotic {

namespace {

using namespace testing;

}  // namespace

TEST_CASE("ControlAgainstMuJoCo") {
  SECTION("ShouldServoAsMuJoCoDoesGivenEveryActuator") {
    // Position servos, one geared, a velocity servo with damping taken
    // implicitly, and a general actuator against its force range, held at
    // their controls for 3 s: every step equal to MuJoCo's.
    Simulation simulation{Scenario{.model = std::string{MODELS} + "arm.xml",
                                   .qpos = {{0, 0.3}, {1, -0.5}},
                                   .control = {0.8, -1.2, 2.0, 0.5},
                                   .constrained = false}};
    REQUIRE(simulation.configure());
    auto [position, velocity] = compare_run(
        InOut(simulation), load_runs("mujoco_control.csv").at("arm"));
    CAPTURE(position, velocity);
    CHECK(position < 1e-15);
    CHECK(velocity < 1e-15);
  }

  SECTION("ShouldBalanceAsMuJoCoDoesGivenFeedback") {
    // The cart-pole from 0.2 rad, its motor driven by MuJoCo's linear
    // quadratic regulator each step, for 15 s: every step equal to
    // MuJoCo's, and the pole upright within a microradian at the end.
    std::vector<std::string> law =
        load_table(std::string{REFERENCE} + "mujoco_feedback.csv").lines.at(0);
    Simulation simulation{
        Scenario{.model = std::string{MODELS} + "cartpole.xml",
                 .qpos = {{1, 0.2}},
                 .feedback = {.gains = parse_numbers(law[1]),
                              .reference = parse_numbers(law[2]),
                              .offset = parse_numbers(law[3])},
                 .constrained = false}};
    REQUIRE(simulation.configure());
    auto [position, velocity] = compare_run(
        InOut(simulation), load_runs("mujoco_control.csv").at("balance"));
    CAPTURE(position, velocity);
    CHECK(position < 1e-15);
    CHECK(velocity < 1e-15);
    CHECK(std::abs(simulation.read_qpos()[1]) < 1e-6);
  }
}

}  // namespace simon::robotic
