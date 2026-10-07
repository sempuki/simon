// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <map>
#include <string>
#include <vector>

#include "application/robotic/simulation.hpp"
#include "application/robotic/testing.hpp"
#include "base/testing.hpp"

// robotic's test models stepped without constraints by simon and by MuJoCo,
// against the tables reference/mujoco_dynamics.py and
// reference/mujoco_checks.py recorded: each step from MuJoCo's own states,
// each whole run within what rounding moves MuJoCo's, and each run's error
// from the converged solution no worse than MuJoCo's.
namespace simon::robotic {

namespace {

using namespace testing;

struct Case final {
  std::string name;
  std::string file;
  Scenario scenario;
};

auto make_cases() -> std::vector<Case> {
  // clang-format off
  std::vector<Case> cases{
      {"pendulum", "pendulum.xml", {.qpos = {{0, 0.7}}}},
      {"double pendulum", "double_pendulum.xml",
       {.qpos = {{0, 1.2}, {1, -0.4}}}},
      {"free body", "free_body.xml",
       {.qvel = {{0, 0.3}, {1, -0.1}, {2, 0.2}, {3, 4.0}, {4, 0.5}, {5, 1.5}}}},
      {"features", "features.xml",
       {.qpos = {{0, 0.1}, {1, 0.9}, {2, 0.2}, {3, -0.3}, {4, 0.25}, {5, 0.3},
                 {6, 0.5}},
        .qvel = {{2, 1.0}}}},
      {"cartpole", "cartpole.xml", {.qpos = {{1, 0.05}}, .control = {0.3}}},
      {"boxes", "boxes.xml", {.qvel = {{7, 0.5}, {23, -1.0}}}},
  };
  // clang-format on
  for (Case& c : cases) {
    c.scenario.model = std::string{MODELS} + c.file;
    c.scenario.constrained = false;
  }
  return cases;
}

auto run_case(const Case& c, std::size_t steps) -> Run {
  Simulation simulation{c.scenario};
  auto configured = simulation.configure();
  if (!configured) {
    FAIL(configured.error().message());
  }
  return record_run(InOut(simulation), steps);
}

}  // namespace

TEST_CASE("DynamicsAgainstMuJoCo") {
  SECTION("ShouldStepAsMuJoCoDoesGivenItsStates") {
    // From 50 of each run's states, one step lands where MuJoCo's does, to
    // rounding.
    auto local = load_local_steps();
    for (const Case& c : make_cases()) {
      CAPTURE(c.name);
      check_local_steps(c.scenario, local.at(c.name));
    }
  }

  SECTION("ShouldRunAsMuJoCoDoesToRounding") {
    // Every step of every run within what nudging MuJoCo's own numbers by
    // a few units in the last place moves its run by then: the double
    // pendulum, chaotic, through all 3,000 steps.
    std::map<std::string, Run, std::less<>> runs =
        load_runs("mujoco_dynamics.csv");
    auto spreads = load_spreads();
    for (const Case& c : make_cases()) {
      CAPTURE(c.name);
      const Run& theirs = runs.at(c.name);
      check_run(run_case(c, theirs.qpos.size() - 1), theirs,
                spreads.at(c.name));
    }
  }

  SECTION("ShouldBeAsAccurateAsMuJoCoGivenConvergedSolution") {
    // Each run's largest error from Runge-Kutta 4 at a fiftieth of the
    // timestep is no more than MuJoCo's own.
    std::map<std::string, Run, std::less<>> runs =
        load_runs("mujoco_dynamics.csv");
    std::map<std::string, Run, std::less<>> converged =
        load_runs("mujoco_converged.csv");
    for (const Case& c : make_cases()) {
      CAPTURE(c.name);
      const Run& theirs = runs.at(c.name);
      const Run& exact = converged.at(c.name);
      Apart ours =
          find_largest_apart(run_case(c, theirs.qpos.size() - 1), exact);
      Apart mujoco = find_largest_apart(theirs, exact);
      CAPTURE(ours.position, ours.velocity, mujoco.position, mujoco.velocity);
      CHECK(ours.position <= mujoco.position * (1.0 + 1e-6) + 1e-12);
      CHECK(ours.velocity <= mujoco.velocity * (1.0 + 1e-6) + 1e-12);
    }
  }
}

}  // namespace simon::robotic
