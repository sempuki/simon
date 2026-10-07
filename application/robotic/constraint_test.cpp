// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <vector>

#include "application/robotic/simulation.hpp"
#include "application/robotic/testing.hpp"
#include "base/testing.hpp"

// Contacts, limits and dry friction by simon and by MuJoCo, against the
// table reference/mujoco_constraints.py recorded, and against physics.
namespace simon::robotic {

namespace {

using namespace testing;

struct Case final {
  std::string name;
  std::string file;
  Scenario scenario;
};

}  // namespace

TEST_CASE("ConstraintsAgainstMuJoCo") {
  SECTION("ShouldStepAsMuJoCoDoesGivenContactsLimitsAndFriction") {
    // Newton's method and PGS, pyramidal and elliptic cones, Euler and
    // implicitfast: each step from MuJoCo's states, and every step of every
    // run, as MuJoCo's to rounding. Where a contact starts exactly touching,
    // or a stack's boxes slip, rounding alone moves MuJoCo's own run far,
    // and the spread it measures says how far.
    std::map<std::string, Run, std::less<>> runs =
        load_runs("mujoco_constraints.csv");
    using Solver = articulated::Physics::Solver;
    using Cone = articulated::Physics::Cone;
    using Integrator = articulated::Physics::Integrator;
    std::vector<double> driven;
    for (int k = 0; k < 21; ++k) {
      driven.push_back(0.3 * ((7 * k) % 11 - 5) / 5);
    }
    // clang-format off
    std::vector<Case> cases{
        {"rolling", "rolling.xml", {.qvel = {{0, 2.0}}}},
        {"sliding", "sliding.xml", {.qvel = {{0, 2.0}, {7, 1.0}}}},
        {"stack", "boxes.xml", {}},
        {"limits", "limits.xml",
         {.qpos = {{0, 0.3}, {5, -0.1}},
          .qvel = {{0, 3.0}, {1, 2.0}, {2, -1.0}, {4, 1.5}, {5, 2.0}}}},
        {"rolling by PGS", "rolling.xml",
         {.qvel = {{0, 2.0}}, .solver = Solver::PGS}},
        {"sliding by PGS", "sliding.xml",
         {.qvel = {{0, 2.0}, {7, 1.0}}, .solver = Solver::PGS}},
        {"stack by PGS", "boxes.xml", {.solver = Solver::PGS}},
        {"limits by PGS", "limits.xml",
         {.qpos = {{0, 0.3}, {5, -0.1}},
          .qvel = {{0, 3.0}, {1, 2.0}, {2, -1.0}, {4, 1.5}, {5, 2.0}},
          .solver = Solver::PGS}},
        {"humanoid falling", "humanoid", {}},
        {"humanoid driven", "humanoid", {.control = driven}},
        {"humanoid falling by PGS", "humanoid", {.solver = Solver::PGS}},
        {"rolling elliptic", "rolling.xml",
         {.qvel = {{0, 2.0}}, .cone = Cone::ELLIPTIC}},
        {"sliding elliptic", "sliding.xml",
         {.qvel = {{0, 2.0}, {7, 1.0}}, .cone = Cone::ELLIPTIC}},
        {"stack elliptic", "boxes.xml", {.cone = Cone::ELLIPTIC}},
        {"humanoid driven elliptic", "humanoid",
         {.control = driven, .cone = Cone::ELLIPTIC}},
        {"rolling elliptic by PGS", "rolling.xml",
         {.qvel = {{0, 2.0}}, .solver = Solver::PGS, .cone = Cone::ELLIPTIC}},
        {"sliding elliptic by PGS", "sliding.xml",
         {.qvel = {{0, 2.0}, {7, 1.0}},
          .solver = Solver::PGS,
          .cone = Cone::ELLIPTIC}},        {"arm implicitfast", "arm.xml",
         {.qpos = {{0, 0.3}, {1, -0.5}},
          .control = {0.8, -1.2, 2.0, 0.5},
          .integrator = Integrator::IMPLICIT_FAST}},
        {"free body implicitfast", "free_body.xml",
         {.qvel = {{0, 0.3}, {1, -0.1}, {2, 0.2}, {3, 4.0}, {4, 0.5}, {5, 1.5}},
          .integrator = Integrator::IMPLICIT_FAST}},
        {"rolling implicitfast", "rolling.xml",
         {.qvel = {{0, 2.0}}, .integrator = Integrator::IMPLICIT_FAST}},
        {"humanoid driven implicitfast", "humanoid",
         {.control = driven, .integrator = Integrator::IMPLICIT_FAST}},
    };
    // clang-format on
    auto local = load_local_steps();
    auto spreads = load_spreads();
    auto solved = load_solved_runs();
    for (Case& c : cases) {
      CAPTURE(c.name);
      c.scenario.model = c.file == "humanoid" ? std::string{HUMANOID}
                                              : std::string{MODELS} + c.file;
      check_local_steps(c.scenario, local.at(c.name));
      Simulation simulation{c.scenario};
      auto configured = simulation.configure();
      if (!configured) {
        FAIL(configured.error().message());
      }
      const Run& theirs = runs.at(c.name);
      check_run(record_run(InOut(simulation), theirs.qpos.size() - 1), theirs,
                spreads.at(c.name), &solved.at(c.name));
    }
  }
}

TEST_CASE("ConstraintsAgainstPhysics") {
  using Solver = articulated::Physics::Solver;
  const std::vector<Solver> solvers{Solver::NEWTON, Solver::PGS};

  SECTION("ShouldSolveTreesApartGivenNoContactBetweenThem") {
    // The box and the log touch only the floor, so each is its own
    // island; a stack's boxes touch each other, so they are one.
    auto islands = [](const std::string& file) {
      Simulation simulation{Scenario{.model = std::string{MODELS} + file}};
      REQUIRE(simulation.configure());
      step_for(InOut(simulation), 0.01);
      std::vector<std::uint32_t> found;
      simulation.world().store_of<Island>().for_each(
          [&](framework::Entity, const Island& island) {
            found.push_back(island.index);
          });
      std::ranges::sort(found);
      return found;
    };
    CHECK(islands("sliding.xml") == std::vector<std::uint32_t>{0, 1});
    CHECK(islands("boxes.xml") == std::vector<std::uint32_t>{0, 0, 0, 0, 0});
  }

  SECTION("ShouldComeToRestGivenAFallenHumanoid") {
    // MuJoCo's humanoid falls from standing, slumps, and lies on the
    // floor, as MuJoCo's does: 0.070 m high at 20 s, still settling.
    Simulation simulation{Scenario{.model = std::string{HUMANOID}}};
    REQUIRE(simulation.configure());
    step_for(InOut(simulation), 20.0);
    std::vector<double> q = simulation.read_qpos();
    std::vector<double> v = simulation.read_qvel();
    double speed = 0.0;
    for (double x : v) {
      speed = std::max(speed, std::abs(x));
    }
    CAPTURE(q[2], speed);
    CHECK(q[2] < 0.1);
    CHECK(speed < 0.1);
  }

  SECTION("ShouldSlideAsFarAsCoulombFrictionAllows") {
    for (Solver solver : solvers) {
      Simulation simulation{
          Scenario{.model = std::string{MODELS} + "sliding.xml",
                   .qvel = {{0, 2.0}},
                   .solver = solver}};
      REQUIRE(simulation.configure());
      step_for(InOut(simulation), 2.0);
      double coulomb = 2.0 * 2.0 / (2 * 0.4 * 9.81);
      double slid = simulation.read_qpos()[0];
      CAPTURE(slid, coulomb);
      CHECK(std::abs(slid - coulomb) < 0.005 * coulomb);
    }
  }

  SECTION("ShouldRollAtFiveSeventhsOfItsSpeed") {
    for (Solver solver : solvers) {
      Simulation simulation{
          Scenario{.model = std::string{MODELS} + "rolling.xml",
                   .qpos = {{2, 0.1}},
                   .qvel = {{0, 2.0}},
                   .solver = solver}};
      REQUIRE(simulation.configure());
      step_for(InOut(simulation), 3.0);
      std::vector<double> v = simulation.read_qvel();
      CAPTURE(v[0], v[4]);
      CHECK(std::abs(v[0] - 2.0 * 5 / 7) < 0.001 * 2.0 * 5 / 7);
      CHECK(std::abs(v[4] * 0.1 - v[0]) < 0.001 * v[0]);
    }
  }

  SECTION("ShouldRestGivenAStack") {
    for (Solver solver : solvers) {
      Simulation simulation{Scenario{.model = std::string{MODELS} + "boxes.xml",
                                     .solver = solver}};
      REQUIRE(simulation.configure());
      step_for(InOut(simulation), 4.0);
      std::vector<double> q = simulation.read_qpos();
      std::vector<double> v = simulation.read_qvel();
      double sink = 0.0;
      double speed = 0.0;
      for (std::size_t b = 0; b < 5; ++b) {
        sink = std::max(sink, std::abs(q[7 * b + 2] - (0.1 + 0.2 * b)));
        for (std::size_t k = 0; k < 6; ++k) {
          speed = std::max(speed, std::abs(v[6 * b + k]));
        }
      }
      CAPTURE(sink, speed);
      CHECK(sink < 0.0025);
      CHECK(speed < (solver == Solver::NEWTON ? 1e-9 : 1e-2));
    }
  }
}

}  // namespace simon::robotic
