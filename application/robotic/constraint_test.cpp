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

// Contacts, limits and dry friction by simon and by MuJoCo, against the
// table reference/mujoco_constraints.py recorded, and against physics.
namespace simon::robotic {

namespace {

constexpr std::string_view MODELS = "application/robotic/models/";
constexpr std::string_view HUMANOID = "3rd_party/mujoco/humanoid.xml";

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
  std::vector<std::vector<double>> qpos;
  std::vector<std::vector<double>> qvel;
};

auto load_runs() -> std::map<std::string, Run> {
  std::ifstream file{"application/robotic/reference/mujoco_constraints.csv"};
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
  std::size_t steps = 1500;
};

auto step_for(InOut<Simulation> simulation, double seconds) -> void {
  double h = simulation->mechanics().model().physics.timestep;
  auto dt = std::chrono::nanoseconds{std::llround(h * 1e9)};
  auto steps = static_cast<std::size_t>(std::llround(seconds / h));
  for (std::size_t k = 0; k < steps; ++k) {
    REQUIRE(simulation->step(
        framework::Step{.time = framework::TimePoint{} + k * dt, .dt = dt}));
  }
}

// Steps `simulation` through every step of `run`, and the largest
// differences from it.
auto compare(InOut<Simulation> simulation, const Run& run, std::size_t steps)
    -> std::pair<double, double> {
  auto dt = std::chrono::nanoseconds{
      std::llround(simulation->mechanics().model().physics.timestep * 1e9)};
  double position = 0.0;
  double velocity = 0.0;
  for (std::size_t k = 0; k <= std::min(steps, run.qpos.size() - 1); ++k) {
    if (k > 0) {
      REQUIRE(simulation->step(framework::Step{
          .time = framework::TimePoint{} + (k - 1) * dt, .dt = dt}));
    }
    std::vector<double> q = simulation->read_qpos();
    std::vector<double> v = simulation->read_qvel();
    for (std::size_t i = 0; i < q.size(); ++i) {
      position = std::max(position, std::abs(q[i] - run.qpos[k][i]));
    }
    for (std::size_t i = 0; i < v.size(); ++i) {
      velocity = std::max(velocity, std::abs(v[i] - run.qvel[k][i]));
    }
  }
  return {position, velocity};
}

}  // namespace

TEST_CASE("ConstraintsAgainstMuJoCo") {
  SECTION("ShouldStepAsMuJoCoDoesGivenContactsLimitsAndFriction") {
    // Newton's method to 1e-11 m and 1e-10 m/s of MuJoCo's at every step;
    // PGS likewise where it converges, else to its tolerance, its sweeps
    // ending a step apart by rounding: a stack over its first 0.4 s.
    std::map<std::string, Run> runs = load_runs();
    using Solver = model::Physics::Solver;
    using Cone = model::Physics::Cone;
    using Integrator = model::Physics::Integrator;
    std::vector<double> driven;
    for (int k = 0; k < 21; ++k) {
      driven.push_back(0.3 * ((7 * k) % 11 - 5) / 5);
    }
    // clang-format off
    std::vector<Case> cases{
        {"rolling", "rolling.xml", {.qvel = {{0, 2.0}}}, 1e-11, 1e-10},
        {"sliding", "sliding.xml", {.qvel = {{0, 2.0}, {7, 1.0}}}, 1e-12,
         1e-12},
        {"stack", "boxes.xml", {}, 1e-12, 1e-12},
        {"limits", "limits.xml",
         {.qpos = {{0, 0.3}, {5, -0.1}},
          .qvel = {{0, 3.0}, {1, 2.0}, {2, -1.0}, {4, 1.5}, {5, 2.0}}},
         1e-12, 1e-12},
        {"rolling by PGS", "rolling.xml",
         {.qvel = {{0, 2.0}}, .solver = Solver::PGS}, 1e-12, 1e-12},
        {"sliding by PGS", "sliding.xml",
         {.qvel = {{0, 2.0}, {7, 1.0}}, .solver = Solver::PGS}, 1e-6, 1e-4},
        {"stack by PGS", "boxes.xml", {.solver = Solver::PGS}, 1e-7, 1e-5,
         200},
        {"limits by PGS", "limits.xml",
         {.qpos = {{0, 0.3}, {5, -0.1}},
          .qvel = {{0, 3.0}, {1, 2.0}, {2, -1.0}, {4, 1.5}, {5, 2.0}},
          .solver = Solver::PGS}, 1e-12, 1e-12},
        {"humanoid falling", "humanoid", {}, 1e-12, 1e-10, 400},
        {"humanoid driven", "humanoid", {.control = driven}, 1e-12, 1e-10,
         400},
        {"humanoid falling by PGS", "humanoid", {.solver = Solver::PGS}, 1e-12,
         1e-10, 400},
        {"rolling elliptic", "rolling.xml",
         {.qvel = {{0, 2.0}}, .cone = Cone::ELLIPTIC}, 1e-12, 1e-12},
        {"sliding elliptic", "sliding.xml",
         {.qvel = {{0, 2.0}, {7, 1.0}}, .cone = Cone::ELLIPTIC}, 1e-6, 1e-4},
        {"stack elliptic", "boxes.xml", {.cone = Cone::ELLIPTIC}, 1e-12,
         1e-12},
        {"humanoid driven elliptic", "humanoid",
         {.control = driven, .cone = Cone::ELLIPTIC}, 1e-12, 1e-10, 400},
        {"rolling elliptic by PGS", "rolling.xml",
         {.qvel = {{0, 2.0}}, .solver = Solver::PGS, .cone = Cone::ELLIPTIC},
         1e-12, 1e-12},
        {"sliding elliptic by PGS", "sliding.xml",
         {.qvel = {{0, 2.0}, {7, 1.0}},
          .solver = Solver::PGS,
          .cone = Cone::ELLIPTIC},
         1e-6, 1e-4},        {"arm implicitfast", "arm.xml",
         {.qpos = {{0, 0.3}, {1, -0.5}},
          .control = {0.8, -1.2, 2.0, 0.5},
          .integrator = Integrator::IMPLICIT_FAST},
         1e-15, 1e-15},
        {"free body implicitfast", "free_body.xml",
         {.qvel = {{0, 0.3}, {1, -0.1}, {2, 0.2}, {3, 4.0}, {4, 0.5}, {5, 1.5}},
          .integrator = Integrator::IMPLICIT_FAST},
         1e-14, 1e-14},
        {"rolling implicitfast", "rolling.xml",
         {.qvel = {{0, 2.0}}, .integrator = Integrator::IMPLICIT_FAST}, 1e-12,
         1e-12},
        {"humanoid driven implicitfast", "humanoid",
         {.control = driven, .integrator = Integrator::IMPLICIT_FAST}, 1e-12,
         1e-10, 400},
    };
    // clang-format on
    for (Case& c : cases) {
      CAPTURE(c.name);
      c.scenario.model = c.file == "humanoid" ? std::string{HUMANOID}
                                              : std::string{MODELS} + c.file;
      Simulation simulation{c.scenario};
      auto configured = simulation.configure();
      if (!configured) {
        FAIL(configured.error().message());
      }
      auto [position, velocity] =
          compare(InOut(simulation), runs.at(c.name), c.steps);
      CAPTURE(position, velocity);
      CHECK(position < c.position);
      CHECK(velocity < c.velocity);
    }
  }
}

TEST_CASE("ConstraintsAgainstPhysics") {
  using Solver = model::Physics::Solver;
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
