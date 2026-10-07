// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "application/robotic/simulation.hpp"
#include "application/testing.hpp"
#include "base/testing.hpp"
#include "core/vocabulary.hpp"

// Shared by the robotic tests: where the models and MuJoCo's tables lie,
// MuJoCo's runs read from them, and simon stepped against them.
namespace simon::robotic::testing {

using simon::testing::load_table;
using simon::testing::parse_numbers;

inline constexpr std::string_view MODELS = "application/robotic/models/";
inline constexpr std::string_view REFERENCE = "application/robotic/reference/";
inline constexpr std::string_view HUMANOID = "3rd_party/mujoco/humanoid.xml";
inline constexpr std::string_view MENAGERIE = "3rd_party/menagerie/";

// A case MuJoCo ran: where it started, its positions and controls, if the
// table gives them, and its positions and velocities after each step, step
// 0 the start.
struct Run final {
  std::vector<double> start;
  std::vector<double> control;
  std::vector<std::vector<double>> qpos;
  std::vector<std::vector<double>> qvel;
};

// The cases of MuJoCo's table `name`, by name: a line of each case's
// positions and velocities after each step, and a line with the step
// "start" for its starting positions and controls.
inline auto load_runs(std::string_view name)
    -> std::map<std::string, Run, std::less<>> {
  std::map<std::string, Run, std::less<>> runs;
  for (std::vector<std::string>& cells :
       load_table(std::string{REFERENCE} + std::string{name}).lines) {
    REQUIRE(cells.size() >= 4);
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

// Steps `simulation` one model step, the `k`th from the start.
inline auto step_once(InOut<Simulation> simulation, std::size_t k) -> void {
  auto dt = std::chrono::nanoseconds{
      std::llround(simulation->mechanics().model().physics.timestep * 1e9)};
  REQUIRE(simulation->step(Step{.time = TimePoint{} + k * dt, .dt = dt}));
}

// Steps `simulation` for `seconds` of model steps from the start.
inline auto step_for(InOut<Simulation> simulation, double seconds) -> void {
  double h = simulation->mechanics().model().physics.timestep;
  auto steps = static_cast<std::size_t>(std::llround(seconds / h));
  for (std::size_t k = 0; k < steps; ++k) {
    step_once(simulation, k);
  }
}

// How far apart two states are: the largest difference in a position and
// in a velocity.
struct Apart final {
  double position = 0.0;
  double velocity = 0.0;
};

inline auto measure_apart(const std::vector<double>& qpos,
                          const std::vector<double>& qvel,
                          const std::vector<double>& their_qpos,
                          const std::vector<double>& their_qvel) -> Apart {
  REQUIRE(qpos.size() == their_qpos.size());
  REQUIRE(qvel.size() == their_qvel.size());
  Apart apart;
  for (std::size_t i = 0; i < qpos.size(); ++i) {
    apart.position =
        std::max(apart.position, std::abs(qpos[i] - their_qpos[i]));
  }
  for (std::size_t i = 0; i < qvel.size(); ++i) {
    apart.velocity =
        std::max(apart.velocity, std::abs(qvel[i] - their_qvel[i]));
  }
  return apart;
}

// How far simon may differ from MuJoCo and still differ only by rounding: a
// few times as far as MuJoCo moves when its own numbers are nudged by a few
// units in the last place (reference/mujoco_checks.py), and never less than
// `floor`.
inline constexpr double SPREADS = 10.0;

inline auto is_within(Apart apart, Apart spread, Apart floor) -> bool {
  return apart.position <= SPREADS * spread.position + floor.position &&
         apart.velocity <= SPREADS * spread.velocity + floor.velocity;
}

// Floors for one step, and for a whole run.
inline constexpr Apart LOCAL_FLOOR{.position = 1e-13, .velocity = 1e-12};
inline constexpr Apart RUN_FLOOR{.position = 1e-12, .velocity = 1e-11};

// Steps `simulation` `steps` model steps, recording its positions and
// velocities from the start, step 0, on.
inline auto record_run(InOut<Simulation> simulation, std::size_t steps) -> Run {
  Run run;
  for (std::size_t k = 0; k <= steps; ++k) {
    if (k > 0) {
      step_once(simulation, k - 1);
    }
    run.qpos.push_back(simulation->read_qpos());
    run.qvel.push_back(simulation->read_qvel());
  }
  return run;
}

// How far apart two runs are at step `k`.
inline auto measure_apart_at(const Run& ours, const Run& theirs, std::size_t k)
    -> Apart {
  return measure_apart(ours.qpos[k], ours.qvel[k], theirs.qpos[k],
                       theirs.qvel[k]);
}

// The largest differences between two runs over their common steps.
inline auto find_largest_apart(const Run& ours, const Run& theirs) -> Apart {
  Apart largest;
  std::size_t steps = std::min(ours.qpos.size(), theirs.qpos.size());
  for (std::size_t k = 0; k < steps; ++k) {
    Apart apart = measure_apart_at(ours, theirs, k);
    largest.position = std::max(largest.position, apart.position);
    largest.velocity = std::max(largest.velocity, apart.velocity);
  }
  return largest;
}

// A run MuJoCo solved to convergence, its states by step, at some steps.
using SolvedRun =
    std::map<std::size_t, std::pair<std::vector<double>, std::vector<double>>>;

// Whether `ours` is no farther from `solved` than `theirs` is, at the
// steps `solved` records: as accurate as MuJoCo, where the two stop
// iterating at different places.
inline auto is_as_accurate(const Run& ours, const Run& theirs,
                           const SolvedRun& solved, Apart floor) -> bool {
  Apart our_error;
  Apart their_error;
  for (const auto& [k, state] : solved) {
    if (k >= ours.qpos.size() || k >= theirs.qpos.size()) {
      continue;
    }
    Apart our =
        measure_apart(ours.qpos[k], ours.qvel[k], state.first, state.second);
    Apart their = measure_apart(theirs.qpos[k], theirs.qvel[k], state.first,
                                state.second);
    our_error.position = std::max(our_error.position, our.position);
    our_error.velocity = std::max(our_error.velocity, our.velocity);
    their_error.position = std::max(their_error.position, their.position);
    their_error.velocity = std::max(their_error.velocity, their.velocity);
  }
  CAPTURE(our_error.position, our_error.velocity, their_error.position,
          their_error.velocity);
  return our_error.position <=
             their_error.position * (1.0 + 1e-6) + floor.position &&
         our_error.velocity <=
             their_error.velocity * (1.0 + 1e-6) + floor.velocity;
}

// Checks every step of `ours` against MuJoCo's run, within the spread
// rounding gives MuJoCo's own run by that step. Given the run solved to
// convergence, a run that leaves the spread passes if it is no farther
// from that than MuJoCo's is.
inline auto check_run(const Run& ours, const Run& theirs,
                      const std::vector<Apart>& spread,
                      const SolvedRun* solved = nullptr) -> void {
  std::size_t steps = std::min(ours.qpos.size(), theirs.qpos.size());
  REQUIRE(spread.size() >= steps);
  for (std::size_t k = 0; k < steps; ++k) {
    Apart apart = measure_apart_at(ours, theirs, k);
    if (!is_within(apart, spread[k], RUN_FLOOR)) {
      CAPTURE(k, apart.position, apart.velocity, spread[k].position,
              spread[k].velocity);
      if (solved == nullptr ||
          !is_as_accurate(ours, theirs, *solved, RUN_FLOOR)) {
        FAIL_CHECK("the run leaves rounding's spread, and is less accurate");
      }
      return;
    }
  }
}

// For each case with constraints, its run solved to convergence.
inline auto load_solved_runs()
    -> std::map<std::string, SolvedRun, std::less<>> {
  std::map<std::string, SolvedRun, std::less<>> runs;
  for (std::vector<std::string>& cells :
       load_table(std::string{REFERENCE} + "mujoco_solved.csv").lines) {
    REQUIRE(cells.size() == 4);
    runs[cells[0]][static_cast<std::size_t>(std::stoull(cells[1]))] = {
        parse_numbers(cells[2]), parse_numbers(cells[3])};
  }
  return runs;
}

// One of MuJoCo's local steps: a state from its run, the state one step
// later from fresh data, and how far rounding moves that step.
struct LocalStep final {
  std::size_t step = 0;
  std::vector<double> qpos;
  std::vector<double> qvel;
  std::vector<double> next_qpos;
  std::vector<double> next_qvel;
  Apart spread;
  // The step solved to convergence, for a case with constraints.
  std::vector<double> solved_qpos;
  std::vector<double> solved_qvel;
};

inline auto load_local_steps()
    -> std::map<std::string, std::vector<LocalStep>, std::less<>> {
  std::map<std::string, std::vector<LocalStep>, std::less<>> steps;
  for (std::vector<std::string>& cells :
       load_table(std::string{REFERENCE} + "mujoco_local.csv").lines) {
    REQUIRE(cells.size() == 10);
    steps[cells[0]].push_back(
        LocalStep{.step = static_cast<std::size_t>(std::stoull(cells[1])),
                  .qpos = parse_numbers(cells[2]),
                  .qvel = parse_numbers(cells[3]),
                  .next_qpos = parse_numbers(cells[4]),
                  .next_qvel = parse_numbers(cells[5]),
                  .spread = {.position = std::stod(cells[6]),
                             .velocity = std::stod(cells[7])},
                  .solved_qpos = parse_numbers(cells[8]),
                  .solved_qvel = parse_numbers(cells[9])});
  }
  return steps;
}

// For each case, by step, how far rounding moves MuJoCo's run by then.
inline auto load_spreads()
    -> std::map<std::string, std::vector<Apart>, std::less<>> {
  std::map<std::string, std::vector<Apart>, std::less<>> spreads;
  for (std::vector<std::string>& cells :
       load_table(std::string{REFERENCE} + "mujoco_spread.csv").lines) {
    REQUIRE(cells.size() == 4);
    spreads[cells[0]].push_back(
        {.position = std::stod(cells[2]), .velocity = std::stod(cells[3])});
  }
  return spreads;
}

// Steps simon once from each of MuJoCo's local steps, from a fresh
// simulation of `scenario` at that state, and checks it lands where
// MuJoCo's did, within that step's spread, or, where the step was also
// solved to convergence, no farther from that than MuJoCo's.
inline auto check_local_steps(const Scenario& scenario,
                              const std::vector<LocalStep>& steps) -> void {
  for (const LocalStep& at : steps) {
    Scenario here = scenario;
    here.qpos.clear();
    here.qvel.clear();
    for (std::uint32_t i = 0; i < at.qpos.size(); ++i) {
      here.qpos.emplace_back(i, at.qpos[i]);
    }
    for (std::uint32_t i = 0; i < at.qvel.size(); ++i) {
      here.qvel.emplace_back(i, at.qvel[i]);
    }
    Simulation simulation{here};
    auto configured = simulation.configure();
    if (!configured) {
      FAIL(configured.error().message());
    }
    step_once(InOut(simulation), 0);
    Apart apart = measure_apart(simulation.read_qpos(), simulation.read_qvel(),
                                at.next_qpos, at.next_qvel);
    if (is_within(apart, at.spread, LOCAL_FLOOR)) {
      continue;
    }
    CAPTURE(at.step, apart.position, apart.velocity, at.spread.position,
            at.spread.velocity);
    if (at.solved_qpos.empty()) {
      FAIL_CHECK("a step leaves rounding's spread");
      continue;
    }
    Apart ours = measure_apart(simulation.read_qpos(), simulation.read_qvel(),
                               at.solved_qpos, at.solved_qvel);
    Apart theirs = measure_apart(at.next_qpos, at.next_qvel, at.solved_qpos,
                                 at.solved_qvel);
    CAPTURE(ours.position, ours.velocity, theirs.position, theirs.velocity);
    CHECK(ours.position <=
          theirs.position * (1.0 + 1e-6) + LOCAL_FLOOR.position);
    CHECK(ours.velocity <=
          theirs.velocity * (1.0 + 1e-6) + LOCAL_FLOOR.velocity);
  }
}

}  // namespace simon::robotic::testing
