// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

// Times the flight simulation at growing populations, system by system: with
// every aircraft on the single-pass model, with every aircraft opted in to
// Runge-Kutta 4, and mixed, with 1% on Runge-Kutta 4 and 0.1% rigid 737s.
//
//   bazel run -c opt //application/aeronautic:aeronautic_benchmark [-- --steps
//   N]
//       [--contend[=N]] [aircraft...]
//
// Traffic density is the same at every size. Each system's line ends with the
// bytes its loop can read per entity (framework::bytes_per_entity_v); for
// Precise that is the sum over its stages' systems, not a per-stage cost.
// Each system runs in its own single-system scheduler, in schedule order,
// against one world.

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <expected>
#include <optional>
#include <print>
#include <string>
#include <string_view>
#include <vector>

#include "application/aeronautic/simulation.hpp"
#include "base/core.hpp"
#include "core/argument.hpp"
#include "format/aircraft_file.hpp"
#include "framework/benchmarking.hpp"

namespace simon::aeronautic {
namespace {

using namespace std::chrono_literals;

constexpr Duration DT = 20ms;
constexpr int DEFAULT_STEPS = 500;  // 10 s simulated.

// The models the aircraft fly: all single pass, all Runge-Kutta 4, or mixed,
// with 1% on Runge-Kutta 4 and 0.1% rigid.
enum class Fidelity { SINGLE_PASS, RUNGE_KUTTA, MIXED };

auto measure(int aircraft, Fidelity fidelity, int steps,
             const model::AircraftData& rigid) -> void {
  Scenario scenario{.aircraft = aircraft};
  if (fidelity == Fidelity::RUNGE_KUTTA) {
    scenario.precise = aircraft;
  } else if (fidelity == Fidelity::MIXED) {
    scenario.precise = aircraft / 100;
    scenario.rigid = std::max(aircraft / 1000, 1);
  }
  World world;
  std::expected<void, framework::Status> built =
      build_world(scenario, Out(world));
  CHECK_POSTCONDITION(built.has_value());
  built =
      build_scenario(scenario, RigidTypes{.airliner = &rigid}, InOut(world));
  CHECK_POSTCONDITION(built.has_value());
  world.sync();

  framework::benchmark::SystemTimerFor<World, Scheduler::FlattenedSystemList>
      timer;
  for (int i = 0; i < steps; ++i) {
    timer.step(Step{.time = TimePoint{} + i * DT, .dt = DT}, InOut(world));
  }
  std::println("\n{} aircraft, {}: {} steps", aircraft,
               fidelity == Fidelity::SINGLE_PASS ? "single pass"
               : fidelity == Fidelity::RUNGE_KUTTA
                   ? "Runge-Kutta 4"
                   : "mixed: " + std::to_string(scenario.precise) +
                         " Runge-Kutta 4, " + std::to_string(scenario.rigid) +
                         " rigid",
               steps);
  timer.print(static_cast<double>(aircraft) * steps);
}

}  // namespace
}  // namespace simon::aeronautic

// aeronautic_benchmark [--steps N] [--contend[=N]] [aircraft...]
auto main(int argc, char** argv) -> int {
  using simon::framework::benchmark::Contention;
  using simon::framework::benchmark::parse_count;
  auto arguments = simon::framework::benchmark::parse_arguments(argc, argv);
  if (!arguments) {
    std::println(stderr, "{}", arguments.error());
    return 1;
  }
  int steps = arguments->steps.value_or(simon::aeronautic::DEFAULT_STEPS);
  std::vector<int> populations;
  for (std::string_view argument : arguments->rest) {
    std::optional<int> count = parse_count(argument);
    if (!count) {
      std::println(stderr, "unknown argument: {}", argument);
      return 1;
    }
    populations.push_back(*count);
  }
  if (populations.empty()) {
    populations = {1'000, 10'000, 100'000};
  }
  auto rigid = simon::format::load_aircraft("3rd_party/jsbsim/737.aircraft");
  if (!rigid) {
    std::println(stderr, "{}", rigid.error().message());
    return 1;
  }
  Contention contention{arguments->threads};
  std::println("{}", Contention::describe(arguments->threads));
  using simon::aeronautic::Fidelity;
  for (int aircraft : populations) {
    for (Fidelity fidelity :
         {Fidelity::SINGLE_PASS, Fidelity::RUNGE_KUTTA, Fidelity::MIXED}) {
      simon::aeronautic::measure(aircraft, fidelity, steps, *rigid);
    }
  }
}
