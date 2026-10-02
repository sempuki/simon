// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

// Times the flight simulation at growing populations, system by system: with
// every aircraft on the single-pass model, with every aircraft opted in to
// Runge-Kutta 4, and mixed, with 1% on Runge-Kutta 4 and 0.1% rigid 737s.
//
//   bazel run -c opt //application/flight:flight_benchmark [-- --steps N]
//       [--contend[=N]] [aircraft...]
//
// Traffic density is the same at every size. Each system's line ends with the
// bytes its loop can read per entity (framework::bytes_per_entity_v); for
// Precise that is the sum over its stages' systems, not a per-stage cost.
// Each system runs in its own single-system scheduler, in schedule order,
// against one world.

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <expected>
#include <optional>
#include <print>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

#include "application/flight/simulation.hpp"
#include "base/core.hpp"
#include "framework/benchmarking.hpp"
#include "framework/type_list.hpp"
#include "framework/vocabulary.hpp"

namespace simon::flight {
namespace {

using namespace std::chrono_literals;
using WallClock = std::chrono::steady_clock;

constexpr Duration DT = 20ms;
constexpr int DEFAULT_STEPS = 500;  // 10 s simulated.

template <typename... SystemTypes>
auto schedulers_of(framework::TypeList<SystemTypes...>) {
  return std::tuple<framework::Scheduler<World, SystemList<SystemTypes>>...>{};
}

template <typename... SystemTypes>
auto names_of(framework::TypeList<SystemTypes...>)
    -> std::array<std::string, sizeof...(SystemTypes)> {
  auto unqualified = [](const std::string& name) {
    std::size_t colons = name.rfind("::");
    return colons == std::string::npos ? name : name.substr(colons + 2);
  };
  // A Continuous element by the state it integrates, the rest by name.
  auto short_name = [&](const std::string& name) {
    std::size_t open = name.find('<');
    std::string head =
        unqualified(open == std::string::npos ? name : name.substr(0, open));
    std::size_t states = name.find("TypeList<");
    if (head == "Continuous" && states != std::string::npos) {
      std::size_t from = states + std::string_view{"TypeList<"}.size();
      std::size_t to = name.find_first_of(",>", from);
      return head + "(" + unqualified(name.substr(from, to - from)) + ")";
    }
    return head;
  };
  return {short_name(lib::to_type_string<SystemTypes>())...};
}

template <typename... SystemTypes>
auto bytes_of(framework::TypeList<SystemTypes...>)
    -> std::array<std::size_t, sizeof...(SystemTypes)> {
  return {framework::bytes_per_entity_v<SystemTypes>...};
}

// The models the aircraft fly: all single pass, all Runge-Kutta 4, or mixed,
// with 1% on Runge-Kutta 4 and 0.1% rigid.
enum class Fidelity { SINGLE_PASS, RUNGE_KUTTA, MIXED };

auto measure(int aircraft, Fidelity fidelity, int steps,
             const model::AircraftData& rigid) -> void {
  using List = Scheduler::FlattenedSystemList;
  constexpr std::size_t SYSTEM_COUNT = List::size;

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
  built = build_scenario(scenario, &rigid, InOut(world));
  CHECK_POSTCONDITION(built.has_value());
  world.sync();

  auto schedulers = schedulers_of(List{});
  std::array<double, SYSTEM_COUNT> seconds{};
  for (int i = 0; i < steps; ++i) {
    framework::Step step{.time = TimePoint{} + i * DT, .dt = DT};
    std::size_t index = 0;
    std::apply(
        [&](auto&... scheduler) {
          (([&] {
             auto start = WallClock::now();
             scheduler.step(step, InOut(world));
             seconds[index++] +=
                 std::chrono::duration<double>(WallClock::now() - start)
                     .count();
           }()),
           ...);
        },
        schedulers);
  }

  double total = 0.0;
  for (double value : seconds) total += value;
  double entity_steps = static_cast<double>(aircraft) * std::max(steps, 1);
  std::println("\n{} aircraft, {}: {} steps", aircraft,
               fidelity == Fidelity::SINGLE_PASS ? "single pass"
               : fidelity == Fidelity::RUNGE_KUTTA
                   ? "Runge-Kutta 4"
                   : "mixed: " + std::to_string(scenario.precise) +
                         " Runge-Kutta 4, " + std::to_string(scenario.rigid) +
                         " rigid",
               steps);
  std::println("  total {:10.3f} ms/step {:10.1f} ns/entity-step",
               1e3 * total / std::max(steps, 1), 1e9 * total / entity_steps);
  auto names = names_of(List{});
  auto bytes = bytes_of(List{});
  for (std::size_t i = 0; i < SYSTEM_COUNT; ++i) {
    std::println("  {:<22} {:10.3f} ms/step {:6.1f}% {:6} B/entity", names[i],
                 1e3 * seconds[i] / std::max(steps, 1),
                 total > 0.0 ? 100.0 * seconds[i] / total : 0.0, bytes[i]);
  }
}

}  // namespace
}  // namespace simon::flight

// flight_benchmark [--steps N] [--contend[=N]] [aircraft...]
auto main(int argc, char** argv) -> int {
  using simon::framework::benchmark::Contention;
  using simon::framework::benchmark::parse_count;
  int steps = simon::flight::DEFAULT_STEPS;
  unsigned threads = 0;
  std::vector<int> populations;
  for (int i = 1; i < argc; ++i) {
    std::string_view argument{argv[i]};
    std::optional<int> count;
    if (argument == "--steps" && i + 1 < argc &&
        (count = parse_count(argv[i + 1]))) {
      steps = *count;
      ++i;
    } else if (auto asked = Contention::threads_from(argument)) {
      threads = *asked;
    } else if ((count = parse_count(argument))) {
      populations.push_back(*count);
    } else {
      std::println(stderr, "unknown argument: {}", argument);
      return 1;
    }
  }
  if (populations.empty()) {
    populations = {1'000, 10'000, 100'000};
  }
  auto rigid =
      simon::model::load_aircraft("application/flight/aircraft/737.aircraft");
  if (!rigid) {
    std::println(stderr, "{}", rigid.error().message());
    return 1;
  }
  Contention contention{threads};
  std::println("{}", Contention::describe(threads));
  using simon::flight::Fidelity;
  for (int aircraft : populations) {
    for (Fidelity fidelity :
         {Fidelity::SINGLE_PASS, Fidelity::RUNGE_KUTTA, Fidelity::MIXED}) {
      simon::flight::measure(aircraft, fidelity, steps, *rigid);
    }
  }
}
