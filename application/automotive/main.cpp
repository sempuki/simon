// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

// Drives a scenario headless and prints where its traffic got to:
//
//   bazel run //application/automotive -- [roads.xodr] [vehicles] [seed]
//       [pedestrians] [--seconds=N]
//
// Its arguments are the viewer's, in the same order.

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <string_view>

#include "application/arguments.hpp"
#include "application/automotive/simulation.hpp"
#include "core/argument.hpp"
#include "engine/driver.hpp"

// How the program is called.
constexpr std::string_view USAGE =
    "automotive [roads.xodr] [vehicles] [seed] [pedestrians] [--seconds=N]";

auto main(int argc, char** argv) -> int {
  using namespace simon;
  using namespace std::chrono_literals;
  automotive::Scenario scenario;
  application::Arguments arguments{argc, argv};
  scenario.roads = std::string{arguments.text(0, scenario.roads)};
  scenario.vehicles = static_cast<int>(arguments.integer(1, scenario.vehicles));
  scenario.seed = static_cast<std::uint64_t>(
      arguments.integer(2, static_cast<std::int64_t>(scenario.seed)));
  scenario.pedestrians =
      static_cast<int>(arguments.integer(3, scenario.pedestrians));
  auto seconds = static_cast<int>(arguments.option_integer("seconds", 300));
  if (arguments.report_error(USAGE)) {
    return EXIT_FAILURE;
  }
  automotive::Simulation simulation{scenario};
  engine::BatchDriver<automotive::Simulation> driver{
      engine::Timing{.max_step = 100ms}, Depend(simulation)};
  auto end = driver.run(TimePoint{std::chrono::seconds{seconds}});
  if (!end) {
    std::cerr << "Error: " << end.error().message() << "\n";
    return EXIT_FAILURE;
  }
  double speeds = 0.0;
  std::size_t vehicles = 0;
  simulation.world().store_of<automotive::LaneState>().for_each(
      [&](framework::Entity, const automotive::LaneState& state) {
        speeds += state.speed.numerical_value_in(meter_per_second);
        ++vehicles;
      });
  std::cout << vehicles << " vehicles after " << seconds
            << " s, at a mean speed of " << speeds / vehicles << " m/s\n";
  return EXIT_SUCCESS;
}
