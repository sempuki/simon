// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

// Drives a scenario headless and prints where its traffic got to:
//
//   bazel run //application/automotive -- [roads.xodr] [vehicles] [seconds]
//       [seed]

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>

#include "application/automotive/simulation.hpp"
#include "engine/driver.hpp"
#include "framework/vocabulary.hpp"

auto main(int argc, char** argv) -> int {
  using namespace simon;
  using namespace std::chrono_literals;
  automotive::Scenario scenario;
  if (argc > 1) {
    scenario.roads = argv[1];
  }
  if (argc > 2) {
    scenario.vehicles = std::atoi(argv[2]);
  }
  int seconds = argc > 3 ? std::atoi(argv[3]) : 300;
  if (argc > 4) {
    scenario.seed = std::strtoull(argv[4], nullptr, 10);
  }
  automotive::Simulation simulation{scenario};
  engine::BatchDriver<automotive::Simulation> driver{
      engine::Timing{.max_step = 100ms}, Depend(simulation)};
  auto end = driver.run(framework::TimePoint{std::chrono::seconds{seconds}});
  if (!end) {
    std::cerr << "Error: " << end.error().message() << "\n";
    return EXIT_FAILURE;
  }
  double speeds = 0.0;
  std::size_t vehicles = 0;
  simulation.world().store_of<automotive::LaneState>().for_each(
      [&](framework::Entity, const automotive::LaneState& state) {
        speeds += state.speed.numerical_value_in(model::meter_per_second);
        ++vehicles;
      });
  std::cout << vehicles << " vehicles after " << seconds
            << " s, at a mean speed of " << speeds / vehicles << " m/s\n";
  return EXIT_SUCCESS;
}
