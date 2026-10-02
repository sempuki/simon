// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

// Flies one flight scenario as fast as possible and prints what it did:
//
//   bazel run //application/flight -- [aircraft] [precise] [rigid] [seed]

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <print>

#include "application/flight/simulation.hpp"
#include "engine/driver.hpp"
#include "framework/vocabulary.hpp"

auto main(int argc, char** argv) -> int {
  using namespace simon;
  using namespace std::chrono_literals;

  flight::Scenario scenario;
  if (argc > 1) {
    scenario.aircraft = std::atoi(argv[1]);
  }
  if (argc > 2) {
    scenario.precise = std::atoi(argv[2]);
  }
  if (argc > 3) {
    scenario.rigid = std::atoi(argv[3]);
  }
  if (argc > 4) {
    scenario.seed = std::strtoull(argv[4], nullptr, 10);
  }
  flight::Simulation simulation{scenario};
  engine::BatchDriver driver{engine::Timing{.max_step = 20ms},
                             Depend(simulation)};

  auto wall_start = std::chrono::steady_clock::now();
  auto reached = driver.run(framework::TimePoint{10min});
  if (!reached) {
    std::println(stderr, "Error: {}", reached.error().message());
    return EXIT_FAILURE;
  }
  double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                              wall_start)
                    .count();

  std::println(
      "seed {}: {} aircraft ({} precise, {} rigid) flew {:.0f} s, reaching {} "
      "waypoints ({} by rigid aircraft), in {:.2f} s",
      scenario.seed, scenario.aircraft, scenario.precise, scenario.rigid,
      std::chrono::duration<double>(reached->time_since_epoch()).count(),
      simulation.waypoints_reached(), simulation.rigid_waypoints_reached(),
      wall);
  return EXIT_SUCCESS;
}
