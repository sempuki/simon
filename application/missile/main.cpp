// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

// Runs one missile scenario as fast as possible and prints the outcome:
//
//   bazel run //application/missile -- [seed]

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <print>

#include "application/missile/simulation.hpp"
#include "engine/driver.hpp"

int main(int argc, char** argv) {
  using namespace simon;
  using namespace std::chrono_literals;

  missile::Scenario scenario;
  if (argc > 1) {
    scenario.seed = std::strtoull(argv[1], nullptr, 10);
  }
  missile::Simulation simulation{scenario};
  engine::BatchDriver driver{engine::Timing{.max_step = 10ms},
                             lib::Depend(simulation)};

  auto reached = driver.run(framework::TimePoint{10min});
  if (!reached) {
    std::println(stderr, "Error: {}", reached.error().message());
    return EXIT_FAILURE;
  }

  const char* outcome = "undecided";
  if (simulation.outcome() == missile::Outcome::BLUE_WINS)
    outcome = "blue wins";
  if (simulation.outcome() == missile::Outcome::RED_WINS) outcome = "red wins";
  std::println(
      "seed {}: {} at {:.2f} s, {} interceptors fired", scenario.seed, outcome,
      std::chrono::duration<double>(reached->time_since_epoch()).count(),
      simulation.interceptors_fired());
  return EXIT_SUCCESS;
}
