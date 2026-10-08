// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

// Runs one defense scenario as fast as possible and prints the outcome:
//
//   bazel run //application/defense -- [seed]

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <print>
#include <string_view>

#include "application/arguments.hpp"
#include "application/defense/simulation.hpp"
#include "core/argument.hpp"
#include "engine/driver.hpp"

// How the program is called.
constexpr std::string_view USAGE = "defense [seed]";

auto main(int argc, char** argv) -> int {
  using namespace simon;
  using namespace std::chrono_literals;

  defense::Scenario scenario;
  application::Arguments arguments{argc, argv};
  scenario.seed = static_cast<std::uint64_t>(
      arguments.integer(0, static_cast<std::int64_t>(scenario.seed)));
  if (arguments.report_error(USAGE)) {
    return EXIT_FAILURE;
  }
  defense::Simulation simulation{scenario};
  engine::BatchDriver driver{engine::Timing{.max_step = 10ms},
                             Depend(simulation)};

  auto reached = driver.run(TimePoint{10min});
  if (!reached) {
    std::println(stderr, "Error: {}", reached.error().message());
    return EXIT_FAILURE;
  }

  const char* outcome = "undecided";
  if (simulation.outcome() == defense::Outcome::BLUE_WINS)
    outcome = "blue wins";
  if (simulation.outcome() == defense::Outcome::RED_WINS) outcome = "red wins";
  std::println(
      "seed {}: {} at {:.2f} s, {} interceptors fired", scenario.seed, outcome,
      std::chrono::duration<double>(reached->time_since_epoch()).count(),
      simulation.interceptors_fired());
  return EXIT_SUCCESS;
}
