// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

// Runs the balls as fast as possible and prints their energy each simulated
// second, with the wall time each second took:
//
//   bazel run -c opt //application/hello -- [balls] [seconds] [restitution]

#include <chrono>
#include <cstdlib>
#include <print>
#include <string_view>

#include "application/arguments.hpp"
#include "application/hello/hello.hpp"
#include "core/argument.hpp"
#include "engine/driver.hpp"

// How the program is called.
constexpr std::string_view USAGE = "hello [balls] [seconds] [restitution]";

auto main(int argc, char** argv) -> int {
  using namespace simon;
  using namespace std::chrono_literals;
  using WallClock = std::chrono::steady_clock;

  hello::Scenario scenario;
  application::Arguments arguments{argc, argv};
  scenario.balls = static_cast<std::size_t>(
      arguments.integer(0, static_cast<std::int64_t>(scenario.balls)));
  auto seconds = static_cast<int>(arguments.integer(1, 10));
  scenario.springiness.restitution =
      arguments.number(2, scenario.springiness.restitution);
  if (arguments.report_error(USAGE)) {
    return EXIT_FAILURE;
  }

  hello::Simulation simulation{scenario};
  engine::Driver driver{engine::Timing{.max_step = hello::STEP},
                        Depend(simulation)};
  if (auto started = driver.start(); !started) {
    std::println(stderr, "Error: {}", started.error().message());
    return EXIT_FAILURE;
  }

  for (int second = 0; second <= seconds; ++second) {
    WallClock::time_point start = WallClock::now();
    if (auto reached = driver.advance_to(TimePoint{1s * second}); !reached) {
      std::println(stderr, "Error: {}", reached.error().message());
      return EXIT_FAILURE;
    }
    double wall =
        std::chrono::duration<double, std::milli>(WallClock::now() - start)
            .count();
    std::println("{:4} s  energy {:12.1f} J  ({:.1f} ms)", second,
                 hello::compute_energy(simulation.world(), scenario.gravity)
                     .numerical_value_in(units::si::joule),
                 wall);
  }

  if (auto finished = driver.finish(); !finished) {
    std::println(stderr, "Error: {}", finished.error().message());
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
