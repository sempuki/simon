// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

// Runs the balls as fast as possible and prints their energy each simulated
// second, with the wall time each second took:
//
//   bazel run -c opt //application/hello -- [balls] [seconds] [restitution]

#include <chrono>
#include <cstdlib>
#include <print>

#include "application/hello/hello.hpp"
#include "engine/driver.hpp"
#include "framework/vocabulary.hpp"

auto main(int argc, char** argv) -> int {
  using namespace simon;
  using namespace std::chrono_literals;
  using WallClock = std::chrono::steady_clock;

  hello::Scenario scenario;
  if (argc > 1) scenario.balls = std::strtoull(argv[1], nullptr, 10);
  int seconds = argc > 2 ? std::atoi(argv[2]) : 10;
  if (argc > 3) {
    scenario.springiness.restitution = std::strtod(argv[3], nullptr);
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
    if (auto reached = driver.advance_to(framework::TimePoint{1s * second});
        !reached) {
      std::println(stderr, "Error: {}", reached.error().message());
      return EXIT_FAILURE;
    }
    double wall =
        std::chrono::duration<double, std::milli>(WallClock::now() - start)
            .count();
    std::println("{:4} s  energy {:12.1f} J  ({:.1f} ms)", second,
                 hello::compute_energy(simulation.world(), scenario.gravity)
                     .numerical_value_in(model::units::si::joule),
                 wall);
  }

  if (auto finished = driver.finish(); !finished) {
    std::println(stderr, "Error: {}", finished.error().message());
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
