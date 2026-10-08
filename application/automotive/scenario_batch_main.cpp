// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

// Plays every permutation of an OpenSCENARIO parameter value distribution
// headless, at 0.05 s steps, on several threads, and prints a table of each
// run's parameter values and its ego's measures by nuPlan's metrics:
//
//   bazel run -c opt //application/automotive:scenario_batch --
//       <distribution.xosc> [threads]

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <print>
#include <string>
#include <string_view>
#include <thread>

#include "application/arguments.hpp"
#include "application/automotive/scenario_batch.hpp"
#include "format/openscenario.hpp"

// How the program is called.
constexpr std::string_view USAGE =
    "scenario_batch <distribution.xosc> [threads]";

auto main(int argc, char** argv) -> int {
  using namespace std::chrono_literals;
  using namespace simon;
  using namespace simon::automotive;
  application::Arguments arguments{argc, argv};
  std::string_view file = arguments.text(0, "");
  auto threads = static_cast<std::size_t>(arguments.integer(
      1, static_cast<std::int64_t>(std::thread::hardware_concurrency())));
  if (arguments.report_error(USAGE)) {
    return EXIT_FAILURE;
  }
  if (file.empty()) {
    std::println(stderr, "Usage: {}", USAGE);
    return EXIT_FAILURE;
  }
  auto distribution = format::load_parameter_distribution(std::string{file});
  if (!distribution) {
    std::println(stderr, "{}", distribution.error().message());
    return EXIT_FAILURE;
  }
  constexpr std::size_t LIMIT = 72'000;  // An hour of steps.
  std::vector<BatchRun> runs = run_batch(*distribution, threads, 50ms, LIMIT);

  std::println(
      "permutation,parameters,duration,comfortable,min_time_to_collision,"
      "time_to_collision_within_bound,min_gap,error");
  for (const BatchRun& run : runs) {
    std::string parameters;
    for (const scenario::ParameterAssignment& assignment : run.assignments) {
      parameters += (parameters.empty() ? "" : " ") + assignment.name + "=" +
                    assignment.value;
    }
    if (!run.measures) {
      std::println("{},{},,,,,,{}", run.permutation, parameters,
                   run.measures.error().message());
      continue;
    }
    const RunMeasures& measures = *run.measures;
    std::println("{},{},{:.2f},{},{},{},{:.3f},", run.permutation, parameters,
                 measures.duration, measures.comfortable,
                 measures.min_time_to_collision
                     ? std::format("{:.1f}", *measures.min_time_to_collision)
                     : "",
                 measures.time_to_collision_within_bound, measures.min_gap);
  }
}
