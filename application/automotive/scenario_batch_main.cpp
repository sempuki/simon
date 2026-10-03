// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

// Plays every permutation of an OpenSCENARIO parameter value distribution
// headless, at 0.05 s steps, on several threads, and prints a table of each
// run's parameter values and its ego's measures by nuPlan's metrics:
//
//   bazel run -c opt //application/automotive:scenario_batch -- \
//       <distribution.xosc> [threads]

#include <chrono>
#include <cstdlib>
#include <print>
#include <string>
#include <thread>

#include "application/automotive/scenario_batch.hpp"
#include "format/openscenario.hpp"

auto main(int argc, char** argv) -> int {
  using namespace std::chrono_literals;
  using namespace simon;
  using namespace simon::automotive;
  if (argc != 2 && argc != 3) {
    std::println(stderr, "scenario_batch <distribution.xosc> [threads]");
    return 1;
  }
  auto distribution = format::load_parameter_distribution(argv[1]);
  if (!distribution) {
    std::println(stderr, "{}", distribution.error().message());
    return 1;
  }
  std::size_t threads = argc == 3 ? std::strtoul(argv[2], nullptr, 10)
                                  : std::thread::hardware_concurrency();
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
