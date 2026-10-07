// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

// Times robotic stepping a scene, for comparison with MuJoCo on one thread
// (reference/mujoco_benchmark.py):
//
//   python application/robotic/reference/make_scenes.py DIR
//   bazel run -c opt //application/robotic:robotic_benchmark --
//       DIR/bodies_10000.xml [steps]
//
// Steps the scene once to settle the first contacts, then times `steps`
// more, and prints the wall time per step and per tree.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <optional>
#include <print>

#include "application/robotic/simulation.hpp"
#include "framework/benchmarking.hpp"

auto main(int argc, char** argv) -> int {
  using namespace simon;
  if (argc < 2) {
    std::println(stderr, "Usage: robotic_benchmark SCENE.xml [steps]");
    return EXIT_FAILURE;
  }
  std::optional<int> steps =
      argc > 2 ? framework::benchmark::parse_count(argv[2]) : 500;
  if (!steps) {
    std::println(stderr, "steps is a whole positive number");
    return EXIT_FAILURE;
  }
  robotic::Simulation simulation{robotic::Scenario{.model = argv[1]}};
  if (auto configured = simulation.configure(); !configured) {
    std::println(stderr, "Error: {}", configured.error().message());
    return EXIT_FAILURE;
  }
  double h = simulation.mechanics().model().physics.timestep;
  auto dt = std::chrono::nanoseconds{std::llround(h * 1e9)};
  auto step = [&](int k) {
    if (auto stepped =
            simulation.step(Step{.time = TimePoint{} + k * dt, .dt = dt});
        !stepped) {
      std::println(stderr, "Error: {}", stepped.error().message());
      std::exit(EXIT_FAILURE);
    }
  };
  step(0);
  std::uint64_t iterations = 0;
  framework::benchmark::Stopwatch stopwatch;
  for (int k = 1; k <= *steps; ++k) {
    step(k);
    iterations += simulation.constraints().iterations;
  }
  double seconds = stopwatch.seconds();
  std::size_t trees = simulation.mechanics().trees().size();
  std::println(
      "{}: {} trees, {:.6g} ms/step, {:.6g} ns/tree-step, {} contacts, {} rows "
      "and {} islands at the end, {:.6g} solver iterations a step",
      argv[1], trees, seconds / *steps * 1e3,
      seconds / *steps / static_cast<double>(trees) * 1e9,
      simulation.contacts().size(), simulation.constraints().rows,
      simulation.constraints().islands,
      static_cast<double>(iterations) / *steps);
  return EXIT_SUCCESS;
}
