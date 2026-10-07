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
#include <cstdlib>
#include <iostream>
#include <string>

#include "application/robotic/simulation.hpp"
#include "core/vocabulary.hpp"

auto main(int argc, char** argv) -> int {
  using namespace simon;
  if (argc < 2) {
    std::cerr << "Usage: robotic_benchmark SCENE.xml [steps]\n";
    return EXIT_FAILURE;
  }
  int steps = argc > 2 ? std::atoi(argv[2]) : 500;
  robotic::Simulation simulation{robotic::Scenario{.model = argv[1]}};
  if (auto configured = simulation.configure(); !configured) {
    std::cerr << "Error: " << configured.error().message() << "\n";
    return EXIT_FAILURE;
  }
  double h = simulation.mechanics().model().physics.timestep;
  auto dt = std::chrono::nanoseconds{std::llround(h * 1e9)};
  auto step = [&](int k) {
    if (auto stepped =
            simulation.step(Step{.time = TimePoint{} + k * dt, .dt = dt});
        !stepped) {
      std::cerr << "Error: " << stepped.error().message() << "\n";
      std::exit(EXIT_FAILURE);
    }
  };
  step(0);
  std::uint64_t iterations = 0;
  auto start = std::chrono::steady_clock::now();
  for (int k = 1; k <= steps; ++k) {
    step(k);
    iterations += simulation.constraints().iterations;
  }
  double seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
          .count();
  std::size_t trees = simulation.mechanics().trees().size();
  std::cout << argv[1] << ": " << trees << " trees, " << seconds / steps * 1e3
            << " ms/step, "
            << seconds / steps / static_cast<double>(trees) * 1e9
            << " ns/tree-step, " << simulation.contacts().size()
            << " contacts, " << simulation.constraints().rows << " rows and "
            << simulation.constraints().islands << " islands at the end, "
            << static_cast<double>(iterations) / steps
            << " solver iterations a step\n";
  return EXIT_SUCCESS;
}
