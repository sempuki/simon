// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

// Times galactic's gravity on one thread, for comparison with REBOUND
// (reference/rebound_benchmark.py):
//
//   bazel run -c opt //application/galactic:galactic_benchmark --
//       direct|tree|restricted N [steps]
//
// direct and tree step a Plummer sphere of N bodies by direct summation or
// on the tree at an opening angle of 0.5; restricted steps two masses with N
// test particles in a disk about the first. Each takes one step to compute
// the first accelerations, then times `steps` more, and prints the wall time
// per step and per body.

#include <chrono>
#include <cstdlib>
#include <print>
#include <string_view>

#include "application/galactic/simulation.hpp"
#include "core/random.hpp"
#include "core/vocabulary.hpp"
#include "engine/driver.hpp"
#include "model/galaxy.hpp"
#include "model/gravity.hpp"

namespace simon::galactic {
namespace {

using model::KILOPARSEC;
using model::SOLAR_MASS;

auto make_plummer_scenario(std::size_t bodies, GravityMethod gravity)
    -> Scenario {
  Scenario scenario{
      .softening = 0.01 * KILOPARSEC, .gravity = gravity, .opening_angle = 0.5};
  Random random{1};
  model::append_plummer(
      model::Plummer{.mass = 1e10 * SOLAR_MASS, .scale = KILOPARSEC}, bodies,
      InOut(random), InOut(scenario.bodies));
  return scenario;
}

// Two masses 50 kpc apart, the first with `particles` test particles spread
// evenly over rings from 2 to 20 kpc about it, on circular orbits.
auto make_restricted_scenario(std::size_t particles) -> Scenario {
  Scenario scenario{.softening = 0.1 * KILOPARSEC};
  model::BodyStart center{.mass = 1e11 * SOLAR_MASS};
  scenario.bodies = {
      center, model::BodyStart{
                  .position = meters(
                      50.0 * KILOPARSEC.numerical_value_in(meter), 0.0, 0.0),
                  .mass = 1e11 * SOLAR_MASS}};
  model::RingDisk disk{.softening = scenario.softening};
  constexpr int RINGS = 100;
  for (int ring = 0; ring < RINGS; ++ring) {
    disk.radii.push_back((2.0 + 18.0 * ring / (RINGS - 1)) * KILOPARSEC);
    disk.counts.push_back(static_cast<int>(particles / RINGS));
  }
  model::append_ring_disk(disk, center, InOut(scenario.test_particles));
  return scenario;
}

}  // namespace
}  // namespace simon::galactic

auto main(int argc, char** argv) -> int {
  using namespace simon;
  using namespace simon::galactic;
  if (argc < 3) {
    std::println(stderr,
                 "Usage: galactic_benchmark direct|tree|restricted N [steps]");
    return EXIT_FAILURE;
  }
  std::string_view kind = argv[1];
  auto count = static_cast<std::size_t>(std::strtoull(argv[2], nullptr, 10));
  int steps = argc > 3 ? std::atoi(argv[3]) : 10;

  Scenario scenario;
  if (kind == "direct") {
    scenario = make_plummer_scenario(count, GravityMethod::DIRECT);
  } else if (kind == "tree") {
    scenario = make_plummer_scenario(count, GravityMethod::TREE);
  } else if (kind == "restricted") {
    scenario = make_restricted_scenario(count);
  } else {
    std::println(stderr, "Unknown kind {}", kind);
    return EXIT_FAILURE;
  }
  std::size_t bodies = scenario.bodies.size() + scenario.test_particles.size();

  Simulation simulation{std::move(scenario)};
  const Year step{10000};
  engine::Driver driver{Timing{.max_step = step}, Depend(simulation)};
  if (auto started = driver.start(); !started) {
    std::println(stderr, "Error: {}", started.error().message());
    return EXIT_FAILURE;
  }
  auto start = std::chrono::steady_clock::now();
  if (auto reached = driver.advance_to(BasicTimePoint<Year>{} + steps * step);
      !reached) {
    std::println(stderr, "Error: {}", reached.error().message());
    return EXIT_FAILURE;
  }
  double seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
          .count();
  std::println("{} {}: {:.3f} ms/step, {:.0f} ns/body-step", kind, bodies,
               seconds / steps * 1e3,
               seconds / steps / static_cast<double>(bodies) * 1e9);
  return EXIT_SUCCESS;
}
