// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

// Plays an OpenSCENARIO scenario headless, at 0.05 s steps, until its
// storyboard stops, and prints where each vehicle ends:
//
//   bazel run //application/automotive:scenario -- <scenario.xosc>

#include <chrono>
#include <cmath>
#include <print>
#include <string>

#include "application/automotive/scenario_simulation.hpp"
#include "engine/driver.hpp"
#include "framework/vocabulary.hpp"

auto main(int argc, char** argv) -> int {
  using namespace std::chrono_literals;
  using namespace simon;
  using namespace simon::automotive;
  if (argc != 2) {
    std::println(stderr, "scenario <scenario.xosc>");
    return 1;
  }
  ScenarioSimulation simulation{argv[1]};
  engine::BatchDriver<ScenarioSimulation> driver{
      engine::Timing{.max_step = 50ms}, Depend(simulation)};
  if (auto end = driver.run(framework::TimePoint{3600s}); !end) {
    std::println(stderr, "{}", end.error().message());
    return 1;
  }
  const ScenarioWorld& world = simulation.world();
  world.store_of<ScenarioActor>().for_each([&](Entity owner,
                                               const ScenarioActor& actor) {
    const VehiclePose& pose = world.store_of<VehiclePose>().component_of(owner);
    const ScenarioSpeed& speed =
        world.store_of<ScenarioSpeed>().component_of(owner);
    Vector3 at = model::eigen(pose.position);
    std::println("{}: ({:.3f}, {:.3f}) heading {:.4f} rad, {:.3f} m/s",
                 simulation.scenario().entities[actor.entity].name, at.x(),
                 at.y(), model::radians(pose.heading), speed.speed);
  });
}
