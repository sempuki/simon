// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <numbers>
#include <string>
#include <vector>

#include "application/automotive/scenario_simulation.hpp"
#include "application/automotive/testing.hpp"
#include "base/testing.hpp"

// esmini's scenarios played by simon and by esmini, from the table
// reference/esmini_scenarios.py recorded: every entity's position, heading
// and speed, step by step.
namespace simon::automotive {

namespace {

using namespace std::chrono_literals;
using namespace testing;

constexpr std::string_view SCENARIOS = "3rd_party/esmini/xosc/";

// An entity's state at a step.
struct Sample final {
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
  double speed = 0.0;
};

// Each step's samples by entity name, the first at time 0.
using Run = std::vector<std::map<std::string, Sample>>;

auto load_esmini(std::string_view scenario) -> Run {
  Run run;
  for (const Row& row : load_rows("esmini_scenarios.csv")) {
    if (row.find("scenario")->second != scenario) {
      continue;
    }
    auto step =
        static_cast<std::size_t>(std::lround(number(row, "time") / 0.05));
    if (run.size() <= step) {
      run.resize(step + 1);
    }
    run[step][row.find("entity")->second] =
        Sample{.x = number(row, "x"),
               .y = number(row, "y"),
               .heading = number(row, "heading"),
               .speed = number(row, "speed")};
  }
  return run;
}

// Plays `scenario` for `steps` steps of 0.05 s, sampling after each.
auto play(std::string_view scenario, std::size_t steps) -> Run {
  ScenarioSimulation simulation{std::string{SCENARIOS} + std::string{scenario}};
  auto configured = simulation.configure();
  if (!configured) {
    FAIL(configured.error().message());
  }
  Run run(1);
  for (std::size_t k = 0; k < steps; ++k) {
    framework::Step step{.time = framework::TimePoint{} + k * 50ms, .dt = 50ms};
    if (simulation.step(step).value() == engine::Flow::STOP) {
      break;
    }
    std::map<std::string, Sample> samples;
    const ScenarioWorld& world = simulation.world();
    world.store_of<ScenarioActor>().for_each(
        [&](Entity owner, const ScenarioActor& actor) {
          const RoadPose& pose = world.store_of<RoadPose>().component_of(owner);
          const ScenarioSpeed& speed =
              world.store_of<ScenarioSpeed>().component_of(owner);
          Vector3 at = model::eigen(pose.position);
          samples[simulation.scenario().entities[actor.entity].name] =
              Sample{.x = at.x(),
                     .y = at.y(),
                     .heading = model::radians(pose.heading),
                     .speed = speed.speed};
        });
    run.push_back(std::move(samples));
  }
  return run;
}

// How far two runs are apart: the largest distance, heading and speed
// difference over every step and entity, and the first step apart by more
// than a centimeter.
struct Apart final {
  double position = 0.0;
  double heading = 0.0;
  double speed = 0.0;
  double first_apart = -1.0;  // s.
  std::size_t steps = 0;
};

auto compare(const Run& ours, const Run& theirs) -> Apart {
  Apart apart;
  for (std::size_t k = 1; k < std::min(ours.size(), theirs.size()); ++k) {
    for (const auto& [name, mine] : ours[k]) {
      auto found = theirs[k].find(name);
      if (found == theirs[k].end()) {
        continue;
      }
      const Sample& other = found->second;
      double distance = std::hypot(mine.x - other.x, mine.y - other.y);
      apart.position = std::max(apart.position, distance);
      apart.heading = std::max(
          apart.heading, std::abs(std::remainder(mine.heading - other.heading,
                                                 2.0 * std::numbers::pi)));
      apart.speed = std::max(apart.speed, std::abs(mine.speed - other.speed));
      if (apart.first_apart < 0.0 && distance > 0.01) {
        apart.first_apart = 0.05 * static_cast<double>(k);
        UNSCOPED_INFO("first apart at " << apart.first_apart << " s: " << name
                                        << " simon (" << mine.x << ", "
                                        << mine.y << ") esmini (" << other.x
                                        << ", " << other.y << ")");
      }
    }
    ++apart.steps;
  }
  return apart;
}

}  // namespace

TEST_CASE("ScenariosAgainstEsmini") {
  // Every step of every entity, until the storyboard stops: positions to
  // esmini's log's six decimals but on e6mini's curves, 1.2 mm; speeds to
  // rounding, or to the six decimals where the log's speed feeds back, but
  // for the lane change's two steps where esmini reports a vehicle
  // teleported off the end of its road at a standstill for a step.
  struct Bounds final {
    std::string_view scenario;
    double position = 0.0;
    double speed = 0.0;
  };
  for (Bounds bounds : {Bounds{"cut-in_simple.xosc", 1e-6, 1e-12},
                        Bounds{"cut-in.xosc", 0.002, 1e-12},
                        Bounds{"lane_change_simple.xosc", 1e-6, 0.004},
                        Bounds{"traffic_lights.xosc", 3e-6, 1e-6}}) {
    CAPTURE(bounds.scenario);
    Run theirs = load_esmini(bounds.scenario);
    Run ours = play(bounds.scenario, theirs.size() - 1);
    Apart apart = compare(ours, theirs);
    CAPTURE(ours.size(), theirs.size(), apart.steps, apart.position,
            apart.heading, apart.speed, apart.first_apart);
    CHECK(ours.size() == theirs.size());
    CHECK(apart.position < bounds.position);
    CHECK(apart.heading < 2e-6);
    CHECK(apart.speed < bounds.speed);
  }
}

}  // namespace simon::automotive
