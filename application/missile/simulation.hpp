// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <numbers>

#include "application/missile/components.hpp"
#include "application/missile/systems.hpp"
#include "base/core.hpp"
#include "engine/lifecycle.hpp"
#include "model/random.hpp"

namespace simon::missile {

// Everything a run depends on. The same scenario gives the same run.
struct Scenario final {
  std::uint64_t seed = 1;

  double asset_health = 30.0;

  int radars = 3;
  Length radar_ring = 500.0 * model::meter;
  Length radar_range = 4000.0 * model::meter;
  Duration scan_period = 1s;

  int launchers = 3;
  Length launcher_ring = 300.0 * model::meter;
  Length launcher_range = 3000.0 * model::meter;
  std::uint32_t inventory = 20;
  Duration reload = 2s;

  int drones = 30;
  Length spawn_distance = 6000.0 * model::meter;
  Length spawn_spread = 1000.0 * model::meter;
  Speed drone_cruise = 40.0 * model::meter_per_second;
  AccelerationMagnitude drone_agility = 20.0 * model::meter_per_second_squared;
  Warhead drone_warhead{.fuse = 30.0 * model::meter,
                        .radius = 100.0 * model::meter,
                        .damage = 10.0};
};

enum class Outcome { UNDECIDED, BLUE_WINS, RED_WINS };

// A world big enough for everything a scenario can create at once: the asset,
// radars, launchers, every drone with its track and blast, and every
// interceptor with its blast.
inline framework::WorldConfiguration world_configuration_of(
    const Scenario& scenario) {
  auto count = [](int value) {
    return static_cast<std::size_t>(std::max(value, 0));
  };
  std::size_t interceptors = count(scenario.launchers) * scenario.inventory;
  std::size_t entities = 1 + count(scenario.radars) +
                         count(scenario.launchers) +
                         3 * count(scenario.drones) + 2 * interceptors;
  return framework::WorldConfiguration{
      .number = 1, .entities = entities, .components = entities};
}

// A point `radius` from the origin at `bearing` radians from east.
inline Position on_ring(Length radius, double bearing) {
  double length = radius.numerical_value_in(model::meter);
  return model::meters(length * std::cos(bearing), length * std::sin(bearing),
                       0.0);
}

// Places the asset at the origin, radars and launchers on rings around it, and
// red drones at random bearings, flying at it. Returns the asset.
inline Entity build_scenario(lib::InOut<World> world,
                             const Scenario& scenario) {
  constexpr double TURN = 2.0 * std::numbers::pi;
  auto asset = world->create<archetype::Asset>("asset")
                   .with(Kinematics{})
                   .with(Health{.points = scenario.asset_health})
                   .with(Asset{})
                   .build();
  CHECK_POSTCONDITION(asset.has_value());
  Entity asset_entity = *asset;

  for (int i = 0; i < scenario.radars; ++i) {
    double bearing = TURN * i / scenario.radars;
    auto radar =
        world->create<archetype::Radar>()
            .with(Kinematics{.position = on_ring(scenario.radar_ring, bearing)})
            .with(Radar{.range = scenario.radar_range,
                        .scan = engine::RateGate{scenario.scan_period}})
            .build();
    CHECK_POSTCONDITION(radar.has_value());
  }

  for (int i = 0; i < scenario.launchers; ++i) {
    double bearing = TURN * (i + 0.5) / scenario.launchers;
    auto launcher = world->create<archetype::Launcher>()
                        .with(Kinematics{.position = on_ring(
                                             scenario.launcher_ring, bearing)})
                        .with(Launcher{.range = scenario.launcher_range,
                                       .inventory = scenario.inventory,
                                       .reload = scenario.reload})
                        .build();
    CHECK_POSTCONDITION(launcher.has_value());
  }

  model::Random random{scenario.seed};
  Kinematics asset_kinematics;
  for (int i = 0; i < scenario.drones; ++i) {
    double bearing = random.uniform(0.0, TURN);
    Length radius = scenario.spawn_distance +
                    scenario.spawn_spread * random.uniform(-0.5, 0.5);
    Kinematics kinematics{.position = on_ring(radius, bearing)};
    kinematics.velocity =
        (asset_kinematics.position - kinematics.position) *
        (scenario.drone_cruise /
         norm(asset_kinematics.position - kinematics.position));
    auto drone = world->create<archetype::RedDrone>()
                     .with(kinematics)
                     .with(Control{})
                     .with(Health{.points = 1.0})
                     .with(scenario.drone_warhead)
                     .with(RedDrone{.target = asset_entity,
                                    .cruise = scenario.drone_cruise,
                                    .agility = scenario.drone_agility})
                     .build();
    CHECK_POSTCONDITION(drone.has_value());
  }
  world->sync();
  return asset_entity;
}

// The missile simulation: builds the scenario when configured, and stops when
// red is defeated or the asset is destroyed. Any driver can run it.
class Simulation final {
 public:
  explicit Simulation(Scenario scenario = {})
      : scenario_{scenario}, world_{world_configuration_of(scenario)} {}

  engine::PhaseResult configure() {
    asset_ = build_scenario(lib::InOut(world_), scenario_);
    return engine::Flow::CONTINUE;
  }

  engine::PhaseResult step(const framework::Step& step) {
    scheduler_.step(lib::InOut(world_), step);
    if (!world_.alive(asset_)) {
      outcome_ = Outcome::RED_WINS;
    } else if (world_.store_of<RedDrone>().size() == 0) {
      outcome_ = Outcome::BLUE_WINS;
    }
    return outcome_ == Outcome::UNDECIDED ? engine::Flow::CONTINUE
                                          : engine::Flow::STOP;
  }

  Outcome outcome() const { return outcome_; }
  const World& world() const { return world_; }
  Entity asset() const { return asset_; }

  // Interceptors fired so far, from what the launchers have left.
  std::uint32_t interceptors_fired() const {
    std::uint32_t remaining = 0;
    for (const Launcher& launcher : world_.store_of<Launcher>().values()) {
      remaining += launcher.inventory;
    }
    return static_cast<std::uint32_t>(scenario_.launchers) *
               scenario_.inventory -
           remaining;
  }

 private:
  Scenario scenario_;
  World world_;
  Scheduler scheduler_;
  Entity asset_;
  Outcome outcome_ = Outcome::UNDECIDED;
};

}  // namespace simon::missile
