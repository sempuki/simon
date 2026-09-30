// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

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

// The missile simulation: builds the scenario when configured, and stops when
// red is defeated or the asset is destroyed. Any driver can run it.
class Simulation final {
 public:
  explicit Simulation(Scenario scenario = {})
      : scenario_{scenario},
        world_{framework::WorldConfiguration{
            .number = 1, .entities = 1024, .components = 1024}} {}

  engine::PhaseResult configure() {
    build(lib::InOut(world_));
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
  static Position on_ring(Length radius, double bearing) {
    double length = radius.numerical_value_in(model::meter);
    return model::meters(length * std::cos(bearing), length * std::sin(bearing),
                         0.0);
  }

  void build(lib::InOut<World> world) {
    constexpr double TURN = 2.0 * std::numbers::pi;
    auto asset = world->create<archetype::Asset>("asset")
                     .with(Kinematics{})
                     .with(Health{.points = scenario_.asset_health})
                     .with(Asset{})
                     .build();
    CHECK_POSTCONDITION(asset.has_value());
    asset_ = *asset;

    for (int i = 0; i < scenario_.radars; ++i) {
      double bearing = TURN * i / scenario_.radars;
      auto radar =
          world->create<archetype::Radar>()
              .with(Kinematics{.position =
                                   on_ring(scenario_.radar_ring, bearing)})
              .with(Radar{.range = scenario_.radar_range,
                          .scan = engine::RateGate{scenario_.scan_period}})
              .build();
      CHECK_POSTCONDITION(radar.has_value());
    }

    for (int i = 0; i < scenario_.launchers; ++i) {
      double bearing = TURN * (i + 0.5) / scenario_.launchers;
      auto launcher =
          world->create<archetype::Launcher>()
              .with(Kinematics{.position =
                                   on_ring(scenario_.launcher_ring, bearing)})
              .with(Launcher{.range = scenario_.launcher_range,
                             .inventory = scenario_.inventory,
                             .reload = scenario_.reload})
              .build();
      CHECK_POSTCONDITION(launcher.has_value());
    }

    model::Random random{scenario_.seed};
    Kinematics asset_kinematics;
    for (int i = 0; i < scenario_.drones; ++i) {
      double bearing = random.uniform(0.0, TURN);
      Length radius = scenario_.spawn_distance +
                      scenario_.spawn_spread * random.uniform(-0.5, 0.5);
      Kinematics kinematics{.position = on_ring(radius, bearing)};
      kinematics.velocity =
          (asset_kinematics.position - kinematics.position) *
          (scenario_.drone_cruise /
           norm(asset_kinematics.position - kinematics.position));
      auto drone = world->create<archetype::RedDrone>()
                       .with(kinematics)
                       .with(Control{})
                       .with(Health{.points = 1.0})
                       .with(scenario_.drone_warhead)
                       .with(RedDrone{.target = asset_,
                                      .cruise = scenario_.drone_cruise,
                                      .agility = scenario_.drone_agility})
                       .build();
      CHECK_POSTCONDITION(drone.has_value());
    }
    world->sync();
  }

  Scenario scenario_;
  World world_;
  Scheduler scheduler_;
  Entity asset_;
  Outcome outcome_ = Outcome::UNDECIDED;
};

}  // namespace simon::missile
