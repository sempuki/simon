// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <expected>
#include <numbers>
#include <optional>
#include <vector>

#include "application/missile/components.hpp"
#include "application/missile/systems.hpp"
#include "base/core.hpp"
#include "engine/lifecycle.hpp"
#include "model/random.hpp"

namespace simon::missile {

// Everything a run depends on. The same scenario gives the same run.
//
// A scenario is one or more sites. Each site has an asset, and radars,
// launchers and drones around it; the counts below are per site.
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

  // Copies of the site on a square grid `site_spacing` apart, the first at the
  // origin, so a larger population covers a larger area at the same density.
  // Each site's drones fly at their own asset, and the outcome follows the
  // first site's.
  int sites = 1;
  Length site_spacing = 20000.0 * model::meter;
};

enum class Outcome { UNDECIDED, BLUE_WINS, RED_WINS };

// Builds in `world` the world a scenario needs: at every site, holding the
// asset, radars and launchers, every drone with its track and blast, and every
// interceptor with its blast.
inline std::expected<void, framework::Status> build_world(
    const Scenario& scenario, lib::Out<World> world) {
  auto count = [](int value) {
    return static_cast<std::size_t>(std::max(value, 0));
  };
  std::size_t sites = count(scenario.sites);
  std::size_t drones = sites * count(scenario.drones);
  std::size_t interceptors =
      sites * count(scenario.launchers) * scenario.inventory;
  // Cells a few times smaller than the sensor and weapon ranges (1 to 4 km).
  return World::set_up()
      .numbered(1)
      .holding<archetype::Asset>(sites)
      .holding<archetype::Radar>(sites * count(scenario.radars))
      .holding<archetype::Launcher>(sites * count(scenario.launchers))
      .holding<archetype::RedDrone>(drones)
      .holding<archetype::Track>(drones)
      .holding<archetype::Interceptor>(interceptors)
      .holding<archetype::Blast>(drones + interceptors)
      .cells_of(250.0 * model::meter)
      .build(world);
}

// A point `radius` from the origin at `bearing` radians from east.
inline Position on_ring(Length radius, double bearing) {
  double length = radius.numerical_value_in(model::meter);
  return model::meters(length * std::cos(bearing), length * std::sin(bearing),
                       0.0);
}

// A band around a site: things fall at `radius`, give or take half of
// `width`.
struct Ring final {
  Length radius = 0.0 * model::meter;
  Length width = 0.0 * model::meter;
};

// Builds one defended site: an asset at its origin, the radars and launchers
// that defend it on rings around it, and the red drones that attack it, flying
// at the asset. A domain builder (see "Domain builders" in
// documents/design.md): one utterance becomes an entity utterance for each
// thing on the site. Start with create_site.
class [[nodiscard]] SiteBuilder final {
 public:
  // Keeps a reference to `world` until the utterance is built.
  SiteBuilder(Position origin, lib::Depend<World> world)
      : origin_{origin}, world_{world.get()} {}

  // The asset the site protects, and its health. 30 points unless given.
  SiteBuilder protecting(Health health) && {
    asset_health_ = health;
    return std::move(*this);
  }

  // `count` radars like `radar`, evenly spaced at `radius` from the asset.
  SiteBuilder watched_by(std::size_t count, Radar radar, Length radius) && {
    radars_ = Placement<Radar>{.count = count, .unit = radar, .radius = radius};
    return std::move(*this);
  }

  // `count` launchers like `launcher`, evenly spaced at `radius` from the
  // asset, starting half a spacing from the radars.
  SiteBuilder defended_by(std::size_t count, Launcher launcher,
                          Length radius) && {
    launchers_ =
        Placement<Launcher>{.count = count, .unit = launcher, .radius = radius};
    return std::move(*this);
  }

  // `count` red drones like `drone`, carrying `warhead`, spawning at random
  // bearings within `ring` and flying at the asset at cruise speed. Draws
  // from `random`, which it keeps until the utterance is built.
  SiteBuilder attacked_by(std::size_t count, RedDrone drone, Warhead warhead,
                          Ring ring, lib::Depend<model::Random> random) && {
    drones_ = count;
    drone_ = drone;
    warhead_ = warhead;
    spawn_ = ring;
    random_ = random.get();
    return std::move(*this);
  }

  // Creates the site and returns its asset. If the world refuses any entity,
  // destroys those already created by this utterance, so nothing of the site
  // is left after the next sync, and returns the world's framework::Status.
  std::expected<Entity, framework::Status> build() && {
    constexpr double TURN = 2.0 * std::numbers::pi;
    std::vector<Entity> created;
    auto refuse = [&](framework::Status status)
        -> std::expected<Entity, framework::Status> {
      for (Entity entity : created) {
        auto destroyed = world_->destroy(entity).build();
        DECLARE_UNUSED(destroyed);
      }
      return std::unexpected(status);
    };
    auto keep = [&](std::expected<Entity, framework::Status> entity) {
      if (entity) {
        created.push_back(*entity);
      }
      return entity.has_value();
    };

    auto asset = world_->create<archetype::Asset>("asset")
                     .with(Kinematics{.position = origin_})
                     .with(asset_health_)
                     .with(Asset{})
                     .build();
    if (!keep(asset)) {
      return refuse(asset.error());
    }
    for (std::size_t i = 0; i < radars_.count; ++i) {
      double bearing =
          TURN * static_cast<double>(i) / static_cast<double>(radars_.count);
      auto radar =
          world_->create<archetype::Radar>()
              .with(Kinematics{.position =
                                   origin_ + on_ring(radars_.radius, bearing)})
              .with(radars_.unit)
              .build();
      if (!keep(radar)) {
        return refuse(radar.error());
      }
    }
    for (std::size_t i = 0; i < launchers_.count; ++i) {
      double bearing = TURN * (static_cast<double>(i) + 0.5) /
                       static_cast<double>(launchers_.count);
      auto launcher =
          world_->create<archetype::Launcher>()
              .with(Kinematics{.position = origin_ +
                                           on_ring(launchers_.radius, bearing)})
              .with(launchers_.unit)
              .build();
      if (!keep(launcher)) {
        return refuse(launcher.error());
      }
    }
    for (std::size_t i = 0; i < drones_; ++i) {
      double bearing = random_->uniform(0.0, TURN);
      Length radius =
          spawn_.radius + spawn_.width * random_->uniform(-0.5, 0.5);
      Kinematics kinematics{.position = origin_ + on_ring(radius, bearing)};
      kinematics.velocity =
          (origin_ - kinematics.position) *
          (drone_.cruise / norm(origin_ - kinematics.position));
      auto drone = world_->create<archetype::RedDrone>()
                       .with(kinematics)
                       .with(Control{})
                       .with(Health{.points = 1.0})
                       .with(warhead_)
                       .with(Target{.entity = *asset})
                       .with(drone_)
                       .build();
      if (!keep(drone)) {
        return refuse(drone.error());
      }
    }
    return *asset;
  }

 private:
  template <typename UnitType>
  struct Placement final {
    std::size_t count = 0;
    UnitType unit{};
    Length radius = 0.0 * model::meter;
  };

  Position origin_;
  // Never null once constructed; Depend checks it.
  World* world_ = nullptr;
  Health asset_health_{.points = 30.0};
  Placement<Radar> radars_;
  Placement<Launcher> launchers_;
  std::size_t drones_ = 0;
  RedDrone drone_;
  Warhead warhead_;
  Ring spawn_;
  model::Random* random_ = nullptr;  // Set with any drones.
};

// Starts the utterance that builds a defended site at `origin` in `world`.
inline SiteBuilder create_site(Position origin, lib::Depend<World> world) {
  return SiteBuilder{origin, world};
}

// Builds every site of a scenario on a square grid, the first at the origin,
// and returns the first site's asset.
inline std::expected<Entity, framework::Status> build_scenario(
    const Scenario& scenario, lib::InOut<World> world) {
  auto count = [](int value) {
    return static_cast<std::size_t>(std::max(value, 0));
  };
  model::Random random{scenario.seed};
  int side = static_cast<int>(std::ceil(std::sqrt(scenario.sites)));
  double spacing = scenario.site_spacing.numerical_value_in(model::meter);
  std::optional<Entity> first;
  for (int site = 0; site < scenario.sites; ++site) {
    Position origin =
        model::meters(spacing * (site % side), spacing * (site / side), 0.0);
    ASSIGN_OR_RETURN(
        Entity asset,
        create_site(origin, lib::Depend(*world))
            .protecting(Health{.points = scenario.asset_health})
            .watched_by(count(scenario.radars),
                        Radar{.range = scenario.radar_range,
                              .scan = engine::RateGate{scenario.scan_period}},
                        scenario.radar_ring)
            .defended_by(count(scenario.launchers),
                         Launcher{.range = scenario.launcher_range,
                                  .inventory = scenario.inventory,
                                  .reload = scenario.reload},
                         scenario.launcher_ring)
            .attacked_by(count(scenario.drones),
                         RedDrone{.cruise = scenario.drone_cruise,
                                  .agility = scenario.drone_agility},
                         scenario.drone_warhead,
                         Ring{.radius = scenario.spawn_distance,
                              .width = scenario.spawn_spread},
                         lib::Depend(random))
            .build());
    if (!first) {
      first = asset;
    }
  }
  world->sync();
  if (!first) {
    return std::unexpected(lib::raise(framework::BuildError::ENTITY_NOT_ALIVE,
                                      "A scenario needs at least one site."));
  }
  return *first;
}

// The missile simulation: builds the scenario when configured, and stops when
// red is defeated or the asset is destroyed. Any driver can run it.
class Simulation final {
 public:
  explicit Simulation(Scenario scenario = {}) : scenario_{scenario} {}

  // Builds the world and the scenario in it. A scenario too big for a world
  // fails this phase with the builder's Status.
  engine::PhaseResult configure() {
    RETURN_IF_UNEXPECTED(build_world(scenario_, lib::Out(world_)));
    ASSIGN_OR_RETURN(asset_, build_scenario(scenario_, lib::InOut(world_)));
    return engine::Flow::CONTINUE;
  }

  engine::PhaseResult step(const framework::Step& step) {
    scheduler_.step(step, lib::InOut(world_));
    if (!world_.alive(asset_)) {
      outcome_ = Outcome::RED_WINS;
    } else if (world_.store_of<RedDrone>().size() == 0) {
      outcome_ = Outcome::BLUE_WINS;
    }
    return outcome_ == Outcome::UNDECIDED ? engine::Flow::CONTINUE
                                          : engine::Flow::STOP;
  }

  Outcome outcome() const { return outcome_; }
  // The world: empty until configured.
  const World& world() const { return world_; }
  Entity asset() const { return asset_; }

  // Interceptors fired so far, from what the launchers have left.
  std::uint32_t interceptors_fired() const {
    std::uint32_t remaining = 0;
    world().store_of<Launcher>().for_each(
        [&](Entity, const Launcher& launcher) {
          remaining += launcher.inventory;
        });
    return static_cast<std::uint32_t>(scenario_.launchers) *
               scenario_.inventory -
           remaining;
  }

 private:
  Scenario scenario_;
  World world_;  // Empty until configure builds it.
  Scheduler scheduler_;
  Entity asset_;
  Outcome outcome_ = Outcome::UNDECIDED;
};

}  // namespace simon::missile
