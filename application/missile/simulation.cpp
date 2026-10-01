// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "application/missile/simulation.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <optional>

namespace simon::missile {

std::expected<void, framework::Status> build_world(const Scenario& scenario,
                                                   lib::Out<World> world) {
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

Position on_ring(Length radius, double bearing) {
  double length = radius.numerical_value_in(model::meter);
  return model::meters(length * std::cos(bearing), length * std::sin(bearing),
                       0.0);
}

std::expected<Entity, framework::Status> SiteBuilder::build() && {
  constexpr double TURN = 2.0 * std::numbers::pi;
  // Any early return rolls back the entities created so far.
  auto transaction = world_->transaction();
  ASSIGN_OR_RETURN(Entity asset, world_->create<archetype::Asset>("asset")
                                     .with(Kinematics{.position = origin_})
                                     .with(asset_health_)
                                     .with(Asset{})
                                     .build());
  for (std::size_t i = 0; i < radars_.count; ++i) {
    double bearing =
        TURN * static_cast<double>(i) / static_cast<double>(radars_.count);
    RETURN_IF_UNEXPECTED(
        world_->create<archetype::Radar>()
            .with(Kinematics{.position =
                                 origin_ + on_ring(radars_.radius, bearing)})
            .with(radars_.unit)
            .build());
  }
  for (std::size_t i = 0; i < launchers_.count; ++i) {
    double bearing = TURN * (static_cast<double>(i) + 0.5) /
                     static_cast<double>(launchers_.count);
    RETURN_IF_UNEXPECTED(
        world_->create<archetype::Launcher>()
            .with(Kinematics{.position =
                                 origin_ + on_ring(launchers_.radius, bearing)})
            .with(launchers_.unit)
            .build());
  }
  for (std::size_t i = 0; i < drones_; ++i) {
    double bearing = random_->uniform(0.0, TURN);
    Length radius = spawn_.radius + spawn_.width * random_->uniform(-0.5, 0.5);
    Kinematics kinematics{.position = origin_ + on_ring(radius, bearing)};
    kinematics.velocity = (origin_ - kinematics.position) *
                          (drone_.cruise / norm(origin_ - kinematics.position));
    RETURN_IF_UNEXPECTED(world_->create<archetype::RedDrone>()
                             .with(kinematics)
                             .with(Control{})
                             .with(Health{.points = 1.0})
                             .with(warhead_)
                             .with(Target{.entity = asset})
                             .with(drone_)
                             .build());
  }
  transaction.commit();
  return asset;
}

SiteBuilder create_site(Position origin, lib::Depend<World> world) {
  return SiteBuilder{origin, world};
}

std::expected<Entity, framework::Status> build_scenario(
    const Scenario& scenario, lib::InOut<World> world) {
  if (scenario.sites < 1) {
    return std::unexpected(lib::raise(framework::BuildError::ENTITY_NOT_ALIVE,
                                      "A scenario needs at least one site."));
  }
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
  CHECK_INVARIANT(first.has_value());
  return *first;
}

engine::PhaseResult Simulation::configure() {
  RETURN_IF_UNEXPECTED(build_world(scenario_, lib::Out(world_)));
  ASSIGN_OR_RETURN(asset_, build_scenario(scenario_, lib::InOut(world_)));
  stock_ = remaining_interceptors();
  for (std::size_t hold = 0; hold < scenario_.holds.size(); ++hold) {
    events_.start_timer(scenario_.holds[hold].from,
                        [this, hold](TimePoint now) { start_hold(now, hold); });
  }
  events_.subscribe<WeaponsHoldExpired>(
      [this](TimePoint now, const WeaponsHoldExpired& expired) {
        end_hold(now, expired);
      });
  return engine::Flow::CONTINUE;
}

engine::PhaseResult Simulation::step(const framework::Step& step) {
  events_.process_until(step.time);
  scheduler_.step(step, lib::InOut(world_));
  if (!world_.alive(asset_)) {
    outcome_ = Outcome::RED_WINS;
  } else if (world_.store_of<RedDrone>().size() == 0) {
    outcome_ = Outcome::BLUE_WINS;
  }
  return outcome_ == Outcome::UNDECIDED ? engine::Flow::CONTINUE
                                        : engine::Flow::STOP;
}

std::expected<std::size_t, framework::Status> hold_weapons(
    const Sector& sector, lib::InOut<World> world) {
  return world->change()
      .each<archetype::Launcher>()
      .within(Kinematics{.position = sector.center}, sector.radius)
      .lacking<WeaponsHold>()
      .attach(WeaponsHold{})
      .build();
}

std::expected<std::size_t, framework::Status> free_weapons(
    const Sector& sector, std::span<const Sector> keeping,
    lib::InOut<World> world) {
  const auto& kinematics = world->store_of<Kinematics>();
  return world->change()
      .each<archetype::Launcher>()
      .within(Kinematics{.position = sector.center}, sector.radius)
      .having<WeaponsHold>()
      .where([&](Entity launcher) {
        Position position = kinematics.component_of(launcher).position;
        return std::ranges::none_of(keeping, [&](const Sector& kept) {
          return distance_between(position, kept.center) <= kept.radius;
        });
      })
      .detach<WeaponsHold>()
      .build();
}

std::expected<std::size_t, framework::Status> destruct_interceptors(
    const Sector& sector, lib::InOut<World> world) {
  return world->destroy()
      .each<archetype::Interceptor>()
      .within(Kinematics{.position = sector.center}, sector.radius)
      .build();
}

std::expected<std::size_t, framework::Status> Simulation::hold_weapons(
    const Sector& sector) {
  return missile::hold_weapons(sector, lib::InOut(world_));
}

std::expected<std::size_t, framework::Status> Simulation::free_weapons(
    const Sector& sector) {
  return missile::free_weapons(sector, {}, lib::InOut(world_));
}

std::expected<std::size_t, framework::Status> Simulation::destruct_interceptors(
    const Sector& sector) {
  return missile::destruct_interceptors(sector, lib::InOut(world_));
}

// Holds and frees cannot be refused: launchers allow WeaponsHold, the store
// holds one for every launcher, and lacking and having skip launchers already
// held or freed. A launcher is never freed and held again in one batch, which
// a full store would refuse, since capacity checks do not count pending
// detaches.
void Simulation::start_hold(TimePoint now, std::size_t hold) {
  const TimedHold& order = scenario_.holds[hold];
  auto held = missile::hold_weapons(order.sector, lib::InOut(world_));
  CHECK_INVARIANT(held.has_value());
  events_.start_timer(now + order.lasting, [this, hold](TimePoint expiry) {
    events_.publish<WeaponsHoldExpired>(expiry,
                                        WeaponsHoldExpired{.hold = hold});
  });
}

void Simulation::end_hold(TimePoint now, const WeaponsHoldExpired& expired) {
  std::vector<Sector> in_force;
  for (const TimedHold& order : scenario_.holds) {
    if (order.from <= now && now < order.from + order.lasting) {
      in_force.push_back(order.sector);
    }
  }
  auto freed = missile::free_weapons(scenario_.holds[expired.hold].sector,
                                     in_force, lib::InOut(world_));
  CHECK_INVARIANT(freed.has_value());
}

std::uint32_t Simulation::interceptors_fired() const {
  return stock_ - remaining_interceptors();
}

std::uint32_t Simulation::remaining_interceptors() const {
  std::uint32_t remaining = 0;
  world_.store_of<Launcher>().for_each([&](Entity, const Launcher& launcher) {
    remaining += launcher.inventory;
  });
  return remaining;
}

}  // namespace simon::missile
