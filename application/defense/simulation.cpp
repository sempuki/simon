// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "application/defense/simulation.hpp"
#include "core/vocabulary.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <optional>

namespace simon::defense {

auto build_world(const Scenario& scenario, Out<World> world)
    -> std::expected<void, framework::Status> {
  auto count = [](int value) {
    return static_cast<std::size_t>(std::max(value, 0));
  };
  std::size_t sites = count(scenario.sites);
  std::size_t drones = sites * count(scenario.drones);
  std::size_t interceptors =
      sites * count(scenario.launchers) * scenario.inventory;
  return World::set_up()
      .numbered(1)
      .holding<archetype::Asset>(sites)
      .holding<archetype::Radar>(sites * count(scenario.radars))
      .holding<archetype::Launcher>(sites * count(scenario.launchers))
      .holding<archetype::RedDrone>(drones)
      .holding<archetype::Track>(drones)
      .holding<archetype::Interceptor>(interceptors)
      .holding<archetype::Blast>(drones + interceptors)
      .build(world);
}

auto on_ring(Length radius, double bearing) -> Position {
  double length = radius.numerical_value_in(meter);
  return meters(length * std::cos(bearing), length * std::sin(bearing), 0.0);
}

auto SiteBuilder::build() && -> std::expected<Entity, framework::Status> {
  constexpr double TURN = 2.0 * std::numbers::pi;
  // Any early return rolls back the entities created so far.
  auto transaction = world_->transaction();
  RETURN_OR_ASSIGN(Entity asset, world_->create<archetype::Asset>("asset")
                                     .with(Kinematics{.position = origin_})
                                     .with(asset_health_)
                                     .with(Asset{})
                                     .build());
  for (std::size_t i = 0; i < radars_.count; ++i) {
    double bearing =
        TURN * static_cast<double>(i) / static_cast<double>(radars_.count);
    Radar radar = radars_.unit;
    if (in_turn_) {
      Duration period = radar.scan.period();
      radar.scan = engine::RateGate{
          period, radar.scan.catch_up(),
          TimePoint{} + period * static_cast<std::int64_t>(i) /
                            static_cast<std::int64_t>(radars_.count)};
    }
    RETURN_IF_UNEXPECTED(
        world_->create<archetype::Radar>()
            .with(Kinematics{.position =
                                 origin_ + on_ring(radars_.radius, bearing)})
            .with(radar)
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

auto create_site(Position origin, Depend<World> world) -> SiteBuilder {
  return SiteBuilder{origin, world};
}

auto build_scenario(const Scenario& scenario, InOut<World> world)
    -> std::expected<Entity, framework::Status> {
  if (scenario.sites < 1) {
    return std::unexpected(lib::raise(framework::BuildError::ENTITY_NOT_ALIVE,
                                      "A scenario needs at least one site."));
  }
  auto count = [](int value) {
    return static_cast<std::size_t>(std::max(value, 0));
  };
  Random random{scenario.seed};
  int side = static_cast<int>(std::ceil(std::sqrt(scenario.sites)));
  double spacing = scenario.site_spacing.numerical_value_in(meter);
  std::optional<Entity> first;
  for (int site = 0; site < scenario.sites; ++site) {
    Position origin =
        meters(spacing * (site % side), spacing * (site / side), 0.0);
    SiteBuilder watched =
        create_site(origin, Depend(*world))
            .protecting(Health{.points = scenario.asset_health})
            .watched_by(count(scenario.radars),
                        Radar{.range = scenario.radar_range,
                              .scan = engine::RateGate{scenario.scan_period}},
                        scenario.radar_ring);
    if (scenario.radars_in_turn) {
      watched = std::move(watched).scanning_in_turn();
    }
    RETURN_OR_ASSIGN(
        Entity asset,
        std::move(watched)
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
                         Depend(random))
            .build());
    if (!first) {
      first = asset;
    }
  }
  world->sync();
  CHECK_INVARIANT(first.has_value());
  return *first;
}

auto Simulation::configure() -> engine::PhaseResult {
  RETURN_IF_UNEXPECTED(build_world(scenario_, Out(world_)));
  RETURN_OR_ASSIGN(asset_, build_scenario(scenario_, InOut(world_)));
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

auto Simulation::step(const Step& step) -> engine::PhaseResult {
  events_.process_until(step.time);
  scheduler_.step(step, InOut(world_));
  if (!world_.alive(asset_)) {
    outcome_ = Outcome::RED_WINS;
  } else if (world_.store_of<RedDrone>().size() == 0) {
    outcome_ = Outcome::BLUE_WINS;
  }
  return outcome_ == Outcome::UNDECIDED ? engine::Flow::CONTINUE
                                        : engine::Flow::STOP;
}

auto hold_weapons(const Sector& sector, InOut<World> world)
    -> std::expected<std::size_t, framework::Status> {
  return world->change()
      .each<archetype::Launcher>()
      .within(Kinematics{.position = sector.center}, sector.radius)
      .lacking<WeaponsHold>()
      .attach(WeaponsHold{})
      .build();
}

auto free_weapons(const Sector& sector, std::span<const Sector> keeping,
                  InOut<World> world)
    -> std::expected<std::size_t, framework::Status> {
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

auto destruct_interceptors(const Sector& sector, InOut<World> world)
    -> std::expected<std::size_t, framework::Status> {
  return world->destroy()
      .each<archetype::Interceptor>()
      .within(Kinematics{.position = sector.center}, sector.radius)
      .build();
}

auto Simulation::hold_weapons(const Sector& sector)
    -> std::expected<std::size_t, framework::Status> {
  return defense::hold_weapons(sector, InOut(world_));
}

auto Simulation::free_weapons(const Sector& sector)
    -> std::expected<std::size_t, framework::Status> {
  return defense::free_weapons(sector, {}, InOut(world_));
}

auto Simulation::destruct_interceptors(const Sector& sector)
    -> std::expected<std::size_t, framework::Status> {
  return defense::destruct_interceptors(sector, InOut(world_));
}

// Holds and frees cannot be refused: launchers allow WeaponsHold, the store
// holds one for every launcher, and lacking and having skip launchers already
// held or freed.
auto Simulation::start_hold(TimePoint now, std::size_t hold) -> void {
  const TimedHold& order = scenario_.holds[hold];
  auto held = defense::hold_weapons(order.sector, InOut(world_));
  CHECK_INVARIANT(held.has_value());
  events_.start_timer(now + order.lasting, [this, hold](TimePoint expiry) {
    events_.publish<WeaponsHoldExpired>(expiry,
                                        WeaponsHoldExpired{.hold = hold});
  });
}

auto Simulation::end_hold(TimePoint now, const WeaponsHoldExpired& expired)
    -> void {
  std::vector<Sector> in_force;
  for (const TimedHold& order : scenario_.holds) {
    if (order.from <= now && now < order.from + order.lasting) {
      in_force.push_back(order.sector);
    }
  }
  auto freed = defense::free_weapons(scenario_.holds[expired.hold].sector,
                                     in_force, InOut(world_));
  CHECK_INVARIANT(freed.has_value());
}

auto Simulation::interceptors_fired() const -> std::uint32_t {
  return stock_ - remaining_interceptors();
}

auto Simulation::remaining_interceptors() const -> std::uint32_t {
  std::uint32_t remaining = 0;
  world_.store_of<Launcher>().for_each([&](Entity, const Launcher& launcher) {
    remaining += launcher.inventory;
  });
  return remaining;
}

}  // namespace simon::defense
