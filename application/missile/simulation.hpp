// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <span>
#include <vector>

#include "application/missile/components.hpp"
#include "application/missile/systems.hpp"
#include "base/core.hpp"
#include "engine/event_queue.hpp"
#include "engine/lifecycle.hpp"
#include "framework/vocabulary.hpp"
#include "model/random.hpp"

namespace simon::missile {

// A circle an operator command applies to.
struct Sector final {
  Position center = model::meters(0.0, 0.0, 0.0);
  Length radius = 0.0 * model::meter;
};

// A weapons hold an operator orders ahead of time, over `sector` from `from`
// for `lasting`: to let friendly aircraft through, say.
struct TimedHold final {
  Sector sector;
  TimePoint from{};
  Duration lasting{};
};

// Raised when a timed hold ends. The simulation frees the hold's sector,
// except where another hold is still in force, so where holds overlap, a
// launcher stays held until the last of them ends.
struct WeaponsHoldExpired final {
  std::size_t hold = 0;  // Its position in Scenario::holds.
};

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
  // Whether each site's radars scan in turn; see SiteBuilder::scanning_in_turn.
  bool radars_in_turn = false;

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
  // Each site's drones fly at their own asset. Red wins when the first site's
  // asset is destroyed; blue wins when no red drones remain at any site.
  int sites = 1;
  Length site_spacing = 20000.0 * model::meter;

  // Weapons holds ordered ahead of time. Each starts on a timer, and when it
  // ends, a second timer raises WeaponsHoldExpired.
  std::vector<TimedHold> holds;
};

enum class Outcome { UNDECIDED, BLUE_WINS, RED_WINS };

// Builds in `world` the world a scenario needs: at every site, holding the
// asset, radars and launchers, every drone with its track and blast, and every
// interceptor with its blast.
auto build_world(const Scenario& scenario, Out<World> world)
    -> std::expected<void, framework::Status>;

// A point `radius` from the origin at `bearing` radians from east.
auto on_ring(Length radius, double bearing) -> Position;

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
  SiteBuilder(Position origin, Depend<World> world)
      : origin_{origin}, world_{world.get()} {}

  // The asset the site protects, and its health. 30 points unless given.
  auto protecting(Health health) && -> SiteBuilder {
    asset_health_ = health;
    return std::move(*this);
  }

  // `count` radars like `radar`, evenly spaced at `radius` from the asset.
  auto watched_by(std::size_t count, Radar radar,
                  Length radius) && -> SiteBuilder {
    radars_ = Placement<Radar>{.count = count, .unit = radar, .radius = radius};
    return std::move(*this);
  }

  // The radars scan in turn instead of together: radar i first scans i / count
  // of the way through its scan period. That spreads the scanning over the
  // period, lowering the slowest step, but every step with a scan pays for it,
  // so the average step costs more.
  auto scanning_in_turn() && -> SiteBuilder {
    in_turn_ = true;
    return std::move(*this);
  }

  // `count` launchers like `launcher`, evenly spaced at `radius` from the
  // asset, starting half a spacing from the radars.
  auto defended_by(std::size_t count, Launcher launcher,
                   Length radius) && -> SiteBuilder {
    launchers_ =
        Placement<Launcher>{.count = count, .unit = launcher, .radius = radius};
    return std::move(*this);
  }

  // `count` red drones like `drone`, carrying `warhead`, spawning at random
  // bearings within `ring` and flying at the asset at cruise speed. Draws
  // from `random`, which it keeps until the utterance is built.
  auto attacked_by(std::size_t count, RedDrone drone, Warhead warhead,
                   Ring ring, Depend<model::Random> random) && -> SiteBuilder {
    drones_ = count;
    drone_ = drone;
    warhead_ = warhead;
    spawn_ = ring;
    random_ = random.get();
    return std::move(*this);
  }

  // Creates the site and returns its asset, atomically: if the world refuses
  // any entity, nothing of the site is planned, and build() returns the
  // world's Status. Draws from the random generator are not undone.
  auto build() && -> std::expected<Entity, framework::Status>;

 private:
  template <typename UnitType>
  struct Placement final {
    std::size_t count = 0;
    UnitType unit{};
    Length radius = 0.0 * model::meter;
  };

  Position origin_;
  World* world_ = nullptr;
  Health asset_health_{.points = 30.0};
  Placement<Radar> radars_;
  bool in_turn_ = false;
  Placement<Launcher> launchers_;
  std::size_t drones_ = 0;
  RedDrone drone_;
  Warhead warhead_;
  Ring spawn_;
  model::Random* random_ = nullptr;  // Set with any drones.
};

// Starts the utterance that builds a defended site at `origin` in `world`.
auto create_site(Position origin, Depend<World> world) -> SiteBuilder;

// Builds every site of a scenario on a square grid, the first at the origin,
// and returns the first site's asset.
auto build_scenario(const Scenario& scenario, InOut<World> world)
    -> std::expected<Entity, framework::Status>;

//-- Operator commands
//----------------------------------------------------------

// Operator commands. Each is one query form: it selects what it applies to,
// changes all of it or none, applies at the next sync, and returns how many
// entities it affected.

// Holds every launcher in `sector`, so none engages until freed. Launchers
// already held, or held by a command still pending, are skipped.
auto hold_weapons(const Sector& sector, InOut<World> world)
    -> std::expected<std::size_t, framework::Status>;

// Frees every held launcher in `sector` to engage again, except those inside
// any sector of `keeping`, which stay held without being freed and held again.
auto free_weapons(const Sector& sector, std::span<const Sector> keeping,
                  InOut<World> world)
    -> std::expected<std::size_t, framework::Status>;

// Destroys every interceptor in flight in `sector`. The tracks they were
// engaging stay engaged until their engagements lapse.
auto destruct_interceptors(const Sector& sector, InOut<World> world)
    -> std::expected<std::size_t, framework::Status>;

// The missile simulation: builds the scenario when configured, and stops when
// red is defeated or the asset is destroyed. Any driver can run it.
class Simulation final {
 public:
  explicit Simulation(Scenario scenario = {}) : scenario_{scenario} {}

  // Builds the world and the scenario in it, and starts a timer for each
  // timed hold. A scenario too big for a world fails this phase with the
  // builder's Status.
  auto configure() -> engine::PhaseResult;

  // Delivers the events due by the step's time, then runs the schedule.
  auto step(const framework::Step& step) -> engine::PhaseResult;

  auto outcome() const -> Outcome { return outcome_; }
  // The world: empty until configured.
  auto world() const -> const World& { return world_; }
  auto asset() const -> Entity { return asset_; }
  // The events the simulation raises, such as WeaponsHoldExpired, for
  // subscribers.
  auto events() -> engine::EventQueue& { return events_; }

  // Interceptors fired so far, from what the launchers have left.
  auto interceptors_fired() const -> std::uint32_t;

  // Operator commands on the simulation's world; see hold_weapons,
  // free_weapons and destruct_interceptors.
  auto hold_weapons(const Sector& sector)
      -> std::expected<std::size_t, framework::Status>;
  auto free_weapons(const Sector& sector)
      -> std::expected<std::size_t, framework::Status>;
  auto destruct_interceptors(const Sector& sector)
      -> std::expected<std::size_t, framework::Status>;

 private:
  auto remaining_interceptors() const -> std::uint32_t;
  // Holds a timed hold's sector, and starts the timer that ends it.
  auto start_hold(TimePoint now, std::size_t hold) -> void;
  // Frees an expired hold's sector, except where another hold is in force.
  auto end_hold(TimePoint now, const WeaponsHoldExpired& expired) -> void;

  Scenario scenario_;
  World world_;  // Empty until configure builds it.
  Scheduler scheduler_;
  engine::EventQueue events_;
  Entity asset_;
  Outcome outcome_ = Outcome::UNDECIDED;
  std::uint32_t stock_ = 0;  // Interceptors the launchers held when built.
};

}  // namespace simon::missile
