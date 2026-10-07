// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <chrono>
#include <expected>
#include <vector>

#include "application/defense/simulation.hpp"
#include "base/testing.hpp"
#include "core/argument.hpp"
#include "engine/driver.hpp"

namespace simon::defense {

namespace {

using namespace std::chrono_literals;

constexpr Duration DT = 10ms;
// Builds a world holding 16 of each archetype, for testing systems alone.
auto build_small_world(Out<World> world) -> void {
  std::expected<void, framework::Status> built =
      World::set_up()
          .numbered(1)
          .holding<archetype::Asset>(16)
          .holding<archetype::Radar>(16)
          .holding<archetype::Launcher>(16)
          .holding<archetype::RedDrone>(16)
          .holding<archetype::Track>(16)
          .holding<archetype::Interceptor>(16)
          .holding<archetype::Blast>(16)
          .build(world);
  CHECK_POSTCONDITION(built.has_value());
}

struct Run final {
  Outcome outcome;
  TimePoint end;
  std::uint32_t fired;
  double asset_health;
};

auto run(Scenario scenario) -> Run {
  Simulation simulation{scenario};
  engine::BatchDriver driver{engine::Timing{.max_step = DT},
                             Depend(simulation)};
  auto end = driver.run(TimePoint{10min});
  REQUIRE(end);
  const Health* asset =
      simulation.world().store_of<Health>().maybe_component_of(
          simulation.asset());
  return Run{.outcome = simulation.outcome(),
             .end = *end,
             .fired = simulation.interceptors_fired(),
             .asset_health = asset ? asset->points : 0.0};
}

// Steps `simulation` from `from` until `until`, DT at a time.
auto advance(TimePoint from, TimePoint until, InOut<Simulation> simulation)
    -> void {
  for (TimePoint time = from; time < until; time += DT) {
    REQUIRE(simulation->step(Step{.time = time, .dt = DT}));
  }
}

// Configures `simulation`, then steps it until `until`, DT at a time.
auto run_until(TimePoint until, InOut<Simulation> simulation) -> void {
  REQUIRE(simulation->configure());
  advance(TimePoint{}, until, simulation);
}

template <typename ScheduleType>
auto step(TimePoint time,
          InOut<framework::Scheduler<World, ScheduleType>> scheduler,
          InOut<World> world) -> void {
  scheduler->step(Step{.time = time, .dt = DT}, world);
}

auto make_drone(Position position, Entity target, InOut<World> world)
    -> Entity {
  return *world->create<archetype::RedDrone>()
              .with(Kinematics{.position = position})
              .with(Control{})
              .with(Health{.points = 1.0})
              .with(Warhead{.fuse = 30.0 * meter,
                            .radius = 100.0 * meter,
                            .damage = 10.0})
              .with(Target{.entity = target})
              .with(RedDrone{})
              .build();
}

auto make_radar(Position position, InOut<World> world) -> Entity {
  return *world->create<archetype::Radar>()
              .with(Kinematics{.position = position})
              .with(
                  Radar{.range = 1000.0 * meter, .scan = engine::RateGate{1s}})
              .build();
}

auto make_launcher(Position position, InOut<World> world) -> Entity {
  return *world->create<archetype::Launcher>()
              .with(Kinematics{.position = position})
              .with(Launcher{
                  .range = 3000.0 * meter, .inventory = 5, .reload = 2s})
              .build();
}

auto make_track(Entity target, Position position, InOut<World> world)
    -> Entity {
  return *world->create<archetype::Track>()
              .with(Track{.target = target})
              .with(Estimate{.position = position})
              .with(Engagement{})
              .build();
}

auto make_interceptor(Position position, Entity target, TimePoint expires_at,
                      InOut<World> world) -> Entity {
  return *world->create<archetype::Interceptor>()
              .with(Kinematics{.position = position})
              .with(Control{})
              .with(InterceptorDesign{}.warhead)
              .with(Target{.entity = target})
              .with(Interceptor{.speed = 150.0 * meter_per_second,
                                .agility = 300.0 * meter_per_second_squared,
                                .seeker_range = 1000.0 * meter,
                                .expires_at = expires_at})
              .build();
}

// The owners of every `ComponentType`, in store order.
template <typename ComponentType>
auto collect_owners(const World& world) -> std::vector<Entity> {
  std::vector<Entity> owners;
  world.store_of<ComponentType>().for_each(
      [&](Entity owner, const ComponentType&) { owners.push_back(owner); });
  return owners;
}

}  // namespace

TEST_CASE("DefenseSimulation") {
  SECTION("ShouldBlueWinGivenDefaultScenario") {
    Run result = run(Scenario{});
    CHECK(result.outcome == Outcome::BLUE_WINS);
    CHECK(result.asset_health > 0.0);
  }

  SECTION("ShouldRepeatExactlyGivenSameSeed") {
    Run first = run(Scenario{.seed = 7});
    Run second = run(Scenario{.seed = 7});
    CHECK(first.outcome == second.outcome);
    CHECK(first.end == second.end);
    CHECK(first.fired == second.fired);
    CHECK(first.asset_health == second.asset_health);
  }

  SECTION("ShouldDifferGivenDifferentSeeds") {
    CHECK(run(Scenario{.seed = 1}).end != run(Scenario{.seed = 3}).end);
  }

  SECTION("ShouldRedWinGivenNoLaunchers") {
    Run result = run(Scenario{.launchers = 0});
    CHECK(result.outcome == Outcome::RED_WINS);
    CHECK(result.fired == 0u);
  }

  SECTION("ShouldRedWinGivenMoreDronesThanInterceptors") {
    Run result = run(Scenario{.inventory = 3, .drones = 40});
    CHECK(result.outcome == Outcome::RED_WINS);
    CHECK(result.fired == 9u);  // Every interceptor was used.
  }

  SECTION("ShouldCountNoneFiredGivenSeveralSitesBeforeAnyLaunch") {
    Simulation simulation{Scenario{.drones = 10, .sites = 4}};
    CHECK(simulation.interceptors_fired() == 0u);  // Before configure.

    REQUIRE(simulation.configure());

    CHECK(simulation.interceptors_fired() == 0u);
  }

  SECTION("ShouldHoldFireUntilHoldExpiresGivenTimedHold") {
    Scenario scenario;
    scenario.holds = {TimedHold{.sector = {.radius = 1000.0 * meter},
                                .from = TimePoint{},
                                .lasting = 90s}};
    Simulation simulation{scenario};
    std::vector<TimePoint> expired;
    simulation.events().subscribe<WeaponsHoldExpired>(
        [&](TimePoint time, const WeaponsHoldExpired&) {
          expired.push_back(time);
        });
    Simulation unheld;

    run_until(TimePoint{90s}, InOut(simulation));
    run_until(TimePoint{90s}, InOut(unheld));
    REQUIRE(unheld.interceptors_fired() > 0u);  // So the hold mattered.
    CHECK(simulation.interceptors_fired() == 0u);
    CHECK(expired.empty());

    advance(TimePoint{90s}, TimePoint{100s}, InOut(simulation));
    CHECK(expired == std::vector{TimePoint{90s}});
    CHECK(simulation.interceptors_fired() > 0u);
  }

  SECTION("ShouldStayHeldGivenOverlappingHoldExpiresFirst") {
    Scenario scenario;
    const Sector home{.radius = 1000.0 * meter};
    scenario.holds = {
        TimedHold{.sector = home, .from = TimePoint{}, .lasting = 120s},
        TimedHold{.sector = home, .from = TimePoint{30s}, .lasting = 30s}};
    Simulation simulation{scenario};

    run_until(TimePoint{100s}, InOut(simulation));

    CHECK(simulation.interceptors_fired() == 0u);
    CHECK(simulation.world().store_of<WeaponsHold>().size() == 3u);
  }

  SECTION("ShouldFailConfigureGivenNoSites") {
    for (int sites : {0, -1}) {
      Simulation simulation{Scenario{.sites = sites}};
      CHECK_FALSE(simulation.configure().has_value());
    }
  }

  SECTION("ShouldLeaveNothingOfSiteGivenWorldRefusesADrone") {
    Scenario scenario{.radars = 3, .launchers = 3, .drones = 10};
    World world;
    // Room for the asset, radars and launchers, but only half the drones.
    REQUIRE(World::set_up()
                .holding<archetype::Asset>(1)
                .holding<archetype::Radar>(3)
                .holding<archetype::Launcher>(3)
                .holding<archetype::RedDrone>(5)
                .build(Out(world)));

    std::expected<Entity, framework::Status> asset =
        build_scenario(scenario, InOut(world));
    world.sync();

    REQUIRE_FALSE(asset.has_value());
    CHECK(asset.error() ==
          lib::watch(framework::BuildError::ENTITY_CAPACITY_EXHAUSTED));
    CHECK(world.size() == 0u);
    // "asset" still names the archetype, but no entity.
    for (framework::Name name : world.find_name_of(framework::Alias{"asset"})) {
      CHECK(name.kind != static_cast<std::uint32_t>(framework::Kind::ENTITY));
    }
  }

  SECTION("ShouldAimEachSitesDronesAtItsOwnAssetGivenSeveralSites") {
    Scenario scenario{.drones = 10, .sites = 4};
    World world;
    REQUIRE(build_world(scenario, Out(world)));
    std::expected<Entity, framework::Status> first =
        build_scenario(scenario, InOut(world));
    REQUIRE(first.has_value());

    std::vector<Entity> assets = collect_owners<Asset>(world);
    REQUIRE(assets.size() == 4u);
    CHECK(assets.front() == *first);
    CHECK(collect_owners<RedDrone>(world).size() == 40u);
    CHECK(collect_owners<Radar>(world).size() == 12u);
    // Each drone flies at the asset of the site it spawned in, which is the
    // nearest asset: sites are 20 km apart and drones spawn within 6.5 km.
    world.store_of<Target>().for_each([&](Entity drone, const Target& target) {
      if (!world.store_of<RedDrone>().contains(drone)) {
        return;
      }
      const Position& at =
          world.store_of<Kinematics>().component_of(drone).position;
      Entity nearest = assets.front();
      for (Entity asset : assets) {
        auto distance_to = [&](Entity to) {
          return norm(world.store_of<Kinematics>().component_of(to).position -
                      at);
        };
        nearest = distance_to(asset) < distance_to(nearest) ? asset : nearest;
      }
      CHECK(target.entity == nearest);
    });
  }
}

TEST_CASE("ScanRadars") {
  World world;
  build_small_world(Out(world));
  framework::Scheduler<World, SystemList<ScanRadars>> scheduler;
  Radar radar{.range = 1000.0 * meter, .scan = engine::RateGate{1s}};

  // Counts a site's radars that scanned on each step of the first second.
  auto scans_per_step = [&](SiteBuilder site) {
    REQUIRE(std::move(site).watched_by(4, radar, 500.0 * meter).build());
    world.sync();
    std::vector<int> scans;
    for (TimePoint time{}; time < TimePoint{1s}; time += 250ms) {
      scheduler.step(Step{.time = time, .dt = 250ms}, InOut(world));
      int scanned = 0;
      world.store_of<Radar>().for_each(
          [&](Entity, const Radar& radar) { scanned += radar.scanned; });
      scans.push_back(scanned);
    }
    return scans;
  };

  SECTION("ShouldScanTogetherGivenSite") {
    CHECK(scans_per_step(create_site(meters(0, 0, 0), Depend(world))) ==
          std::vector<int>{4, 0, 0, 0});
  }

  SECTION("ShouldScanInTurnGivenSiteScanningInTurn") {
    CHECK(scans_per_step(
              create_site(meters(0, 0, 0), Depend(world)).scanning_in_turn()) ==
          std::vector<int>{1, 1, 1, 1});
  }
}

TEST_CASE("DetectDrones") {
  World world;
  build_small_world(Out(world));
  framework::Scheduler<World, SystemList<ScanRadars, DetectDrones>> scheduler;

  SECTION("ShouldCreateOneTrackGivenTwoRadarsSeeingOneDrone") {
    make_radar(meters(0.0, 0.0, 0.0), InOut(world));
    make_radar(meters(100.0, 0.0, 0.0), InOut(world));
    Entity drone = make_drone(meters(500.0, 0.0, 0.0), Entity{}, InOut(world));
    world.sync();

    step(TimePoint{}, InOut(scheduler), InOut(world));

    REQUIRE(world.store_of<Track>().size() == 1u);
    Entity track = collect_owners<Track>(world).front();
    CHECK(world.store_of<Track>().component_of(track).target == drone);
    CHECK(world.store_of<Tracked>().component_of(drone).track == track);
  }

  SECTION("ShouldNotTrackGivenDroneOutOfRange") {
    make_radar(meters(0.0, 0.0, 0.0), InOut(world));
    make_drone(meters(5000.0, 0.0, 0.0), Entity{}, InOut(world));
    world.sync();

    step(TimePoint{}, InOut(scheduler), InOut(world));

    CHECK(world.store_of<Track>().size() == 0u);
  }

  SECTION("ShouldNotTrackAgainGivenDroneAlreadyTracked") {
    make_radar(meters(0.0, 0.0, 0.0), InOut(world));
    make_drone(meters(500.0, 0.0, 0.0), Entity{}, InOut(world));
    world.sync();

    step(TimePoint{}, InOut(scheduler), InOut(world));
    step(TimePoint{1s}, InOut(scheduler),
         InOut(world));  // The next scan.

    CHECK(world.store_of<Track>().size() == 1u);
  }
}

TEST_CASE("UpdateTracks") {
  World world;
  build_small_world(Out(world));
  framework::Scheduler<World, SystemList<ScanRadars, UpdateTracks>> scheduler;

  SECTION("ShouldUpdateEstimateGivenScanningRadarCoversTarget") {
    make_radar(meters(0.0, 0.0, 0.0), InOut(world));
    Entity drone = make_drone(meters(500.0, 0.0, 0.0), Entity{}, InOut(world));
    Entity track = make_track(drone, meters(0.0, 0.0, 0.0), InOut(world));
    world.sync();

    step(TimePoint{2s}, InOut(scheduler), InOut(world));

    CHECK(world.store_of<Estimate>().component_of(track).position ==
          meters(500.0, 0.0, 0.0));
    CHECK(world.store_of<Track>().component_of(track).last_seen ==
          TimePoint{2s});
  }

  SECTION("ShouldKeepEstimateGivenTargetOutOfRange") {
    make_radar(meters(0.0, 0.0, 0.0), InOut(world));
    Entity drone = make_drone(meters(5000.0, 0.0, 0.0), Entity{}, InOut(world));
    Entity track = make_track(drone, meters(4500.0, 0.0, 0.0), InOut(world));
    world.sync();

    step(TimePoint{2s}, InOut(scheduler), InOut(world));

    CHECK(world.store_of<Estimate>().component_of(track).position ==
          meters(4500.0, 0.0, 0.0));
    CHECK(world.store_of<Track>().component_of(track).last_seen == TimePoint{});
  }
}

TEST_CASE("DropStaleTracks") {
  World world;
  build_small_world(Out(world));
  framework::Scheduler<World, SystemList<DropStaleTracks>> scheduler;

  SECTION("ShouldDropTrackAndUnmarkDroneGivenNotSeenForTimeout") {
    Entity drone = make_drone(meters(0.0, 0.0, 0.0), Entity{}, InOut(world));
    Entity track = make_track(drone, meters(0.0, 0.0, 0.0), InOut(world));
    world.sync();
    REQUIRE(world.change(drone).attach(Tracked{.track = track}).build());
    world.sync();

    step(TimePoint{6s}, InOut(scheduler), InOut(world));

    CHECK_FALSE(world.alive(track));
    CHECK_FALSE(world.store_of<Tracked>().contains(drone));
  }
}

TEST_CASE("Engaging") {
  World world;
  build_small_world(Out(world));
  framework::Scheduler<World, Engaging> scheduler;

  SECTION("ShouldLaunchOneInterceptorGivenTwoLaunchersProposingOneTrack") {
    make_launcher(meters(0.0, 0.0, 0.0), InOut(world));
    make_launcher(meters(50.0, 0.0, 0.0), InOut(world));
    Entity drone = make_drone(meters(2000.0, 0.0, 0.0), Entity{}, InOut(world));
    make_track(drone, meters(2000.0, 0.0, 0.0), InOut(world));
    world.sync();

    step(TimePoint{}, InOut(scheduler), InOut(world));

    REQUIRE(world.store_of<Interceptor>().size() == 1u);
    // The nearer launcher, at 50 m, won the engagement.
    Entity interceptor = collect_owners<Interceptor>(world).front();
    Entity launcher = *world.parent_of(interceptor);
    CHECK(world.store_of<Kinematics>().component_of(launcher).position ==
          meters(50.0, 0.0, 0.0));
    CHECK(world.store_of<Target>().component_of(interceptor).entity == drone);
  }

  SECTION("ShouldWaitForReloadGivenSecondTrack") {
    Entity launcher = make_launcher(meters(0.0, 0.0, 0.0), InOut(world));
    for (double x : {1000.0, 2000.0}) {
      Entity drone = make_drone(meters(x, 0.0, 0.0), Entity{}, InOut(world));
      make_track(drone, meters(x, 0.0, 0.0), InOut(world));
    }
    world.sync();

    step(TimePoint{}, InOut(scheduler), InOut(world));
    step(TimePoint{1s}, InOut(scheduler), InOut(world));
    CHECK(world.store_of<Interceptor>().size() == 1u);  // Reloading.

    step(TimePoint{2s}, InOut(scheduler), InOut(world));
    CHECK(world.store_of<Interceptor>().size() == 2u);
    CHECK(world.store_of<Launcher>().component_of(launcher).inventory == 3u);
  }

  SECTION("ShouldNotEngageGivenTrackOutOfRange") {
    make_launcher(meters(0.0, 0.0, 0.0), InOut(world));
    Entity drone = make_drone(meters(9000.0, 0.0, 0.0), Entity{}, InOut(world));
    make_track(drone, meters(9000.0, 0.0, 0.0), InOut(world));
    world.sync();

    step(TimePoint{}, InOut(scheduler), InOut(world));

    CHECK(world.store_of<Interceptor>().size() == 0u);
  }
}

TEST_CASE("OperatorCommands") {
  World world;
  build_small_world(Out(world));
  framework::Scheduler<World, Engaging> scheduler;
  const Sector home{.center = meters(0.0, 0.0, 0.0), .radius = 500.0 * meter};
  Entity drone = make_drone(meters(2000.0, 0.0, 0.0), Entity{}, InOut(world));
  make_track(drone, meters(2000.0, 0.0, 0.0), InOut(world));

  SECTION("ShouldNotEngageGivenWeaponsHold") {
    make_launcher(meters(0.0, 0.0, 0.0), InOut(world));
    world.sync();

    auto held = hold_weapons(home, InOut(world));
    world.sync();  // Commands apply at the next sync.
    step(TimePoint{}, InOut(scheduler), InOut(world));

    CHECK(held == 1u);
    CHECK(world.store_of<Interceptor>().size() == 0u);
  }

  SECTION("ShouldEngageAgainGivenWeaponsFree") {
    make_launcher(meters(0.0, 0.0, 0.0), InOut(world));
    world.sync();
    REQUIRE(hold_weapons(home, InOut(world)));
    world.sync();
    step(TimePoint{}, InOut(scheduler), InOut(world));
    REQUIRE(world.store_of<Interceptor>().size() == 0u);

    auto freed = free_weapons(home, {}, InOut(world));
    world.sync();
    step(TimePoint{1s}, InOut(scheduler), InOut(world));

    CHECK(freed == 1u);
    CHECK(world.store_of<Interceptor>().size() == 1u);
  }

  SECTION("ShouldHoldOnlyLaunchersInSectorGivenSector") {
    make_launcher(meters(0.0, 0.0, 0.0), InOut(world));
    Entity distant = make_launcher(meters(4000.0, 0.0, 0.0), InOut(world));
    world.sync();

    REQUIRE(hold_weapons(home, InOut(world)) == 1u);
    world.sync();
    step(TimePoint{}, InOut(scheduler), InOut(world));

    REQUIRE(world.store_of<Interceptor>().size() == 1u);
    CHECK(world.parent_of(collect_owners<Interceptor>(world).front()) ==
          distant);
  }

  SECTION("ShouldSkipHeldLaunchersGivenSecondHoldBeforeSync") {
    make_launcher(meters(0.0, 0.0, 0.0), InOut(world));
    world.sync();

    auto first = hold_weapons(home, InOut(world));
    auto second = hold_weapons(home, InOut(world));

    CHECK(first == 1u);
    CHECK(second == 0u);
  }

  SECTION("ShouldDestroyInterceptorsInSectorGivenCommandDestruct") {
    Entity near = make_interceptor(meters(100.0, 0.0, 0.0), drone,
                                   TimePoint{1min}, InOut(world));
    Entity far = make_interceptor(meters(1500.0, 0.0, 0.0), drone,
                                  TimePoint{1min}, InOut(world));
    world.sync();

    auto destroyed = destruct_interceptors(home, InOut(world));
    world.sync();

    CHECK(destroyed == 1u);
    CHECK_FALSE(world.alive(near));
    CHECK(world.alive(far));
  }
}

TEST_CASE("GuideInterceptors") {
  World world;
  build_small_world(Out(world));
  framework::Scheduler<World, SystemList<GuideInterceptors>> scheduler;

  SECTION("ShouldRetargetNearestDroneGivenTargetGone") {
    Entity near = make_drone(meters(300.0, 0.0, 0.0), Entity{}, InOut(world));
    make_drone(meters(600.0, 0.0, 0.0), Entity{}, InOut(world));
    Entity interceptor = make_interceptor(meters(0.0, 0.0, 0.0), Entity{},
                                          TimePoint{1min}, InOut(world));
    world.sync();

    step(TimePoint{}, InOut(scheduler), InOut(world));

    CHECK(world.store_of<Target>().component_of(interceptor).entity == near);
  }

  SECTION("ShouldSelfDestructGivenNoDroneInSeekerRange") {
    make_drone(meters(5000.0, 0.0, 0.0), Entity{}, InOut(world));
    Entity interceptor = make_interceptor(meters(0.0, 0.0, 0.0), Entity{},
                                          TimePoint{1min}, InOut(world));
    world.sync();

    step(TimePoint{}, InOut(scheduler), InOut(world));

    CHECK_FALSE(world.alive(interceptor));
  }

  SECTION("ShouldSelfDestructGivenFlightTimeUp") {
    Entity drone = make_drone(meters(300.0, 0.0, 0.0), Entity{}, InOut(world));
    Entity interceptor = make_interceptor(meters(0.0, 0.0, 0.0), drone,
                                          TimePoint{5s}, InOut(world));
    world.sync();

    step(TimePoint{5s}, InOut(scheduler), InOut(world));

    CHECK_FALSE(world.alive(interceptor));
  }
}

TEST_CASE("Blasts") {
  World world;
  build_small_world(Out(world));
  framework::Scheduler<World, Blasts> scheduler;

  SECTION("ShouldDestroyDroneAndInterceptorGivenFuseDistance") {
    Entity drone = make_drone(meters(10.0, 0.0, 0.0), Entity{}, InOut(world));
    Entity interceptor = make_interceptor(meters(0.0, 0.0, 0.0), drone,
                                          TimePoint{1min}, InOut(world));
    world.sync();

    step(TimePoint{}, InOut(scheduler), InOut(world));

    CHECK_FALSE(world.alive(interceptor));
    CHECK_FALSE(world.alive(drone));
    CHECK(world.store_of<Blast>().size() == 0u);  // Expired within the step.
  }

  SECTION("ShouldDamageAssetOnceGivenDroneDetonatingAtIt") {
    Entity asset = *world.create<archetype::Asset>()
                        .with(Kinematics{})
                        .with(Health{.points = 30.0})
                        .with(Asset{})
                        .build();
    Entity drone = make_drone(meters(20.0, 0.0, 0.0), asset, InOut(world));
    world.sync();

    step(TimePoint{}, InOut(scheduler), InOut(world));

    CHECK_FALSE(world.alive(drone));
    CHECK(world.store_of<Health>().component_of(asset).points == 20.0);
  }
}

}  // namespace simon::defense
