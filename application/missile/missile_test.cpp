// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include <chrono>
#include <expected>
#include <vector>

#include "application/missile/simulation.hpp"
#include "base/testing.hpp"
#include "engine/driver.hpp"

namespace simon::missile {

namespace {

using namespace std::chrono_literals;

constexpr Duration DT = 10ms;
// A world holding 16 of each archetype, for testing systems alone.
World small_world() {
  std::expected<World, framework::Status> world =
      World::set_up()
          .numbered(1)
          .holding<archetype::Asset>(16)
          .holding<archetype::Radar>(16)
          .holding<archetype::Launcher>(16)
          .holding<archetype::RedDrone>(16)
          .holding<archetype::Track>(16)
          .holding<archetype::Interceptor>(16)
          .holding<archetype::Blast>(16)
          .build();
  CHECK_POSTCONDITION(world.has_value());
  return *std::move(world);
}

struct Run final {
  Outcome outcome;
  TimePoint end;
  std::uint32_t fired;
  double asset_health;
};

Run run(Scenario scenario) {
  Simulation simulation{scenario};
  engine::BatchDriver driver{lib::Depend(simulation),
                             engine::Timing{.max_step = DT}};
  auto end = driver.run(TimePoint{10min});
  REQUIRE(end);
  const Health* asset = simulation.world().store_of<Health>().try_component_of(
      simulation.asset());
  return Run{.outcome = simulation.outcome(),
             .end = *end,
             .fired = simulation.interceptors_fired(),
             .asset_health = asset ? asset->points : 0.0};
}

template <typename ScheduleType>
void step(framework::Scheduler<World, ScheduleType>& scheduler,
          lib::InOut<World> world, TimePoint time) {
  scheduler.step(world, framework::Step{.time = time, .dt = DT});
}

Entity make_drone(lib::InOut<World> world, Position position,
                  Entity target = Entity{}) {
  return *world->create<archetype::RedDrone>()
              .with(Kinematics{.position = position})
              .with(Control{})
              .with(Health{.points = 1.0})
              .with(Warhead{.fuse = 30.0 * model::meter,
                            .radius = 100.0 * model::meter,
                            .damage = 10.0})
              .with(Target{.entity = target})
              .with(RedDrone{})
              .build();
}

Entity make_radar(lib::InOut<World> world, Position position) {
  return *world->create<archetype::Radar>()
              .with(Kinematics{.position = position})
              .with(Radar{.range = 1000.0 * model::meter,
                          .scan = engine::RateGate{1s}})
              .build();
}

Entity make_launcher(lib::InOut<World> world, Position position) {
  return *world->create<archetype::Launcher>()
              .with(Kinematics{.position = position})
              .with(Launcher{
                  .range = 3000.0 * model::meter, .inventory = 5, .reload = 2s})
              .build();
}

Entity make_track(lib::InOut<World> world, Entity target, Position position) {
  return *world->create<archetype::Track>()
              .with(Track{.target = target})
              .with(Estimate{.position = position})
              .with(Engagement{})
              .build();
}

Entity make_interceptor(lib::InOut<World> world, Position position,
                        Entity target, TimePoint expires_at = TimePoint{1min}) {
  return *world->create<archetype::Interceptor>()
              .with(Kinematics{.position = position})
              .with(Control{})
              .with(InterceptorDesign{}.warhead)
              .with(Target{.entity = target})
              .with(Interceptor{
                  .speed = 150.0 * model::meter_per_second,
                  .agility = 300.0 * model::meter_per_second_squared,
                  .seeker_range = 1000.0 * model::meter,
                  .expires_at = expires_at})
              .build();
}

// The owners of every `ComponentType`, in store order.
template <typename ComponentType>
std::vector<Entity> owners_of(const World& world) {
  std::vector<Entity> owners;
  world.store_of<ComponentType>().for_each(
      [&](Entity owner, const ComponentType&) { owners.push_back(owner); });
  return owners;
}

}  // namespace

TEST_CASE("MissileSimulation") {
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

  SECTION("ShouldAimEachSitesDronesAtItsOwnAssetGivenSeveralSites") {
    Scenario scenario{.drones = 10, .sites = 4};
    std::expected<World, framework::Status> built = world_for(scenario);
    REQUIRE(built.has_value());
    World& world = *built;
    Entity first = build_scenario(lib::InOut(world), scenario);

    std::vector<Entity> assets = owners_of<Asset>(world);
    REQUIRE(assets.size() == 4u);
    CHECK(assets.front() == first);
    CHECK(owners_of<RedDrone>(world).size() == 40u);
    CHECK(owners_of<Radar>(world).size() == 12u);
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

TEST_CASE("DetectDrones") {
  World world = small_world();
  framework::Scheduler<World, SystemList<ScanRadars, DetectDrones>> scheduler;

  SECTION("ShouldCreateOneTrackGivenTwoRadarsSeeingOneDrone") {
    make_radar(lib::InOut(world), model::meters(0.0, 0.0, 0.0));
    make_radar(lib::InOut(world), model::meters(100.0, 0.0, 0.0));
    Entity drone =
        make_drone(lib::InOut(world), model::meters(500.0, 0.0, 0.0));
    world.sync();

    step(scheduler, lib::InOut(world), TimePoint{});

    REQUIRE(world.store_of<Track>().size() == 1u);
    Entity track = owners_of<Track>(world).front();
    CHECK(world.store_of<Track>().component_of(track).target == drone);
    CHECK(world.store_of<Tracked>().component_of(drone).track == track);
  }

  SECTION("ShouldNotTrackGivenDroneOutOfRange") {
    make_radar(lib::InOut(world), model::meters(0.0, 0.0, 0.0));
    make_drone(lib::InOut(world), model::meters(5000.0, 0.0, 0.0));
    world.sync();

    step(scheduler, lib::InOut(world), TimePoint{});

    CHECK(world.store_of<Track>().size() == 0u);
  }

  SECTION("ShouldNotTrackAgainGivenDroneAlreadyTracked") {
    make_radar(lib::InOut(world), model::meters(0.0, 0.0, 0.0));
    make_drone(lib::InOut(world), model::meters(500.0, 0.0, 0.0));
    world.sync();

    step(scheduler, lib::InOut(world), TimePoint{});
    step(scheduler, lib::InOut(world), TimePoint{1s});  // The next scan.

    CHECK(world.store_of<Track>().size() == 1u);
  }
}

TEST_CASE("UpdateTracks") {
  World world = small_world();
  framework::Scheduler<World, SystemList<ScanRadars, UpdateTracks>> scheduler;

  SECTION("ShouldUpdateEstimateGivenScanningRadarCoversTarget") {
    make_radar(lib::InOut(world), model::meters(0.0, 0.0, 0.0));
    Entity drone =
        make_drone(lib::InOut(world), model::meters(500.0, 0.0, 0.0));
    Entity track =
        make_track(lib::InOut(world), drone, model::meters(0.0, 0.0, 0.0));
    world.sync();

    step(scheduler, lib::InOut(world), TimePoint{2s});

    CHECK(world.store_of<Estimate>().component_of(track).position ==
          model::meters(500.0, 0.0, 0.0));
    CHECK(world.store_of<Track>().component_of(track).last_seen ==
          TimePoint{2s});
  }

  SECTION("ShouldKeepEstimateGivenTargetOutOfRange") {
    make_radar(lib::InOut(world), model::meters(0.0, 0.0, 0.0));
    Entity drone =
        make_drone(lib::InOut(world), model::meters(5000.0, 0.0, 0.0));
    Entity track =
        make_track(lib::InOut(world), drone, model::meters(4500.0, 0.0, 0.0));
    world.sync();

    step(scheduler, lib::InOut(world), TimePoint{2s});

    CHECK(world.store_of<Estimate>().component_of(track).position ==
          model::meters(4500.0, 0.0, 0.0));
    CHECK(world.store_of<Track>().component_of(track).last_seen == TimePoint{});
  }
}

TEST_CASE("DropStaleTracks") {
  World world = small_world();
  framework::Scheduler<World, SystemList<DropStaleTracks>> scheduler;

  SECTION("ShouldDropTrackAndUnmarkDroneGivenNotSeenForTimeout") {
    Entity drone = make_drone(lib::InOut(world), model::meters(0.0, 0.0, 0.0));
    Entity track =
        make_track(lib::InOut(world), drone, model::meters(0.0, 0.0, 0.0));
    world.sync();
    REQUIRE(world.change(drone).attach(Tracked{.track = track}).build());
    world.sync();

    step(scheduler, lib::InOut(world), TimePoint{6s});

    CHECK_FALSE(world.alive(track));
    CHECK_FALSE(world.store_of<Tracked>().contains(drone));
  }
}

TEST_CASE("Engaging") {
  World world = small_world();
  framework::Scheduler<World, Engaging> scheduler;

  SECTION("ShouldLaunchOneInterceptorGivenTwoLaunchersProposingOneTrack") {
    make_launcher(lib::InOut(world), model::meters(0.0, 0.0, 0.0));
    make_launcher(lib::InOut(world), model::meters(50.0, 0.0, 0.0));
    Entity drone =
        make_drone(lib::InOut(world), model::meters(2000.0, 0.0, 0.0));
    make_track(lib::InOut(world), drone, model::meters(2000.0, 0.0, 0.0));
    world.sync();

    step(scheduler, lib::InOut(world), TimePoint{});

    REQUIRE(world.store_of<Interceptor>().size() == 1u);
    // The nearer launcher, at 50 m, won the engagement.
    Entity interceptor = owners_of<Interceptor>(world).front();
    Entity launcher = *world.parent_of(interceptor);
    CHECK(world.store_of<Kinematics>().component_of(launcher).position ==
          model::meters(50.0, 0.0, 0.0));
    CHECK(world.store_of<Target>().component_of(interceptor).entity == drone);
  }

  SECTION("ShouldWaitForReloadGivenSecondTrack") {
    Entity launcher =
        make_launcher(lib::InOut(world), model::meters(0.0, 0.0, 0.0));
    for (double x : {1000.0, 2000.0}) {
      Entity drone = make_drone(lib::InOut(world), model::meters(x, 0.0, 0.0));
      make_track(lib::InOut(world), drone, model::meters(x, 0.0, 0.0));
    }
    world.sync();

    step(scheduler, lib::InOut(world), TimePoint{});
    step(scheduler, lib::InOut(world), TimePoint{1s});
    CHECK(world.store_of<Interceptor>().size() == 1u);  // Reloading.

    step(scheduler, lib::InOut(world), TimePoint{2s});
    CHECK(world.store_of<Interceptor>().size() == 2u);
    CHECK(world.store_of<Launcher>().component_of(launcher).inventory == 3u);
  }

  SECTION("ShouldNotEngageGivenTrackOutOfRange") {
    make_launcher(lib::InOut(world), model::meters(0.0, 0.0, 0.0));
    Entity drone =
        make_drone(lib::InOut(world), model::meters(9000.0, 0.0, 0.0));
    make_track(lib::InOut(world), drone, model::meters(9000.0, 0.0, 0.0));
    world.sync();

    step(scheduler, lib::InOut(world), TimePoint{});

    CHECK(world.store_of<Interceptor>().size() == 0u);
  }
}

TEST_CASE("GuideInterceptors") {
  World world = small_world();
  framework::Scheduler<World, SystemList<GuideInterceptors>> scheduler;

  SECTION("ShouldRetargetNearestDroneGivenTargetGone") {
    Entity near = make_drone(lib::InOut(world), model::meters(300.0, 0.0, 0.0));
    make_drone(lib::InOut(world), model::meters(600.0, 0.0, 0.0));
    Entity interceptor = make_interceptor(
        lib::InOut(world), model::meters(0.0, 0.0, 0.0), Entity{});
    world.sync();

    step(scheduler, lib::InOut(world), TimePoint{});

    CHECK(world.store_of<Target>().component_of(interceptor).entity == near);
  }

  SECTION("ShouldSelfDestructGivenNoDroneInSeekerRange") {
    make_drone(lib::InOut(world), model::meters(5000.0, 0.0, 0.0));
    Entity interceptor = make_interceptor(
        lib::InOut(world), model::meters(0.0, 0.0, 0.0), Entity{});
    world.sync();

    step(scheduler, lib::InOut(world), TimePoint{});

    CHECK_FALSE(world.alive(interceptor));
  }

  SECTION("ShouldSelfDestructGivenFlightTimeUp") {
    Entity drone =
        make_drone(lib::InOut(world), model::meters(300.0, 0.0, 0.0));
    Entity interceptor = make_interceptor(
        lib::InOut(world), model::meters(0.0, 0.0, 0.0), drone, TimePoint{5s});
    world.sync();

    step(scheduler, lib::InOut(world), TimePoint{5s});

    CHECK_FALSE(world.alive(interceptor));
  }
}

TEST_CASE("Blasts") {
  World world = small_world();
  framework::Scheduler<World, Blasts> scheduler;

  SECTION("ShouldDestroyDroneAndInterceptorGivenFuseDistance") {
    Entity drone = make_drone(lib::InOut(world), model::meters(10.0, 0.0, 0.0));
    Entity interceptor = make_interceptor(lib::InOut(world),
                                          model::meters(0.0, 0.0, 0.0), drone);
    world.sync();

    step(scheduler, lib::InOut(world), TimePoint{});

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
    Entity drone =
        make_drone(lib::InOut(world), model::meters(20.0, 0.0, 0.0), asset);
    world.sync();

    step(scheduler, lib::InOut(world), TimePoint{});

    CHECK_FALSE(world.alive(drone));
    CHECK(world.store_of<Health>().component_of(asset).points == 20.0);
  }
}

}  // namespace simon::missile
