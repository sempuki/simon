// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include <chrono>

#include "application/missile/simulation.hpp"
#include "base/testing.hpp"
#include "engine/driver.hpp"

namespace simon::missile {

namespace {

using namespace std::chrono_literals;

constexpr Duration DT = 10ms;
const framework::WorldConfiguration SMALL{
    .number = 1, .entities = 64, .components = 64};

struct Run final {
  Outcome outcome;
  TimePoint end;
  std::uint32_t fired;
  double asset_health;
};

Run run(Scenario scenario) {
  Simulation simulation{scenario};
  engine::BatchDriver driver{lib::Depend<Simulation>{simulation},
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
              .with(RedDrone{.target = target})
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
              .with(Track{.target = target, .position = position})
              .build();
}

Entity make_interceptor(lib::InOut<World> world, Position position,
                        Entity target, TimePoint expires_at = TimePoint{1min}) {
  return *world->create<archetype::Interceptor>()
              .with(Kinematics{.position = position})
              .with(Control{})
              .with(InterceptorDesign{}.warhead)
              .with(Interceptor{
                  .target = target,
                  .speed = 150.0 * model::meter_per_second,
                  .agility = 300.0 * model::meter_per_second_squared,
                  .seeker_range = 1000.0 * model::meter,
                  .expires_at = expires_at})
              .build();
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
}

TEST_CASE("ScanRadars") {
  World world{SMALL};
  framework::Scheduler<World, SystemList<ScanRadars>> scheduler;

  SECTION("ShouldCreateOneTrackGivenTwoRadarsSeeingOneDrone") {
    make_radar(lib::InOut(world), model::meters(0.0, 0.0, 0.0));
    make_radar(lib::InOut(world), model::meters(100.0, 0.0, 0.0));
    Entity drone =
        make_drone(lib::InOut(world), model::meters(500.0, 0.0, 0.0));
    world.sync();

    step(scheduler, lib::InOut(world), TimePoint{});

    REQUIRE(world.store_of<Track>().size() == 1u);
    CHECK(world.store_of<Track>().data(0).target == drone);
    CHECK(world.store_of<Tracked>().component_of(drone).track ==
          world.store_of<Track>().owner(0));
  }

  SECTION("ShouldNotTrackGivenDroneOutOfRange") {
    make_radar(lib::InOut(world), model::meters(0.0, 0.0, 0.0));
    make_drone(lib::InOut(world), model::meters(5000.0, 0.0, 0.0));
    world.sync();

    step(scheduler, lib::InOut(world), TimePoint{});

    CHECK(world.store_of<Track>().size() == 0u);
  }
}

TEST_CASE("DropStaleTracks") {
  World world{SMALL};
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

TEST_CASE("Engagement") {
  World world{SMALL};
  framework::Scheduler<World, Engagement> scheduler;

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
    Entity interceptor = world.store_of<Interceptor>().owner(0);
    Entity launcher = *world.parent_of(interceptor);
    CHECK(world.store_of<Kinematics>().component_of(launcher).position ==
          model::meters(50.0, 0.0, 0.0));
    CHECK(world.store_of<Interceptor>().data(0).target == drone);
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
  World world{SMALL};
  framework::Scheduler<World, SystemList<GuideInterceptors>> scheduler;

  SECTION("ShouldRetargetNearestDroneGivenTargetGone") {
    Entity near = make_drone(lib::InOut(world), model::meters(300.0, 0.0, 0.0));
    make_drone(lib::InOut(world), model::meters(600.0, 0.0, 0.0));
    Entity interceptor = make_interceptor(
        lib::InOut(world), model::meters(0.0, 0.0, 0.0), Entity{});
    world.sync();

    step(scheduler, lib::InOut(world), TimePoint{});

    CHECK(world.store_of<Interceptor>().component_of(interceptor).target ==
          near);
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
  World world{SMALL};
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
