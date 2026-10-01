// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "application/hello/hello.hpp"

#include <chrono>
#include <expected>
#include <string>
#include <vector>

#include "base/testing.hpp"
#include "engine/driver.hpp"

namespace simon::hello {

namespace {
const framework::Duration DT = std::chrono::milliseconds{10};
}  // namespace

TEST_CASE("Hello") {
  World world;
  REQUIRE(build_world(8, lib::Out(world)));
  Scheduler scheduler;
  Balls balls = build_balls(lib::InOut(world));

  SECTION("ShouldFindBallsByAliasGivenScenario") {
    CHECK(world.find_name_of(framework::Alias{"red"}) ==
          std::vector{world.name_of(balls.red)});
    CHECK(world.find_name_of(framework::Alias{"blue"}) ==
          std::vector{world.name_of(balls.blue)});
    CHECK(world.aliases_of(world.archetype_of(balls.red)) ==
          std::vector<framework::Alias>{"ball"});
  }

  SECTION("ShouldAccelerateByThrustGivenNoDragAndNoWind") {
    REQUIRE(world.change(balls.red).detach<Drag>().build());
    world.sync();

    scheduler.step(framework::Step{.time = {}, .dt = DT}, lib::InOut(world));

    const Kinematics& red =
        world.store_of<Kinematics>().component_of(balls.red);
    CHECK(
        red.velocity.numerical_value_in(model::meter_per_second)
            .is_approximately(model::Vector3d{10.0, -10.0 + 9.8 * 0.01, 0.0}));
  }

  SECTION("ShouldNotCollideGivenBallsFarApart") {
    scheduler.step(framework::Step{.time = {}, .dt = DT}, lib::InOut(world));
    CHECK_FALSE(any_collision(world));
  }

  SECTION("ShouldCollideBothWaysGivenRunUntilContact") {
    framework::TimePoint time{};
    int steps = 0;
    while (!any_collision(world) && steps < 100'000) {
      scheduler.step(framework::Step{.time = time, .dt = DT},
                     lib::InOut(world));
      time += DT;
      ++steps;
    }

    REQUIRE(any_collision(world));
    const Collision& red = world.store_of<Collision>().component_of(balls.red);
    const Collision& blue =
        world.store_of<Collision>().component_of(balls.blue);
    CHECK(red.hit);
    CHECK(red.other == balls.blue);
    CHECK(blue.hit);
    CHECK(blue.other == balls.red);
  }

  SECTION("ShouldListSystemsInOrderGivenSchedule") {
    std::string text = Scheduler::describe();
    CHECK(text.find("ApplyForces") < text.find("Integrate"));
    CHECK(text.find("Integrate") < text.find("DetectCollisions"));
  }
}

TEST_CASE("HelloSimulation") {
  using namespace std::chrono_literals;
  const engine::Timing timing{.max_step = 10ms};

  SECTION("ShouldStopAtFirstCollisionGivenBatchRun") {
    Simulation simulation;
    engine::BatchDriver driver{timing, lib::Depend(simulation)};

    auto reached = driver.run(framework::TimePoint{60s});

    REQUIRE(reached);
    CHECK(*reached < framework::TimePoint{60s});
    CHECK(any_collision(simulation.world()));
  }

  SECTION("ShouldEndAtSameTimeAndPlaceGivenTwoRuns") {
    Simulation first;
    Simulation second;
    engine::BatchDriver first_driver{timing, lib::Depend(first)};
    engine::BatchDriver second_driver{timing, lib::Depend(second)};

    auto first_end = first_driver.run(framework::TimePoint{60s});
    auto second_end = second_driver.run(framework::TimePoint{60s});

    REQUIRE(first_end);
    REQUIRE(second_end);
    CHECK(*first_end == *second_end);
    const auto& first_red =
        first.world().store_of<Kinematics>().component_of(first.balls().red);
    const auto& second_red =
        second.world().store_of<Kinematics>().component_of(second.balls().red);
    CHECK(first_red.position == second_red.position);  // Bit for bit.
  }
}

}  // namespace simon::hello
