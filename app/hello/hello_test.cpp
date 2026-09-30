// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "app/hello/hello.hpp"

#include <chrono>
#include <string>
#include <vector>

#include "base/testing.hpp"

namespace simon::hello {

namespace {
const core::WorldConfiguration CONFIGURATION{.number = 1, .entities = 8, .components = 8};
const core::Duration DT{0.01};
}  // namespace

TEST_CASE("Hello") {
  World world{CONFIGURATION};
  Scheduler scheduler;
  Balls balls = build_balls(lib::InOut(world));

  SECTION("ShouldFindBallsByAliasGivenScenario") {
    CHECK(world.find_alias("red") == std::vector{world.name_of(balls.red)});
    CHECK(world.find_alias("blue") == std::vector{world.name_of(balls.blue)});
    CHECK(world.aliases_of(world.archetype_of(balls.red)) == std::vector<std::string>{"ball"});
  }

  SECTION("ShouldAccelerateByThrustGivenNoDragAndNoWind") {
    REQUIRE(world.change(balls.red).detach<Drag>().build());
    world.sync();

    scheduler.step(lib::InOut(world), core::Step{.time = {}, .dt = DT});

    const Kinematics& red = world.store<Kinematics>().get(balls.red);
    CHECK(red.velocity.numerical_value_in(model::metre_per_second)
              .is_approximately(model::Vector3{10.0, -10.0 + 9.8 * 0.01, 0.0}));
  }

  SECTION("ShouldNotCollideGivenBallsFarApart") {
    scheduler.step(lib::InOut(world), core::Step{.time = {}, .dt = DT});
    CHECK_FALSE(any_collision(world));
  }

  SECTION("ShouldCollideBothWaysGivenRunUntilContact") {
    core::TimePoint time{};
    int steps = 0;
    while (!any_collision(world) && steps < 100'000) {
      scheduler.step(lib::InOut(world), core::Step{.time = time, .dt = DT});
      time += DT;
      ++steps;
    }

    REQUIRE(any_collision(world));
    const Collision& red = world.store<Collision>().get(balls.red);
    const Collision& blue = world.store<Collision>().get(balls.blue);
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

}  // namespace simon::hello
