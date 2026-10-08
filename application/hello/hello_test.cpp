// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "application/hello/hello.hpp"

#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include "base/testing.hpp"
#include "core/argument.hpp"
#include "engine/driver.hpp"

namespace simon::hello {

namespace {

using namespace std::chrono_literals;

// Creates a ball of `radius` and `mass` at `x`, `y` moving at `vx`, `vy`, with
// no gravity.
auto create_ball(double x, double y, double vx, double vy, double radius,
                 double mass, InOut<World> world) -> Entity {
  auto ball = world->create<Ball>()
                  .with(Kinematics{.position = meters(x, y, 0.0),
                                   .velocity = meters_per_second(vx, vy, 0.0)})
                  .with(Control{})
                  .with(Body{.radius = radius * meter, .mass = mass * kilogram})
                  .with(Contact{})
                  .build();
  REQUIRE(ball);
  world->sync();
  return *ball;
}

auto velocity_of(const World& world, Entity ball) -> QuantityVector {
  return world.store_of<Kinematics>()
      .component_of(ball)
      .velocity.numerical_value_in(meter_per_second);
}

auto run(Time duration, InOut<Scheduler> scheduler, InOut<World> world)
    -> void {
  TimePoint time{};
  while (time < TimePoint{} + std::chrono::duration_cast<Duration>(
                                  std::chrono::duration<double>(
                                      duration.numerical_value_in(second)))) {
    scheduler->step(Step{.time = time, .dt = STEP}, world);
    time += STEP;
  }
}

}  // namespace

TEST_CASE("Hello") {
  World world;
  REQUIRE(build_world(8, Out(world)));
  Scheduler scheduler;

  SECTION("ShouldSwapVelocitiesGivenEqualBallsHeadOn") {
    // Preconditions.
    Entity left = create_ball(10.0, 10.0, 1.0, 0.0, 0.5, 1.0, InOut(world));
    Entity right = create_ball(11.2, 10.0, -1.0, 0.0, 0.5, 1.0, InOut(world));

    // Under Test.
    run(1.0 * second, InOut(scheduler), InOut(world));

    // Postconditions.
    CHECK(velocity_of(world, left)
              .is_approximately(QuantityVector{-1.0, 0.0, 0.0}, 0.01));
    CHECK(velocity_of(world, right)
              .is_approximately(QuantityVector{1.0, 0.0, 0.0}, 0.01));
  }

  SECTION("ShouldConserveMomentumAndEnergyGivenGlancingBlow") {
    // Preconditions.
    create_ball(10.0, 10.0, 3.0, 0.0, 0.5, 1.0, InOut(world));
    create_ball(13.0, 10.6, 0.0, 0.0, 0.8, 4.1, InOut(world));
    Momentum momentum = compute_momentum(world);
    Energy energy = compute_energy(world, 0.0 * meter_per_second_squared);

    // Under Test.
    run(2.0 * second, InOut(scheduler), InOut(world));

    // Postconditions.
    CHECK(compute_momentum(world)
              .numerical_value_in(kilogram * meter_per_second)
              .is_approximately(
                  momentum.numerical_value_in(kilogram * meter_per_second)));
    Energy after = compute_energy(world, 0.0 * meter_per_second_squared);
    CHECK(std::abs(number_of(after / energy) - 1.0) < 0.01);
  }

  SECTION("ShouldPartSlowerGivenRestitution") {
    // Preconditions.
    scheduler.system<DetectContacts>().springiness.restitution = 0.5;
    Entity left = create_ball(10.0, 10.0, 1.0, 0.0, 0.5, 1.0, InOut(world));
    Entity right = create_ball(11.2, 10.0, -1.0, 0.0, 0.5, 1.0, InOut(world));

    // Under Test.
    run(1.0 * second, InOut(scheduler), InOut(world));

    // Postconditions.
    // Ten steps a contact damp a little more than the dashpot would: the
    // balls part at 0.467 m/s.
    CHECK(velocity_of(world, left)
              .is_approximately(QuantityVector{-0.5, 0.0, 0.0}, 0.1));
    CHECK(velocity_of(world, right)
              .is_approximately(QuantityVector{0.5, 0.0, 0.0}, 0.1));
  }

  SECTION("ShouldBounceOffWallGivenBallHeadingIntoIt") {
    // Preconditions.
    Entity ball = create_ball(78.0, 10.0, 2.0, 1.0, 0.5, 1.0, InOut(world));

    // Under Test.
    run(1.5 * second, InOut(scheduler), InOut(world));

    // Postconditions.
    CHECK(velocity_of(world, ball)
              .is_approximately(QuantityVector{-2.0, 1.0, 0.0}, 0.01));
  }

  SECTION("ShouldListSystemsInOrderGivenSchedule") {
    // Under Test.
    std::string text = Scheduler::describe();

    // Postconditions.
    CHECK(text.find("Integrate") < text.find("DetectContacts"));
    CHECK(text.find("DetectContacts") < text.find("ApplyContacts"));
  }
}

TEST_CASE("HelloSimulation") {
  const engine::Timing timing{.max_step = STEP};
  const Scenario scenario{.balls = 300};

  SECTION("ShouldRefuseGivenMoreBallsThanCells") {
    // Preconditions.
    Simulation simulation{Scenario{.balls = compute_capacity(scenario) + 1}};
    engine::BatchDriver driver{timing, Depend(simulation)};

    // Postconditions.
    CHECK_FALSE(driver.run(TimePoint{1s}));
  }

  SECTION("ShouldKeepBallsInBoxGivenLongRun") {
    // Preconditions.
    Simulation simulation{scenario};
    engine::BatchDriver driver{timing, Depend(simulation)};

    // Under Test.
    REQUIRE(driver.run(TimePoint{20s}));

    // Postconditions.
    double width = scenario.box.width.numerical_value_in(meter);
    double height = scenario.box.height.numerical_value_in(meter);
    std::size_t outside = 0;
    simulation.world().store_of<Kinematics>().for_each(
        [&](Entity, const Kinematics& ball) {
          QuantityVector at = ball.position.numerical_value_in(meter);
          if (at.x() < 0.0 || at.y() < 0.0 || at.x() > width ||
              at.y() > height) {
            ++outside;
          }
        });
    CHECK(outside == 0);
  }

  SECTION("ShouldKeepEnergyGivenElasticRun") {
    // Preconditions.
    Simulation simulation{scenario};
    engine::Driver driver{timing, Depend(simulation)};
    REQUIRE(driver.start());
    Energy start = compute_energy(simulation.world(), scenario.gravity);

    // Under Test.
    REQUIRE(driver.advance_to(TimePoint{20s}));

    // Postconditions.
    Energy end = compute_energy(simulation.world(), scenario.gravity);
    CHECK(std::abs(number_of(end / start) - 1.0) < 0.03);
    REQUIRE(driver.finish());
  }

  SECTION("ShouldLoseEnergyGivenInelasticRun") {
    // Preconditions.
    Scenario inelastic = scenario;
    inelastic.springiness.restitution = 0.5;
    Simulation simulation{inelastic};
    engine::Driver driver{timing, Depend(simulation)};
    REQUIRE(driver.start());
    Energy start = compute_energy(simulation.world(), inelastic.gravity);

    // Under Test.
    REQUIRE(driver.advance_to(TimePoint{20s}));

    // Postconditions.
    CHECK(compute_energy(simulation.world(), inelastic.gravity) < 0.5 * start);
    REQUIRE(driver.finish());
  }

  SECTION("ShouldEndInSameStateGivenTwoRuns") {
    // Preconditions.
    Simulation first{scenario};
    Simulation second{scenario};
    engine::BatchDriver first_driver{timing, Depend(first)};
    engine::BatchDriver second_driver{timing, Depend(second)};

    // Under Test.
    REQUIRE(first_driver.run(TimePoint{5s}));
    REQUIRE(second_driver.run(TimePoint{5s}));

    // Postconditions.
    std::vector<Kinematics> first_balls;
    std::vector<Kinematics> second_balls;
    first.world().store_of<Kinematics>().for_each(
        [&](Entity, const Kinematics& ball) { first_balls.push_back(ball); });
    second.world().store_of<Kinematics>().for_each(
        [&](Entity, const Kinematics& ball) { second_balls.push_back(ball); });
    REQUIRE(first_balls.size() == second_balls.size());
    bool same = true;  // Bit for bit.
    for (std::size_t i = 0; i < first_balls.size(); ++i) {
      same = same && first_balls[i].position == second_balls[i].position &&
             first_balls[i].velocity == second_balls[i].velocity;
    }
    CHECK(same);
  }
}

}  // namespace simon::hello
