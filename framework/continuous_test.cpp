// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "framework/continuous.hpp"

#include <chrono>
#include <cmath>
#include <expected>
#include <numbers>
#include <stdexcept>
#include <string>

#include "base/testing.hpp"
#include "catch2/matchers/catch_matchers_floating_point.hpp"
#include "core/argument.hpp"
#include "framework/archetype.hpp"
#include "framework/system.hpp"
#include "framework/world.hpp"

namespace simon::framework {

using Catch::Matchers::WithinAbs;
using namespace std::chrono_literals;

namespace {

// A point on a line, with its velocity: the continuous state, and the world's
// spatial component.
struct Point final {
  using RateComponent = struct PointRate;
  double x = 0.0;
  double v = 0.0;
};

struct PointRate final {
  double dx = 0.0;
  double dv = 0.0;
};

auto operator+(const PointRate& a, const PointRate& b) -> PointRate {
  return {.dx = a.dx + b.dx, .dv = a.dv + b.dv};
}
auto operator*(double weight, const PointRate& rate) -> PointRate {
  return {.dx = weight * rate.dx, .dv = weight * rate.dv};
}
auto advance(const Point& point, const PointRate& rate, Duration dt) -> Point {
  double seconds = std::chrono::duration<double>(dt).count();
  return {.x = point.x + rate.dx * seconds, .v = point.v + rate.dv * seconds};
}

// Only the Spatial concept uses these, so Clang finds them unused.
[[maybe_unused]] auto distance(const Point& a, const Point& b) -> double {
  return std::abs(a.x - b.x);
}
[[maybe_unused]] auto pose(const Point& a) -> Point { return a; }
auto coordinates(const Point& a) -> Coordinates { return {a.x, 0.0, 0.0}; }
auto coordinate_length(const Point&, double length) -> double { return length; }

// Another entity whose position drives this one's rate.
struct Follow final {
  Entity leader;
};

// Counts derivative evaluations.
struct Probe final {
  int evaluations = 0;
};

struct Mass final : Archetype<"mass", Requires<Point, PointRate>> {};
struct Follower final
    : Archetype<"follower", Requires<Point, PointRate, Follow>> {};
// Requires the state but cannot have a rate, so Continuous leaves it to some
// other system.
struct Rigid final : Archetype<"rigid", Requires<Point>> {};
// Allows the state without requiring it, so it may lack a rate.
struct Loose final : Archetype<"loose", Requires<>, Allows<Point, PointRate>> {
};

using TestWorld = World<Point, TypeList<PointRate, Follow, Probe>,
                        TypeList<Mass, Follower, Rigid, Loose>>;

auto build(Out<TestWorld> world) -> void {
  std::expected<void, Status> built = TestWorld::set_up()
                                          .numbered(1)
                                          .holding<Mass>(4)
                                          .holding<Follower>(4)
                                          .holding<Rigid>(4)
                                          .holding<Loose>(4)
                                          .build(world);
  REQUIRE(built);
}

// A harmonic oscillator with unit angular frequency: x'' = -x.
struct Spring final      //
    : System<PointRate,  //
             const Point> {
  using SystemWorld = ProjectedWorld<Spring, TestWorld>;

  auto operator()(SystemWorld&, Entity,  //
                  PointRate& rate,       //
                  const Point* point) const -> void {
    rate = {.dx = point->v, .dv = -point->x};
  }
};

// A leader moves at unit speed; a follower's position grows at the leader's
// position, so it reaches t^2 / 2.
struct Lead final          //
    : System<PointRate,    //
             const Point,  //
             const Follow> {
  using SystemWorld = ProjectedWorld<Lead, TestWorld>;
  using AllowComponentList = TypeList<Point>;
  auto operator()(SystemWorld& world, Entity,  //
                  PointRate& rate,             //
                  const Point*,                //
                  const Follow* follow) const -> void {
    if (!follow) {
      rate = {.dx = 1.0};
      return;
    }
    rate = {.dx = world.component_of<Point>(follow->leader).x};
  }
};

// Sets a rate equal to the stage's time, so x reaches t^2 / 2.
struct Clock final       //
    : System<PointRate,  //
             const Point> {
  using SystemWorld = ProjectedWorld<Clock, TestWorld>;

  auto operator()(SystemWorld&, Entity,  //
                  PointRate& rate,       //
                  const Point*,          //
                  Step step) const -> void {
    rate = {.dx = std::chrono::duration<double>(step.time.time_since_epoch())
                      .count()};
  }
};

// Plans a structural change, which a derivative system may not.
struct Spawn final       //
    : System<PointRate,  //
             const Point> {
  using SystemWorld = ProjectedWorld<Spawn, TestWorld>;

  auto operator()(SystemWorld& world, Entity, PointRate&, const Point*) const
      -> void {
    REQUIRE(world.create<Loose>().build());
  }
};

// Finds its own entity in the spatial index at the stage's position.
struct Locate final      //
    : System<PointRate,  //
             const Point> {
  using SystemWorld = ProjectedWorld<Locate, TestWorld>;
  using AllowComponentList = TypeList<Point>;
  auto operator()(SystemWorld& world, Entity self,  //
                  PointRate& rate,                  //
                  const Point* point) -> void {
    bool found = false;
    world.within(*point, 1e-9, [&](Entity entity, const Point&) {
      found = found || entity == self;
    });
    CHECK(found);
    rate = {.dx = 1.0};
    ++located;
  }
  int located = 0;
};

template <typename MethodType>
using Oscillate = Continuous<MethodType, TypeList<Point>, SystemList<Spring>>;

// The error in the oscillator's state after one period, started at x = 1. At
// the end x is at its peak, where a phase error barely shows, so the velocity
// counts too.
template <typename MethodType>
auto period_error(int steps) -> double {
  TestWorld world;
  build(Out(world));
  REQUIRE(world.create<Mass>().with(Point{.x = 1.0}).with(PointRate{}).build());
  world.sync();

  Scheduler<TestWorld, SystemList<Oscillate<MethodType>>> scheduler;
  auto dt = std::chrono::duration_cast<Duration>(
      std::chrono::duration<double>(2.0 * std::numbers::pi / steps));
  TimePoint time{};
  for (int i = 0; i < steps; ++i) {
    scheduler.step(Step{.time = time, .dt = dt}, InOut(world));
    time += dt;
  }
  // The steps cover the period up to rounding of dt; compare against the
  // exact solution at the time reached.
  double t = std::chrono::duration<double>(time.time_since_epoch()).count();
  Point end;
  world.store_of<Point>().for_each([&](Entity, const Point& p) { end = p; });
  return std::hypot(end.x - std::cos(t), end.v + std::sin(t));
}

}  // namespace

TEST_CASE("Continuous") {
  SECTION("ShouldConvergeAtEachMethodsOrder") {
    // Under Test.
    // Halving the step divides the error by 2^order.
    double euler = period_error<Euler>(1000) / period_error<Euler>(2000);
    double midpoint =
        period_error<Midpoint>(1000) / period_error<Midpoint>(2000);
    double rk4 =
        period_error<RungeKutta4>(100) / period_error<RungeKutta4>(200);

    // Postconditions.
    CHECK_THAT(euler, WithinAbs(2.0, 0.1));
    CHECK_THAT(midpoint, WithinAbs(4.0, 0.2));
    CHECK_THAT(rk4, WithinAbs(16.0, 1.0));
    CHECK(period_error<RungeKutta4>(100) < 1e-6);
  }

  SECTION("ShouldShowEachStageOtherEntitiesTrialState") {
    // Preconditions.
    TestWorld world;
    build(Out(world));
    auto leader = world.create<Mass>().with(Point{}).with(PointRate{}).build();
    REQUIRE(leader);
    REQUIRE(world.create<Follower>()
                .with(Point{})
                .with(PointRate{})
                .with(Follow{.leader = *leader})
                .build());
    world.sync();
    Scheduler<
        TestWorld,
        SystemList<Continuous<RungeKutta4, TypeList<Point>, SystemList<Lead>>>>
        scheduler;
    TimePoint time{};

    // Under Test.
    for (int i = 0; i < 10; ++i) {
      scheduler.step(Step{.time = time, .dt = 100ms}, InOut(world));
      time += 100ms;
    }

    // Postconditions.
    // RK4 integrates the follower's t^2 / 2 exactly, but only if every stage
    // sees the leader where that stage put it.
    CHECK_THAT(world.store_of<Point>().component_of(*leader).x,
               WithinAbs(1.0, 1e-12));
    world.store_of<Follow>().for_each([&](Entity follower, const Follow&) {
      CHECK_THAT(world.store_of<Point>().component_of(follower).x,
                 WithinAbs(0.5, 1e-12));
    });
  }

  SECTION("ShouldGiveDerivativeSystemsTheStagesTime") {
    // Preconditions.
    TestWorld world;
    build(Out(world));
    REQUIRE(world.create<Mass>().with(Point{}).with(PointRate{}).build());
    world.sync();
    Scheduler<
        TestWorld,
        SystemList<Continuous<Midpoint, TypeList<Point>, SystemList<Clock>>>>
        scheduler;
    TimePoint time{};

    // Under Test.
    for (int i = 0; i < 4; ++i) {
      scheduler.step(Step{.time = time, .dt = 250ms}, InOut(world));
      time += 250ms;
    }

    // Postconditions.
    // The midpoint rule integrates a linear rate exactly.
    world.store_of<Point>().for_each([](Entity, const Point& point) {
      CHECK_THAT(point.x, WithinAbs(0.5, 1e-9));
    });
  }

  SECTION("ShouldKeepStateGivenEntityWithoutRate") {
    // Preconditions.
    TestWorld world;
    build(Out(world));
    auto without =
        world.create<Loose>().with(Point{.x = 3.0, .v = 1.0}).build();
    auto with =
        world.create<Loose>().with(Point{.x = 1.0}).with(PointRate{}).build();
    REQUIRE(without);
    REQUIRE(with);
    world.sync();
    Scheduler<TestWorld, SystemList<Oscillate<RungeKutta4>>> scheduler;

    // Under Test.
    scheduler.step(Step{.time = TimePoint{}, .dt = 100ms}, InOut(world));

    // Postconditions.
    CHECK(world.store_of<Point>().component_of(*without).x == 3.0);
    CHECK(world.store_of<Point>().component_of(*with).x < 1.0);
  }

  SECTION("ShouldSkipArchetypeThatCannotHaveRate") {
    // Preconditions.
    TestWorld world;
    build(Out(world));
    auto rigid = world.create<Rigid>().with(Point{.x = 2.0, .v = 1.0}).build();
    REQUIRE(rigid);
    world.sync();
    Scheduler<TestWorld, SystemList<Oscillate<RungeKutta4>>> scheduler;

    // Under Test.
    scheduler.step(Step{.time = TimePoint{}, .dt = 100ms}, InOut(world));

    // Postconditions.
    CHECK(world.store_of<Point>().component_of(*rigid).x == 2.0);
  }

  SECTION("ShouldMatchEulerGivenOneStageMethod") {
    // Preconditions.
    TestWorld world;
    build(Out(world));
    auto mass =
        world.create<Mass>().with(Point{.x = 1.0}).with(PointRate{}).build();
    REQUIRE(mass);
    world.sync();
    Scheduler<TestWorld, SystemList<Oscillate<Euler>>> scheduler;

    // Under Test.
    scheduler.step(Step{.time = TimePoint{}, .dt = 500ms}, InOut(world));

    // Postconditions.
    const Point& point = world.store_of<Point>().component_of(*mass);
    CHECK(point.x == 1.0);
    CHECK(point.v == -0.5);
  }

  SECTION("ShouldFailContractGivenDerivativeSystemThatPlansChanges") {
    // Preconditions.
    TestWorld world;
    build(Out(world));
    REQUIRE(world.create<Mass>().with(Point{}).with(PointRate{}).build());
    world.sync();
    Scheduler<TestWorld,
              SystemList<Continuous<Euler, TypeList<Point>, SystemList<Spawn>>>>
        scheduler;

    // Postconditions.
    CHECK_THROWS_AS(
        scheduler.step(Step{.time = TimePoint{}, .dt = 100ms}, InOut(world)),
        std::logic_error);
  }

  SECTION("ShouldQuerySpaceAtStagesState") {
    // Preconditions.
    TestWorld world;
    build(Out(world));
    REQUIRE(world.create<Mass>().with(Point{}).with(PointRate{}).build());
    world.sync();
    using Dynamics =
        Continuous<RungeKutta4, TypeList<Point>, SystemList<Locate>>;
    Scheduler<TestWorld, SystemList<Dynamics>> scheduler;

    // Under Test.
    scheduler.step(Step{.time = TimePoint{}, .dt = 100ms}, InOut(world));

    // Postconditions.
    CHECK(scheduler.system<Dynamics>().system<Locate>().located == 4);
  }

  SECTION("ShouldDescribeWhatItWritesAndReads") {
    // Preconditions.
    using Dynamics = Continuous<RungeKutta4, TypeList<Point>, SystemList<Lead>>;

    // Under Test.
    std::string text = Scheduler<TestWorld, SystemList<Dynamics>>::describe();

    // Postconditions.
    static_assert(
        std::is_same_v<write_list_of_t<Dynamics>, TypeList<Point, PointRate>>);
    static_assert(std::is_same_v<read_list_of_t<Dynamics>, TypeList<Follow>>);
    static_assert(
        std::is_same_v<allow_component_list_of_t<Dynamics>, TypeList<Point>>);
    CHECK(text.find("writes: ") != std::string::npos);
  }
}

}  // namespace simon::framework
