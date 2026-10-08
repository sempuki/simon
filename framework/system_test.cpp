// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "framework/system.hpp"

#include <chrono>
#include <expected>
#include <stdexcept>
#include <string>
#include <vector>

#include "base/testing.hpp"
#include "core/argument.hpp"
#include "framework/test_world.hpp"

namespace simon::framework {

using testing::Body;
using testing::Health;
using testing::Position;
using testing::TestWorld;
using testing::Velocity;

namespace {

const Step STEP{.time = TimePoint{}, .dt = std::chrono::milliseconds{500}};

// Records which entities it ran for, and whether each had a Velocity.
struct Record final           //
    : System<const Position,  //
             const Velocity> {
  using SystemWorld = ProjectedWorld<Record, TestWorld>;

  auto operator()(SystemWorld&, Entity entity,  //
                  const Position&,              //
                  const Velocity* velocity) -> void {
    seen.push_back({entity, velocity != nullptr});
  }
  std::vector<std::pair<Entity, bool>> seen;
};

struct Integrate final  //
    : System<Position,  //
             const Velocity> {
  using SystemWorld = ProjectedWorld<Integrate, TestWorld>;

  auto operator()(SystemWorld&, Entity,      //
                  Position& position,        //
                  const Velocity* velocity,  //
                  Step step) -> void {
    if (!velocity) return;
    position.x += velocity->x * std::chrono::duration<double>(step.dt).count();
  }
};

struct Chase final      //
    : System<Velocity,  //
             const Health> {
  using SystemWorld = ProjectedWorld<Chase, TestWorld>;
  using AllowComponentList = TypeList<Position>;
  auto operator()(SystemWorld& world, Entity self,  //
                  Velocity& velocity,               //
                  const Health*) -> void {
    const Position* mine = world.maybe_component_of<Position>(self);
    const Position* other = world.maybe_component_of<Position>(target);
    velocity.x = (mine && other) ? other->x - mine->x : 0.0;
  }
  Entity target;
};

// Records which entities it ran for: every Position but those with a
// Velocity.
struct RecordStill final  //
    : System<const Position> {
  using SystemWorld = ProjectedWorld<RecordStill, TestWorld>;
  using ExcludeComponentList = TypeList<Velocity>;
  auto operator()(SystemWorld&, Entity entity, const Position&) -> void {
    seen.push_back(entity);
  }
  std::vector<Entity> seen;
};

// Destroys every entity whose health is gone.
struct Cull final  //
    : System<const Health> {
  using SystemWorld = ProjectedWorld<Cull, TestWorld>;

  auto operator()(SystemWorld& world, Entity self, const Health& health)
      -> void {
    if (health.points <= 0.0) {
      REQUIRE(world.destroy(self).build());
    }
  }
};

// Spawns a child under every launcher, from inside a system.
struct Spawn final  //
    : System<const Position> {
  using SystemWorld = ProjectedWorld<Spawn, TestWorld>;

  auto operator()(SystemWorld& world, Entity self, const Position& position)
      -> void {
    REQUIRE(world.create<testing::Interceptor>()
                .under(self)
                .with(Position{position.x})
                .with(Velocity{})
                .build());
  }
};

// Counts, for each entity, whether it has a Velocity and whether that Velocity
// is its own. Every entity in the test has equal Position and Velocity.
struct CheckSiblings final    //
    : System<const Position,  //
             const Velocity> {
  using SystemWorld = ProjectedWorld<CheckSiblings, TestWorld>;

  auto operator()(SystemWorld&, Entity,      //
                  const Position& position,  //
                  const Velocity* velocity) -> void {
    if (!velocity) {
      ++without;
      return;
    }
    ++with;
    matched += velocity->x == position.x;
  }
  int with = 0;
  int without = 0;
  int matched = 0;
};

// Skips its loop when told to, and counts what ran.
struct Skippable final  //
    : System<const Health> {
  using SystemWorld = ProjectedWorld<Skippable, TestWorld>;

  auto prepare(SystemWorld&) -> bool { return run; }
  auto operator()(SystemWorld&, Entity, const Health&) -> void { ++called; }
  auto resolve(SystemWorld&) -> void { resolved = true; }
  int called = 0;
  bool run = true;
  bool resolved = false;
};

// Counts entities with health; runs after Cull, so it sees Cull's commands.
struct Count final  //
    : System<const Health> {
  using SystemWorld = ProjectedWorld<Count, TestWorld>;
  using SequenceAfterSystemList = SystemList<Cull>;
  auto prepare(SystemWorld&) -> void { count = 0; }
  auto operator()(SystemWorld&, Entity, const Health&) -> void { ++count; }
  auto resolve(SystemWorld&) -> void { resolved = true; }
  int count = 0;
  bool resolved = false;
};

// Destroys every body near the origin through a query form, once per step,
// and skips the per-entity loop.
struct ClearOrigin final  //
    : System<const Health> {
  using SystemWorld = ProjectedWorld<ClearOrigin, TestWorld>;
  using AllowComponentList = TypeList<Position>;
  auto prepare(SystemWorld& world) -> bool {
    destroyed = world.destroy().each<Body>().within(Position{0.0}, 2.0).build();
    return false;
  }
  auto operator()(SystemWorld&, Entity, const Health&) -> void {}
  std::expected<std::size_t, Status> destroyed;
};

// Records when it runs and the step it gets, and skips its loop.
struct Tally final  //
    : System<const Health> {
  using SystemWorld = ProjectedWorld<Tally, TestWorld>;
  auto prepare(SystemWorld&, Step step) -> bool {
    runs.push_back(step);
    return false;
  }
  auto operator()(SystemWorld&, Entity, const Health&) -> void {}
  std::vector<Step> runs;
};

// Tally at a declared period.
struct Paced final  //
    : System<const Health> {
  using SystemWorld = ProjectedWorld<Paced, TestWorld>;
  static constexpr Duration PERIOD = std::chrono::milliseconds{200};
  auto prepare(SystemWorld&, Step step) -> bool {
    runs.push_back(step);
    return false;
  }
  auto operator()(SystemWorld&, Entity, const Health&) -> void {}
  std::vector<Step> runs;
};

auto times_of(const std::vector<Step>& runs) -> std::vector<TimePoint> {
  std::vector<TimePoint> times;
  for (const Step& run : runs) {
    times.push_back(run.time);
  }
  return times;
}

auto steps_of(Duration dt, int count) -> std::vector<Step> {
  std::vector<Step> steps;
  for (int i = 0; i < count; ++i) {
    steps.push_back(Step{.time = TimePoint{} + i * dt, .dt = dt});
  }
  return steps;
}

}  // namespace

TEST_CASE("Period") {
  using std::chrono::milliseconds;
  TestWorld world;
  testing::build_small_world(Out(world));

  SECTION("ShouldRunOnlyInStepsHoldingABoundaryGivenPeriod") {
    // Preconditions.
    Scheduler<TestWorld, SystemList<Tally>> scheduler;
    scheduler.set_period<Tally>(milliseconds{100});

    // Under Test.
    for (const Step& step : steps_of(milliseconds{50}, 5)) {
      scheduler.step(step, InOut(world));
    }

    // Postconditions.
    const auto& runs = scheduler.system<Tally>().runs;
    CHECK(times_of(runs) ==
          std::vector<TimePoint>{TimePoint{}, TimePoint{milliseconds{100}},
                                 TimePoint{milliseconds{200}}});
    // Its period on the first run, the time since its last after.
    for (const Step& run : runs) {
      CHECK(run.dt == milliseconds{100});
    }
  }

  SECTION("ShouldStartAtPhaseGivenPhase") {
    // Preconditions.
    Scheduler<TestWorld, SystemList<Tally>> scheduler;
    scheduler.set_period<Tally>(milliseconds{100}, milliseconds{50});

    // Under Test.
    for (const Step& step : steps_of(milliseconds{50}, 5)) {
      scheduler.step(step, InOut(world));
    }

    // Postconditions.
    CHECK(times_of(scheduler.system<Tally>().runs) ==
          std::vector<TimePoint>{TimePoint{milliseconds{50}},
                                 TimePoint{milliseconds{150}}});
  }

  SECTION("ShouldRunOnceWithWholeSpanGivenStepPastSeveralBoundaries") {
    // Preconditions.
    Scheduler<TestWorld, SystemList<Tally>> scheduler;
    scheduler.set_period<Tally>(milliseconds{100});

    // Under Test.
    scheduler.step(Step{.time = TimePoint{}, .dt = milliseconds{50}},
                   InOut(world));
    scheduler.step(
        Step{.time = TimePoint{milliseconds{50}}, .dt = milliseconds{300}},
        InOut(world));

    // Postconditions.
    const auto& runs = scheduler.system<Tally>().runs;
    REQUIRE(runs.size() == 2u);
    CHECK(runs[1].dt == milliseconds{50});
  }

  SECTION("ShouldUseDeclaredPeriodUntilClearedGivenPeriodOnType") {
    // Preconditions.
    Scheduler<TestWorld, SystemList<Paced>> scheduler;

    // Postconditions.
    CHECK(scheduler.period_of<Paced>() == milliseconds{200});
    CHECK(Scheduler<TestWorld, SystemList<Paced>>::describe().contains(
        "period: 200000000ns"));

    // Under Test.
    scheduler.clear_period<Paced>();
    for (const Step& step : steps_of(milliseconds{50}, 3)) {
      scheduler.step(step, InOut(world));
    }

    // Postconditions.
    CHECK(scheduler.system<Paced>().runs.size() == 3u);
  }

  SECTION("ShouldTellTimelineNextBoundaryGivenAttached") {
    // Preconditions.
    Timeline timeline;
    Scheduler<TestWorld, SystemList<Tally, Paced>> scheduler;

    // Under Test.
    scheduler.attach(Depend(timeline));

    // Postconditions.
    // Tally runs every step, so the simulation always has work.
    CHECK(timeline.continuous());

    // Under Test.
    scheduler.set_period<Tally>(milliseconds{100}, milliseconds{30});

    // Postconditions.
    CHECK_FALSE(timeline.continuous());
    // Boundaries count from time zero, so Paced is due there already.
    CHECK(timeline.earliest() == TimePoint{});

    // Under Test.
    scheduler.step(Step{.time = TimePoint{}, .dt = milliseconds{30}},
                   InOut(world));

    // Postconditions.
    // Paced ran at 0 and is next due at 200 ms; Tally first at 30 ms.
    CHECK(timeline.earliest() == TimePoint{milliseconds{30}});

    // Under Test.
    scheduler.clear_period<Tally>();

    // Postconditions.
    CHECK(timeline.continuous());
    CHECK(timeline.earliest() == TimePoint{milliseconds{200}});
  }
}

TEST_CASE("BytesPerEntity") {
  SECTION("ShouldCountOwnerAndNamedComponentsGivenSystem") {
    // Postconditions.
    // Integrate names Position and const Velocity.
    STATIC_CHECK(bytes_per_entity_v<Integrate> ==
                 sizeof(Entity) + sizeof(Position) + sizeof(Velocity));
    // Cull names only const Health.
    STATIC_CHECK(bytes_per_entity_v<Cull> == sizeof(Entity) + sizeof(Health));
  }
}

TEST_CASE("System") {
  TestWorld world;
  testing::build_small_world(Out(world));

  SECTION("ShouldRunForDriverWithOptionalPointerGivenMixedComponents") {
    // Preconditions.
    // x holds (a), y holds (b), z holds (a, b).
    Entity x = *world.create<Body>().with(Position{}).build();
    [[maybe_unused]] Entity y = *world.create<Body>().with(Velocity{}).build();
    Entity z = *world.create<Body>().with(Position{}).with(Velocity{}).build();
    world.sync();
    Scheduler<TestWorld, SystemList<Record>> scheduler;

    // Under Test.
    scheduler.step(STEP, InOut(world));

    // Postconditions.
    auto& seen = scheduler.system<Record>().seen;
    REQUIRE(seen.size() == 2u);
    CHECK(seen[0] == std::pair{x, false});
    CHECK(seen[1] == std::pair{z, true});
  }

  SECTION("ShouldSkipOwnersOfExcludedComponentGivenEveryArchetype") {
    // Preconditions.
    // A launcher cannot have a Velocity, an interceptor always has one, and a
    // body may.
    Entity launcher =
        *world.create<testing::Launcher>().with(Position{}).build();
    auto _ = *world.create<testing::Interceptor>()
                  .with(Position{})
                  .with(Velocity{})
                  .build();
    Entity still = *world.create<Body>().with(Position{}).build();
    auto _ = *world.create<Body>().with(Position{}).with(Velocity{}).build();
    world.sync();
    Scheduler<TestWorld, SystemList<RecordStill>> scheduler;

    // Under Test.
    scheduler.step(STEP, InOut(world));

    // Postconditions.
    // The launcher's segment, then the segment of archetypes that only allow
    // Position.
    CHECK(scheduler.system<RecordStill>().seen ==
          std::vector<Entity>{launcher, still});
  }

  SECTION("ShouldSkipEntityGivenExcludedComponentAttachedLater") {
    // Preconditions.
    Entity body = *world.create<Body>().with(Position{}).build();
    world.sync();
    Scheduler<TestWorld, SystemList<RecordStill>> scheduler;

    // Under Test.
    scheduler.step(STEP, InOut(world));
    REQUIRE(world.change(body).attach(Velocity{}).build());
    world.sync();
    scheduler.step(STEP, InOut(world));

    // Postconditions.
    CHECK(scheduler.system<RecordStill>().seen == std::vector<Entity>{body});
  }

  SECTION("ShouldWriteDrivingComponentGivenStep") {
    // Preconditions.
    Entity entity =
        *world.create<Body>().with(Position{1.0}).with(Velocity{2.0}).build();
    world.sync();
    Scheduler<TestWorld, SystemList<Integrate>> scheduler;

    // Under Test.
    scheduler.step(STEP, InOut(world));

    // Postconditions.
    CHECK(world.store_of<Position>().component_of(entity).x == 2.0);
  }

  SECTION("ShouldQueryMovedPositionsGivenSystemWroteSpatialComponent") {
    // Preconditions.
    Entity entity =
        *world.create<Body>().with(Position{1.0}).with(Velocity{2.0}).build();
    world.sync();
    auto found_near = [&](double x) {
      std::vector<Entity> found;
      world.within(Position{x}, 0.5,
                   [&](Entity e, const Position&) { found.push_back(e); });
      return found;
    };
    REQUIRE(found_near(1.0) == std::vector<Entity>{entity});
    Scheduler<TestWorld, SystemList<Integrate>> scheduler;

    // Under Test.
    scheduler.step(STEP, InOut(world));

    // Postconditions.
    CHECK(found_near(1.0).empty());
    CHECK(found_near(2.0) == std::vector<Entity>{entity});
  }

  SECTION("ShouldPassEachEntitysOwnSiblingGivenArchetypesAndChurn") {
    // Preconditions.
    // Interceptors require both components, so their Velocity is found at the
    // same slot. Launchers cannot have one, and bodies may.
    std::vector<Entity> interceptors;
    for (int i = 0; i < 4; ++i) {
      interceptors.push_back(*world.create<testing::Interceptor>()
                                  .with(Position{static_cast<double>(i)})
                                  .with(Velocity{static_cast<double>(i)})
                                  .build());
    }
    REQUIRE(world.create<testing::Launcher>().with(Position{20.0}).build());
    REQUIRE(
        world.create<Body>().with(Position{10.0}).with(Velocity{10.0}).build());
    REQUIRE(world.create<Body>().with(Position{11.0}).build());
    world.sync();
    REQUIRE(world.destroy(interceptors[1]).build());
    REQUIRE(world.create<testing::Interceptor>()
                .with(Position{4.0})
                .with(Velocity{4.0})
                .build());
    world.sync();
    Scheduler<TestWorld, SystemList<CheckSiblings>> scheduler;

    // Under Test.
    scheduler.step(STEP, InOut(world));

    // Postconditions.
    const CheckSiblings& check = scheduler.system<CheckSiblings>();
    CHECK(check.with == 5);
    CHECK(check.matched == 5);
    CHECK(check.without == 2);
  }

  SECTION("ShouldSkipLoopAndResolveGivenPrepareReturnsFalse") {
    // Preconditions.
    REQUIRE(world.create<Body>().with(Health{1.0}).build());
    world.sync();
    Scheduler<TestWorld, SystemList<Skippable>> scheduler;

    // Under Test.
    scheduler.system<Skippable>().run = false;
    scheduler.step(STEP, InOut(world));

    // Postconditions.
    CHECK(scheduler.system<Skippable>().called == 0);
    CHECK_FALSE(scheduler.system<Skippable>().resolved);

    // Under Test.
    scheduler.system<Skippable>().run = true;
    scheduler.step(STEP, InOut(world));

    // Postconditions.
    CHECK(scheduler.system<Skippable>().called == 1);
    CHECK(scheduler.system<Skippable>().resolved);
  }

  SECTION("ShouldReadOtherEntitiesGivenDeclaredLookup") {
    // Preconditions.
    Entity target = *world.create<Body>().with(Position{10.0}).build();
    Entity chaser =
        *world.create<Body>().with(Position{4.0}).with(Velocity{}).build();
    world.sync();
    Scheduler<TestWorld, SystemList<Chase>> scheduler;
    scheduler.system<Chase>().target = target;

    // Under Test.
    scheduler.step(STEP, InOut(world));

    // Postconditions.
    CHECK(world.store_of<Velocity>().component_of(chaser).x == 6.0);
  }

  SECTION("ShouldSeeEarlierSystemsCommandsGivenSyncBetweenSystems") {
    // Preconditions.
    REQUIRE(world.create<Body>().with(Health{0.0}).build());
    REQUIRE(world.create<Body>().with(Health{1.0}).build());
    world.sync();
    Scheduler<TestWorld, SystemList<Cull, Count>> scheduler;

    // Under Test.
    scheduler.step(STEP, InOut(world));

    // Postconditions.
    CHECK(scheduler.system<Count>().count == 1);
    CHECK(scheduler.system<Count>().resolved);
    CHECK(world.size() == 1u);
  }

  SECTION("ShouldDestroySelectedGivenQueryFormInPrepare") {
    // Preconditions.
    REQUIRE(world.create<Body>().with(Position{1.0}).build());
    REQUIRE(world.create<Body>().with(Position{1.5}).with(Health{}).build());
    Entity distant = *world.create<Body>().with(Position{9.0}).build();
    world.sync();
    Scheduler<TestWorld, SystemList<ClearOrigin>> scheduler;

    // Under Test.
    scheduler.step(STEP, InOut(world));

    // Postconditions.
    CHECK(scheduler.system<ClearOrigin>().destroyed == 2u);
    CHECK(world.size() == 1u);
    CHECK(world.alive(distant));
  }

  SECTION("ShouldThrowGivenComponentOfMissingComponent") {
    // Preconditions.
    Entity bare = *world.create<Body>().build();
    Entity placed = *world.create<Body>().with(Position{2.0}).build();
    world.sync();
    ProjectedWorld<Chase, TestWorld> access{Depend(world)};

    // Postconditions.
    CHECK(access.component_of<Position>(placed).x == 2.0);
    CHECK(access.maybe_component_of<Position>(bare) == nullptr);
    CHECK_THROWS_AS(access.component_of<Position>(bare), std::logic_error);
  }

  SECTION("ShouldRunLambdaWithStateGivenCallableSystem") {
    // Preconditions.
    REQUIRE(world.create<Body>().with(Health{1.0}).build());
    REQUIRE(world.create<Body>().with(Health{2.0}).build());
    world.sync();
    auto count = system<const Health>(
        [seen = 0](auto&, Entity, const Health&) mutable { return ++seen; });
    Scheduler<TestWorld, SystemList<decltype(count)>> scheduler{
        SystemList{count}};

    // Under Test.
    scheduler.step(STEP, InOut(world));
    scheduler.step(STEP, InOut(world));

    // Postconditions.
    // The capture persists across steps: two entities, two steps.
    auto& lambda = scheduler.system<decltype(count)>().callable();
    int unused_world = 0;
    CHECK(lambda(unused_world, Entity{}, Health{}) == 5);
  }

  SECTION("ShouldReadOtherEntitiesGivenLambdaWithLookups") {
    // Preconditions.
    Entity target = *world.create<Body>().with(Position{10.0}).build();
    Entity chaser =
        *world.create<Body>().with(Position{4.0}).with(Velocity{}).build();
    world.sync();
    auto chase = system<Velocity, const Position>(
        TypeList<Position>{}, [target](auto& world, Entity, Velocity& velocity,
                                       const Position* mine) {
          const Position* other =
              world.template maybe_component_of<Position>(target);
          velocity.x = (mine && other) ? other->x - mine->x : 0.0;
        });
    Scheduler<TestWorld, SystemList<decltype(chase)>> scheduler{
        SystemList{chase}};

    // Under Test.
    scheduler.step(STEP, InOut(world));

    // Postconditions.
    CHECK(world.store_of<Velocity>().component_of(chaser).x == 6.0);
  }

  SECTION("ShouldRunInOrderGivenMixedStructAndLambdaSystems") {
    // Preconditions.
    Entity entity =
        *world.create<Body>().with(Position{0.0}).with(Velocity{1.0}).build();
    world.sync();
    auto double_velocity = system<Velocity>(
        [](auto&, Entity, Velocity& velocity) { velocity.x *= 2.0; });
    using Schedule =
        SystemList<decltype(double_velocity), SystemList<Integrate>>;
    Scheduler<TestWorld, Schedule> scheduler{
        Schedule{double_velocity, SystemList<Integrate>{}}};

    // Under Test.
    scheduler.step(STEP, InOut(world));

    // Postconditions.
    CHECK(world.store_of<Velocity>().component_of(entity).x == 2.0);
    CHECK(world.store_of<Position>().component_of(entity).x ==
          1.0);  // 2.0 * 0.5
  }

  SECTION("ShouldCreateChildrenGivenSystemThatSpawns") {
    // Preconditions.
    Entity launcher =
        *world.create<testing::Launcher>().with(Position{3.0}).build();
    world.sync();
    Scheduler<TestWorld, SystemList<Spawn>> scheduler;

    // Under Test.
    scheduler.step(STEP, InOut(world));

    // Postconditions.
    REQUIRE(world.store_of<Position>().size() == 2u);
    Entity child;
    world.store_of<Position>().for_each([&](Entity owner, const Position&) {
      child = owner != launcher ? owner : child;
    });
    CHECK(world.parent_of(child) == launcher);
    CHECK(world.store_of<Position>().component_of(child).x == 3.0);
  }

  SECTION("ShouldFlattenInOrderGivenNestedSchedules") {
    // Preconditions.
    using Nested = SystemList<Integrate, SystemList<Cull, Count>>;

    // Postconditions.
    static_assert(std::is_same_v<flattened_list_t<Nested>,
                                 TypeList<Integrate, Cull, Count>>);
  }

  SECTION("ShouldBeInvalidOnlyGivenSystemBeforeItsAfter") {
    // Postconditions.
    static_assert(is_valid_schedule_v<SystemList<Cull, Count>>);
    static_assert(!is_valid_schedule_v<SystemList<Count, Cull>>);
    static_assert(
        is_valid_schedule_v<SystemList<Count>>);  // Cull absent: fine.
    static_assert(!is_valid_schedule_v<SystemList<Cull, Cull, Count>>);
  }

  SECTION("ShouldNameSystemsBySchedulePositionGivenNestedSchedule") {
    // Preconditions.
    using Scheduled =
        Scheduler<TestWorld, SystemList<Integrate, SystemList<Cull, Count>>>;

    // Postconditions.
    static_assert(Scheduled::name_of<Integrate>() == Name{Kind::SYSTEM, 0});
    static_assert(Scheduled::name_of<Count>() == Name{Kind::SYSTEM, 2});
    CHECK(Scheduled::describe(1).contains("/world/1/system/2 "));
  }

  SECTION("ShouldDeriveReadsAndWritesGivenSystemDeclaration") {
    // Under Test.
    std::string text = Scheduler<TestWorld, SystemList<Integrate>>::describe();
    std::string still =
        Scheduler<TestWorld, SystemList<RecordStill>>::describe();

    // Postconditions.
    static_assert(
        std::is_same_v<write_list_of_t<Integrate>, TypeList<Position>>);
    static_assert(
        std::is_same_v<read_list_of_t<Integrate>, TypeList<Velocity>>);
    static_assert(
        std::is_same_v<allow_component_list_of_t<Chase>, TypeList<Position>>);
    CHECK(text.contains("writes: simon::framework::testing::Position"));
    CHECK(text.contains("reads: simon::framework::testing::Velocity"));
    CHECK(text.contains("excludes: -"));
    CHECK(still.contains("excludes: simon::framework::testing::Velocity"));
  }
}

}  // namespace simon::framework
