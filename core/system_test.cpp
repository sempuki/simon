// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "core/system.hpp"

#include <chrono>
#include <string>
#include <vector>

#include "base/testing.hpp"
#include "core/test_world.hpp"

namespace simon::core {

using testing::Body;
using testing::Health;
using testing::Position;
using testing::TestWorld;
using testing::Velocity;

namespace {

const Step STEP{.time = TimePoint{}, .dt = std::chrono::milliseconds{500}};

// Records which entities it ran for, and whether each had a Velocity.
struct Record : System<const Position, const Velocity> {
  void operator()(Entity entity, const Position&, const Velocity* velocity,
                  auto&) {
    seen.push_back({entity, velocity != nullptr});
  }
  std::vector<std::pair<Entity, bool>> seen;
};

struct Integrate : System<Position, const Velocity> {
  void operator()(Entity, Position& position, const Velocity* velocity,
                  auto& context) {
    if (!velocity) return;
    position.x +=
        velocity->x * std::chrono::duration<double>(context.step().dt).count();
  }
};

struct Chase : System<Velocity, const Health> {
  using Lookups = Stores<Position>;
  void operator()(Entity self, Velocity& velocity, const Health*,
                  auto& context) {
    const Position* mine = lookup<Position>(context, self);
    const Position* other = lookup<Position>(context, target);
    velocity.x = (mine && other) ? other->x - mine->x : 0.0;
  }
  Entity target;
};

// Destroys every entity whose health is gone.
struct Cull : System<const Health> {
  void operator()(Entity self, const Health& health, auto& context) {
    if (health.points <= 0.0) {
      REQUIRE(context.destroy(self).build());
    }
  }
};

// Spawns a child under every launcher, from inside a system.
struct Spawn : System<const Position> {
  void operator()(Entity self, const Position& position, auto& context) {
    REQUIRE(create<testing::Interceptor>(lib::InOut(context))
                .under(self)
                .with(Position{position.x})
                .with(Velocity{})
                .build());
  }
};

// Counts entities with health; runs after Cull, so it sees Cull's commands.
struct Count : System<const Health> {
  using After = Systems<Cull>;
  void prepare(auto&) { count = 0; }
  void operator()(Entity, const Health&, auto&) { ++count; }
  void resolve(auto&) { resolved = true; }
  int count = 0;
  bool resolved = false;
};

}  // namespace

TEST_CASE("System") {
  TestWorld world{testing::small_world()};

  SECTION("ShouldRunForDriverWithOptionalPointerGivenMixedComponents") {
    // x holds (a), y holds (b), z holds (a, b).
    Entity x = *world.create<Body>().with(Position{}).build();
    Entity y = *world.create<Body>().with(Velocity{}).build();
    Entity z = *world.create<Body>().with(Position{}).with(Velocity{}).build();
    world.sync();

    Scheduler<TestWorld, Systems<Record>> scheduler;
    scheduler.step(lib::InOut(world), STEP);

    auto& seen = scheduler.system<Record>().seen;
    REQUIRE(seen.size() == 2u);
    CHECK(seen[0] == std::pair{x, false});
    CHECK(seen[1] == std::pair{z, true});
    (void)y;
  }

  SECTION("ShouldWriteDrivingComponentGivenStep") {
    Entity entity =
        *world.create<Body>().with(Position{1.0}).with(Velocity{2.0}).build();
    world.sync();

    Scheduler<TestWorld, Systems<Integrate>> scheduler;
    scheduler.step(lib::InOut(world), STEP);

    CHECK(world.store<Position>().get(entity).x == 2.0);
  }

  SECTION("ShouldReadOtherEntitiesGivenDeclaredLookup") {
    Entity target = *world.create<Body>().with(Position{10.0}).build();
    Entity chaser =
        *world.create<Body>().with(Position{4.0}).with(Velocity{}).build();
    world.sync();

    Scheduler<TestWorld, Systems<Chase>> scheduler;
    scheduler.system<Chase>().target = target;
    scheduler.step(lib::InOut(world), STEP);

    CHECK(world.store<Velocity>().get(chaser).x == 6.0);
  }

  SECTION("ShouldSeeEarlierSystemsCommandsGivenSyncBetweenSystems") {
    REQUIRE(world.create<Body>().with(Health{0.0}).build());
    REQUIRE(world.create<Body>().with(Health{1.0}).build());
    world.sync();

    Scheduler<TestWorld, Systems<Cull, Count>> scheduler;
    scheduler.step(lib::InOut(world), STEP);

    CHECK(scheduler.system<Count>().count == 1);
    CHECK(scheduler.system<Count>().resolved);
    CHECK(world.size() == 1u);
  }

  SECTION("ShouldRunLambdaWithStateGivenCallableSystem") {
    REQUIRE(world.create<Body>().with(Health{1.0}).build());
    REQUIRE(world.create<Body>().with(Health{2.0}).build());
    world.sync();

    auto count = system<const Health>(
        [seen = 0](Entity, const Health&, auto&) mutable { return ++seen; });
    Scheduler<TestWorld, Systems<decltype(count)>> scheduler{Systems{count}};
    scheduler.step(lib::InOut(world), STEP);
    scheduler.step(lib::InOut(world), STEP);

    // The capture persists across steps: two entities, two steps.
    auto& lambda = scheduler.system<decltype(count)>().callable();
    int unused_context = 0;
    CHECK(lambda(Entity{}, Health{}, unused_context) == 5);
  }

  SECTION("ShouldReadOtherEntitiesGivenLambdaWithLookups") {
    Entity target = *world.create<Body>().with(Position{10.0}).build();
    Entity chaser =
        *world.create<Body>().with(Position{4.0}).with(Velocity{}).build();
    world.sync();

    auto chase = system<Velocity, const Position>(
        Stores<Position>{}, [target](Entity, Velocity& velocity,
                                     const Position* mine, auto& context) {
          const Position* other = lookup<Position>(context, target);
          velocity.x = (mine && other) ? other->x - mine->x : 0.0;
        });
    Scheduler<TestWorld, Systems<decltype(chase)>> scheduler{Systems{chase}};
    scheduler.step(lib::InOut(world), STEP);

    CHECK(world.store<Velocity>().get(chaser).x == 6.0);
  }

  SECTION("ShouldRunInOrderGivenMixedStructAndLambdaSystems") {
    Entity entity =
        *world.create<Body>().with(Position{0.0}).with(Velocity{1.0}).build();
    world.sync();

    auto double_velocity = system<Velocity>(
        [](Entity, Velocity& velocity, auto&) { velocity.x *= 2.0; });
    using Schedule = Systems<decltype(double_velocity), Systems<Integrate>>;
    Scheduler<TestWorld, Schedule> scheduler{
        Schedule{double_velocity, Systems<Integrate>{}}};
    scheduler.step(lib::InOut(world), STEP);

    CHECK(world.store<Velocity>().get(entity).x == 2.0);
    CHECK(world.store<Position>().get(entity).x == 1.0);  // 2.0 * 0.5
  }

  SECTION("ShouldCreateChildrenGivenSystemThatSpawns") {
    Entity launcher =
        *world.create<testing::Launcher>().with(Position{3.0}).build();
    world.sync();

    Scheduler<TestWorld, Systems<Spawn>> scheduler;
    scheduler.step(lib::InOut(world), STEP);

    REQUIRE(world.store<Position>().size() == 2u);
    Entity child = world.store<Position>().owner(1);
    CHECK(world.parent_of(child) == launcher);
    CHECK(world.store<Position>().get(child).x == 3.0);
  }

  SECTION("ShouldFlattenInOrderGivenNestedSchedules") {
    using Nested = Systems<Integrate, Systems<Cull, Count>>;
    static_assert(
        std::is_same_v<flatten_t<Nested>, TypeList<Integrate, Cull, Count>>);
  }

  SECTION("ShouldBeInvalidGivenSystemBeforeItsAfter") {
    static_assert(is_valid_schedule_v<Systems<Cull, Count>>);
    static_assert(!is_valid_schedule_v<Systems<Count, Cull>>);
    static_assert(!is_valid_schedule_v<Systems<Count>>);  // Cull is missing.
    static_assert(!is_valid_schedule_v<Systems<Cull, Cull, Count>>);
  }

  SECTION("ShouldNameSystemsBySchedulePositionGivenNestedSchedule") {
    using Scheduled =
        Scheduler<TestWorld, Systems<Integrate, Systems<Cull, Count>>>;
    static_assert(Scheduled::name_of<Integrate>() == Name{Kind::SYSTEM, 0});
    static_assert(Scheduled::name_of<Count>() == Name{Kind::SYSTEM, 2});
    CHECK(Scheduled::describe(1).contains("/world/1/system/2 "));
  }

  SECTION("ShouldDeriveReadsAndWritesGivenSystemDeclaration") {
    static_assert(std::is_same_v<writes_of_t<Integrate>, TypeList<Position>>);
    static_assert(std::is_same_v<reads_of_t<Integrate>, TypeList<Velocity>>);
    static_assert(std::is_same_v<lookups_of_t<Chase>, TypeList<Position>>);

    std::string text = Scheduler<TestWorld, Systems<Integrate>>::describe();
    CHECK(text.contains("writes: simon::core::testing::Position"));
    CHECK(text.contains("reads: simon::core::testing::Velocity"));
  }
}

}  // namespace simon::core
