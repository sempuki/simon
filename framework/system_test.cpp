// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "framework/system.hpp"

#include <chrono>
#include <stdexcept>
#include <string>
#include <vector>

#include "base/testing.hpp"
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
struct Record final : System<const Position, const Velocity> {
  void operator()(auto&, Entity entity, const Position&,
                  const Velocity* velocity) {
    seen.push_back({entity, velocity != nullptr});
  }
  std::vector<std::pair<Entity, bool>> seen;
};

struct Integrate final : System<Position, const Velocity> {
  void operator()(auto&, Entity, Position& position, const Velocity* velocity,
                  Step step) {
    if (!velocity) return;
    position.x += velocity->x * std::chrono::duration<double>(step.dt).count();
  }
};

struct Chase final : System<Velocity, const Health> {
  using AllowComponentList = TypeList<Position>;
  void operator()(auto& world, Entity self, Velocity& velocity, const Health*) {
    const Position* mine = try_component_of<Position>(world, self);
    const Position* other = try_component_of<Position>(world, target);
    velocity.x = (mine && other) ? other->x - mine->x : 0.0;
  }
  Entity target;
};

// Destroys every entity whose health is gone.
struct Cull final : System<const Health> {
  void operator()(auto& world, Entity self, const Health& health) {
    if (health.points <= 0.0) {
      REQUIRE(world.destroy(self).build());
    }
  }
};

// Spawns a child under every launcher, from inside a system.
struct Spawn final : System<const Position> {
  void operator()(auto& world, Entity self, const Position& position) {
    REQUIRE(create<testing::Interceptor>(lib::InOut(world))
                .under(self)
                .with(Position{position.x})
                .with(Velocity{})
                .build());
  }
};

// Counts, for each entity, whether it has a Velocity and whether that Velocity
// is its own. Every entity in the test has equal Position and Velocity.
struct CheckSiblings final : System<const Position, const Velocity> {
  void operator()(auto&, Entity, const Position& position,
                  const Velocity* velocity) {
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
struct Skippable final : System<const Health> {
  bool prepare(auto&) { return run; }
  void operator()(auto&, Entity, const Health&) { ++called; }
  void resolve(auto&) { resolved = true; }
  bool run = true;
  int called = 0;
  bool resolved = false;
};

// Counts entities with health; runs after Cull, so it sees Cull's commands.
struct Count final : System<const Health> {
  using SequenceAfterSystemList = SystemList<Cull>;
  void prepare(auto&) { count = 0; }
  void operator()(auto&, Entity, const Health&) { ++count; }
  void resolve(auto&) { resolved = true; }
  int count = 0;
  bool resolved = false;
};

}  // namespace

TEST_CASE("BytesPerEntity") {
  SECTION("ShouldCountOwnerAndNamedComponentsGivenSystem") {
    // Integrate names Position and const Velocity.
    STATIC_CHECK(bytes_per_entity_v<Integrate> ==
                 sizeof(Entity) + sizeof(Position) + sizeof(Velocity));
    // Cull names only const Health.
    STATIC_CHECK(bytes_per_entity_v<Cull> == sizeof(Entity) + sizeof(Health));
  }
}

TEST_CASE("System") {
  TestWorld world{testing::small_world()};

  SECTION("ShouldRunForDriverWithOptionalPointerGivenMixedComponents") {
    // x holds (a), y holds (b), z holds (a, b).
    Entity x = *world.create<Body>().with(Position{}).build();
    Entity y = *world.create<Body>().with(Velocity{}).build();
    Entity z = *world.create<Body>().with(Position{}).with(Velocity{}).build();
    world.sync();

    Scheduler<TestWorld, SystemList<Record>> scheduler;
    scheduler.step(lib::InOut(world), STEP);

    auto& seen = scheduler.system<Record>().seen;
    REQUIRE(seen.size() == 2u);
    CHECK(seen[0] == std::pair{x, false});
    CHECK(seen[1] == std::pair{z, true});
    DECLARE_UNUSED(y);
  }

  SECTION("ShouldWriteDrivingComponentGivenStep") {
    Entity entity =
        *world.create<Body>().with(Position{1.0}).with(Velocity{2.0}).build();
    world.sync();

    Scheduler<TestWorld, SystemList<Integrate>> scheduler;
    scheduler.step(lib::InOut(world), STEP);

    CHECK(world.store_of<Position>().component_of(entity).x == 2.0);
  }

  SECTION("ShouldQueryMovedPositionsGivenSystemWroteSpatialComponent") {
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
    scheduler.step(lib::InOut(world), STEP);

    CHECK(found_near(1.0).empty());
    CHECK(found_near(2.0) == std::vector<Entity>{entity});
  }

  SECTION("ShouldPassEachEntitysOwnSiblingGivenArchetypesAndChurn") {
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
    scheduler.step(lib::InOut(world), STEP);

    const CheckSiblings& check = scheduler.system<CheckSiblings>();
    CHECK(check.with == 5);
    CHECK(check.matched == 5);
    CHECK(check.without == 2);
  }

  SECTION("ShouldSkipLoopButResolveGivenPrepareReturnsFalse") {
    REQUIRE(world.create<Body>().with(Health{1.0}).build());
    world.sync();
    Scheduler<TestWorld, SystemList<Skippable>> scheduler;

    scheduler.system<Skippable>().run = false;
    scheduler.step(lib::InOut(world), STEP);
    CHECK(scheduler.system<Skippable>().called == 0);
    CHECK(scheduler.system<Skippable>().resolved);

    scheduler.system<Skippable>().run = true;
    scheduler.step(lib::InOut(world), STEP);
    CHECK(scheduler.system<Skippable>().called == 1);
  }

  SECTION("ShouldReadOtherEntitiesGivenDeclaredLookup") {
    Entity target = *world.create<Body>().with(Position{10.0}).build();
    Entity chaser =
        *world.create<Body>().with(Position{4.0}).with(Velocity{}).build();
    world.sync();

    Scheduler<TestWorld, SystemList<Chase>> scheduler;
    scheduler.system<Chase>().target = target;
    scheduler.step(lib::InOut(world), STEP);

    CHECK(world.store_of<Velocity>().component_of(chaser).x == 6.0);
  }

  SECTION("ShouldSeeEarlierSystemsCommandsGivenSyncBetweenSystems") {
    REQUIRE(world.create<Body>().with(Health{0.0}).build());
    REQUIRE(world.create<Body>().with(Health{1.0}).build());
    world.sync();

    Scheduler<TestWorld, SystemList<Cull, Count>> scheduler;
    scheduler.step(lib::InOut(world), STEP);

    CHECK(scheduler.system<Count>().count == 1);
    CHECK(scheduler.system<Count>().resolved);
    CHECK(world.size() == 1u);
  }

  SECTION("ShouldThrowGivenComponentOfMissingComponent") {
    Entity bare = *world.create<Body>().build();
    Entity placed = *world.create<Body>().with(Position{2.0}).build();
    world.sync();

    WorldAccess<Chase, TestWorld> access{lib::Depend<TestWorld>{world}};

    CHECK(access.component_of<Position>(placed).x == 2.0);
    CHECK(access.try_component_of<Position>(bare) == nullptr);
    CHECK_THROWS_AS(access.component_of<Position>(bare), std::logic_error);
  }

  SECTION("ShouldRunLambdaWithStateGivenCallableSystem") {
    REQUIRE(world.create<Body>().with(Health{1.0}).build());
    REQUIRE(world.create<Body>().with(Health{2.0}).build());
    world.sync();

    auto count = system<const Health>(
        [seen = 0](auto&, Entity, const Health&) mutable { return ++seen; });
    Scheduler<TestWorld, SystemList<decltype(count)>> scheduler{
        SystemList{count}};
    scheduler.step(lib::InOut(world), STEP);
    scheduler.step(lib::InOut(world), STEP);

    // The capture persists across steps: two entities, two steps.
    auto& lambda = scheduler.system<decltype(count)>().callable();
    int unused_world = 0;
    CHECK(lambda(unused_world, Entity{}, Health{}) == 5);
  }

  SECTION("ShouldReadOtherEntitiesGivenLambdaWithLookups") {
    Entity target = *world.create<Body>().with(Position{10.0}).build();
    Entity chaser =
        *world.create<Body>().with(Position{4.0}).with(Velocity{}).build();
    world.sync();

    auto chase = system<Velocity, const Position>(
        TypeList<Position>{}, [target](auto& world, Entity, Velocity& velocity,
                                       const Position* mine) {
          const Position* other = try_component_of<Position>(world, target);
          velocity.x = (mine && other) ? other->x - mine->x : 0.0;
        });
    Scheduler<TestWorld, SystemList<decltype(chase)>> scheduler{
        SystemList{chase}};
    scheduler.step(lib::InOut(world), STEP);

    CHECK(world.store_of<Velocity>().component_of(chaser).x == 6.0);
  }

  SECTION("ShouldRunInOrderGivenMixedStructAndLambdaSystems") {
    Entity entity =
        *world.create<Body>().with(Position{0.0}).with(Velocity{1.0}).build();
    world.sync();

    auto double_velocity = system<Velocity>(
        [](auto&, Entity, Velocity& velocity) { velocity.x *= 2.0; });
    using Schedule =
        SystemList<decltype(double_velocity), SystemList<Integrate>>;
    Scheduler<TestWorld, Schedule> scheduler{
        Schedule{double_velocity, SystemList<Integrate>{}}};
    scheduler.step(lib::InOut(world), STEP);

    CHECK(world.store_of<Velocity>().component_of(entity).x == 2.0);
    CHECK(world.store_of<Position>().component_of(entity).x ==
          1.0);  // 2.0 * 0.5
  }

  SECTION("ShouldCreateChildrenGivenSystemThatSpawns") {
    Entity launcher =
        *world.create<testing::Launcher>().with(Position{3.0}).build();
    world.sync();

    Scheduler<TestWorld, SystemList<Spawn>> scheduler;
    scheduler.step(lib::InOut(world), STEP);

    REQUIRE(world.store_of<Position>().size() == 2u);
    Entity child;
    world.store_of<Position>().for_each([&](Entity owner, const Position&) {
      child = owner != launcher ? owner : child;
    });
    CHECK(world.parent_of(child) == launcher);
    CHECK(world.store_of<Position>().component_of(child).x == 3.0);
  }

  SECTION("ShouldFlattenInOrderGivenNestedSchedules") {
    using Nested = SystemList<Integrate, SystemList<Cull, Count>>;
    static_assert(std::is_same_v<flattened_list_t<Nested>,
                                 TypeList<Integrate, Cull, Count>>);
  }

  SECTION("ShouldBeInvalidOnlyGivenSystemBeforeItsAfter") {
    static_assert(is_valid_schedule_v<SystemList<Cull, Count>>);
    static_assert(!is_valid_schedule_v<SystemList<Count, Cull>>);
    static_assert(
        is_valid_schedule_v<SystemList<Count>>);  // Cull absent: fine.
    static_assert(!is_valid_schedule_v<SystemList<Cull, Cull, Count>>);
  }

  SECTION("ShouldNameSystemsBySchedulePositionGivenNestedSchedule") {
    using Scheduled =
        Scheduler<TestWorld, SystemList<Integrate, SystemList<Cull, Count>>>;
    static_assert(Scheduled::name_of<Integrate>() == Name{Kind::SYSTEM, 0});
    static_assert(Scheduled::name_of<Count>() == Name{Kind::SYSTEM, 2});
    CHECK(Scheduled::describe(1).contains("/world/1/system/2 "));
  }

  SECTION("ShouldDeriveReadsAndWritesGivenSystemDeclaration") {
    static_assert(
        std::is_same_v<write_list_of_t<Integrate>, TypeList<Position>>);
    static_assert(
        std::is_same_v<read_list_of_t<Integrate>, TypeList<Velocity>>);
    static_assert(
        std::is_same_v<allow_component_list_of_t<Chase>, TypeList<Position>>);

    std::string text = Scheduler<TestWorld, SystemList<Integrate>>::describe();
    CHECK(text.contains("writes: simon::framework::testing::Position"));
    CHECK(text.contains("reads: simon::framework::testing::Velocity"));
  }
}

}  // namespace simon::framework
