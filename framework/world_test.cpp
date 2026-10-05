// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "framework/world.hpp"
#include "framework/vocabulary.hpp"

#include <expected>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <vector>

#include "base/testing.hpp"
#include "framework/test_world.hpp"

namespace simon::framework {

using testing::Body;
using testing::Health;
using testing::Interceptor;
using testing::Launcher;
using testing::Position;
using testing::TestWorld;
using testing::Velocity;

template <typename BuilderType>
concept CanParent =
    requires(BuilderType builder) { std::move(builder).under(Entity{}); };

TEST_CASE("SetUpBuilder") {
  SECTION("ShouldSizeEachStoreByArchetypesThatPermitItGivenHoldings") {
    TestWorld world;
    std::expected<void, Status> built =
        TestWorld::set_up()
            .numbered(3)
            .holding<Body>(5)
            .holding<testing::Launcher>(2)
            .holding<testing::Launcher>(1)  // Holdings add up.
            .build(Out(world));

    REQUIRE(built.has_value());
    CHECK(world.number() == 3u);
    // Every archetype requires or allows Position; only bodies allow Health;
    // bodies allow and interceptors require Velocity.
    CHECK(world.store_of<Position>().capacity() == 8u);
    CHECK(world.store_of<Health>().capacity() == 5u);
    CHECK(world.store_of<Velocity>().capacity() == 5u);
  }

  SECTION("ShouldRefuseGivenCellSizeNotPositive") {
    TestWorld world;
    auto built =
        TestWorld::set_up().holding<Body>(1).cells_of(0.0).build(Out(world));

    REQUIRE_FALSE(built.has_value());
    CHECK(built.error() == lib::watch(BuildError::CELL_SIZE_INVALID));
  }

  SECTION("ShouldRefuseGivenHoldingMoreThanAStoreCanIndex") {
    TestWorld world;
    auto built = TestWorld::set_up()
                     .holding<Body>(std::size_t{1} << 40)
                     .build(Out(world));

    REQUIRE_FALSE(built.has_value());
    CHECK(built.error() == lib::watch(BuildError::CAPACITY_TOO_LARGE));
  }

  SECTION("ShouldRefuseGivenHoldingsThatWrapWhenAdded") {
    TestWorld world;
    auto built = TestWorld::set_up()
                     .holding<Body>(std::numeric_limits<std::size_t>::max())
                     .holding<testing::Launcher>(2)
                     .build(Out(world));

    REQUIRE_FALSE(built.has_value());
    CHECK(built.error() == lib::watch(BuildError::CAPACITY_TOO_LARGE));
  }

  SECTION("ShouldHoldNothingGivenDefaultWorld") {
    TestWorld world;

    auto created = world.create<Body>().build();

    REQUIRE_FALSE(created.has_value());
    CHECK(created.error() == lib::watch(BuildError::ENTITY_CAPACITY_EXHAUSTED));
  }

  SECTION("ShouldDiscardEverythingGivenWorldBuiltAgain") {
    TestWorld world;
    testing::build_small_world(Out(world));
    Entity entity = *world.create<Body>("ego").with(Position{}).build();
    world.sync();

    testing::build_small_world(Out(world));

    CHECK_FALSE(world.alive(entity));
    CHECK(world.size() == 0u);
    CHECK(world.find_name_of(Alias{"ego"}).empty());
    CHECK(world.store_of<Position>().size() == 0u);
  }

  SECTION("ShouldKeepTheWorldGivenRefusedPlan") {
    TestWorld world;
    testing::build_small_world(Out(world));
    Entity entity = *world.create<Body>().build();
    world.sync();

    auto built = TestWorld::set_up().cells_of(-1.0).build(Out(world));

    REQUIRE_FALSE(built.has_value());
    CHECK(world.alive(entity));
  }

  SECTION("ShouldNeverMoveGivenAWorld") {
    STATIC_CHECK_FALSE(std::is_move_constructible_v<TestWorld>);
    STATIC_CHECK_FALSE(std::is_move_assignable_v<TestWorld>);
  }
}

TEST_CASE("Transaction") {
  TestWorld world;
  testing::build_small_world(Out(world));
  Entity kept = *world.create<Body>("kept").with(Position{1.0}).build();
  world.sync();

  SECTION("ShouldUndoEverythingPlannedGivenRollBack") {
    // Planned before the transaction, so it stays.
    REQUIRE(world.create<Body>().build());
    std::size_t pending = world.pending();
    Entity created;
    {
      auto transaction = world.transaction();
      created = *world.create<Body>("ghost").with(Position{2.0}).build();
      REQUIRE(world.change(kept).attach(Velocity{}).alias("renamed").build());
      REQUIRE(world.destroy(kept).build());
      transaction.roll_back();
    }

    CHECK(world.pending() == pending);
    CHECK_FALSE(world.alive(created));
    CHECK(world.find_name_of(Alias{"ghost"}).empty());
    CHECK(world.find_name_of(Alias{"renamed"}).empty());
    // The plan is as it was: kept can gain Velocity and be destroyed again.
    REQUIRE(world.change(kept).attach(Velocity{}).build());
    REQUIRE(world.destroy(kept).build());
    world.sync();
    CHECK_FALSE(world.alive(kept));
  }

  SECTION("ShouldRollBackGivenTransactionEndsWithoutCommit") {
    Entity created;
    {
      auto transaction = world.transaction();
      created = *world.create<Body>().build();
    }

    CHECK_FALSE(world.alive(created));
    CHECK(world.pending() == 0u);
  }

  SECTION("ShouldApplyAtSyncGivenCommit") {
    Entity created;
    {
      auto transaction = world.transaction();
      created = *world.create<Body>().with(Position{3.0}).build();
      REQUIRE(world.change(kept).detach<Position>().build());
      transaction.commit();
    }
    world.sync();

    CHECK(world.store_of<Position>().component_of(created).x == 3.0);
    CHECK_FALSE(world.store_of<Position>().contains(kept));
  }

  SECTION("ShouldUndoInnerCommitGivenOuterRollsBack") {
    Entity inner;
    {
      auto outer = world.transaction();
      {
        auto nested = world.transaction();
        inner = *world.create<Body>().build();
        nested.commit();
      }
      outer.roll_back();
    }

    CHECK_FALSE(world.alive(inner));
  }

  SECTION("ShouldKeepOuterWorkGivenInnerRollsBack") {
    Entity outer_entity;
    Entity inner_entity;
    {
      auto outer = world.transaction();
      outer_entity = *world.create<Body>().build();
      {
        auto nested = world.transaction();
        inner_entity = *world.create<Body>().build();
      }
      outer.commit();
    }
    world.sync();

    CHECK(world.alive(outer_entity));
    CHECK_FALSE(world.alive(inner_entity));
  }

  SECTION("ShouldReturnCapacityGivenRolledBackCreations") {
    {
      auto transaction = world.transaction();
      while (world.create<Body>().build()) {
      }
    }

    // Everything the transaction reserved is free again.
    CHECK(world.create<Body>().build().has_value());
  }

  SECTION("ShouldRefuseSyncGivenOpenTransaction") {
    auto transaction = world.transaction();

    CHECK_THROWS_AS(world.sync(), std::logic_error);
  }
}

TEST_CASE("DestroyQueryBuilder") {
  TestWorld world;
  testing::build_small_world(Out(world));
  Entity near = *world.create<testing::Launcher>().with(Position{1.0}).build();
  Entity far = *world.create<testing::Launcher>().with(Position{9.0}).build();
  Entity body =
      *world.create<Body>().with(Position{1.5}).with(Health{1.0}).build();
  world.sync();

  SECTION("ShouldDestroyEveryEntityOfArchetypeGivenEachArchetype") {
    auto destroyed = world.destroy().each<testing::Launcher>().build();
    world.sync();

    REQUIRE(destroyed.has_value());
    CHECK(*destroyed == 2u);
    CHECK_FALSE(world.alive(near));
    CHECK_FALSE(world.alive(far));
    CHECK(world.alive(body));
  }

  SECTION("ShouldDestroyEveryEntityWithComponentGivenEachComponent") {
    auto destroyed = world.destroy().each<Health>().build();
    world.sync();

    CHECK(destroyed == 1u);
    CHECK_FALSE(world.alive(body));
    CHECK(world.alive(near));
  }

  SECTION("ShouldDestroyOnlyNearbyGivenWithin") {
    auto destroyed = world.destroy()
                         .each<testing::Launcher>()
                         .within(Position{0.0}, 2.0)
                         .build();
    world.sync();

    CHECK(destroyed == 1u);
    CHECK_FALSE(world.alive(near));
    CHECK(world.alive(far));
    CHECK(world.alive(body));  // Nearby, but not a launcher.
  }

  SECTION("ShouldDestroyOnlyAcceptedGivenWhere") {
    auto destroyed = world.destroy()
                         .each<Position>()
                         .where([&](Entity entity) { return entity != far; })
                         .where([&](Entity entity) { return entity != body; })
                         .build();
    world.sync();

    CHECK(destroyed == 1u);
    CHECK_FALSE(world.alive(near));
    CHECK(world.alive(far));
    CHECK(world.alive(body));
  }

  SECTION("ShouldSkipEntitiesGivenDestructionAlreadyPlanned") {
    REQUIRE(world.destroy(near).build());

    auto destroyed = world.destroy().each<testing::Launcher>().build();

    REQUIRE(destroyed.has_value());
    CHECK(*destroyed == 1u);
    world.sync();
    CHECK_FALSE(world.alive(far));
  }
}

TEST_CASE("ChangeQueryBuilder") {
  TestWorld world;
  testing::build_small_world(Out(world));
  Entity wounded =
      *world.create<Body>().with(Position{1.0}).with(Health{1.0}).build();
  Entity healthy = *world.create<Body>().with(Position{1.5}).build();
  Entity distant = *world.create<Body>().with(Position{9.0}).build();
  Entity launcher =
      *world.create<testing::Launcher>().with(Position{1.2}).build();
  world.sync();

  SECTION("ShouldChangeEveryEntityOfArchetypeGivenEachArchetype") {
    auto changed = world.change().each<Body>().attach(Velocity{}).build();
    world.sync();

    REQUIRE(changed.has_value());
    CHECK(*changed == 3u);
    CHECK(world.store_of<Velocity>().contains(wounded));
    CHECK(world.store_of<Velocity>().contains(healthy));
    CHECK(world.store_of<Velocity>().contains(distant));
  }

  SECTION("ShouldChangeOnlyNearbyAcceptedGivenWithinAndWhere") {
    auto changed = world.change()
                       .each<Body>()
                       .within(Position{0.0}, 2.0)
                       .where([&](Entity entity) { return entity != wounded; })
                       .detach<Position>()
                       .build();
    world.sync();

    CHECK(changed == 1u);
    CHECK_FALSE(world.store_of<Position>().contains(healthy));
    CHECK(world.store_of<Position>().contains(wounded));
    CHECK(world.store_of<Position>().contains(distant));
    CHECK(world.store_of<Position>().contains(launcher));
  }

  SECTION("ShouldChangeNothingGivenAnyMatchRefuses") {
    // Bodies allow Velocity; launchers do not.
    auto changed = world.change().each<Position>().attach(Velocity{}).build();

    REQUIRE_FALSE(changed.has_value());
    CHECK(changed.error() == lib::watch(BuildError::COMPONENT_NOT_PERMITTED));
    CHECK(world.pending() == 0u);
  }

  SECTION("ShouldSkipEntitiesGivenDestructionAlreadyPlanned") {
    REQUIRE(world.destroy(distant).build());

    auto changed = world.change().each<Body>().attach(Velocity{}).build();

    CHECK(changed == 2u);
  }

  SECTION("ShouldChangeOnlyHoldersGivenHaving") {
    auto changed =
        world.change().each<Body>().having<Health>().detach<Health>().build();
    world.sync();

    CHECK(changed == 1u);
    CHECK(world.store_of<Health>().size() == 0u);
  }

  SECTION("ShouldSkipPlannedAttachGivenLacking") {
    REQUIRE(world.change(wounded).attach(Velocity{}).build());

    auto changed = world.change()
                       .each<Body>()
                       .lacking<Velocity>()
                       .attach(Velocity{})
                       .build();
    world.sync();

    CHECK(changed == 2u);  // Without lacking, wounded would refuse it all.
    CHECK(world.store_of<Velocity>().size() == 3u);
  }

  SECTION("ShouldAliasEveryMatchGivenAlias") {
    auto changed = world.change()
                       .each<Body>()
                       .within(Position{0.0}, 2.0)
                       .alias("hostile")
                       .build();

    CHECK(changed == 2u);
    CHECK(world.find_name_of(Alias{"hostile"}).size() == 2u);
    CHECK(world.aliases_of(world.name_of(distant)).empty());
  }
}

TEST_CASE("World") {
  TestWorld world;
  testing::build_small_world(Out(world));

  SECTION("ShouldVisitInKeyOrderGivenReorder") {
    // Four interceptors created out of order, and a launcher: reordered by
    // position, the interceptors come in that order in every store they
    // require, each keeping its own components, and the launcher stays.
    Entity launcher = *world.create<Launcher>().with(Position{9.0}).build();
    std::vector<Entity> interceptors;
    for (double x : {3.0, 1.0, 4.0, 2.0}) {
      interceptors.push_back(*world.create<Interceptor>()
                                  .with(Position{x})
                                  .with(Velocity{10.0 * x})
                                  .build());
    }
    world.sync();
    world.reorder<Interceptor>([&](Entity entity) {
      return world.store_of<Position>().component_of(entity).x;
    });

    std::vector<double> positions;
    std::vector<double> velocities;
    world.store_of<Position>().for_each([&](Entity entity, const Position& p) {
      if (entity != launcher) {
        positions.push_back(p.x);
      }
    });
    world.store_of<Velocity>().for_each(
        [&](Entity, const Velocity& v) { velocities.push_back(v.x); });
    CHECK(positions == std::vector{1.0, 2.0, 3.0, 4.0});
    CHECK(velocities == std::vector{10.0, 20.0, 30.0, 40.0});
    for (Entity entity : interceptors) {
      CHECK(world.store_of<Velocity>().component_of(entity).x ==
            10.0 * world.store_of<Position>().component_of(entity).x);
    }
    CHECK(world.store_of<Position>().component_of(launcher).x == 9.0);
    std::vector<Entity> near;
    world.within(Position{1.0}, 0.5, [&](Entity entity, const Position&) {
      near.push_back(entity);
    });
    CHECK(near == std::vector{interceptors[1]});
  }

  SECTION("ShouldDeferComponentsUntilSyncGivenCreate") {
    auto entity =
        world.create<Body>().with(Position{1.0}).with(Velocity{2.0}).build();
    REQUIRE(entity);
    CHECK(world.alive(*entity));
    CHECK_FALSE(world.store_of<Position>().contains(*entity));

    world.sync();

    CHECK(world.store_of<Position>().component_of(*entity).x == 1.0);
    CHECK(world.store_of<Velocity>().component_of(*entity).x == 2.0);
    CHECK(world.pending() == 0u);
  }

  SECTION("ShouldNameEntitiesInCreationOrderGivenCreate") {
    Entity first = *world.create<Body>().build();
    Entity second = *world.create<Body>().build();

    CHECK(world.name_of(first) == Name{Kind::ENTITY, 0});
    CHECK(world.name_of(second) == Name{Kind::ENTITY, 1});
    CHECK(world.entity_of(Name{Kind::ENTITY, 1}) == second);
  }

  SECTION("ShouldNeverReuseNameGivenIndexReused") {
    TestWorld tiny;
    REQUIRE(TestWorld::set_up().numbered(1).holding<Body>(1).build(Out(tiny)));
    Entity first = *tiny.create<Body>().build();
    tiny.sync();
    REQUIRE(tiny.destroy(first).build());
    tiny.sync();

    Entity second = *tiny.create<Body>().build();

    CHECK(second.index == first.index);
    CHECK(tiny.name_of(second) == Name{Kind::ENTITY, 1});
    CHECK_FALSE(tiny.entity_of(Name{Kind::ENTITY, 0}));
  }

  SECTION("ShouldFormatAndFindIdentityGivenEntityAndComponent") {
    Entity entity = *world.create<Body>().with(Position{}).build();
    world.sync();
    Name position = world.name_of<Position>(entity);

    CHECK(world.format_identity(world.name()) == "/world/1");
    CHECK(world.format_identity(world.name_of(entity)) == "/world/1/entity/0");
    CHECK(world.format_identity(position) == "/world/1/entity/0/component/0");
    CHECK(world.find_name_of(Identity{"/world/1/entity/0"}) ==
          world.name_of(entity));
    CHECK(world.find_name_of(Identity{"/world/1/entity/0/component/0"}) ==
          position);
    CHECK(world.entity_of(position) == entity);
  }

  SECTION("ShouldNotFindGivenIdentityOfSomethingAbsent") {
    auto _ = *world.create<Body>().build();  // Entity 0, without a Position.
    world.sync();

    CHECK_FALSE(
        world.find_name_of(Identity{"/world/2/entity/0"}));  // Another world.
    CHECK_FALSE(
        world.find_name_of(Identity{"/world/1/entity/9"}));  // No such entity.
    CHECK_FALSE(world.find_name_of(
        Identity{"/world/1/entity/0/component/0"}));  // No Position.
    CHECK_FALSE(world.find_name_of(
        Identity{"/world/1/component/99"}));               // No such component.
    CHECK_FALSE(world.find_name_of(Identity{"world/1"}));  // Not an identity.
    CHECK(world.find_name_of(Identity{"/world/1/component/0"}) ==
          TestWorld::name_of<Position>());
  }

  SECTION("ShouldFindComponentByTypeNameGivenBuiltInAliases") {
    CHECK(world.find_name_of(Alias{"Position"}) ==
          std::vector{TestWorld::name_of<Position>()});
    CHECK(world.find_name_of(Alias{"simon::framework::testing::Position"}) ==
          std::vector{TestWorld::name_of<Position>()});
  }

  SECTION("ShouldAliasEntityGivenCreateAlias") {
    Entity luke = *world.create<Body>("Luke Skywalker").build();

    CHECK(world.find_name_of(Alias{"Luke Skywalker"}) ==
          std::vector{world.name_of(luke)});
    CHECK(world.aliases_of(world.name_of(luke)) ==
          std::vector<Alias>{"Luke Skywalker"});
  }

  SECTION("ShouldShareAliasGivenManyEntities") {
    Entity first = *world.create<Body>("red drone").build();
    Entity second = *world.create<Body>("red drone").build();

    CHECK(world.find_name_of(Alias{"red drone"}) ==
          std::vector{world.name_of(first), world.name_of(second)});
  }

  SECTION("ShouldGiveAndTakeAliasesGivenChange") {
    Entity entity = *world.create<Body>().build();

    REQUIRE(world.change(entity).alias("ego").alias("hero").build());
    CHECK(world.find_name_of(Alias{"ego"}) ==
          std::vector{world.name_of(entity)});

    REQUIRE(world.change(entity).unalias("ego").build());
    CHECK(world.find_name_of(Alias{"ego"}).empty());
    CHECK(world.aliases_of(world.name_of(entity)) ==
          std::vector<Alias>{"hero"});
  }

  SECTION("ShouldRefuseAliasChangeGivenInvalidDuplicateOrMissingAlias") {
    Entity entity = *world.create<Body>("ego").build();

    auto empty = world.change(entity).alias("").build();
    auto duplicate = world.change(entity).alias("ego").build();
    auto twice = world.change(entity).alias("hero").alias("hero").build();
    auto missing = world.change(entity).unalias("villain").build();

    CHECK(empty.error() == lib::watch(BuildError::ALIAS_INVALID));
    CHECK(duplicate.error() == lib::watch(BuildError::ALIAS_ALREADY_GIVEN));
    CHECK(twice.error() == lib::watch(BuildError::ALIAS_ALREADY_GIVEN));
    CHECK(missing.error() == lib::watch(BuildError::ALIAS_NOT_GIVEN));
    CHECK(world.aliases_of(world.name_of(entity)) == std::vector<Alias>{"ego"});
  }

  SECTION("ShouldRecordArchetypeGivenCreate") {
    Entity launcher =
        *world.create<testing::Launcher>().with(Position{}).build();
    Entity body = *world.create<Body>().build();
    world.sync();

    Name archetype = world.archetype_of(launcher);
    CHECK(archetype.kind == static_cast<std::uint32_t>(Kind::ARCHETYPE));
    CHECK(world.aliases_of(archetype) == std::vector<Alias>{"launcher"});
    CHECK(world.archetype_of(body) != archetype);
    CHECK(world.format_identity(archetype) == "/world/1/archetype/0");
  }

  SECTION("ShouldRecordParentGivenUnder") {
    Entity launcher =
        *world.create<testing::Launcher>().with(Position{}).build();
    Entity interceptor = *world.create<testing::Interceptor>()
                              .under(launcher)
                              .with(Position{})
                              .with(Velocity{})
                              .build();
    world.sync();

    CHECK(world.parent_of(interceptor) == launcher);
    CHECK_FALSE(world.parent_of(launcher));
  }

  SECTION("ShouldRejectUnderGivenAfterWith") {
    using Parenting = decltype(world.create<Body>());
    using Declaring = decltype(world.create<Body>().with(Position{}));
    static_assert(CanParent<Parenting>);
    static_assert(!CanParent<Declaring>);
  }

  SECTION("ShouldDescribeForConsoleGivenEntity") {
    Entity entity =
        *world.create<Body>("red").with(Position{}).with(Health{}).build();
    world.sync();

    CHECK(world.describe(world.name_of(entity)) ==
          "/world/1/entity/0 (red) archetype body: Position, Health");
  }

  SECTION("ShouldEraseFromEveryStoreNameAndAliasGivenDestroy") {
    Entity entity =
        *world.create<Body>("ball").with(Position{}).with(Health{}).build();
    world.sync();
    Name name = world.name_of(entity);

    REQUIRE(world.destroy(entity).build());
    world.sync();

    CHECK_FALSE(world.alive(entity));
    CHECK(world.store_of<Position>().size() == 0u);
    CHECK(world.store_of<Health>().size() == 0u);
    CHECK(world.store_of<EntityArchetype>().size() == 0u);
    CHECK_FALSE(world.entity_of(name));
    CHECK(world.find_name_of(Alias{"ball"}).empty());
  }

  SECTION("ShouldAttachAndDetachGivenChange") {
    Entity entity = *world.create<Body>().with(Position{}).build();
    world.sync();

    REQUIRE(
        world.change(entity).attach(Health{5.0}).detach<Position>().build());
    world.sync();

    CHECK(world.store_of<Health>().component_of(entity).points == 5.0);
    CHECK_FALSE(world.store_of<Position>().contains(entity));
  }

  SECTION("ShouldReturnErrorWithoutCommandsGivenInvalidChange") {
    Entity entity = *world.create<Body>().with(Position{}).build();
    world.sync();

    auto result =
        world.change(entity).attach(Velocity{}).attach(Position{}).build();

    CHECK_FALSE(result);
    CHECK(world.pending() == 0u);
  }

  SECTION("ShouldReturnErrorGivenStaleEntity") {
    Entity entity = *world.create<Body>().build();
    world.sync();
    REQUIRE(world.destroy(entity).build());
    world.sync();

    CHECK_FALSE(world.destroy(entity).build());
    CHECK_FALSE(world.change(entity).attach(Health{}).build());
  }

  SECTION("ShouldRefuseSecondDestroyGivenDestroyAlreadyPending") {
    Entity entity = *world.create<Body>().with(Health{}).build();
    world.sync();

    REQUIRE(world.destroy(entity).build());
    auto second = world.destroy(entity).build();
    world.sync();

    REQUIRE_FALSE(second);
    CHECK(second.error() == lib::watch(BuildError::ENTITY_NOT_ALIVE));
    CHECK_FALSE(world.alive(entity));
  }

  SECTION("ShouldRefuseChangeGivenDestroyPending") {
    Entity entity = *world.create<Body>().build();
    world.sync();
    REQUIRE(world.destroy(entity).build());

    auto change = world.change(entity).attach(Health{}).build();

    REQUIRE_FALSE(change);
    CHECK(change.error() == lib::watch(BuildError::ENTITY_NOT_ALIVE));
    world.sync();
  }

  SECTION("ShouldRefuseSecondAttachGivenAttachAlreadyPending") {
    Entity entity = *world.create<Body>().build();
    world.sync();
    REQUIRE(world.change(entity).attach(Health{1.0}).build());

    auto second = world.change(entity).attach(Health{2.0}).build();

    REQUIRE_FALSE(second);
    CHECK(second.error() == lib::watch(BuildError::COMPONENT_ALREADY_ATTACHED));
    REQUIRE_NOTHROW(world.sync());
    CHECK(world.store_of<Health>().component_of(entity).points == 1.0);
  }

  SECTION("ShouldAllowReattachGivenDetachPendingInSameBatch") {
    Entity entity = *world.create<Body>().with(Health{1.0}).build();
    world.sync();
    REQUIRE(world.change(entity).detach<Health>().build());

    REQUIRE(world.change(entity).attach(Health{2.0}).build());
    world.sync();

    CHECK(world.store_of<Health>().component_of(entity).points == 2.0);
  }

  SECTION("ShouldRefuseAttachGivenStoreFullAfterPendingAttaches") {
    // Only bodies allow Health, so its store holds 2; launchers fill the
    // rest of the 8 entities.
    TestWorld small;
    REQUIRE(TestWorld::set_up()
                .numbered(1)
                .holding<Body>(2)
                .holding<testing::Launcher>(6)
                .build(Out(small)));
    REQUIRE(small.create<Body>().with(Health{}).build());
    REQUIRE(small.create<Body>()
                .with(Health{})
                .build());  // Pending: store will be full.
    Entity extra = *small.create<Body>().build();

    auto create = small.create<Body>().with(Health{}).build();
    auto attach = small.change(extra).attach(Health{}).build();

    REQUIRE_FALSE(create);
    CHECK(create.error() ==
          lib::watch(BuildError::COMPONENT_CAPACITY_EXHAUSTED));
    REQUIRE_FALSE(attach);
    CHECK(attach.error() ==
          lib::watch(BuildError::COMPONENT_CAPACITY_EXHAUSTED));
    REQUIRE_NOTHROW(small.sync());
    CHECK(small.store_of<Health>().size() == 2u);
  }

  SECTION("ShouldAttachGivenStoreFullButDetachOrDestroyPending") {
    // Only bodies allow Health, so its store holds 2.
    TestWorld small;
    REQUIRE(TestWorld::set_up()
                .numbered(1)
                .holding<Body>(2)
                .holding<testing::Launcher>(6)
                .build(Out(small)));
    Entity detached = *small.create<Body>().with(Health{}).build();
    Entity destroyed = *small.create<Body>().with(Health{}).build();
    Entity first = *small.create<Body>().build();
    Entity second = *small.create<Body>().build();
    small.sync();

    REQUIRE_FALSE(small.change(first).attach(Health{}).build());  // Full.
    REQUIRE(small.change(detached).detach<Health>().build());
    auto after_detach = small.change(first).attach(Health{}).build();
    REQUIRE(small.destroy(destroyed).build());
    auto after_destroy = small.change(second).attach(Health{}).build();
    small.sync();

    CHECK(after_detach.has_value());
    CHECK(after_destroy.has_value());
    CHECK(small.store_of<Health>().size() == 2u);
  }

  SECTION("ShouldRefuseAttachGivenRoomMadeOnlyInRolledBackTransaction") {
    TestWorld small;
    REQUIRE(TestWorld::set_up()
                .numbered(1)
                .holding<Body>(1)
                .holding<testing::Launcher>(3)
                .build(Out(small)));
    Entity holder = *small.create<Body>().with(Health{}).build();
    small.sync();
    {
      auto transaction = small.transaction();
      REQUIRE(small.destroy(holder).build());
    }  // Rolled back.

    Entity other = *small.create<Body>().build();
    auto attach = small.change(other).attach(Health{}).build();

    REQUIRE_FALSE(attach);
    CHECK(attach.error() ==
          lib::watch(BuildError::COMPONENT_CAPACITY_EXHAUSTED));
  }

  SECTION("ShouldRefuseCreateGivenDeadParent") {
    Entity parent = *world.create<testing::Launcher>().with(Position{}).build();
    world.sync();
    REQUIRE(world.destroy(parent).build());
    world.sync();

    auto child = world.create<testing::Interceptor>()
                     .under(parent)
                     .with(Position{})
                     .with(Velocity{})
                     .build();

    REQUIRE_FALSE(child);
    CHECK(child.error() == lib::watch(BuildError::ENTITY_NOT_ALIVE));
  }

  SECTION("ShouldVisitOnlyNearbyGivenSpatialQuery") {
    Entity near = *world.create<Body>().with(Position{1.0}).build();
    REQUIRE(world.create<Body>().with(Position{10.0}).build());
    world.sync();

    std::vector<Entity> found;
    world.within(Position{0.0}, 2.0,
                 [&](Entity e, const Position&) { found.push_back(e); });

    CHECK(found == std::vector<Entity>{near});
  }

  SECTION("ShouldFindNearestAcceptedGivenSpatialQuery") {
    Entity nearer = *world.create<Body>().with(Position{1.0}).build();
    Entity farther = *world.create<Body>().with(Position{-3.0}).build();
    REQUIRE(world.create<Body>().with(Position{9.0}).build());
    world.sync();

    auto any = [](Entity, const Position&) { return true; };
    auto not_nearer = [&](Entity e, const Position&) { return e != nearer; };
    CHECK(world.nearest(Position{0.0}, 5.0, any) == nearer);
    CHECK(world.nearest(Position{0.0}, 5.0, not_nearer) == farther);
    CHECK(world.nearest(Position{0.0}, 0.5, any) == std::nullopt);
  }

  SECTION("ShouldSeeCreatedAndDestroyedEntitiesGivenSyncAfterQuery") {
    Entity first = *world.create<Body>().with(Position{1.0}).build();
    world.sync();
    auto found = [&] {
      std::vector<Entity> entities;
      world.within(Position{0.0}, 2.0,
                   [&](Entity e, const Position&) { entities.push_back(e); });
      return entities;
    };
    REQUIRE(found() == std::vector<Entity>{first});

    Entity second = *world.create<Body>().with(Position{-1.0}).build();
    REQUIRE(world.destroy(first).build());
    world.sync();

    CHECK(found() == std::vector<Entity>{second});
  }

  SECTION("ShouldRefuseDetachGivenComponentTheArchetypeRequires") {
    Entity launcher =
        *world.create<testing::Launcher>().with(Position{}).build();
    world.sync();

    auto detached = world.change(launcher).detach<Position>().build();

    REQUIRE_FALSE(detached);
    CHECK(detached.error() == lib::watch(BuildError::COMPONENT_REQUIRED));
    CHECK(world.pending() == 0u);
  }

  SECTION("ShouldRefuseAttachGivenComponentTheArchetypeDoesNotPermit") {
    Entity launcher =
        *world.create<testing::Launcher>().with(Position{}).build();
    world.sync();

    auto attached = world.change(launcher).attach(Velocity{}).build();

    REQUIRE_FALSE(attached);
    CHECK(attached.error() == lib::watch(BuildError::COMPONENT_NOT_PERMITTED));
    CHECK(world.pending() == 0u);
  }

  SECTION("ShouldDescribeEntityGivenCreatedBeforeSync") {
    Entity launcher =
        *world.create<testing::Launcher>("lookout").with(Position{}).build();

    CHECK(world.aliases_of(world.archetype_of(launcher)) ==
          std::vector<Alias>{Alias{"launcher"}});
    CHECK(world.describe(world.name_of(launcher)).contains("lookout"));
    CHECK(world.describe(world.name_of(launcher)).contains("launcher"));
  }

  SECTION("ShouldReturnErrorGivenEntityCapacityExhausted") {
    for (int i = 0; i < 16; ++i) {
      REQUIRE(world.create<Body>().build());
    }
    CHECK_FALSE(world.create<Body>().build());
  }
}

}  // namespace simon::framework
