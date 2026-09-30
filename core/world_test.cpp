// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "core/world.hpp"

#include "core/status.hpp"

#include <stdexcept>
#include <vector>

#include "base/testing.hpp"
#include "core/test_world.hpp"

namespace simon::core {

using testing::Body;
using testing::Health;
using testing::Position;
using testing::TestWorld;
using testing::Velocity;

template <typename Builder>
concept CanParent =
    requires(Builder builder) { std::move(builder).under(Entity{}); };

TEST_CASE("World") {
  TestWorld world{testing::small_world()};

  SECTION("ShouldDeferComponentsUntilSyncGivenCreate") {
    auto entity =
        world.create<Body>().with(Position{1.0}).with(Velocity{2.0}).build();
    REQUIRE(entity);
    CHECK(world.alive(*entity));
    CHECK_FALSE(world.store<Position>().contains(*entity));

    world.sync();

    CHECK(world.store<Position>().get(*entity).x == 1.0);
    CHECK(world.store<Velocity>().get(*entity).x == 2.0);
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
    TestWorld tiny{
        WorldConfiguration{.number = 1, .entities = 1, .components = 1}};
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

    CHECK(world.identity_of(world.name()) == "/world/1");
    CHECK(world.identity_of(world.name_of(entity)) == "/world/1/entity/0");
    CHECK(world.identity_of(position) == "/world/1/entity/0/component/0");
    CHECK(world.find("/world/1/entity/0") == world.name_of(entity));
    CHECK(world.find("/world/1/entity/0/component/0") == position);
    CHECK(world.entity_of(position) == entity);
  }

  SECTION("ShouldNotFindGivenIdentityOfSomethingAbsent") {
    Entity entity = *world.create<Body>().build();
    world.sync();

    CHECK_FALSE(world.find("/world/2/entity/0"));  // Another world.
    CHECK_FALSE(world.find("/world/1/entity/9"));  // No such entity.
    CHECK_FALSE(world.find("/world/1/entity/0/component/0"));  // No Position.
    CHECK_FALSE(world.find("/world/1/component/99"));  // No such component.
    CHECK_FALSE(world.find("world/1"));                // Not an identity.
    CHECK(world.find("/world/1/component/0") == TestWorld::name_of<Position>());
    (void)entity;
  }

  SECTION("ShouldFindComponentByTypeNameGivenBuiltInAliases") {
    CHECK(world.find_alias("Position") ==
          std::vector{TestWorld::name_of<Position>()});
    CHECK(world.find_alias("simon::core::testing::Position") ==
          std::vector{TestWorld::name_of<Position>()});
  }

  SECTION("ShouldAliasEntityGivenCreateAlias") {
    Entity luke = *world.create<Body>("Luke Skywalker").build();

    CHECK(world.find_alias("Luke Skywalker") ==
          std::vector{world.name_of(luke)});
    CHECK(world.aliases_of(world.name_of(luke)) ==
          std::vector<std::string>{"Luke Skywalker"});
  }

  SECTION("ShouldShareAliasGivenManyEntities") {
    Entity first = *world.create<Body>("red drone").build();
    Entity second = *world.create<Body>("red drone").build();

    CHECK(world.find_alias("red drone") ==
          std::vector{world.name_of(first), world.name_of(second)});
  }

  SECTION("ShouldGiveAndTakeAliasesGivenChange") {
    Entity entity = *world.create<Body>().build();

    REQUIRE(world.change(entity).alias("ego").alias("hero").build());
    CHECK(world.find_alias("ego") == std::vector{world.name_of(entity)});

    REQUIRE(world.change(entity).unalias("ego").build());
    CHECK(world.find_alias("ego").empty());
    CHECK(world.aliases_of(world.name_of(entity)) ==
          std::vector<std::string>{"hero"});
  }

  SECTION("ShouldRefuseAliasChangeGivenInvalidDuplicateOrMissingAlias") {
    Entity entity = *world.create<Body>("ego").build();

    auto empty = world.change(entity).alias("").build();
    auto duplicate = world.change(entity).alias("ego").build();
    auto twice = world.change(entity).alias("hero").alias("hero").build();
    auto missing = world.change(entity).unalias("villain").build();

    CHECK(empty.error() == lib::watch(BuildCondition::ALIAS_INVALID));
    CHECK(duplicate.error() == lib::watch(BuildCondition::ALIAS_ALREADY_GIVEN));
    CHECK(twice.error() == lib::watch(BuildCondition::ALIAS_ALREADY_GIVEN));
    CHECK(missing.error() == lib::watch(BuildCondition::ALIAS_NOT_GIVEN));
    CHECK(world.aliases_of(world.name_of(entity)) ==
          std::vector<std::string>{"ego"});
  }

  SECTION("ShouldRecordArchetypeGivenCreate") {
    Entity launcher =
        *world.create<testing::Launcher>().with(Position{}).build();
    Entity body = *world.create<Body>().build();
    world.sync();

    Name archetype = world.archetype_of(launcher);
    CHECK(archetype.kind == static_cast<std::uint32_t>(Kind::ARCHETYPE));
    CHECK(world.aliases_of(archetype) == std::vector<std::string>{"launcher"});
    CHECK(world.archetype_of(body) != archetype);
    CHECK(world.identity_of(archetype) == "/world/1/archetype/0");
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
    CHECK(world.store<Position>().size() == 0u);
    CHECK(world.store<Health>().size() == 0u);
    CHECK(world.store<EntityArchetype>().size() == 0u);
    CHECK_FALSE(world.entity_of(name));
    CHECK(world.find_alias("ball").empty());
  }

  SECTION("ShouldAttachAndDetachGivenChange") {
    Entity entity = *world.create<Body>().with(Position{}).build();
    world.sync();

    REQUIRE(
        world.change(entity).attach(Health{5.0}).detach<Position>().build());
    world.sync();

    CHECK(world.store<Health>().get(entity).points == 5.0);
    CHECK_FALSE(world.store<Position>().contains(entity));
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
    CHECK(second.error() == lib::watch(BuildCondition::ENTITY_NOT_ALIVE));
    CHECK_FALSE(world.alive(entity));
  }

  SECTION("ShouldRefuseChangeGivenDestroyPending") {
    Entity entity = *world.create<Body>().build();
    world.sync();
    REQUIRE(world.destroy(entity).build());

    auto change = world.change(entity).attach(Health{}).build();

    REQUIRE_FALSE(change);
    CHECK(change.error() == lib::watch(BuildCondition::ENTITY_NOT_ALIVE));
    world.sync();
  }

  SECTION("ShouldRefuseSecondAttachGivenAttachAlreadyPending") {
    Entity entity = *world.create<Body>().build();
    world.sync();
    REQUIRE(world.change(entity).attach(Health{1.0}).build());

    auto second = world.change(entity).attach(Health{2.0}).build();

    REQUIRE_FALSE(second);
    CHECK(second.error() ==
          lib::watch(BuildCondition::COMPONENT_ALREADY_ATTACHED));
    REQUIRE_NOTHROW(world.sync());
    CHECK(world.store<Health>().get(entity).points == 1.0);
  }

  SECTION("ShouldAllowReattachGivenDetachPendingInSameBatch") {
    Entity entity = *world.create<Body>().with(Health{1.0}).build();
    world.sync();
    REQUIRE(world.change(entity).detach<Health>().build());

    REQUIRE(world.change(entity).attach(Health{2.0}).build());
    world.sync();

    CHECK(world.store<Health>().get(entity).points == 2.0);
  }

  SECTION("ShouldRefuseAttachGivenStoreFullAfterPendingAttaches") {
    TestWorld small{
        WorldConfiguration{.number = 1, .entities = 8, .components = 2}};
    REQUIRE(small.create<Body>().with(Health{}).build());
    REQUIRE(small.create<Body>()
                .with(Health{})
                .build());  // Pending: store will be full.
    Entity extra = *small.create<Body>().build();

    auto create = small.create<Body>().with(Health{}).build();
    auto attach = small.change(extra).attach(Health{}).build();

    REQUIRE_FALSE(create);
    CHECK(create.error() ==
          lib::watch(BuildCondition::COMPONENT_CAPACITY_EXHAUSTED));
    REQUIRE_FALSE(attach);
    CHECK(attach.error() ==
          lib::watch(BuildCondition::COMPONENT_CAPACITY_EXHAUSTED));
    REQUIRE_NOTHROW(small.sync());
    CHECK(small.store<Health>().size() == 2u);
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
    CHECK(child.error() == lib::watch(BuildCondition::ENTITY_NOT_ALIVE));
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

  SECTION("ShouldReturnErrorGivenEntityCapacityExhausted") {
    for (int i = 0; i < 16; ++i) {
      REQUIRE(world.create<Body>().build());
    }
    CHECK_FALSE(world.create<Body>().build());
  }
}

}  // namespace simon::core
