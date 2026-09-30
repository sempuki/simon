// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "framework/entity.hpp"

#include <stdexcept>

#include "base/testing.hpp"

namespace simon::framework {

TEST_CASE("EntityTable") {
  EntityTable table{3};

  SECTION("ShouldBeAliveGivenCreated") {
    Entity entity = table.create();
    CHECK(table.alive(entity));
    CHECK(table.size() == 1u);
  }

  SECTION("ShouldNotBeAliveGivenDefaultEntity") {
    CHECK_FALSE(table.alive(Entity{}));
  }

  SECTION("ShouldGoStaleGivenDestroyed") {
    Entity entity = table.create();
    table.destroy(entity);
    CHECK_FALSE(table.alive(entity));
    CHECK(table.size() == 0u);
  }

  SECTION("ShouldReuseIndexWithNewGenerationGivenFirstInFirstOut") {
    Entity a = table.create();
    Entity b = table.create();
    Entity c = table.create();
    table.destroy(b);
    table.destroy(a);

    Entity reused_b = table.create();
    Entity reused_a = table.create();

    CHECK(reused_b.index == b.index);
    CHECK(reused_b.generation == b.generation + 1);
    CHECK(reused_a.index == a.index);
    CHECK_FALSE(table.alive(b));
    CHECK(table.alive(reused_b));
    CHECK(table.alive(c));
  }

  SECTION("ShouldThrowGivenCapacityExhausted") {
    for (int i = 0; i < 3; ++i) {
      table.create();
    }
    CHECK_THROWS_AS(table.create(), std::logic_error);
  }

  SECTION("ShouldThrowGivenStaleEntityDestroyed") {
    Entity entity = table.create();
    table.destroy(entity);
    CHECK_THROWS_AS(table.destroy(entity), std::logic_error);
  }
}

}  // namespace simon::framework
