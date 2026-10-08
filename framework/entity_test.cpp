// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "framework/entity.hpp"

#include <stdexcept>

#include "base/testing.hpp"

namespace simon::framework {

TEST_CASE("EntityTable") {
  EntityTable table{3};

  SECTION("ShouldBeAliveGivenCreated") {
    // Under Test.
    Entity entity = table.create();

    // Postconditions.
    CHECK(table.alive(entity));
    CHECK(table.size() == 1u);
  }

  SECTION("ShouldNotBeAliveGivenDefaultEntity") {
    // Postconditions.
    CHECK_FALSE(table.alive(Entity{}));
  }

  SECTION("ShouldGoStaleGivenDestroyed") {
    // Preconditions.
    Entity entity = table.create();

    // Under Test.
    table.destroy(entity);

    // Postconditions.
    CHECK_FALSE(table.alive(entity));
    CHECK(table.size() == 0u);
  }

  SECTION("ShouldReuseIndexWithNewGenerationGivenFirstInFirstOut") {
    // Preconditions.
    Entity a = table.create();
    Entity b = table.create();
    Entity c = table.create();
    table.destroy(b);
    table.destroy(a);

    // Under Test.
    Entity reused_b = table.create();
    Entity reused_a = table.create();

    // Postconditions.
    CHECK(reused_b.index == b.index);
    CHECK(reused_b.generation == b.generation + 1);
    CHECK(reused_a.index == a.index);
    CHECK_FALSE(table.alive(b));
    CHECK(table.alive(reused_b));
    CHECK(table.alive(c));
  }

  SECTION("ShouldThrowGivenCapacityExhausted") {
    // Preconditions.
    for (int i = 0; i < 3; ++i) {
      table.create();
    }

    // Postconditions.
    CHECK_THROWS_AS(table.create(), std::logic_error);
  }

  SECTION("ShouldThrowGivenStaleEntityDestroyed") {
    // Preconditions.
    Entity entity = table.create();
    table.destroy(entity);

    // Postconditions.
    CHECK_THROWS_AS(table.destroy(entity), std::logic_error);
  }
}

}  // namespace simon::framework
