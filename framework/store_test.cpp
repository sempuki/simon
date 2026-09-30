// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "framework/store.hpp"

#include <stdexcept>

#include "base/testing.hpp"

namespace simon::framework {

namespace {
struct Mass final {
  double kilograms = 0.0;
};
}  // namespace

TEST_CASE("Store") {
  EntityTable entities{8};
  Store<Mass> store{4, 8};
  Entity a = entities.create();
  Entity b = entities.create();
  Entity c = entities.create();

  SECTION("ShouldFindValueGivenAppended") {
    store.append(a, Mass{1.0});
    REQUIRE(store.try_component_of(a));
    CHECK(store.component_of(a).kilograms == 1.0);
    CHECK(store.size() == 1u);
  }

  SECTION("ShouldReturnNullGivenAbsent") {
    CHECK(store.try_component_of(a) == nullptr);
    CHECK_THROWS_AS(store.component_of(a), std::logic_error);
  }

  SECTION("ShouldStayDenseGivenMiddleErased") {
    store.append(a, Mass{1.0});
    store.append(b, Mass{2.0});
    store.append(c, Mass{3.0});

    store.erase(a);

    CHECK(store.size() == 2u);
    CHECK(store.owner(0) == c);  // The last moved into the gap.
    CHECK(store.component_of(c).kilograms == 3.0);
    CHECK(store.component_of(b).kilograms == 2.0);
    CHECK_FALSE(store.contains(a));
  }

  SECTION("ShouldReturnNullGivenStaleEntityWhoseIndexWasReused") {
    store.append(a, Mass{1.0});
    store.erase(a);
    entities.destroy(a);
    Entity reused = Entity{};
    for (int i = 0; i < 6 && reused.index != a.index; ++i) {
      reused = entities.create();
    }
    REQUIRE(reused.index == a.index);
    store.append(reused, Mass{9.0});

    CHECK(store.try_component_of(a) == nullptr);
    CHECK(store.component_of(reused).kilograms == 9.0);
  }

  SECTION("ShouldKeepAddressesGivenAddsWithinCapacity") {
    store.append(a, Mass{1.0});
    const Mass* before = store.try_component_of(a);
    store.append(b, Mass{2.0});
    store.append(c, Mass{3.0});
    CHECK(store.try_component_of(a) == before);
  }

  SECTION("ShouldThrowGivenAppendBeyondCapacity") {
    Entity d = entities.create();
    Entity e = entities.create();
    store.append(a, Mass{});
    store.append(b, Mass{});
    store.append(c, Mass{});
    store.append(d, Mass{});
    CHECK_THROWS_AS(store.append(e, Mass{}), std::logic_error);
  }

  SECTION("ShouldThrowGivenDuplicateAppend") {
    store.append(a, Mass{});
    CHECK_THROWS_AS(store.append(a, Mass{}), std::logic_error);
  }
}

}  // namespace simon::framework
